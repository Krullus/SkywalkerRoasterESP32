// RoasterReceiver.cpp
#include "RoasterReceiver.h"

RoasterReceiver::RoasterReceiver(int pin)
: rxPin(pin),
  rmtActive(false),
  messageReady(false),
  num_symbols(RX_MAX_SYMBOLS),
  capturedCount(0),
  rawReady(false)
{}

void RoasterReceiver::rxTaskTrampoline(void* arg) {
  static_cast<RoasterReceiver*>(arg)->rxTaskLoop();
}

void RoasterReceiver::begin() {
  pinMode(rxPin, INPUT);

  // RMT RX channel at 1 MHz tick (2 blocks = 96 symbols on ESP32-C3)
  rmtInit(rxPin, RMT_RX_MODE, RMT_MEM_NUM_BLOCKS_2, RMT_CLK_HZ);

  // Klipper vid ~8001 µs tystnad (~3 ms marginal i paket-gapet)
  rmtSetRxMaxThreshold(rxPin, RoasterProto::PREAMBLE_MAX_US + 1);
  rmtSetRxMinThreshold(rxPin, 2);

  // Starta första fångsten
  num_symbols = RX_MAX_SYMBOLS;
  rmtReadAsync(rxPin, symbols, &num_symbols);
  rmtActive = true;

  // Liten stack (2048 bytes) räcker nu när tasken bara gör memcpy + omstart!
  xTaskCreate(rxTaskTrampoline, "RmtRxTask", 2048, this, 5, nullptr);
}

// Högprioriterad bakgrunds-task: Bara kopiera rådata och starta om RMT!
void RoasterReceiver::rxTaskLoop() {
  for (;;) {
    if (rmtActive && rmtReceiveCompleted(rxPin)) {
      rmtActive = false;
      size_t count = num_symbols;

      // Kopiera bara över om vi fick tillräckligt med symboler för ett paket (minst 56)
      // och om update() inte precis håller på att läsa capturedSymbols
      if (count >= (ROASTER_LEN * 8) && count <= RX_MAX_SYMBOLS && !rawReady) {
        memcpy(capturedSymbols, symbols, count * sizeof(rmt_data_t));
        capturedCount = count;
        rawReady = true; // Signalera till vanliga loop() att rådata finns att parsa
      }

      // Starta om RMT omedelbart inför nästa paket!
      num_symbols = RX_MAX_SYMBOLS;
      rmtReadAsync(rxPin, symbols, &num_symbols);
      rmtActive = true;
    } else if (!rmtActive) {
      num_symbols = RX_MAX_SYMBOLS;
      rmtReadAsync(rxPin, symbols, &num_symbols);
      rmtActive = true;
    }

    vTaskDelay(pdMS_TO_TICKS(1)); // Sov 1 ms
  }
}

// Körs i vanliga loop(): Parsar symbolerna utan tidspress
void RoasterReceiver::update() {
  if (!rawReady) return;

  // Parsa råsymbolerna som tasken kopierade undan
  if (find7ByteMessage(capturedSymbols, capturedCount, receiveBuffer)) {
    memcpy(lastMessage, receiveBuffer, ROASTER_LEN);
    messageReady = true;
  }

  // Släpp låset så tasken får lägga in nästa råfångst
  rawReady = false;
}

bool RoasterReceiver::available() const { return messageReady; }

void RoasterReceiver::getMessage(uint8_t *dest) {
  if (!dest) return;
  memcpy(dest, lastMessage, ROASTER_LEN);
  messageReady = false;
}

bool RoasterReceiver::decodeBit(const rmt_data_t &s, uint8_t &bit) const {
  uint16_t low = 0, high = 0;
  if (s.level0 == 0 && s.level1 == 1) { low = s.duration0; high = s.duration1; }
  else if (s.level0 == 1 && s.level1 == 0) { high = s.duration0; low = s.duration1; }
  else return false;

  // HIGH gap between bits
  if (high != 0 && (high < RoasterProto::BIT_GAP_MIN_US || high > RoasterProto::BIT_GAP_MAX_US)) {
    return false;
  }

  if (low >= RoasterProto::BIT1_MIN_US && low <= RoasterProto::BIT1_MAX_US) { bit = 1; return true; }
  if (low >= RoasterProto::BIT0_MIN_US && low <= RoasterProto::BIT0_MAX_US) { bit = 0; return true; }
  return false;
}

bool RoasterReceiver::calculateChecksum(const uint8_t *buffer) const {
  if (!buffer) return false;
  uint8_t sum = 0;
  for (int i = 0; i < (int)ROASTER_LEN - 1; ++i) sum += buffer[i];
  return sum == buffer[ROASTER_LEN - 1];
}

bool RoasterReceiver::find7ByteMessage(rmt_data_t *sym, size_t count, uint8_t *buffer) {
  if (!sym || !buffer) return false;
  const size_t total_bits = ROASTER_LEN * 8;
  if (count < total_bits) return false;

  auto tryDecode = [&](size_t start) -> bool {
    memset(buffer, 0, ROASTER_LEN);
    for (size_t b = 0; b < total_bits; ++b) {
      uint8_t bit;
      if (!decodeBit(sym[start + b], bit)) return false;
      if (bit) buffer[b / 8] |= (1 << (b % 8)); // LSB-first
    }
    return calculateChecksum(buffer);
  };

  if (count == total_bits) {
    return tryDecode(0);
  }

  for (size_t i = 0; i <= count - total_bits; ++i) {
    if (tryDecode(i)) return true;
  }
  return false;
}
