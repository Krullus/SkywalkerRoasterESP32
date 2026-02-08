// RoasterReceiver.cpp
#include "RoasterReceiver.h"

#if ROASTER_DEBUG
  #define RDBG(...)        Serial.printf(__VA_ARGS__)
  #define RDBG_PRINT(x)    Serial.print(x)
  #define RDBG_PRINTLN(x)  Serial.println(x)
#else
  #define RDBG(...)        do{}while(0)
  #define RDBG_PRINT(x)    do{}while(0)
  #define RDBG_PRINTLN(x)  do{}while(0)
#endif

RoasterReceiver* RoasterReceiver::instance = nullptr;

RoasterReceiver::RoasterReceiver(int pin)
: rxPin(pin),
  rmtActive(false),
  newMessageFlag(false),
  lastFallingTime(0),
  messageReady(false),
  num_symbols(RX_MAX_SYMBOLS)
{
  instance = this;
}

void RoasterReceiver::begin() {
  pinMode(rxPin, INPUT);

  // RMT RX channel at 1 MHz tick
  rmtInit(rxPin, RMT_RX_MODE, RMT_MEM_NUM_BLOCKS_2, RMT_CLK_HZ);

  // End a capture when the line has been idle ~> message gap
  rmtSetRxMaxThreshold(rxPin, RoasterProto::PREAMBLE_MAX_US + 1000); // ~9500us
  rmtSetRxMinThreshold(rxPin, 2); // your glitch filter

  // Start first capture right away
  num_symbols = RX_MAX_SYMBOLS;
  rmtReadAsync(rxPin, symbols, &num_symbols);
  rmtActive = true;

  // No GPIO interrupts needed
}


void RoasterReceiver::update() {
  // If capture finished, parse and restart immediately
  if (rmtActive && rmtReceiveCompleted(rxPin)) {
    rmtActive = false;

    if (num_symbols > 0 && num_symbols <= RX_MAX_SYMBOLS) {
      if (find7ByteMessage(symbols, num_symbols, receiveBuffer)) {
        memcpy(lastMessage, receiveBuffer, ROASTER_LEN);
        messageReady = true;
      }
    }

    // Restart capture for next message window
    num_symbols = RX_MAX_SYMBOLS;
    rmtReadAsync(rxPin, symbols, &num_symbols);
    rmtActive = true;
  }

  // If for some reason we ever stopped, kick it again
  if (!rmtActive) {
    num_symbols = RX_MAX_SYMBOLS;
    rmtReadAsync(rxPin, symbols, &num_symbols);
    rmtActive = true;
  }
}


bool RoasterReceiver::available() const { return messageReady; }

void RoasterReceiver::getMessage(uint8_t *dest) {
  if (!dest) return;
  memcpy(dest, lastMessage, ROASTER_LEN);
  messageReady = false;
}

// void IRAM_ATTR RoasterReceiver::isrRouter() {
//   if (instance) instance->onEdge();
// }

// // CHANGE interrupt: measure LOW pulse width (FALLING -> RISING), active low signal
// void IRAM_ATTR RoasterReceiver::onEdge() {
//   const unsigned long now = micros();
//   const int lvl = digitalRead(rxPin);
//   if (lvl == LOW) {
//     lastFallingTime = now;
//   } else {
//     const unsigned long lowWidth = now - lastFallingTime;
//     static volatile unsigned long lastPreambleAt = 0;
//     // preamble LOW window
//     if (lowWidth >= RoasterProto::PREAMBLE_MIN_US && lowWidth <= RoasterProto::PREAMBLE_MAX_US) {
//       if (now - lastPreambleAt > RoasterProto::PREAMBLE_COOLDOWN_US) {
//         newMessageFlag = true;
//         lastPreambleAt = now;
//       }
//     }
//   }
// }

bool RoasterReceiver::decodeBit(const rmt_data_t &s, uint8_t &bit) const {
  uint16_t low = 0, high = 0;
  if (s.level0 == 0 && s.level1 == 1) { low = s.duration0; high = s.duration1; }
  else if (s.level0 == 1 && s.level1 == 0) { high = s.duration0; low = s.duration1; }
  else return false;

  // HIGH gap between bits
  if (high != 0 && (high < RoasterProto::BIT_GAP_MIN_US || high > RoasterProto::BIT_GAP_MAX_US)) {
    return false;
  }

  // Bit 1
  if (low >= RoasterProto::BIT1_MIN_US && low <= RoasterProto::BIT1_MAX_US) { bit = 1; return true; }
  // Bit 0
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

  // Fast path: exact 56 symbols
  if (count == total_bits) {
    return tryDecode(0);
  }

  // Fallback scanning
  for (size_t i = 0; i <= count - total_bits; ++i) {
    if (tryDecode(i)) return true;
  }
  return false;
}
