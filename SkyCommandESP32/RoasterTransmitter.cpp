// RoasterTransmitter.cpp
#include "RoasterTransmitter.h"

RoasterTransmitter::RoasterTransmitter(int pin)
: txPin(pin) {}

bool RoasterTransmitter::begin() {
  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, HIGH); // idle HIGH
  if (!rmtInit(txPin, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, RMT_TX_CLK_HZ)) {
    return false;
  }
  rmtSetEOT(txPin, HIGH); // end-of-transmission idle level
  return true;
}

bool RoasterTransmitter::ready() const {
  return !hasSent || rmtTransmitCompleted(txPin);
}

size_t RoasterTransmitter::buildSymbols(const uint8_t* buffer, size_t len) {
  if (buffer == nullptr || len == 0) return 0;
  if (len > MAX_BYTES) len = MAX_BYTES;

  size_t idx = 0;
  // Preamble (active LOW), then HIGH gap
  sym[idx++] = (rmt_data_t){PREAMBLE_LOW_US, 0, PREAMBLE_HIGH_US, 1};

  // Bits: LSB-first per byte
  for (size_t b = 0; b < len; ++b) {
    uint8_t val = buffer[b];
    for (int bit = 0; bit < 8; ++bit) {
      bool isOne = bitRead(val, bit);
      if (isOne) {
        sym[idx++] = (rmt_data_t){BIT1_LOW_US, 0, BIT_GAP_US, 1};
      } else {
        sym[idx++] = (rmt_data_t){BIT0_LOW_US, 0, BIT_GAP_US, 1};
      }
    }
  }

  // // Terminal: hold HIGH replaced by eot
  // sym[idx++] = (rmt_data_t){0, 1, 0, 1};
  return idx;
}

bool RoasterTransmitter::send(const uint8_t* buffer, size_t length) {
  if (hasSent && !rmtTransmitCompleted(txPin)) return false;

  const size_t count = buildSymbols(buffer, length);
  if (count == 0) return false;

  hasSent = true;
  return rmtWriteAsync(txPin, sym, count);
}

void RoasterTransmitter::service(const uint8_t* buf, size_t len) {
  static unsigned long lastPollMs = 0;
  unsigned long now = millis();

  if (now - lastSendMs < intervalMs) return;
  if (now - lastPollMs < 100) return;  // max 1 kHz polling
  lastPollMs = now;
  yield();
  if (!ready()) return;
  if (!send(buf, len)) return;
  yield();
  lastSendMs = now;
}