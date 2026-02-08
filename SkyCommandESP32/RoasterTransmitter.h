// RoasterTransmitter.h
#pragma once
#include <Arduino.h>

// RMT tick: 1 MHz => 1 µs per tick
#ifndef RMT_TX_CLK_HZ
#define RMT_TX_CLK_HZ 1000000u
#endif

class RoasterTransmitter {
public:
  explicit RoasterTransmitter(int pin);
  bool begin();                               // Initialize RMT TX
  bool send(const uint8_t* buffer, size_t length); // Non-blocking send
  bool ready() const;                          // TX complete?
  void setIntervalMs(uint32_t ms) { intervalMs = ms; }
  void service(const uint8_t* buf, size_t len);    // periodic non-blocking send

private:
  // --- Protocol timing (ticks at 1 µs per tick) ---
  static constexpr uint16_t PREAMBLE_LOW_US   = 7500; // active LOW preamble
  static constexpr uint16_t PREAMBLE_HIGH_US  = 3800; // gap before bits
  static constexpr uint16_t BIT_GAP_US        = 750;  // HIGH gap between bits
  static constexpr uint16_t BIT0_LOW_US       = 650;  // '0' low duration
  static constexpr uint16_t BIT1_LOW_US       = 1500; // '1' low duration
  bool hasSent = false;
  int txPin;
  uint32_t intervalMs = 600;
  unsigned long lastSendMs = 0;

  // Buffer: preamble + up to 8 bytes (LSB-first) + terminal high
  static constexpr size_t MAX_BYTES = 8;
  static constexpr size_t TX_MAX_SYMBOLS = 1 + (MAX_BYTES * 8) + 1;
  rmt_data_t sym[TX_MAX_SYMBOLS];

  size_t buildSymbols(const uint8_t* buf, size_t len);
};
