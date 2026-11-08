// RoasterReceiver.h
#pragma once
#include <Arduino.h>

// RMT tick: 1 MHz => 1 µs per tick
#ifndef RMT_CLK_HZ
#define RMT_CLK_HZ 1000000u
#endif

// Debug toggles
#ifndef ROASTER_DEBUG
#define ROASTER_DEBUG 0
#endif
#ifndef ROASTER_DEBUG_PRINT_SYMBOLS
#define ROASTER_DEBUG_PRINT_SYMBOLS 0
#endif

// Protocol parameters (µs at 1 MHz)
namespace RoasterProto {
  constexpr uint16_t PREAMBLE_MIN_US = 7000;  // accept 7.0 ms
  constexpr uint16_t PREAMBLE_MAX_US = 8000;  // accept 8.0 ms
  constexpr uint16_t BIT_GAP_MIN_US  = 700;   // ~768 µs nominal
  constexpr uint16_t BIT_GAP_MAX_US  = 850;
  constexpr uint16_t BIT0_MIN_US     = 500;   // ~605 µs nominal
  constexpr uint16_t BIT0_MAX_US     = 900;
  constexpr uint16_t BIT1_MIN_US     = 1200;  // ~1480 µs nominal
  constexpr uint16_t BIT1_MAX_US     = 1800;
  constexpr uint16_t PREAMBLE_COOLDOWN_US = 5000;
}

// Receiver configuration (ESP32-C3: 2 blocks * 48 = 96 symbols max)
constexpr size_t RX_MAX_SYMBOLS = 96;
constexpr size_t ROASTER_LEN    = 7;   // bytes (including checksum)

class RoasterReceiver {
public:
  explicit RoasterReceiver(int pin);
  void begin();
  void update(); // Körs i vanliga loop() och parsar symbolerna!
  bool available() const;
  void getMessage(uint8_t* dest);

private:
  int rxPin;
  volatile bool rmtActive;
  bool messageReady;

  // RMT hårdvarans mottagningsbuffert
  size_t num_symbols;
  rmt_data_t symbols[RX_MAX_SYMBOLS];

  // Mellanbuffert för råsymboler (från FreeRTOS-task -> update() i loop)
  rmt_data_t capturedSymbols[RX_MAX_SYMBOLS];
  volatile size_t capturedCount;
  volatile bool rawReady;

  uint8_t receiveBuffer[ROASTER_LEN];
  uint8_t lastMessage[ROASTER_LEN];

  static void rxTaskTrampoline(void* arg);
  void rxTaskLoop();

  bool decodeBit(const rmt_data_t &s, uint8_t &bit) const;
  bool calculateChecksum(const uint8_t *buffer) const;
  bool find7ByteMessage(rmt_data_t *symbols, size_t count, uint8_t *buffer);
};
