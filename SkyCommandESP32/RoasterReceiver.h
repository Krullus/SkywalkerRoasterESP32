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

// ==================================================================
//         STATISTIK (MEDEL, MIN, MAX & ANTAL)
// ==================================================================
// - Preamble (låg del):     Medel:  7475 us  [Min:  7459, Max:  7513] 
// - Preamble (hög del/gap): Medel:  3869 us  [Min:  3858, Max:  3889] 
// - Bit-gap (mellan bitar): Medel:   751 us  [Min:   737, Max:   765] 
// - Bit '0' (låg tid):      Medel:   620 us  [Min:   607, Max:   628] 
// - Bit '1' (låg tid):      Medel:  1492 us  [Min:  1478, Max:  1511] 
// - Packet-gap (slutvila):  Medel: 10853 us  [Min: 10829, Max: 10883] 

// Protocol parameters (µs at 1 MHz)
namespace RoasterProto {
  constexpr uint16_t PREAMBLE_MIN_US = 7000;
  constexpr uint16_t PREAMBLE_MAX_US = 8000;
  constexpr uint16_t BIT_GAP_MIN_US  = 700;   
  constexpr uint16_t BIT_GAP_MAX_US  = 850;
  constexpr uint16_t BIT0_MIN_US     = 500;   
  constexpr uint16_t BIT0_MAX_US     = 900;
  constexpr uint16_t BIT1_MIN_US     = 1200;
  constexpr uint16_t BIT1_MAX_US     = 1800;
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
  volatile bool messageReady;

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
