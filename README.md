# Skywalker Roaster ESP32 (RMT Non-Blocking Fork)

A non-blocking firmware implementation to connect an **ESP32-C3 Super Mini** to the Skywalker coffee roaster and integrate it seamlessly with **Artisan Scope**.

> **WORK IN PROGRESS** — This project is a rewrite/fork of the original concept, focusing on robust, asynchronous communication using the ESP32's native RMT (Remote Control Transceiver) peripheral and FreeRTOS.
> 

⚠️ **WARNING: DO NOT LEAVE ROASTER UNATTENDED!** 
The sketch requires continuous commands from Artisan to stay alive; otherwise, it triggers a safety shutdown. Use at your own risk.

---
## Level Shifting

To ensure reliable signal translation and protect the 3.3V ESP32-C3 GPIO pins from higher logic levels on the roaster I used a **TXS0102** bidirectional voltage-level translator is used. 
But there is probably better ways to do this. I also tested with at pair of NPN transistor (BC548), for fun, that seams to work but haven't tested it during a roast. I will just stick to TXS0102 for now.

## TMC2209

I used bidirectional communication as described by Peter Polidoro. Thanks!
This way the code don't need to send every step manually and can just send the target speed.
https://github.com/janelia-arduino/TMC2209

## 🛠️ Hardware & Pinout
This project is built and tested specifically for the **ESP32-C3 Super Mini**, paired with a **TMC2209 stepper motor driver** communicating over UART to handle drum rotation efficiently without blocking the main loop.

| Component / Function | ESP32-C3 Pin | Description |
| :--- | :--- | :--- |
| **LED_PIN** | GPIO 10 | Status LED Not really using this but lights a LED :D|
| **OE_PIN** | GPIO 2 | TXS0102 Level Shifter Output Enable |
| **RMT_TX_PIN** | GPIO 0 | Transmit data to roaster (RMT) |
| **RMT_RX_PIN** | GPIO 1 | Receive data from roaster (RMT) |
| **UART_RX_PIN** | GPIO 20 | TMC2209 Stepper Driver UART RX |
| **UART_TX_PIN** | GPIO 21 | TMC2209 Stepper Driver UART TX |
| **I2C_SDA_PIN** | GPIO 8 | AHT20 Temperature/Humidity Sensor SDA |
| **I2C_SCL_PIN** | GPIO 9 | AHT20 Temperature/Humidity Sensor SCL |

---

## 🚀 Quick Start & Installation

1. Install the **Arduino IDE** (with the ESP32 v3.x core supporting the modern RMT API).
2. Install required libraries (e.g., SparkFun AHT20 sensor library and TMC2209 if applicable).
3. Clone or download this repository or download the files in SkyCommandESP32
4. Open the project folder in the Arduino IDE, select **ESP32C3 Dev Module** under Tools, and upload the sketch.

---

## 📊 Protocol & Message Structure (Reverse Engineering)

Communication relies on pulse-width modulation over a single GPIO pin (preamble ~7.5ms, bit '1' ~1.5ms, bit '0' ~0.6ms, bit-gap ~0.75ms). Both devices transmit packets independently without traditional handshakes, secured via a checksum in the final byte.

### Controller (Sent to Roaster - 6 bytes, LSB)
| Position | Function | Value / Range |
|----------|----------|----------------|
| 0 | Fan Duty Cycle | 0 to 100 (steps of 5) |
| 1 | Filter | Values 0, 1, 2, 3, 4 |
| 2 | Bean Cooling Fan | 0 or 100 |
| 3 | Drum | Target RPM |
| 4 | Heater Duty Cycle | 0 to 100 (steps of 5) |
| 5 | Checksum | Sum of bytes 0–4 |

### Roaster (Sent from Roaster - 7 bytes, LSB)
| Position | Function | Description |
|----------|----------|-------------|
| 0, 1 | Value A (`intx`) | Combined 16-bit raw value used for lower/standard temperature range (via 3rd-degree polynomial) |
| 2, 3 | Value B (`inty`) | Combined 16-bit raw value used for higher temperature range (via 3rd-degree polynomial) |
| 4 | Reserved / Mystery | Still a mystery byte, typically 0 |
| 5 | Motor Current / Drum Load | Indicates motor activity and drum load status |
| 6 | Checksum | Sum of bytes 0–5 |

---

#### Temperature Conversion Logic
The firmware combines bytes `[0,1]` into `intx` and bytes `[2,3]` into `inty`. Depending on the threshold (`intx > 836 && inty > 221`), the code automatically switches between two different 3rd-degree polynomial equations to calculate an accurate temperature (`v`), which is then passed through a median and EMA filter for stability.

## ⚙️ Artisan Setup

1. **Load Preset Configuration:**
   Go to `Help -> Load Settings...` and select `ArtisanConfigs/basic-skitop-artisan-settings.aset`.
2. **Manual Events & Sliders:**
   * **Buttons:** 
     * `DRUM;100`
     * `DRUM;0`
     * `FILTER;1`
   * **Sliders:** 
     * `OT2;{}`
     * `OT1;{}`

---

## 🔍 What's New in This Fork?
* **RMT Peripheral Integration:** Uses hardware-based RMT for both transmission and reception to completely eliminate timing bottlenecks.
* **FreeRTOS Background Task:** Rx handling runs in a dedicated high-priority task, ensuring packet gaps are never missed—solving previous freezing issues during rapid sequential commands (like step-adjusting the fan or sending several commands at once when dropping the beans).
