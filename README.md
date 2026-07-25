# VehicleTracker ESP32-C3

![Status](https://img.shields.io/badge/status-testing-orange)
![Platform](https://img.shields.io/badge/platform-ESP32--C3-blue)
![LoRa](https://img.shields.io/badge/radio-LoRa%20868MHz-green)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

> **⚠️ Testing phase** — This project is still under development. Pin assignments, payload format and parameters may still change.

Deep-sleep GPS vehicle tracker based on the ESP32-C3. Wakes itself up via an SW-520D vibration sensor, obtains a GPS position and sends at most 3 messages per day over LoRa (SX1276, 868 MHz). When there is no movement the position remains unchanged — so the battery lasts for years.

---

## Components

| Component | Type | Description |
|---|---|---|
| Microcontroller | ESP32-C3 Super Mini | Deep sleep ~5 µA |
| GPS module | GY-GPS6MV2 (NEO-6M) | 9600 baud NMEA, backup battery for hot start |
| LoRa module | SX1276 | 868 MHz, SF9, BW125, max 14 dBm |
| OLED display | SSD1306 0.96" 4-pin | 128×64 pixels, I2C |
| Motion sensor | SW-520D | Vibration/tilt switch, wakeup trigger |

---

## Wiring

```
ESP32-C3 GPIO → Component
─────────────────────────────────────────────────────────
GPIO0   ←  GPS TX        (UART1 RX, receives NMEA)
GPIO1   →  GPS PWR       (NPN base via 1kΩ, HIGH = GPS on)
GPIO2   ←  SW-520D       (other leg → GND, internal pull-up)
                          RTC GPIO — ext0 wakeup on LOW
GPIO3   ←  Bat ADC       (voltage divider: 1MΩ from VBat, 1MΩ to GND)
GPIO4   →  LoRa SCK      (SPI clock)
GPIO5   ←  LoRa MISO     (SPI data in)
GPIO6   →  LoRa MOSI     (SPI data out)
GPIO7   →  LoRa NSS/CS   (SPI chip select)
GPIO8   ↔  OLED SDA      (I2C data)
GPIO9   ↔  OLED SCL      (I2C clock)
GPIO10  →  LoRa RST      (hardware reset)
```

### GPS power switch (NPN low-side, e.g. BC547 or 2N2222)

```
GPIO1 ──[1kΩ]── Base
                Collector ── GPS GND
                Emitter   ── System GND
```
`HIGH` on GPIO1 → transistor on → GPS active  
`LOW` on GPIO1 (also during deep sleep via gpio_hold) → GPS fully off

### Battery voltage divider (GPIO3)

```
VBat ──[1MΩ]── GPIO3 ──[1MΩ]── GND
```
Quiescent current: ~2 µA at 4V. Scale factor ×2 in firmware.

---

## Operation

```
Deep sleep (SW-520D waits for vibration)
         │
         ▼ LOW on GPIO2 (vibration detected)
Wake-up ESP32-C3
         │
         ├─ Daily quota reached? → go straight back to sleep
         │
         ▼
GPS on, try to get a fix (max 90s)
         │
         ├─ Fix acquired → store position in RTC memory
         │                  day-rollover check via GPS date
         │
         ├─ No fix → use last known position (from RTC memory)
         │
         ▼
Messages today < 3?
         │
         ├─ Yes → send LoRa packet (lat, lon, battery%)
         │
         ▼
Show OLED status (4 seconds)
         │
         ▼
GPS off, LoRa sleep mode, OLED off
         │
         ▼
Deep sleep (+ 24h timer if quota reached)
```

---

## LoRa payload (10 bytes)

| Byte | Type | Contents |
|---|---|---|
| 0 | `uint8` | Device ID (configurable via `DEVICE_ID`) |
| 1–4 | `float` | Latitude (IEEE 754 LE) |
| 5–8 | `float` | Longitude (IEEE 754 LE) |
| 9 | `uint8` | Battery level (0–100%) |

Sync word: `0xAB` (private network, not LoRaWAN-compatible)

---

## Battery life (estimate)

| Scenario | Consumption/day | Lifetime (18650 3000 mAh) |
|---|---|---|
| GPS warm start ~5s | ~0.77 mAh/day | **~8–9 years** |
| GPS cold start ~60s | ~5.6 mAh/day | ~1–2 years |

> **Tip:** The backup battery on the GY-GPS6MV2 preserves the almanac between wakeups. This makes the difference between a 5-second warm start and a 60-second cold start — and thus between years or months of battery life.

---

## Required libraries

Install via the Arduino Library Manager:

- **RadioLib** — Jan Gromeš
- **TinyGPS++** — Mikal Hart
- **U8g2** — Oliver Kraus

Board: `ESP32C3 Dev Module` (or Super Mini variant)  
Upload speed: 115200 / 921600

---

## Configuration (`VehicleTracker_ESP32C3.ino`)

```cpp
#define DEVICE_ID        0x01   // Unique per tracker
#define LORA_FREQUENCY   868.0  // MHz (868 EU, 915 USA)
#define LORA_SF          9      // Spreading factor (7–12)
#define LORA_TX_POWER    14     // dBm
#define MAX_MSGS_PER_DAY 3      // Max messages per day
#define GPS_FIX_TIMEOUT_MS 90000 // Max GPS wait time (ms)
```

---

## Arduino Pro Micro compatible?

No. The ATmega32U4 has only 32KB of flash — not enough for TinyGPS++ + RadioLib + U8g2 combined. It also lacks native ext0 deep sleep wakeup. Use the ESP32-C3.

---

## TODO / Test items

- [ ] Measure current draw in deep sleep (µA logging)
- [ ] Validate GPS hot-start time after >24h of sleep
- [ ] Test LoRa range on a vehicle
- [ ] Receive and decode the payload on the gateway
- [ ] Calibrate the voltage divider per individual board
- [ ] Day-rollover test (24h soak test)
- [ ] Evaluate vibration debounce (multiple wakeups from a single shock)

---

## License

MIT — free to use and modify.
