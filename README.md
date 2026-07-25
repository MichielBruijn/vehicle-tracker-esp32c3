# VehicleTracker ESP32-C3

![Status](https://img.shields.io/badge/status-testing-orange)
![Platform](https://img.shields.io/badge/platform-ESP32--C3-blue)
![LoRaWAN](https://img.shields.io/badge/radio-LoRaWAN%20EU868-green)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

> **⚠️ Testing phase** — This project is still under development. Pin assignments, payload format and parameters may still change.

Deep-sleep GPS vehicle tracker based on the ESP32-C3. A timer wakes the device once every 24 hours; a CD4013B latch remembers whether the SW-520D vibration sensor detected motion in the meantime. Only then is the GPS powered up for a fresh fix. The position is sent over **LoRaWAN (The Things Network, EU868)** using an SX1262 module. No motion means no GPS — so the battery lasts for years.

---

## Components

| Component | Type | Description |
|---|---|---|
| Microcontroller | ESP32-C3 Super Mini | Deep sleep ~5 µA |
| GPS module | ATGM336H | 9600 baud NMEA, almanac in flash |
| LoRa module | DX-LR30-900M22S (Semtech SX1262) | LoRaWAN EU868, up to 22 dBm |
| OLED display | SSD1306 0.96" 4-pin | 128×64 pixels, I2C |
| Motion sensor | SW-520D | Vibration/tilt switch on a CD4013B SR latch |
| GPS power switch | ME6211C33 LDO + IRLB3034 MOSFET | GPS fully powered off in sleep |
| Voltage regulator | HT7833 | 3.3V LDO, ~55 µA quiescent |

---

## Wiring

```
ESP32-C3 GPIO → Component
─────────────────────────────────────────────────────────
GPIO0   ←  LoRa DIO1     (SX1262 interrupt)
GPIO1   ←  GPS TX        (UART1 RX, receives NMEA)
GPIO2   ←  CD4013B Q1    (latch output: HIGH = motion occurred)
GPIO3   →  LoRa SCK      (SPI clock)
GPIO4   ←  LoRa BUSY     (SX1262 busy)
GPIO5   ←  LoRa MISO     (SPI data in)
GPIO6   →  LoRa MOSI     (SPI data out)
GPIO7   →  LoRa NSS/CS   (SPI chip select, active low)
GPIO8   →  IRLB3034 gate (HIGH = GPS off; strapping pull-up keeps
                          it HIGH in deep sleep)
GPIO9      free          (BOOT pull-up 10kΩ→3V3, do not use)
GPIO10  →  GPS RX + CD4013B R1  (UART1 TX; pulse HIGH clears latch)
GPIO20  ↔  OLED SDA      (I2C data)
GPIO21  ↔  OLED SCL      (I2C clock)
```

The SX1262 NRST pin is left unconnected (`RADIOLIB_NC`, software reset via SPI). Always connect the antenna before powering the radio.

### GPS power switch

```
GPIO8 HIGH → MOSFET on  → ME6211C CE LOW  → GPS off
GPIO8 LOW  → MOSFET off → drain high (160kΩ→3V3) → CE HIGH → GPS on
```

The strapping pull-up (~8 kΩ) keeps GPIO8 HIGH during deep sleep, so the GPS is always off while sleeping — no `gpio_hold` needed. Switching circuit sleep current: 3.3 V / 160 kΩ ≈ 20 µA.

### Motion latch (CD4013B)

The SW-520D drives the CD4013B **Set** input. Any vibration latches Q1 HIGH, no matter when it happens. On wakeup the firmware reads Q1 (GPIO2) and then clears the latch with a HIGH pulse on R1 (GPIO10).

---

## Operation

```
Deep sleep (timer, 24h) — CD4013B latch collects motion events
         │
         ▼ timer wakeup
Read latch Q1, clear latch
         │
         ├─ No motion, no cached position → back to sleep
         │
         ├─ No motion, position known → send cached position (status 0), GPS stays off
         │
         ▼ motion detected (or first boot)
GPS on, wait for a fix (max 90s, then refine up to 10s)
         │
         ├─ Fix acquired → store in RTC memory, send (status 1)
         │
         ├─ No fix → send cached position (status 2)
         │
         ▼
LoRaWAN uplink (port 1), OLED status (4s), everything off, deep sleep
```

---

## LoRaWAN payload (14 bytes, port 1)

| Bytes | Type | Contents |
|---|---|---|
| 0 | `uint8` | Status: 0 = no motion (cache), 1 = motion + new fix, 2 = motion but GPS timeout (cache) |
| 1–4 | `int32` LE | Latitude × 1 000 000 |
| 5–8 | `int32` LE | Longitude × 1 000 000 |
| 9–11 | `int24` LE | Altitude × 100 (m) |
| 12–13 | `int16` LE | HDOP × 100 |

### TTN payload formatter

```javascript
function decodeUplink(input) {
  var b = input.bytes;
  function int32le(b,i){var v=b[i]|(b[i+1]<<8)|(b[i+2]<<16)|(b[i+3]<<24);return v;}
  function int24le(b,i){var v=b[i]|(b[i+1]<<8)|(b[i+2]<<16);if(v&0x800000)v|=0xFF000000;return v|0;}
  var status    = b[0]; // 0=cache, 1=new fix, 2=moved+no fix
  var latitude  = int32le(b,1) / 1e6;
  var longitude = int32le(b,5) / 1e6;
  var altitude  = int24le(b,9) / 100;
  var hdop      = (b[12]|(b[13]<<8)) / 100;
  return { data: { status: status, latitude: latitude, longitude: longitude, altitude: altitude, hdop: hdop } };
}
```

After a successful OTAA join, the session (DevAddr, keys, frame counters) is stored in RTC memory and restored on every wakeup, so the device only rejoins after a full power loss.

---

## Battery life (estimate)

| Item | Consumption |
|---|---|
| Deep sleep | ESP32-C3 ~5 µA + HT7833 ~4 µA + divider ~2 µA ≈ 11 µA |
| GPS warm start | ~20–30 s at 20 mA (almanac in flash) |
| GPS cold start | 30–120 s at 20 mA (first use or after months) |
| LoRa TX | ~280 ms at ~90 mA (SX1262) |
| OLED | 4 s at 10 mA |

With warm GPS starts this works out to roughly **0.8 mAh/day** → about 8 years on an 18650 (3000 mAh, 80% usable). Cold starts every day would reduce that to roughly 1.2 years.

---

## Getting started

1. Install the libraries via the Arduino Library Manager:
   - **RadioLib** — Jan Gromeš
   - **TinyGPS++** — Mikal Hart
   - **U8g2** — Oliver Kraus
2. Copy `secrets.h.example` to `secrets.h` and fill in your TTN OTAA credentials (JoinEUI, DevEUI, AppKey) and optionally WiFi credentials for OTA debugging.
3. Register the device in the TTN console and paste the payload formatter above.
4. Board: `ESP32C3 Dev Module` (or Super Mini variant).

### Configuration (`VehicleTracker_ESP32C3.ino`)

```cpp
#define ENABLE_LORA       1     // SX1262 connected
#define ENABLE_GPS_PWR    1     // GPS power switch connected
#define ENABLE_WIFI_DEBUG 0     // WiFi + OTA + telnet (0 for production!)
#define ENABLE_SERIAL     0     // Serial debug output

#define GPS_FIX_TIMEOUT_MS  90000UL  // Max wait for first GPS fix
#define GPS_REFINE_MS       10000UL  // Refine time after first fix
#define SLEEP_SEC           86400ULL // Wakeup interval (24h)
```

### OTA upload

With `ENABLE_WIFI_DEBUG 1` the device accepts OTA updates and telnet (port 23) while awake:

```bash
./ota_upload.sh [ip-address]
```

---

## TODO / Test items

- [ ] Measure current draw in deep sleep (µA logging)
- [ ] Validate GPS warm-start time after >24h of sleep
- [ ] Test LoRaWAN range on a vehicle
- [ ] Day-rollover test (24h soak test)
- [ ] Long-term latch reliability (multiple wakeups from a single shock)

---

## License

MIT — free to use and modify.
