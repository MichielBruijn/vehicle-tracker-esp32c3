# VehicleTracker ESP32-C3

![Status](https://img.shields.io/badge/status-testfase-orange)
![Platform](https://img.shields.io/badge/platform-ESP32--C3-blue)
![LoRa](https://img.shields.io/badge/radio-LoRa%20868MHz-green)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

> **⚠️ Testfase** — Dit project is nog in ontwikkeling. Pinbezetting, payload-formaat en parameters kunnen nog wijzigen.

Deep-sleep GPS voertuigtracker op ESP32-C3. Wekt zichzelf op via een SW-520D trillingssensor, haalt een GPS-positie op en verstuurt maximaal 3 berichten per dag via LoRa (SX1276, 868 MHz). Bij geen beweging blijft de positie onveranderd — zo gaat de batterij jaren mee.

---

## Onderdelen

| Onderdeel | Type | Omschrijving |
|---|---|---|
| Microcontroller | ESP32-C3 Super Mini | Deep sleep ~5 µA |
| GPS module | GY-GPS6MV2 (NEO-6M) | 9600 baud NMEA, backup-batterij voor hot-start |
| LoRa module | SX1276 | 868 MHz, SF9, BW125, max 14 dBm |
| OLED display | SSD1306 0.96" 4-pin | 128×64 pixels, I2C |
| Bewegingssensor | SW-520D | Trilling/kantel switch, wakeup-trigger |

---

## Bedrading

```
ESP32-C3 GPIO → Onderdeel
─────────────────────────────────────────────────────────
GPIO0   ←  GPS TX        (UART1 RX, ontvang NMEA)
GPIO1   →  GPS PWR       (NPN basis via 1kΩ, HIGH = GPS aan)
GPIO2   ←  SW-520D       (andere poot → GND, interne pull-up)
                          RTC GPIO — ext0 wakeup bij LOW
GPIO3   ←  Bat ADC       (spanningsdeler: 1MΩ van VBat, 1MΩ naar GND)
GPIO4   →  LoRa SCK      (SPI klok)
GPIO5   ←  LoRa MISO     (SPI data in)
GPIO6   →  LoRa MOSI     (SPI data uit)
GPIO7   →  LoRa NSS/CS   (SPI chip select)
GPIO8   ↔  OLED SDA      (I2C data)
GPIO9   ↔  OLED SCL      (I2C klok)
GPIO10  →  LoRa RST      (hardware reset)
```

### GPS voedingsschakelaar (NPN low-side, bijv. BC547 of 2N2222)

```
GPIO1 ──[1kΩ]── Basis
                Collector ── GPS GND
                Emitter   ── Systeem GND
```
`HIGH` op GPIO1 → transistor aan → GPS actief  
`LOW` op GPIO1 (ook tijdens deep sleep via gpio_hold) → GPS volledig uit

### Batterij spanningsdeler (GPIO3)

```
VBat ──[1MΩ]── GPIO3 ──[1MΩ]── GND
```
Quiëstroom: ~2 µA bij 4V. Schaalfactor ×2 in firmware.

---

## Werking

```
Deep sleep (SW-520D wacht op trilling)
         │
         ▼ LOW op GPIO2 (trilling gedetecteerd)
Wake-up ESP32-C3
         │
         ├─ Dagquotum vol? → direct terug slapen
         │
         ▼
GPS aan, fix proberen (max 90s)
         │
         ├─ Fix gekregen → positie opslaan in RTC geheugen
         │                  dag-rollover check via GPS datum
         │
         ├─ Geen fix → gebruik laatste bekende positie (uit RTC geheugen)
         │
         ▼
Berichten vandaag < 3?
         │
         ├─ Ja → LoRa pakket versturen (lat, lon, batterij%)
         │
         ▼
OLED status tonen (4 seconden)
         │
         ▼
GPS uit, LoRa slaapstand, OLED uit
         │
         ▼
Deep sleep (+ 24u timer als quotum vol)
```

---

## LoRa payload (10 bytes)

| Byte | Type | Inhoud |
|---|---|---|
| 0 | `uint8` | Device ID (instelbaar via `DEVICE_ID`) |
| 1–4 | `float` | Breedtegraad (IEEE 754 LE) |
| 5–8 | `float` | Lengtegraad (IEEE 754 LE) |
| 9 | `uint8` | Batterijniveau (0–100%) |

Sync word: `0xAB` (privé netwerk, niet LoRaWAN-compatibel)

---

## Batterijlevensduur (schatting)

| Scenario | Verbruik/dag | Levensduur (18650 3000 mAh) |
|---|---|---|
| GPS warm-start ~5s | ~0.77 mAh/dag | **~8–9 jaar** |
| GPS cold-start ~60s | ~5.6 mAh/dag | ~1–2 jaar |

> **Tip:** De backup-batterij op de GY-GPS6MV2 bewaart de almanac tussen wakeups. Dit maakt het verschil tussen een 5-seconden warm-start en een 60-seconden cold-start — en dus tussen jaren of maanden batterijduur.

---

## Benodigde bibliotheken

Installeer via Arduino Library Manager:

- **RadioLib** — Jan Gromeš
- **TinyGPS++** — Mikal Hart
- **U8g2** — Oliver Kraus

Board: `ESP32C3 Dev Module` (of Super Mini variant)  
Upload speed: 115200 / 921600

---

## Configuratie (`VehicleTracker_ESP32C3.ino`)

```cpp
#define DEVICE_ID        0x01   // Uniek per tracker
#define LORA_FREQUENCY   868.0  // MHz (868 EU, 915 USA)
#define LORA_SF          9      // Spreading factor (7–12)
#define LORA_TX_POWER    14     // dBm
#define MAX_MSGS_PER_DAY 3      // Max berichten per dag
#define GPS_FIX_TIMEOUT_MS 90000 // Max GPS wachttijd (ms)
```

---

## Arduino Pro Micro compatibel?

Nee. De ATmega32U4 heeft slechts 32KB flash — onvoldoende voor TinyGPS++ + RadioLib + U8g2 tegelijk. Daarnaast ontbreekt native ext0 deep sleep wakeup. Gebruik ESP32-C3.

---

## TODO / Testpunten

- [ ] Stroomverbruik meten in deep sleep (µA logging)
- [ ] GPS hot-start tijd valideren na >24u slaap
- [ ] LoRa bereik testen op voertuig
- [ ] Payload ontvangen en decoderen op gateway
- [ ] Spanning deler kalibreren per individueel board
- [ ] Dag-rollover test (24u soak test)
- [ ] Trilling debounce beoordelen (meerdere wakeups bij 1 schok)

---

## Licentie

MIT — vrij te gebruiken en aan te passen.
