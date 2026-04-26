/*
 * VehicleTracker_ESP32C3
 * ──────────────────────────────────────────────────────────────────────────────
 * Deep-sleep GPS tracker voor langdurige battrijwerking op een voertuig.
 * De SW-520D trilling/kantel sensor wekt de ESP32-C3 uit diepe slaap.
 * Maximaal 3 LoRa-berichten per dag worden verstuurd met de GPS-positie.
 *
 * HARDWARE:
 *   ESP32-C3 Super Mini (of vergelijkbaar)
 *   GY-GPS6MV2 (NEO-6M) GPS module    — UART, 9600 baud NMEA
 *   SX1276 LoRa module                 — SPI, 868 MHz (EU)
 *   SSD1306 0.96" OLED (4-pin, I2C)   — 128×64 pixels
 *   SW-520D trilling/kantel sensor
 *
 * BEDRADING:
 * ┌──────────────┬──────────┬─────────────────────────────────────────────┐
 * │ ESP32-C3 pin │ Verbonden│ Opmerking                                   │
 * ├──────────────┼──────────┼─────────────────────────────────────────────┤
 * │ GPIO0        │ GPS TX   │ UART1 RX — ontvang NMEA van GPS module      │
 * │ GPIO1        │ GPS PWR  │ NPN basis via 1kΩ (HIGH=GPS aan)           │
 * │              │          │   NPN: basis→1kΩ→GPIO1, collector→GPS GND  │
 * │              │          │   emitter→systeem GND                       │
 * │ GPIO2        │ SW-520D  │ Andere poot naar GND, interne pull-up       │
 * │              │          │ RTC GPIO — ext0 wakeup bij LOW              │
 * │ GPIO3        │ BAT ADC  │ Spanning deler: 1MΩ van VBat, 1MΩ naar GND │
 * │ GPIO4        │ LoRa SCK │ SPI klok                                    │
 * │ GPIO5        │ LoRa MISO│ SPI data in                                 │
 * │ GPIO6        │ LoRa MOSI│ SPI data uit                                │
 * │ GPIO7        │ LoRa NSS │ SPI chip select                             │
 * │ GPIO8        │ OLED SDA │ I2C data                                    │
 * │ GPIO9        │ OLED SCL │ I2C klok                                    │
 * │ GPIO10       │ LoRa RST │ LoRa hardware reset                         │
 * └──────────────┴──────────┴─────────────────────────────────────────────┘
 *
 * GPS VOEDING (NPN low-side schakelaar, bijv. BC547 of 2N2222):
 *   Basis   → 1kΩ → GPIO1
 *   Collector → GPS module GND
 *   Emitter   → Systeem GND
 *   HIGH op GPIO1 → transistor aan → GPS GND verbonden → GPS actief
 *   LOW op GPIO1  → transistor uit → GPS uit (0 µA extra)
 *
 * BATTERIJ SPANNINGSDELER (GPIO3):
 *   VBat ──┤1MΩ├── GPIO3 ──┤1MΩ├── GND
 *   Quiëstroom: VBat/2MΩ ≈ 2µA bij 4V (verwaarloosbaar)
 *   Schaalcoëfficiënt: ADC-waarde × 2 = werkelijke batterijspanning
 *
 * LORA PAYLOAD (10 bytes):
 *   Byte 0    : Device ID (uint8, instelbaar via DEVICE_ID)
 *   Bytes 1–4 : Breedtegraad (float, IEEE 754 little-endian)
 *   Bytes 5–8 : Lengtegraad  (float, IEEE 754 little-endian)
 *   Byte 9    : Batterijniveau (uint8, 0–100%)
 *
 * BATTERIJLEVENSDUUR (schatting):
 *   Deep sleep:       ESP32-C3 ~5µA + spanningsdeler ~2µA ≈ 7µA
 *   GPS warm-start:   ~5s bij 30mA (backup batterij op GY-GPS6MV2 behoudt almanac)
 *   GPS cold-start:   30–120s bij 30mA (alleen bij eerste gebruik of na maanden)
 *   LoRa TX (SF9):    ~280ms bij 120mA
 *   OLED:             4s bij 10mA
 *   Per wakeup:       ≈ 5s×30mA + 0.3s×120mA + 4s×10mA ≈ 0.20 mAh
 *   3× per dag:       0.60 mAh actief + 24h×0.007mA = 0.17 mAh slaap ≈ 0.77 mAh/dag
 *   18650 (3000 mAh, 80% bruikbaar = 2400 mAh):
 *     2400 / 0.77 ≈ 3117 dagen ≈ 8.5 JAAR (met warme GPS start)
 *   Worst-case cold-start elke keer (60s):
 *     3×(60s×30mA + 0.3s×120mA + 4s×10mA) ≈ 5.6 mAh/dag → ~428 dagen ≈ 1.2 jaar
 *   TIP: De backup batterij op GY-GPS6MV2 is cruciaal voor lange levensduur!
 *        Zorg dat die niet leeg is (werkt jaren op de ingebouwde cell).
 *
 * DAGELIJKS QUOTUM MECHANISME:
 *   - RTC-geheugen slaat GPS-datum en berichtteller op (overleeft deep sleep)
 *   - Als GPS-datum verandert: teller reset naar 0
 *   - Na 3 berichten: 24-uurs timer wakeup ingesteld (als reservemechanisme)
 *   - Bij trilling na vol quotum: direct terug naar slaap (geen GPS, geen LoRa)
 *
 * BENODIGDE BIBLIOTHEKEN (Arduino Library Manager):
 *   - RadioLib  door Jan Gromeš
 *   - TinyGPS++ door Mikal Hart
 *   - U8g2      door Oliver Kraus
 *
 * ARDUINO PRO MICRO COMPATIBILITEIT?
 *   Kort antwoord: NEE, niet praktisch.
 *   Redenen:
 *   • ATmega32U4 heeft slechts 32KB flash — TinyGPS++ (~3KB) + RadioLib (~20KB)
 *     + U8g2 (~15KB) overschrijden dit ruimschoots
 *   • Deep sleep via ext0 GPIO wakeup bestaat niet op AVR; alleen pin-change
 *     interrupt met power-down sleep, wat minder efficiënt en complexer is
 *   • Geen RTC-geheugen voor data-behoud tijdens slaap (EEPROM nodig)
 *   • 3.3V Pro Micro variant is nodig (SX1276 en GPS zijn 3.3V)
 *   Alternatief bij voorkeur voor AVR: gebruik ATtiny85 + aparte GPS module
 *   met bare-metal code, maar dat is een volledig ander project.
 *   Aanbeveling: blijf bij ESP32-C3.
 * ──────────────────────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <TinyGPS++.h>
#include "driver/gpio.h"
#include "esp_sleep.h"

// ═══════════════════════════════════════════════════════════════════════════
//  TESTCONFIGURATIE
//  Zet op 1 als het betreffende onderdeel is aangesloten.
// ═══════════════════════════════════════════════════════════════════════════
#define ENABLE_LORA      0   // SX1276 module (GPIO4–7, GPIO10)
#define ENABLE_BATT_ADC  0   // Spanningsdeler op GPIO3
#define ENABLE_GPS_PWR   0   // GPS transistorschakelaar op GPIO1

// ═══════════════════════════════════════════════════════════════════════════
//  PIN DEFINITIES
// ═══════════════════════════════════════════════════════════════════════════
#define PIN_GPS_RX      0
#define PIN_GPS_PWR     1
#define PIN_SW520D      2   // RTC GPIO — ext0 wakeup
#define PIN_BATT_ADC    3
#define PIN_LORA_SCK    4
#define PIN_LORA_MISO   5
#define PIN_LORA_MOSI   6   // !! Bij LoRa aan: OLED SDA verplaatsen naar vrije pin
#define PIN_LORA_CS     7   // !! Bij LoRa aan: OLED SCL verplaatsen naar vrije pin
#define PIN_OLED_SDA    6   // Draad van GPIO8 naar GPIO6 verplaatsen
#define PIN_OLED_SCL    7   // Draad van GPIO9 naar GPIO7 verplaatsen
#define PIN_LED         8   // Ingebouwde LED (was SDA — nu vrij)
#define PIN_BTN         9   // User/BOOT button (was SCL — nu vrij)
#define PIN_LORA_RST    10

// ═══════════════════════════════════════════════════════════════════════════
//  CONFIGURATIE — pas aan naar wens
// ═══════════════════════════════════════════════════════════════════════════
#define DEVICE_ID           0x01       // Uniek per apparaat (1–255)

#define LORA_FREQUENCY      868.0      // MHz  (pas aan: 915.0 voor USA)
#define LORA_BANDWIDTH      125.0      // kHz
#define LORA_SF             9          // Spreading factor 7–12 (hoger = verder, trager)
#define LORA_CR             5          // Coding rate 4/5
#define LORA_SYNC_WORD      0xAB       // 0xAB = privé netwerk (0x34 = LoRaWAN)
#define LORA_TX_POWER       14         // dBm (max 14 zonder PA_BOOST jumper)
#define LORA_PREAMBLE       8          // Preambule symbolen

#define GPS_BAUD            9600
#define GPS_FIX_TIMEOUT_MS  90000UL    // Max 90s wachten op GPS fix
#define SENSOR_TEST_MS      30000UL    // Sensor testfase na GPS (ms)
#define SENSOR_DEBOUNCE_MS  600UL      // Minimale tijd tussen twee tellingen
#define MAX_MSGS_PER_DAY    3          // Max LoRa berichten per kalenderdag

#define BATT_DIVIDER_RATIO  2.0f       // 1MΩ/1MΩ → factor 2
#define BATT_ADC_SAMPLES    16         // Gemiddelde over N metingen

// Conditionele includes — alleen laden wat ook bedraad is
#if ENABLE_LORA
#include <SPI.h>
#include <RadioLib.h>
#endif

// ─── LED blink patronen ─────────────────────────────────────────────────────
// 1 kort:  wakeup door beweging
// 2 kort:  GPS fix verkregen
// 3 kort:  LoRa verstuurd
// 1 lang:  button actie bevestiging
inline void ledOn()  { digitalWrite(PIN_LED, LOW);  }  // actief-laag
inline void ledOff() { digitalWrite(PIN_LED, HIGH); }

void ledBlink(uint8_t n, uint16_t onMs = 80, uint16_t offMs = 120) {
    for (uint8_t i = 0; i < n; i++) {
        ledOn();  delay(onMs);
        ledOff(); if (i < n - 1) delay(offMs);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  RTC GEHEUGEN — overleeft deep sleep
// ═══════════════════════════════════════════════════════════════════════════
RTC_DATA_ATTR uint8_t  rtc_msgsToday     = 0;
RTC_DATA_ATTR uint32_t rtc_currentDay    = 0;    // Encoded YYYYMMDD
RTC_DATA_ATTR float    rtc_lastLat       = 0.0f;
RTC_DATA_ATTR float    rtc_lastLon       = 0.0f;
RTC_DATA_ATTR bool     rtc_hasPosition   = false;
RTC_DATA_ATTR uint32_t rtc_wakeupCount   = 0;
RTC_DATA_ATTR uint32_t rtc_motionCount   = 0;   // Alleen SW-520D triggers
RTC_DATA_ATTR uint32_t rtc_totalMsgsSent = 0;

// ═══════════════════════════════════════════════════════════════════════════
//  OBJECTEN
// ═══════════════════════════════════════════════════════════════════════════
HardwareSerial gpsSerial(1);
TinyGPSPlus    gps;

// SSD1306 128×64 via hardware I2C — pins expliciet meegeven aan constructor
// Volgorde: (rotatie, reset, klok, data)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE, PIN_OLED_SCL, PIN_OLED_SDA);

#if ENABLE_LORA
// SX1276: CS, DIO0 (niet gebruikt=NC), RST, DIO1 (niet gebruikt=NC)
SX1276 radio = new Module(PIN_LORA_CS, RADIOLIB_NC, PIN_LORA_RST, RADIOLIB_NC);
#endif

// ═══════════════════════════════════════════════════════════════════════════
//  HELPERS
// ═══════════════════════════════════════════════════════════════════════════

uint32_t encodeDateYMD(uint16_t y, uint8_t m, uint8_t d) {
    return (uint32_t)y * 10000 + (uint32_t)m * 100 + d;
}

float readBatteryVoltage() {
#if ENABLE_BATT_ADC
    analogSetAttenuation(ADC_11db);
    uint32_t sum = 0;
    for (uint8_t i = 0; i < BATT_ADC_SAMPLES; i++) {
        sum += analogRead(PIN_BATT_ADC);
        delayMicroseconds(500);
    }
    float vAdc = (sum / (float)BATT_ADC_SAMPLES / 4095.0f) * 3.3f;
    return vAdc * BATT_DIVIDER_RATIO;
#else
    return 0.0f;  // Onbekend zonder spanningsdeler
#endif
}

uint8_t voltageToPercent(float v) {
    if (v >= 4.20f) return 100;
    if (v <= 3.00f) return 0;
    return (uint8_t)((v - 3.0f) / 1.2f * 100.0f + 0.5f);
}

void oledShowSearching(uint32_t elapsed_ms, bool sensorActive, uint8_t sats) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 10, "VehicleTracker");
    oled.drawHLine(0, 12, 128);

    // GPS zoekstatus + satellieten
    char gpsLine[22];
    snprintf(gpsLine, sizeof(gpsLine), "GPS zoeken... sat:%u", sats);
    oled.drawStr(0, 24, gpsLine);

    // Voortgangsbalk (max GPS_FIX_TIMEOUT_MS)
    uint8_t barWidth = (uint8_t)((elapsed_ms * 118UL) / GPS_FIX_TIMEOUT_MS);
    if (barWidth > 118) barWidth = 118;
    oled.drawFrame(4, 28, 120, 8);
    if (barWidth > 0) oled.drawBox(5, 29, barWidth, 6);

    // Sensor indicator
    oled.drawStr(0, 46, "Sensor:");
    if (sensorActive) {
        oled.drawBox(50, 37, 40, 11);
        oled.setDrawColor(0);
        oled.drawStr(54, 46, "AAN");
        oled.setDrawColor(1);
    } else {
        oled.drawFrame(50, 37, 40, 11);
        oled.drawStr(54, 46, "---");
    }

    char buf[22];
    snprintf(buf, sizeof(buf), "%lus  Beweging:%lu", elapsed_ms / 1000, rtc_motionCount);
    oled.drawStr(0, 64, buf);
    oled.sendBuffer();
}

void oledShowTest(float lat, float lon, bool newFix, bool sensorActive,
                  uint8_t sats, uint32_t secsLeft) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);

    if (rtc_hasPosition) {
        char latStr[20], lonStr[20];
        snprintf(latStr, sizeof(latStr), "%c %.5f", lat >= 0 ? 'N' : 'S', (double)fabsf(lat));
        snprintf(lonStr, sizeof(lonStr), "%c %.5f", lon >= 0 ? 'E' : 'W', (double)fabsf(lon));
        oled.drawStr(0, 10, latStr);
        oled.drawStr(0, 22, lonStr);
        if (!newFix) {
            oled.setFont(u8g2_font_5x7_tf);
            oled.drawStr(104, 22, "CACHE");
            oled.setFont(u8g2_font_6x10_tf);
        }
    } else {
        oled.drawStr(0, 10, "Geen GPS fix");
        oled.drawStr(0, 22, "---");
    }

    oled.drawHLine(0, 25, 128);

    // Sensor indicator — groot en duidelijk
    oled.drawStr(0, 37, "Sensor:");
    if (sensorActive) {
        oled.drawBox(50, 28, 50, 11);
        oled.setDrawColor(0);
        oled.drawStr(54, 37, " AAN ");
        oled.setDrawColor(1);
    } else {
        oled.drawFrame(50, 28, 50, 11);
        oled.drawStr(54, 37, " --- ");
    }

    char satBuf[16];
    snprintf(satBuf, sizeof(satBuf), "sat:%u", sats);
    oled.drawStr(108, 37, satBuf);

    char motBuf[22];
    snprintf(motBuf, sizeof(motBuf), "Beweging: %lu", rtc_motionCount);
    oled.drawStr(0, 50, motBuf);

    char slpBuf[22];
    snprintf(slpBuf, sizeof(slpBuf), "Slaap in: %lus", secsLeft);
    oled.drawStr(0, 62, slpBuf);

    oled.sendBuffer();
}

void oledShowStatus(float lat, float lon, bool newFix, uint8_t batPct, bool sent, bool loraFail) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);

    char title[22];
    snprintf(title, sizeof(title), "VehicleTracker #%03u", DEVICE_ID);
    oled.drawStr(0, 10, title);
    oled.drawHLine(0, 12, 128);

    if (rtc_hasPosition) {
        char latStr[20], lonStr[20];
        snprintf(latStr, sizeof(latStr), "%c %.5f",
                 lat >= 0 ? 'N' : 'S', (double)fabsf(lat));
        snprintf(lonStr, sizeof(lonStr), "%c %.5f",
                 lon >= 0 ? 'E' : 'W', (double)fabsf(lon));
        oled.drawStr(0, 26, latStr);
        oled.drawStr(0, 38, lonStr);
        if (!newFix) {
            oled.setFont(u8g2_font_5x7_tf);
            oled.drawStr(104, 38, "CACHE");
            oled.setFont(u8g2_font_6x10_tf);
        }
    } else {
        oled.drawStr(0, 26, "Geen GPS fix");
        oled.drawStr(0, 38, "Geen positie");
    }

    // Statusregel: batterij + LoRa
    char statusLine[22];
#if ENABLE_BATT_ADC
    snprintf(statusLine, sizeof(statusLine), "Bat:%3u%%", batPct);
#else
    snprintf(statusLine, sizeof(statusLine), "Bat:--");
#endif
    oled.drawStr(0, 52, statusLine);

#if ENABLE_LORA
    oled.setFont(u8g2_font_5x7_tf);
    if (sent) {
        oled.drawStr(72, 52, "Tx:SENT");
    } else if (loraFail) {
        oled.drawStr(72, 52, "Tx:FAIL");
    } else {
        char txBuf[12];
        snprintf(txBuf, sizeof(txBuf), "Tx:%u/%u", rtc_msgsToday, MAX_MSGS_PER_DAY);
        oled.drawStr(72, 52, txBuf);
    }
    oled.setFont(u8g2_font_6x10_tf);
#else
    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(72, 52, "LoRa:UIT");
    oled.setFont(u8g2_font_6x10_tf);
#endif

    char motStr[22];
    snprintf(motStr, sizeof(motStr), "Beweging: %lu", rtc_motionCount);
    oled.drawStr(0, 64, motStr);
    oled.sendBuffer();
}

#if ENABLE_LORA
bool sendLoRaPacket(float lat, float lon, uint8_t batPct) {
    uint8_t payload[10];
    payload[0] = DEVICE_ID;
    memcpy(&payload[1], &lat, 4);
    memcpy(&payload[5], &lon, 4);
    payload[9] = batPct;
    int state = radio.transmit(payload, sizeof(payload));
    return (state == RADIOLIB_ERR_NONE);
}
#endif

void goDeepSleep() {
#if ENABLE_GPS_PWR
    // Zet GPS uit en houd GPIO laag tijdens slaap
    digitalWrite(PIN_GPS_PWR, LOW);
    gpio_hold_en((gpio_num_t)PIN_GPS_PWR);
    gpio_deep_sleep_hold_en();
#endif

#if ENABLE_LORA
    radio.sleep();
#endif

    ledOff();
    oled.setPowerSave(1);

    // Pull-up op SW-520D actief houden tijdens deep sleep
    // (GPIO 0-5 behouden hun pull-up tijdens deep sleep op ESP32-C3)
    gpio_pullup_en((gpio_num_t)PIN_SW520D);
    gpio_pulldown_dis((gpio_num_t)PIN_SW520D);

    // Wake op verandering: stel het tegenovergestelde van de huidige stand in.
    // Zo maakt oriëntatie niet uit — elke overgang (open→dicht of dicht→open) wekt op.
    bool pinLow = (digitalRead(PIN_SW520D) == LOW);
    esp_deep_sleep_enable_gpio_wakeup(
        1ULL << PIN_SW520D,
        pinLow ? ESP_GPIO_WAKEUP_GPIO_HIGH : ESP_GPIO_WAKEUP_GPIO_LOW
    );

#if ENABLE_LORA
    // Als daglimiet bereikt: ook een 24-uurs timer als vangnet voor dag-reset
    if (rtc_msgsToday >= MAX_MSGS_PER_DAY) {
        esp_sleep_enable_timer_wakeup(24ULL * 3600ULL * 1000000ULL);
    }
#endif

    esp_deep_sleep_start();
    // Komt hier nooit — ESP reset na wakeup
}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP — alles zit hier, loop() wordt nooit bereikt (deep sleep + reset)
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    pinMode(PIN_LED, OUTPUT);  ledOff();
    pinMode(PIN_BTN, INPUT_PULLUP);
    rtc_wakeupCount++;

    // ── Wakeup reden controleren ─────────────────────────────────────────
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_GPIO) {
        rtc_motionCount++;
        ledBlink(1);   // 1 kort = beweging gedetecteerd
    }

#if ENABLE_LORA
    if (cause == ESP_SLEEP_WAKEUP_TIMER) {
        // 24-uurs timer: nieuwe dag aangenomen, teller resetten
        rtc_msgsToday = 0;
        // Meteen terug slapen — wacht op echte beweging
#if ENABLE_GPS_PWR
        gpio_hold_dis((gpio_num_t)PIN_GPS_PWR);
        gpio_deep_sleep_hold_dis();
        pinMode(PIN_GPS_PWR, OUTPUT);
        digitalWrite(PIN_GPS_PWR, LOW);
#endif
        goDeepSleep();
        return;
    }

    // Bij ext0 wakeup maar dagquotum al vol: direct terug slapen
    if (cause == ESP_SLEEP_WAKEUP_GPIO && rtc_msgsToday >= MAX_MSGS_PER_DAY) {
#if ENABLE_GPS_PWR
        gpio_hold_dis((gpio_num_t)PIN_GPS_PWR);
        gpio_deep_sleep_hold_dis();
        pinMode(PIN_GPS_PWR, OUTPUT);
        digitalWrite(PIN_GPS_PWR, LOW);
#endif
        goDeepSleep();
        return;
    }
#endif  // ENABLE_LORA

    // ── GPIO hold vrijgeven en GPS voeding instellen ─────────────────────
#if ENABLE_GPS_PWR
    gpio_hold_dis((gpio_num_t)PIN_GPS_PWR);
    gpio_deep_sleep_hold_dis();
    pinMode(PIN_GPS_PWR, OUTPUT);
    digitalWrite(PIN_GPS_PWR, HIGH);  // GPS aan
#endif

    // ── SW-520D input ────────────────────────────────────────────────────
    pinMode(PIN_SW520D, INPUT_PULLUP);

    // ── I2C scan + OLED ─────────────────────────────────────────────────
    Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
    Serial.println("I2C scan:");
    bool i2cFound = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  Gevonden: 0x%02X", addr);
            if (addr == 0x3C || addr == 0x3D) Serial.print(" <- OLED");
            Serial.println();
            i2cFound = true;
        }
    }
    if (!i2cFound) Serial.println("  Niets gevonden — check bedrading SDA/SCL");

    if (!oled.begin()) {
        Serial.println("OLED begin() mislukt!");
    } else {
        Serial.println("OLED OK");
    }

    // ── SPI + LoRa ───────────────────────────────────────────────────────
#if ENABLE_LORA
    SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);
    int loraState = radio.begin(
        LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SF, LORA_CR,
        LORA_SYNC_WORD, LORA_TX_POWER, LORA_PREAMBLE
    );
    if (loraState != RADIOLIB_ERR_NONE) {
        Serial.printf("LoRa init mislukt: %d\n", loraState);
    }
#endif

    // ── GPS UART (ontvang-only) ──────────────────────────────────────────
    // NEO-6M stuurt NMEA automatisch; geef 1s boot-tijd na powerup
    delay(1000);
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, -1);

    // ── Batterijspanning meten ───────────────────────────────────────────
    float batV    = readBatteryVoltage();
    uint8_t batPct = voltageToPercent(batV);

    // ── GPS fix proberen ─────────────────────────────────────────────────
    bool newFix      = false;
    uint32_t gpsStart    = millis();
    uint32_t lastOledMs  = 0;
    uint32_t lastTrigger = 0;
    bool prevSensor      = (digitalRead(PIN_SW520D) == LOW);  // beginstand

    while (millis() - gpsStart < GPS_FIX_TIMEOUT_MS) {
        while (gpsSerial.available()) {
            if (gps.encode(gpsSerial.read())) {
                if (gps.location.isValid() && gps.location.age() < 2000) {
                    newFix = true;
                }
            }
        }
        if (newFix) break;

        bool sensorNow = (digitalRead(PIN_SW520D) == LOW);
        uint32_t now   = millis();

        // Tellen op elke verandering (beide richtingen) + tijdsblokkering
        if ((sensorNow != prevSensor) && (now - lastTrigger >= SENSOR_DEBOUNCE_MS)) {
            rtc_motionCount++;
            lastTrigger = now;
        }
        prevSensor = sensorNow;

        // OLED update elke 200ms
        if (now - lastOledMs >= 200) {
            lastOledMs = now;
            uint8_t sats = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
            oledShowSearching(now - gpsStart, sensorNow, sats);
        }
    }

    // ── GPS resultaat verwerken ──────────────────────────────────────────
    float curLat = rtc_lastLat;
    float curLon = rtc_lastLon;

    if (newFix) {
        curLat = (float)gps.location.lat();
        curLon = (float)gps.location.lng();
        rtc_lastLat     = curLat;
        rtc_lastLon     = curLon;
        rtc_hasPosition = true;
        ledBlink(2);   // 2 kort = GPS fix

        // Dag-rollover detecteren via GPS datum
        if (gps.date.isValid()) {
            uint32_t today = encodeDateYMD(
                gps.date.year(), gps.date.month(), gps.date.day()
            );
            if (today != rtc_currentDay) {
                rtc_currentDay = today;
                rtc_msgsToday  = 0;
            }
        }
    }

    // ── LoRa verzenden indien quotum niet vol ────────────────────────────
    bool sent     = false;
    bool loraFail = false;

#if ENABLE_LORA
    if (rtc_hasPosition && rtc_msgsToday < MAX_MSGS_PER_DAY) {
        if (loraState == RADIOLIB_ERR_NONE) {
            sent = sendLoRaPacket(curLat, curLon, batPct);
            if (sent) {
                rtc_msgsToday++;
                rtc_totalMsgsSent++;
            } else {
                loraFail = true;
            }
        } else {
            loraFail = true;
        }
    }
#endif

    // ── Sensor testloop — live AAN/UIT indicator + satellite count ───────
    {
        uint32_t testStart    = millis();
        uint32_t extraMs      = 0;          // verlengingen via button
        uint32_t lastSensor   = 0;
        uint32_t btnPressedAt = 0;          // tijdstip button ingedrukt
        bool prevSensor       = (digitalRead(PIN_SW520D) == LOW);
        bool prevBtn          = (digitalRead(PIN_BTN) == LOW);
        uint8_t sats        = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;

        while (millis() - testStart < SENSOR_TEST_MS + extraMs) {
            bool sensorNow = (digitalRead(PIN_SW520D) == LOW);
            bool btnNow    = (digitalRead(PIN_BTN) == LOW);
            uint32_t now   = millis();
            uint32_t elapsed  = now - testStart;
            uint32_t total    = SENSOR_TEST_MS + extraMs;
            uint32_t secsLeft = elapsed < total ? (total - elapsed) / 1000 : 0;

            // ── Button detectie ─────────────────────────────────────────
            if (btnNow && !prevBtn) {
                btnPressedAt = now;           // begin indrukken
            }
            if (!btnNow && prevBtn) {
                uint32_t held = now - btnPressedAt;
                if (held >= 2000) {
                    extraMs += 30000;         // lang: +30s testijd
                    ledBlink(1, 400);         // 1 lang = verlengd
                } else {
                    rtc_motionCount = 0;      // kort: teller reset
                    ledBlink(3, 60, 60);      // 3 snel = gereset
                }
            }
            prevBtn = btnNow;

            // Alleen tellen op neergaande flank (open→gesloten) + tijdsblokkering
            if (sensorNow && !prevSensor && (now - lastSensor >= SENSOR_DEBOUNCE_MS)) {
                rtc_motionCount++;
                lastSensor = now;
            }
            prevSensor = sensorNow;

            // Satellietcount bijwerken als GPS nog data stuurt
            while (gpsSerial.available()) gps.encode(gpsSerial.read());
            if (gps.satellites.isValid()) sats = (uint8_t)gps.satellites.value();

            oledShowTest(curLat, curLon, newFix, sensorNow, sats, secsLeft);
            delay(50);  // ~20 fps
        }
    }

    // ── GPS UART stoppen en in slaap gaan ────────────────────────────────
    gpsSerial.end();
    goDeepSleep();
    // Komt hier nooit
}

void loop() {
    // Wordt nooit bereikt — ESP32 reset na elke deep sleep wakeup
}
