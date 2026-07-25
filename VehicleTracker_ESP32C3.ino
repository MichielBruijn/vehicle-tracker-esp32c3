/*
 * VehicleTracker_ESP32C3
 * ──────────────────────────────────────────────────────────────────────────────
 * Deep-sleep GPS tracker for long battery life on a vehicle.
 * The SW-520D vibration/tilt sensor wakes the ESP32-C3 from deep sleep.
 * Every 24 hours the position is transmitted if the vehicle has moved.
 *
 * HARDWARE:
 *   ESP32-C3 Super Mini (or similar)
 *   ATGM336H GPS module                     — UART, 9600 baud NMEA
 *   DX-LR30-900M22S (Semtech SX1262)       — SPI, LoRaWAN EU868
 *   SSD1306 0.96" OLED (4-pin, I2C)        — 128×64 pixels
 *   SW-520D vibration/tilt sensor
 *
 * WIRING:
 * ┌──────────────┬───────────────┬──────────────────────────────────────────┐
 * │ ESP32-C3 pin │ Connected to  │ Note                                     │
 * ├──────────────┼───────────────┼──────────────────────────────────────────┤
 * │ GPIO0        │ LoRa DIO1     │ SX1262 DIO1 interrupt                   │
 * │ GPIO1        │ GPS TX        │ UART1 RX — receives NMEA from GPS module │
 * │ GPIO2        │ CD4013B Q1    │ Latch output: HIGH = motion occurred     │
 * │              │               │ SW-520D now on CD4013B Set input         │
 * │ GPIO3        │ LoRa SCK      │ SPI clock (pure SCK, no shared function) │
 * │ GPIO4        │ LoRa BUSY     │ SX1262 BUSY                              │
 * │ GPIO5        │ LoRa MISO     │ SPI data in                              │
 * │ GPIO6        │ LoRa MOSI     │ SPI data out                             │
 * │ GPIO7        │ LoRa NSS      │ SPI chip select (active low)             │
 * │ GPIO8        │ IRLB3034 Gate │ GPS via ME6211C CE: HIGH=GPS off         │
 * │              │               │ Strapping pull-up keeps GPIO8 HIGH in sleep│
 * │ GPIO9        │ free          │ (BOOT pullup 10kΩ→3V3, do not use)      │
 * │ GPIO10       │ GPS RX /      │ UART1 TX to GPS; also CD4013B R1 reset   │
 * │              │ CD4013B R1    │ Pulse HIGH on wakeup to clear the latch  │
 * │ GPIO20       │ OLED SDA      │ I2C data                                 │
 * │ GPIO21       │ OLED SCL      │ I2C clock                                │
 * └──────────────┴───────────────┴──────────────────────────────────────────┘
 *
 * DX-LR30-900M22S MODULE CONNECTIONS (SMD pins):
 *   GND  → GND
 *   VCC  → 3.3V  (2.0–3.7V range)
 *   SCK  → GPIO3
 *   MOSI → GPIO6
 *   MISO → GPIO5
 *   NSS  → GPIO7
 *   NRST → do not connect (RADIOLIB_NC, software reset via SPI)
 *   BUSY → GPIO4
 *   DIO1 → GPIO0
 *   ANT  → antenna (always connect before powering on!)
 *
 * GPS POWER:
 *   ME6211C33 LDO switches GPS VCC (CE active HIGH: CE HIGH = LDO on, CE LOW = LDO off)
 *   IRLB3034 N-channel MOSFET as inverter on GPIO8:
 *     GPIO8 HIGH → MOSFET on  → drain low  → CE LOW  → ME6211C off → GPS off
 *     GPIO8 LOW  → MOSFET off → drain high (160kΩ→3V3) → CE HIGH → ME6211C on → GPS on
 *   Strapping pull-up (~8kΩ) keeps GPIO8 HIGH in deep sleep → GPS always off, no gpio_hold needed
 *   Sleep current of the switching circuit: 3.3V / 160kΩ ≈ 20µA
 *   GPS TX → GPIO1 (receive NMEA), GPS RX → GPIO10 (send NMEA commands)
 *
 * LORAWAN PAYLOAD (14 bytes, port 1):
 *   Byte  0    : Status (uint8)
 *                  0 = no motion, cached position (probably still current)
 *                  1 = motion + new GPS fix
 *                  2 = motion but GPS timeout, cached position (vehicle may have moved)
 *   Bytes 1–4  : Latitude  × 1 000 000 (int32, little-endian)
 *   Bytes 5–8  : Longitude × 1 000 000 (int32, little-endian)
 *   Bytes 9–11 : Altitude  × 100       (int32, 3 bytes, little-endian)
 *   Bytes 12–13: HDOP      × 100       (int16, little-endian)
 *
 * TTN PAYLOAD FORMATTER (JavaScript, paste into TTN console → Payload formatters):
 *   function decodeUplink(input) {
 *     var b = input.bytes;
 *     function int32le(b,i){var v=b[i]|(b[i+1]<<8)|(b[i+2]<<16)|(b[i+3]<<24);return v;}
 *     function int24le(b,i){var v=b[i]|(b[i+1]<<8)|(b[i+2]<<16);if(v&0x800000)v|=0xFF000000;return v|0;}
 *     var status    = b[0]; // 0=cache, 1=new fix, 2=moved+no fix
 *     var latitude  = int32le(b,1) / 1e6;
 *     var longitude = int32le(b,5) / 1e6;
 *     var altitude  = int24le(b,9) / 100;
 *     var hdop      = (b[12]|(b[13]<<8)) / 100;
 *     return { data: { status: status, latitude: latitude, longitude: longitude, altitude: altitude, hdop: hdop } };
 *   }
 *
 * LORAWAN SESSION (deep sleep persistence):
 *   After a successful OTAA join the session (DevAddr, keys, frame counters)
 *   is stored in RTC memory. On every wakeup the session is restored so that
 *   rejoining is not needed. Only on RTC loss (battery dead) a rejoin happens
 *   automatically.
 *
 * POWER SUPPLY:
 *   Li-ion battery → HT7833 (3.3V LDO, ~55µA quiescent) → 3V3 pin ESP32-C3
 *   Onboard AMS1117 stays in place but is inactive (no USB input during use).
 *
 * BATTERY LIFE (estimate):
 *   Deep sleep:       ESP32-C3 ~5µA + HT7833 ~4µA + voltage divider ~2µA ≈ 11µA
 *   GPS warm start:   ~20-30s at 20mA (ATGM336H after PMTK standby; almanac in flash)
 *   GPS cold start:   30–120s at 20mA (only on first use or after months)
 *   LoRa TX (SF9):    ~280ms at ~90mA (SX1262 at 22 dBm)
 *   OLED:             4s at 10mA
 *   Per wakeup:       ≈ 5s×30mA + 0.3s×90mA + 4s×10mA ≈ 0.20 mAh
 *   3× per day:       0.60 mAh active + 24h×0.007mA = 0.17 mAh sleep ≈ 0.77 mAh/day
 *   18650 (3000 mAh, 80% usable = 2400 mAh):
 *     2400 / 0.77 ≈ 3117 days ≈ 8.5 YEARS (with warm GPS start)
 *   Worst-case cold start every time (60s):
 *     3×(60s×30mA + 0.3s×90mA + 4s×10mA) ≈ 5.6 mAh/day → ~428 days ≈ 1.2 years
 *   TIP: The backup battery on the GPS module is crucial for long battery life!
 *        Make sure it is not empty (runs for years on the built-in cell).
 *
 * OPERATION:
 *   - Timer wakes the ESP32 once every 24 hours
 *   - CD4013B SR latch: SW-520D contacts close → Q1 HIGH (latched), regardless of wakeup moment
 *   - Wakeup: read Q1 (GPIO2), reset the latch (GPIO10 pulse HIGH)
 *   - Motion detected: turn GPS on, wait up to 90s for a first fix,
 *     then refine for up to 10s; status 1 (new fix) or 2 (timeout, cache)
 *   - No motion, position known: send cached position, GPS stays off; status 0
 *   - No motion, no position yet: sleep immediately (nothing to send)
 *   - First boot: always join and send a GPS fix as initialisation
 *
 * REQUIRED LIBRARIES (Arduino Library Manager):
 *   - RadioLib  by Jan Gromeš  (SX1262 support built in)
 *   - TinyGPS++ by Mikal Hart
 *   - U8g2      by Oliver Kraus
 *

 */

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <TinyGPS++.h>
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "soc/usb_serial_jtag_reg.h"
#include "esp_private/periph_ctrl.h"

// ═══════════════════════════════════════════════════════════════════════════
//  TEST CONFIGURATION
//  Set to 1 if the corresponding component is connected.
// ═══════════════════════════════════════════════════════════════════════════
#define ENABLE_LORA       1   // DX-LR30-900M22S / SX1262
#define ENABLE_GPS_PWR    1   // GPS switch: IRLB3034 inverter + ME6211C33 LDO on GPIO8
#define ENABLE_WIFI_DEBUG 0   // WiFi + OTA + telnet (set to 0 for production!)
#define ENABLE_SERIAL     0   // Serial debug output (set to 0 if USB CDC On Boot = Disabled)

#if ENABLE_SERIAL
  #define DBG_BEGIN(baud) Serial.begin(baud)
  #define DBG(...)        Serial.print(__VA_ARGS__)
  #define DBGLN(...)      Serial.println(__VA_ARGS__)
  #define DBGF(...)       Serial.printf(__VA_ARGS__)
  #define DBG_FLUSH()     Serial.flush()
#else
  #define DBG_BEGIN(baud)
  #define DBG(...)
  #define DBGLN(...)
  #define DBGF(...)
  #define DBG_FLUSH()
#endif

#if ENABLE_WIFI_DEBUG
  #include <WiFi.h>
  #include <ArduinoOTA.h>
#endif

// ═══════════════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════
#define PIN_GPS_RX      1   // GPS TX of module → GPIO1 (ESP32 receives NMEA here)
#define PIN_GPS_TX      10  // GPS RX → GPIO10 (freed up from LoRa RST)
#define PIN_GPS_PWR     8   // IRLB3034 gate: HIGH = GPS off (CE LOW), LOW = GPS on (CE HIGH)
#define PIN_SW520D      2   // CD4013B Q1: HIGH = motion occurred
#define PIN_LORA_BUSY   4   // SX1262 BUSY
#define PIN_LORA_SCK    3
#define PIN_LORA_MISO   5
#define PIN_LORA_MOSI   6
#define PIN_LORA_CS     7
#define PIN_LORA_DIO1   0   // SX1262 DIO1 interrupt
#define PIN_LORA_RST    RADIOLIB_NC  // RST not connected
#define PIN_OLED_SDA    20  // I2C data (SDA) — GPIO20
#define PIN_OLED_SCL    21  // I2C clock (SCL) — GPIO21

// ═══════════════════════════════════════════════════════════════════════════
//  CONFIGURATION — adjust as needed
// ═══════════════════════════════════════════════════════════════════════════
// Credentials come from secrets.h (see that file)

#define GPS_BAUD            9600
#define GPS_FIX_TIMEOUT_MS  90000UL    // Wait at most 90s for a first GPS fix
#define GPS_REFINE_MS       10000UL    // Refine for at most 10s after the first fix

#define SLEEP_SEC           86400ULL  // CD4013B latch: wakeup once per day
//#define SLEEP_SEC           5ULL  // CD4013B latch: wakeup once per day

// Conditional includes — only load what is actually wired up
#if ENABLE_LORA
#include <SPI.h>
#include <RadioLib.h>
#include "secrets.h"
#endif

// LED resistor removed — LED functions are no-ops
void ledOn()  {}
void ledOff() {}
void ledBlink(uint8_t n, uint16_t onMs = 80, uint16_t offMs = 120) { (void)n; (void)onMs; (void)offMs; }

// ═══════════════════════════════════════════════════════════════════════════
//  RTC MEMORY — survives deep sleep
// ═══════════════════════════════════════════════════════════════════════════
RTC_DATA_ATTR float    rtc_lastLat       = 0.0f;
RTC_DATA_ATTR float    rtc_lastLon       = 0.0f;
RTC_DATA_ATTR float    rtc_lastAlt       = 0.0f;
RTC_DATA_ATTR float    rtc_lastHdop      = 99.0f;
RTC_DATA_ATTR bool     rtc_hasPosition   = false;
RTC_DATA_ATTR uint32_t rtc_wakeupCount      = 0;
RTC_DATA_ATTR uint32_t rtc_motionCount      = 0;   // Number of 24h periods with motion transmitted
RTC_DATA_ATTR uint32_t rtc_totalMsgsSent    = 0;

// LoRaWAN session persistence across deep sleep
RTC_DATA_ATTR bool    rtc_lwJoined  = false;
RTC_DATA_ATTR uint8_t rtc_lwNonces[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
RTC_DATA_ATTR uint8_t rtc_lwSession[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];

// ═══════════════════════════════════════════════════════════════════════════
//  OBJECTS
// ═══════════════════════════════════════════════════════════════════════════
HardwareSerial gpsSerial(1);
TinyGPSPlus    gps;

// SSD1306 128×64 via hardware I2C — pass pins explicitly to the constructor
// Order: (rotation, reset, clock, data)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE, PIN_OLED_SCL, PIN_OLED_SDA);

#if ENABLE_LORA
// SX1262 (DX-LR30): CS, DIO1=NC (polling via BUSY on GPIO3), RST, BUSY
SX1262 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY);
LoRaWANNode node(&radio, &EU868);
#endif

void gpsSleep() {
    // GPS power is cut via PIN_GPS_PWR in goDeepSleep()
}

void gpsWakeup() {
#if ENABLE_GPS_PWR
    pinMode(PIN_GPS_PWR, OUTPUT);
    digitalWrite(PIN_GPS_PWR, LOW);   // MOSFET off: CE HIGH → ME6211C on → GPS on
    delay(800);
#endif
    pinMode(PIN_GPS_TX, OUTPUT);
    digitalWrite(PIN_GPS_TX, HIGH);
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
}

// ═══════════════════════════════════════════════════════════════════════════
//  HELPERS
// ═══════════════════════════════════════════════════════════════════════════


void oledShowSearching(uint32_t elapsed_ms, bool sensorActive, uint8_t sats) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 10, "VehicleTracker");
    oled.drawHLine(0, 12, 128);

    // GPS search status + satellites
    char gpsLine[22];
    snprintf(gpsLine, sizeof(gpsLine), "GPS search... sat:%u", sats);
    oled.drawStr(0, 24, gpsLine);

    // Progress bar (max GPS_FIX_TIMEOUT_MS)
    uint8_t barWidth = (uint8_t)((elapsed_ms * 118UL) / GPS_FIX_TIMEOUT_MS);
    if (barWidth > 118) barWidth = 118;
    oled.drawFrame(4, 28, 120, 8);
    if (barWidth > 0) oled.drawBox(5, 29, barWidth, 6);

    // Sensor indicator
    oled.drawStr(0, 46, "Sensor:");
    if (sensorActive) {
        oled.drawBox(50, 37, 40, 11);
        oled.setDrawColor(0);
        oled.drawStr(54, 46, "ON");
        oled.setDrawColor(1);
    } else {
        oled.drawFrame(50, 37, 40, 11);
        oled.drawStr(54, 46, "---");
    }

    char buf[22];
    snprintf(buf, sizeof(buf), "%lus  Motion:%lu", elapsed_ms / 1000, rtc_motionCount);
    oled.drawStr(0, 62, buf);
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
        oled.drawStr(0, 10, "No GPS fix");
        oled.drawStr(0, 22, "---");
    }

    oled.drawHLine(0, 25, 128);

    // Sensor indicator — big and clear
    oled.drawStr(0, 37, "Sensor:");
    if (sensorActive) {
        oled.drawBox(44, 28, 26, 11);
        oled.setDrawColor(0);
        oled.drawStr(47, 37, "ON");
        oled.setDrawColor(1);
    } else {
        oled.drawFrame(44, 28, 26, 11);
        oled.drawStr(47, 37, "---");
    }

    char satBuf[16];
    snprintf(satBuf, sizeof(satBuf), "sat:%u", sats);
    oled.drawStr(74, 37, satBuf);

    char motBuf[22];
    snprintf(motBuf, sizeof(motBuf), "Motion: %lu", rtc_motionCount);
    oled.drawStr(0, 50, motBuf);

    char slpBuf[22];
    snprintf(slpBuf, sizeof(slpBuf), "Sleep in: %lus", secsLeft);
    oled.drawStr(0, 62, slpBuf);

    oled.sendBuffer();
}

void oledShowStatus(float lat, float lon, bool newFix, bool sent, bool loraFail) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);

    oled.drawStr(0, 10, "VehicleTracker");
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
        oled.drawStr(0, 26, "No GPS fix");
        oled.drawStr(0, 38, "No position");
    }

    oled.drawStr(0, 52, "VehicleTracker");

#if ENABLE_LORA
    oled.setFont(u8g2_font_5x7_tf);
    if (sent) {
        oled.drawStr(72, 52, "Tx:SENT");
    } else if (loraFail) {
        oled.drawStr(72, 52, "Tx:FAIL");
    } else {
        oled.drawStr(72, 52, "Tx:---");
    }
    oled.setFont(u8g2_font_6x10_tf);
#else
    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(72, 52, "LoRa:OFF");
    oled.setFont(u8g2_font_6x10_tf);
#endif

    char motStr[22];
    snprintf(motStr, sizeof(motStr), "Motion: %lu", rtc_motionCount);
    oled.drawStr(0, 62, motStr);
    oled.sendBuffer();
}


void goDeepSleep(bool gpsStarted = false, bool peripheralsStarted = true) {
#if ENABLE_WIFI_DEBUG
    if (WiFi.status() == WL_CONNECTED) {
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        delay(100);
    }
#endif
    if (gpsStarted) {
        gpsSleep();
        gpsSerial.end();
        // gpsSerial.end() sometimes leaves UART TX (GPIO10 = R1) high — would hold
        // the CD4013B latch in reset during sleep
        pinMode(PIN_GPS_TX, OUTPUT);
        digitalWrite(PIN_GPS_TX, LOW);
    }
#if ENABLE_GPS_PWR
    // Strapping pull-up (~8kΩ) pulls GPIO8 HIGH in deep sleep → MOSFET on → GPS off
    pinMode(PIN_GPS_PWR, INPUT);
#endif

#if ENABLE_LORA
    if (peripheralsStarted) {
        radio.sleep();
        delay(10);
    }
#endif

    if (peripheralsStarted) {
        oled.setPowerSave(1);
        // Disable the SSD1306 charge pump — setPowerSave only sends Display OFF (0xAE),
        // otherwise the charge pump keeps running and draws ~1mA
        Wire.beginTransmission(0x3C);
        Wire.write(0x00);   // command mode
        Wire.write(0x8D);   // Charge Pump Setting
        Wire.write(0x10);   // disable
        Wire.endTransmission();
    }

    esp_sleep_enable_timer_wakeup(SLEEP_SEC * 1000000ULL);

    // Switch off unneeded power domains
    esp_sleep_pd_config(ESP_PD_DOMAIN_RC_FAST, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL,    ESP_PD_OPTION_OFF);

    // LP GPIOs: disable pull-up/-down via gpio_pullup/down_dis().
    // Arduino's pinMode(INPUT) only configures the HP GPIO matrix, not the LP IO
    // peripheral. GPIO2 is also a strapping pin with a built-in pull-up (~16kΩ) that
    // stays active in deep sleep → Q1 LOW sinks 200µA. gpio_pullup_dis() also
    // covers the LP-domain register and fixes that.
    gpio_pullup_dis(GPIO_NUM_2);
    gpio_pulldown_dis(GPIO_NUM_2);
    gpio_pullup_dis(GPIO_NUM_3);
    gpio_pulldown_dis(GPIO_NUM_3);  // SCK: SPI.begin() may leave a pull-up behind
    pinMode(PIN_SW520D,   INPUT);   // LP GPIO2: HP matrix high-Z as well
    pinMode(PIN_LORA_SCK, INPUT);   // LP GPIO3: SPI.begin() sets it as OUTPUT

    // Disable the USB Serial/JTAG PHY — as late as possible, sleep entry overrides it otherwise
    REG_CLR_BIT(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_USB_PAD_ENABLE);
    esp_deep_sleep_start();
    // Never reached — ESP resets after wakeup
}

// ═══════════════════════════════════════════════════════════════════════════
//  WIFI DEBUG — OTA + telnet on port 23
// ═══════════════════════════════════════════════════════════════════════════
#if ENABLE_WIFI_DEBUG
static WiFiServer _telnetSrv(23);
static WiFiClient _telnetClient;

void wifiLog(const char* s) {
    Serial.print(s);
    if (_telnetClient && _telnetClient.connected()) _telnetClient.print(s);
}

void wifiLogf(const char* fmt, ...) {
    char buf[160];
    va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
    wifiLog(buf);
}

void wifiDebugBegin() {
    Serial.printf("Connecting WiFi to %s...\n", SECRET_WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASS);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) delay(100);
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi failed — continuing without");
        return;
    }
    Serial.printf("WiFi OK — IP: %s\n", WiFi.localIP().toString().c_str());
    ArduinoOTA.setHostname("vehicletracker");
    ArduinoOTA.onStart([]() { Serial.println("OTA start"); });
    ArduinoOTA.onEnd([]()   { Serial.println("OTA done"); });
    ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA error %u\n", e); });
    ArduinoOTA.begin();
    _telnetSrv.begin();
    Serial.printf("Telnet: telnet %s\n", WiFi.localIP().toString().c_str());
    Serial.println("OTA: visible in Arduino IDE under Tools > Port");
}

// Run this in a loop — handles OTA and telnet clients
void wifiDebugHandle() {
    ArduinoOTA.handle();
    if (!_telnetClient || !_telnetClient.connected()) {
        _telnetClient = _telnetSrv.accept();
        if (_telnetClient) Serial.println("Telnet client connected");
    }
    // Echo telnet input back (optional)
    while (_telnetClient && _telnetClient.available()) _telnetClient.read();
}
#else
// Stubs so wifiLog() can be used everywhere without #if
void wifiLog(const char* s)                 { DBG(s); }
void wifiLogf(const char* fmt, ...) {
    char buf[160]; va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
    DBG(buf);
}
void wifiDebugBegin()  {}
void wifiDebugHandle() {}
#endif

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP — everything happens here, loop() is never reached (deep sleep + reset)
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
#if ENABLE_GPS_PWR
    pinMode(PIN_GPS_PWR, OUTPUT);
    digitalWrite(PIN_GPS_PWR, HIGH);  // MOSFET on: CE LOW → ME6211C off → GPS off
#endif

    // ── CD4013B latch: read, then reset ──────────────────────────────────
    // Read first (Q1), only then reset — otherwise the motion event is lost
    pinMode(PIN_SW520D, INPUT);           // GPIO2 = Q1, no pull-up
    delayMicroseconds(100);
    bool hasMoved = (digitalRead(PIN_SW520D) == HIGH);

    pinMode(PIN_GPS_TX, OUTPUT);          // GPIO10 = R1 reset pulse
    digitalWrite(PIN_GPS_TX, HIGH);       // clear the latch
    delayMicroseconds(100);
    digitalWrite(PIN_GPS_TX, LOW);

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    bool isTimerWakeup = (cause == ESP_SLEEP_WAKEUP_TIMER);
    bool isFirstBoot   = !isTimerWakeup;

    rtc_wakeupCount++;

    // ── No motion and no cached position → go straight back to sleep ────
    // If we do have a cached position, we send it (GPS stays off).
    if (isTimerWakeup && !hasMoved && !rtc_hasPosition) {
        goDeepSleep(false, false);
    }

    // ── From here on: first boot or 24h heartbeat ────────────────────────
    DBG_BEGIN(115200);
    delay(100);
    wifiDebugBegin();

    DBGF("Wakeup: %s  hasMoved:%d  wakeupCount:%lu\n",
         isFirstBoot ? "first boot" : "24h heartbeat",
         hasMoved, rtc_wakeupCount);

    // ── First boot: clear the LoRa session ──────────────────────────────
    if (isFirstBoot) {
        rtc_lwJoined = false;
        memset(rtc_lwNonces,  0, sizeof(rtc_lwNonces));
        memset(rtc_lwSession, 0, sizeof(rtc_lwSession));
        DBGLN("First boot — RTC LoRa cleared");
    } else if (hasMoved) {
        rtc_motionCount++;
        DBGF("24h heartbeat: motion — new GPS + TX (#%lu)\n", rtc_motionCount);
    } else {
        DBGLN("24h heartbeat: no motion — cached position TX");
    }

    // ── LED not in use (resistor removed) ────────────────────────────────

    // ── OLED ─────────────────────────────────────────────────────────────
    Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
    oled.begin();

    // ── SPI + LoRaWAN ────────────────────────────────────────────────────
#if ENABLE_LORA
    SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);
    pinMode(PIN_LORA_CS,   OUTPUT); digitalWrite(PIN_LORA_CS,   HIGH);
    pinMode(PIN_LORA_BUSY, INPUT);

    // The SX1262 may be in a CMD_TIMEOUT state after the previous boot (expired RX window).
    // RadioLib checks the STATUS byte on every SPI command and then returns -705.
    // One raw SetStandby before radio.begin() clears that sticky status without
    // RadioLib checking the STATUS byte — after that radio.begin() works normally.
    {
        uint32_t _bt = millis();
        while (digitalRead(PIN_LORA_BUSY) && millis() - _bt < 500) { delay(1); }
        if (!digitalRead(PIN_LORA_BUSY)) {
            SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
            digitalWrite(PIN_LORA_CS, LOW);  delayMicroseconds(50);
            SPI.transfer(0x80); SPI.transfer(0x00);  // CMD_SET_STANDBY STDBY_RC
            delayMicroseconds(50);  digitalWrite(PIN_LORA_CS, HIGH);
            SPI.endTransaction();
            delay(5);
        } else {
            DBGLN("SX1262 BUSY stuck — power cycle the module");
        }
    }

    bool loraReady = false;
    int loraState = radio.begin(868.0, 125.0, 9, 7,
                                RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8, 0.0, true);
    if (loraState != RADIOLIB_ERR_NONE) {
        DBGF("Radio init failed: %d\n", loraState);
        oled.clearBuffer();
        oled.setFont(u8g2_font_6x10_tf);
        oled.drawStr(0, 20, "Radio FAIL");
        char _eb[20]; snprintf(_eb, sizeof(_eb), "err: %d", loraState);
        oled.drawStr(0, 35, _eb);
        oled.sendBuffer();
        delay(3000);
    } else {
        node.beginOTAA(SECRET_JOIN_EUI, SECRET_DEV_EUI,
                       nullptr, (uint8_t*)SECRET_APP_KEY);
        if (rtc_lwJoined) {
            node.setBufferNonces(rtc_lwNonces);
            node.setBufferSession(rtc_lwSession);
            if (node.isActivated()) {
                DBGLN("LoRaWAN session restored");
                loraReady = true;
            } else {
                DBGLN("Session expired, rejoining...");
                rtc_lwJoined = false;
            }
        }
        if (!rtc_lwJoined) {
            oled.clearBuffer();
            oled.setFont(u8g2_font_6x10_tf);
            oled.drawStr(0, 20, "LoRa join...");
            oled.sendBuffer();
            loraState = node.activateOTAA();
            if (node.isActivated()) {
                memcpy(rtc_lwNonces,  node.getBufferNonces(),  RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
                memcpy(rtc_lwSession, node.getBufferSession(), RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
                rtc_lwJoined = true;
                loraReady    = true;
                DBGLN("LoRaWAN join successful");
                ledBlink(1, 500); ledOn();
            } else {
                DBGF("LoRaWAN join failed: %d\n", loraState);
                oled.clearBuffer();
                oled.setFont(u8g2_font_6x10_tf);
                oled.drawStr(0, 20, "Join FAIL");
                char _jb[20]; snprintf(_jb, sizeof(_jb), "err: %d", loraState);
                oled.drawStr(0, 35, _jb);
                oled.sendBuffer();
                delay(3000);
            }
        }
    }
#endif

    // ── GPS: only on motion or first boot, not on heartbeat ──────────────
    bool  newFix  = false;
    float curLat  = rtc_lastLat;
    float curLon  = rtc_lastLon;
    float curAlt  = rtc_lastAlt;
    float curHdop = rtc_lastHdop;

    bool gpsNeeded  = hasMoved || isFirstBoot;
    bool gpsStarted = false;

    // Get a GPS fix only on motion or first boot — otherwise send the cached position
    if (gpsNeeded) {
        gpsWakeup();
        gpsStarted = true;

        float    bestHdop   = 99.0f;
        float    bestLat    = 0.0f, bestLon = 0.0f, bestAlt = 0.0f;
        uint32_t gpsStart   = millis();
        uint32_t firstFixMs = 0;
        uint32_t lastOledMs = 0;

        while (true) {
            uint32_t now = millis();
            if (!newFix && now - gpsStart  >= GPS_FIX_TIMEOUT_MS) break; // 90s without a fix
            if ( newFix && now - firstFixMs >= GPS_REFINE_MS)      break; // 10s of refining

            while (gpsSerial.available()) {
                char _c = gpsSerial.read();
                if (gps.encode(_c)) {
                    if (gps.location.isValid() && gps.location.age() < 2000) {
                        // ATGM336H sends $GNGSA instead of $GPGSA → TinyGPS++ cannot parse HDOP
                        // default 2.0 so a fix with enough satellites is accepted immediately
                        float h = gps.hdop.isValid() ? gps.hdop.hdop() : 2.0f;
                        if (h < bestHdop) {
                            bestHdop = h;
                            bestLat  = (float)gps.location.lat();
                            bestLon  = (float)gps.location.lng();
                            bestAlt  = gps.altitude.isValid() ? (float)gps.altitude.meters() : 0.0f;
                            if (!newFix) firstFixMs = millis();
                            newFix   = true;
                        }
                        if (h < 2.5f) break;
                    }
                }
            }
            if (newFix && bestHdop < 2.5f) break;

            if (millis() - lastOledMs >= 200) {
                lastOledMs = millis();
                uint8_t sats = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
                oledShowSearching(millis() - gpsStart, hasMoved, sats);
            }
            wifiDebugHandle();
        }

        if (newFix) {
            curLat  = bestLat;  curLon  = bestLon;
            curAlt  = bestAlt;  curHdop = bestHdop;
            rtc_lastLat  = curLat;  rtc_lastLon  = curLon;
            rtc_lastAlt  = curAlt;  rtc_lastHdop = curHdop;
            rtc_hasPosition = true;
            DBGF("GPS fix: %.6f, %.6f hdop=%.1f\n",
                 (double)curLat, (double)curLon, (double)curHdop);
            ledBlink(2); ledOn();
        }
    } // if (gpsNeeded)

    // ── LoRa transmit ────────────────────────────────────────────────────
    bool sent     = false;
    bool loraFail = false;

#if ENABLE_LORA
    if (rtc_hasPosition) {
        if (loraReady) {
            uint8_t port = 1;
            uint8_t payload[14];
            // status: 0=cache/no motion, 1=new fix, 2=moved+no fix
            payload[0] = newFix ? 1 : (hasMoved ? 2 : 0);
            int32_t latF  = (int32_t)(curLat  * 1000000.0f);
            int32_t lonF  = (int32_t)(curLon  * 1000000.0f);
            int32_t altF  = (int32_t)(curAlt  * 100.0f);
            int16_t hdopF = (int16_t)(curHdop * 100.0f);
            memcpy(&payload[1],  &latF,  4);
            memcpy(&payload[5],  &lonF,  4);
            memcpy(&payload[9],  &altF,  3);
            memcpy(&payload[12], &hdopF, 2);
            int state = node.sendReceive(payload, sizeof(payload), port);
            sent = (state == RADIOLIB_ERR_NONE || state == RADIOLIB_LORAWAN_DOWNLINK);
            if (sent) { rtc_totalMsgsSent++; ledBlink(3); ledOn(); }
            else       loraFail = true;
        } else {
            loraFail = true;
        }
    }
    // Always save the session before sleep — even without a TX (e.g. no GPS fix).
    // Without this node.isActivated() is false on the next wakeup.
    if (rtc_lwJoined) {
        memcpy(rtc_lwNonces,  node.getBufferNonces(),  RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
        memcpy(rtc_lwSession, node.getBufferSession(), RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
    }
#endif

    // ── Status screen ────────────────────────────────────────────────────
    oledShowStatus(curLat, curLon, newFix, sent, loraFail);
    delay(4000);

    DBGLN("Going to sleep...");
    DBG_FLUSH();
    goDeepSleep(gpsStarted);
    // Never reached
}

void loop() {
    // Never reached — the ESP32 resets after every deep sleep wakeup
}
