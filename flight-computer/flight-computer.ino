// === FlightComputer_ESP32_v3_9_0.ino ===
/*
 * Flight Computer Firmware for ESP32 – Version 3.9.0
 *
 * ─────────────────────────────────────────────────────────────
 *  What’s new since 3.8.0 (Reliability, Safety Guards, Logging/IO fixes):
 *
 *  ✦ Sensor Safety & Robustness:
 *    - Hardened all BMP280/MPU6050 call sites: no more accidental reads when a sensor is missing.
 *    - Prevents NaN propagation into altitude/vertical speed logic when BMP280 is not available.
 *    - Calibration and arming now validate sensor availability and will not silently compute garbage.
 *
 *  ✦ Flight/Altitude Logic Fixes:
 *    - Tracks armedMaxRelativeAltitude continuously while ARMED (so altitudeDrop is correct in real flights).
 *    - Altitude-based triggers are automatically suppressed if BMP280 is not available (safety behavior).
 *
 *  ✦ API Pressure Fetch Fix:
 *    - apiSuccess is reset at each API fetch attempt (previously could remain “true” indefinitely).
 *
 *  ✦ SD / Endpoint Behavior Fixes:
 *    - /uploadsd now correctly returns “SD feature disabled” (501) when FEATURE_SD_MMC == 0.
 *    - (Previously it could claim success even though SD writes were compiled out.)
 *
 *  ✦ Comments / Maintainability:
 *    - Added extensive comments across flight logic, telemetry, logging, and file management.
 *    - Clarified several ambiguous behaviors and “gotchas” (axis mapping, NaN behavior, trigger gating).
 *
 *  Release Date: December 2025
 *
 * ┌───────────────────────────────────────────────┐
 * │                  Endpoints Overview           │
 * └───────────────────────────────────────────────┘
 *
 * ───── HTTP REST API & Static Content ──────────────
 *  GET   /                  → Redirects to /index.html, sets flight mode
 *  GET   /visualization     → Redirects to visualization UI, sets visualization mode
 *  GET   /visualization/    → Serves static visualization files from SPIFFS
 *  GET   /listfiles         → Lists files on SD & SPIFFS (JSON)
 *  GET   /files             → Redirect to UI for files on SD & SPIFFS
 *  GET   /deleteFile        → Delete file from SD or SPIFFS (param: name)
 *  GET   /deleteFileSPIFFS  → Delete file from SPIFFS (param: name)
 *  GET   /deleteFileSD      → Delete file from SD card (param: name)
 *  GET   /downloadFile      → Download file from SD, fallback to SPIFFS (param: name)
 *  GET   /downloadFileSD    → Download file from SD only
 *  GET   /downloadFileSPIFFS→ Download file from SPIFFS only
 *  POST  /upload            → Upload file to SPIFFS (multipart/form-data)
 *  POST  /uploadsd          → Upload file to SD card (multipart/form-data) (501 if SD disabled)
 *  GET   /spiffsupload      → HTML page for uploading files to SPIFFS (multi-file/folder support)
 *  POST  /spiffsupload      → Actual SPIFFS upload handler (uses folder argument)
 *  GET   /gyro              → Get gyroscope readings as JSON
 *  GET   /acc               → Get accelerometer readings as JSON
 *  GET   /temp              → Get temperature (plain text)
 *  GET   /reset             → Reset all gyro values to zero
 *  GET   /resetX|/resetY|/resetZ → Reset specific gyro axis to zero
 *  GET   /index.html, /style.css, /script.js, /visualization/* → Serve static files from SPIFFS
 *
 * ───── OTA Firmware Update ──────────────
 *  POST  /update            → Firmware update via ESP32 HTTPUpdateServer (auto-registered)
 *
 * ───── WebSocket (port 81) ──────────────
 *  Handles:
 *      - Sensor and threshold settings (JSON keys: newThreshold, newAccX/Y/Z, enAltDrop/AccX/Y/Z, etc.)
 *      - Trigger logic selection (AND/OR)
 *      - Sensor calibration (calibrateSensors)
 *      - Parachute commands (Armed/Released/Unarmed)
 *      - Axis correction config (axisConfig)
 *      - Location update (latitude/longitude) triggers API pressure refresh and broadcasts new settings
 *      - Live telemetry/diagnostic broadcast to all clients
 *
 * ───── Outbound API Calls ──────────────
 *  - OpenWeatherMap API for live pressure calibration
 *
 * ───── Dashboard & LED Feedback Reference ──────────────
 *  • Color Wheel Indices (0–255 → RGB via strip.Wheel(index)):
 *      0   → red
 *     30   → orange
 *     85   → green
 *    125   → aqua
 *    170   → blue
 *    200   → purple
 *
 *  • Core LED Functions:
 *      setAllLEDs(color)
 *      blinkColor(color, times, delayms)
 *      showLEDColorsSequentially(color, direction, rotations)
 *
 *  • Data Logging:
 *      – All data always logged to SD card if available
 *      – Data also logged to SPIFFS as long as there is enough free space
 *      – If SPIFFS free space drops below threshold:
 *          - Attempt offload to SD (if enabled/mounted)
 *          - Otherwise suspend SPIFFS logging and show warning indicator
 */

// Forward declaration so Arduino's auto-prototype knows the type for functions below.
struct DelStats;

// ============================================================================
//                         BUILD / BOARD FEATURE SWITCHES
// ============================================================================

// 1 = Freenove with SD_MMC support, 0 = standard ESP32-S3 build without SD
#define FEATURE_SD_MMC 0
static bool sdMounted = false;

// 1 = external 39 LED ring/strip on GPIO17, 0 = onboard RGB LED only
#define FEATURE_EXTERNAL_NEOPIXEL_STRIP 0
#define FEATURE_ONBOARD_NEOPIXEL 1

// IMPORTANT: verify for your specific ESP32-S3 board. 48 is common, but not universal.
#ifndef ONBOARD_NEOPIXEL_PIN
#define ONBOARD_NEOPIXEL_PIN 48
#endif

// ============================================================================
//                                INCLUDES
// ============================================================================

// Workaround: some environments define sensor_t and conflict with Adafruit headers.
#define sensor_t adafruit_sensor_t
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#undef sensor_t

#include <Wire.h>
#include <Adafruit_Sensor.h>

#if FEATURE_SD_MMC
#include <SD_MMC.h>
#endif

#include "sd_read_write.h"

#include <ESP32Servo.h>
#include <WiFi.h>
extern "C" {
#include "esp_wifi.h"  // esp_wifi_set_ps
}
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Arduino_ESP32_OTA.h>
#include <HTTPUpdateServer.h>
#include <EEPROM.h>
#include <SPIFFS.h>
#include <math.h>
#include <vector>
#include <Adafruit_NeoPixel.h>

// ============================================================================
//                           LED STRIP CONFIGURATION
// ============================================================================

#if FEATURE_EXTERNAL_NEOPIXEL_STRIP
#define LEDS_COUNT 39
#define LEDS_PIN 17
#else
// Onboard LED: treat as 1 NeoPixel
#define LEDS_COUNT 1
#define LEDS_PIN ONBOARD_NEOPIXEL_PIN
#endif

Adafruit_NeoPixel strip(LEDS_COUNT, LEDS_PIN, NEO_GRB + NEO_KHZ800);

// ============================================================================
//                         GLOBALS / CONFIG / STATE
// ============================================================================

// EEPROM storage: currently used for pressure caching.
#define EEPROM_SIZE 10
#define EEPROM_PRESSURE_ADDR 0

// SPIFFS “soft limit”: if SPIFFS used space exceeds this threshold we attempt offload to SD (if enabled),
// otherwise we disable SPIFFS logging and show warning LEDs.
const size_t MAX_SPIFFS_USED_KB = 700;

bool spiffsLoggingAllowed = false;
bool alreadyWarned = false;

// IMPORTANT FIX (3.9):
// apiSuccess now represents the *result of the last API call attempt* only.
// It is reset to false at the start of updatePressureFromAPI().
bool apiSuccess = false;

bool debugSerial = false;      // Serial debug spam toggle
bool sensorModeFlight = true;  // true = flight dashboard mode, false = visualization mode

// Axis mapping selection (UI-controlled).
// NOTE: mapping 0 and 1 currently produce the same output in this code (kept for UI compatibility).
int axisConfig = 5;

bool sensorsCalibrated = false;
bool sensorsCalibWarned = false;  // reserved / optional warning behavior
bool triggerAbs = true;           // If true, |acc| threshold; else positive-only threshold

String lastTriggeredBy = "NotTriggered";  // persistent last cause
String uiTriggeredBy = "NotTriggered";    // what the UI currently displays

// Trigger enable toggles (UI controlled)
bool enAltDrop = true;
bool enAccX = true;
bool enAccY = false;
bool enAccZ = true;
bool enApogee = true;

// SD card storage info (in MB in your code)
uint64_t totalSpace;
uint64_t usedSpace;

// Pressure / sea-level reference
String PressureSource = "";
float lastLocalPressure = 1013.25;
bool apiPressureUpdated = false;

bool showSensorInitLog = true;
bool bmpFound = false;
bool mpuFound = false;

// ============================================================================
//                                  WIFI
// ============================================================================
const char *ssid = "TDGC-Rocket";
const char *wifiPassword = "Rocket2022!";
const char *apSSID = "RocketAP";
const char *apPassword = "Rocket2022!";

// ============================================================================
//                            OPENWEATHER SETTINGS
// ============================================================================
const char *openWeatherMapApiKey = "xxxxxx";
float currentLatitude = 51.07741431;
float currentLongitude = 5.88510756;
const char *owmEndpoint = "https://api.openweathermap.org/data/3.0/onecall";

// ============================================================================
//                                    TIME
// ============================================================================
#define UTC_OFFSET_IN_SECONDS 3600
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", UTC_OFFSET_IN_SECONDS);
unsigned long lastSuccessfulNTP = 0;
unsigned long lastSyncMillis = 0;

// ============================================================================
//                    TRIGGERS, CALIBRATION, FLIGHT STATE
// ============================================================================

float altitudeDropThreshold = 0.8;
float accXThreshold = 8.0;
float accYThreshold = 8.0;
float accZThreshold = 8.0;

// If true: AND logic among enabled triggers; if false: OR logic among enabled triggers
bool useAndLogic = false;

// Baseline altitude when armed/calibrated
float baselineAltitude = 0;
bool baselineCaptured = false;

// --- Altitude filtering & apogee detection ---
float ALT_EWMA_ALPHA = 0.20f;
float filteredRelAlt = 0.0f;
float prevFilteredRelAlt = 0.0f;
float verticalSpeed = 0.0f;
unsigned long lastAltUpdateMs = 0;

// Apogee detection state
bool apogeeDetected = false;
float apogeeAltitude = 0.0f;
unsigned long apogeeMillis = 0;
int negVsCount = 0;

// Apogee tuning thresholds
float MIN_ALTITUDE_FOR_APOGEE = 5.0f;
float NEG_VS_CONFIRM = -0.20f;
int NEG_COUNT_CONFIRM = 3;

// Launch detection and ground settle behavior
float LAUNCH_ACC_THRESHOLD = 15.0f;
unsigned long groundCandidateSince = 0;
unsigned long GROUND_SETTLE_MS = 2000;

// Flight actuator and states
String parachuteStatus = "unarmed";
String rocketStatus = "On Launchpad";
String parachutePreStatus = "unknown";

// Logging extremes for altitude values
float maxAbsoluteAltitude = -1000000.0;
float minAbsoluteAltitude = 1000000.0;
float maxRelativeAltitude = -1000000.0;
float minRelativeAltitude = 1000000.0;
float maxAltitudeDrop = -1000000.0;
float minAltitudeDrop = 1000000.0;

// IMPORTANT FIX (3.9):
// This value must be updated continuously while ARMED, not only at arming time.
float armedMaxRelativeAltitude = 0;

// Accelerometer offsets measured during calibration
float accXOffset = 0, accYOffset = 0, accZOffset = 0;

// LED colors resolved at runtime via Wheel()
uint32_t redColor, blueColor, purpleColor, greenColor, aquaColor, orangeColor;

// Sensors / servo
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
Servo parachuteservo;
int servoPin = 14;

// Single source of truth for CSV header line (used for both SD and SPIFFS logs)
const char *CSV_HEADER_LINE =
  "Timestamp,BMP Temp,Pressure,Absolute Altitude,Relative Altitude,Altitude Drop,MPU Temp,RawAccX,RawAccY,RawAccZ,AccX_Calib,AccY_Calib,AccZ_Calib,GyroX,GyroY,GyroZ,Parachute Status,Local Pressure,Default Sea-Level Pressure,API Status,Max Abs Altitude,Min Abs Altitude,Max Rel Altitude,Min Rel Altitude,Max Alt Drop,Min Alt Drop,Total Space,Used Space,SPIFFS Total,SPIFFS Used,Latitude,Longitude,AltDropThres,AccXThres,AccYThres,AccZThres,TriggerLogic,AxisConfig,TriggeredBy,EnAltDrop,EnAccX,EnAccY,EnAccZ,Vertical Speed,RocketStatus,ApogeeDetected,ApogeeAltitude,EnApogee";

// Web server / websocket / OTA
WebServer server(80);
WebSocketsServer webSocket(81);
HTTPUpdateServer httpUpdater;

// I2C pins for your ESP32-S3 build
#define SDA_1 42
#define SCL_1 37

// SD_MMC pins (only relevant when FEATURE_SD_MMC == 1)
#define SD_MMC_CMD 38
#define SD_MMC_CLK 39
#define SD_MMC_D0 40

// Gyro accumulators for visualization
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
const float gyroXerror = 0.03;
const float gyroYerror = 0.03;
const float gyroZerror = 0.03;

File uploadFile;

// Visualization timing
unsigned long lastTimeGyro = 0;
unsigned long lastTimeAcc = 0;
unsigned long lastTimeTemperature = 0;
const unsigned long gyroDelay = 10;
const unsigned long accelerometerDelay = 200;
const unsigned long temperatureDelay = 1000;

// ============================================================================
//                             SPIFFS UPLOADER PAGE
// ============================================================================

const char spiffsUploaderHTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <title>SPIFFS Uploader</title>
  <style>
    body { font-family:sans-serif; margin:30px; }
    h2 { color:#333; }
    .msg { margin:10px 0 15px 0; color: green;}
    input[type=file], input[type=text], button { font-size:1em; margin:5px 0; }
  </style>
</head>
<body>
  <div style="margin-bottom:20px;text-align:center;">
    <a href="/files"><button type="button">← Back to File Manager</button></a>
  </div>
  <h2>ESP32 SPIFFS File Uploader</h2>
  <form id="uploadForm" enctype="multipart/form-data" method="POST">
    <label>SPIFFS Folder (start with /):</label><br>
    <input type="text" id="folder" name="folder" value="/" size="30" required /><br>
    <label>Select file(s):</label><br>
    <input type="file" id="files" name="files" multiple required /><br>
    <button type="submit">Upload</button>
  </form>
  <div class="msg" id="result"></div>
  <script>
document.getElementById('uploadForm').onsubmit = async function(e) {
  e.preventDefault();
  var folder = document.getElementById('folder').value;
  var files = document.getElementById('files').files;
  if(!folder.startsWith('/')) { alert('Folder must start with /'); return false; }
  if(files.length === 0) { alert('Select at least one file!'); return false; }
  let results = [];
  for(let i=0; i<files.length; i++) {
    let formData = new FormData();
    formData.append('folder', folder);
    formData.append('file', files[i]);
    try {
      let res = await fetch('/spiffsupload', {method:'POST', body:formData});
      let txt = await res.text();
      results.push(folder + files[i].name + ': ' + txt);
    } catch (err) {
      results.push(folder + files[i].name + ': Error - ' + err);
    }
  }
  document.getElementById('result').innerHTML = results.join('<br>');
  return false;
}
  </script>
</body>
</html>
)rawliteral";

// ============================================================================
//                         AXIS CORRECTION / ORIENTATION
// ============================================================================

// This function maps raw sensor axes into a “rocket reference frame” chosen by axisConfig.
// NOTE: Cases 0 and 1 currently do the same transformation; kept to preserve UI options.
void correctAxes(float rawX, float rawY, float rawZ, float &corrX, float &corrY, float &corrZ) {
  switch (axisConfig) {
    case 0:
      corrX = -rawX;
      corrY = rawY;
      corrZ = rawZ;
      break;
    case 1:
      corrX = -rawX;
      corrY = rawY;
      corrZ = rawZ;
      break;
    case 2:
      corrX = rawY;
      corrY = rawX;
      corrZ = rawZ;
      break;
    case 3:
      corrX = rawY;
      corrY = rawZ;
      corrZ = rawX;
      break;
    case 4:
      corrX = rawZ;
      corrY = rawY;
      corrZ = rawX;
      break;
    case 5:
      corrX = rawZ;
      corrY = rawX;
      corrZ = rawY;
      break;
    default:
      // Conservative fallback
      corrX = rawY;
      corrY = rawZ;
      corrZ = rawX;
      break;
  }
}

// ============================================================================
//                                 LED HELPERS
// ============================================================================

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void initLEDColors() {
  redColor = Wheel(0);
  blueColor = Wheel(170);
  purpleColor = Wheel(200);
  greenColor = Wheel(85);
  aquaColor = Wheel(125);
  orangeColor = Wheel(30);
}

void setAllLEDs(uint32_t color) {
  for (int i = 0; i < LEDS_COUNT; i++) strip.setPixelColor(i, color);
  strip.show();
}

void showLEDColorsSequentially(uint32_t color, int8_t direction = 1, uint16_t rotations = 1) {
  // Onboard LED: emulate “rotation” as pulse
  if (LEDS_COUNT <= 1) {
    for (uint16_t r = 0; r < rotations; r++) {
      strip.setPixelColor(0, color);
      strip.show();
      delay(120);
      strip.setPixelColor(0, 0);
      strip.show();
      delay(120);
    }
    return;
  }

  for (uint16_t r = 0; r < rotations; r++) {
    if (direction >= 0) {
      for (int i = 0; i < LEDS_COUNT; i++) {
        strip.setPixelColor(i, color);
        strip.show();
        delay(40);
      }
    } else {
      for (int i = LEDS_COUNT - 1; i >= 0; i--) {
        strip.setPixelColor(i, color);
        strip.show();
        delay(40);
      }
    }
    setAllLEDs(0);
    delay(100);
  }
}

void blinkColor(uint32_t color, int times, int delayms) {
  for (int t = 0; t < times; t++) {
    setAllLEDs(color);
    delay(delayms);
    setAllLEDs(0);
    delay(delayms);
  }
}

void setWarningPatternLEDs() {
  if (LEDS_COUNT <= 1) {
    strip.setPixelColor(0, redColor);
    strip.show();
    return;
  }
  for (int i = 0; i < LEDS_COUNT; i++) strip.setPixelColor(i, (i % 2 == 0) ? redColor : purpleColor);
  strip.show();
}

void animateWarningPatternLEDs() {
  if (LEDS_COUNT <= 1) {
    for (int k = 0; k < 12; k++) {
      strip.setPixelColor(0, (k % 2 == 0) ? redColor : purpleColor);
      strip.show();
      delay(80);
    }
    strip.setPixelColor(0, redColor);
    strip.show();
    return;
  }

  for (int i = 0; i < LEDS_COUNT; i++) {
    for (int j = 0; j <= i; j++) strip.setPixelColor(j, (j % 2 == 0) ? redColor : purpleColor);
    for (int j = i + 1; j < LEDS_COUNT; j++) strip.setPixelColor(j, 0);
    strip.show();
    delay(30);
  }
  setWarningPatternLEDs();
}

void indicateWiFiStatus(bool connected) {
  blinkColor(connected ? greenColor : orangeColor, 5, 250);
}

// ============================================================================
//                           SENSOR JSON ENDPOINT HELPERS
// ============================================================================

String getGyroReadings() {
  sensors_event_t a, g, temp;
  if (mpuFound) {
    mpu.getEvent(&a, &g, &temp);

    float corrGx, corrGy, corrGz;
    correctAxes(g.gyro.x, g.gyro.y, g.gyro.z, corrGx, corrGy, corrGz);

    // Integrate gyro into a running orientation-like value.
    // NOTE: This is not a true attitude solution; it’s a simple accumulator.
    if (fabs(corrGx) > gyroXerror) gyroX += corrGx / 30.00;
    if (fabs(corrGy) > gyroYerror) gyroY += corrGy / 30.00;
    if (fabs(corrGz) > gyroZerror) gyroZ += corrGz / 30.00;

    StaticJsonDocument<512> doc;
    doc["gyroX"] = gyroX;
    doc["gyroY"] = gyroY;
    doc["gyroZ"] = gyroZ;

    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString;
  }

  StaticJsonDocument<256> doc;
  doc["gyroX"] = "N/A";
  doc["gyroY"] = "N/A";
  doc["gyroZ"] = "N/A";
  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

String getAccReadings() {
  sensors_event_t a, g, temp;
  StaticJsonDocument<512> doc;

  if (mpuFound) {
    mpu.getEvent(&a, &g, &temp);
    float corrAx, corrAy, corrAz;
    correctAxes(a.acceleration.x, a.acceleration.y, a.acceleration.z, corrAx, corrAy, corrAz);
    doc["accX"] = corrAx;
    doc["accY"] = corrAy;
    doc["accZ"] = corrAz;
  } else {
    doc["accX"] = "N/A";
    doc["accY"] = "N/A";
    doc["accZ"] = "N/A";
  }

  String accString;
  serializeJson(doc, accString);
  return accString;
}

String getTemperatureReading() {
  sensors_event_t a, g, temp;
  if (mpuFound) {
    mpu.getEvent(&a, &g, &temp);
    return String(temp.temperature);
  }
  return "N/A";
}

static inline float f_or(float v, float fallback) {
  return isfinite(v) ? v : fallback;  // requires <math.h> which you already include
}

// ============================================================================
//                               TIME / PRESSURE
// ============================================================================

String getTimeStampString() {
  // If we have a “real” NTP epoch cached, we derive time from millis() drift.
  // Otherwise, we fall back to timeClient formatted time (HH:MM:SS only).
  if (lastSuccessfulNTP != 0) {
    unsigned long currentEpoch = lastSuccessfulNTP + ((millis() - lastSyncMillis) / 1000);
    time_t t = (time_t)currentEpoch;
    struct tm *ti = localtime(&t);

    String yearStr = String(ti->tm_year + 1900);
    String monthStr = (ti->tm_mon + 1) < 10 ? "0" + String(ti->tm_mon + 1) : String(ti->tm_mon + 1);
    String dayStr = ti->tm_mday < 10 ? "0" + String(ti->tm_mday) : String(ti->tm_mday);
    String hoursStr = ti->tm_hour < 10 ? "0" + String(ti->tm_hour) : String(ti->tm_hour);
    String minuteStr = ti->tm_min < 10 ? "0" + String(ti->tm_min) : String(ti->tm_min);
    String secondStr = ti->tm_sec < 10 ? "0" + String(ti->tm_sec) : String(ti->tm_sec);

    return yearStr + "-" + monthStr + "-" + dayStr + " " + hoursStr + ":" + minuteStr + ":" + secondStr;
  }

  timeClient.update();
  return String(timeClient.getFormattedTime());
}

float getLocalSeaLevelPressure() {
  return lastLocalPressure;
}

void updatePressureFromAPI() {
  // IMPORTANT FIX (3.9): reset apiSuccess for this attempt.
  apiSuccess = false;

  float localPressure = 1013.25;

  HTTPClient http;
  String url = String(owmEndpoint) + "?lat=" + String(currentLatitude, 6) + "&lon=" + String(currentLongitude, 6) + "&exclude=minutely,hourly,daily,alerts&appid=" + String(openWeatherMapApiKey);

  http.begin(url);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();

    if (debugSerial) {
      Serial.println("API Call successful. Payload:");
      Serial.println(payload);
    }

    StaticJsonDocument<1536> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (!error) {
      if (doc.containsKey("cod")) {
        int apiErrorCode = doc["cod"];
        String apiErrorMessage = doc["message"].as<String>();
        Serial.printf("API Error %d: %s\n", apiErrorCode, apiErrorMessage.c_str());
        PressureSource = "API Error";
      } else {
        localPressure = doc["current"]["pressure"];
        PressureSource = "API OpenWeather";
        EEPROM.put(EEPROM_PRESSURE_ADDR, localPressure);
        EEPROM.commit();
        Serial.print("API success. Pressure updated to: ");
        Serial.println(localPressure);
        apiSuccess = true;
      }
    } else {
      Serial.println("JSON parse error:");
      Serial.println(error.f_str());
      PressureSource = "API JSON Error";
    }
  } else {
    Serial.printf("HTTP Error: %d (%s)\n", httpCode, http.errorToString(httpCode).c_str());
    PressureSource = "HTTP Error";
  }

  http.end();

  float storedPressure = 0;
  EEPROM.get(EEPROM_PRESSURE_ADDR, storedPressure);

  if (debugSerial) {
    Serial.print("EEPROM stored pressure: ");
    Serial.println(storedPressure);
  }

  // If API failed, fallback to EEPROM or default.
  if (!apiSuccess) {
    if (storedPressure > 500.0 && storedPressure < 1100.0) {
      localPressure = storedPressure;
      PressureSource = "EEPROM";
      Serial.print("Using EEPROM Pressure: ");
      Serial.println(localPressure);
    } else {
      localPressure = 1013.25;
      PressureSource = "Default Sea-Level";
      Serial.print("Using Default Sea-Level Pressure: ");
      Serial.println(localPressure);
    }
  }

  lastLocalPressure = localPressure;
  apiPressureUpdated = true;
}

// ============================================================================
//                          SPIFFS SPACE CHECK & OFFLOAD
// ============================================================================

void ensureLogFileHasHeader(fs::FS &fs, const char *path);

void checkSpiffsSpaceAndWarn() {
  size_t spiffsUsedKB = SPIFFS.usedBytes() / 1024;

  if (spiffsUsedKB >= MAX_SPIFFS_USED_KB) {
    bool ok = false;

#if FEATURE_SD_MMC
    // If SD enabled, try to copy current SPIFFS /log.csv to SD and reset SPIFFS log.
    if (sdMounted && SPIFFS.exists("/log.csv")) {
      String ts = getTimeStampString();
      ts.replace(":", "");
      ts.replace(" ", "_");
      String dst = "/SPIFFS_COPY_" + ts + ".csv";

      ok = copyFileFS(SPIFFS, "/log.csv", SD_MMC, dst.c_str());
      if (ok) {
        SPIFFS.remove("/log.csv");
        ensureLogFileHasHeader(SPIFFS, "/log.csv");
      }
    }
#endif

    // If we could not offload (or SD feature disabled), stop SPIFFS logging and show warning.
    if (!ok) {
      spiffsLoggingAllowed = false;
      alreadyWarned = true;
      setWarningPatternLEDs();
      return;
    }

    // Offload succeeded, resume SPIFFS logging.
    spiffsLoggingAllowed = true;
    alreadyWarned = false;
    return;
  }

  spiffsLoggingAllowed = true;
  alreadyWarned = false;
}

// ============================================================================
//                          PARACHUTE / FLIGHT ACTIONS
// ============================================================================

void calibrateSensors();
void parachuteUnarmed();

void parachuteRelease() {
  Serial.println("Trigger condition met! Releasing parachute...");

  // Servo actuation: repeated write() is not strictly necessary but kept from your original.
  for (int i = 0; i < 2; i++) parachuteservo.write(0);

  parachuteStatus = "released";

  blinkColor(greenColor, 5, 100);
  showLEDColorsSequentially(greenColor, -1, 2);
  setAllLEDs(greenColor);
}

void parachuteArmed() {
  uiTriggeredBy = "NotTriggered";
  lastTriggeredBy = "NotTriggered";
  rocketStatus = "On Launchpad";

  // Always calibrate before arming.
  calibrateSensors();

  // If calibration failed (e.g., sensors missing), do not arm.
  if (!sensorsCalibrated) {
    Serial.println("Refusing to ARM: sensors are not calibrated or missing.");
    blinkColor(orangeColor, 4, 250);
    parachuteStatus = "unarmed";
    setAllLEDs(blueColor);
    return;
  }

  Serial.println("Arming parachute...");

  // Baseline altitude capture requires BMP.
  if (!bmpFound) {
    Serial.println("Refusing to ARM: BMP280 not found (altitude unavailable).");
    blinkColor(orangeColor, 4, 250);
    parachuteStatus = "unarmed";
    setAllLEDs(blueColor);
    return;
  }

  if (!baselineCaptured) {
    baselineAltitude = bmp.readAltitude(getLocalSeaLevelPressure());
    baselineCaptured = true;
    Serial.print("Baseline altitude captured: ");
    Serial.print(baselineAltitude);
    Serial.println(" m");
  }

  // Initialize “armed max” using current relative altitude at arming time.
  float currentRel = bmp.readAltitude(getLocalSeaLevelPressure()) - baselineAltitude;
  armedMaxRelativeAltitude = currentRel;

  // Reset extremes for the upcoming flight
  maxRelativeAltitude = 0;
  minRelativeAltitude = 1000000.0;
  maxAltitudeDrop = 0;
  minAltitudeDrop = 1000000.0;

  parachuteservo.write(180);
  parachuteStatus = "armed";
  rocketStatus = "On Launchpad";

  apogeeDetected = false;
  apogeeAltitude = 0.0f;
  negVsCount = 0;

  if (alreadyWarned) setWarningPatternLEDs();
  else {
    showLEDColorsSequentially(redColor, 1, 3);
    setAllLEDs(redColor);
  }
}

void calibrateSensors() {
  rocketStatus = "Calibrating";
  uiTriggeredBy = "NotTriggered";
  lastTriggeredBy = "NotTriggered";

  Serial.println("Calibrating sensors...");

  parachutePreStatus = parachuteStatus;
  parachuteStatus = "calibrating";

  // Default assumption until proven otherwise
  sensorsCalibrated = false;

  // Calibration depends on sensors. If missing, fail fast.
  if (!mpuFound) {
    Serial.println("Calibration failed: MPU6050 not found.");
  }
  if (!bmpFound) {
    Serial.println("Calibration warning: BMP280 not found (altitude baseline not available).");
  }

  // BMP baseline capture (only if available)
  if (bmpFound) {
    baselineAltitude = bmp.readAltitude(getLocalSeaLevelPressure());
    baselineCaptured = true;
  } else {
    // Without BMP, we cannot define meaningful baseline altitude.
    baselineCaptured = false;
    baselineAltitude = 0.0f;
  }

  // MPU offsets (only if available)
  if (mpuFound) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float corrAx, corrAy, corrAz;
    correctAxes(a.acceleration.x, a.acceleration.y, a.acceleration.z, corrAx, corrAy, corrAz);

    accXOffset = corrAx;
    accYOffset = corrAy;
    accZOffset = corrAz;
  } else {
    accXOffset = 0;
    accYOffset = 0;
    accZOffset = 0;
  }

  // Reset altitude extremes
  maxRelativeAltitude = 0;
  minRelativeAltitude = 1000000.0;
  maxAltitudeDrop = 0;
  minAltitudeDrop = 1000000.0;

  // “Calibrated” means at least MPU present (for launch detection / accel triggers).
  // If you want BMP-required calibration, change this condition accordingly.
  sensorsCalibrated = mpuFound;

  Serial.print("Calibration complete. BMP baseline captured: ");
  Serial.println(baselineCaptured ? "YES" : "NO");

  showLEDColorsSequentially(orangeColor, 1, 1);
  showLEDColorsSequentially(orangeColor, -1, 1);

  parachuteStatus = parachutePreStatus;

  if (!alreadyWarned) {
    if (parachuteStatus == "armed") setAllLEDs(redColor);
    else if (parachuteStatus == "unarmed") setAllLEDs(blueColor);
    else if (parachuteStatus == "released") setAllLEDs(greenColor);
  } else {
    setWarningPatternLEDs();
  }

  // Always return to Flight mode after calibration
  sensorModeFlight = true;

  Serial.print("Reset sensor mode to Flight: ");
  Serial.println(sensorModeFlight ? "true" : "false");
}

// ============================================================================
//                             CSV LOGGING HELPERS
// ============================================================================

void appendCsv(fs::FS &fs, const char *path, const String &line) {
  // Ensure header exists for new or empty file.
  bool needHeader = !fs.exists(path);

  if (!needHeader) {
    File fr = fs.open(path, FILE_READ);
    if (!fr) needHeader = true;
    else {
      if (fr.size() == 0) needHeader = true;
      fr.close();
    }
  }

  if (needHeader) {
    File fw = fs.open(path, FILE_APPEND);
    if (fw) {
      fw.println(CSV_HEADER_LINE);
      fw.close();
    }
  }

  File f = fs.open(path, FILE_APPEND);
  if (f) {
    f.print(line);
    f.close();
  }
}

void ensureLogFileHasHeader(fs::FS &fs, const char *path) {
  if (!fs.exists(path)) {
    File f = fs.open(path, FILE_WRITE);
    if (f) {
      f.println(CSV_HEADER_LINE);
      f.close();
    }
    return;
  }

  File f = fs.open(path, FILE_READ);
  if (f && f.size() == 0) {
    f.close();
    File fw = fs.open(path, FILE_WRITE);
    if (fw) {
      fw.println(CSV_HEADER_LINE);
      fw.close();
    }
  } else if (f) {
    f.close();
  }
}

// ============================================================================
//                              WEBSOCKET HANDLER
// ============================================================================

void webSocketEvent(byte num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("Client " + String(num) + " disconnected");
      break;

    case WStype_CONNECTED:
      Serial.println("Client " + String(num) + " connected");
      break;

    case WStype_TEXT:
      {
        StaticJsonDocument<2048> doc;
        DeserializationError error = deserializeJson(doc, payload);
        if (error) {
          Serial.print(F("WS JSON parse failed: "));
          Serial.println(error.f_str());
          return;
        }

        // Threshold updates
        if (doc.containsKey("newThreshold")) { altitudeDropThreshold = doc["newThreshold"]; }
        if (doc.containsKey("newAccX")) { accXThreshold = doc["newAccX"]; }
        if (doc.containsKey("newAccY")) { accYThreshold = doc["newAccY"]; }
        if (doc.containsKey("newAccZ")) { accZThreshold = doc["newAccZ"]; }

        // Trigger enable toggles
        if (doc.containsKey("enAltDrop")) enAltDrop = doc["enAltDrop"];
        if (doc.containsKey("enAccX")) enAccX = doc["enAccX"];
        if (doc.containsKey("enAccY")) enAccY = doc["enAccY"];
        if (doc.containsKey("enAccZ")) enAccZ = doc["enAccZ"];
        if (doc.containsKey("enApogee")) enApogee = doc["enApogee"];

        // Advanced apogee and launch tuning
        if (doc.containsKey("minApogeeAlt")) MIN_ALTITUDE_FOR_APOGEE = doc["minApogeeAlt"].as<float>();
        if (doc.containsKey("negVsConfirm")) NEG_VS_CONFIRM = doc["negVsConfirm"].as<float>();
        if (doc.containsKey("negCountConfirm")) NEG_COUNT_CONFIRM = doc["negCountConfirm"].as<int>();
        if (doc.containsKey("launchAccThreshold")) LAUNCH_ACC_THRESHOLD = doc["launchAccThreshold"].as<float>();
        if (doc.containsKey("groundSettleMs")) GROUND_SETTLE_MS = doc["groundSettleMs"].as<unsigned long>();

        // Immediately ack back to UI so fields stay in sync
        {
          StaticJsonDocument<512> ack;
          ack["LaunchAccThreshold"] = LAUNCH_ACC_THRESHOLD;
          ack["MinApogeeAlt"] = MIN_ALTITUDE_FOR_APOGEE;
          ack["NegVsConfirm"] = NEG_VS_CONFIRM;
          ack["NegCountConfirm"] = NEG_COUNT_CONFIRM;
          ack["GroundSettleMs"] = GROUND_SETTLE_MS;

          ack["EnAltDrop"] = enAltDrop;
          ack["EnAccX"] = enAccX;
          ack["EnAccY"] = enAccY;
          ack["EnAccZ"] = enAccZ;

          ack["AltDropThreshold"] = altitudeDropThreshold;
          ack["AccXThreshold"] = accXThreshold;
          ack["AccYThreshold"] = accYThreshold;
          ack["AccZThreshold"] = accZThreshold;

          String s;
          serializeJson(ack, s);
          webSocket.broadcastTXT(s);
        }

        if (doc.containsKey("newTriggerLogic")) {
          String newLogic = doc["newTriggerLogic"];
          useAndLogic = (newLogic == "AND");
        }

        if (doc.containsKey("triggerAbs")) triggerAbs = doc["triggerAbs"];

        if (doc.containsKey("calibrateSensors")) calibrateSensors();

        if (doc.containsKey("sensorModeFlight")) sensorModeFlight = doc["sensorModeFlight"];
        if (doc.containsKey("axisConfig")) axisConfig = doc["axisConfig"];

        // Parachute command channel
        if (doc.containsKey("parachute")) {
          const char *command = doc["parachute"];
          Serial.println("Received parachute command from client " + String(num));
          if (String(command) == "Armed") parachuteArmed();
          else if (String(command) == "Released") parachuteRelease();
          else if (String(command) == "Unarmed") parachuteUnarmed();
        }

        // Location updates -> refresh API pressure and broadcast back
        if (doc.containsKey("latitude") && doc.containsKey("longitude")) {
          currentLatitude = doc["latitude"];
          currentLongitude = doc["longitude"];

          apiPressureUpdated = false;
          updatePressureFromAPI();

          StaticJsonDocument<128> out;
          out["Latitude"] = currentLatitude;
          out["Longitude"] = currentLongitude;
          out["LocalPressure"] = lastLocalPressure;
          out["PressureSource"] = PressureSource;

          String s;
          serializeJson(out, s);
          webSocket.broadcastTXT(s);
        }

        break;
      }
  }
}

void parachuteUnarmed() {
  parachuteStatus = "unarmed";
  uiTriggeredBy = "NotTriggered";
  lastTriggeredBy = "NotTriggered";
  baselineCaptured = false;

  if (!alreadyWarned) setAllLEDs(blueColor);
  else setWarningPatternLEDs();

  Serial.println("Parachute is now UNARMED. Logging to SPIFFS will stop unless armed/released.");
}

// ============================================================================
//                          FILE MANAGEMENT ENDPOINTS
// ============================================================================

String listFilesJSON(fs::FS &fs, const char *path = "/") {
  String json = "[";
  File root = fs.open(path);
  if (!root || !root.isDirectory()) return "[]";

  File file = root.openNextFile();
  bool first = true;
  while (file) {
    if (!first) json += ",";
    json += "{";
    json += "\"name\":\"" + String(file.name()) + "\"";
    json += ",\"size\":" + String(file.size());
    json += "}";
    first = false;
    file = root.openNextFile();
  }

  json += "]";
  return json;
}

void handleListFilesJson() {
  String json = "{";

#if FEATURE_SD_MMC
  if (sdMounted) json += "\"sd\":" + listFilesJSON(SD_MMC);
  else json += "\"sd\":[]";
#else
  json += "\"sd\":[]";
#endif

  json += ",\"spiffs\":" + listFilesJSON(SPIFFS);
  json += "}";
  server.send(200, "application/json", json);
}

void handleSPIFFSFileDelete() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing file name");
    return;
  }
  String fname = server.arg("name");
  if (!SPIFFS.exists(fname)) {
    server.send(404, "text/plain", "File not found in SPIFFS");
    return;
  }
  if (SPIFFS.remove(fname)) server.send(200, "text/plain", "Deleted from SPIFFS");
  else server.send(500, "text/plain", "Failed to delete from SPIFFS");
}

void handleSDFileDelete() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing file name");
    return;
  }
  String fname = server.arg("name");

#if FEATURE_SD_MMC
  if (!sdMounted) {
    server.send(503, "text/plain", "SD not mounted");
    return;
  }
  if (!SD_MMC.exists(fname)) {
    server.send(404, "text/plain", "File not found on SD");
    return;
  }
  if (SD_MMC.remove(fname)) server.send(200, "text/plain", "Deleted from SD");
  else server.send(500, "text/plain", "Failed to delete from SD");
#else
  server.send(501, "text/plain", "SD feature disabled");
#endif
}

void handleCombinedFileDownload() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing file name");
    return;
  }
  String fname = server.arg("name");
  File file;

#if FEATURE_SD_MMC
  if (sdMounted) file = SD_MMC.open(fname, FILE_READ);
#endif

  if (!file) file = SPIFFS.open(fname, FILE_READ);
  if (!file) {
    server.send(404, "text/plain", "File not found on SD or SPIFFS");
    return;
  }

  String outFilename = fname;
  int slash = outFilename.lastIndexOf('/');
  if (slash != -1) outFilename = outFilename.substring(slash + 1);

  server.sendHeader("Content-Disposition", "attachment; filename=\"" + outFilename + "\"");
  server.streamFile(file, "application/octet-stream");
  file.close();
}

void handleSDFileDownload() {
#if !FEATURE_SD_MMC
  server.send(501, "text/plain", "SD feature disabled");
  return;
#else
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing file name");
    return;
  }
  if (!sdMounted) {
    server.send(503, "text/plain", "SD not mounted");
    return;
  }

  String fname = server.arg("name");
  File f = SD_MMC.open(fname, FILE_READ);
  if (!f) {
    server.send(404, "text/plain", "File not found on SD");
    return;
  }

  String out = fname.substring(fname.lastIndexOf('/') + 1);
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + out + "\"");
  server.streamFile(f, "application/octet-stream");
  f.close();
#endif
}

void handleSPIFFSFileDownload() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing file name");
    return;
  }
  String fname = server.arg("name");
  File f = SPIFFS.open(fname, FILE_READ);
  if (!f) {
    server.send(404, "text/plain", "File not found on SPIFFS");
    return;
  }

  String out = fname.substring(fname.lastIndexOf('/') + 1);
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + out + "\"");
  server.streamFile(f, "application/octet-stream");
  f.close();
}

void handleFileUpload() {
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String fname = upload.filename;
    if (!fname.startsWith("/")) fname = "/" + fname;
    uploadFile = SPIFFS.open(fname, FILE_WRITE);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) uploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile) uploadFile.close();
  }
}

void handleFileUploadSD() {
#if !FEATURE_SD_MMC
  // If SD is compiled out, we do nothing here; the HTTP handler returns 501 in setup().
  return;
#else
  if (!sdMounted) return;

  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String fname = upload.filename;
    if (!fname.startsWith("/")) fname = "/" + fname;
    uploadFile = SD_MMC.open(fname, FILE_WRITE);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) uploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile) uploadFile.close();
  }
#endif
}

void handleCombinedFileDelete() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing file name");
    return;
  }
  String fname = server.arg("name");
  bool deleted = false;

#if FEATURE_SD_MMC
  if (sdMounted && SD_MMC.exists(fname)) deleted |= SD_MMC.remove(fname);
#endif
  if (SPIFFS.exists(fname)) deleted |= SPIFFS.remove(fname);

  if (deleted) server.send(200, "text/plain", "Deleted from SD or SPIFFS");
  else server.send(404, "text/plain", "File not found on SD or SPIFFS");
}

// ---- SD "Delete All" helpers ----
struct DelStats {
  uint32_t removed_files = 0;
  uint32_t removed_dirs = 0;
  uint32_t failed = 0;
};

static bool isProtectedEntry(const String &path) {
  if (path.equalsIgnoreCase("/System Volume Information")) return true;
  return false;
}

static bool buildChildPath(const char *parent, const char *name, String &out) {
  if (!parent || !name) return false;
  if (name[0] == '/') {
    out = name;
    return true;
  }
  out = parent;
  if (!out.endsWith("/")) out += "/";
  out += name;
  return true;
}

bool deleteRecursivelyWithStats(fs::FS &fs, const char *dirPath, DelStats &stats) {
  File root = fs.open(dirPath);
  if (!root || !root.isDirectory()) return false;

  File f = root.openNextFile();
  while (f) {
    String childPath;
    const char *entryName = f.name();
    if (!buildChildPath(dirPath, entryName, childPath)) {
      stats.failed++;
      f.close();
      f = root.openNextFile();
      continue;
    }

    if (isProtectedEntry(childPath)) {
      f.close();
      f = root.openNextFile();
      continue;
    }

    if (f.isDirectory()) {
      f.close();
      deleteRecursivelyWithStats(fs, childPath.c_str(), stats);
      if (fs.rmdir(childPath.c_str())) stats.removed_dirs++;
      else stats.failed++;
    } else {
      f.close();
      if (fs.remove(childPath.c_str())) stats.removed_files++;
      else stats.failed++;
    }

    f = root.openNextFile();
  }

  root.close();
  return true;
}

void handleDeleteAllSD() {
  blinkColor(redColor, 3, 150);

#if !FEATURE_SD_MMC
  server.send(501, "text/plain", "SD feature disabled");
  return;
#else
  if (!sdMounted) {
    server.send(503, "text/plain", "SD not mounted");
    return;
  }

  DelStats stats;
  deleteRecursivelyWithStats(SD_MMC, "/", stats);

  File file = SD_MMC.open("/log.csv", FILE_WRITE);
  if (file) {
    file.println(CSV_HEADER_LINE);
    file.close();
  } else stats.failed++;

  String json = "{";
  json += "\"removed_files\":" + String(stats.removed_files) + ",";
  json += "\"removed_dirs\":" + String(stats.removed_dirs) + ",";
  json += "\"failed\":" + String(stats.failed) + "}";

  server.send(stats.failed == 0 ? 200 : 207, "application/json", json);
#endif
}

void handleSpiffsUpload() {
  HTTPUpload &upload = server.upload();
  static String targetFolder;
  static File uploadFileLocal;

  if (upload.status == UPLOAD_FILE_START) {
    targetFolder = server.arg("folder");
    if (targetFolder.length() == 0) targetFolder = "/";
    if (!targetFolder.startsWith("/")) targetFolder = "/" + targetFolder;
    if (!targetFolder.endsWith("/")) targetFolder += "/";

    String fname = upload.filename;
    String fullPath = targetFolder + fname;

    // NOTE: SPIFFS “folders” are emulated; mkdir may succeed/fail depending on core version.
    if (!SPIFFS.exists(targetFolder.c_str())) SPIFFS.mkdir(targetFolder.c_str());

    uploadFileLocal = SPIFFS.open(fullPath, FILE_WRITE);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFileLocal) uploadFileLocal.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFileLocal) uploadFileLocal.close();
  }
}

// ============================================================================
//                                    SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting setup..."));

  rocketStatus = "Booting";
  Serial.println(rocketStatus);

  // SPIFFS mount (format on fail)
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
  } else {
    Serial.println("SPIFFS mounted successfully");
  }
  ensureLogFileHasHeader(SPIFFS, "/log.csv");

  // EEPROM init and read cached pressure
  EEPROM.begin(EEPROM_SIZE);
  float storedPressure;
  EEPROM.get(EEPROM_PRESSURE_ADDR, storedPressure);

  // Disable WiFi power saving for more stable websocket usage
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Connect WiFi station, else fallback to AP
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, wifiPassword);

  Serial.print("Connecting to WiFi");
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed. Starting Access Point...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apSSID, apPassword);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
  }

  // Pressure update only if connected to internet
  if (WiFi.status() == WL_CONNECTED) updatePressureFromAPI();

  // Determine baseline pressure source
  if (apiSuccess == true) {
    PressureSource = "API OpenWeather";
  } else if (storedPressure > 500.0 && storedPressure < 1100.0) {
    lastLocalPressure = storedPressure;
    PressureSource = "EEPROM Memory";
  } else {
    lastLocalPressure = 1013.25;
    PressureSource = "Default Sea-Level Pressure";
  }

  // NTP init
  timeClient.begin();
  timeClient.update();
  unsigned long currentEpoch = timeClient.getEpochTime();
  if (currentEpoch > 100000) {
    lastSuccessfulNTP = currentEpoch;
    lastSyncMillis = millis();
    Serial.print("NTP Time set to: ");
    Serial.println(getTimeStampString());
  } else {
    Serial.println("Failed to get NTP time.");
  }

  // -----------------------
  // SD card init (if enabled)
  // -----------------------
#if FEATURE_SD_MMC
  Serial.print("Initializing SD card...");
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);

  sdMounted = SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5);
  if (!sdMounted) {
    Serial.println("Card Mount Failed");
  } else {
    Serial.println("SD Card initialized.");
    ensureLogFileHasHeader(SD_MMC, "/log.csv");

    // Rotate old log.csv to timestamped file
    String oldLogFile = "/log.csv";
    String timestamp = getTimeStampString();
    timestamp.replace(":", "");
    timestamp.replace(" ", "_");
    String newLogFile = "/" + timestamp + ".csv";

    if (SD_MMC.exists(newLogFile.c_str())) SD_MMC.remove(newLogFile.c_str());

    if (SD_MMC.exists(oldLogFile.c_str())) {
      if (SD_MMC.rename(oldLogFile.c_str(), newLogFile.c_str())) {
        Serial.println("Previous log file renamed to " + newLogFile);
        ensureLogFileHasHeader(SD_MMC, "/log.csv");
      } else {
        Serial.println("Failed to rename log file " + oldLogFile);
      }
    } else {
      Serial.println("No previous log file found.");
    }
  }

  if (sdMounted) {
    totalSpace = SD_MMC.totalBytes() / (1024 * 1024);
    usedSpace = SD_MMC.usedBytes() / (1024 * 1024);
  } else {
    totalSpace = 0;
    usedSpace = 0;
  }
#else
  sdMounted = false;
  totalSpace = 0;
  usedSpace = 0;
  Serial.println("SD_MMC disabled for this build.");
#endif

  // I2C bus init
  Wire.begin(SDA_1, SCL_1);

  // Sensor init
  if (mpu.begin(0x68)) {
    mpuFound = true;
    if (showSensorInitLog) Serial.println("MPU6050 sensor found.");
  } else {
    mpuFound = false;
    if (showSensorInitLog) Serial.println("Could not find MPU6050 sensor. Check wiring!");
  }

  if (bmp.begin(0x76)) {
    bmpFound = true;
    if (showSensorInitLog) Serial.println("BMP280 sensor found.");
  } else {
    bmpFound = false;
    if (showSensorInitLog) Serial.println("Could not find BMP280 sensor. Check wiring!");
  }

  if (mpuFound) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  if (bmpFound) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  }

  // OTA
  httpUpdater.setup(&server);

  // Servo timers and attach
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  parachuteservo.setPeriodHertz(50);
  parachuteservo.attach(servoPin, 1000, 2000);
  delay(1000);

  // LED init
  strip.begin();
  strip.setBrightness(20);
  strip.show();
  initLEDColors();

  showLEDColorsSequentially(purpleColor, 1, 3);

  Serial.println("Setup complete.");
  indicateWiFiStatus(WiFi.status() == WL_CONNECTED);
  setAllLEDs(blueColor);

  // -----------------------
  // Routes
  // -----------------------
  server.on("/", HTTP_GET, []() {
    sensorModeFlight = true;
    setAllLEDs(blueColor);
    server.sendHeader("Location", "/index.html");
    server.send(302, "text/plain", "Redirecting...");
  });

  server.on("/visualization", HTTP_GET, []() {
    sensorModeFlight = false;
    setAllLEDs(purpleColor);
    server.sendHeader("Location", "/visualization/index.html");
    server.send(302, "text/plain", "Redirecting...");
  });

  server.on("/files", HTTP_GET, []() {
    setAllLEDs(orangeColor);
    server.sendHeader("Location", "/files/index.html");
    server.send(302, "text/plain", "Redirecting...");
  });

  server.serveStatic("/visualization/", SPIFFS, "/visualization/");
  server.serveStatic("/files/", SPIFFS, "/files/");

  server.on("/deleteFile", HTTP_GET, []() {
    blinkColor(redColor, 2, 250);
    handleCombinedFileDelete();
  });
  server.on("/deleteFileSPIFFS", HTTP_GET, []() {
    blinkColor(redColor, 2, 250);
    handleSPIFFSFileDelete();
  });
  server.on("/deleteFileSD", HTTP_GET, []() {
    blinkColor(redColor, 2, 250);
    handleSDFileDelete();
  });

  server.on("/downloadFile", HTTP_GET, []() {
    blinkColor(purpleColor, 2, 250);
    handleCombinedFileDownload();
  });
  server.on("/downloadFileSD", HTTP_GET, []() {
    blinkColor(purpleColor, 2, 250);
    handleSDFileDownload();
  });
  server.on("/downloadFileSPIFFS", HTTP_GET, []() {
    blinkColor(purpleColor, 2, 250);
    handleSPIFFSFileDownload();
  });

  // Upload to SPIFFS
  server.on(
    "/upload", HTTP_POST,
    []() {
      blinkColor(greenColor, 2, 250);
      server.send(200, "text/plain", "SPIFFS Upload Successful");
    },
    handleFileUpload);

  // Upload to SD:
  // IMPORTANT FIX (3.9): if SD is disabled, respond 501 instead of pretending success.
  server.on(
    "/uploadsd", HTTP_POST,
    []() {
#if FEATURE_SD_MMC
      blinkColor(greenColor, 2, 250);
      server.send(200, "text/plain", "SD Card Upload Successful");
#else
      server.send(501, "text/plain", "SD feature disabled");
#endif
    },
    handleFileUploadSD);

  server.on("/listfiles", HTTP_GET, handleListFilesJson);

  server.on("/gyro", HTTP_GET, []() {
    server.send(200, "application/json", getGyroReadings());
  });
  server.on("/acc", HTTP_GET, []() {
    server.send(200, "application/json", getAccReadings());
  });
  server.on("/temp", HTTP_GET, []() {
    server.send(200, "text/plain", getTemperatureReading());
  });

  server.on("/reset", HTTP_GET, []() {
    gyroX = gyroY = gyroZ = 0;
    server.send(200, "text/plain", "All gyro values reset");
  });
  server.on("/resetX", HTTP_GET, []() {
    gyroX = 0;
    server.send(200, "text/plain", "Gyro X reset");
  });
  server.on("/resetY", HTTP_GET, []() {
    gyroY = 0;
    server.send(200, "text/plain", "Gyro Y reset");
  });
  server.on("/resetZ", HTTP_GET, []() {
    gyroZ = 0;
    server.send(200, "text/plain", "Gyro Z reset");
  });

  server.serveStatic("/index.html", SPIFFS, "/index.html");
  server.serveStatic("/style.css", SPIFFS, "/style.css");
  server.serveStatic("/script.js", SPIFFS, "/script.js");

  server.on("/spiffsupload", HTTP_GET, []() {
    server.send_P(200, "text/html", spiffsUploaderHTML);
  });

  server.on("/deleteAllSD", HTTP_GET, []() {
    handleDeleteAllSD();
  });
  server.on("/deleteAllSD", HTTP_POST, []() {
    handleDeleteAllSD();
  });

  server.on(
    "/spiffsupload", HTTP_POST,
    []() {
      server.send(200, "text/plain", "Upload complete! Refresh to upload more files.");
    },
    handleSpiffsUpload);

  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

// ============================================================================
//                                     LOOP
// ============================================================================

void loop() {

server.handleClient();
webSocket.loop();

// Pressure refresh control:
// - When connected: refresh exactly once after reconnect or when apiPressureUpdated false.
// - When disconnected: force apiPressureUpdated false so it will refresh on next reconnect.
if (WiFi.status() == WL_CONNECTED) {
  if (!apiPressureUpdated) updatePressureFromAPI();
} else {
  apiPressureUpdated = false;
}

// Update SD stats
#if FEATURE_SD_MMC
if (sdMounted) {
  totalSpace = SD_MMC.totalBytes() / (1024 * 1024);
  usedSpace = SD_MMC.usedBytes() / (1024 * 1024);
} else {
  totalSpace = 0;
  usedSpace = 0;
}
#else
totalSpace = 0;
usedSpace = 0;
#endif

size_t spiffsTotal = SPIFFS.totalBytes() / 1024;
size_t spiffsUsed = SPIFFS.usedBytes() / 1024;

// --------------------------------------------------------------------------
// Read sensors with guards
// --------------------------------------------------------------------------
float bmpTemp = NAN, pressure = NAN, absoluteAltitude = NAN;
if (bmpFound) {
  bmpTemp = bmp.readTemperature();
  pressure = bmp.readPressure() / 100.0F;
  absoluteAltitude = bmp.readAltitude(getLocalSeaLevelPressure());
}

sensors_event_t a, g, temp;
memset(&a, 0, sizeof(a));
memset(&g, 0, sizeof(g));
memset(&temp, 0, sizeof(temp));

if (mpuFound) {
  mpu.getEvent(&a, &g, &temp);
}

// Axis correction and calibration offsets:
// If MPU missing, a.* is zeroed; offsets are also zero -> relAcc is 0.
float corrAx, corrAy, corrAz;
correctAxes(a.acceleration.x, a.acceleration.y, a.acceleration.z, corrAx, corrAy, corrAz);

float relAccX = corrAx - accXOffset;
float relAccY = corrAy - accYOffset;
float relAccZ = corrAz - accZOffset;

// Raw accelerometer readings (uncorrected)
float rawAccX = a.acceleration.x;
float rawAccY = a.acceleration.y;
float rawAccZ = a.acceleration.z;

// --------------------------------------------------------------------------
// Altitude logic with guards to prevent NaN propagation
// --------------------------------------------------------------------------
float relativeAltitude = 0.0f;

// If parachute is unarmed OR no BMP OR no baseline captured, treat relative altitude as 0
// to prevent NaN from flowing into filters and triggers.
if (parachuteStatus != "unarmed" && bmpFound && baselineCaptured && !isnan(absoluteAltitude)) {
  relativeAltitude = absoluteAltitude - baselineAltitude;
} else {
  relativeAltitude = 0.0f;
}

// Filtered altitude & vertical speed:
// Only meaningful if altitude is valid; otherwise keep stable at 0.
unsigned long _nowAltMs = millis();

if (lastAltUpdateMs == 0) {
  filteredRelAlt = relativeAltitude;
  prevFilteredRelAlt = relativeAltitude;
  lastAltUpdateMs = _nowAltMs;
}

float _dt = (_nowAltMs - lastAltUpdateMs) / 1000.0f;
if (_dt <= 0.0f) _dt = 0.001f;

filteredRelAlt = ALT_EWMA_ALPHA * relativeAltitude + (1.0f - ALT_EWMA_ALPHA) * filteredRelAlt;
verticalSpeed = (filteredRelAlt - prevFilteredRelAlt) / _dt;

prevFilteredRelAlt = filteredRelAlt;
lastAltUpdateMs = _nowAltMs;

// Update altitude extremes only when BMP is valid
if (bmpFound && !isnan(absoluteAltitude)) {
  if (absoluteAltitude > maxAbsoluteAltitude) maxAbsoluteAltitude = absoluteAltitude;
  if (absoluteAltitude < minAbsoluteAltitude) minAbsoluteAltitude = absoluteAltitude;
}

if (relativeAltitude > maxRelativeAltitude) maxRelativeAltitude = relativeAltitude;
if (relativeAltitude < minRelativeAltitude) minRelativeAltitude = relativeAltitude;

// IMPORTANT FIX (3.9): while ARMED, update armedMaxRelativeAltitude continuously.
if (parachuteStatus == "armed") {
  if (relativeAltitude > armedMaxRelativeAltitude) armedMaxRelativeAltitude = relativeAltitude;
}

// Altitude drop:
// - While armed: drop from the highest altitude reached since arming (armedMaxRelativeAltitude).
// - Otherwise: drop from global maximum seen.
float altitudeDrop = (parachuteStatus == "armed")
                       ? (armedMaxRelativeAltitude - relativeAltitude)
                       : (maxRelativeAltitude - relativeAltitude);

if (altitudeDrop > maxAltitudeDrop) maxAltitudeDrop = altitudeDrop;
if (altitudeDrop < minAltitudeDrop) minAltitudeDrop = altitudeDrop;

// --------------------------------------------------------------------------
// Rocket status & apogee detection
// --------------------------------------------------------------------------

// Launch detection depends on MPU; if MPU missing, relAccZ will be ~0 and never triggers.
if (rocketStatus == "On Launchpad" && relAccZ > LAUNCH_ACC_THRESHOLD) {
  rocketStatus = "Launched";
  apogeeDetected = false;
  apogeeAltitude = 0.0f;
  negVsCount = 0;
}

// Apogee detection requires altitude validity.
if (bmpFound && !apogeeDetected && filteredRelAlt > MIN_ALTITUDE_FOR_APOGEE) {
  if (verticalSpeed < NEG_VS_CONFIRM) {
    negVsCount++;
    if (negVsCount >= NEG_COUNT_CONFIRM) {
      apogeeDetected = true;
      apogeeAltitude = filteredRelAlt;
      apogeeMillis = millis();
      rocketStatus = "Max Apogee";
    }
  } else {
    if (negVsCount > 0) negVsCount--;
  }
}

// Ground detection only allowed after we have been in flight.
bool wasInFlight = (rocketStatus == "Launched" || rocketStatus == "Max Apogee");
if (wasInFlight && fabs(filteredRelAlt) < 1.0f && fabs(verticalSpeed) < 0.05f) {
  if (groundCandidateSince == 0) groundCandidateSince = millis();
  else if (millis() - groundCandidateSince > GROUND_SETTLE_MS) rocketStatus = "On Ground";
} else {
  groundCandidateSince = 0;
}

// --------------------------------------------------------------------------
// Trigger evaluation (only when ARMED)
// --------------------------------------------------------------------------
uiTriggeredBy = lastTriggeredBy;

if (parachuteStatus == "armed") {
  uiTriggeredBy = "NotTriggered";

  // Suppress altitude trigger if BMP missing (safety behavior).
  const bool altitudeValidForTriggers = bmpFound && baselineCaptured;

  const bool triggerAlt = (enAltDrop && altitudeValidForTriggers) ? (altitudeDrop >= altitudeDropThreshold) : false;
  const bool triggerAccX = enAccX ? (triggerAbs ? (fabs(relAccX) >= accXThreshold) : (relAccX >= accXThreshold)) : false;
  const bool triggerAccY = enAccY ? (triggerAbs ? (fabs(relAccY) >= accYThreshold) : (relAccY >= accYThreshold)) : false;
  const bool triggerAccZ = enAccZ ? (triggerAbs ? (fabs(relAccZ) >= accZThreshold) : (relAccZ >= accZThreshold)) : false;
  const bool triggerApogee = enApogee ? apogeeDetected : false;

  const bool enabled[5] = { enAltDrop && altitudeValidForTriggers, enAccX, enAccY, enAccZ, enApogee };
  const bool fired[5] = { triggerAlt, triggerAccX, triggerAccY, triggerAccZ, triggerApogee };

  const bool anyEnabled = enabled[0] || enabled[1] || enabled[2] || enabled[3] || enabled[4];

  bool triggerCondition = false;
  if (anyEnabled) {
    if (useAndLogic) {
      triggerCondition = true;
      for (int i = 0; i < 5; i++) {
        if (enabled[i] && !fired[i]) {
          triggerCondition = false;
          break;
        }
      }
    } else {
      for (int i = 0; i < 5; i++) {
        if (enabled[i] && fired[i]) {
          triggerCondition = true;
          break;
        }
      }
    }
  }

  if (triggerCondition) {
    String triggersList;
    if (triggerAlt) triggersList += "Threshold Altitude Drop,";
    if (triggerAccX) triggersList += "Threshold AccX,";
    if (triggerAccY) triggersList += "Threshold AccY,";
    if (triggerAccZ) triggersList += "Threshold AccZ,";
    if (triggerApogee) triggersList += "Apogee Descent,";

    if (triggersList.length() > 0) triggersList.remove(triggersList.length() - 1);

    lastTriggeredBy = triggersList;
    uiTriggeredBy = lastTriggeredBy;

    parachuteRelease();
  }
} else {
  uiTriggeredBy = lastTriggeredBy.length() ? lastTriggeredBy : "NotTriggered";
}

// --------------------------------------------------------------------------
// Debug prints
// --------------------------------------------------------------------------
if (debugSerial) {
  Serial.print(getTimeStampString());
  Serial.print(" BMP280 Temp: ");
  Serial.println(bmpTemp);
  Serial.print(getTimeStampString());
  Serial.print(" BMP280 Pressure: ");
  Serial.println(pressure);
  Serial.print(getTimeStampString());
  Serial.print(" Absolute Altitude: ");
  Serial.println(absoluteAltitude);
  Serial.print(getTimeStampString());
  Serial.print(" Relative Altitude: ");
  Serial.println(relativeAltitude);
  Serial.print(getTimeStampString());
  Serial.print(" Altitude Drop: ");
  Serial.println(altitudeDrop);
  Serial.print(getTimeStampString());
  Serial.print(" MPU6050 Temp: ");
  Serial.println(temp.temperature);

  Serial.print(getTimeStampString());
  Serial.print(" Accelerometer (raw): ");
  Serial.print(rawAccX);
  Serial.print(", ");
  Serial.print(rawAccY);
  Serial.print(", ");
  Serial.println(rawAccZ);

  Serial.print(getTimeStampString());
  Serial.print(" Accelerometer (calibrated): ");
  Serial.print(relAccX);
  Serial.print(", ");
  Serial.print(relAccY);
  Serial.print(", ");
  Serial.println(relAccZ);

  Serial.print(getTimeStampString());
  Serial.print(" Gyroscope (raw): ");
  Serial.print(g.gyro.x);
  Serial.print(", ");
  Serial.print(g.gyro.y);
  Serial.print(", ");
  Serial.println(g.gyro.z);

  Serial.print(getTimeStampString());
  Serial.print(" Local Pressure: ");
  Serial.println(lastLocalPressure);
  Serial.print(getTimeStampString());
  Serial.print(" Parachute Status: ");
  Serial.println(parachuteStatus);

  Serial.print(getTimeStampString());
  Serial.print(" TriggeredBy: ");
  Serial.println(uiTriggeredBy);
  Serial.println("--------------------");
}

// --------------------------------------------------------------------------
// CSV logging
// --------------------------------------------------------------------------
char dataString[700];
String currentTimestamp = getTimeStampString();

snprintf(
  dataString, sizeof(dataString),
  "%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%.2f,1013.25,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%llu,%llu,%u,%u,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%s,%d,\"%s\",%d,%d,%d,%d,%.3f,%s,%d,%.2f,%d\n",
  currentTimestamp.c_str(),
  bmpTemp,
  pressure,
  absoluteAltitude,
  relativeAltitude,
  altitudeDrop,
  temp.temperature,
  rawAccX, rawAccY, rawAccZ,
  relAccX, relAccY, relAccZ,
  g.gyro.x, g.gyro.y, g.gyro.z,
  parachuteStatus.c_str(),
  lastLocalPressure,
  PressureSource.c_str(),
  maxAbsoluteAltitude, minAbsoluteAltitude,
  maxRelativeAltitude, minRelativeAltitude,
  maxAltitudeDrop, minAltitudeDrop,
  totalSpace, usedSpace,
  spiffsTotal, spiffsUsed,
  currentLatitude, currentLongitude,
  altitudeDropThreshold, accXThreshold, accYThreshold, accZThreshold,
  useAndLogic ? "AND" : "OR",
  axisConfig,
  uiTriggeredBy.c_str(),
  enAltDrop ? 1 : 0,
  enAccX ? 1 : 0,
  enAccY ? 1 : 0,
  enAccZ ? 1 : 0,
  verticalSpeed,
  rocketStatus.c_str(),
  apogeeDetected ? 1 : 0,
  apogeeAltitude,
  enApogee ? 1 : 0);

checkSpiffsSpaceAndWarn();

#if FEATURE_SD_MMC
if (sdMounted) appendCsv(SD_MMC, "/log.csv", dataString);
else appendCsv(SPIFFS, "/log.csv", dataString);
#else
appendCsv(SPIFFS, "/log.csv", dataString);
#endif

// SPIFFS logging only while armed/released and storage is allowed
if ((parachuteStatus == "armed" || parachuteStatus == "released") && spiffsLoggingAllowed) {
  appendCsv(SPIFFS, "/log.csv", dataString);
}

// --------------------------------------------------------------------------
// WebSocket broadcast (flight mode vs visualization mode)
// --------------------------------------------------------------------------
if (sensorModeFlight) {
  static unsigned long previousMillis = 0;
  const int interval = 40;
  unsigned long nowMillis = millis();

  if ((nowMillis - previousMillis) > interval) {
    String jsonString;
    StaticJsonDocument<1536> doc;
    JsonObject object = doc.to<JsonObject>();

    auto safeFloat = [](float v, float fallback) -> float {
      return isfinite(v) ? v : fallback;
    };

    float absAltOut = safeFloat(absoluteAltitude, 0.0f);
    float bmpTempOut = safeFloat(bmpTemp, 0.0f);
    float pressOut = safeFloat(pressure, 0.0f);

    object["AbsoluteAltitude"] = f_or(absoluteAltitude, 0.0f);
    object["RelativeAltitude"] = f_or(relativeAltitude, 0.0f);
    object["VerticalSpeed"] = f_or(verticalSpeed, 0.0f);
    object["ApogeeAltitude"] = f_or(apogeeAltitude, 0.0f);
    object["AltitudeDrop"] = f_or(altitudeDrop, 0.0f);

    object["MaxAbsAltitude"] = f_or(maxAbsoluteAltitude, 0.0f);
    object["MinAbsAltitude"] = f_or(minAbsoluteAltitude, 0.0f);
    object["MaxRelAltitude"] = f_or(maxRelativeAltitude, 0.0f);
    object["MinRelAltitude"] = f_or(minRelativeAltitude, 0.0f);
    object["MaxAltDrop"] = f_or(maxAltitudeDrop, 0.0f);
    object["MinAltDrop"] = f_or(minAltitudeDrop, 0.0f);

    object["BMP280Temp"] = f_or(bmpTemp, 0.0f);
    object["BMP280Pressure"] = f_or(pressure, 0.0f);
    object["MPU6050Temp"] = f_or(temp.temperature, 0.0f);

    object["AccX"] = f_or(relAccX, 0.0f);
    object["AccY"] = f_or(relAccY, 0.0f);
    object["AccZ"] = f_or(relAccZ, 0.0f);

    object["LocalPressure"] = f_or(lastLocalPressure, 1013.25f);
    object["DefaultSeaLevelPressure"] = 1013.25f;
    object["PressureSource"] = PressureSource;

    object["ParachuteStatus"] = parachuteStatus;
    object["RocketStatus"] = rocketStatus;
    object["TriggeredBy"] = uiTriggeredBy;

    object["SensorsCalibrated"] = sensorsCalibrated;
    object["ApogeeDetected"] = apogeeDetected;

    object["EnAltDrop"] = enAltDrop;
    object["EnAccX"] = enAccX;
    object["EnAccY"] = enAccY;
    object["EnAccZ"] = enAccZ;
    object["ApogeeTriggerEnabled"] = enApogee;

    object["TriggerLogic"] = useAndLogic ? "AND" : "OR";

    object["Latitude"] = f_or(currentLatitude, 0.0f);
    object["Longitude"] = f_or(currentLongitude, 0.0f);

    // emit both keys so your JS always finds one
    object["axisConfig"] = axisConfig;
    object["Axis Config"] = axisConfig;

    object["LaunchAccThreshold"] = f_or(LAUNCH_ACC_THRESHOLD, 0.0f);
    object["MinApogeeAlt"] = f_or(MIN_ALTITUDE_FOR_APOGEE, 0.0f);
    object["NegVsConfirm"] = f_or(NEG_VS_CONFIRM, -0.2f);
    object["NegCountConfirm"] = NEG_COUNT_CONFIRM;
    object["GroundSettleMs"] = (uint32_t)GROUND_SETTLE_MS;


    serializeJson(doc, jsonString);
    webSocket.broadcastTXT(jsonString);

    previousMillis = nowMillis;
  }
} else {
  // Visualization mode: lower-bandwidth “event” messages
  if ((millis() - lastTimeGyro) > gyroDelay) {
    String msg = "{\"event\":\"gyro_readings\", \"data\":" + getGyroReadings() + "}";
    webSocket.broadcastTXT(msg);
    lastTimeGyro = millis();
  }
  if ((millis() - lastTimeAcc) > accelerometerDelay) {
    String msg = "{\"event\":\"accelerometer_readings\", \"data\":" + getAccReadings() + "}";
    webSocket.broadcastTXT(msg);
    lastTimeAcc = millis();
  }
  if ((millis() - lastTimeTemperature) > temperatureDelay) {
    String msg = "{\"event\":\"temperature_reading\", \"data\":\"" + getTemperatureReading() + "\"}";
    webSocket.broadcastTXT(msg);
    lastTimeTemperature = millis();
  }
}

// Keep loop timing stable; note that web and sensor work already consumes time.
delay(40);
}
