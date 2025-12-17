/*
 * Flight Computer Firmware for ESP32 – Version 3.8.0
 *
 * ─────────────────────────────────────────────────────────────
 *  What’s new since 3.7.0 (Dashboard & SPIFFS File Manager):
 *
 *  ✦ Improved Height Determination:
 *    - Refined altitude calculation with better noise filtering and apogee logic.
 *    - Vertical speed derived directly from filtered altitude data.
 *
 *  ✦ New Sensors & Telemetry Fields:
 *    - Vertical Speed (m/s): Real-time rate of ascent/descent.
 *    - Apogee Detected: Flags when peak altitude is reached.
 *    - Apogee Altitude: Highest altitude recorded.
 *    - Rocket Status: Displays current flight phase (Idle, Launch, Coast, Descent, Landed).
 *    - Triggered By: Logs the condition that activated parachute deployment.
 *    - Launch Acc Threshold: Minimum acceleration to confirm launch.
 *    - Min Apogee Altitude: Prevents false apogee triggers at low altitude.
 *    - Neg VS Confirm: Requires negative vertical speed to validate apogee.
 *    - Neg Samples Confirm: Ensures multiple samples confirm descent.
 *    - Ground Settle (ms): Stabilization delay before calibration at startup.
 *
 *  ✦ Data Handling & Reliability:
 *    - Automatic SPIFFS-to-SD copy when internal storage is full.
 *    - Fixed SD log download function.
 *    - Added “Delete All” option for SD card cleanup.
 *
 *  ✦ General:
 *    - Improved system stability and synchronization with dashboard.
 *    - Enhanced recovery from calibration and sensor restarts.
 *
 *  Release Date: October 2025
 *
 * ┌───────────────────────────────────────────────┐
 * │                  Endpoints Overview           │
 * └───────────────────────────────────────────────┘
 *
 * ───── HTTP REST API & Static Content ──────────────
 *  GET   /                  → Redirects to /index.html, sets flight mode
 *  GET   /visualization     → Redirects to visualization UI, sets visualization mode
 *  GET   /visualization/    → Serves static visualization files from SPIFFS
 *  GET   /listfiles          → Lists files on SD & SPIFFS (JSON)
 *  GET   /files             → Redirect to UI for files on SD & SPIFFS
 *  GET   /deleteFile        → Delete file from SD or SPIFFS (param: name)
 *  GET   /deleteFileSPIFFS  → Delete file from SPIFFS (param: name)
 *  GET   /deleteFileSD      → Delete file from SD card (param: name)
 *  GET   /downloadFile      → Download file from SD, fallback to SPIFFS (param: name)
 *  POST  /upload            → Upload file to SPIFFS (multipart/form-data)
 *  POST  /uploadsd          → Upload file to SD card (multipart/form-data)
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
 *      showRainbowCycle(wait, direction, rotations)
 *
 *  • Main LED Sequences & System States:
 *      Startup/Unarmed, Wi-Fi status, Parachute Armed/Released, Calibrate Sensors, Visualization Mode
 *
 *  • File Management, Upload/Download, and Warning Indicators:
 *      File Manager, Delete File, Download File, Upload File, SPIFFS Storage Full
 *
 *  • Data Logging:
 *      – All data always logged to SD card if available
 *      – Data also logged to SPIFFS as long as there is enough free space
 *      – If SPIFFS free space drops below threshold, logging is suspended and visual warning shown
 *
 * For troubleshooting, see README and module docs.
 */

//
struct DelStats;  // forward declaration so Arduino's auto-prototype knows the type

// ===== Board feature switches =====
#define FEATURE_SD_MMC 0  // 1 = Freenove with SD_MMC, 0 = standard ESP32-S3 without SD
static bool sdMounted = false;

#define FEATURE_EXTERNAL_NEOPIXEL_STRIP 0  // 1 = your 39 LED ring/strip on GPIO17
#define FEATURE_ONBOARD_NEOPIXEL 1         // 1 = onboard RGB LED

// IMPORTANT: set this to the onboard LED data pin for YOUR board.
#ifndef ONBOARD_NEOPIXEL_PIN
#define ONBOARD_NEOPIXEL_PIN 48  // very common on ESP32-S3 dev boards, but verify for yours
#endif

// -----------------------
// Library Inclusions
// -----------------------
#define sensor_t adafruit_sensor_t
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#undef sensor_t

#include <Wire.h>
#include <Adafruit_Sensor.h>
// #include <Adafruit_BMP280.h>
// #include <Adafruit_MPU6050.h>
#if FEATURE_SD_MMC
#include <SD_MMC.h>
#endif
#include "sd_read_write.h"
#include <ESP32Servo.h>
#include <WiFi.h>
extern "C" {
#include "esp_wifi.h"  // For disabling WiFi power saving (esp_wifi_set_ps)
}
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Arduino_ESP32_OTA.h>
#include <HTTPUpdateServer.h>
// #include <ESPmDNS.h>
#include <EEPROM.h>
#include <SPIFFS.h>
#include <math.h>
#include <vector>




// -----------------------
// LED Circle Setup (Freenove_WS2812)
// -----------------------
// #include "Freenove_WS2812_Lib_for_ESP32.h"
#include <Adafruit_NeoPixel.h>

#if FEATURE_EXTERNAL_NEOPIXEL_STRIP
#define LEDS_COUNT 39
#define LEDS_PIN 17
#else
// Onboard LED: treat it as a 1-pixel NeoPixel strip
#define LEDS_COUNT 1
#define LEDS_PIN ONBOARD_NEOPIXEL_PIN
#endif

Adafruit_NeoPixel strip(LEDS_COUNT, LEDS_PIN, NEO_GRB + NEO_KHZ800);


// -----------------------
// Macro Definitions & Global Variables
// -----------------------
#define EEPROM_SIZE 10                  // EEPROM size
#define EEPROM_PRESSURE_ADDR 0          // EEPROM address for storing pressure
const size_t MAX_SPIFFS_USED_KB = 700;  // Set your preferred limit (in KB)
bool spiffsLoggingAllowed = false;
bool alreadyWarned = false;
bool apiSuccess = false;
bool debugSerial = false;         // Enable/disable serial debug printing
bool sensorModeFlight = true;     // true: Flight sensor mode; false: Visualization mode
int axisConfig = 5;               // 0 = default mapping, 1 = alternative mapping (or more states as needed)
bool sensorsCalibrated = false;   // true when calibrated
bool sensorsCalibWarned = false;  // helper for one-time warning (optional)
bool triggerAbs = true;           // Trigger on both positive and negative acceleration by default
String lastTriggeredBy = "NotTriggered";
String uiTriggeredBy = "NotTriggered";
bool enAltDrop = true;
bool enAccX = true;
bool enAccY = false;
bool enAccZ = true;

bool enApogee = true;
// SD card storage info
uint64_t totalSpace;
uint64_t usedSpace;

// Pressure variables for sensor readings and logging
String PressureSource = "";
float lastLocalPressure = 1013.25;
bool apiPressureUpdated = false;
bool showSensorInitLog = true;
bool bmpFound = false;
bool mpuFound = false;

// -----------------------
// WiFi Credentials and Access Point settings
// -----------------------

const char *ssid = "TDGC-Rocket";
const char *wifiPassword = "Rocket2022!";
const char *apSSID = "RocketAP";
const char *apPassword = "Rocket2022!";

// -----------------------
// OpenWeatherMap API Settings (Anonymized)
// -----------------------
const char *openWeatherMapApiKey = "xxxxxx";  // Anonymized API key
float currentLatitude = 51.07741431;          // Anonymized Latitude
float currentLongitude = 5.88510756;          // Anonymized Longitude
const char *owmEndpoint = "https://api.openweathermap.org/data/3.0/onecall";

// -----------------------
// Time Setup using NTP
// -----------------------
#define UTC_OFFSET_IN_SECONDS 3600  // Offset for local time zone (1 hour)
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", UTC_OFFSET_IN_SECONDS);
unsigned long lastSuccessfulNTP = 0;
unsigned long lastSyncMillis = 0;

// -----------------------
// Trigger & Calibration Variables for Parachute Operation
// -----------------------
float altitudeDropThreshold = 0.8;
float accXThreshold = 8.0;
float accYThreshold = 8.0;
float accZThreshold = 8.0;
bool useAndLogic = false;  // If true, all conditions must meet; if false, any condition triggers

// Baseline for altitude and parachute status variables
float baselineAltitude = 0;
bool baselineCaptured = false;

// --- Altitude filtering & apogee detection additions ---
float ALT_EWMA_ALPHA = 0.20f;
float filteredRelAlt = 0.0f;
float prevFilteredRelAlt = 0.0f;
float verticalSpeed = 0.0f;  // m/s
unsigned long lastAltUpdateMs = 0;

bool apogeeDetected = false;
float apogeeAltitude = 0.0f;
unsigned long apogeeMillis = 0;
int negVsCount = 0;

float MIN_ALTITUDE_FOR_APOGEE = 5.0f;  // meters (tune)
float NEG_VS_CONFIRM = -0.20f;         // m/s (tune)
int NEG_COUNT_CONFIRM = 3;             // consecutive samples (tune)

float LAUNCH_ACC_THRESHOLD = 15.0f;  // m/s^2 (tune)
unsigned long groundCandidateSince = 0;
unsigned long GROUND_SETTLE_MS = 2000;
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
float armedMaxRelativeAltitude = 0;

// Accelerometer calibration offsets
float accXOffset = 0, accYOffset = 0, accZOffset = 0;

// -----------------------
// LED Color Global Variables (will be initialized later)
// -----------------------
uint32_t redColor, blueColor, purpleColor, greenColor, aquaColor, orangeColor;

// -----------------------
// Sensor & Servo Instances
// -----------------------
Adafruit_BMP280 bmp;   // BMP280 sensor instance
Adafruit_MPU6050 mpu;  // MPU6050 sensor instance
Servo parachuteservo;  // Servo controlling the parachute deployment
int servoPin = 14;     // Servo control pin

// Eén bron van waarheid voor de CSV-header
const char *CSV_HEADER_LINE =
  "Timestamp,BMP Temp,Pressure,Absolute Altitude,Relative Altitude,Altitude Drop,MPU Temp,RawAccX,RawAccY,RawAccZ,AccX_Calib,AccY_Calib,AccZ_Calib,GyroX,GyroY,GyroZ,Parachute Status,Local Pressure,Default Sea-Level Pressure,API Status,Max Abs Altitude,Min Abs Altitude,Max Rel Altitude,Min Rel Altitude,Max Alt Drop,Min Alt Drop,Total Space,Used Space,SPIFFS Total,SPIFFS Used,Latitude,Longitude,AltDropThres,AccXThres,AccYThres,AccZThres,TriggerLogic,AxisConfig,TriggeredBy,EnAltDrop,EnAccX,EnAccY,EnAccZ,Vertical Speed,RocketStatus,ApogeeDetected,ApogeeAltitude,EnApogee";

// -----------------------
// Web Server & OTA Instances
// -----------------------
WebServer server(80);
WebSocketsServer webSocket(81);
HTTPUpdateServer httpUpdater;

// -----------------------
// I2C Bus & SD_MMC Pin Definitions
// -----------------------
#define SDA_1 42
#define SCL_1 37
#define SD_MMC_CMD 38
#define SD_MMC_CLK 39
#define SD_MMC_D0 40

// -----------------------
// NEW FEATURE CODE GLOBALS (For visualization updates and gyro calibration)
// -----------------------
// Gyroscope sensor deviation values
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
const float gyroXerror = 0.03;
const float gyroYerror = 0.03;
const float gyroZerror = 0.03;
File uploadFile;  // Global handle for file uploads

// -----------------------
// WebSocket timing variables for Visualization updates
// -----------------------
unsigned long lastTimeGyro = 0;
unsigned long lastTimeAcc = 0;
unsigned long lastTimeTemperature = 0;
const unsigned long gyroDelay = 10;
const unsigned long accelerometerDelay = 200;
const unsigned long temperatureDelay = 1000;

// ---------------------------------------------------------------------------
// Small SPIFF uploader
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Sensor Reading Functions (with Axis Correction applied)
// ---------------------------------------------------------------------------
String getGyroReadings() {
  sensors_event_t a, g, temp;
  if (mpuFound) {
    mpu.getEvent(&a, &g, &temp);
    float rawGx = g.gyro.x, rawGy = g.gyro.y, rawGz = g.gyro.z;
    float corrGx, corrGy, corrGz;
    // Adjust gyroscope axes to correct orientation using the helper function
    correctAxes(rawGx, rawGy, rawGz, corrGx, corrGy, corrGz);
    // Increment the global gyro accumulators if the change is above noise threshold
    if (fabs(corrGx) > gyroXerror) {
      gyroX += corrGx / 30.00;
    }
    if (fabs(corrGy) > gyroYerror) {
      gyroY += corrGy / 30.00;
    }
    if (fabs(corrGz) > gyroZerror) {
      gyroZ += corrGz / 30.00;
    }
    StaticJsonDocument<512> doc;
    doc["gyroX"] = gyroX;
    doc["gyroY"] = gyroY;
    doc["gyroZ"] = gyroZ;
    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString;
  } else {
    StaticJsonDocument<512> doc;
    doc["gyroX"] = "N/A";
    doc["gyroY"] = "N/A";
    doc["gyroZ"] = "N/A";
    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString;
  }
}

String getAccReadings() {
  sensors_event_t a, g, temp;
  StaticJsonDocument<512> doc;
  if (mpuFound) {
    mpu.getEvent(&a, &g, &temp);
    float corrAx, corrAy, corrAz;
    // Correct accelerometer axes for proper orientation
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
  } else {
    return "N/A";
  }
}

// ---------------------------------------------------------------------------
// Route Handler: Redirects to Visualization index page (for sensor mode switch)
// ---------------------------------------------------------------------------
// void handleIndex() {
//
//  server.sendHeader("Location", "/visualization/index.html");
//  server.send(302, "text/plain", "Redirecting...");
//}

// -----------------------
// Function: correctAxes
// -----------------------
// This function swaps and inverts raw sensor axis values based on the chosen configuration.
void correctAxes(float rawX, float rawY, float rawZ, float &corrX, float &corrY, float &corrZ) {
  switch (axisConfig) {
    case 0:
      // Default transformation (as currently implemented).
      // Mapping: Swap x>-X and y>Z and z>Y
      corrX = -rawX;
      corrY = rawY;
      corrZ = rawZ;
      break;
    case 1:
      // Variant 1: Mapping: Swap x>-X and y>Y and z>Z
      corrX = -rawX;
      corrY = rawY;
      corrZ = rawZ;
      break;
    case 2:
      // Variant 2: Mapping: Swap x>Y and y>X and z>Z
      corrX = rawY;
      corrY = rawX;
      corrZ = rawZ;
      break;
    case 3:
      // Variant 3: Mapping: Swap x>Y and y>Z and z>X
      corrX = rawY;
      corrY = rawZ;
      corrZ = rawX;
      break;
    case 4:
      // Variant 4: Mapping: Swap x>Z and y>Y and z>X
      corrX = rawZ;
      corrY = rawY;
      corrZ = rawX;
      break;
    case 5:
      // Variant 5: Mapping Swap x>Z and y>X and z>Y
      corrX = rawZ;
      corrY = rawX;
      corrZ = rawY;
      break;
    default:
      // Fallback to the default transformation if an unrecognized value is used.
      corrX = rawY;
      corrY = rawZ;
      corrZ = rawX;
      break;
  }
}

// NeoPixel color wheel helper, same as Adafruit's demo
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

// -----------------------
// LED Helper Functions
// -----------------------

// Initializes global LED color variables using a color wheel function
void initLEDColors() {
  redColor = Wheel(0);
  blueColor = Wheel(170);
  purpleColor = Wheel(200);
  greenColor = Wheel(85);
  aquaColor = Wheel(125);
  orangeColor = Wheel(30);
}

// Set all LEDs to a specific color
void setAllLEDs(uint32_t color) {
  for (int i = 0; i < LEDS_COUNT; i++) {
    // strip.setLedColorData(i, color);
    strip.setPixelColor(i, color);
  }
  strip.show();
}

/*
// Display a rainbow cycle on the LEDs for a given number of rotations
void showRainbowCycle(uint8_t wait, int8_t direction = 1, uint16_t rotations = 1) {
  for (uint16_t r = 0; r < rotations; r++) {
    for (uint16_t j = 0; j < 256; j++) {
      // 1) Set *all* LED colors in RAM first:
      for (uint16_t i = 0; i < LEDS_COUNT; i++) {
        uint8_t wheelIndex = (((i * 256 / LEDS_COUNT) + (direction * j) + 256) % 256);
        uint32_t color = Wheel(wheelIndex);
        //strip.setLedColorData(i, color);
        strip.setPixelColor(i, color);
      }
      // 2) Now *one* show() for the whole frame:
      strip.show();
      // 3) And a single delay:
      delay(wait);
    }
    setAllLEDs(0);
    delay(100);
  }
}
*/

// Sequentially display a color across the LED strip
void showLEDColorsSequentially(uint32_t color, int8_t direction = 1, uint16_t rotations = 1) {
  // Onboard LED case: no "sequential" animation possible
  if (LEDS_COUNT <= 1) {
    // One "rotation" = one pulse (on/off)
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

  // Strip/ring case (original behavior)
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


// Blink the entire LED strip with a specified color, number of times, and delay between blinks
void blinkColor(uint32_t color, int times, int delayms) {
  for (int t = 0; t < times; t++) {
    setAllLEDs(color);
    delay(delayms);
    setAllLEDs(0);
    delay(delayms);
  }
}


// Set LEDs to alternating red and purple pattern
void setWarningPatternLEDs() {
  if (LEDS_COUNT <= 1) {
    strip.setPixelColor(0, redColor);  // choose red as the "steady warning" color
    strip.show();
    return;
  }

  for (int i = 0; i < LEDS_COUNT; i++) {
    strip.setPixelColor(i, (i % 2 == 0) ? redColor : purpleColor);
  }
  strip.show();
}


// Show the pattern as an animation first, then leave it on
void animateWarningPatternLEDs() {
  if (LEDS_COUNT <= 1) {
    // Onboard LED: alternate red/purple quickly
    for (int k = 0; k < 12; k++) {
      strip.setPixelColor(0, (k % 2 == 0) ? redColor : purpleColor);
      strip.show();
      delay(80);
    }
    // Leave the warning "state" on
    strip.setPixelColor(0, redColor);
    strip.show();
    return;
  }

  // Strip/ring animation (your original logic)
  for (int i = 0; i < LEDS_COUNT; i++) {
    for (int j = 0; j <= i; j++) {
      strip.setPixelColor(j, (j % 2 == 0) ? redColor : purpleColor);
    }
    for (int j = i + 1; j < LEDS_COUNT; j++) {
      strip.setPixelColor(j, 0);
    }
    strip.show();
    delay(30);
  }

  setWarningPatternLEDs();
}


// Indicate WiFi connection status via LED blinking (green if connected, orange if not)
void indicateWiFiStatus(bool connected) {
  if (connected) {
    blinkColor(greenColor, 5, 250);
  } else {
    blinkColor(orangeColor, 5, 250);
  }
}

// -----------------------
// Utility Functions
// -----------------------

// Returns a formatted timestamp string based on NTP or local time
String getTimeStampString() {
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
  } else {
    timeClient.update();
    return String(timeClient.getFormattedTime());
  }
}

// Returns the locally stored sea level pressure
float getLocalSeaLevelPressure() {
  return lastLocalPressure;
}

// ---------------------------------------------------------------------------
// updatePressureFromAPI()
//
// Updates local pressure using an API call to OpenWeatherMap. Falls back to EEPROM or default if necessary.
// ---------------------------------------------------------------------------
void updatePressureFromAPI() {
  float localPressure = 1013.25;

  HTTPClient http;
  String url = String(owmEndpoint) + "?lat=" + String(currentLatitude, 6) + "&lon=" + String(currentLongitude, 6) + "&exclude=minutely,hourly,daily,alerts&appid=" + String(openWeatherMapApiKey);
  http.begin(url);
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    Serial.println("API Call successful. Payload:");
    Serial.println(payload);
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
  Serial.print("EEPROM stored pressure: ");
  Serial.println(storedPressure);
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

//
// Check SPIFF Space and warn
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Check SPIFF Space and warn
// ---------------------------------------------------------------------------
void checkSpiffsSpaceAndWarn() {
  size_t spiffsUsedKB = SPIFFS.usedBytes() / 1024;

  if (spiffsUsedKB >= MAX_SPIFFS_USED_KB) {
    bool ok = false;

#if FEATURE_SD_MMC
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

    // If we could not offload to SD, stop SPIFFS logging and show warning.
    if (!ok) {
      spiffsLoggingAllowed = false;
      alreadyWarned = true;
      setWarningPatternLEDs();  // onboard -> red
      return;
    }

    // Offload succeeded (or SD feature enabled and copy worked): resume normal behavior.
    spiffsLoggingAllowed = true;
    alreadyWarned = false;
    // NOTE: Consider removing this if you don't want SPIFFS housekeeping to override flight LEDs.
    // setAllLEDs(blueColor);
    return;
  }

  // below threshold -> normal
  spiffsLoggingAllowed = true;
  alreadyWarned = false;
}



// ---------------------------------------------------------------------------
// Actuation Functions for Parachute Deployment
// ---------------------------------------------------------------------------

// Executes a parachute release sequence with servo actuation and logs the event.
void parachuteRelease() {
  Serial.println("Trigger condition met! Releasing parachute...");
  char eventLog[128];
  String eventTimestamp = getTimeStampString();
  //  snprintf(eventLog, sizeof(eventLog), "Timestamp: %s, Event: Parachute Released!\n", eventTimestamp.c_str());
  //  appendCsv(SD_MMC, "/log.csv", eventLog);
  // if (spiffsLoggingAllowed) {
  //    appendCsv(SPIFFS, "/log.csv", eventLog);
  //  }

  for (int i = 0; i < 2; i++) {
    parachuteservo.write(0);
  }
  parachuteStatus = "released";
  blinkColor(greenColor, 5, 100);
  showLEDColorsSequentially(greenColor, -1, 2);
  //  strip.show();

  setAllLEDs(greenColor);
  // strip.show();
}

// Arms the parachute system, captures baseline altitude, and initializes sensor calibration.
void parachuteArmed() {

  uiTriggeredBy = "NotTriggered";
  lastTriggeredBy = "NotTriggered";
  rocketStatus = "On Launchpad";

  // Altijd eerst kalibreren vóór armeren
  calibrateSensors();

  Serial.println("Arming parachute...");
  char eventLog[128];
  String eventTimestamp = getTimeStampString();
  //  snprintf(eventLog, sizeof(eventLog), "Timestamp: %s, Event: Parachute Armed!\n", eventTimestamp.c_str());
  //  appendCsv(SD_MMC, "/log.csv", eventLog);
  // checkSpiffsSpaceAndWarn();
  //  if (spiffsLoggingAllowed) {
  //    appendCsv(SPIFFS, "/log.csv", eventLog);
  //  }

  if (!baselineCaptured) {
    baselineAltitude = bmp.readAltitude(getLocalSeaLevelPressure());
    baselineCaptured = true;
    Serial.print("Baseline altitude captured: ");
    Serial.print(baselineAltitude);
    Serial.println(" m");
  }
  float currentRel = bmp.readAltitude(getLocalSeaLevelPressure()) - baselineAltitude;
  armedMaxRelativeAltitude = currentRel;
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

  if (alreadyWarned) {
    setWarningPatternLEDs();  // Or animateWarningPatternLEDs();
  } else {
    showLEDColorsSequentially(redColor, 1, 3);
    setAllLEDs(redColor);
  }
}

// Calibrates sensors by capturing baseline altitude and accelerometer offsets.
void calibrateSensors() {
  rocketStatus = "Calibrating";
  uiTriggeredBy = "NotTriggered";
  lastTriggeredBy = "NotTriggered";
  Serial.println("Calibrating sensors...");
  parachutePreStatus = parachuteStatus;
  parachuteStatus = "calibrating";
  Serial.println("Parachute status set to 'calibrating' for calibration.");
  //   strip.show();
  baselineAltitude = bmp.readAltitude(getLocalSeaLevelPressure());
  baselineCaptured = true;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float corrAx, corrAy, corrAz;
  correctAxes(a.acceleration.x, a.acceleration.y, a.acceleration.z, corrAx, corrAy, corrAz);
  accXOffset = corrAx;
  accYOffset = corrAy;
  accZOffset = corrAz;
  maxRelativeAltitude = 0;
  minRelativeAltitude = 1000000.0;
  maxAltitudeDrop = 0;
  minAltitudeDrop = 1000000.0;
  Serial.print("Calibration complete. Baseline altitude: ");
  Serial.println(baselineAltitude);
  showLEDColorsSequentially(orangeColor, 1, 1);   // seq. orange, forward, 1 rot.
  showLEDColorsSequentially(orangeColor, -1, 1);  // seq. orange, reverse, 1 rot.

  parachuteStatus = parachutePreStatus;
  Serial.println("Parachute status restored post-calibration.");
  // If we’re back to “armed”, show solid red LEDs:
  if (!alreadyWarned) {
    if (parachuteStatus == "armed")
      setAllLEDs(redColor);
    if (parachuteStatus == "unarmed")
      setAllLEDs(blueColor);
    if (parachuteStatus == "released")
      setAllLEDs(greenColor);
  } else {
    setWarningPatternLEDs();
  }
  // Reset sensor mode to Flight mode after calibration
  sensorsCalibrated = true;
  sensorModeFlight = true;
  Serial.print("Reset sensor mode to Flight");
  Serial.println(sensorModeFlight);
}

void appendCsv(fs::FS &fs, const char *path, const String &line) {
  bool needHeader = !fs.exists(path);
  if (!needHeader) {
    File fr = fs.open(path, FILE_READ);
    if (!fr)
      needHeader = true;
    else {
      if (fr.size() == 0)
        needHeader = true;
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

// Function to make sure headers are always written to file
void ensureLogFileHasHeader(fs::FS &fs, const char *path) {
  if (!fs.exists(path)) {
    // Bestaat niet: header toevoegen
    File f = fs.open(path, FILE_WRITE);
    if (f) {
      f.println(CSV_HEADER_LINE);
      f.close();
    }
  } else {
    // Bestaat wél: check of hij leeg is (==0 bytes)
    File f = fs.open(path, FILE_READ);
    if (f && f.size() == 0) {
      f.close();
      // Voeg alsnog headers toe
      File fw = fs.open(path, FILE_WRITE);
      if (fw) {
        fw.println(CSV_HEADER_LINE);
        fw.close();
      }
    } else if (f) {
      f.close();
    }
  }
}

void webSocketEvent(byte num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("Client " + String(num) + " disconnected");
      //   blinkColor(orangeColor, 4, 300);
      break;
    case WStype_CONNECTED:
      Serial.println("Client " + String(num) + " connected");
      //  blinkColor(aquaColor, 4, 300);
      break;
    case WStype_TEXT:
      {
        StaticJsonDocument<2048> doc;  // plenty for your largest UI payloads
        DeserializationError error = deserializeJson(doc, payload);
        if (error) {
          Serial.print(F("WS JSON parse failed: "));
          Serial.println(error.f_str());
          return;
        } else {
          if (doc.containsKey("newThreshold")) {
            altitudeDropThreshold = doc["newThreshold"];
            Serial.print("New Altitude Drop Threshold: ");
            Serial.println(altitudeDropThreshold);
          }
          if (doc.containsKey("newAccX")) {
            accXThreshold = doc["newAccX"];
            Serial.print("New Accelerometer X threshold: ");
            Serial.println(accXThreshold);
          }
          if (doc.containsKey("newAccY")) {
            accYThreshold = doc["newAccY"];
            Serial.print("New Accelerometer Y threshold: ");
            Serial.println(accYThreshold);
          }
          if (doc.containsKey("newAccZ")) {
            accZThreshold = doc["newAccZ"];
            Serial.print("New Accelerometer Z threshold: ");
            Serial.println(accZThreshold);
          }

          // NEW: handle trigger checkbox updates
          if (doc.containsKey("enAltDrop")) {
            enAltDrop = doc["enAltDrop"];
            Serial.printf("Enable Altitude Drop Trigger: %s\n", enAltDrop ? "Yes" : "No");
          }
          if (doc.containsKey("enAccX")) {
            enAccX = doc["enAccX"];
            Serial.printf("Enable AccX Trigger: %s\n", enAccX ? "Yes" : "No");
          }
          if (doc.containsKey("enAccY")) {
            enAccY = doc["enAccY"];
            Serial.printf("Enable AccY Trigger: %s\n", enAccY ? "Yes" : "No");
          }
          if (doc.containsKey("enAccZ")) {
            enAccZ = doc["enAccZ"];
            Serial.printf("Enable AccZ Trigger: %s\n", enAccZ ? "Yes" : "No");
          }
          if (doc.containsKey("enApogee")) {
            enApogee = doc["enApogee"];
            Serial.printf("Enable Apogee Trigger: %s\n", enApogee ? "Yes" : "No");
          }

          // --- Nieuwe drempel-instellingen vanuit de UI ---
          if (doc.containsKey("minApogeeAlt")) {
            MIN_ALTITUDE_FOR_APOGEE = doc["minApogeeAlt"].as<float>();
            Serial.printf("MIN_ALTITUDE_FOR_APOGEE = %.2f\n", MIN_ALTITUDE_FOR_APOGEE);
          }
          if (doc.containsKey("negVsConfirm")) {
            NEG_VS_CONFIRM = doc["negVsConfirm"].as<float>();
            Serial.printf("NEG_VS_CONFIRM = %.3f\n", NEG_VS_CONFIRM);
          }
          if (doc.containsKey("negCountConfirm")) {
            NEG_COUNT_CONFIRM = doc["negCountConfirm"].as<int>();
            Serial.printf("NEG_COUNT_CONFIRM = %d\n", NEG_COUNT_CONFIRM);
          }
          if (doc.containsKey("launchAccThreshold")) {
            LAUNCH_ACC_THRESHOLD = doc["launchAccThreshold"].as<float>();
            Serial.printf("LAUNCH_ACC_THRESHOLD = %.2f\n", LAUNCH_ACC_THRESHOLD);
          }
          if (doc.containsKey("groundSettleMs")) {
            GROUND_SETTLE_MS = doc["groundSettleMs"].as<unsigned long>();
            Serial.printf("GROUND_SETTLE_MS = %lu\n", (unsigned long)GROUND_SETTLE_MS);
          }

          // Stuur de actuele waarden meteen terug naar de UI zodat velden syncen
          {
            StaticJsonDocument<512> ack;
            ack["LaunchAccThreshold"] = LAUNCH_ACC_THRESHOLD;
            ack["MinApogeeAlt"] = MIN_ALTITUDE_FOR_APOGEE;
            ack["NegVsConfirm"] = NEG_VS_CONFIRM;
            ack["NegCountConfirm"] = NEG_COUNT_CONFIRM;
            ack["GroundSettleMs"] = GROUND_SETTLE_MS;
            // add current trigger config too:
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
            Serial.print("New Trigger Logic: ");
            Serial.println(useAndLogic ? "AND" : "OR");
          }
          if (doc.containsKey("triggerAbs")) {
            triggerAbs = doc["triggerAbs"];
            Serial.print("New Trigger Absolute (both sides): ");
            Serial.println(triggerAbs ? "Yes" : "No");
          }
          if (doc.containsKey("calibrateSensors")) {
            calibrateSensors();
            Serial.println("Sensors calibrated.");
          }
          // Handle axis configuration update from the web interface
          if (doc.containsKey("sensorModeFlight")) {
            sensorModeFlight = doc["sensorModeFlight"];
            Serial.printf("SensorModeFlight: %s\n", sensorModeFlight ? "ON" : "OFF");
          }
          if (doc.containsKey("axisConfig")) {
            axisConfig = doc["axisConfig"];
            Serial.print("Axis configuration updated to: ");
            Serial.println(axisConfig);
          }

          const char *command = doc["parachute"];
          Serial.println("Received parachute command from client " + String(num));
          if (String(command) == "Armed") {
            parachuteArmed();
            Serial.println("Parachute armed command processed.");
          } else if (String(command) == "Released") {
            parachuteRelease();
            Serial.println("Parachute release command processed.");
          } else if (String(command) == "Unarmed") {
            parachuteUnarmed();
            Serial.println("Parachute unarmed command processed.");
          }
          if (doc.containsKey("request") && String(doc["request"]) == "telemetry") {
            // One-shot snapshot is handled by main loop publishing shortly after
          }

          // NEW: handle location updates
          if (doc.containsKey("latitude") && doc.containsKey("longitude")) {
            currentLatitude = doc["latitude"];
            currentLongitude = doc["longitude"];
            // 1) clear the flag
            apiPressureUpdated = false;
            // 2) fetch right away, using the new coords
            updatePressureFromAPI();
            // 3) push the fresh pressure + coords back to the page
            StaticJsonDocument<128> out;
            out["Latitude"] = currentLatitude;
            out["Longitude"] = currentLongitude;
            out["LocalPressure"] = lastLocalPressure;
            out["PressureSource"] = PressureSource;
            String s;
            serializeJson(out, s);
            webSocket.broadcastTXT(s);
          }
        }
        Serial.println("");
        break;
      }
  }
}

void parachuteUnarmed() {
  parachuteStatus = "unarmed";
  uiTriggeredBy = "NotTriggered";
  lastTriggeredBy = "NotTriggered";
  baselineCaptured = false;  // Optional: Clear baseline so it's recalculated next time
  if (!alreadyWarned) {
    setAllLEDs(blueColor);  // Blue = unarmed/safe
  } else {
    setWarningPatternLEDs();  // If SPIFFS warning active, keep warning pattern
  }
  Serial.println("Parachute is now UNARMED. Logging to SPIFFS will stop unless armed.");
}

// ========== FILE MANAGEMENT FUNCTIONS ==========

// -- Helper to list files in a given FS (SPIFFS or SD_MMC)
String listFilesJSON(fs::FS &fs, const char *path = "/") {
  String json = "[";
  File root = fs.open(path);
  if (!root || !root.isDirectory()) {
    return "[]";
  }
  File file = root.openNextFile();
  bool first = true;
  while (file) {
    if (!first)
      json += ",";
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

// JSON endpoint voor SD & SPIFFS files
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

// -- Handle delete from SPIFFS (expects GET param: name=/filename.txt)
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
  if (SPIFFS.remove(fname)) {
    server.send(200, "text/plain", "Deleted from SPIFFS");
  } else {
    server.send(500, "text/plain", "Failed to delete from SPIFFS");
  }
}

// -- Handle delete from SD (expects GET param: name=/filename.txt)
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

  if (SD_MMC.remove(fname))
    server.send(200, "text/plain", "Deleted from SD");
  else
    server.send(500, "text/plain", "Failed to delete from SD");
#else
  server.send(501, "text/plain", "SD feature disabled");
#endif
}


// -- Handle download: tries SD first, then SPIFFS (expects GET param: name=/filename.txt)
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

// -- Handle upload to SPIFFS
void handleFileUpload() {
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String fname = upload.filename;
    if (!fname.startsWith("/"))
      fname = "/" + fname;
    uploadFile = SPIFFS.open(fname, FILE_WRITE);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile)
      uploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile)
      uploadFile.close();
  }
}

// -- Handle upload to SD
void handleFileUploadSD() {
#if !FEATURE_SD_MMC
  // If someone hits /uploadsd, reject cleanly
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


// -- Optionally: Delete file from both SD and SPIFFS
void handleCombinedFileDelete() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing file name");
    return;
  }
  String fname = server.arg("name");
  bool deleted = false;

#if FEATURE_SD_MMC
  if (sdMounted && SD_MMC.exists(fname))
    deleted |= SD_MMC.remove(fname);
#endif

  if (SPIFFS.exists(fname))
    deleted |= SPIFFS.remove(fname);

  if (deleted)
    server.send(200, "text/plain", "Deleted from SD or SPIFFS");
  else
    server.send(404, "text/plain", "File not found on SD or SPIFFS");
}


// ---- SD "Delete All" helpers (place above setup()) -------------------------
struct DelStats {
  uint32_t removed_files = 0;
  uint32_t removed_dirs = 0;
  uint32_t failed = 0;
};

static bool isProtectedEntry(const String &path) {
  // Windows leaves this read-only folder; skip it so the sweep continues.
  if (path.equalsIgnoreCase("/System Volume Information"))
    return true;
  return false;
}

static bool buildChildPath(const char *parent, const char *name, String &out) {
  if (!parent || !name)
    return false;
  // If 'name' is already an absolute path, just use it.
  if (name[0] == '/') {
    out = name;
    return true;
  }
  out = parent;
  if (!out.endsWith("/"))
    out += "/";
  out += name;
  return true;
}

bool deleteRecursivelyWithStats(fs::FS &fs, const char *dirPath, DelStats &stats) {
  File root = fs.open(dirPath);
  if (!root || !root.isDirectory())
    return false;

  File f = root.openNextFile();
  while (f) {
    // Build absolute child path (SD_MMC sometimes returns only the entry name)
    String childPath;
    const char *entryName = f.name();
    if (!buildChildPath(dirPath, entryName, childPath)) {
      stats.failed++;
      f.close();
      f = root.openNextFile();
      continue;
    }

    // Skip protected entries; keep going
    if (isProtectedEntry(childPath)) {
      f.close();
      f = root.openNextFile();
      continue;
    }

    if (f.isDirectory()) {
      f.close();  // Close before recursing into it
      deleteRecursivelyWithStats(fs, childPath.c_str(), stats);

      // Try to remove the now-empty directory (some FS may not support rmdir)
      if (fs.rmdir(childPath.c_str())) {
        stats.removed_dirs++;
      } else {
        // Not fatal; record and continue
        stats.failed++;
      }
    } else {
      f.close();
      if (fs.remove(childPath.c_str())) {
        stats.removed_files++;
      } else {
        stats.failed++;
      }
    }
    f = root.openNextFile();
  }
  root.close();
  return true;
}

void handleDeleteAllSD() {
  blinkColor(redColor, 3, 150);

#if !FEATURE_SD_MMC
  // SD not supported in this build
  server.send(501, "text/plain", "SD feature disabled");
  return;
#else
  if (!sdMounted) {
    server.send(503, "text/plain", "SD not mounted");
    return;
  }

  DelStats stats;

  // Sweep everything under "/" (we do not attempt to remove the root itself)
  deleteRecursivelyWithStats(SD_MMC, "/", stats);

  // Recreate the log file so logging can immediately resume
  File file = SD_MMC.open("/log.csv", FILE_WRITE);
  if (file) {
    file.println(CSV_HEADER_LINE);  // Use your single source of truth header
    file.close();
  } else {
    stats.failed++;
  }

  String json = "{";
  json += "\"removed_files\":" + String(stats.removed_files) + ",";
  json += "\"removed_dirs\":" + String(stats.removed_dirs) + ",";
  json += "\"failed\":" + String(stats.failed) + "}";
  // 207 Multi-Status if anything failed; 200 if a clean sweep
  server.send(stats.failed == 0 ? 200 : 207, "application/json", json);
#endif
}


void handleSpiffsUpload() {
  HTTPUpload &upload = server.upload();
  static String targetFolder;
  static File uploadFile;
  if (upload.status == UPLOAD_FILE_START) {
    // Get the folder from POST body, fallback to root if empty
    targetFolder = server.arg("folder");
    if (targetFolder.length() == 0)
      targetFolder = "/";
    if (!targetFolder.startsWith("/"))
      targetFolder = "/" + targetFolder;
    if (!targetFolder.endsWith("/"))
      targetFolder += "/";
    String fname = upload.filename;
    String fullPath = targetFolder + fname;

    // Create folder if not exist
    if (!SPIFFS.exists(targetFolder.c_str()))
      SPIFFS.mkdir(targetFolder.c_str());
    uploadFile = SPIFFS.open(fullPath, FILE_WRITE);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile)
      uploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile)
      uploadFile.close();
  }
}

// -----------------------
// Setup Function: Initializes hardware, network, sensors, web server, and endpoints
// -----------------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting setup..."));
  rocketStatus = "Booting";
  Serial.print(rocketStatus);

  // Mount SPIFFS for file serving and upload functionality.
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
  } else {
    Serial.println("SPIFFS mounted successfully");
  }
  ensureLogFileHasHeader(SPIFFS, "/log.csv");


  // 1) Load your storedPressure from EEPROM up front:
  EEPROM.begin(EEPROM_SIZE);
  float storedPressure;
  EEPROM.get(EEPROM_PRESSURE_ADDR, storedPressure);


  // Disable WiFi power saving mode
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);


  // Connect to WiFi as a station
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
  if (WiFi.status() == WL_CONNECTED) {
    updatePressureFromAPI();
  }

  // 2) Now do your three‑way branch:
  if (apiSuccess == true) {
    PressureSource = "API OpenWeather";
  } else if (storedPressure > 500.0 && storedPressure < 1100.0) {
    lastLocalPressure = storedPressure;
    PressureSource = "EEPROM Memory";
  } else {
    lastLocalPressure = 1013.25;
    PressureSource = "Default Sea-Level Pressure";
  }

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
  // SD Card Initialization and Log File Handling
  // -----------------------
#if FEATURE_SD_MMC

  Serial.print("Initializing SD card...");
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);

  sdMounted = SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5);
  if (!sdMounted) {
    Serial.println("Card Mount Failed");
  } else {
    Serial.println("SD Card initialized.");
    ensureLogFileHasHeader(SD_MMC, "/log.csv");  // SD is mounted

    String oldLogFile = "/log.csv";
    String timestamp = getTimeStampString();
    timestamp.replace(":", "");
    timestamp.replace(" ", "_");
    String newLogFile = "/" + timestamp + ".csv";

    // These exists/remove/rename calls are safe now because sdMounted == true
    if (SD_MMC.exists(newLogFile.c_str()))
      SD_MMC.remove(newLogFile.c_str());

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


  // -----------------------
  // Initialize I2C Bus for sensor communication
  // -----------------------
  Wire.begin(SDA_1, SCL_1);

  // -----------------------
  // Initialize sensors (MPU6050 and BMP280)
  // -----------------------
  if (mpu.begin(0x68)) {
    mpuFound = true;
    if (showSensorInitLog) {
      Serial.println("MPU6050 sensor found.");
    }
  } else {
    mpuFound = false;
    if (showSensorInitLog) {
      Serial.println("Could not find MPU6050 sensor. Check wiring!");
    }
  }
  if (bmp.begin(0x76)) {
    bmpFound = true;
    if (showSensorInitLog) {
      Serial.println("BMP280 sensor found.");
    }
  } else {
    bmpFound = false;
    if (showSensorInitLog) {
      Serial.println("Could not find BMP280 sensor. Check wiring!");
    }
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

  // -----------------------
  // Setup OTA update mechanism and mDNS services
  // -----------------------
  httpUpdater.setup(&server);
  // if (MDNS.begin("esp32-webupdate")) {
  //  Serial.println("MDNS responder started");
  // }
  // MDNS.addService("http", "tcp", 80);
  // Serial.printf("HTTPUpdateServer ready! Open http://esp32-webupdate.local/update in your browser\n");

  // Allocate timers for PWM (required by the servo library)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Configure and attach the servo controlling the parachute
  parachuteservo.setPeriodHertz(50);
  parachuteservo.attach(servoPin, 1000, 2000);
  delay(1000);

  // -----------------------
  // Initialize LED Strip
  // -----------------------
  strip.begin();
  Serial.println("Strip Begin");
  strip.setBrightness(20);
  Serial.println("Set Brightness");
  strip.show();
  Serial.println("Strip Show");
  delay(1000);
  initLEDColors();
  Serial.println("LED Colors Initialized");
  strip.show();
  delay(1000);
  showLEDColorsSequentially(purpleColor, 1, 3);
  Serial.println("Sequential Display Done");
  delay(1000);

  Serial.println("Setup complete.");
  indicateWiFiStatus(WiFi.status() == WL_CONNECTED);
  setAllLEDs(blueColor);
  // strip.show();

  // -----------------------
  // Register Web Server Endpoints
  // -----------------------
  // Root route: serves static flight information page
  server.on("/", HTTP_GET, []() {
    sensorModeFlight = true;
    setAllLEDs(blueColor);
    server.sendHeader("Location", "/index.html");
    server.send(302, "text/plain", "Redirecting...");
  });

  // Inline visualization‐mode handler:
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

  // Serve the static visualization files from SPIFFS
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

  // --- Upload Endpoints ---
  server.on(
    "/upload", HTTP_POST, []() {
      blinkColor(greenColor, 2, 250);
      server.send(200, "text/plain", "SPIFFS Upload Successful");
    },
    handleFileUpload  // <-- Handles actual file upload for SPIFFS
  );

  server.on(
    "/uploadsd", HTTP_POST, []() {
      blinkColor(greenColor, 2, 250);
      server.send(200, "text/plain", "SD Card Upload Successful");
    },
    handleFileUploadSD  // <-- Handles actual file upload for SD
  );

  server.on("/listfiles", HTTP_GET, handleListFilesJson);

  // Sensor endpoints: serve gyro, accelerometer, and temperature data.
  server.on("/gyro", HTTP_GET, []() {
    server.send(200, "application/json", getGyroReadings());
  });
  server.on("/acc", HTTP_GET, []() {
    server.send(200, "application/json", getAccReadings());
  });
  server.on("/temp", HTTP_GET, []() {
    server.send(200, "text/plain", getTemperatureReading());
  });
  // Reset endpoints for individual gyro components.
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

  // Serve static dashboard files from SPIFFS
  // server.serveStatic("/", SPIFFS, "/index.html");  // This will serve /index.html for root
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
    "/spiffsupload", HTTP_POST, []() {
      server.send(200, "text/plain", "Upload complete! Refresh to upload more files.");
    },
    handleSpiffsUpload);

  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

// -----------------------
// Main Loop Function
// -----------------------
void loop() {
  server.handleClient();
  webSocket.loop();

  // Update pressure from API if needed
  if (WiFi.status() == WL_CONNECTED) {
    if (!apiPressureUpdated)
      updatePressureFromAPI();
  } else {
    apiPressureUpdated = false;
  }

  // Update SD and SPIFFS stats
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

  // Sensor reads
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


  // Axis correction + calibration
  float corrAx, corrAy, corrAz;
  correctAxes(a.acceleration.x, a.acceleration.y, a.acceleration.z, corrAx, corrAy, corrAz);
  float relAccX = corrAx - accXOffset;
  float relAccY = corrAy - accYOffset;
  float relAccZ = corrAz - accZOffset;

  // Raw readings
  float rawAccX = a.acceleration.x;
  float rawAccY = a.acceleration.y;
  float rawAccZ = a.acceleration.z;

  // Altitude & drops

  float relativeAltitude = (parachuteStatus == "unarmed") ? 0 : absoluteAltitude - baselineAltitude;
  // --- Filtered altitude + vertical speed ---
  unsigned long _nowAltMs = millis();
  if (lastAltUpdateMs == 0) {
    filteredRelAlt = relativeAltitude;
    prevFilteredRelAlt = relativeAltitude;
    lastAltUpdateMs = _nowAltMs;
  }
  float _dt = (_nowAltMs - lastAltUpdateMs) / 1000.0f;
  if (_dt <= 0.0f)
    _dt = 0.001f;
  filteredRelAlt = ALT_EWMA_ALPHA * relativeAltitude + (1.0f - ALT_EWMA_ALPHA) * filteredRelAlt;
  verticalSpeed = (filteredRelAlt - prevFilteredRelAlt) / _dt;
  prevFilteredRelAlt = filteredRelAlt;
  lastAltUpdateMs = _nowAltMs;

  if (absoluteAltitude > maxAbsoluteAltitude)
    maxAbsoluteAltitude = absoluteAltitude;
  if (absoluteAltitude < minAbsoluteAltitude)
    minAbsoluteAltitude = absoluteAltitude;
  if (relativeAltitude > maxRelativeAltitude)
    maxRelativeAltitude = relativeAltitude;
  if (relativeAltitude < minRelativeAltitude)
    minRelativeAltitude = relativeAltitude;
  float altitudeDrop = (parachuteStatus == "armed") ? armedMaxRelativeAltitude - relativeAltitude : maxRelativeAltitude - relativeAltitude;
  if (altitudeDrop > maxAltitudeDrop)
    maxAltitudeDrop = altitudeDrop;
  if (altitudeDrop < minAltitudeDrop)
    minAltitudeDrop = altitudeDrop;

  // --- Rocket status & apogee detection ---
  // Launch detection (use Z-up calibrated acceleration)
  if (rocketStatus == "On Launchpad" && relAccZ > LAUNCH_ACC_THRESHOLD) {
    rocketStatus = "Launched";
    apogeeDetected = false;
    apogeeAltitude = 0.0f;
    negVsCount = 0;
  }

  // Apogee detection (requires sufficient altitude and sustained negative vertical speed)
  if (!apogeeDetected && filteredRelAlt > MIN_ALTITUDE_FOR_APOGEE) {
    if (verticalSpeed < NEG_VS_CONFIRM) {
      negVsCount++;
      if (negVsCount >= NEG_COUNT_CONFIRM) {
        apogeeDetected = true;
        apogeeAltitude = filteredRelAlt;
        apogeeMillis = millis();
        rocketStatus = "Max Apogee";
      }
    } else {
      if (negVsCount > 0)
        negVsCount--;
    }
  }

  // Alleen “On Ground” toestaan ná daadwerkelijke vlucht
  bool wasInFlight = (rocketStatus == "Launched" || rocketStatus == "Max Apogee");
  if (wasInFlight && fabs(filteredRelAlt) < 1.0f && fabs(verticalSpeed) < 0.05f) {
    if (groundCandidateSince == 0)
      groundCandidateSince = millis();
    else if (millis() - groundCandidateSince > GROUND_SETTLE_MS) {
      rocketStatus = "On Ground";
    }
  } else {
    groundCandidateSince = 0;
  }

  // --- Trigger evaluatie & oorzaakvastlegging (vervangt het hele oude blok) ---
  uiTriggeredBy = lastTriggeredBy;  // wat we aan de UI tonen

  if (parachuteStatus == "armed") {
    // Terwijl we armed zijn, is de default-weergave "NotTriggered" totdat iets triggert
    uiTriggeredBy = "NotTriggered";

    // Evaluate only enabled triggers
    const bool triggerAlt = enAltDrop ? (altitudeDrop >= altitudeDropThreshold) : false;
    const bool triggerAccX = enAccX ? (triggerAbs ? (fabs(relAccX) >= accXThreshold) : (relAccX >= accXThreshold)) : false;
    const bool triggerAccY = enAccY ? (triggerAbs ? (fabs(relAccY) >= accYThreshold) : (relAccY >= accYThreshold)) : false;
    const bool triggerAccZ = enAccZ ? (triggerAbs ? (fabs(relAccZ) >= accZThreshold) : (relAccZ >= accZThreshold)) : false;
    const bool triggerApogee = enApogee ? apogeeDetected : false;

    // AND/OR logica over alleen de ingeschakelde triggers
    const bool enabled[5] = { enAltDrop, enAccX, enAccY, enAccZ, enApogee };
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

    // Track what triggered
    if (triggerCondition) {
      String triggersList;
      if (triggerAlt)
        triggersList += "Threshold Altitude Drop,";
      if (triggerAccX)
        triggersList += "Threshold AccX,";
      if (triggerAccY)
        triggersList += "Threshold AccY,";
      if (triggerAccZ)
        triggersList += "Threshold AccZ,";
      if (triggerApogee)
        triggersList += "Apogee Descent,";
      if (triggersList.length() > 0)
        triggersList.remove(triggersList.length() - 1);  // strip trailing comma
      lastTriggeredBy = triggersList;
      uiTriggeredBy = lastTriggeredBy;
      parachuteRelease();  // zet status → "released"
    }
  } else {
    // Niet-armed: blijf de laatst bekende oorzaak tonen (of NotTriggered als er geen is)
    uiTriggeredBy = lastTriggeredBy.length() ? lastTriggeredBy : "NotTriggered";
  }

  // --- Print sensor info (now uses updated triggeredBy) ---
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
    Serial.print(" Max Abs Altitude: ");
    Serial.print(maxAbsoluteAltitude);
    Serial.print(" m, Min Abs Altitude: ");
    Serial.println(minAbsoluteAltitude);
    Serial.print(getTimeStampString());
    Serial.print(" Max Rel Altitude: ");
    Serial.print(maxRelativeAltitude);
    Serial.print(" m, Min Rel Altitude: ");
    Serial.println(minRelativeAltitude);
    Serial.print(getTimeStampString());
    Serial.print(" Thresholds: AltDrop=");
    Serial.print(altitudeDropThreshold);
    Serial.print(" AccX=");
    Serial.print(accXThreshold);
    Serial.print(" AccY=");
    Serial.print(accYThreshold);
    Serial.print(" AccZ=");
    Serial.print(accZThreshold);
    Serial.print(" TriggerLogic=");
    Serial.print(useAndLogic ? "AND" : "OR");
    Serial.print(" AxisConfig=");
    Serial.print(axisConfig);
    Serial.print(" Lat=");
    Serial.print(currentLatitude, 6);
    Serial.print(" Lon=");
    Serial.println(currentLongitude, 6);
    Serial.print(getTimeStampString());
    Serial.print(" TriggeredBy: ");
    Serial.println(uiTriggeredBy);
    Serial.println("--------------------");
  }

  // --- LOG FORMAT (add all extra columns) ---
  char dataString[700];
  String currentTimestamp = getTimeStampString();
  snprintf(
    dataString, sizeof(dataString),
    "%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%.2f,1013.25,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%llu,%llu,%u,%u,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%s,%d,\"%s\",%d,%d,%d,%d,%.3f,%s,%d,%.2f,%d\n",
    currentTimestamp.c_str(),  // Timestamp
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
  if (sdMounted) {
    appendCsv(SD_MMC, "/log.csv", dataString);
  } else {
    appendCsv(SPIFFS, "/log.csv", dataString);
  }
#else
  appendCsv(SPIFFS, "/log.csv", dataString);
#endif

  if ((parachuteStatus == "armed" || parachuteStatus == "released") && spiffsLoggingAllowed) {
    appendCsv(SPIFFS, "/log.csv", dataString);
  }


  // --- WebSocket Output (also includes new fields) ---
  if (sensorModeFlight) {
    static unsigned long previousMillis = 0;
    int interval = 40;
    unsigned long nowMillis = millis();
    if ((nowMillis - previousMillis) > interval) {
      String jsonString = "";
      StaticJsonDocument<1536> doc;
      JsonObject object = doc.to<JsonObject>();
      object["AbsoluteAltitude"] = absoluteAltitude;
      object["RelativeAltitude"] = relativeAltitude;
      object["AltitudeDrop"] = altitudeDrop;
      object["MaxAltDrop"] = maxAltitudeDrop;
      object["MinAltDrop"] = minAltitudeDrop;
      object["BMP280Temp"] = bmpTemp;
      object["BMP280Pressure"] = pressure;
      object["MPU6050Temp"] = temp.temperature;  // GOED
      object["RawAccX"] = rawAccX;
      object["RawAccY"] = rawAccY;
      object["RawAccZ"] = rawAccZ;
      object["AccX"] = relAccX;
      object["AccY"] = relAccY;
      object["AccZ"] = relAccZ;
      float rawGx = g.gyro.x, rawGy = g.gyro.y, rawGz = g.gyro.z;
      float corrGx, corrGy, corrGz;
      correctAxes(rawGx, rawGy, rawGz, corrGx, corrGy, corrGz);
      object["Gyroscope"] = String(corrGx) + "; " + String(corrGy) + "; " + String(corrGz);
      object["ParachuteStatus"] = parachuteStatus;
      object["LocalPressure"] = lastLocalPressure;
      object["DefaultSeaLevelPressure"] = 1013.25;
      object["PressureSource"] = PressureSource;
      object["MaxAbsAltitude"] = maxAbsoluteAltitude;
      object["MinAbsAltitude"] = minAbsoluteAltitude;
      object["MaxRelAltitude"] = maxRelativeAltitude;
      object["MinRelAltitude"] = minRelativeAltitude;
      object["AltDropThreshold"] = altitudeDropThreshold;
      object["AccXThreshold"] = accXThreshold;
      object["AccYThreshold"] = accYThreshold;
      object["AccZThreshold"] = accZThreshold;
      object["TriggerLogic"] = useAndLogic ? "AND" : "OR";
      object["TotalSpace"] = totalSpace;
      object["UsedSpace"] = usedSpace;
      object["SPIFFSTotalSpace"] = spiffsTotal;
      object["SPIFFSUsedSpace"] = spiffsUsed;
      object["Latitude"] = currentLatitude;
      object["Longitude"] = currentLongitude;
      object["SensorModeFlight"] = sensorModeFlight;
      object["Axis Config"] = axisConfig;
      object["SpiffsWarning"] = alreadyWarned;
      object["SensorsCalibrated"] = sensorsCalibrated;
      object["SensorsCalibWarning"] = !sensorsCalibrated;
      object["TriggeredBy"] = uiTriggeredBy;
      // Stuur de enables ook periodiek mee
      object["EnAltDrop"] = enAltDrop;
      object["EnAccX"] = enAccX;
      object["EnAccY"] = enAccY;
      object["EnAccZ"] = enAccZ;

      object["VerticalSpeed"] = verticalSpeed;
      object["FilteredRelAlt"] = filteredRelAlt;
      object["ApogeeDetected"] = apogeeDetected;
      object["ApogeeAltitude"] = apogeeAltitude;
      object["RocketStatus"] = rocketStatus;
      object["ApogeeTriggerEnabled"] = enApogee;
      object["LaunchAccThreshold"] = LAUNCH_ACC_THRESHOLD;
      object["MinApogeeAlt"] = MIN_ALTITUDE_FOR_APOGEE;
      object["NegVsConfirm"] = NEG_VS_CONFIRM;
      object["NegCountConfirm"] = NEG_COUNT_CONFIRM;
      object["GroundSettleMs"] = GROUND_SETTLE_MS;

      serializeJson(doc, jsonString);
      webSocket.broadcastTXT(jsonString);
      previousMillis = nowMillis;
    }
  } else {
    // Visualization mode (unchanged)
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
  delay(40);
}
