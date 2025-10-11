/*
 * Flight Computer Firmware for ESP32 – Version 3.7.0
 *
 * What’s new since 3.6.0 (Major Refactor, Modularization & API Documentation):
 *
 *  - **Dashboard & Visualization to SPIFFS (v3.7.0):**
 *      - All web UI assets (index.html, style.css, script.js, visualization files) are now hosted
 *        directly from SPIFFS instead of being embedded in firmware.
 *      - Benefits:
 *          • Reduced firmware binary size
 *          • Faster dashboard/visualization loading times
 *          • UI can be updated or replaced without reflashing firmware
 *          • Easier maintenance and separation of UI from flight logic
 *
 *  - **File Management Integration (v3.7.0):**
 *      - SPIFFS File Manager can now be used to update dashboard/visualization assets.
 *      - This enables UI-only updates during development or field testing.
 *
 *  - **Other Improvements (v3.7.0):**
 *      - Minor bug fixes and performance optimizations
 *      - Improved consistency in logging warnings when SPIFFS storage is nearly full
 *
 * ┌───────────────────────────────────────────────┐
 * │                  Endpoints Overview           │
 * └───────────────────────────────────────────────┘
 *
 * ───── HTTP REST API & Static Content ──────────────
 *  GET   /                  → Redirects to /index.html, sets flight mode
 *  GET   /visualization     → Redirects to visualization UI, sets visualization mode
 *  GET   /visualization/    → Serves static visualization files from SPIFFS
 *  GET   /files             → Lists files on SD & SPIFFS (JSON)
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
#include <SD_MMC.h>
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
#define LEDS_COUNT 39  // Number of LEDs in the strip
#define LEDS_PIN 17    // GPIO pin for the LED strip data line
#define CHANNEL 0      // PWM channel (if applicable)
//Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);
Adafruit_NeoPixel strip(LEDS_COUNT, LEDS_PIN, NEO_GRB + NEO_KHZ800);

// -----------------------
// Macro Definitions & Global Variables
// -----------------------
#define EEPROM_SIZE 10                   // EEPROM size
#define EEPROM_PRESSURE_ADDR 0           // EEPROM address for storing pressure
const size_t MAX_SPIFFS_USED_KB = 1000;  // Set your preferred limit (in KB)
bool spiffsLoggingAllowed = false;
bool alreadyWarned = false;
bool apiSuccess = false;
bool debugSerial = false;         // Enable/disable serial debug printing
bool sensorModeFlight = true;     // true: Flight sensor mode; false: Visualization mode
int axisConfig = 1;               // 0 = default mapping, 1 = alternative mapping (or more states as needed)
bool sensorsCalibrated = false;   // true when calibrated
bool sensorsCalibWarned = false;  // helper for one-time warning (optional)
bool triggerAbs = true;           // Trigger on both positive and negative acceleration by default
String triggeredBy = "NotTriggered";
String lastTriggeredBy = "NotTriggered";
bool enAltDrop = true;
bool enAccX = false;
bool enAccY = true;
bool enAccZ = true;



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
float currentLatitude = 51.07741431;                              // Anonymized Latitude
float currentLongitude = 5.88510756;                              // Anonymized Longitude
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
float accXThreshold = 15.0;
float accYThreshold = 15.0;
float accZThreshold = 15.0;
bool useAndLogic = false;  // If true, all conditions must meet; if false, any condition triggers

// Baseline for altitude and parachute status variables
float baselineAltitude = 0;
bool baselineCaptured = false;
String parachuteStatus = "unarmed";
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
    StaticJsonDocument<200> doc;
    doc["gyroX"] = gyroX;
    doc["gyroY"] = gyroY;
    doc["gyroZ"] = gyroZ;
    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString;
  } else {
    StaticJsonDocument<200> doc;
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
  StaticJsonDocument<200> doc;
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
//void handleIndex() {
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
  for (uint16_t r = 0; r < rotations; r++) {
    if (direction >= 0) {
      for (int i = 0; i < LEDS_COUNT; i++) {
        // strip.setLedColorData(i, color);
        strip.setPixelColor(i, color);

        strip.show();
        delay(40);
      }
    } else {
      for (int i = LEDS_COUNT - 1; i >= 0; i--) {
        //  strip.setLedColorData(i, color);
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
    setAllLEDs(color);  // update RAM buffer
    strip.show();       // ← actually send the data to the LEDs
    delay(delayms);
    setAllLEDs(0);  // clear buffer
    strip.show();   // ← turn LEDs off
    delay(delayms);
  }
}

// Set LEDs to alternating red and purple pattern
void setWarningPatternLEDs() {
  for (int i = 0; i < LEDS_COUNT; i++) {
    if (i % 2 == 0) {
      strip.setPixelColor(i, redColor);
    } else {
      strip.setPixelColor(i, purpleColor);
    }
  }
  strip.show();
}


// Show the pattern as an animation first, then leave it on
void animateWarningPatternLEDs() {
  // Sequential animation, similar to showLEDColorsSequentially
  for (int i = 0; i < LEDS_COUNT; i++) {
    for (int j = 0; j <= i; j++) {
      if (j % 2 == 0) {
        strip.setPixelColor(j, redColor);
      } else {
        strip.setPixelColor(j, purpleColor);
      }
    }
    // Set all the rest to off
    for (int j = i + 1; j < LEDS_COUNT; j++) {
      strip.setPixelColor(j, 0);
    }
    strip.show();
    delay(30);  // Adjust delay for desired speed
  }
  // Leave all LEDs in pattern
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
    StaticJsonDocument<1024> doc;
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
void checkSpiffsSpaceAndWarn() {
  size_t spiffsUsedKB = SPIFFS.usedBytes() / 1024;

  if (spiffsUsedKB >= MAX_SPIFFS_USED_KB) {
    if (!alreadyWarned) {
      animateWarningPatternLEDs();
      Serial.printf("SPIFFS used: %u KB (limit: %u KB). Logging naar SPIFFS wordt uitgeschakeld.\n", (unsigned)spiffsUsedKB, (unsigned)MAX_SPIFFS_USED_KB);
      alreadyWarned = true;
    }
    spiffsLoggingAllowed = false;
  } else {
    if (alreadyWarned) {
      setAllLEDs(blueColor);
      parachuteStatus = "unarmed";
      Serial.println("SPIFFS heeft weer voldoende ruimte, logging wordt hervat.");
      alreadyWarned = false;
    }
    spiffsLoggingAllowed = true;
  }
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
//  appendFile(SD_MMC, "/log.csv", eventLog);
// if (spiffsLoggingAllowed) {
//    appendFile(SPIFFS, "/log.csv", eventLog);
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

  triggeredBy = "NotTriggered";
  lastTriggeredBy = "NotTriggered";


  if (!sensorsCalibrated) {
    // Flash orange LEDs as a warning
    blinkColor(orangeColor, 15, 100);  // 15 times, 80 ms on/off (adjust as needed)
    Serial.println("Cannot arm: Sensors are not calibrated!");
    return;  // Don't arm the parachute
  }
  Serial.println("Arming parachute...");
  char eventLog[128];
  String eventTimestamp = getTimeStampString();
//  snprintf(eventLog, sizeof(eventLog), "Timestamp: %s, Event: Parachute Armed!\n", eventTimestamp.c_str());
//  appendFile(SD_MMC, "/log.csv", eventLog);
// checkSpiffsSpaceAndWarn();
//  if (spiffsLoggingAllowed) {
//    appendFile(SPIFFS, "/log.csv", eventLog);
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

  if (alreadyWarned) {
    setWarningPatternLEDs();  // Or animateWarningPatternLEDs();
  } else {
    showLEDColorsSequentially(redColor, 1, 3);
    setAllLEDs(redColor);
  }
}

// Calibrates sensors by capturing baseline altitude and accelerometer offsets.
void calibrateSensors() {
  triggeredBy = "NotTriggered";
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
    if (parachuteStatus == "armed") setAllLEDs(redColor);
    if (parachuteStatus == "unarmed") setAllLEDs(blueColor);
    if (parachuteStatus == "released") setAllLEDs(greenColor);
  } else {
    setWarningPatternLEDs();
  }
  // Reset sensor mode to Flight mode after calibration
  sensorsCalibrated = true;
  sensorModeFlight = true;
  Serial.print("Reset sensor mode to Flight");
  Serial.println(sensorModeFlight);
}

// Function to make sure headers are always writen to file
void ensureLogFileHasHeader(fs::FS &fs, const char *path) {
  if (!fs.exists(path)) {
    // Bestaat niet: header toevoegen
    File f = fs.open(path, FILE_WRITE);
    if (f) {
      f.println("Timestamp,BMP Temp,Pressure,Absolute Altitude,Relative Altitude,Altitude Drop,MPU Temp,RawAccX,RawAccY,RawAccZ,AccX_Calib,AccY_Calib,AccZ_Calib,GyroX,GyroY,GyroZ,Parachute Status,Local Pressure,Default Sea-Level Pressure,API Status,Max Abs Altitude,Min Abs Altitude,Max Rel Altitude,Min Rel Altitude,Max Alt Drop,Min Alt Drop,Total Space,Used Space,SPIFFS Total,SPIFFS Used,Latitude,Longitude,AltDropThres,AccXThres,AccYThres,AccZThres,TriggerLogic,AxisConfig,TriggeredBy,EnAltDrop,EnAccX,EnAccY,EnAccZ");
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
        fw.println("Timestamp,BMP Temp,Pressure,Absolute Altitude,Relative Altitude,Altitude Drop,MPU Temp,RawAccX,RawAccY,RawAccZ,AccX_Calib,AccY_Calib,AccZ_Calib,GyroX,GyroY,GyroZ,Parachute Status,Local Pressure,Default Sea-Level Pressure,API Status,Max Abs Altitude,Min Abs Altitude,Max Rel Altitude,Min Rel Altitude,Max Alt Drop,Min Alt Drop,Total Space,Used Space,SPIFFS Total,SPIFFS Used,Latitude,Longitude,AltDropThres,AccXThres,AccYThres,AccZThres,TriggerLogic,AxisConfig,TriggeredBy,EnAltDrop,EnAccX,EnAccY,EnAccZ");
        fw.close();
      }
    } else if (f) {
      f.close();
    }
  }
}



// ---------------------------------------------------------------------------
// WebSocket Event Handler for real-time communication with the client dashboard
// ---------------------------------------------------------------------------
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
        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, payload);
        if (error) {
          Serial.print(F("deserializeJson() failed: "));
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
          if (doc.containsKey("axisConfig")) {
            axisConfig = doc["axisConfig"];
            Serial.print("Axis configuration updated to: ");
            Serial.println(axisConfig);
          }
          const char *command = doc["parachute"];
          Serial.println("Received parachute command from client " + String(num));
          if (String(command) == "Armed" && parachuteStatus != "armed") {
            parachuteArmed();
            Serial.println("Parachute armed command processed.");
          } else if (String(command) == "Released") {
            parachuteRelease();
            Serial.println("Parachute release command processed.");
          } else if (String(command) == "Unarmed") {
            parachuteUnarmed();
            Serial.println("Parachute unarmed command processed.");
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
  triggeredBy = "NotTriggered";
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


// JSON endpoint voor SD & SPIFFS files
void handleListFilesJson() {
  String json = "{";
  json += "\"sd\":" + listFilesJSON(SD_MMC);
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
  if (!SD_MMC.exists(fname)) {
    server.send(404, "text/plain", "File not found on SD");
    return;
  }
  if (SD_MMC.remove(fname)) {
    server.send(200, "text/plain", "Deleted from SD");
  } else {
    server.send(500, "text/plain", "Failed to delete from SD");
  }
}

// -- Handle download: tries SD first, then SPIFFS (expects GET param: name=/filename.txt)
void handleCombinedFileDownload() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing file name");
    return;
  }
  String fname = server.arg("name");
  File file = SD_MMC.open(fname, FILE_READ);
  if (!file) file = SPIFFS.open(fname, FILE_READ);
  if (!file) {
    server.send(404, "text/plain", "File not found on SD or SPIFFS");
    return;
  }

  // --- Fix: Add correct filename to Content-Disposition ---
  String outFilename = fname;
  int slash = outFilename.lastIndexOf('/');
  if (slash != -1) outFilename = outFilename.substring(slash + 1);

  server.sendHeader("Content-Disposition", "attachment; filename=\"" + outFilename + "\"");
  server.streamFile(file, "application/octet-stream");
  file.close();
}

// -- Handle upload to SPIFFS
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

// -- Handle upload to SD
void handleFileUploadSD() {
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
}

// -- Optionally: Delete file from both SD and SPIFFS
void handleCombinedFileDelete() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing file name");
    return;
  }
  String fname = server.arg("name");
  bool deleted = false;
  if (SD_MMC.exists(fname)) deleted |= SD_MMC.remove(fname);
  if (SPIFFS.exists(fname)) deleted |= SPIFFS.remove(fname);
  if (deleted) {
    server.send(200, "text/plain", "Deleted from SD or SPIFFS");
  } else {
    server.send(404, "text/plain", "File not found on SD or SPIFFS");
  }
}






void handleSpiffsUpload() {
  HTTPUpload &upload = server.upload();
  static String targetFolder;
  static File uploadFile;
  if (upload.status == UPLOAD_FILE_START) {
    // Get the folder from POST body, fallback to root if empty
    targetFolder = server.arg("folder");
    if (targetFolder.length() == 0) targetFolder = "/";
    if (!targetFolder.startsWith("/")) targetFolder = "/" + targetFolder;
    if (!targetFolder.endsWith("/")) targetFolder += "/";
    String fname = upload.filename;
    String fullPath = targetFolder + fname;

    // Create folder if not exist
    if (!SPIFFS.exists(targetFolder.c_str())) SPIFFS.mkdir(targetFolder.c_str());
    uploadFile = SPIFFS.open(fullPath, FILE_WRITE);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) uploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile) uploadFile.close();
  }
}

// -----------------------
// Setup Function: Initializes hardware, network, sensors, web server, and endpoints
// -----------------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting setup..."));

  // Mount SPIFFS for file serving and upload functionality.
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
  } else {
    Serial.println("SPIFFS mounted successfully");
  }

  ensureLogFileHasHeader(SD_MMC, "/log.csv");
  ensureLogFileHasHeader(SPIFFS, "/log.csv");

  if (!SPIFFS.exists("/log.csv")) {
    File spiffsFile = SPIFFS.open("/log.csv", FILE_WRITE);
    if (spiffsFile) {
      spiffsFile.println("Timestamp,BMP Temp,Pressure,Absolute Altitude,Relative Altitude,Altitude Drop,MPU Temp,RawAccX,RawAccY,RawAccZ,AccX_Calib,AccY_Calib,AccZ_Calib,GyroX,GyroY,GyroZ,Parachute Status,Local Pressure,Default Sea-Level Pressure,API Status,Max Abs Altitude,Min Abs Altitude,Max Rel Altitude,Min Rel Altitude,Max Alt Drop,Min Alt Drop,Total Space,Used Space,SPIFFS Total,SPIFFS Used,Latitude,Longitude,AltDropThres,AccXThres,AccYThres,AccZThres,TriggerLogic,AxisConfig,TriggeredBy,EnAltDrop,EnAccX,EnAccY,EnAccZ");
      spiffsFile.close();
    }
  }

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

  // 1) Load your storedPressure from EEPROM up front:
  EEPROM.begin(EEPROM_SIZE);
  float storedPressure;
  EEPROM.get(EEPROM_PRESSURE_ADDR, storedPressure);

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
  Serial.print("Initializing SD card...");
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  if (!SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5)) {
    Serial.println("Card Mount Failed");
  } else {
    Serial.println("SD Card initialized.");
    String oldLogFile = "/log.csv";
    String timestamp = getTimeStampString();
    timestamp.replace(":", "");
    timestamp.replace(" ", "_");
    String newLogFile = "/" + timestamp + ".csv";
    if (SD_MMC.exists(newLogFile.c_str())) {
      SD_MMC.remove(newLogFile.c_str());
    }
    if (SD_MMC.exists(oldLogFile.c_str())) {
      if (SD_MMC.rename(oldLogFile.c_str(), newLogFile.c_str())) {
        Serial.println("Previous log file renamed to " + newLogFile);
      } else {
        Serial.println("Failed to rename log file " + oldLogFile);
      }
    } else {
      Serial.println("No previous log file found.");
    }
    File file = SD_MMC.open(oldLogFile.c_str(), FILE_WRITE);
    if (file) {
      file.println("Timestamp,BMP Temp,Pressure,Absolute Altitude,Relative Altitude,Altitude Drop,MPU Temp,RawAccX,RawAccY,RawAccZ,AccX_Calib,AccY_Calib,AccZ_Calib,GyroX,GyroY,GyroZ,Parachute Status,Local Pressure,Default Sea-Level Pressure,API Status,Max Abs Altitude,Min Abs Altitude,Max Rel Altitude,Min Rel Altitude,Max Alt Drop,Min Alt Drop,Total Space,Used Space,SPIFFS Total,SPIFFS Used,Latitude,Longitude,AltDropThres,AccXThres,AccYThres,AccZThres,TriggerLogic,AxisConfig,TriggeredBy,EnAltDrop,EnAccX,EnAccY,EnAccZ");
      file.close();
    }
  }
  totalSpace = SD_MMC.totalBytes() / (1024 * 1024);
  usedSpace = SD_MMC.usedBytes() / (1024 * 1024);
  Serial.printf("Total space: %lluMB\n", totalSpace);
  Serial.printf("Used space: %lluMB\n", usedSpace);

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
    if (!apiPressureUpdated) updatePressureFromAPI();
  } else {
    apiPressureUpdated = false;
  }

  // Update SD and SPIFFS stats
  totalSpace = SD_MMC.totalBytes() / (1024 * 1024);
  usedSpace = SD_MMC.usedBytes() / (1024 * 1024);
  size_t spiffsTotal = SPIFFS.totalBytes() / 1024;
  size_t spiffsUsed = SPIFFS.usedBytes() / 1024;

  // Sensor reads
  float bmpTemp = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0F;
  float absoluteAltitude = bmp.readAltitude(getLocalSeaLevelPressure());
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

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
  if (absoluteAltitude > maxAbsoluteAltitude) maxAbsoluteAltitude = absoluteAltitude;
  if (absoluteAltitude < minAbsoluteAltitude) minAbsoluteAltitude = absoluteAltitude;
  if (relativeAltitude > maxRelativeAltitude) maxRelativeAltitude = relativeAltitude;
  if (relativeAltitude < minRelativeAltitude) minRelativeAltitude = relativeAltitude;
  float altitudeDrop = (parachuteStatus == "armed") ? armedMaxRelativeAltitude - relativeAltitude : maxRelativeAltitude - relativeAltitude;
  if (altitudeDrop > maxAltitudeDrop) maxAltitudeDrop = altitudeDrop;
  if (altitudeDrop < minAltitudeDrop) minAltitudeDrop = altitudeDrop;

  // ----------- Check trigger conditions, track TriggeredBy -----------
  triggeredBy = "NotTriggered";
  if (parachuteStatus == "armed") {
    // Evaluate only enabled triggers
    bool triggerAlt = enAltDrop ? (altitudeDrop >= altitudeDropThreshold) : false;
    bool triggerAccX = enAccX ? (triggerAbs ? (fabs(relAccX) >= accXThreshold) : (relAccX >= accXThreshold)) : false;
    bool triggerAccY = enAccY ? (triggerAbs ? (fabs(relAccY) >= accYThreshold) : (relAccY >= accYThreshold)) : false;
    bool triggerAccZ = enAccZ ? (triggerAbs ? (fabs(relAccZ) >= accZThreshold) : (relAccZ >= accZThreshold)) : false;

    // Build triggers array for generic AND/OR logic
    bool triggers[] = { triggerAlt, triggerAccX, triggerAccY, triggerAccZ };

    // Check which are enabled for logic
    bool enabled[] = { enAltDrop, enAccX, enAccY, enAccZ };
    bool anyEnabled = enAltDrop || enAccX || enAccY || enAccZ;

    bool triggerCondition = false;
    if (anyEnabled) {
      if (useAndLogic) {
        triggerCondition = true;
        for (int i = 0; i < 4; i++) {
          if (enabled[i] && !triggers[i]) triggerCondition = false;
        }
      } else {
        for (int i = 0; i < 4; i++) {
          if (enabled[i] && triggers[i]) triggerCondition = true;
        }
      }
    }

    // Track what triggered
    if (triggerCondition) {
      String triggersList = "";
      if (triggerAlt) triggersList += "Threshold Altitude Drop,";
      if (triggerAccX) triggersList += "Threshold AccX,";
      if (triggerAccY) triggersList += "Threshold AccY,";
      if (triggerAccZ) triggersList += "Threshold AccZ,";
      if (triggersList.length() > 0) triggersList.remove(triggersList.length() - 1);
      triggeredBy = triggersList;
      lastTriggeredBy = triggersList;
      parachuteRelease();
    } else {
      triggeredBy = lastTriggeredBy;
    }
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
    Serial.println(triggeredBy);
    Serial.println("--------------------");
  }

  // --- LOG FORMAT (add all extra columns) ---
  char dataString[700];
  String currentTimestamp = getTimeStampString();

snprintf(dataString, sizeof(dataString),
    "%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%.2f,1013.25,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%llu,%llu,%u,%u,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%s,%d,\"%s\",%d,%d,%d,%d\n",
    currentTimestamp.c_str(),      // Timestamp (zonder quotes)
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
    triggeredBy.c_str(),   // ← alleen deze tussen dubbele quotes!
    enAltDrop ? 1 : 0,
    enAccX ? 1 : 0,
    enAccY ? 1 : 0,
    enAccZ ? 1 : 0
);


  checkSpiffsSpaceAndWarn();
  appendFile(SD_MMC, "/log.csv", dataString);
  if ((parachuteStatus == "armed" || parachuteStatus == "released") && spiffsLoggingAllowed) {
    appendFile(SPIFFS, "/log.csv", dataString);
  }

  // --- WebSocket Output (also includes new fields) ---
  if (sensorModeFlight) {
    static unsigned long previousMillis = 0;
    int interval = 40;
    unsigned long nowMillis = millis();
    if ((nowMillis - previousMillis) > interval) {
      String jsonString = "";
      StaticJsonDocument<800> doc;
      JsonObject object = doc.to<JsonObject>();
      object["AbsoluteAltitude"] = absoluteAltitude;
      object["RelativeAltitude"] = relativeAltitude;
      object["AltitudeDrop"] = altitudeDrop;
      object["MaxAltDrop"] = maxAltitudeDrop;
      object["MinAltDrop"] = minAltitudeDrop;
      object["BMP280Temp"] = bmpTemp;
      object["BMP280Pressure"] = pressure;
      object["MPU6050Temp"] = temp.temperature;
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
      object["TriggeredBy"] = triggeredBy;
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
