/*
 * Flight Computer Firmware for ESP32 (Optimized & Fully Commented)
 *
 * Key changes in this version:
 * 1. When the parachute is armed, the code resets:
 *    - Relative altitude metrics (maxRelativeAltitude, minRelativeAltitude)
 *    - Altitude drop metrics (maxAltitudeDrop, minAltitudeDrop)
 *    - lastAltitude (so that future measurements start from zero)
 *
 * 2. The following configuration variables are now broadcasted, logged, and shown on the web page:
 *    - logBMP280Data, logMPU6050Data, logSpaceUsage, showCameraOutput, showTimelapseOutput,
 *      showSensorInitLog, timelapseInterval, SD_LOG_INTERVAL, WEB_UPDATE_INTERVAL, enableOTA,
 *      altitudeDropThreshold, and the loop execution time (time used per loop).
 *
 * 3. The HTML page has been updated to include rows for these configuration values.
 *
 * 4. The API call for pressure is executed once after a successful WiFi connection in setup,
 *    and once each time the parachute is armed.
 *
 * 5. OTA update webserver can be enabled or disabled via the "enableOTA" flag.
 */

// -----------------------
// Include Libraries and Drivers
// -----------------------
#include "esp_camera.h"            // Camera driver

// -----------------------
// I2C Pin Definitions
// -----------------------
#define SDA_1 42                  // I2C bus for sensors (I2C_NUM_1)
#define SCL_1 37
#define SDA_2 4                   // I2C bus for camera (overridden SCCB pins)
#define SCL_2 5

// -----------------------
// Override Camera SCCB Pins Before Including camera_pins.h
// -----------------------
#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"          // Uses our overridden camera pins

// -----------------------
// Library Inclusions for Sensors, SD, WiFi, OTA, etc.
// -----------------------
// Rename sensor_t to avoid conflicts with camera driver
#define sensor_t adafruit_sensor_t  
#include <Adafruit_MPU6050.h>       // MPU6050 sensor library
#include <Adafruit_BMP280.h>        // BMP280 sensor library
#undef sensor_t

#include <Wire.h>                   // I²C communication library
#include <Adafruit_Sensor.h>        // Unified sensor library
#include <SD_MMC.h>                 // SD card interface for ESP32-S3
#include "sd_read_write.h"          // Helper functions for SD card operations
#include <ESP32Servo.h>             // Servo control library for ESP32
#include <WiFi.h>                   // WiFi connectivity
#include <WiFiUdp.h>                // UDP library (for NTP)
#include <NTPClient.h>              // NTP client library
#include <WebServer.h>              // Web server library
#include <WebSocketsServer.h>       // WebSockets for real-time data transmission
#include <ArduinoJson.h>            // JSON library for building JSON objects
#include <HTTPClient.h>             // HTTP client library for API calls
#include <Arduino_ESP32_OTA.h>      // OTA update library for ESP32 (HTTP OTA)
#include <HTTPUpdateServer.h>       // HTTP update server library
#include <ESPmDNS.h>                // mDNS for network service discovery
#include <EEPROM.h>                 // EEPROM library for non-volatile storage

// -----------------------
// Macro Definitions
// -----------------------
#define EEPROM_SIZE 10                    // EEPROM storage size in bytes
#define EEPROM_PRESSURE_ADDR 0            // EEPROM address for pressure storage

// -----------------------
// Global Control Variables for Logging and Output
// -----------------------
bool enableSensorSerialLog = true;        // Master toggle for serial sensor log output

// Toggle flags for various logs and outputs:
bool logBMP280Data        = false;         // If true, BMP280 sensor readings (temp, pressure, altitude) are logged to serial.
bool logMPU6050Data       = false;         // If true, MPU6050 sensor readings (accelerometer, gyro, temp) are logged.
bool logSpaceUsage        = true;          // If true, SD card space usage is printed to serial.
bool showCameraOutput     = true;          // If true, camera initialization and capture events are logged.
bool showTimelapseOutput  = true;          // If true, timelapse image capture events are logged.
bool showSensorInitLog    = true;          // If true, sensor initialization messages are logged.

// -----------------------
// Timelapse and Task Interval Settings
// -----------------------
bool timelapseActive = false;             // Flag indicating if timelapse capture is active.
const unsigned long timelapseInterval = 500; // Interval (ms) between timelapse image captures.

// Configurable intervals for lower priority tasks:
const unsigned long SD_LOG_INTERVAL       = 1000;  // Interval (ms) for logging data to SD card.
const unsigned long WEB_UPDATE_INTERVAL     = 200;   // Interval (ms) for updating web clients (WebSocket).

// -----------------------
// OTA Webserver Control
// -----------------------
bool enableOTA = false;  // Set to true to run the OTA update webserver; false to disable.

// -----------------------
// Other Global Flags
// -----------------------
bool cameraEnabled = true;              // Controls whether the camera is used

// -----------------------
// Global Variables for SD Card Space
// -----------------------
uint64_t totalSpace = 0;                // Total SD card space in MB.
uint64_t usedSpace = 0;                 // Used SD card space in MB.

// -----------------------
// WiFi and Access Point Credentials
// -----------------------
const char* ssid = "TDGC-Rocket";
const char* wifiPassword = "Rocket2022!";
const char* apSSID = "RocketAP";
const char* apPassword = "Rocket2022!";

// -----------------------
// OpenWeatherMap API Details
// -----------------------
const char* openWeatherMapApiKey = "API_KEY";  // Replace with your actual API key.
const char* lat = "latitude";                  // Replace with your latitude.
const char* lon = "longitude";                   // Replace with your longitude.
const char* owmEndpoint = "https://api.openweathermap.org/data/3.0/onecall";

// -----------------------
// NTP Client Setup
// -----------------------
#define UTC_OFFSET_IN_SECONDS 3600            // UTC offset (in seconds) for your timezone.
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", UTC_OFFSET_IN_SECONDS);
unsigned long lastSuccessfulNTP = 0;
unsigned long lastSyncMillis = 0;

// -----------------------
// Parachute and Altitude Variables
// -----------------------
const float altitudeDropThreshold = 0.8;      // Altitude drop threshold (in meters) for triggering parachute release.
float lastAltitude = 0;                       // Last measured relative altitude.
String parachuteStatus = "unarmed";           // Parachute status: "unarmed", "armed", or "released".
float baselineAltitude = 0;                   // Baseline altitude captured upon arming.
bool baselineCaptured = false;                // Flag indicating if baseline altitude has been captured.
const float defaultSeaLevelPressure = 1026.0; // Default sea-level pressure (hPa).

// -----------------------
// Global Variables for Pressure and Altitude Tracking
// -----------------------
String PressureSource = "";           // Indicates where pressure data came from.
float lastLocalPressure = 1026.0;       // Default sea-level pressure (hPa).
bool apiPressureUpdated = false;      // Flag if pressure API call succeeded.

// -----------------------
// Global Variables for Altitude Tracking
// -----------------------
float maxAbsoluteAltitude = -1000000.0;
float minAbsoluteAltitude =  1000000.0;
float maxRelativeAltitude = -1000000.0;
float minRelativeAltitude =  1000000.0;
float maxAltitudeDrop = -1000000.0;
float minAltitudeDrop =  1000000.0;

// -----------------------
// Sensor Availability Flags
// -----------------------
bool bmpFound = false;                        // True if BMP280 sensor is found.
bool mpuFound = false;                        // True if MPU6050 sensor is found.

// -----------------------
// Web Server HTML Content
// -----------------------
// The HTML page displays flight data and control buttons in two tables:
// Left table: Sensor Data (including Max/Min Altitude Drop)
// Right table: System Status / Configuration values.
String webpage ="<!DOCTYPE html><html><head> <meta name='viewport' content='width=device-width,initial-scale=1'> <title>Flight Computer</title> <style> body { background-color: #EEEEEE; font-family: Arial, sans-serif; color: #003366; margin: 0; padding: 20px; } h1 { text-align: center; margin-bottom: 20px; } .data-table { margin: 0 auto; border-collapse: collapse; width: 90%; max-width: 600px; background-color: #FFF; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); } .data-table th, .data-table td { padding: 12px 15px; border: 1px solid #CCC; text-align: left; } .data-table th { background-color: #003366; color: #FFF; } .data-table tr:nth-child(even) { background-color: #F9F9F9; } .button-container { text-align: center; margin-top: 20px; } button { background-color: #003366; color: #FFF; border: none; padding: 10px 20px; font-size: 16px; cursor: pointer; margin: 5px; } button:hover { background-color: #0055AA; } </style></head><body> <h1>Flight Information</h1> <table class='data-table'> <tr> <th>Parameter</th> <th>Value</th> </tr> <tr> <td>Absolute Altitude</td> <td id='AbsoluteAltitude'>-</td> </tr> <tr> <td>Relative Altitude</td> <td id='RelativeAltitude'>-</td> </tr> <tr> <td>Altitude Drop</td> <td id='AltitudeDrop'>-</td> </tr> <tr> <td>BMP280 Temp</td> <td id='BMP280Temp'>-</td> </tr> <tr> <td>BMP280 Pressure</td> <td id='BMP280Pressure'>-</td> </tr> <tr> <td>MPU6050 Temp</td> <td id='MPU6050Temp'>-</td> </tr> <tr> <td>Accelerometer</td> <td id='Accelerometer'>-</td> </tr> <tr> <td>Gyroscope</td> <td id='Gyroscope'>-</td> </tr> <tr> <td>Parachute Status</td> <td id='ParachuteStatus'>-</td> </tr> <tr> <td>Local Pressure</td> <td id='LocalPressure'>-</td> </tr> <tr> <td>Default Sea-Level Pressure</td> <td id='DefaultSeaLevelPressure'>-</td> </tr> <tr> <td>Pressure Source</td> <td id='PressureSource'>-</td> </tr> <tr> <td>Max Abs Altitude</td> <td id='MaxAbsAltitude'>-</td> </tr> <tr> <td>Min Abs Altitude</td> <td id='MinAbsAltitude'>-</td> </tr> <tr> <td>Max Rel Altitude</td> <td id='MaxRelAltitude'>-</td> </tr> <tr> <td>Min Rel Altitude</td> <td id='MinRelAltitude'>-</td> </tr> <tr> <td>Total Space (MB)</td> <td id='TotalSpace'>-</td> </tr> <tr> <td>Used Space (MB)</td> <td id='UsedSpace'>-</td> </tr> <tr> <td>Timelapse Status</td> <td id='TimelapseStatus'>-</td> </tr> <tr> <td>logBMP280Data</td> <td id='logBMP280Data'>-</td> </tr> <tr> <td>logMPU6050Data</td> <td id='logMPU6050Data'>-</td> </tr> <tr> <td>logSpaceUsage</td> <td id='logSpaceUsage'>-</td> </tr> <tr> <td>showCameraOutput</td> <td id='showCameraOutput'>-</td> </tr> <tr> <td>showTimelapseOutput</td> <td id='showTimelapseOutput'>-</td> </tr> <tr> <td>showSensorInitLog</td> <td id='showSensorInitLog'>-</td> </tr> <tr> <td>timelapseInterval (ms)</td> <td id='timelapseInterval'>-</td> </tr> <tr> <td>SD_LOG_INTERVAL (ms)</td> <td id='SD_LOG_INTERVAL'>-</td> </tr> <tr> <td>WEB_UPDATE_INTERVAL (ms)</td> <td id='WEB_UPDATE_INTERVAL'>-</td> </tr> <tr> <td>enableOTA</td> <td id='enableOTA'>-</td> </tr> <tr> <td>altitudeDropThreshold (m)</td> <td id='altitudeDropThreshold'>-</td> </tr> <tr> <td>Loop Execution Time (ms)</td> <td id='loopExecutionTime'>-</td> </tr> </table> <div class='button-container'><button type='button' id='BTN_SEND_BACK'>Arm Parachute</button><button type='button' id='BTN_START_TIMELAPSE'>Start Picture Timelapse</button><button type='button' id='BTN_STOP_TIMELAPSE'>Stop Picture Timelapse</button></div> <script> var Socket; document.getElementById('BTN_SEND_BACK').addEventListener('click', button_send_back); document.getElementById('BTN_START_TIMELAPSE').addEventListener('click', button_start_timelapse); document.getElementById('BTN_STOP_TIMELAPSE').addEventListener('click', button_stop_timelapse); function init() { Socket = new WebSocket('ws://' + window.location.hostname + ':81/'); Socket.onmessage = function(event) { processCommand(event); }; } function button_send_back() { var msg = { parachute: 'Armed' }; Socket.send(JSON.stringify(msg)); } function button_start_timelapse() { var msg = { timelapse: 'start' }; Socket.send(JSON.stringify(msg)); } function button_stop_timelapse() { var msg = { timelapse: 'stop' }; Socket.send(JSON.stringify(msg)); } function processCommand(event) { var obj = JSON.parse(event.data); document.getElementById('AbsoluteAltitude').innerHTML = obj.AbsoluteAltitude || '-'; document.getElementById('RelativeAltitude').innerHTML = obj.RelativeAltitude || '-'; document.getElementById('AltitudeDrop').innerHTML = obj.AltitudeDrop || '-'; document.getElementById('BMP280Temp').innerHTML = obj.BMP280Temp || '-'; document.getElementById('BMP280Pressure').innerHTML = obj.BMP280Pressure || '-'; document.getElementById('MPU6050Temp').innerHTML = obj.MPU6050Temp || '-'; document.getElementById('Accelerometer').innerHTML = obj.Accelerometer || '-'; document.getElementById('Gyroscope').innerHTML = obj.Gyroscope || '-'; document.getElementById('ParachuteStatus').innerHTML = obj.ParachuteStatus || '-'; document.getElementById('LocalPressure').innerHTML = obj.LocalPressure || '-'; document.getElementById('DefaultSeaLevelPressure').innerHTML = obj.DefaultSeaLevelPressure || '-'; document.getElementById('PressureSource').innerHTML = obj.PressureSource || '-'; document.getElementById('MaxAbsAltitude').innerHTML = obj.MaxAbsAltitude || '-'; document.getElementById('MinAbsAltitude').innerHTML = obj.MinAbsAltitude || '-'; document.getElementById('MaxRelAltitude').innerHTML = obj.MaxRelAltitude || '-'; document.getElementById('MinRelAltitude').innerHTML = obj.MinRelAltitude || '-'; document.getElementById('TotalSpace').innerHTML = obj.TotalSpace || '-'; document.getElementById('UsedSpace').innerHTML = obj.UsedSpace || '-'; document.getElementById('TimelapseStatus').innerHTML = obj.TimelapseStatus || '-'; document.getElementById('logBMP280Data').innerHTML = obj.logBMP280Data || '-'; document.getElementById('logMPU6050Data').innerHTML = obj.logMPU6050Data || '-'; document.getElementById('logSpaceUsage').innerHTML = obj.logSpaceUsage || '-'; document.getElementById('showCameraOutput').innerHTML = obj.showCameraOutput || '-'; document.getElementById('showTimelapseOutput').innerHTML = obj.showTimelapseOutput || '-'; document.getElementById('showSensorInitLog').innerHTML = obj.showSensorInitLog || '-'; document.getElementById('timelapseInterval').innerHTML = obj.timelapseInterval || '-'; document.getElementById('SD_LOG_INTERVAL').innerHTML = obj.SD_LOG_INTERVAL || '-'; document.getElementById('WEB_UPDATE_INTERVAL').innerHTML = obj.WEB_UPDATE_INTERVAL || '-'; document.getElementById('enableOTA').innerHTML = obj.enableOTA || '-'; document.getElementById('altitudeDropThreshold').innerHTML = obj.altitudeDropThreshold || '-'; document.getElementById('loopExecutionTime').innerHTML = obj.loopExecutionTime || '-'; } window.onload = function(event) { init(); } </script></body></html>";

// -----------------------
// Web Server and WebSocket Instances
// -----------------------
WebServer server(80);                   // HTTP server on port 80
WebSocketsServer webSocket = WebSocketsServer(81); // WebSocket server on port 81
HTTPUpdateServer httpUpdater;           // OTA update server instance

// -----------------------
// SD_MMC and I2C Pin Definitions for SD and Sensors
// -----------------------
#define SD_MMC_CMD 38
#define SD_MMC_CLK 39
#define SD_MMC_D0  40

// -----------------------
// Sensor Instances
// -----------------------
Adafruit_BMP280 bmp;   // BMP280 sensor instance (I2C_NUM_1)
Adafruit_MPU6050 mpu;  // MPU6050 sensor instance

// -----------------------
// Servo Configuration for Parachute Deployment
// -----------------------
Servo parachuteservo;
int servoPin = 14;     // Pin connected to the parachute servo

// -----------------------
// Function: getTimeStampString()
// -----------------------
// Returns a formatted timestamp string using NTP time (if available) or from the NTP client.
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

// -----------------------
// Function: getLocalSeaLevelPressure()
// -----------------------
// Returns the locally stored sea-level pressure.
float getLocalSeaLevelPressure() {
  return lastLocalPressure;
}

// -----------------------
// Function: updatePressureFromAPI()
// -----------------------
// Calls the OpenWeatherMap API to update the local pressure.
// If the API call fails, it falls back to EEPROM or a default pressure.
void updatePressureFromAPI() {
  float localPressure = defaultSeaLevelPressure;
  HTTPClient http;
  String url = String(owmEndpoint) + "?lat=" + lat + "&lon=" + lon + "&exclude=minutely,hourly,daily,alerts&appid=" + openWeatherMapApiKey;
  http.begin(url);
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (!error) {
      if (doc.containsKey("cod")) {
        int apiErrorCode = doc["cod"];
        String apiErrorMessage = doc["message"].as<String>();
        Serial.printf("API Error %d: %s\n", apiErrorCode, apiErrorMessage.c_str());
        PressureSource = "API Error";
      } else {
        localPressure = doc["current"]["pressure"] | defaultSeaLevelPressure;
        PressureSource = "API OpenWeather";
        EEPROM.put(EEPROM_PRESSURE_ADDR, localPressure);
        EEPROM.commit();
        Serial.println("API success. Pressure saved to EEPROM.");
      }
    } else {
      Serial.println("JSON parse error.");
      PressureSource = "API Error";
    }
  } else {
    Serial.printf("HTTP Error: %d (%s)\n", httpCode, http.errorToString(httpCode).c_str());
    PressureSource = "HTTP Error";
  }
  http.end();

  // Fallback to EEPROM if API call fails
  if (PressureSource == "API Error" || PressureSource == "HTTP Error") {
    float storedPressure;
    EEPROM.get(EEPROM_PRESSURE_ADDR, storedPressure);
    if (storedPressure > 500.0 && storedPressure < 1100.0) {
      localPressure = storedPressure;
      PressureSource = "EEPROM";
      Serial.printf("Using stored EEPROM pressure: %.2f hPa\n", localPressure);
    } else {
      localPressure = defaultSeaLevelPressure;
      PressureSource = "Default Sea-Level";
      Serial.println("Using default sea-level pressure.");
    }
  }
  lastLocalPressure = localPressure;
  apiPressureUpdated = true;
}

// -----------------------
// Function: rotateLogFile()
// -----------------------
// Log rotation is handled during setup.
void rotateLogFile() {
  // Handled in setup()
}

// -----------------------
// Function: parachuteRelease()
// -----------------------
// Immediately releases the parachute by actuating the servo without delays.
// This is a high-priority action.
void parachuteRelease() {
  Serial.println("Altitude drop detected! Releasing parachute...");
  char eventLog[128];
  String eventTimestamp = getTimeStampString();
  snprintf(eventLog, sizeof(eventLog), "Timestamp: %s, Event: Parachute Released!\n", eventTimestamp.c_str());
  appendFile(SD_MMC, "/log.txt", eventLog);
  // Actuate servo instantly.
  parachuteservo.write(180);
  delay(50);
  parachuteservo.write(0);
  parachuteStatus = "released";
}

// -----------------------
// Function: parachuteArmed()
// -----------------------
// Arms the parachute. It logs the event, updates pressure (if WiFi is connected),
// captures the baseline altitude, and resets altitude metrics (including max/min altitude drop).
void parachuteArmed() {
  Serial.println("Arming parachute...");
  char eventLog[128];
  String eventTimestamp = getTimeStampString();
  snprintf(eventLog, sizeof(eventLog), "Timestamp: %s, Event: Parachute Armed!\n", eventTimestamp.c_str());
  appendFile(SD_MMC, "/log.txt", eventLog);

  // Update pressure upon arming if WiFi is connected.
  if (WiFi.status() == WL_CONNECTED) {
    updatePressureFromAPI();
  }
  
  // Capture baseline altitude (if available)
  if (!baselineCaptured) {
    if (bmpFound) {
      bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                      Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
                      Adafruit_BMP280::STANDBY_MS_500);
      baselineAltitude = bmp.readAltitude(getLocalSeaLevelPressure());
    } else {
      baselineAltitude = 0;
    }
    baselineCaptured = true;
    Serial.print("Baseline altitude captured: ");
    Serial.print(baselineAltitude);
    Serial.println(" m");
  }
  
  // Reset relative altitude and altitude drop metrics.
  maxRelativeAltitude = 0;
  minRelativeAltitude = 0;
  maxAltitudeDrop = 0;
  minAltitudeDrop = 0;
  lastAltitude = 0;
  
  // Actuate servo to arm position without delays.
  parachuteStatus = "armed";
  parachuteservo.write(0);
  delay(50);
  parachuteservo.write(180);
  
}

// -----------------------
// Function: webSocketEvent()
// -----------------------
// Handles incoming WebSocket events and commands.
void webSocketEvent(byte num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("Client " + String(num) + " disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("Client " + String(num) + " connected");
      break;
    case WStype_TEXT: {
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, payload);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }
      // Process parachute command.
      if (doc.containsKey("parachute")) {
        const char* g_parachute = doc["parachute"];
        Serial.println("Received parachute command from client " + String(num));
        Serial.println("Parachute: " + String(g_parachute));
        if (parachuteStatus != "armed") {
          if (String(g_parachute) == "Armed") {
            parachuteArmed();
            Serial.println("Parachute armed command processed.");
          }
        }
      }
      // Process timelapse command.
      if (doc.containsKey("timelapse")) {
        String tlCommand = doc["timelapse"];
        if (tlCommand == "start") {
          timelapseActive = true;
          if (showTimelapseOutput) {
            Serial.println("Timelapse started.");
          }
        } else if (tlCommand == "stop") {
          timelapseActive = false;
          if (showTimelapseOutput) {
            Serial.println("Timelapse stopped.");
          }
        }
      }
      Serial.println("");
      break;
    }
  }
}

// -----------------------
// Function: cameraSetup()
// -----------------------
// Configures and initializes the camera. Adjusts settings based on PSRAM availability.
int cameraSetup(void) {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // JPEG for streaming.
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    if (showCameraOutput) {
      Serial.printf("Camera init failed with error 0x%x", err);
    }
    return 0;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1);       // Flip image vertically.
  s->set_brightness(s, 1);  // Slightly increase brightness.
  s->set_saturation(s, 0);  // Lower saturation.

  if (showCameraOutput) {
    Serial.println("Camera configuration complete!");
  }
  return 1;
}

// -----------------------
// Function: captureAndSavePicture()
// -----------------------
// Captures a picture with the camera and saves it to the SD card with a unique filename.
String captureAndSavePicture() {
  if (!cameraEnabled) {
    if (showCameraOutput) {
      Serial.println("Camera disabled; skipping capture.");
    }
    return "";
  }
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == NULL) {
    if (showCameraOutput) {
      Serial.println("Camera capture failed.");
    }
    return "";
  }
  
  int photo_index = readFileNum(SD_MMC, "/camera");
  if (photo_index != -1) {
    String path = "/camera/" + String(photo_index) + ".jpg";
    writejpg(SD_MMC, path.c_str(), fb->buf, fb->len);
    if (showCameraOutput) {
      Serial.print("Picture saved: ");
      Serial.println(path);
    }
    esp_camera_fb_return(fb);
    return path;
  } else {
    if (showCameraOutput) {
      Serial.println("Error reading photo index.");
    }
    esp_camera_fb_return(fb);
    return "";
  }
}

// -----------------------
// Function: setup()
// -----------------------
// Runs once at boot. Initializes WiFi, NTP, SD card, sensors, OTA (if enabled), and camera.
// Also rotates logs and sets up web server routes.
void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting setup..."));

  // Initialize EEPROM for pressure data storage.
  EEPROM.begin(EEPROM_SIZE);
  float storedPressure;
  EEPROM.get(EEPROM_PRESSURE_ADDR, storedPressure);
  if (storedPressure > 500.0 && storedPressure < 1100.0) {
    lastLocalPressure = storedPressure;
    PressureSource = "EEPROM Memory";
  } else {
    lastLocalPressure = defaultSeaLevelPressure;
    PressureSource = "Default Sea-Level Pressure";
  }

  // Initialize WiFi as a station.
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
    // If connection fails, start as an Access Point.
    Serial.println("\nWiFi connection failed. Starting Access Point...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apSSID, apPassword);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
  }
  
  // Call the pressure API once after a successful WiFi connection.
  if (WiFi.status() == WL_CONNECTED) {
    updatePressureFromAPI();
  }
  
  // Initialize NTP client for time synchronization.
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
  
  // Initialize SD card and perform log rotation.
  Serial.print("Initializing SD card...");
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  if (!SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5)) {
    Serial.println("Card Mount Failed");
  } else {
    Serial.println("SD Card initialized.");
    createDir(SD_MMC, "/camera");
    listDir(SD_MMC, "/camera", 0);

    String oldLogFile = "/log.txt";
    String timestamp = getTimeStampString();
    timestamp.replace(":", "");
    timestamp.replace(" ", "_");
    String newLogFile = "/log_" + timestamp + ".txt";
    
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
      file.println("New log session started: " + getTimeStampString());
      file.close();
    }
  }
  
  // Calculate SD card space usage.
  totalSpace = SD_MMC.totalBytes() / (1024 * 1024);
  usedSpace = SD_MMC.usedBytes() / (1024 * 1024);
  Serial.printf("Total space: %lluMB\n", totalSpace);
  Serial.printf("Used space: %lluMB\n", usedSpace);
  
  // Initialize I2C bus for sensors.
  Wire.begin(42, 37);
  
  // Initialize MPU6050 sensor.
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
  
  // Initialize BMP280 sensor.
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
  
  // Configure MPU6050 sensor ranges and filters.
  if (mpuFound) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  
  // Setup OTA update server and mDNS if OTA is enabled.
  if (enableOTA) {
    httpUpdater.setup(&server);
    if (MDNS.begin("esp32-webupdate")) {
      Serial.println("MDNS responder started");
    }
    MDNS.addService("http", "tcp", 80);
    Serial.println("HTTPUpdateServer ready! Open http://esp32-webupdate.local/update in your browser");
  }
  
  // Allocate PWM timers and attach the servo.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  parachuteservo.setPeriodHertz(50);
  parachuteservo.attach(servoPin, 1000, 2000);
  
  lastAltitude = 0;
  
  // Setup web server route.
  server.on("/", []() {
    // Capture variables from global scope via lambda capture by reference.
    server.send(200, "text/html", webpage);
  });
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  // Initialize I2C bus for the camera on separate pins.
  Wire.begin(SDA_2, SCL_2, 100000);
  
  // Initialize camera if enabled.
  if (cameraEnabled) {
    if (cameraSetup() == 1) {
      if (showCameraOutput) {
        Serial.println("Camera initialized successfully");
      }
    } else {
      if (showCameraOutput) {
        Serial.println("Camera initialization failed");
      }
    }
  } else {
    if (showCameraOutput) {
      Serial.println("Camera initialization skipped (disabled).");
    }
  }
  
  Serial.println("Setup complete.");
}

// -----------------------
// Function: loop()
// -----------------------
// Main loop prioritized as follows:
// 1. High-priority: Sensor readings and altitude calculations (including parachute release check).
// 2. Lower-priority: SD logging, timelapse capture, web updates.
// Also, measures loop execution time and broadcasts configuration values.
void loop() {
  unsigned long loopStart = millis();  // Start time for loop execution measurement.
  unsigned long now = millis();
  
  // 1. High-Priority: Sensor Readings & Altitude Calculations
  float bmpTemp = 0, absoluteAltitude = 0, bmpPressure = 0;
  if (bmpFound) {
    bmpTemp = bmp.readTemperature();
    absoluteAltitude = bmp.readAltitude(getLocalSeaLevelPressure());
    bmpPressure = bmp.readPressure() / 100.0F;
  }
  
  sensors_event_t a, g, temp;
  if (mpuFound) {
    mpu.getEvent(&a, &g, &temp);
  } else {
    a.acceleration.x = a.acceleration.y = a.acceleration.z = 0;
    g.gyro.x = g.gyro.y = g.gyro.z = 0;
    temp.temperature = 0;
  }
  
  // Calculate relative altitude (current altitude minus baseline) and altitude drop.
  float relativeAltitude = (parachuteStatus == "unarmed") ? 0 : absoluteAltitude - baselineAltitude;
  float altitudeDrop = lastAltitude - relativeAltitude;
  
  // Update min/max values for absolute and relative altitudes and altitude drop.
  if (absoluteAltitude > maxAbsoluteAltitude) maxAbsoluteAltitude = absoluteAltitude;
  if (absoluteAltitude < minAbsoluteAltitude) minAbsoluteAltitude = absoluteAltitude;
  if (relativeAltitude > maxRelativeAltitude) maxRelativeAltitude = relativeAltitude;
  if (relativeAltitude < minRelativeAltitude) minRelativeAltitude = relativeAltitude;
  if (altitudeDrop > maxAltitudeDrop) maxAltitudeDrop = altitudeDrop;
  if (altitudeDrop < minAltitudeDrop) minAltitudeDrop = altitudeDrop;
  
  // High-priority: If armed and altitude drop exceeds threshold, release parachute.
  if (parachuteStatus == "armed" && altitudeDrop >= altitudeDropThreshold) {
    parachuteRelease();
    parachuteStatus = "released";
  }
  lastAltitude = relativeAltitude;
  
  // 2. Lower Priority: SD Card Logging (every SD_LOG_INTERVAL ms)
  static unsigned long lastSdLogTime = 0;
  if (now - lastSdLogTime >= SD_LOG_INTERVAL) {
    char dataString[1024];
    String currentTimestamp = getTimeStampString();
    // Append sensor data and configuration values to the SD log.
    snprintf(dataString, sizeof(dataString),
      "Timestamp: %s, BMP Temp: %.2f, Pressure: %.2f, Abs Alt: %.2f, Rel Alt: %.2f, Alt Drop: %.2f, MPU Temp: %.2f, "
      "Acc: (%.2f; %.2f; %.2f), Gyro: (%.2f; %.2f; %.2f), Parachute: %s, Local Pressure: %.2f, Default Sea-Level: %.2f, API: %s, "
      "MaxAbs: %.2f, MinAbs: %.2f, MaxRel: %.2f, MinRel: %.2f, MaxAltDrop: %.2f, MinAltDrop: %.2f, TotalSpace: %lluMB, UsedSpace: %lluMB, "
      "logBMP280Data: %s, logMPU6050Data: %s, logSpaceUsage: %s, showCameraOutput: %s, showTimelapseOutput: %s, showSensorInitLog: %s, "
      "timelapseInterval: %lu, SD_LOG_INTERVAL: %lu, WEB_UPDATE_INTERVAL: %lu, enableOTA: %s, altitudeDropThreshold: %.2f, LoopTime: %lums\n",
      currentTimestamp.c_str(), bmpTemp, bmpPressure, absoluteAltitude, relativeAltitude, altitudeDrop, temp.temperature,
      a.acceleration.x, a.acceleration.y, a.acceleration.z,
      g.gyro.x, g.gyro.y, g.gyro.z,
      parachuteStatus.c_str(), lastLocalPressure, defaultSeaLevelPressure, PressureSource.c_str(),
      maxAbsoluteAltitude, minAbsoluteAltitude, maxRelativeAltitude, minRelativeAltitude, maxAltitudeDrop, minAltitudeDrop,
      totalSpace, usedSpace,
      logBMP280Data ? "true" : "false",
      logMPU6050Data ? "true" : "false",
      logSpaceUsage ? "true" : "false",
      showCameraOutput ? "true" : "false",
      showTimelapseOutput ? "true" : "false",
      showSensorInitLog ? "true" : "false",
      timelapseInterval, SD_LOG_INTERVAL, WEB_UPDATE_INTERVAL,
      enableOTA ? "true" : "false",
      altitudeDropThreshold,
      millis() - loopStart);
    appendFile(SD_MMC, "/log.txt", dataString);
    lastSdLogTime = now;
  }
  
  // 3. Lower Priority: Timelapse Image Capture (every timelapseInterval ms if active)
  static unsigned long lastTimelapseCaptureTime = 0;
  if (timelapseActive && (now - lastTimelapseCaptureTime >= timelapseInterval)) {
    String picPath = captureAndSavePicture();
    if (showTimelapseOutput) {
      Serial.print("Timelapse capture saved at: ");
      Serial.println(picPath);
    }
    lastTimelapseCaptureTime = now;
  }
  
  // 4. Lowest Priority: Web Server and WebSocket Updates (every WEB_UPDATE_INTERVAL ms)
  server.handleClient();
  webSocket.loop();
  static unsigned long lastWebUpdateTime = 0;
  unsigned long loopTime = millis() - loopStart; // Calculate loop execution time.
  if (now - lastWebUpdateTime >= WEB_UPDATE_INTERVAL) {
    String jsonString = "";
    StaticJsonDocument<1024> doc;
    JsonObject object = doc.to<JsonObject>();
    // Sensor data and altitude values.
    object["AbsoluteAltitude"] = absoluteAltitude;
    object["RelativeAltitude"] = relativeAltitude;
    object["AltitudeDrop"] = altitudeDrop;
    object["MaxAltDrop"] = maxAltitudeDrop;
    object["MinAltDrop"] = minAltitudeDrop;
    object["BMP280Temp"] = bmpTemp;
    object["BMP280Pressure"] = bmpPressure;
    object["MPU6050Temp"] = temp.temperature;
    object["Accelerometer"] = String(a.acceleration.x) + "; " + String(a.acceleration.y) + "; " + String(a.acceleration.z);
    object["Gyroscope"] = String(g.gyro.x) + "; " + String(g.gyro.y) + "; " + String(g.gyro.z);
    object["ParachuteStatus"] = parachuteStatus;
    object["LocalPressure"] = lastLocalPressure;
    object["DefaultSeaLevelPressure"] = defaultSeaLevelPressure;
    object["PressureSource"] = PressureSource;
    object["MaxAbsAltitude"] = maxAbsoluteAltitude;
    object["MinAbsAltitude"] = minAbsoluteAltitude;
    object["MaxRelAltitude"] = maxRelativeAltitude;
    object["MinRelAltitude"] = minRelativeAltitude;
    // SD card space usage.
    object["TotalSpace"] = totalSpace;
    object["UsedSpace"] = usedSpace;
    object["TimelapseStatus"] = timelapseActive ? "active" : "inactive";
    // Configuration values.
    object["logBMP280Data"] = logBMP280Data ? "true" : "false";
    object["logMPU6050Data"] = logMPU6050Data ? "true" : "false";
    object["logSpaceUsage"] = logSpaceUsage ? "true" : "false";
    object["showCameraOutput"] = showCameraOutput ? "true" : "false";
    object["showTimelapseOutput"] = showTimelapseOutput ? "true" : "false";
    object["showSensorInitLog"] = showSensorInitLog ? "true" : "false";
    object["timelapseInterval"] = timelapseInterval;
    object["SD_LOG_INTERVAL"] = SD_LOG_INTERVAL;
    object["WEB_UPDATE_INTERVAL"] = WEB_UPDATE_INTERVAL;
    object["enableOTA"] = enableOTA ? "true" : "false";
    object["altitudeDropThreshold"] = altitudeDropThreshold;
    object["loopExecutionTime"] = loopTime;
    serializeJson(doc, jsonString);
    webSocket.broadcastTXT(jsonString);
    lastWebUpdateTime = now;
  }
  
  // Update SD card space usage values.
  totalSpace = SD_MMC.totalBytes() / (1024 * 1024);
  usedSpace = SD_MMC.usedBytes() / (1024 * 1024);
  
  // A very short delay allows background tasks to run.
  delay(400);
}
