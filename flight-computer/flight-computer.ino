/*
 * Flight Computer Firmware for ESP32
 *
 * Features:
 * - Reads sensor data from BMP280 (altitude, temperature, pressure) and MPU6050 (accelerometer, gyroscope)
 * - Logs data to an SD card; the log file is rotated only once during boot after a successful NTP update.
 * - Provides OTA updates and a web server with WebSocket broadcasting for real-time monitoring.
 * - Retrieves local sea-level pressure from the OpenWeatherMap API once per connection event,
 *   with fallback to EEPROM or a default value.
 * - Uses the NTP time (updated only once at boot) for all timestamps.
 * - Each sensor log line now includes the current SD card TotalSpace and UsedSpace values.
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
#include "camera_pins.h"          // Now uses our overridden camera pins

// -----------------------
// Library Inclusions for Sensors, SD, WiFi, OTA, etc.
// -----------------------
#define sensor_t adafruit_sensor_t  // Rename sensor_t to avoid conflicts with camera driver
#include <Adafruit_MPU6050.h>       // MPU6050 sensor library
#include <Adafruit_BMP280.h>        // BMP280 sensor library
#undef sensor_t

#include <Wire.h>                  // I²C communication library
#include <Adafruit_Sensor.h>       // Unified sensor library
#include <SD_MMC.h>                // SD card interface for ESP32-S3
#include "sd_read_write.h"         // Helper functions for SD card operations
#include <ESP32Servo.h>            // Servo control library for ESP32
#include <WiFi.h>                  // WiFi connectivity
#include <WiFiUdp.h>               // UDP library (for NTP)
#include <NTPClient.h>             // NTP client library
#include <WebServer.h>             // Web server library
#include <WebSocketsServer.h>      // WebSockets for real-time data transmission
#include <ArduinoJson.h>           // JSON library for building JSON objects
#include <HTTPClient.h>            // HTTP client library for API calls
#include <Arduino_ESP32_OTA.h>     // OTA update library for ESP32 (HTTP OTA)
#include <HTTPUpdateServer.h>      // HTTP update server library
#include <ESPmDNS.h>               // mDNS for network service discovery
#include <EEPROM.h>                // EEPROM library for non-volatile storage

// -----------------------
// Macro Definitions
// -----------------------
#define EEPROM_SIZE 10
#define EEPROM_PRESSURE_ADDR 0

// -----------------------
// Global Control Variables for Logging and Output
// -----------------------

// Master toggle for sensor serial log output
bool enableSensorSerialLog = true;  

// Toggle logging for specific sensor groups:
bool logBMP280Data        = false;    // BMP280 sensor (temperature, pressure, altitude)
bool logMPU6050Data       = false;    // MPU6050 sensor (accelerometer, gyroscope, temperature)
bool logSpaceUsage        = true;     // SD card space usage logs

// Toggle logging for camera output (initialization and capture events)
bool showCameraOutput     = true;    

// Toggle logging for timelapse events (start/stop and capture events)
bool showTimelapseOutput  = true;    

// Toggle logging for sensor initialization (detecting sensors during setup)
bool showSensorInitLog    = true;  

// -----------------------
// Global Variables for System State and Measurements
// -----------------------
bool cameraEnabled = true;         // Flag to control whether the camera is enabled

uint64_t totalSpace;               // Total SD card space (MB)
uint64_t usedSpace;                // Used SD card space (MB)

String PressureSource = "";        // String to indicate where pressure data came from
float lastLocalPressure = 1026.0;  // Default/fallback sea-level pressure (hPa)

bool apiPressureUpdated = false;   // Flag to indicate if API pressure update has occurred

// Variables for altitude tracking
float maxAbsoluteAltitude = -1000000.0;
float minAbsoluteAltitude = 1000000.0;
float maxRelativeAltitude = -1000000.0;
float minRelativeAltitude = 1000000.0;
float maxAltitudeDrop = -1000000.0;
float minAltitudeDrop = 1000000.0;

bool timelapseActive = false;      // Flag to control if timelapse capturing is active
unsigned long lastTimelapseCapture = 0;
const unsigned long timelapseInterval = 500; // Timelapse capture interval in milliseconds

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
const char* openWeatherMapApiKey = "API_KEY";
const char* lat = "LATITUDE";
const char* lon = "LONGITUDE";
const char* owmEndpoint = "https://api.openweathermap.org/data/3.0/onecall";

// -----------------------
// NTP Client Setup
// -----------------------
#define UTC_OFFSET_IN_SECONDS 3600
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", UTC_OFFSET_IN_SECONDS);
unsigned long lastSuccessfulNTP = 0;
unsigned long lastSyncMillis = 0;

// -----------------------
// Parachute and Altitude Calculations
// -----------------------
const float altitudeDropThreshold = 0.8;
float lastAltitude = 0;
String parachuteStatus = "unarmed";
float baselineAltitude = 0;
bool baselineCaptured = false;
const float defaultSeaLevelPressure = 1026.0;

// -----------------------
// Sensor Availability Flags
// -----------------------
bool bmpFound = false;
bool mpuFound = false;

// -----------------------
// Web Server and WebSocket HTML Content
// -----------------------
String webpage = "<!DOCTYPE html><html><head> <meta name='viewport' content='width=device-width,initial-scale=1'> <title>Flight Computer</title> <style> body { background-color: #EEEEEE; font-family: Arial, sans-serif; color: #003366; margin: 0; padding: 20px; } h1 { text-align: center; margin-bottom: 20px; } .data-table { margin: 0 auto; border-collapse: collapse; width: 90%; max-width: 600px; background-color: #FFF; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); } .data-table th, .data-table td { padding: 12px 15px; border: 1px solid #CCC; text-align: left; } .data-table th { background-color: #003366; color: #FFF; } .data-table tr:nth-child(even) { background-color: #F9F9F9; } .button-container { text-align: center; margin-top: 20px; } button { background-color: #003366; color: #FFF; border: none; padding: 10px 20px; font-size: 16px; cursor: pointer; margin: 5px; } button:hover { background-color: #0055AA; } </style></head><body> <h1>Flight Information</h1> <table class='data-table'> <tr> <th>Parameter</th> <th>Value</th> </tr> <tr> <td>Absolute Altitude</td> <td id='AbsoluteAltitude'>-</td> </tr> <tr> <td>Relative Altitude</td> <td id='RelativeAltitude'>-</td> </tr> <tr> <td>Altitude Drop</td> <td id='AltitudeDrop'>-</td> </tr> <tr> <td>BMP280 Temp</td> <td id='BMP280Temp'>-</td> </tr> <tr> <td>BMP280 Pressure</td> <td id='BMP280Pressure'>-</td> </tr> <tr> <td>MPU6050 Temp</td> <td id='MPU6050Temp'>-</td> </tr> <tr> <td>Accelerometer</td> <td id='Accelerometer'>-</td> </tr> <tr> <td>Gyroscope</td> <td id='Gyroscope'>-</td> </tr> <tr> <td>Parachute Status</td> <td id='ParachuteStatus'>-</td> </tr> <tr> <td>Local Pressure</td> <td id='LocalPressure'>-</td> </tr> <tr> <td>Default Sea-Level Pressure</td> <td id='DefaultSeaLevelPressure'>-</td> </tr> <tr> <td>Pressure Source</td> <td id='PressureSource'>-</td> </tr> <tr> <td>Max Abs Altitude</td> <td id='MaxAbsAltitude'>-</td> </tr> <tr> <td>Min Abs Altitude</td> <td id='MinAbsAltitude'>-</td> </tr> <tr> <td>Max Rel Altitude</td> <td id='MaxRelAltitude'>-</td> </tr> <tr> <td>Min Rel Altitude</td> <td id='MinRelAltitude'>-</td> </tr> <tr> <td>Total Space (MB)</td> <td id='TotalSpace'>-</td> </tr> <tr> <td>Used Space (MB)</td> <td id='UsedSpace'>-</td> </tr> <tr> <td>Timelapse Status</td> <td id='TimelapseStatus'>-</td> </tr> </table> <div class='button-container'> <button type='button' id='BTN_SEND_BACK'>Arm Parachute</button> <button type='button' id='BTN_START_TIMELAPSE'>Start Picture Timelapse</button> <button type='button' id='BTN_STOP_TIMELAPSE'>Stop Picture Timelapse</button> </div> <script> var Socket; document.getElementById('BTN_SEND_BACK').addEventListener('click', button_send_back); document.getElementById('BTN_START_TIMELAPSE').addEventListener('click', button_start_timelapse); document.getElementById('BTN_STOP_TIMELAPSE').addEventListener('click', button_stop_timelapse); function init() { Socket = new WebSocket('ws://' + window.location.hostname + ':81/'); Socket.onmessage = function(event) { processCommand(event); }; } function button_send_back() { var msg = { parachute: 'Armed' }; Socket.send(JSON.stringify(msg)); } function button_start_timelapse() { var msg = { timelapse: 'start' }; Socket.send(JSON.stringify(msg)); } function button_stop_timelapse() { var msg = { timelapse: 'stop' }; Socket.send(JSON.stringify(msg)); } function processCommand(event) { var obj = JSON.parse(event.data); document.getElementById('AbsoluteAltitude').innerHTML = obj.AbsoluteAltitude || '-'; document.getElementById('RelativeAltitude').innerHTML = obj.RelativeAltitude || '-'; document.getElementById('AltitudeDrop').innerHTML = obj.AltitudeDrop || '-'; document.getElementById('BMP280Temp').innerHTML = obj.BMP280Temp || '-'; document.getElementById('BMP280Pressure').innerHTML = obj.BMP280Pressure || '-'; document.getElementById('MPU6050Temp').innerHTML = obj.MPU6050Temp || '-'; document.getElementById('Accelerometer').innerHTML = obj.Accelerometer || '-'; document.getElementById('Gyroscope').innerHTML = obj.Gyroscope || '-'; document.getElementById('ParachuteStatus').innerHTML = obj.ParachuteStatus || '-'; document.getElementById('LocalPressure').innerHTML = obj.LocalPressure || '-'; document.getElementById('DefaultSeaLevelPressure').innerHTML = obj.DefaultSeaLevelPressure || '-'; document.getElementById('PressureSource').innerHTML = obj.PressureSource || '-'; document.getElementById('MaxAbsAltitude').innerHTML = obj.MaxAbsAltitude || '-'; document.getElementById('MinAbsAltitude').innerHTML = obj.MinAbsAltitude || '-'; document.getElementById('MaxRelAltitude').innerHTML = obj.MaxRelAltitude || '-'; document.getElementById('MinRelAltitude').innerHTML = obj.MinRelAltitude || '-'; document.getElementById('TotalSpace').innerHTML = obj.TotalSpace || '-'; document.getElementById('UsedSpace').innerHTML = obj.UsedSpace || '-'; document.getElementById('TimelapseStatus').innerHTML = obj.TimelapseStatus || '-'; } window.onload = function(event) { init(); } </script></body></html>";

// -----------------------
// Web Server and WebSocket Instances
// -----------------------
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
HTTPUpdateServer httpUpdater;  // For OTA updates

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
int servoPin = 14;

// -----------------------
// Function to Get a Timestamp String Using NTP Time
// -----------------------
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
// Function to Get Local Sea-Level Pressure
// -----------------------
float getLocalSeaLevelPressure() {
  return lastLocalPressure;
}

// -----------------------
// Function to Update Pressure Data from the OpenWeatherMap API
// -----------------------
void updatePressureFromAPI() {
  float localPressure = defaultSeaLevelPressure;
  HTTPClient http;
  String url = String(owmEndpoint) + "?lat=" + lat + "&lon=" + lon +
               "&exclude=minutely,hourly,daily,alerts&appid=" + openWeatherMapApiKey;
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
// Function to Rotate Log Files (Not used since rotation is handled at setup)
// -----------------------
void rotateLogFile() {
  // Log rotation is handled in the setup() function.
}

// -----------------------
// Function to Release the Parachute
// -----------------------
void parachuteRelease() {
  Serial.println("Altitude drop detected! Releasing parachute...");
  char eventLog[128];
  String eventTimestamp = getTimeStampString();
  snprintf(eventLog, sizeof(eventLog), "Timestamp: %s, Event: Parachute Released!\n", eventTimestamp.c_str());
  appendFile(SD_MMC, "/log.txt", eventLog);
  for (int i = 0; i < 2; i++) {
    parachuteservo.write(180);
    delay(200);
    parachuteservo.write(0);
  }
  parachuteStatus = "released";
}

// -----------------------
// Function to Arm the Parachute
// -----------------------
void parachuteArmed() {
  Serial.println("Arming parachute...");
  char eventLog[128];
  String eventTimestamp = getTimeStampString();
  snprintf(eventLog, sizeof(eventLog), "Timestamp: %s, Event: Parachute Armed!\n", eventTimestamp.c_str());
  appendFile(SD_MMC, "/log.txt", eventLog);
  
  // Capture baseline altitude if not already done
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
  
  // Move servo to arm the parachute
  parachuteservo.write(0);
  delay(200);
  parachuteservo.write(180);
  parachuteStatus = "armed";
}

// -----------------------
// WebSocket Event Handler
// -----------------------
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
      // Handle parachute command
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
      // Handle timelapse command
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
// Camera Setup Function
// -----------------------
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
  config.pixel_format = PIXFORMAT_JPEG; // JPEG for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // Use PSRAM if available for higher resolution
  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    // Limit the frame size if PSRAM is not available
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    if (showCameraOutput) {
      Serial.printf("Camera init failed with error 0x%x", err);
    }
    return 0;
  }

  sensor_t *s = esp_camera_sensor_get();
  // Adjust sensor settings: flip image, adjust brightness and saturation
  s->set_vflip(s, 1);       // Flip the image vertically
  s->set_brightness(s, 1);  // Increase brightness slightly
  s->set_saturation(s, 0);  // Lower saturation

  if (showCameraOutput) {
    Serial.println("Camera configuration complete!");
  }
  return 1;
}

// -----------------------
// Function to Capture and Save a Picture from the Camera
// -----------------------
// Updated to use readFileNum and writejpg for correct file naming.
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
  
  // Use the helper function readFileNum to get the next available photo index
  int photo_index = readFileNum(SD_MMC, "/camera");
  if (photo_index != -1) {
    String path = "/camera/" + String(photo_index) + ".jpg";
    // Save the image using writejpg (assumed defined in sd_read_write.h)
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
// Setup Function: Runs Once on Boot
// -----------------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting setup..."));

  // Initialize EEPROM for storing pressure data
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

  // Initialize WiFi: try to connect as a station first
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
  
  // Update pressure from API if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) {
    updatePressureFromAPI();
  }
  
  // Initialize and update NTP client for timestamps
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
  
  // Initialize SD Card and prepare logging
  Serial.print("Initializing SD card...");
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  if (!SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5)) {
    Serial.println("Card Mount Failed");
  } else {
    Serial.println("SD Card initialized.");
    createDir(SD_MMC, "/camera");
    listDir(SD_MMC, "/camera", 0);

    // Rename previous log file if exists
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
        Serial.println("Previous log file renamed successfully to " + newLogFile);
      } else {
        Serial.println("Failed to rename log file " + oldLogFile + " to " + newLogFile);
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
  
  // Calculate SD card space in MB
  totalSpace = SD_MMC.totalBytes() / (1024 * 1024);
  usedSpace = SD_MMC.usedBytes() / (1024 * 1024);
  Serial.printf("Total space: %lluMB\n", totalSpace);
  Serial.printf("Used space: %lluMB\n", usedSpace);
  
  // Initialize I2C bus for sensors (I2C_NUM_1) on pins 42 and 37
  Wire.begin(42, 37);
  
  // Initialize MPU6050 sensor
  if (mpu.begin(0x68)) {
    mpuFound = true;
    if (showSensorInitLog) {
      Serial.println("MPU6050 sensor found.");
    }
  } else {
    mpuFound = false;
    if (showSensorInitLog) {
      Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    }
  }
  
  // Initialize BMP280 sensor
  if (bmp.begin(0x76)) {
    bmpFound = true;
    if (showSensorInitLog) {
      Serial.println("BMP280 sensor found.");
    }
  } else {
    bmpFound = false;
    if (showSensorInitLog) {
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    }
  }
  
  // Configure MPU6050 sensor ranges and filters if available
  if (mpuFound) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  
  // Setup OTA update server and mDNS service
  httpUpdater.setup(&server);
  if (MDNS.begin("esp32-webupdate")) {
    Serial.println("MDNS responder started");
  }
  MDNS.addService("http", "tcp", 80);
  Serial.println("HTTPUpdateServer ready! Open http://esp32-webupdate.local/update in your browser");
  
  // Allocate PWM timers for servo control
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  parachuteservo.setPeriodHertz(50);
  parachuteservo.attach(servoPin, 1000, 2000);
  
  lastAltitude = 0;
  
  // Set up the web server routes
  server.on("/", []() {
    server.send(200, "text/html", webpage);
  });
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  // Initialize the I2C bus for the camera on pins SDA_2/SCL_2 (I2C_NUM_0)
  Wire.begin(SDA_2, SCL_2, 100000);
  
  // Conditionally initialize the camera if enabled
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
      Serial.println("Camera initialization skipped (camera disabled).");
    }
  }
  
  Serial.println("Setup complete.");
}

// -----------------------
// Main Loop Function
// -----------------------
void loop() {
  server.handleClient();
  webSocket.loop();
  
  // Update pressure via API if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) {
    if (!apiPressureUpdated) {
      updatePressureFromAPI();
    }
  } else {
    apiPressureUpdated = false;
  }
  
  // Update SD card space usage
  totalSpace = SD_MMC.totalBytes() / (1024 * 1024);
  usedSpace = SD_MMC.usedBytes() / (1024 * 1024);
  
  // Read sensor data from BMP280
  float bmpTemp = 0, absoluteAltitude = 0, bmpPressure = 0;
  if (bmpFound) {
    bmpTemp = bmp.readTemperature();
    absoluteAltitude = bmp.readAltitude(getLocalSeaLevelPressure());
    bmpPressure = bmp.readPressure() / 100.0F;
  }
  
  // Read sensor data from MPU6050
  sensors_event_t a, g, temp;
  if (mpuFound) {
    mpu.getEvent(&a, &g, &temp);
  } else {
    a.acceleration.x = a.acceleration.y = a.acceleration.z = 0;
    g.gyro.x = g.gyro.y = g.gyro.z = 0;
    temp.temperature = 0;
  }
  
  // Calculate relative altitude and altitude drop
  float relativeAltitude = (parachuteStatus == "unarmed") ? 0 : absoluteAltitude - baselineAltitude;
  float altitudeDrop = lastAltitude - relativeAltitude;
  
  // Update min/max altitude values
  if (absoluteAltitude > maxAbsoluteAltitude) maxAbsoluteAltitude = absoluteAltitude;
  if (absoluteAltitude < minAbsoluteAltitude) minAbsoluteAltitude = absoluteAltitude;
  if (relativeAltitude > maxRelativeAltitude) maxRelativeAltitude = relativeAltitude;
  if (relativeAltitude < minRelativeAltitude) minRelativeAltitude = relativeAltitude;
  if (altitudeDrop > maxAltitudeDrop) maxAltitudeDrop = altitudeDrop;
  if (altitudeDrop < minAltitudeDrop) minAltitudeDrop = altitudeDrop;
  
  // -----------------------
  // Sensor Data Logging to Serial (Conditional)
  // -----------------------
  if (enableSensorSerialLog && logBMP280Data) {
    Serial.print(getTimeStampString()); Serial.print(" BMP280 Temp: "); Serial.print(bmpTemp); Serial.println(" *C");
    Serial.print(getTimeStampString()); Serial.print(" BMP280 Pressure: "); Serial.print(bmpPressure); Serial.println(" hPa");
    Serial.print(getTimeStampString()); Serial.print(" Absolute Altitude: "); Serial.print(absoluteAltitude); Serial.println(" m");
    Serial.print(getTimeStampString()); Serial.print(" Relative Altitude: "); Serial.print(relativeAltitude); Serial.println(" m");
    Serial.print(getTimeStampString()); Serial.print(" Altitude Drop: "); Serial.print(altitudeDrop); Serial.println(" m");
    Serial.print(getTimeStampString()); Serial.print(" Max Altitude Drop: "); Serial.print(maxAltitudeDrop); Serial.println(" m");
    Serial.print(getTimeStampString()); Serial.print(" Min Altitude Drop: "); Serial.print(minAltitudeDrop); Serial.println(" m");
  }

  if (enableSensorSerialLog && logMPU6050Data) {
    Serial.print(getTimeStampString()); Serial.print(" MPU6050 Temp: "); Serial.print(temp.temperature); Serial.println(" *C");
    Serial.print(getTimeStampString()); Serial.print(" Accelerometer: "); Serial.print(a.acceleration.x); 
    Serial.print(", "); Serial.print(a.acceleration.y); Serial.print(", "); Serial.println(a.acceleration.z);
    Serial.print(getTimeStampString()); Serial.print(" Gyroscope: "); Serial.print(g.gyro.x); 
    Serial.print(", "); Serial.print(g.gyro.y); Serial.print(", "); Serial.println(g.gyro.z);
  }

  // General sensor logs (pressure, parachute status, altitude extremes)
  if (enableSensorSerialLog) {
    Serial.print(getTimeStampString()); Serial.print(" Local Pressure: "); Serial.print(lastLocalPressure); Serial.println(" hPa");
    Serial.print(getTimeStampString()); Serial.print(" Default Sea-Level Pressure: "); Serial.println(defaultSeaLevelPressure);
    Serial.print(getTimeStampString()); Serial.print(" Pressure Source: "); Serial.println(PressureSource);
    Serial.print(getTimeStampString()); Serial.print(" Parachute Status: "); Serial.println(parachuteStatus);
    Serial.print(getTimeStampString()); Serial.print(" Max Abs Altitude: "); Serial.print(maxAbsoluteAltitude);
    Serial.print(" m, Min Abs Altitude: "); Serial.println(minAbsoluteAltitude);
    Serial.print(getTimeStampString()); Serial.print(" Max Rel Altitude: "); Serial.print(maxRelativeAltitude);
    Serial.print(" m, Min Rel Altitude: "); Serial.println(minRelativeAltitude);
  }

  if (enableSensorSerialLog && logSpaceUsage) {
    Serial.printf("Total space: %lluMB\n", totalSpace);
    Serial.printf("Used space: %lluMB\n", usedSpace);
  }

  Serial.println("--------------------");
  
  // -----------------------
  // Logging Data to SD Card
  // -----------------------
  char dataString[512];
  String currentTimestamp = getTimeStampString();
  snprintf(dataString, sizeof(dataString),
    "Timestamp: %s, BMP Temp: %.2f, Pressure: %.2f, Absolute Altitude: %.2f, Relative Altitude: %.2f, Altitude Drop: %.2f, MPU Temp: %.2f, Acc: (%.2f; %.2f; %.2f), Gyro: (%.2f; %.2f; %.2f), Parachute Status: %s, Local Pressure: %.2f, Default Sea-Level Pressure: %.2f, API Status: %s, Max Abs Altitude: %.2f, Min Abs Altitude: %.2f, Max Rel Altitude: %.2f, Min Rel Altitude: %.2f, Max Alt Drop: %.2f, Min Alt Drop: %.2f, Total Space: %lluMB, Used Space: %lluMB\n",
    currentTimestamp.c_str(), bmpTemp, bmpPressure, absoluteAltitude, relativeAltitude, altitudeDrop, temp.temperature,
    a.acceleration.x, a.acceleration.y, a.acceleration.z,
    g.gyro.x, g.gyro.y, g.gyro.z,
    parachuteStatus.c_str(), lastLocalPressure, defaultSeaLevelPressure, PressureSource.c_str(),
    maxAbsoluteAltitude, minAbsoluteAltitude, maxRelativeAltitude, minRelativeAltitude, maxAltitudeDrop, minAltitudeDrop,
    totalSpace, usedSpace);
  appendFile(SD_MMC, "/log.txt", dataString);
  
  // Check if parachute should be released based on altitude drop
  if (parachuteStatus == "armed" && altitudeDrop >= altitudeDropThreshold) {
    parachuteRelease();
    parachuteStatus = "released";
  }
  lastAltitude = relativeAltitude;
  
  // -----------------------
  // Send Data to WebSocket Clients at Regular Intervals
  // -----------------------
  static unsigned long previousMillis = 0;
  int interval = 200;
  unsigned long nowMillis = millis();
  if ((nowMillis - previousMillis) > interval) {
    String jsonString = "";
    StaticJsonDocument<512> doc;
    JsonObject object = doc.to<JsonObject>();
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
    object["TotalSpace"] = totalSpace;
    object["UsedSpace"] = usedSpace;
       // Add timelapse status ("active" or "inactive")
    object["TimelapseStatus"] = timelapseActive ? "active" : "inactive";
    serializeJson(doc, jsonString);
    Serial.println(jsonString);
    webSocket.broadcastTXT(jsonString);
    previousMillis = nowMillis;
  }
  
  // -----------------------
  // Timelapse Image Capture
  // -----------------------
  if (timelapseActive) {
    if (millis() - lastTimelapseCapture >= timelapseInterval) {
      String picPath = captureAndSavePicture();
      if (showTimelapseOutput) {
        Serial.print("Timelapse capture saved at: ");
        Serial.println(picPath);
      }
      lastTimelapseCapture = millis();
    }
  }

  delay(100); // Loop delay for stability
}
