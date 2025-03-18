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
// Library Inclusions
// -----------------------
#include <Wire.h>                  // I2C communication
#include <Adafruit_Sensor.h>       // Unified sensor library
#include <Adafruit_BMP280.h>       // BMP280 sensor library
#include <Adafruit_MPU6050.h>      // MPU6050 sensor library
#include <SD_MMC.h>                // SD card interface for ESP32-S3
#include "sd_read_write.h"         // Helper functions for SD card operations (e.g., appendFile)
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
// Global Variables
// -----------------------

// SD Card space variables (to be included in every sensor log line)
uint64_t totalSpace;
uint64_t usedSpace;

// Global variable for pressure measurement and its source
String PressureSource = "";
float lastLocalPressure = 1026.0;  // Default/fallback sea-level pressure (in hPa)

// Flag to ensure the API is called only once per connection event
bool apiPressureUpdated = false;

// -----------------------
// WiFi and AP Credentials
// -----------------------
const char* ssid = "TDGC-Rocket";
const char* wifiPassword = "Rocket2022!";
const char* apSSID = "RocketAP";
const char* apPassword = "Rocket2022!";

// -----------------------
// OpenWeatherMap API Details
// -----------------------
const char* openWeatherMapApiKey = "API_KEY";
const char* lat = "LAT";
const char* lon = "LON";
const char* owmEndpoint = "https://api.openweathermap.org/data/3.0/onecall";

// -----------------------
// NTP Client Setup
// -----------------------
#define UTC_OFFSET_IN_SECONDS 3600
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", UTC_OFFSET_IN_SECONDS);
// Variables to store the NTP time after a successful update
unsigned long lastSuccessfulNTP = 0;
unsigned long lastSyncMillis = 0;

// -----------------------
// Global Variables for Parachute and Altitude Calculations
// -----------------------
const float altitudeDropThreshold = 0.8; // Threshold to trigger parachute release
float lastAltitude = 0;                  // Previous relative altitude for drop calculation
String parachuteStatus = "unarmed";      // Parachute status ("unarmed", "armed", "released")
float baselineAltitude = 0;              // Baseline altitude when arming parachute
bool baselineCaptured = false;           // Flag indicating if baseline altitude is captured
const float defaultSeaLevelPressure = 1026.0;  // Default sea-level pressure (in hPa)

// -----------------------
// Global Variables for Altitude Extremes
// -----------------------
float maxAbsoluteAltitude = -1000000.0;
float minAbsoluteAltitude =  1000000.0;
float maxRelativeAltitude = -1000000.0;
float minRelativeAltitude =  1000000.0;
float maxAltitudeDrop = -1000000.0;
float minAltitudeDrop =  1000000.0;

// -----------------------
// Web Server and WebSocket HTML
// -----------------------
String webpage = "<!DOCTYPE html><html><head> <meta name='viewport' content='width=device-width,initial-scale=1'> <title>Flight Computer</title> <style> body { background-color: #EEEEEE; font-family: Arial, sans-serif; color: #003366; margin: 0; padding: 20px } h1 { text-align: center; margin-bottom: 20px } .data-table { margin: 0 auto; border-collapse: collapse; width: 90%; max-width: 600px; background-color: #FFF; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1) } .data-table th, .data-table td { padding: 12px 15px; border: 1px solid #CCC; text-align: left } .data-table th { background-color: #003366; color: #FFF } .data-table tr:nth-child(even) { background-color: #F9F9F9 } .button-container { text-align: center; margin-top: 20px } button { background-color: #003366; color: #FFF; border: none; padding: 10px 20px; font-size: 16px; cursor: pointer } button:hover { background-color: #0055AA } </style></head><body> <h1>Flight Information</h1> <table class='data-table'> <tr> <th>Parameter</th> <th>Value</th> </tr> <tr> <td>Absolute Altitude</td> <td id='AbsoluteAltitude'>-</td> </tr> <tr> <td>Relative Altitude</td> <td id='RelativeAltitude'>-</td> </tr> <tr> <td>Altitude Drop</td> <td id='AltitudeDrop'>-</td> </tr> <tr> <td>BMP280 Temp</td> <td id='BMP280Temp'>-</td> </tr> <tr> <td>BMP280 Pressure</td> <td id='BMP280Pressure'>-</td> </tr> <tr> <td>MPU6050 Temp</td> <td id='MPU6050Temp'>-</td> </tr> <tr> <td>Accelerometer</td> <td id='Accelerometer'>-</td> </tr> <tr> <td>Gyroscope</td> <td id='Gyroscope'>-</td> </tr> <tr> <td>Parachute Status</td> <td id='ParachuteStatus'>-</td> </tr> <tr> <td>Local Pressure</td> <td id='LocalPressure'>-</td> </tr> <tr> <td>Default Sea-Level Pressure</td> <td id='DefaultSeaLevelPressure'>-</td> </tr> <tr> <td>Pressure Source</td> <td id='PressureSource'>-</td> </tr> <tr> <td>Max Abs Altitude</td> <td id='MaxAbsAltitude'>-</td> </tr> <tr> <td>Min Abs Altitude</td> <td id='MinAbsAltitude'>-</td> </tr> <tr> <td>Max Rel Altitude</td> <td id='MaxRelAltitude'>-</td> </tr> <tr> <td>Min Rel Altitude</td> <td id='MinRelAltitude'>-</td> </tr> <tr> <td>Total Space (MB)</td> <td id='TotalSpace'>-</td> </tr> <tr> <td>Used Space (MB)</td> <td id='UsedSpace'>-</td> </tr> </table> <div class='button-container'><button type='button' id='BTN_SEND_BACK'>Arm Parachute</button></div> <script> var Socket; document.getElementById('BTN_SEND_BACK').addEventListener('click', button_send_back); function init() { Socket = new WebSocket('ws://' + window.location.hostname + ':81/'); Socket.onmessage = function(event) { processCommand(event); }; } function button_send_back() { var msg = { parachute: 'Armed' }; Socket.send(JSON.stringify(msg)); } function processCommand(event) { var obj = JSON.parse(event.data); document.getElementById('AbsoluteAltitude').innerHTML = obj.AbsoluteAltitude || '-'; document.getElementById('RelativeAltitude').innerHTML = obj.RelativeAltitude || '-'; document.getElementById('AltitudeDrop').innerHTML = obj.AltitudeDrop || '-'; document.getElementById('BMP280Temp').innerHTML = obj.BMP280Temp || '-'; document.getElementById('BMP280Pressure').innerHTML = obj.BMP280Pressure || '-'; document.getElementById('MPU6050Temp').innerHTML = obj.MPU6050Temp || '-'; document.getElementById('Accelerometer').innerHTML = obj.Accelerometer || '-'; document.getElementById('Gyroscope').innerHTML = obj.Gyroscope || '-'; document.getElementById('ParachuteStatus').innerHTML = obj.ParachuteStatus || '-'; document.getElementById('LocalPressure').innerHTML = obj.LocalPressure || '-'; document.getElementById('DefaultSeaLevelPressure').innerHTML = obj.DefaultSeaLevelPressure || '-'; document.getElementById('PressureSource').innerHTML = obj.PressureSource || '-'; document.getElementById('MaxAbsAltitude').innerHTML = obj.MaxAbsAltitude || '-'; document.getElementById('MinAbsAltitude').innerHTML = obj.MinAbsAltitude || '-'; document.getElementById('MaxRelAltitude').innerHTML = obj.MaxRelAltitude || '-'; document.getElementById('MinRelAltitude').innerHTML = obj.MinRelAltitude || '-'; document.getElementById('TotalSpace').innerHTML = obj.TotalSpace || '-'; document.getElementById('UsedSpace').innerHTML = obj.UsedSpace || '-'; } window.onload = function(event) { init(); } </script></body></html>";

// -----------------------
// Web Server and WebSocket Instances
// -----------------------
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
HTTPUpdateServer httpUpdater;  // For OTA updates

// -----------------------
// I2C Bus and SD_MMC Pin Definitions
// -----------------------
#define SDA_1 42
#define SCL_1 41
#define SDA_2 47
#define SCL_2 21
#define SD_MMC_CMD 38
#define SD_MMC_CLK 39
#define SD_MMC_D0  40

// -----------------------
// I2C Bus Instances
// -----------------------
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

// -----------------------
// Sensor Instances
// -----------------------
Adafruit_BMP280 bmp(&I2Cone); // BMP280 sensor instance
Adafruit_MPU6050 mpu;         // MPU6050 sensor instance

// -----------------------
// Servo Configuration
// -----------------------
Servo parachuteservo;
int servoPin = 14;

// -----------------------
// Function Definitions
// -----------------------

/**
 * @brief Returns a formatted timestamp string (YYYY-MM-DD HH:MM:SS)
 * using the stored NTP time plus elapsed time.
 */
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

/**
 * @brief Returns the cached local sea-level pressure.
 */
float getLocalSeaLevelPressure() {
  return lastLocalPressure;
}

/**
 * @brief Calls the OpenWeatherMap API to update the local sea-level pressure.
 * On success, stores the pressure value in EEPROM and caches it.
 * On failure, falls back to EEPROM (if valid) or a default value.
 */
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

  // Fallback: if API failed, use EEPROM if valid
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
  apiPressureUpdated = true; // Mark API update as complete
}

/**
 * @brief Rotates the log file.
 * This function is not called in loop() because log rotation is performed only once at boot.
 */
void rotateLogFile() {
  // Not used; log rotation is handled in setup().
}

/**
 * @brief Releases the parachute by actuating the servo.
 * Logs the event and updates the parachute status.
 */
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

/**
 * @brief Arms the parachute.
 * Captures the baseline altitude (using cached pressure) if not already captured,
 * logs the event, and actuates the servo.
 */
void parachuteArmed() {
  Serial.println("Arming parachute...");
  char eventLog[128];
  String eventTimestamp = getTimeStampString();
  snprintf(eventLog, sizeof(eventLog), "Timestamp: %s, Event: Parachute Armed!\n", eventTimestamp.c_str());
  appendFile(SD_MMC, "/log.txt", eventLog);
  if (!baselineCaptured) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    baselineAltitude = bmp.readAltitude(getLocalSeaLevelPressure());
    baselineCaptured = true;
    Serial.print("Baseline altitude captured: ");
    Serial.print(baselineAltitude);
    Serial.println(" m");
  }
  parachuteservo.write(0);
  delay(200);
  parachuteservo.write(180);
  parachuteStatus = "armed";
}

/**
 * @brief Handles incoming WebSocket events.
 * Processes connections, disconnections, and messages.
 */
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
      } else {
        const char* g_parachute = doc["parachute"];
        Serial.println("Received command from user: " + String(num));
        Serial.println("Parachute: " + String(g_parachute));
        if (parachuteStatus != "armed") {
          if (String(g_parachute) == "Armed") {
            parachuteArmed();
            Serial.println("Parachute armed command processed.");
          }
        }
      }
      Serial.println("");
      break;
    }
  }
}

// -----------------------
// Setup Function
// -----------------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting setup..."));

  // Initialize EEPROM for pressure data
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

  // --- WiFi Connection ---
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
  
  // If connected, update pressure from API once during setup
  if (WiFi.status() == WL_CONNECTED) {
    updatePressureFromAPI();
  }
  
  // --- NTP Time Update (Once) ---
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
  
  // --- Log Rotation (Only at Boot) ---
  // Now that we have a valid timestamp, initialize the SD card and rotate the log.
  Serial.print("Initializing SD card...");
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  if (!SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5)) {
    Serial.println("Card Mount Failed");
  } else {
    Serial.println("SD Card initialized.");
    
    // Rotate log file: if "/log.txt" exists, rename it with the current timestamp.
    String oldLogFile = "/log.txt";
    String timestamp = getTimeStampString();
    timestamp.replace(":", "");   // Remove colons for filename safety
    timestamp.replace(" ", "_");  // Replace spaces with underscores
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
    
    // Create a new log file with a header.
    File file = SD_MMC.open(oldLogFile.c_str(), FILE_WRITE);
    if (file) {
      file.println("New log session started: " + getTimeStampString());
      file.close();
    }
  }
  
  // Initialize and log SD card space info (will be updated in every sensor log later)
  totalSpace = SD_MMC.totalBytes() / (1024 * 1024);
  usedSpace = SD_MMC.usedBytes() / (1024 * 1024);
  Serial.printf("Total space: %lluMB\n", totalSpace);
  Serial.printf("Used space: %lluMB\n", usedSpace);
  
  // --- I2C and Sensor Initialization ---
  I2Cone.begin(SDA_1, SCL_1, 100000);
  I2Ctwo.begin(SDA_2, SCL_2, 100000);
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  if (!mpu.begin(0x68, &I2Ctwo)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // --- HTTP OTA Setup ---
  httpUpdater.setup(&server);
  if (MDNS.begin("esp32-webupdate")) {
    Serial.println("MDNS responder started");
  }
  MDNS.addService("http", "tcp", 80);
  Serial.printf("HTTPUpdateServer ready! Open http://esp32-webupdate.local/update in your browser\n");
  
  // --- Servo Initialization ---
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  parachuteservo.setPeriodHertz(50);
  parachuteservo.attach(servoPin, 1000, 2000);
  
  lastAltitude = 0;
  
  // --- Web Server and WebSocket Initialization ---
  server.on("/", []() {
    server.send(200, "text/html", webpage);
  });
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  Serial.println("Setup complete.");
}

// -----------------------
// Main Loop Function
// -----------------------
void loop() {
  server.handleClient();
  webSocket.loop();
  
  // Do not update NTP time periodically; use the initial time from setup.
  
  // Check WiFi connection: if connected and API not yet called for this session, update pressure.
  if (WiFi.status() == WL_CONNECTED) {
    if (!apiPressureUpdated) {
      updatePressureFromAPI();
    }
  } else {
    apiPressureUpdated = false; // Reset flag if WiFi disconnects
  }
  
  // Update SD card space info before each sensor log so that every line includes the latest values.
  totalSpace = SD_MMC.totalBytes() / (1024 * 1024);
  usedSpace = SD_MMC.usedBytes() / (1024 * 1024);
  
  // Set BMP280 sensor sampling settings
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  
  // Read sensor data from BMP280 and MPU6050
  float bmpTemp = bmp.readTemperature();
  float absoluteAltitude = bmp.readAltitude(getLocalSeaLevelPressure());
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calculate relative altitude and altitude drop
  float relativeAltitude = (parachuteStatus == "unarmed") ? 0 : absoluteAltitude - baselineAltitude;
  float altitudeDrop = lastAltitude - relativeAltitude;
  
  // Update altitude extremes
  if (absoluteAltitude > maxAbsoluteAltitude) maxAbsoluteAltitude = absoluteAltitude;
  if (absoluteAltitude < minAbsoluteAltitude) minAbsoluteAltitude = absoluteAltitude;
  if (relativeAltitude > maxRelativeAltitude) maxRelativeAltitude = relativeAltitude;
  if (relativeAltitude < minRelativeAltitude) minRelativeAltitude = relativeAltitude;
  if (altitudeDrop > maxAltitudeDrop) maxAltitudeDrop = altitudeDrop;
  if (altitudeDrop < minAltitudeDrop) minAltitudeDrop = altitudeDrop;
  
  // Print sensor data and system status to Serial monitor
  Serial.print(getTimeStampString()); Serial.print(" BMP280 Temp: "); Serial.print(bmpTemp); Serial.println(" *C");
  Serial.print(getTimeStampString()); Serial.print(" BMP280 Pressure: "); Serial.print(bmp.readPressure() / 100.0F); Serial.println(" hPa");
  Serial.print(getTimeStampString()); Serial.print(" Absolute Altitude: "); Serial.print(absoluteAltitude); Serial.println(" m");
  Serial.print(getTimeStampString()); Serial.print(" Relative Altitude: "); Serial.print(relativeAltitude); Serial.println(" m");
  Serial.print(getTimeStampString()); Serial.print(" Altitude Drop: "); Serial.print(altitudeDrop); Serial.println(" m");
  Serial.print(getTimeStampString()); Serial.print(" Max Altitude Drop: "); Serial.print(maxAltitudeDrop); Serial.println(" m");
  Serial.print(getTimeStampString()); Serial.print(" Min Altitude Drop: "); Serial.print(minAltitudeDrop); Serial.println(" m");
  Serial.print(getTimeStampString()); Serial.print(" MPU6050 Temp: "); Serial.print(temp.temperature); Serial.println(" *C");
  Serial.print(getTimeStampString()); Serial.print(" Accelerometer: "); Serial.print(a.acceleration.x); Serial.print(", "); Serial.print(a.acceleration.y); Serial.print(", "); Serial.println(a.acceleration.z);
  Serial.print(getTimeStampString()); Serial.print(" Gyroscope: "); Serial.print(g.gyro.x); Serial.print(", "); Serial.print(g.gyro.y); Serial.print(", "); Serial.println(g.gyro.z);
  Serial.print(getTimeStampString()); Serial.print(" Local Pressure: "); Serial.print(lastLocalPressure); Serial.println(" hPa");
  Serial.print(getTimeStampString()); Serial.print(" Default Sea-Level Pressure: "); Serial.println(defaultSeaLevelPressure);
  Serial.print(getTimeStampString()); Serial.print(" Pressure Source: "); Serial.println(PressureSource);
  Serial.print(getTimeStampString()); Serial.print(" Parachute Status: "); Serial.println(parachuteStatus);
  Serial.print(getTimeStampString()); Serial.print(" Max Abs Altitude: "); Serial.print(maxAbsoluteAltitude); Serial.print(" m, Min Abs Altitude: "); Serial.println(minAbsoluteAltitude);
  Serial.print(getTimeStampString()); Serial.print(" Max Rel Altitude: "); Serial.print(maxRelativeAltitude); Serial.print(" m, Min Rel Altitude: "); Serial.println(minRelativeAltitude);
  Serial.println("--------------------");
  
  // Log sensor data and system status to SD card with TotalSpace and UsedSpace included in the same line
  char dataString[512];
  String currentTimestamp = getTimeStampString();
  snprintf(dataString, sizeof(dataString),
    "Timestamp: %s, BMP Temp: %.2f, Pressure: %.2f, Absolute Altitude: %.2f, Relative Altitude: %.2f, Altitude Drop: %.2f, MPU Temp: %.2f, Acc: (%.2f; %.2f; %.2f), Gyro: (%.2f; %.2f; %.2f), Parachute Status: %s, Local Pressure: %.2f, Default Sea-Level Pressure: %.2f, API Status: %s, Max Abs Altitude: %.2f, Min Abs Altitude: %.2f, Max Rel Altitude: %.2f, Min Rel Altitude: %.2f, Max Alt Drop: %.2f, Min Alt Drop: %.2f, Total Space: %lluMB, Used Space: %lluMB\n",
    currentTimestamp.c_str(), bmpTemp, bmp.readPressure() / 100.0F, absoluteAltitude, relativeAltitude, altitudeDrop, temp.temperature,
    a.acceleration.x, a.acceleration.y, a.acceleration.z,
    g.gyro.x, g.gyro.y, g.gyro.z,
    parachuteStatus.c_str(), lastLocalPressure, defaultSeaLevelPressure, PressureSource.c_str(),
    maxAbsoluteAltitude, minAbsoluteAltitude, maxRelativeAltitude, minRelativeAltitude, maxAltitudeDrop, minAltitudeDrop,
    totalSpace, usedSpace);
  appendFile(SD_MMC, "/log.txt", dataString);
  
  // Trigger parachute release if armed and altitude drop exceeds threshold
  if (parachuteStatus == "armed" && altitudeDrop >= altitudeDropThreshold) {
    parachuteRelease();
    parachuteStatus = "released";
  }
  lastAltitude = relativeAltitude;
  
  // Broadcast sensor data and system status via WebSocket every 200 ms
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
    object["BMP280Pressure"] = bmp.readPressure() / 100.0F;
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
    serializeJson(doc, jsonString);
    Serial.println(jsonString);
    webSocket.broadcastTXT(jsonString);
    previousMillis = nowMillis;
  }
  delay(100);
}
