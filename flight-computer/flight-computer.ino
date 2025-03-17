// -----------------------
// Library Inclusions
// -----------------------
#include <Wire.h>                  // I2C communication library
#include <Adafruit_Sensor.h>       // Unified sensor library
#include <Adafruit_BMP280.h>       // BMP280 sensor library (temperature, pressure, altitude)
#include <Adafruit_MPU6050.h>      // MPU6050 sensor library (accelerometer & gyroscope)
#include <SD_MMC.h>                // SD_MMC interface for the ESP32-S3 built-in SD
#include "sd_read_write.h"         // Helper functions for SD card operations
#include <ESP32Servo.h>            // Servo control library for ESP32
#include <WiFi.h>                  // WiFi connectivity library
#include <WiFiUdp.h>               // UDP library (needed for NTP client)
#include <NTPClient.h>             // NTP client library
#include <WebServer.h>             // Web server library
#include <WebSocketsServer.h>      // WebSockets server library
#include <ArduinoJson.h>           // JSON library for encapsulating multiple variables
#include <HTTPClient.h>            // HTTP client library for API calls
#include <Arduino_ESP32_OTA.h>     // OTA update library for ESP32 (HTTP OTA)
#include <HTTPUpdateServer.h>      // HTTP update server library for OTA updates
#include <ESPmDNS.h>               // mDNS library

// -----------------------
// Function Prototypes
// -----------------------
String getTimeStampString();
void parachuteRelease();
void parachuteArmed();
void webSocketEvent(byte num, WStype_t type, uint8_t * payload, size_t length);
float getLocalSeaLevelPressure();  // Retrieve local sea-level pressure via API (blocking until WiFi connected)

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
const char* lat = "LATITUDE";
const char* lon = "LONTITUDE";
const char* owmEndpoint = "https://api.openweathermap.org/data/3.0/onecall";

// -----------------------
// NTP Client Configuration
// -----------------------
#define UTC_OFFSET_IN_SECONDS 3600
char daysOfTheWeek[7][12] = {"Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", UTC_OFFSET_IN_SECONDS);
unsigned long lastSuccessfulNTP = 0;
unsigned long lastSyncMillis = 0;

// -----------------------
// Global Variables for Parachute and Altitude Calculations
// -----------------------
const float altitudeDropThreshold = 0.8;
float lastAltitude = 0;
String parachuteStatus = "unarmed";
float baselineAltitude = 0;
bool baselineCaptured = false;
const float defaultSeaLevelPressure = 1026.0;
float lastLocalPressure = defaultSeaLevelPressure;
String apiStatus = "Not Checked";

// -----------------------
// Global Variables for Altitude Extremes
// -----------------------
float maxAbsoluteAltitude = -1000000.0;
float minAbsoluteAltitude =  1000000.0;
float maxRelativeAltitude = -1000000.0;
float minRelativeAltitude =  1000000.0;
// New global variables for altitude drops
float maxAltitudeDrop = -1000000.0;
float minAltitudeDrop =  1000000.0;

// -----------------------
// Webserver/Websocket HTML (Compressed to 1 line)
// -----------------------
String webpage = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'><title>Flight Computer</title><style>body{background-color:#EEEEEE;font-family:Arial,sans-serif;color:#003366;margin:0;padding:20px}h1{text-align:center;margin-bottom:20px}.data-table{margin:0 auto;border-collapse:collapse;width:90%;max-width:600px;background-color:#FFF;box-shadow:0 0 10px rgba(0,0,0,0.1)}.data-table th,.data-table td{padding:12px 15px;border:1px solid #CCC;text-align:left}.data-table th{background-color:#003366;color:#FFF}.data-table tr:nth-child(even){background-color:#F9F9F9}.button-container{text-align:center;margin-top:20px}button{background-color:#003366;color:#FFF;border:none;padding:10px 20px;font-size:16px;cursor:pointer}button:hover{background-color:#0055AA}</style></head><body><h1>Flight Information</h1><table class='data-table'><tr><th>Parameter</th><th>Value</th></tr><tr><td>Absolute Altitude</td><td id='AbsoluteAltitude'>-</td></tr><tr><td>Relative Altitude</td><td id='RelativeAltitude'>-</td></tr><tr><td>Altitude Drop</td><td id='AltitudeDrop'>-</td></tr><tr><td>Max Altitude Drop</td><td id='MaxAltDrop'>-</td></tr><tr><td>Min Altitude Drop</td><td id='MinAltDrop'>-</td></tr><tr><td>BMP280 Temp</td><td id='BMP280Temp'>-</td></tr><tr><td>BMP280 Pressure</td><td id='BMP280Pressure'>-</td></tr><tr><td>MPU6050 Temp</td><td id='MPU6050Temp'>-</td></tr><tr><td>Accelerometer</td><td id='Accelerometer'>-</td></tr><tr><td>Gyroscope</td><td id='Gyroscope'>-</td></tr><tr><td>Parachute Status</td><td id='ParachuteStatus'>-</td></tr><tr><td>Local Pressure</td><td id='LocalPressure'>-</td></tr><tr><td>Default Sea-Level Pressure</td><td id='DefaultSeaLevelPressure'>-</td></tr><tr><td>API Status</td><td id='APIStatus'>-</td></tr><tr><td>Max Abs Altitude</td><td id='MaxAbsAltitude'>-</td></tr><tr><td>Min Abs Altitude</td><td id='MinAbsAltitude'>-</td></tr><tr><td>Max Rel Altitude</td><td id='MaxRelAltitude'>-</td></tr><tr><td>Min Rel Altitude</td><td id='MinRelAltitude'>-</td></tr></table><div class='button-container'><button type='button' id='BTN_SEND_BACK'>Arm Parachute</button></div><script>var Socket;document.getElementById('BTN_SEND_BACK').addEventListener('click',button_send_back);function init(){Socket=new WebSocket('ws://'+window.location.hostname+':81/');Socket.onmessage=function(event){processCommand(event);};}function button_send_back(){var msg={parachute:'Armed'};Socket.send(JSON.stringify(msg));}function processCommand(event){var obj=JSON.parse(event.data);document.getElementById('AbsoluteAltitude').innerHTML=obj.AbsoluteAltitude||'-';document.getElementById('RelativeAltitude').innerHTML=obj.RelativeAltitude||'-';document.getElementById('AltitudeDrop').innerHTML=obj.AltitudeDrop||'-';document.getElementById('MaxAltDrop').innerHTML=obj.MaxAltDrop||'-';document.getElementById('MinAltDrop').innerHTML=obj.MinAltDrop||'-';document.getElementById('BMP280Temp').innerHTML=obj.BMP280Temp||'-';document.getElementById('BMP280Pressure').innerHTML=obj.BMP280Pressure||'-';document.getElementById('MPU6050Temp').innerHTML=obj.MPU6050Temp||'-';document.getElementById('Accelerometer').innerHTML=obj.Accelerometer||'-';document.getElementById('Gyroscope').innerHTML=obj.Gyroscope||'-';document.getElementById('ParachuteStatus').innerHTML=obj.ParachuteStatus||'-';document.getElementById('LocalPressure').innerHTML=obj.LocalPressure||'-';document.getElementById('DefaultSeaLevelPressure').innerHTML=obj.DefaultSeaLevelPressure||'-';document.getElementById('APIStatus').innerHTML=obj.APIStatus||'-';document.getElementById('MaxAbsAltitude').innerHTML=obj.MaxAbsAltitude||'-';document.getElementById('MinAbsAltitude').innerHTML=obj.MinAbsAltitude||'-';document.getElementById('MaxRelAltitude').innerHTML=obj.MaxRelAltitude||'-';document.getElementById('MinRelAltitude').innerHTML=obj.MinRelAltitude||'-';}window.onload=function(event){init();}</script></body></html>";

// -----------------------
// Timing Variables for WebSocket Updates
// -----------------------
int interval = 200;
unsigned long previousMillis = 0;

// -----------------------
// Web Server and WebSocket Instances
// -----------------------
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
HTTPUpdateServer httpUpdater;  // HTTP OTA update server instance

// -----------------------
// I2C Bus Pin Definitions
// -----------------------
#define SDA_1 42
#define SCL_1 41
#define SDA_2 47
#define SCL_2 21

// -----------------------
// SD_MMC Pin Definitions
// -----------------------
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
Adafruit_BMP280 bmp(&I2Cone);
Adafruit_MPU6050 mpu;

// -----------------------
// Servo Configuration
// -----------------------
Servo parachuteservo;
int servoPin = 14;

// -----------------------
// getLocalSeaLevelPressure Function (Blocking Version)
// -----------------------
float getLocalSeaLevelPressure() {
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected. Waiting for connection...");
    delay(500);
  }
  float localPressure = defaultSeaLevelPressure;
  HTTPClient http;
  String url = String(owmEndpoint) + "?lat=" + lat + "&lon=" + lon + "&exclude=minutely,hourly,daily,alerts&appid=" + openWeatherMapApiKey;
  http.begin(url);
  int httpCode = http.GET();
  if (httpCode > 0) {
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      StaticJsonDocument<1024> doc;
      DeserializationError error = deserializeJson(doc, payload);
      if (!error) {
        localPressure = doc["current"]["pressure"] | defaultSeaLevelPressure;
        apiStatus = "API Connection Successful";
        Serial.println("API connection successful. Pressure retrieved.");
      } else {
        apiStatus = "JSON Parse Error";
        Serial.println("JSON parse error.");
      }
    } else {
      apiStatus = "HTTP error: " + String(httpCode);
      Serial.print("HTTP error code: ");
      Serial.println(httpCode);
    }
  } else {
    apiStatus = "HTTP Request Failed";
    Serial.print("HTTP request failed, error: ");
    Serial.println(http.errorToString(httpCode));
  }
  http.end();
  lastLocalPressure = localPressure;
  return localPressure;
}

// -----------------------
// Setup Function
// -----------------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting setup..."));
  
  // --- WiFi Connection Attempt ---
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
  
  // --- HTTP OTA Setup ---
  // Set up an HTTP OTA update endpoint (default URL: /update)
  httpUpdater.setup(&server);
  
  // Start mDNS with the host name "esp32-webupdate"
  if (MDNS.begin("esp32-webupdate")) {
    Serial.println("MDNS responder started");
  }
  MDNS.addService("http", "tcp", 80);
  Serial.printf("HTTPUpdateServer ready! Open http://esp32-webupdate.local/update in your browser\n");
  
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
  
  // --- SD Card Initialization ---
  Serial.print("Initializing SD card...");
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  if (!SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5)) {
    Serial.println("Card Mount Failed");
  } else {
    Serial.println("SD Card initialized.");
    listDir(SD_MMC, "/", 0);
  }
  
  // --- Servo Initialization ---
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  parachuteservo.setPeriodHertz(50);
  parachuteservo.attach(servoPin, 1000, 2000);
  
  lastAltitude = 0;
  timeClient.begin();
  timeClient.update();
  unsigned long currentEpoch = timeClient.getEpochTime();
  if (currentEpoch > 100000) {
    lastSuccessfulNTP = currentEpoch;
    lastSyncMillis = millis();
  }
  
  // --- Webserver and Websocket Initialization ---
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
  // Handle OTA updates via HTTP endpoint "/update"
  server.handleClient();
  webSocket.loop();
  
  if (millis() - lastSyncMillis > 60000) {
    timeClient.update();
    unsigned long currentEpoch = timeClient.getEpochTime();
    if (currentEpoch > 100000) {
      lastSuccessfulNTP = currentEpoch;
      lastSyncMillis = millis();
    }
  }
  
  // Configure BMP280 sampling settings
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  
  float bmpTemp = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0F;
  float absoluteAltitude = bmp.readAltitude(getLocalSeaLevelPressure());
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float relativeAltitude = (parachuteStatus == "unarmed") ? 0 : absoluteAltitude - baselineAltitude;
  float altitudeDrop = lastAltitude - relativeAltitude;
  
  // Update max/min values for absolute and relative altitudes
  if (absoluteAltitude > maxAbsoluteAltitude) maxAbsoluteAltitude = absoluteAltitude;
  if (absoluteAltitude < minAbsoluteAltitude) minAbsoluteAltitude = absoluteAltitude;
  if (relativeAltitude > maxRelativeAltitude) maxRelativeAltitude = relativeAltitude;
  if (relativeAltitude < minRelativeAltitude) minRelativeAltitude = relativeAltitude;
  
  // --- New: Update max/min altitude drops ---
  if (altitudeDrop > maxAltitudeDrop) maxAltitudeDrop = altitudeDrop;
  if (altitudeDrop < minAltitudeDrop) minAltitudeDrop = altitudeDrop;
  
  // --- Serial Monitor Output ---
  Serial.print(getTimeStampString()); Serial.print(" BMP280 Temp: "); Serial.print(bmpTemp); Serial.println(" *C");
  Serial.print(getTimeStampString()); Serial.print(" BMP280 Pressure: "); Serial.print(pressure); Serial.println(" hPa");
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
  Serial.print(getTimeStampString()); Serial.print(" API Status: "); Serial.println(apiStatus);
  Serial.print(getTimeStampString()); Serial.print(" Parachute Status: "); Serial.println(parachuteStatus);
  Serial.print(getTimeStampString()); Serial.print(" Max Abs Altitude: "); Serial.print(maxAbsoluteAltitude); Serial.print(" m, Min Abs Altitude: "); Serial.println(minAbsoluteAltitude);
  Serial.print(getTimeStampString()); Serial.print(" Max Rel Altitude: "); Serial.print(maxRelativeAltitude); Serial.print(" m, Min Rel Altitude: "); Serial.println(minRelativeAltitude);
  Serial.println("--------------------");
  
  // --- SD Card Logging ---
  char dataString[512];
  String currentTimestamp = getTimeStampString();
  snprintf(dataString, sizeof(dataString),
    "Timestamp: %s, BMP Temp: %.2f, Pressure: %.2f, Absolute Altitude: %.2f, Relative Altitude: %.2f, Altitude Drop: %.2f, MPU Temp: %.2f, Acc: (%.2f, %.2f, %.2f), Gyro: (%.2f, %.2f, %.2f), Parachute Status: %s, Local Pressure: %.2f, Default Sea-Level Pressure: %.2f, API Status: %s, Max Abs Altitude: %.2f, Min Abs Altitude: %.2f, Max Rel Altitude: %.2f, Min Rel Altitude: %.2f, Max Alt Drop: %.2f, Min Alt Drop: %.2f\n",
    currentTimestamp.c_str(), bmpTemp, pressure, absoluteAltitude, relativeAltitude, altitudeDrop, temp.temperature,
    a.acceleration.x, a.acceleration.y, a.acceleration.z,
    g.gyro.x, g.gyro.y, g.gyro.z,
    parachuteStatus.c_str(), lastLocalPressure, defaultSeaLevelPressure, maxAbsoluteAltitude, minAbsoluteAltitude, maxRelativeAltitude, minRelativeAltitude, maxAltitudeDrop, minAltitudeDrop);
  appendFile(SD_MMC, "/log.txt", dataString);
  
  if (parachuteStatus == "armed" && altitudeDrop >= altitudeDropThreshold) {
      parachuteRelease();
      parachuteStatus = "released";
  }
  lastAltitude = relativeAltitude;
  
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
      object["BMP280Pressure"] = pressure;
      object["MPU6050Temp"] = temp.temperature;
      object["Accelerometer"] = String(a.acceleration.x) + ", " + String(a.acceleration.y) + ", " + String(a.acceleration.z);
      object["Gyroscope"] = String(g.gyro.x) + ", " + String(g.gyro.y) + ", " + String(g.gyro.z);
      object["ParachuteStatus"] = parachuteStatus;
      object["LocalPressure"] = lastLocalPressure;
      object["DefaultSeaLevelPressure"] = defaultSeaLevelPressure;
      object["APIStatus"] = apiStatus;
      object["MaxAbsAltitude"] = maxAbsoluteAltitude;
      object["MinAbsAltitude"] = minAbsoluteAltitude;
      object["MaxRelAltitude"] = maxRelativeAltitude;
      object["MinRelAltitude"] = minRelativeAltitude;
      serializeJson(doc, jsonString);
      Serial.println(jsonString);
      webSocket.broadcastTXT(jsonString);
      previousMillis = nowMillis;
  }
  delay(100);
}

// -----------------------
// getTimeStampString Function
// -----------------------
String getTimeStampString() {
    unsigned long rawtime = timeClient.getEpochTime();
    if (rawtime < 100000 && lastSuccessfulNTP != 0) {
        rawtime = lastSuccessfulNTP + (millis() - lastSyncMillis) / 1000;
    }
    time_t t = (time_t)rawtime;
    struct tm *ti = localtime(&t);
    String yearStr = String(ti->tm_year + 1900);
    String monthStr = (ti->tm_mon + 1) < 10 ? "0" + String(ti->tm_mon + 1) : String(ti->tm_mon + 1);
    String dayStr = ti->tm_mday < 10 ? "0" + String(ti->tm_mday) : String(ti->tm_mday);
    String hoursStr = ti->tm_hour < 10 ? "0" + String(ti->tm_hour) : String(ti->tm_hour);
    String minuteStr = ti->tm_min < 10 ? "0" + String(ti->tm_min) : String(ti->tm_min);
    String secondStr = ti->tm_sec < 10 ? "0" + String(ti->tm_sec) : String(ti->tm_sec);
    return yearStr + "-" + monthStr + "-" + dayStr + " " + hoursStr + ":" + minuteStr + ":" + secondStr;
}

// -----------------------
// parachuteRelease Function
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
// parachuteArmed Function
// -----------------------
void parachuteArmed() {
  Serial.println("Arming parachute...");
  char eventLog[128];
  String eventTimestamp = getTimeStampString();
  snprintf(eventLog, sizeof(eventLog), "Timestamp: %s, Event: Parachute Armed!\n", eventTimestamp.c_str());
  appendFile(SD_MMC, "/log.txt", eventLog);
  if (!baselineCaptured) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);
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

// -----------------------
// webSocketEvent Function
// -----------------------
void webSocketEvent(byte num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("Client " + String(num) + " disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("Client " + String(num) + " connected");
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
      }
      break;
  }
}
