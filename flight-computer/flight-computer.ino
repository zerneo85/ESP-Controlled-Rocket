/*
 * Flight Computer Firmware for ESP32 - Version V3.2.0
 *
 * Summary of Changes and Improvements:
 * - Sensors can now be calibrated separately via a new "Calibrate Sensors" button.
 *   When calibration is initiated while the parachute status is "armed", the status is
 *   updated to "calibrating". Pressing "Arm" afterwards resets the status back to "armed".
 * - Calibration resets both the baseline altitude (for relative altitude) and the accelerometer 
 *   offsets for the X, Y, and Z axes.
 * - The accelerometer display on the web interface now shows X, Y and Z separately.
 * - LED color definitions have been updated with explicit WS2812 wheel values:
 *     • Red (0)    for armed,
 *     • Blue (170) for unarmed,
 *     • Purple (210) for released,
 *     • Green (85) for WiFi connected,
 *     • Orange (40) for AP mode,
 *     • Aqua Turquoise (125) for calibration feedback.
 * - The web interface remains a one-liner compressed HTML.
 * - Code comments have been expanded and clarified for ease of maintenance and for GitHub release documentation.
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
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <SD_MMC.h>
#include "sd_read_write.h"
#include <ESP32Servo.h>
#include <WiFi.h>
extern "C" {
  #include "esp_wifi.h" // For esp_wifi_set_ps()
}
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Arduino_ESP32_OTA.h>
#include <HTTPUpdateServer.h>
#include <ESPmDNS.h>
#include <EEPROM.h>
#include <math.h>

// -----------------------
// LED Circle Setup using Freenove_WS2812 Library
// -----------------------
#include "Freenove_WS2812_Lib_for_ESP32.h"
#define LEDS_COUNT 12       // Number of LEDs in the ring
#define LEDS_PIN 17         // GPIO pin for LED data
#define CHANNEL 0           // LED channel (typically 0)
Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);

// -----------------------
// Macro Definitions
// -----------------------
#define EEPROM_SIZE 10
#define EEPROM_PRESSURE_ADDR 0

// -----------------------
// Global Variables
// -----------------------
// SD card storage variables
uint64_t totalSpace;               // Total SD card space (in MB)
uint64_t usedSpace;                // Used SD card space (in MB)

// Pressure variables
String PressureSource = "";
float lastLocalPressure = 1026.0;  // Default sea-level pressure in hPa
bool apiPressureUpdated = false;
bool showSensorInitLog = true;
bool bmpFound = false;
bool mpuFound = false;

// WiFi Credentials and AP settings
const char* ssid = "TDGC-Rocket";
const char* wifiPassword = "Rocket2022!";
const char* apSSID = "RocketAP";
const char* apPassword = "Rocket2022!";

// OpenWeatherMap API settings
const char* openWeatherMapApiKey = "API_KEY";
const char* lat = "LAT";
const char* lon = "LON";
const char* owmEndpoint = "https://api.openweathermap.org/data/3.0/onecall";

// -----------------------
// Time Setup (NTP)
// -----------------------
#define UTC_OFFSET_IN_SECONDS 3600
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", UTC_OFFSET_IN_SECONDS);
unsigned long lastSuccessfulNTP = 0;
unsigned long lastSyncMillis = 0;

// -----------------------
// Trigger & Calibration Variables
// -----------------------
float altitudeDropThreshold = 0.8; // Trigger threshold for altitude drop
float accXThreshold = 15.0;        // Trigger threshold for accelerometer X
float accYThreshold = 15.0;        // Trigger threshold for accelerometer Y
float accZThreshold = 15.0;        // Trigger threshold for accelerometer Z
bool useAndLogic = false;          // false = OR logic; true = AND logic for triggers

// Baseline and status variables for altitude and parachute
float baselineAltitude = 0;
bool baselineCaptured = false;
String parachuteStatus = "unarmed";  // Possible values: "unarmed", "armed", "released", "calibrating"

// Tracking variables for logging extremes
float maxAbsoluteAltitude = -1000000.0;
float minAbsoluteAltitude = 1000000.0;
float maxRelativeAltitude = -1000000.0;
float minRelativeAltitude = 1000000.0;
float maxAltitudeDrop = -1000000.0;
float minAltitudeDrop = 1000000.0;
float armedMaxRelativeAltitude = 0;

// Accelerometer calibration offsets (to zero relative measurements)
float accXOffset = 0, accYOffset = 0, accZOffset = 0;

// -----------------------
// LED Color Global Variables (explicitly assigned via WS2812 Wheel values)
// -----------------------
uint32_t redColor, blueColor, purpleColor, greenColor, aquaColor, orangeColor;

// -----------------------
// Sensor & Servo Instances (declared early for global access)
// -----------------------
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
Servo parachuteservo;
int servoPin = 14;

// -----------------------
// Web Server & OTA Instances
// -----------------------
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
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
// LED Helper Functions
// -----------------------

// Initializes LED colors using explicit WS2812 wheel values.
void initLEDColors() {
  redColor    = strip.Wheel(0);    // Armed: Red
  blueColor   = strip.Wheel(170);  // Unarmed: Blue
  purpleColor = strip.Wheel(210);  // Released: Purple
  greenColor  = strip.Wheel(85);   // WiFi Connected: Green
  aquaColor   = strip.Wheel(125);  // Calibration Feedback: Aqua Turquoise
  orangeColor = strip.Wheel(40);   // AP Mode: Orange
}

// Displays a sequential LED pattern using the defined colors.
void showLEDColorsSequentially() {
  for (int i = 0; i < LEDS_COUNT; i++) { strip.setLedColorData(i, redColor); strip.show(); delay(100); }
  delay(500);
  for (int i = 0; i < LEDS_COUNT; i++) { strip.setLedColorData(i, purpleColor); strip.show(); delay(100); }
  delay(500);
  for (int i = 0; i < LEDS_COUNT; i++) { strip.setLedColorData(i, blueColor); strip.show(); delay(100); }
  delay(500);
}

// Blinks the LED ring with a specified color a given number of times.
void blinkColor(uint32_t color, int times, int delayms) {
  for (int t = 0; t < times; t++) {
    for (int i = 0; i < LEDS_COUNT; i++) { strip.setLedColorData(i, color); }
    strip.show();
    delay(delayms);
    for (int i = 0; i < LEDS_COUNT; i++) { strip.setLedColorData(i, 0); }
    strip.show();
    delay(delayms);
  }
}

// Provides visual feedback on WiFi status using LEDs.
void indicateWiFiStatus(bool connected) {
  if (connected) { blinkColor(greenColor, 3, 250); }
  else { blinkColor(orangeColor, 3, 250); }
}

// -----------------------
// Utility Functions
// -----------------------

// Returns a formatted timestamp string based on NTP or local time.
String getTimeStampString() {
  if (lastSuccessfulNTP != 0) {
    unsigned long currentEpoch = lastSuccessfulNTP + ((millis() - lastSyncMillis) / 1000);
    time_t t = (time_t)currentEpoch;
    struct tm* ti = localtime(&t);
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

// Returns the latest local sea-level pressure.
float getLocalSeaLevelPressure() { return lastLocalPressure; }

// Retrieves and updates local pressure via the OpenWeatherMap API.
void updatePressureFromAPI() {
  float localPressure = 1026.0;
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
        localPressure = doc["current"]["pressure"] | 1026.0;
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
    if (storedPressure > 500.0 && storedPressure < 1100.0) { localPressure = storedPressure; PressureSource = "EEPROM"; }
    else { localPressure = 1026.0; PressureSource = "Default Sea-Level"; }
  }
  lastLocalPressure = localPressure;
  apiPressureUpdated = true;
}

// -----------------------
// Actuation Functions
// -----------------------

// Releases the parachute by actuating the servo and providing LED feedback.
// Also logs the event to the SD card.
void parachuteRelease() {
  Serial.println("Trigger condition met! Releasing parachute...");
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
  // Provide visual feedback using green LEDs after release.
  blinkColor(greenColor, 5, 250);
  for (int i = 0; i < LEDS_COUNT; i++) { strip.setLedColorData(i, greenColor); }
  strip.show();
}

// Arms the parachute by capturing the current altitude as baseline (if not yet calibrated)
// and actuating the servo, then sets the status to "armed".
void parachuteArmed() {
  Serial.println("Arming parachute...");
  char eventLog[128];
  String eventTimestamp = getTimeStampString();
  snprintf(eventLog, sizeof(eventLog), "Timestamp: %s, Event: Parachute Armed!\n", eventTimestamp.c_str());
  appendFile(SD_MMC, "/log.txt", eventLog);
  // If calibration has not been performed, capture baseline altitude.
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
  // Actuate the servo to signal arming.
  parachuteservo.write(0);
  delay(200);
  parachuteservo.write(180);
  parachuteStatus = "armed";
  for (int i = 0; i < LEDS_COUNT; i++) { strip.setLedColorData(i, redColor); }
  strip.show();
}

// Calibrates the sensors by reading the current altitude and accelerometer values.
// If the parachute was previously armed, its status is updated to "calibrating" for the duration
// of the calibration process. After calibration, the user must re-arm to re-establish the "armed" state.
void calibrateSensors() {
  Serial.println("Calibrating sensors...");
  // If the parachute was armed, update its status to "calibrating"
  if (parachuteStatus == "armed") {
    parachuteStatus = "calibrating";
    Serial.println("Parachute status updated to 'calibrating' due to sensor calibration.");
  }
  // Visual feedback for calibration: cycle through purple LED pattern.
  for (int i = 0; i < LEDS_COUNT; i++) { strip.setLedColorData(i, purpleColor); strip.show(); delay(100); }
  // Capture the current altitude as the new baseline.
  baselineAltitude = bmp.readAltitude(getLocalSeaLevelPressure());
  baselineCaptured = true;
  // Capture current accelerometer values to set offsets (zeroing relative measurements).
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  accXOffset = a.acceleration.x;
  accYOffset = a.acceleration.y;
  accZOffset = a.acceleration.z;
  // Reset tracking variables.
  maxRelativeAltitude = 0;
  minRelativeAltitude = 1000000.0;
  maxAltitudeDrop = 0;
  minAltitudeDrop = 1000000.0;
  Serial.print("Calibration complete. Baseline altitude: ");
  Serial.println(baselineAltitude);
  // Provide additional LED feedback using purple color.
  blinkColor(purpleColor, 5, 250);
}

// -----------------------
// WebSocket Event Handler
// -----------------------
// Processes commands received via WebSocket. Supported commands include updating trigger thresholds,
// initiating calibration, and arming or releasing the parachute.
void webSocketEvent(byte num, WStype_t type, uint8_t* payload, size_t length) {
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
      if (error) { Serial.print(F("deserializeJson() failed: ")); Serial.println(error.f_str()); return; }
      else {
        // Update trigger thresholds based on incoming JSON.
        if (doc.containsKey("newThreshold")) {
          altitudeDropThreshold = doc["newThreshold"];
          Serial.print("New Altitude Drop Threshold set: ");
          Serial.println(altitudeDropThreshold);
        }
        if (doc.containsKey("newAccX")) {
          accXThreshold = doc["newAccX"];
          Serial.print("New Accelerometer X threshold set: ");
          Serial.println(accXThreshold);
        }
        if (doc.containsKey("newAccY")) {
          accYThreshold = doc["newAccY"];
          Serial.print("New Accelerometer Y threshold set: ");
          Serial.println(accYThreshold);
        }
        if (doc.containsKey("newAccZ")) {
          accZThreshold = doc["newAccZ"];
          Serial.print("New Accelerometer Z threshold set: ");
          Serial.println(accZThreshold);
        }
        if (doc.containsKey("newTriggerLogic")) {
          String newLogic = doc["newTriggerLogic"];
          useAndLogic = (newLogic == "AND");
          Serial.print("New Trigger Logic set: ");
          Serial.println(useAndLogic ? "AND" : "OR");
        }
        // If calibration is requested, perform sensor calibration.
        if (doc.containsKey("calibrateSensors")) {
          calibrateSensors();
          Serial.println("Sensors calibrated.");
        }
        // Process parachute commands.
        const char* command = doc["parachute"];
        Serial.println("Received command from client " + String(num));
        if (String(command) == "Armed" && parachuteStatus != "armed") {
          parachuteArmed();
          Serial.println("Parachute armed command processed.");
        } else if (String(command) == "Released") {
          parachuteRelease();
          Serial.println("Parachute release command processed.");
        }
      }
      Serial.println("");
      break;
    }
  }
}

// -----------------------
// Compressed Web Page (One-liner HTML)
// -----------------------
// The HTML web interface includes tables displaying sensor data and buttons for arming, releasing,
// calibrating, and updating trigger thresholds. This is provided as a one-liner string.
String webpage = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'><title>Flight Computer</title><style>body{background-color:#EEEEEE;font-family:Arial,sans-serif;color:#003366;margin:0;padding:20px}h1{text-align:center;margin-bottom:20px}.data-table{margin:0 auto;border-collapse:collapse;width:90%;max-width:600px;background-color:#FFF;box-shadow:0 0 10px rgba(0,0,0,0.1)}.data-table th,.data-table td{padding:12px 15px;border:1px solid #CCC;text-align:left}.data-table th{background-color:#003366;color:#FFF}.data-table tr:nth-child(even){background-color:#F9F9F9}.button-container{text-align:center;margin-top:20px}button{background-color:#003366;color:#FFF;border:none;padding:10px 20px;font-size:16px;cursor:pointer}button:hover{background-color:#0055AA}input[type='number'],select{padding:8px;font-size:16px;width:150px;margin-right:10px;margin-top:5px}</style></head><body><h1>Flight Information</h1><table class='data-table'><tr><th>Parameter</th><th>Value</th></tr><tr><td>Absolute Altitude</td><td id='AbsoluteAltitude'>-</td></tr><tr><td>Relative Altitude</td><td id='RelativeAltitude'>-</td></tr><tr><td>Altitude Drop</td><td id='AltitudeDrop'>-</td></tr><tr><td>BMP280 Temp</td><td id='BMP280Temp'>-</td></tr><tr><td>BMP280 Pressure</td><td id='BMP280Pressure'>-</td></tr><tr><td>MPU6050 Temp</td><td id='MPU6050Temp'>-</td></tr><tr><td>Acc X</td><td id='AccX'>-</td></tr><tr><td>Acc Y</td><td id='AccY'>-</td></tr><tr><td>Acc Z</td><td id='AccZ'>-</td></tr><tr><td>Gyroscope</td><td id='Gyroscope'>-</td></tr><tr><td>Parachute Status</td><td id='ParachuteStatus'>-</td></tr><tr><td>Local Pressure</td><td id='LocalPressure'>-</td></tr><tr><td>Default Sea-Level Pressure</td><td id='DefaultSeaLevelPressure'>-</td></tr><tr><td>Pressure Source</td><td id='PressureSource'>-</td></tr><tr><td>Max Abs Altitude</td><td id='MaxAbsAltitude'>-</td></tr><tr><td>Min Abs Altitude</td><td id='MinAbsAltitude'>-</td></tr><tr><td>Max Rel Altitude</td><td id='MaxRelAltitude'>-</td></tr><tr><td>Min Rel Altitude</td><td id='MinRelAltitude'>-</td></tr><tr><td>Max Alt Drop</td><td id='MaxAltDrop'>-</td></tr><tr><td>Min Alt Drop</td><td id='MinAltDrop'>-</td></tr><tr><td>Altitude Drop Threshold</td><td id='AltDropThreshold'>-</td></tr><tr><td>Acc X Threshold</td><td id='AccXThreshold'>-</td></tr><tr><td>Acc Y Threshold</td><td id='AccYThreshold'>-</td></tr><tr><td>Acc Z Threshold</td><td id='AccZThreshold'>-</td></tr><tr><td>Trigger Logic</td><td id='TriggerLogic'>-</td></tr><tr><td>Total Space (MB)</td><td id='TotalSpace'>-</td></tr><tr><td>Used Space (MB)</td><td id='UsedSpace'>-</td></tr></table><div class='button-container'><button type='button' id='BTN_ARM'>Arm Parachute</button> <button type='button' id='BTN_RELEASE'>Release Parachute</button> <button type='button' id='BTN_CALIBRATE'>Calibrate Sensors</button></div><div class='button-container'><input type='number' step='0.1' id='newAltThreshold' placeholder='Altitude Drop Threshold' value='0.8'><br><input type='number' step='0.1' id='newAccX' placeholder='Acc X Threshold' value='15.0'><br><input type='number' step='0.1' id='newAccY' placeholder='Acc Y Threshold' value='15.0'><br><input type='number' step='0.1' id='newAccZ' placeholder='Acc Z Threshold' value='15.0'><br><select id='triggerLogic'><option value='OR'>OR</option><option value='AND'>AND</option></select><br><button type='button' id='BTN_UPDATE_TRIGGERS'>Update Triggers</button></div><script>var Socket;document.getElementById('BTN_ARM').addEventListener('click', button_arm);document.getElementById('BTN_RELEASE').addEventListener('click', button_release);document.getElementById('BTN_UPDATE_TRIGGERS').addEventListener('click', button_update_triggers);document.getElementById('BTN_CALIBRATE').addEventListener('click', button_calibrate);function init(){Socket=new WebSocket('ws://'+window.location.hostname+':81/');Socket.onmessage=function(event){processCommand(event);};}function button_arm(){var msg={ parachute:'Armed' };Socket.send(JSON.stringify(msg));}function button_release(){var msg={ parachute:'Released' };Socket.send(JSON.stringify(msg));}function button_update_triggers(){var altVal=parseFloat(document.getElementById('newAltThreshold').value);var accXVal=parseFloat(document.getElementById('newAccX').value);var accYVal=parseFloat(document.getElementById('newAccY').value);var accZVal=parseFloat(document.getElementById('newAccZ').value);var logicVal=document.getElementById('triggerLogic').value;var msg={ newThreshold:altVal, newAccX:accXVal, newAccY:accYVal, newAccZ:accZVal, newTriggerLogic:logicVal };Socket.send(JSON.stringify(msg));}function button_calibrate(){var msg={ calibrateSensors:true };Socket.send(JSON.stringify(msg));}function processCommand(event){var obj=JSON.parse(event.data);document.getElementById('AbsoluteAltitude').innerHTML=obj.AbsoluteAltitude||'-';document.getElementById('RelativeAltitude').innerHTML=obj.RelativeAltitude||'-';document.getElementById('AltitudeDrop').innerHTML=obj.AltitudeDrop||'-';document.getElementById('BMP280Temp').innerHTML=obj.BMP280Temp||'-';document.getElementById('BMP280Pressure').innerHTML=obj.BMP280Pressure||'-';document.getElementById('MPU6050Temp').innerHTML=obj.MPU6050Temp||'-';document.getElementById('AccX').innerHTML=obj.AccX||'-';document.getElementById('AccY').innerHTML=obj.AccY||'-';document.getElementById('AccZ').innerHTML=obj.AccZ||'-';document.getElementById('Gyroscope').innerHTML=obj.Gyroscope||'-';document.getElementById('ParachuteStatus').innerHTML=obj.ParachuteStatus||'-';document.getElementById('LocalPressure').innerHTML=obj.LocalPressure||'-';document.getElementById('DefaultSeaLevelPressure').innerHTML=obj.DefaultSeaLevelPressure||'-';document.getElementById('PressureSource').innerHTML=obj.PressureSource||'-';document.getElementById('MaxAbsAltitude').innerHTML=obj.MaxAbsAltitude||'-';document.getElementById('MinAbsAltitude').innerHTML=obj.MinAbsAltitude||'-';document.getElementById('MaxRelAltitude').innerHTML=obj.MaxRelAltitude||'-';document.getElementById('MinRelAltitude').innerHTML=obj.MinRelAltitude||'-';document.getElementById('MaxAltDrop').innerHTML=obj.MaxAltDrop||'-';document.getElementById('MinAltDrop').innerHTML=obj.MinAltDrop||'-';document.getElementById('AltDropThreshold').innerHTML=obj.AltDropThreshold||'-';document.getElementById('AccXThreshold').innerHTML=obj.AccXThreshold||'-';document.getElementById('AccYThreshold').innerHTML=obj.AccYThreshold||'-';document.getElementById('AccZThreshold').innerHTML=obj.AccZThreshold||'-';document.getElementById('TriggerLogic').innerHTML=obj.TriggerLogic||'-';document.getElementById('TotalSpace').innerHTML=obj.TotalSpace||'-';document.getElementById('UsedSpace').innerHTML=obj.UsedSpace||'-';}window.onload=function(event){init();}</script></body></html>";

// -----------------------
// Setup Function
// -----------------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting setup..."));

  // Disable WiFi power saving so that peripherals (e.g., RMT, LEDs) function properly.
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Attempt to connect to WiFi as a station.
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
  if (WiFi.status() == WL_CONNECTED) { updatePressureFromAPI(); }

  EEPROM.begin(EEPROM_SIZE);
  float storedPressure;
  EEPROM.get(EEPROM_PRESSURE_ADDR, storedPressure);
  if (storedPressure > 500.0 && storedPressure < 1100.0) { 
    lastLocalPressure = storedPressure; 
    PressureSource = "EEPROM Memory"; 
  } else { 
    lastLocalPressure = 1026.0; 
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

  Serial.print("Initializing SD card...");
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  if (!SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5)) { 
    Serial.println("Card Mount Failed"); 
  } else {
    Serial.println("SD Card initialized.");
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
  totalSpace = SD_MMC.totalBytes() / (1024 * 1024);
  usedSpace = SD_MMC.usedBytes() / (1024 * 1024);
  Serial.printf("Total space: %lluMB\n", totalSpace);
  Serial.printf("Used space: %lluMB\n", usedSpace);

  Wire.begin(42, 37);

  if (mpu.begin(0x68)) { 
    mpuFound = true; 
    if (showSensorInitLog) { Serial.println("MPU6050 sensor found."); } 
  } else { 
    mpuFound = false; 
    if (showSensorInitLog) { Serial.println("Could not find MPU6050 sensor. Check wiring!"); } 
  }
  if (bmp.begin(0x76)) { 
    bmpFound = true; 
    if (showSensorInitLog) { Serial.println("BMP280 sensor found."); } 
  } else { 
    bmpFound = false; 
    if (showSensorInitLog) { Serial.println("Could not find BMP280 sensor. Check wiring!"); } 
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

  httpUpdater.setup(&server);
  if (MDNS.begin("esp32-webupdate")) { 
    Serial.println("MDNS responder started"); 
  }
  MDNS.addService("http", "tcp", 80);
  Serial.printf("HTTPUpdateServer ready! Open http://esp32-webupdate.local/update in your browser\n");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  parachuteservo.setPeriodHertz(50);
  parachuteservo.attach(servoPin, 1000, 2000);
  delay(1000);

  strip.begin(); 
  Serial.println("Strip Begin");
  strip.setBrightness(20); 
  Serial.println("Set Brightness");
  strip.show(); 
  Serial.println("Strip Show");
  delay(1000);
  initLEDColors(); 
  Serial.println("LED Colors Initialized");
  delay(1000);
  showLEDColorsSequentially(); 
  Serial.println("Sequential Display Done");
  delay(1000);

  Serial.println("Setup complete.");
  indicateWiFiStatus(WiFi.status() == WL_CONNECTED);
  for (int i = 0; i < LEDS_COUNT; i++) { 
    strip.setLedColorData(i, blueColor); 
  }
  strip.show();

  server.on("/", []() { server.send(200, "text/html", webpage); });
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
  if (WiFi.status() == WL_CONNECTED) { 
    if (!apiPressureUpdated) { 
      updatePressureFromAPI(); 
    } 
  } else { 
    apiPressureUpdated = false; 
  }

  totalSpace = SD_MMC.totalBytes() / (1024 * 1024);
  usedSpace = SD_MMC.usedBytes() / (1024 * 1024);

  float bmpTemp = bmp.readTemperature();
  float absoluteAltitude = bmp.readAltitude(getLocalSeaLevelPressure());
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Compute relative altitude (zeroed by the baseline)
  float relativeAltitude = (parachuteStatus == "unarmed") ? 0 : absoluteAltitude - baselineAltitude;
  if (absoluteAltitude > maxAbsoluteAltitude) maxAbsoluteAltitude = absoluteAltitude;
  if (absoluteAltitude < minAbsoluteAltitude) minAbsoluteAltitude = absoluteAltitude;
  if (relativeAltitude > maxRelativeAltitude) maxRelativeAltitude = relativeAltitude;
  if (relativeAltitude < minRelativeAltitude) minRelativeAltitude = relativeAltitude;

  float altitudeDrop = (parachuteStatus == "armed") ? armedMaxRelativeAltitude - relativeAltitude : maxRelativeAltitude - relativeAltitude;
  if (altitudeDrop > maxAltitudeDrop) maxAltitudeDrop = altitudeDrop;
  if (altitudeDrop < minAltitudeDrop) minAltitudeDrop = altitudeDrop;

  // Compute relative accelerometer values (subtracting calibration offsets)
  float relAccX = a.acceleration.x - accXOffset;
  float relAccY = a.acceleration.y - accYOffset;
  float relAccZ = a.acceleration.z - accZOffset;

  // Verbose output for debugging:
  Serial.print(getTimeStampString()); Serial.print(" BMP280 Temp: "); Serial.print(bmpTemp); Serial.println(" *C");
  Serial.print(getTimeStampString()); Serial.print(" BMP280 Pressure: "); Serial.print(bmp.readPressure()/100.0F); Serial.println(" hPa");
  Serial.print(getTimeStampString()); Serial.print(" Absolute Altitude: "); Serial.print(absoluteAltitude); Serial.println(" m");
  Serial.print(getTimeStampString()); Serial.print(" Relative Altitude: "); Serial.print(relativeAltitude); Serial.println(" m");
  Serial.print(getTimeStampString()); Serial.print(" Altitude Drop: "); Serial.print(altitudeDrop); Serial.println(" m");
  Serial.print(getTimeStampString()); Serial.print(" MPU6050 Temp: "); Serial.print(temp.temperature); Serial.println(" *C");
  Serial.print(getTimeStampString()); Serial.print(" Accelerometer (rel): "); Serial.print(relAccX); Serial.print(", "); Serial.print(relAccY); Serial.print(", "); Serial.println(relAccZ);
  Serial.print(getTimeStampString()); Serial.print(" Gyroscope: "); Serial.print(g.gyro.x); Serial.print(", "); Serial.print(g.gyro.y); Serial.print(", "); Serial.println(g.gyro.z);
  Serial.print(getTimeStampString()); Serial.print(" Local Pressure: "); Serial.print(lastLocalPressure); Serial.println(" hPa");
  Serial.print(getTimeStampString()); Serial.print(" Parachute Status: "); Serial.println(parachuteStatus);
  Serial.print(getTimeStampString()); Serial.print(" Max Abs Altitude: "); Serial.print(maxAbsoluteAltitude); Serial.print(" m, Min Abs Altitude: "); Serial.println(minAbsoluteAltitude);
  Serial.print(getTimeStampString()); Serial.print(" Max Rel Altitude: "); Serial.print(maxRelativeAltitude); Serial.print(" m, Min Rel Altitude: "); Serial.println(minRelativeAltitude);
  Serial.println("--------------------");

  // CSV-formatted output for Serial Plotter:
  Serial.print(bmpTemp,2); Serial.print(",");
  Serial.print(bmp.readPressure()/100.0F,2); Serial.print(",");
  Serial.print(absoluteAltitude,2); Serial.print(",");
  Serial.print(relativeAltitude,2); Serial.print(",");
  Serial.print(altitudeDrop,2); Serial.print(",");
  Serial.print(temp.temperature,2); Serial.print(",");
  Serial.print(relAccX,2); Serial.print(",");
  Serial.print(relAccY,2); Serial.print(",");
  Serial.print(relAccZ,2); Serial.print(",");
  Serial.print(g.gyro.x,2); Serial.print(",");
  Serial.print(g.gyro.y,2); Serial.print(",");
  Serial.println(g.gyro.z,2);

  // Log sensor data to SD card:
  char dataString[512];
  String currentTimestamp = getTimeStampString();
  snprintf(dataString, sizeof(dataString),
           "Timestamp: %s, BMP Temp: %.2f, Pressure: %.2f, Absolute Altitude: %.2f, Relative Altitude: %.2f, Altitude Drop: %.2f, MPU Temp: %.2f, Acc (rel): (%.2f; %.2f; %.2f), Gyro: (%.2f; %.2f; %.2f), Parachute Status: %s, Local Pressure: %.2f, API Status: %s, Max Abs Altitude: %.2f, Min Abs Altitude: %.2f, Max Rel Altitude: %.2f, Min Rel Altitude: %.2f, Max Alt Drop: %.2f, Min Alt Drop: %.2f, Total Space: %lluMB, Used Space: %lluMB\n",
           currentTimestamp.c_str(), bmpTemp, bmp.readPressure()/100.0F, absoluteAltitude, relativeAltitude, altitudeDrop, temp.temperature,
           relAccX, relAccY, relAccZ,
           g.gyro.x, g.gyro.y, g.gyro.z,
           parachuteStatus.c_str(), lastLocalPressure, PressureSource.c_str(),
           maxAbsoluteAltitude, minAbsoluteAltitude, maxRelativeAltitude, minRelativeAltitude, maxAltitudeDrop, minAltitudeDrop,
           totalSpace, usedSpace);
  appendFile(SD_MMC, "/log.txt", dataString);

  // Check trigger conditions using relative altitude and relative accelerometer readings.
  if (parachuteStatus == "armed") {
    bool triggerAlt = (altitudeDrop >= altitudeDropThreshold);
    bool triggerAccX = (fabs(relAccX) >= accXThreshold);
    bool triggerAccY = (fabs(relAccY) >= accYThreshold);
    bool triggerAccZ = (fabs(relAccZ) >= accZThreshold);
    bool triggerCondition = useAndLogic ? (triggerAlt && triggerAccX && triggerAccY && triggerAccZ)
                                        : (triggerAlt || triggerAccX || triggerAccY || triggerAccZ);
    if (triggerCondition) { parachuteRelease(); }
  }

  // Build JSON for WebSocket broadcast (updates the web interface in near-real time):
  static unsigned long previousMillis = 0;
  int interval = 50;
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
    object["BMP280Pressure"] = bmp.readPressure()/100.0F;
    object["MPU6050Temp"] = temp.temperature;
    object["AccX"] = relAccX;
    object["AccY"] = relAccY;
    object["AccZ"] = relAccZ;
    object["Gyroscope"] = String(g.gyro.x) + "; " + String(g.gyro.y) + "; " + String(g.gyro.z);
    object["ParachuteStatus"] = parachuteStatus;
    object["LocalPressure"] = lastLocalPressure;
    object["DefaultSeaLevelPressure"] = 1026.0;
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
    serializeJson(doc, jsonString);
    webSocket.broadcastTXT(jsonString);
    previousMillis = nowMillis;
  }
  
  delay(50);
}
