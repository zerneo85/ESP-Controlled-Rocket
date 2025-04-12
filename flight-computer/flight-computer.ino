/*
 * Flight Computer Firmware for ESP32 - Integrated, Corrected & Enhanced Version (V3.2.3)
 *
 * Summary of New Features:
 * - Visualization subpage is now served as a static file from SPIFFS.
 *   (Upload your visualization HTML, JS, and CSS under "/visualization/visualization.html" to SPIFFS.)
 * - File Manager now includes extra buttons ("Upload Files" and "Delete All Logs").
 *   The "Delete All Logs" endpoint deletes all log files (.csv and .txt) except the active log ("/log.csv").
 * - New endpoint (/fs) is added to list all files and folders from the SPIFFS filesystem.
 * - Retains sensor endpoints (/gyro, /acc, /temp, resets) and OTA update functionality.
 * - Main UI page remains a compressed one-liner with additional buttons (including "Visualization").
 * 
 * NOTE: This integrated version removes duplicate asynchronous server declarations from the FEATURE CODE.
 *       The firmware now uses a single, synchronous WebServer (and WebSocketsServer for real‐time sensor data),
 *       and file upload endpoints (SPIFFS) are handled synchronously.
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
#include <WebServer.h>          // Synchronous WebServer used for endpoints
#include <WebSocketsServer.h>   // For real-time sensor data via WebSockets
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Arduino_ESP32_OTA.h>
#include <HTTPUpdateServer.h>
#include <ESPmDNS.h>
#include <EEPROM.h>

#include <SPIFFS.h>             // For file upload & static file serving (visualization & fs browser)
#include <math.h>
#include <vector>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>







// -----------------------
// LED Circle Setup using Freenove_WS2812 Library
// -----------------------
#include "Freenove_WS2812_Lib_for_ESP32.h"
#define LEDS_COUNT 12       // Number of LEDs in the ring
#define LEDS_PIN 17         // GPIO pin for LED data
#define CHANNEL 0           // LED channel
Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);

// -----------------------
// Macro Definitions & Global Variables
// -----------------------
#define EEPROM_SIZE 10
#define EEPROM_PRESSURE_ADDR 0

// SD card storage variables
uint64_t totalSpace;     // Total SD card space (in MB)
uint64_t usedSpace;      // Used SD card space (in MB)

// Pressure variables
String PressureSource = "";
float lastLocalPressure = 1026.0;  // Default sea-level pressure (hPa)
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
float currentLatitude = 52.03323004349591;
float currentLongitude = 4.36483383178711;
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
float altitudeDropThreshold = 0.8; // Altitude drop trigger threshold
float accXThreshold = 15.0;        // Accelerometer X threshold
float accYThreshold = 15.0;        // Accelerometer Y threshold
float accZThreshold = 15.0;        // Accelerometer Z threshold
bool useAndLogic = false;          // Trigger logic: false = OR, true = AND

// Baseline & Parachute Variables
float baselineAltitude = 0;
bool baselineCaptured = false;
String parachuteStatus = "unarmed";  // ("unarmed", "armed", "released", "calibrating")
String parachutePreStatus = "unknown";

// Logging Extremes
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
// LED Color Global Variables
// -----------------------
uint32_t redColor, blueColor, purpleColor, greenColor, aquaColor, orangeColor;

// -----------------------
// Sensor & Servo Instances
// -----------------------
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
Servo parachuteservo;
int servoPin = 14;

// -----------------------
// Web Server & OTA Instances
// -----------------------
// Use a single synchronous WebServer (from MAIN CODE)
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
HTTPUpdateServer httpUpdater;

// -----------------------
// I2C Bus & SD_MMC Pin Definitions (for both sensor and SD_MMC)
// -----------------------
#define SDA_1 42
#define SCL_1 37
#define SD_MMC_CMD 38
#define SD_MMC_CLK 39
#define SD_MMC_D0 40

// -----------------------
// NEW FEATURE CODE VARIABLES (Integrated)
// -----------------------
// Integrated gyroscope reading variables (with accumulation)
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
// Gyro threshold values for update accumulation (from FEATURE CODE)
const float gyroXerror = 0.07;
const float gyroYerror = 0.03;
const float gyroZerror = 0.01;

// Global file handle for file uploads (using SPIFFS)
File uploadFile;

// --- ADD this near your other global declarations ---
File sdUploadFile;


// NOTE: The duplicate AsyncWebServer and AsyncEventSource declarations from the FEATURE CODE
// have been removed in favor of using the synchronous server (see above).

// -----------------------
// HTML form for file upload (compressed one-liner version)
// -----------------------
const char upload_form[] PROGMEM = R"rawliteral(<!DOCTYPE HTML><html><head><title>ESP32 File Upload</title><meta name='viewport' content='width=device-width, initial-scale=1'></head><body><h2>Upload File to SPIFFS</h2><form method='POST' action='/upload' enctype='multipart/form-data'><input type='file' name='upload'><input type='submit' value='Upload'></form></body></html>)rawliteral";

// -----------------------
// Main UI Web Page (One-liner compressed)
// -----------------------
String webpage = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'><title>Flight Computer</title><style>body{background-color:#EEEEEE;font-family:Arial,sans-serif;color:#003366;margin:0;padding:20px}h1{text-align:center;margin-bottom:20px}.data-table{margin:0 auto;border-collapse:collapse;width:90%;max-width:600px;background-color:#FFF;box-shadow:0 0 10px rgba(0,0,0,0.1)}.data-table th,.data-table td{padding:12px 15px;border:1px solid #CCC;text-align:left}.data-table th{background-color:#003366;color:#FFF}.data-table tr:nth-child(even){background-color:#F9F9F9}.button-container{text-align:center;margin-top:20px}button{background-color:#003366;color:#FFF;border:none;padding:10px 20px;font-size:16px;cursor:pointer}button:hover{background-color:#0055AA}input[type='number'],select{padding:8px;font-size:16px;width:150px;margin-right:10px;margin-top:5px}</style></head><body><h1>Flight Information</h1><table class='data-table'><tr><th>Parameter</th><th>Value</th></tr><tr><td>Absolute Altitude</td><td id='AbsoluteAltitude'>-</td></tr><tr><td>Relative Altitude</td><td id='RelativeAltitude'>-</td></tr><tr><td>Altitude Drop</td><td id='AltitudeDrop'>-</td></tr><tr><td>BMP280 Temp</td><td id='BMP280Temp'>-</td></tr><tr><td>BMP280 Pressure</td><td id='BMP280Pressure'>-</td></tr><tr><td>MPU6050 Temp</td><td id='MPU6050Temp'>-</td></tr><tr><td>Acc X</td><td id='AccX'>-</td></tr><tr><td>Acc Y</td><td id='AccY'>-</td></tr><tr><td>Acc Z</td><td id='AccZ'>-</td></tr><tr><td>Gyroscope</td><td id='Gyroscope'>-</td></tr><tr><td>Parachute Status</td><td id='ParachuteStatus'>-</td></tr><tr><td>Local Pressure</td><td id='LocalPressure'>-</td></tr><tr><td>Default Sea-Level Pressure</td><td id='DefaultSeaLevelPressure'>-</td></tr><tr><td>Pressure Source</td><td id='PressureSource'>-</td></tr><tr><td>Max Abs Altitude</td><td id='MaxAbsAltitude'>-</td></tr><tr><td>Min Abs Altitude</td><td id='MinAbsAltitude'>-</td></tr><tr><td>Max Rel Altitude</td><td id='MaxRelAltitude'>-</td></tr><tr><td>Min Rel Altitude</td><td id='MinRelAltitude'>-</td></tr><tr><td>Max Alt Drop</td><td id='MaxAltDrop'>-</td></tr><tr><td>Min Alt Drop</td><td id='MinAltDrop'>-</td></tr><tr><td>Altitude Drop Threshold</td><td id='AltDropThreshold'>-</td></tr><tr><td>Acc X Threshold</td><td id='AccXThreshold'>-</td></tr><tr><td>Acc Y Threshold</td><td id='AccYThreshold'>-</td></tr><tr><td>Acc Z Threshold</td><td id='AccZThreshold'>-</td></tr><tr><td>Trigger Logic</td><td id='TriggerLogic'>-</td></tr><tr><td>Total Space (MB)</td><td id='TotalSpace'>-</td></tr><tr><td>Used Space (MB)</td><td id='UsedSpace'>-</td></tr></table><div class='button-container'><button type='button' id='BTN_ARM'>Arm Parachute</button> <button type='button' id='BTN_RELEASE'>Release Parachute</button> <button type='button' id='BTN_CALIBRATE'>Calibrate Sensors</button></div><div class='button-container'><input type='number' step='0.1' id='newAltThreshold' placeholder='Altitude Drop Threshold' value='0.8'><br><input type='number' step='0.1' id='newAccX' placeholder='Acc X Threshold' value='15.0'><br><input type='number' step='0.1' id='newAccY' placeholder='Acc Y Threshold' value='15.0'><br><input type='number' step='0.1' id='newAccZ' placeholder='Acc Z Threshold' value='15.0'><br><select id='triggerLogic'><option value='OR'>OR</option><option value='AND'>AND</option></select><br><button type='button' id='BTN_UPDATE_TRIGGERS'>Update Triggers</button></div><div class='button-container'><button type='button' id='BTN_OTA' onclick=\'window.open('/update','_blank')\'>OTA Upgrade</button> <button type='button' id='BTN_FILES' onclick=\'window.open('/files','_blank')\'>Files</button> <button type='button' id='BTN_VIS' onclick=\'window.open('/visualization','_blank')\'>Visualization</button></div><script>var Socket; function init(){ Socket = new WebSocket('ws://'+window.location.hostname+':81/'); Socket.onmessage = function(event){ var obj = JSON.parse(event.data); document.getElementById('AbsoluteAltitude').innerHTML = obj.AbsoluteAltitude || '-'; document.getElementById('RelativeAltitude').innerHTML = obj.RelativeAltitude || '-'; document.getElementById('AltitudeDrop').innerHTML = obj.AltitudeDrop || '-'; document.getElementById('BMP280Temp').innerHTML = obj.BMP280Temp || '-'; document.getElementById('BMP280Pressure').innerHTML = obj.BMP280Pressure || '-'; document.getElementById('MPU6050Temp').innerHTML = obj.MPU6050Temp || '-'; document.getElementById('AccX').innerHTML = obj.AccX || '-'; document.getElementById('AccY').innerHTML = obj.AccY || '-'; document.getElementById('AccZ').innerHTML = obj.AccZ || '-'; document.getElementById('Gyroscope').innerHTML = obj.Gyroscope || '-'; document.getElementById('ParachuteStatus').innerHTML = obj.ParachuteStatus || '-'; document.getElementById('LocalPressure').innerHTML = obj.LocalPressure || '-'; document.getElementById('DefaultSeaLevelPressure').innerHTML = obj.DefaultSeaLevelPressure || '-'; document.getElementById('PressureSource').innerHTML = obj.PressureSource || '-'; document.getElementById('MaxAbsAltitude').innerHTML = obj.MaxAbsAltitude || '-'; document.getElementById('MinAbsAltitude').innerHTML = obj.MinAbsAltitude || '-'; document.getElementById('MaxRelAltitude').innerHTML = obj.MaxRelAltitude || '-'; document.getElementById('MinRelAltitude').innerHTML = obj.MinRelAltitude || '-'; document.getElementById('MaxAltDrop').innerHTML = obj.MaxAltDrop || '-'; document.getElementById('MinAltDrop').innerHTML = obj.MinAltDrop || '-'; document.getElementById('AltDropThreshold').innerHTML = obj.AltDropThreshold || '-'; document.getElementById('AccXThreshold').innerHTML = obj.AccXThreshold || '-'; document.getElementById('AccYThreshold').innerHTML = obj.AccYThreshold || '-'; document.getElementById('AccZThreshold').innerHTML = obj.AccZThreshold || '-'; document.getElementById('TriggerLogic').innerHTML = obj.TriggerLogic || '-'; document.getElementById('TotalSpace').innerHTML = obj.TotalSpace || '-'; document.getElementById('UsedSpace').innerHTML = obj.UsedSpace || '-'; }; document.getElementById('BTN_ARM').addEventListener('click', button_arm); document.getElementById('BTN_RELEASE').addEventListener('click', button_release); document.getElementById('BTN_CALIBRATE').addEventListener('click', button_calibrate); document.getElementById('BTN_UPDATE_TRIGGERS').addEventListener('click', button_update_triggers); } function button_arm(){ var msg = { parachute:'Armed' }; Socket.send(JSON.stringify(msg)); } function button_release(){ var msg = { parachute:'Released' }; Socket.send(JSON.stringify(msg)); } function button_update_triggers(){ var altVal = parseFloat(document.getElementById('newAltThreshold').value); var accXVal = parseFloat(document.getElementById('newAccX').value); var accYVal = parseFloat(document.getElementById('newAccY').value); var accZVal = parseFloat(document.getElementById('newAccZ').value); var logicVal = document.getElementById('triggerLogic').value; var msg = { newThreshold: altVal, newAccX: accXVal, newAccY: accYVal, newAccZ: accZVal, newTriggerLogic: logicVal }; Socket.send(JSON.stringify(msg)); } function button_calibrate(){ var msg = { calibrateSensors: true }; Socket.send(JSON.stringify(msg)); } window.onload = function(){ init(); }; </script></body></html>)";

// -----------------------
// NEW FEATURE: Sensor Reading Functions (with Axis Correction)
// -----------------------

// getGyroReadings() returns a JSON string with the accumulated and corrected gyroscope readings.
String getGyroReadings() {
  sensors_event_t a, g, temp;
  if (mpuFound) {
    mpu.getEvent(&a, &g, &temp);
    // Apply correction to raw gyro values
    float rawGx = g.gyro.x, rawGy = g.gyro.y, rawGz = g.gyro.z;
    float corrGx, corrGy, corrGz;
    correctAxes(rawGx, rawGy, rawGz, corrGx, corrGy, corrGz);
    if (fabs(corrGx) > gyroXerror) { gyroX += corrGx / 50.00; }
    if (fabs(corrGy) > gyroYerror) { gyroY += corrGy / 70.00; }
    if (fabs(corrGz) > gyroZerror) { gyroZ += corrGz / 90.00; }
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

// getAccReadings() returns a JSON string with the corrected accelerometer readings.
String getAccReadings() {
  sensors_event_t a, g, temp;
  StaticJsonDocument<200> doc;
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

// getTemperatureReading() returns the MPU6050 temperature (corrected) as a string.
String getTemperatureReading() {
  sensors_event_t a, g, temp;
  if (mpuFound) {
    mpu.getEvent(&a, &g, &temp);
    return String(temp.temperature);
  } else {
    return "N/A";
  }
}

// -----------------------
// NEW FEATURE: File Upload Endpoints (using SPIFFS)
// -----------------------
// Handler for file upload events (SPIFFS).
void handleFileUpload() {
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) {
      filename = "/" + filename;
    }
    Serial.printf("Upload Start: %s\n", filename.c_str());
    uploadFile = SPIFFS.open(filename, FILE_WRITE);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) {
      uploadFile.write(upload.buf, upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile) {
      uploadFile.close();
    }
    Serial.printf("Upload End: %s, %u bytes\n", upload.filename.c_str(), upload.totalSize);
  }
}


// --- ADD this new function for SD card file upload handling ---
void handleSDFileUpload() {
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    sdUploadFile = SD_MMC.open(filename.c_str(), FILE_WRITE);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (sdUploadFile) {
      sdUploadFile.write(upload.buf, upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (sdUploadFile) {
      sdUploadFile.close();
    }
    Serial.printf("SD upload complete: %s (%u bytes)\n", upload.filename.c_str(), upload.totalSize);
  }
}



// --- ADD this new function for SPIFFS file listing ---
void handleSPIFFSFileManager() {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'><title>SPIFFS File Manager</title><style>body{font-family:Arial,sans-serif;padding:20px;background:#EEE;}h1{text-align:center;}ul{list-style:none;padding:0;}li{margin:10px 0;}</style></head><body><h1>SPIFFS File Manager</h1><ul>";
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while(file) {
    String fname = file.name();
    html += "<li><a href='/spiffdownload?name=" + fname + "' target='_blank'>" + fname + "</a> (" + String(file.size()) + " bytes) ";
    html += "<button onclick=\"if(confirm('Delete file " + fname + "?')){window.location.href='/spiffdelete?name=" + fname + "';}\">Delete</button></li>";
    file.close();
    file = root.openNextFile();
  }
    html += "<button style='display:inline-block;margin-right:10px;' onclick=\"window.open('/uploadform','_blank')\">Upload to SPIFFS Files</button>";
  html += "</ul><div style='text-align:center;'><a href='/files'><button type='button'>Back</button></a></div></body></html>";
  server.send(200, "text/html", html);
}





// -----------------------
// NEW FEATURE: File Manager - Delete All Logs Endpoint
// -----------------------
// Deletes all SD_MMC files with ".csv" or ".txt" extensions except the active log "/log.csv"
void handleDeleteAllLogs() {
  int count = 0;
  // First, collect the names of files that are not "/log.csv"
  std::vector<String> filesToDelete;
  File root = SD_MMC.open("/");
  File file = root.openNextFile();
  while(file) {
    if (!file.isDirectory()) {
      String filename = file.name();
      if (!filename.equalsIgnoreCase("/log.csv")) {
        filesToDelete.push_back(filename);
      }
    }
    file.close();
    file = root.openNextFile();
  }
  // Now delete each file from the collected list
  for (size_t i = 0; i < filesToDelete.size(); i++) {
    if (SD_MMC.remove(filesToDelete[i].c_str())) {
      count++;
    }
  }
  server.send(200, "text/plain", "Deleted " + String(count) + " log file(s)");
}

// -----------------------
// File Management Endpoints (for SD_MMC) - Listing, Download & Delete
// -----------------------
void handleFiles() {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'><title>File Manager</title>";
  html += "<style>body{font-family:Arial,sans-serif;padding:20px;background-color:#EEEEEE;}h1{text-align:center;}ul{list-style-type:none;padding:0;}li{margin:10px 0;}</style>";
  
  // Extra buttons arranged in one row at the top
  html += "<div style='text-align:center;margin-bottom:20px;'>";
  html += "<a href='/spiffsfiles'><button style='display:inline-block;margin-right:10px;'>View SPIFFS Files</button></a>";
  html += "<a href='/sduploadform'><button style='display:inline-block;margin-right:10px;'>Upload to SD Card</button></a>";

  html += "<button style='display:inline-block;' onclick=\"if(confirm('Delete all log files?')){window.location.href='/deleteAllLogs';}\">Delete All Logs</button>";
  html += "</div>";

  html += "<h1>File Manager</h1><ul>";
  
  File root = SD_MMC.open("/");
  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      String filename = file.name();
      float fileSizeMB = file.size() / (1024.0 * 1024.0);
      html += "<li>";
      html += "<a href='/file?name=" + filename + "' target='_blank'>" + filename + "</a>";
      html += " - " + String(fileSizeMB, 2) + " MB ";
      html += "<button onclick=\"window.open('/downloadFile?name=" + filename + "','_blank')\">Download</button> ";
      html += "<button onclick=\"window.location.href='/deleteFile?name=" + filename + "'\">Delete</button>";
      html += "</li>";
    }
    file = root.openNextFile();
  }
  
  html += "</ul><div style='text-align:center;'><a href='/'><button type='button'>Back</button></a></div></body></html>";
  server.send(200, "text/html", html);
}


void handleSpiffDownload() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "File name not specified");
    return;
  }
  String filename = server.arg("name");
  if (filename.charAt(0) != '/') {
    filename = "/" + filename;
  }
  if (!SPIFFS.exists(filename)) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  File file = SPIFFS.open(filename, "r");
  if (!file) {
    server.send(500, "text/plain", "Failed to open file");
    return;
  }
  server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
  server.streamFile(file, "text/plain");
  file.close();
}

void handleSpiffDelete() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "File name not specified");
    return;
  }
  String filename = server.arg("name");
  if (filename.charAt(0) != '/') {
    filename = "/" + filename;
  }
  if (!SPIFFS.exists(filename)) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  SPIFFS.remove(filename);
  // Redirect to the file spiffsfiles page (refresh)
  server.sendHeader("Location", "/spiffsfiles", true);
  server.send(302, "text/plain", "");

}



void handleFileDownloadForDownload() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "File name not specified");
    return;
  }
  String filename = server.arg("name");
  if (filename.charAt(0) != '/') {
    filename = "/" + filename;
  }
  if (!SD_MMC.exists(filename.c_str())) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  File file = SD_MMC.open(filename.c_str(), "r");
  if (!file) {
    server.send(500, "text/plain", "Failed to open file");
    return;
  }
  server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
  server.streamFile(file, "text/plain");
  file.close();
}

void handleFileDelete() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "File name not specified");
    return;
  }
  String filename = server.arg("name");
  if (filename.charAt(0) != '/') {
    filename = "/" + filename;
  }
  if (!SD_MMC.exists(filename.c_str())) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  // Delete the file (no confirmation here)
  SD_MMC.remove(filename.c_str());
  // Redirect to the file manager page (refresh)
  server.sendHeader("Location", "/files", true);
  server.send(302, "text/plain", "");
}


void handleFileDownload() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "File name not specified");
    return;
  }
  String filename = server.arg("name");
  if (filename.charAt(0) != '/') {
    filename = "/" + filename;
  }
  if (!SD_MMC.exists(filename.c_str())) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  File file = SD_MMC.open(filename.c_str(), "r");
  if (!file) {
    server.send(500, "text/plain", "Failed to open file");
    return;
  }
  server.streamFile(file, "text/plain");
  file.close();
}

// -----------------------
// Utility Function: correctAxes
// -----------------------
// Corrects sensor readings due to sensor placement.
// Mapping:
//   Raw X  --> Corrected Y becomes -rawX
//   Raw Y  --> Corrected X becomes rawY
//   Raw Z  --> Remains unchanged
void correctAxes(float rawX, float rawY, float rawZ, float &corrX, float &corrY, float &corrZ) {
  corrX = rawY;      // Map raw Y to corrected X
  corrY = -rawX;     // Map raw X (with negative) to corrected Y
  corrZ = rawZ;      // Z remains unchanged
}

// -----------------------
// LED Helper Functions (remain as in working code)
// -----------------------
void initLEDColors() {
  redColor    = strip.Wheel(0);
  blueColor   = strip.Wheel(170);
  purpleColor = strip.Wheel(210);
  greenColor  = strip.Wheel(85);
  aquaColor   = strip.Wheel(125);
  orangeColor = strip.Wheel(40);
}

void setAllLEDs(uint32_t color) {
  for (int i = 0; i < LEDS_COUNT; i++) {
    strip.setLedColorData(i, color);
  }
  strip.show();
}

void showRainbowCycle(uint8_t wait, int8_t direction = 1, uint16_t rotations = 1) {
  for (uint16_t r = 0; r < rotations; r++) {
    for (uint16_t j = 0; j < 256; j++) {
      for (uint16_t i = 0; i < LEDS_COUNT; i++) {
        uint8_t wheelIndex = (((i * 256 / LEDS_COUNT) + (direction * j) + 256) % 256);
        uint32_t color = strip.Wheel(wheelIndex);
        strip.setLedColorData(i, color);
        strip.show();
        delay(wait);
      }
    }
    setAllLEDs(0);
    delay(500);
  }
}

void showLEDColorsSequentially(uint32_t color, int8_t direction = 1, uint16_t rotations = 1) {
  for (uint16_t r = 0; r < rotations; r++) {
    if (direction >= 0) {
      for (int i = 0; i < LEDS_COUNT; i++) {
        strip.setLedColorData(i, color);
        strip.show();
        delay(100);
      }
    } else {
      for (int i = LEDS_COUNT - 1; i >= 0; i--) {
        strip.setLedColorData(i, color);
        strip.show();
        delay(100);
      }
    }
    setAllLEDs(0);
    delay(500);
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

void indicateWiFiStatus(bool connected) {
  if (connected) { blinkColor(greenColor, 5, 250); }
  else { blinkColor(orangeColor, 5, 250); }
}

// -----------------------
// Utility Function: getTimeStampString
// -----------------------
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

float getLocalSeaLevelPressure() { return lastLocalPressure; }

void updatePressureFromAPI() {
  float localPressure = 1026.0;  // Default value
  bool apiSuccess = false;
  HTTPClient http;
  String url = String(owmEndpoint) + "?lat=" + String(currentLatitude, 6) +
               "&lon=" + String(currentLongitude, 6) +
               "&exclude=minutely,hourly,daily,alerts&appid=" + String(openWeatherMapApiKey);
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
        localPressure = doc["current"]["pressure"] | 1026.0;
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
      localPressure = 1026.0;
      PressureSource = "Default Sea-Level";
      Serial.print("Using Default Sea-Level Pressure: ");
      Serial.println(localPressure);
    }
  }
  
  lastLocalPressure = localPressure;
  apiPressureUpdated = true;
}

// -----------------------
// Actuation Functions
// -----------------------
void parachuteRelease() {
  Serial.println("Trigger condition met! Releasing parachute...");
  char eventLog[128];
  String eventTimestamp = getTimeStampString();
  snprintf(eventLog, sizeof(eventLog), "Timestamp: %s, Event: Parachute Released!\n", eventTimestamp.c_str());
  appendFile(SD_MMC, "/log.csv", eventLog);
  for (int i = 0; i < 2; i++) {
    parachuteservo.write(180);
    delay(50);
    parachuteservo.write(0);
  }
  parachuteStatus = "released";
  blinkColor(greenColor, 5, 250);
  setAllLEDs(greenColor);
  strip.show();
}

void parachuteArmed() {
  Serial.println("Arming parachute...");
  char eventLog[128];
  String eventTimestamp = getTimeStampString();
  snprintf(eventLog, sizeof(eventLog), "Timestamp: %s, Event: Parachute Armed!\n", eventTimestamp.c_str());
  appendFile(SD_MMC, "/log.csv", eventLog);
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
  showLEDColorsSequentially(redColor, 1, 3);
  setAllLEDs(redColor);
  strip.show();
}

void calibrateSensors() {
  Serial.println("Calibrating sensors...");
  parachutePreStatus = parachuteStatus;
  parachuteStatus = "calibrating";
  Serial.println("Parachute status set to 'calibrating' for calibration.");
  showLEDColorsSequentially(purpleColor, -1, 1);
  
  // Capture baseline altitude using BMP280
  baselineAltitude = bmp.readAltitude(getLocalSeaLevelPressure());
  baselineCaptured = true;
  
  // Calibrate accelerometer offsets with corrected axes
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
  showLEDColorsSequentially(purpleColor, 1, 1);
  delay(1000);
  parachuteStatus = parachutePreStatus;
  Serial.println("Parachute status restored post-calibration.");
}

// -----------------------
// WebSocket Event Handler
// -----------------------
void webSocketEvent(byte num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("Client " + String(num) + " disconnected");
      blinkColor(redColor, 4, 300);
      break;
    case WStype_CONNECTED:
      Serial.println("Client " + String(num) + " connected");
      blinkColor(greenColor, 4, 300);
      break;
    case WStype_TEXT: {
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
        if (doc.containsKey("newTriggerLogic")) {
          String newLogic = doc["newTriggerLogic"];
          useAndLogic = (newLogic == "AND");
          Serial.print("New Trigger Logic: ");
          Serial.println(useAndLogic ? "AND" : "OR");
        }
        if (doc.containsKey("calibrateSensors")) {
          calibrateSensors();
          Serial.println("Sensors calibrated.");
        }
        const char* command = doc["parachute"];
        Serial.println("Received parachute command from client " + String(num));
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
// Register Web Server Endpoints in setup()
// -----------------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting setup..."));

  // Begin SPIFFS for file uploads & static file serving.
  if(!SPIFFS.begin(true)){
    Serial.println("An error occurred while mounting SPIFFS");
  } else {
    Serial.println("SPIFFS mounted successfully");
  }

  // Disable WiFi power saving.
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Connect as a station.
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, wifiPassword);
  Serial.print("Connecting to WiFi");
  unsigned long startAttemptTime = millis();
  while(WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
    Serial.print(".");
    delay(500);
  }
  if(WiFi.status() == WL_CONNECTED) {
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
  if(WiFi.status() == WL_CONNECTED) { updatePressureFromAPI(); }

  EEPROM.begin(EEPROM_SIZE);
  float storedPressure;
  EEPROM.get(EEPROM_PRESSURE_ADDR, storedPressure);
  if(storedPressure > 500.0 && storedPressure < 1100.0) {
    lastLocalPressure = storedPressure;
    PressureSource = "EEPROM Memory";
  } else {
    lastLocalPressure = 1026.0;
    PressureSource = "Default Sea-Level Pressure";
  }

  timeClient.begin();
  timeClient.update();
  unsigned long currentEpoch = timeClient.getEpochTime();
  if(currentEpoch > 100000) {
    lastSuccessfulNTP = currentEpoch;
    lastSyncMillis = millis();
    Serial.print("NTP Time set to: ");
    Serial.println(getTimeStampString());
  } else {
    Serial.println("Failed to get NTP time.");
  }

  Serial.print("Initializing SD card...");
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  if(!SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5)) {
    Serial.println("Card Mount Failed");
  } else {
    Serial.println("SD Card initialized.");
String oldLogFile = "/log.csv";
String timestamp = getTimeStampString();
timestamp.replace(":", "");
timestamp.replace(" ", "_");
String newLogFile = "/" + timestamp + ".csv";  // Removed "log_" prefix
if(SD_MMC.exists(oldLogFile.c_str())) {
  if(SD_MMC.rename(oldLogFile.c_str(), newLogFile.c_str())) {
    Serial.println("Previous log file renamed to " + newLogFile);
  } else {
    Serial.println("Failed to rename log file " + oldLogFile);
  }
} else {
  Serial.println("No previous log file found.");
}


    File file = SD_MMC.open(oldLogFile.c_str(), FILE_WRITE);
    if(file) {
      file.println("Timestamp,BMP Temp,Pressure,Absolute Altitude,Relative Altitude,Altitude Drop,MPU Temp,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,Parachute Status,Local Pressure,Default Sea-Level Pressure,API Status,Max Abs Altitude,Min Abs Altitude,Max Rel Altitude,Min Rel Altitude,Max Alt Drop,Min Alt Drop,Total Space,Used Space");
      file.close();
    }
  }
  totalSpace = SD_MMC.totalBytes() / (1024 * 1024);
  usedSpace = SD_MMC.usedBytes() / (1024 * 1024);
  Serial.printf("Total space: %lluMB\n", totalSpace);
  Serial.printf("Used space: %lluMB\n", usedSpace);

  Wire.begin(SDA_1, SCL_1);

  // Initialize MPU6050 sensor
  if(mpu.begin(0x68)) {
    mpuFound = true;
    if(showSensorInitLog) { Serial.println("MPU6050 sensor found."); }
  } else {
    mpuFound = false;
    if(showSensorInitLog) { Serial.println("Could not find MPU6050 sensor. Check wiring!"); }
  }
  if(bmp.begin(0x76)) {
    bmpFound = true;
    if(showSensorInitLog) { Serial.println("BMP280 sensor found."); }
  } else {
    bmpFound = false;
    if(showSensorInitLog) { Serial.println("Could not find BMP280 sensor. Check wiring!"); }
  }
  if(mpuFound) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  if(bmpFound) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  }

  httpUpdater.setup(&server);
  if(MDNS.begin("esp32-webupdate")) {
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
  showLEDColorsSequentially(purpleColor, 1, 3);
  Serial.println("Sequential Display Done");
  delay(1000);

  Serial.println("Setup complete.");
  indicateWiFiStatus(WiFi.status() == WL_CONNECTED);
  setAllLEDs(blueColor);
  strip.show();

  // -----------------------
  // Register Web Server Endpoints (Existing & NEW)
  // -----------------------
  // Main UI page (one-liner compressed)
  server.on("/", []() { server.send(200, "text/html", webpage); });
  // New Visualization subpage endpoint (files served from SPIFFS)
server.on("/visualization", HTTP_GET, [](){ File file = SPIFFS.open("/index.html", "r"); if(!file){ server.send(404, "text/plain", "File not found"); return; } server.streamFile(file, "text/html"); file.close(); });




  // --- ADD inside setup() where you register endpoints ---
server.on("/spiffsfiles", HTTP_GET, [](){ handleSPIFFSFileManager(); });

server.on("/spiffdownload", HTTP_GET, handleSpiffDownload);
server.on("/spiffdelete", HTTP_GET, handleSpiffDelete);


  
  // File manager endpoints (SD_MMC)
  server.on("/files", []() { handleFiles(); });
  server.on("/deleteFile", HTTP_GET, []() { handleFileDelete(); });
  server.on("/downloadFile", HTTP_GET, []() { handleFileDownloadForDownload(); });
  server.on("/file", HTTP_GET, []() { handleFileDownload(); });
  // New endpoint: Delete all log files (CSV and TXT) except the active log "/log.csv"
  server.on("/deleteAllLogs", HTTP_GET, []() { handleDeleteAllLogs(); });
  
server.on("/sduploadform", HTTP_GET, [](){ server.send_P(200, "text/html", R"rawliteral(<!DOCTYPE HTML><html><head><title>SD Card Upload</title><meta name="viewport" content="width=device-width, initial-scale=1"></head><body><h2>Upload File to SD Card</h2><form method="POST" action="/sdupload" enctype="multipart/form-data"><input type="file" name="upload"><input type="submit" value="Upload"></form></body></html>)rawliteral"); }); server.on("/sdupload", HTTP_POST, [](){ server.send(200, "text/plain", "SD Card Upload Successful"); }, handleSDFileUpload);

  
  // File upload endpoints (SPIFFS)
  server.on("/uploadform", HTTP_GET, []() { server.send_P(200, "text/html", upload_form); });
  server.on("/upload", HTTP_POST, []() {
    server.send(200, "text/plain", "Upload Successful");
  }, handleFileUpload);
  
  // Sensor endpoints (with corrected axes)
  server.on("/gyro", HTTP_GET, []() { server.send(200, "application/json", getGyroReadings()); });
  server.on("/acc", HTTP_GET, []() { server.send(200, "application/json", getAccReadings()); });
  server.on("/temp", HTTP_GET, []() { server.send(200, "text/plain", getTemperatureReading()); });
  // Reset endpoints for integrated gyroscope values
  server.on("/reset", HTTP_GET, []() { gyroX = 0; gyroY = 0; gyroZ = 0; server.send(200, "text/plain", "All gyro values reset"); });
  server.on("/resetX", HTTP_GET, []() { gyroX = 0; server.send(200, "text/plain", "Gyro X reset"); });
  server.on("/resetY", HTTP_GET, []() { gyroY = 0; server.send(200, "text/plain", "Gyro Y reset"); });
  server.on("/resetZ", HTTP_GET, []() { gyroZ = 0; server.send(200, "text/plain", "Gyro Z reset"); });

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
  if(WiFi.status() == WL_CONNECTED) {
    if(!apiPressureUpdated) { updatePressureFromAPI(); }
  } else {
    apiPressureUpdated = false;
  }

  totalSpace = SD_MMC.totalBytes() / (1024 * 1024);
  usedSpace = SD_MMC.usedBytes() / (1024 * 1024);

  float bmpTemp = bmp.readTemperature();
  float absoluteAltitude = bmp.readAltitude(getLocalSeaLevelPressure());
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Compute relative altitude.
  float relativeAltitude = (parachuteStatus == "unarmed") ? 0 : absoluteAltitude - baselineAltitude;
  if(absoluteAltitude > maxAbsoluteAltitude) maxAbsoluteAltitude = absoluteAltitude;
  if(absoluteAltitude < minAbsoluteAltitude) minAbsoluteAltitude = absoluteAltitude;
  if(relativeAltitude > maxRelativeAltitude) maxRelativeAltitude = relativeAltitude;
  if(relativeAltitude < minRelativeAltitude) minRelativeAltitude = relativeAltitude;

  float altitudeDrop = (parachuteStatus == "armed") ? armedMaxRelativeAltitude - relativeAltitude : maxRelativeAltitude - relativeAltitude;
  if(altitudeDrop > maxAltitudeDrop) maxAltitudeDrop = altitudeDrop;
  if(altitudeDrop < minAltitudeDrop) minAltitudeDrop = altitudeDrop;

  // Compute relative accelerometer values with axis correction.
  float corrAx, corrAy, corrAz;
  correctAxes(a.acceleration.x, a.acceleration.y, a.acceleration.z, corrAx, corrAy, corrAz);
  float relAccX = corrAx - accXOffset;
  float relAccY = corrAy - accYOffset;
  float relAccZ = corrAz - accZOffset;

  // Verbose debugging output.
  Serial.print(getTimeStampString()); Serial.print(" BMP280 Temp: "); Serial.print(bmpTemp); Serial.println(" *C");
  Serial.print(getTimeStampString()); Serial.print(" BMP280 Pressure: "); Serial.print(bmp.readPressure()/100.0F); Serial.println(" hPa");
  Serial.print(getTimeStampString()); Serial.print(" Absolute Altitude: "); Serial.print(absoluteAltitude); Serial.println(" m");
  Serial.print(getTimeStampString()); Serial.print(" Relative Altitude: "); Serial.print(relativeAltitude); Serial.println(" m");
  Serial.print(getTimeStampString()); Serial.print(" Altitude Drop: "); Serial.print(altitudeDrop); Serial.println(" m");
  Serial.print(getTimeStampString()); Serial.print(" MPU6050 Temp: "); Serial.print(temp.temperature); Serial.println(" *C");
  Serial.print(getTimeStampString()); Serial.print(" Accelerometer (rel, corrected): ");
  Serial.print(relAccX); Serial.print(", "); Serial.print(relAccY); Serial.print(", "); Serial.println(relAccZ);
  Serial.print(getTimeStampString()); Serial.print(" Gyroscope (raw): ");
  Serial.print(g.gyro.x); Serial.print(", "); Serial.print(g.gyro.y); Serial.print(", "); Serial.println(g.gyro.z);
  Serial.print(getTimeStampString()); Serial.print(" Local Pressure: "); Serial.print(lastLocalPressure); Serial.println(" hPa");
  Serial.print(getTimeStampString()); Serial.print(" Parachute Status: "); Serial.println(parachuteStatus);
  Serial.print(getTimeStampString()); Serial.print(" Max Abs Altitude: "); Serial.print(maxAbsoluteAltitude); Serial.print(" m, Min Abs Altitude: "); Serial.println(minAbsoluteAltitude);
  Serial.print(getTimeStampString()); Serial.print(" Max Rel Altitude: "); Serial.print(maxRelativeAltitude); Serial.print(" m, Min Rel Altitude: "); Serial.println(minRelativeAltitude);
  Serial.println("--------------------");

  // CSV-formatted output for Serial Plotter.
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

  // Log sensor data to SD card as CSV.
  char dataString[512];
  String currentTimestamp = getTimeStampString();
  snprintf(dataString, sizeof(dataString),
           "\"%s\",%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,\"%s\",%.2f,1026.00,\"%s\",%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%llu,%llu\n",
           currentTimestamp.c_str(),
           bmpTemp,
           bmp.readPressure()/100.0F,
           absoluteAltitude,
           relativeAltitude,
           altitudeDrop,
           temp.temperature,
           relAccX,
           relAccY,
           relAccZ,
           g.gyro.x,
           g.gyro.y,
           g.gyro.z,
           parachuteStatus.c_str(),
           lastLocalPressure,
           PressureSource.c_str(),
           maxAbsoluteAltitude,
           minAbsoluteAltitude,
           maxRelativeAltitude,
           minRelativeAltitude,
           maxAltitudeDrop,
           minAltitudeDrop,
           totalSpace,
           usedSpace);
  appendFile(SD_MMC, "/log.csv", dataString);

  // Check trigger conditions.
  if(parachuteStatus == "armed") {
    bool triggerAlt = (altitudeDrop >= altitudeDropThreshold);
    bool triggerAccX = (fabs(relAccX) >= accXThreshold);
    bool triggerAccY = (fabs(relAccY) >= accYThreshold);
    bool triggerAccZ = (fabs(relAccZ) >= accZThreshold);
    bool triggerCondition = useAndLogic ? (triggerAlt && triggerAccX && triggerAccY && triggerAccZ)
                                        : (triggerAlt || triggerAccX || triggerAccY || triggerAccZ);
    if(triggerCondition) { parachuteRelease(); }
  }

  // Build JSON for WebSocket broadcast.
  static unsigned long previousMillis = 0;
  int interval = 50;
  unsigned long nowMillis = millis();
  if((nowMillis - previousMillis) > interval) {
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
