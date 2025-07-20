/*
 * Flight Computer Firmware for ESP32 - Integrated, Corrected & Enhanced Version (V3.5.0)
 *
 * Changes from previous versions:
 *  - Added comprehensive inline documentation to clarify code behavior.
 *  - Anonymized API key and location data (longitude and latitude).
 *  - Minor improvements to code readability and structure.
 *  - Corrected Axis mapping and user-selectable axis configuration.
 *  - **Added saving log data to both SD card and SPIFFS (with auto-disable if SPIFFS is almost full).**
 *  - Improved LED sequences to clearly indicate every state, including warnings.
 *
 * ┌──────────────────────────────────────────────────────────────────────────────┐
 * │                         LED Behavior & System Status Summary                │
 * └──────────────────────────────────────────────────────────────────────────────┘
 *
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
 *        – color: 24-bit RGB value
 *
 *      blinkColor(color, times, delayms)
 *        – color: 24-bit RGB
 *        – times: number of on/off cycles
 *        – delayms: ms between each on/off
 *
 *      showLEDColorsSequentially(color, direction, rotations)
 *        – color: 24-bit RGB
 *        – direction: +1 = ascending index, –1 = descending
 *        – rotations: full passes across all LEDs
 *
 *      showRainbowCycle(wait, direction, rotations)
 *        – wait: ms between frames
 *        – direction: +1 forward through hue wheel, –1 backward
 *        – rotations: number of full rainbow cycles
 *
 *  • Main LED Sequences & System States:
 *      – **Startup / Unarmed:**
 *          setAllLEDs(blueColor);                // Solid blue = system powered, safe
 *
 *      – **Wi-Fi status:**
 *          if connected: blinkColor(greenColor,5,250);  // 5 green blinks @250ms = WiFi OK
 *          else:        blinkColor(orangeColor,5,250); // 5 orange blinks @250ms = WiFi failed/AP mode
 *
 *      – **Parachute Armed:**
 *          showLEDColorsSequentially(redColor,  1, 3);  // Sequential red (forward), 3 rotations = arming
 *          setAllLEDs(redColor);                        // Solid red = system ARMED, ready for flight
 *
 *      – **Parachute Released:**
 *          blinkColor(greenColor,5,100);                // Fast green blink = release triggered
 *          showLEDColorsSequentially(greenColor, -1, 2);// Sequential green (reverse), 2 rotations = confirmation
 *          setAllLEDs(greenColor);                      // Solid green = parachute deployed/released
 *
 *      – **Calibrate Sensors:**
 *          showLEDColorsSequentially(orangeColor,  1, 1); // Orange forward, 1 rot = begin calibration
 *          showLEDColorsSequentially(orangeColor, -1, 1); // Orange reverse, 1 rot = finish calibration
 *
 *      – **Visualization Mode:**
 *          setAllLEDs(purpleColor);                 // Solid purple = visualization/dashboard mode active
 *
 *  • File Management, Upload/Download, and Warning Indicators:
 *      – **File Manager (/files endpoint):**
 *          setAllLEDs(orangeColor);                 // Solid orange = file management in progress
 *
 *      – **Delete File:**
 *          blinkColor(redColor, 2, 250);            // 2 slow red blinks = file deleted
 *
 *      – **Download File:**
 *          blinkColor(purpleColor, 2, 250);         // 2 purple blinks = file downloaded
 *
 *      – **Upload File (SD or SPIFFS):**
 *          blinkColor(greenColor, 2, 250);          // 2 green blinks = upload successful
 *
 *      – **SPIFFS Storage Almost Full:**
 *          animateWarningPatternLEDs();             // Alternating red/purple LEDs and warning bar on web UI
 *          Logging to SPIFFS is auto-disabled until free space restored.
 *          When resolved: solid blue (unarmed) and logging resumes.
 *
 *  • Data Logging:
 *      – All data is always logged to SD card if available.
 *      – Data is also logged to SPIFFS **as long as there is enough free space**.
 *      – If SPIFFS free space drops below threshold, logging is suspended and visual warning shown.
 *
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
#define LEDS_COUNT 41  // Number of LEDs in the strip
#define LEDS_PIN 17    // GPIO pin for the LED strip data line
#define CHANNEL 0      // PWM channel (if applicable)
//Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);
Adafruit_NeoPixel strip(LEDS_COUNT, LEDS_PIN, NEO_GRB + NEO_KHZ800);

// -----------------------
// Macro Definitions & Global Variables
// -----------------------
#define EEPROM_SIZE 10                  // EEPROM size
#define EEPROM_PRESSURE_ADDR 0          // EEPROM address for storing pressure
const size_t MAX_SPIFFS_USED_KB = 800;  // Set your preferred limit (in KB)
bool spiffsLoggingAllowed = false;
bool alreadyWarned = false;
bool apiSuccess = false;
bool debugSerial = false;         // Enable/disable serial debug printing
bool sensorModeFlight = true;     // true: Flight sensor mode; false: Visualization mode
int axisConfig = 3;               // 0 = default mapping, 1 = alternative mapping (or more states as needed)
bool sensorsCalibrated = false;   // true when calibrated
bool sensorsCalibWarned = false;  // helper for one-time warning (optional)
bool triggerAbs = true;  // Trigger on both positive and negative acceleration by default



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
const char *openWeatherMapApiKey = "xxxxxxxxxx";  // Anonymized API key
float currentLatitude = 52.42523004349591;        // Anonymized Latitude
float currentLongitude = 4.5483383178711;         // Anonymized Longitude

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
// Main Web Page String (Flight Information Page) – HTML content served to clients
// ---------------------------------------------------------------------------
const char *webpage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name='viewport' content='width=device-width,initial-scale=1'>
  <title>Flight Computer</title>
  <style>
    body {
      background-color: #EEEEEE;
      font-family: Arial, sans-serif;
      color: #003366;
      margin: 0;
      padding: 0;
    }
    h1 {
      text-align: center;
      margin-top: 24px;
      margin-bottom: 10px;
      font-size: 2.2rem;
      padding: 0 0 4px 0;
    }
    .data-columns {
      display: flex;
      justify-content: center;
      gap: 32px;
      flex-wrap: wrap;
      max-width: 1200px;
      margin: 0 auto 0 auto;
    }
    .data-table {
      flex: 1 1 0;
      min-width: 280px;
      max-width: 340px;
      background-color: #FFF;
      box-shadow: 0 0 10px rgba(0,0,0,0.08);
      border-collapse: collapse;
      margin: 0 0 30px 0;
    }
    .data-table th,
    .data-table td {
      padding: 10px 14px;
      border: 1px solid #CCC;
      text-align: left;
    }
    .data-table th {
      background-color: #003366;
      color: #FFF;
      text-align: left;
      font-size: 1rem;
    }
    .data-table tr:nth-child(even) {
      background-color: #F9F9F9;
    }
    .data-table-caption {
      background: #e4eaf2;
      font-weight: bold;
      text-align: center;
      font-size: 1.1rem;
      padding: 5px 0;
      border-radius: 5px 5px 0 0;
      letter-spacing: 1px;
    }
    /* Warning boxes */
    #spiffsWarningBox, #calibWarningBox {
      width: 100%;
      max-width: 900px;
      margin: 18px auto 14px auto;
      color: #fff;
      font-weight: bold;
      text-align: center;
      padding: 11px 0;
      border-radius: 4px;
      font-size: 1rem;
    }
    #spiffsWarningBox { background: #e53935; }
    #calibWarningBox { background: #FFA500; }

    /* Button bar */
    .button-bar {
      display: flex;
      flex-wrap: wrap;
      justify-content: center;
      gap: 10px;
      margin: 0 auto 18px auto;
      max-width: 900px;
    }
    button {
      background-color: #003366;
      color: #FFF;
      border: none;
      border-radius: 4px;
      padding: 10px 18px;
      font-size: 1rem;
      margin-bottom: 0;
      cursor: pointer;
      transition: background 0.15s;
      min-width: 110px;
    }
    button:hover {
      background-color: #0055AA;
    }

    /* Input styling */
    .controls-row {
      display: flex;
      flex-wrap: wrap;
      justify-content: center;
      gap: 10px 16px;
      margin: 0 auto 12px auto;
      max-width: 900px;
    }
    .controls-row label {
      margin-right: 2px;
      font-size: 1rem;
    }
    .controls-row input[type='number'],
    .controls-row select {
      padding: 8px;
      font-size: 1rem;
      border-radius: 4px;
      border: 1px solid #bbb;
      width: 105px;
      min-width: 65px;
      margin-right: 4px;
      margin-bottom: 2px;
    }
    /* For mobile: vertical stack */
    @media (max-width: 1050px) {
      .data-columns { gap: 16px; }
    }
    @media (max-width: 900px) {
      .data-columns { flex-direction: column; align-items: center; gap: 0; }
      .data-table { margin-bottom: 24px; }
    }
    @media (max-width: 640px) {
      h1 { font-size: 1.4rem; margin-top: 18px; margin-bottom: 7px;}
      .data-table { min-width: 98vw; max-width: 98vw; font-size: 0.99em;}
      .button-bar, .controls-row { max-width: 99vw; }
      .controls-row input[type='number'] { width: 75px; }
      .controls-row select { width: 105px; }
    }
  </style>
</head>
<body>
  <h1>Flight Computer</h1>

  <div id="spiffsWarningBox" style="display:none;">
    <span>SPIFFS WARNING: Internal storage almost full! Logging disabled. Download/delete files.</span>
  </div>
  <div id="calibWarningBox" style="display:none;">
    <span>SENSOR CALIBRATION REQUIRED: Sensors are not yet calibrated. Please calibrate before arming!</span>
  </div>

  <!-- Three equally wide, centered columns -->
  <div class="data-columns">
    <table class="data-table">
      <caption class="data-table-caption">Flight / Altitude</caption>
      <tr><th>Parameter</th><th>Value</th></tr>
      <tr><td>Absolute Altitude</td><td id='AbsoluteAltitude'>-</td></tr>
      <tr><td>Relative Altitude</td><td id='RelativeAltitude'>-</td></tr>
      <tr><td>Altitude Drop</td><td id='AltitudeDrop'>-</td></tr>
      <tr><td>Max Abs Altitude</td><td id='MaxAbsAltitude'>-</td></tr>
      <tr><td>Min Abs Altitude</td><td id='MinAbsAltitude'>-</td></tr>
      <tr><td>Max Rel Altitude</td><td id='MaxRelAltitude'>-</td></tr>
      <tr><td>Min Rel Altitude</td><td id='MinRelAltitude'>-</td></tr>
      <tr><td>Max Alt Drop</td><td id='MaxAltDrop'>-</td></tr>
      <tr><td>Min Alt Drop</td><td id='MinAltDrop'>-</td></tr>
      <tr><td>Altitude Drop Threshold</td><td id='AltDropThreshold'>-</td></tr>
      <tr><td>Latitude</td><td id="currentLatitude">-</td></tr>
      <tr><td>Longitude</td><td id="currentLongitude">-</td></tr>
      <tr><td>Axis Mapping Config</td><td id="axisConfigDisplay">-</td></tr>
    </table>
    <table class="data-table">
      <caption class="data-table-caption">Sensor Data</caption>
      <tr><th>Parameter</th><th>Value</th></tr>
      <tr><td>BMP280 Temp</td><td id='BMP280Temp'>-</td></tr>
      <tr><td>BMP280 Pressure</td><td id='BMP280Pressure'>-</td></tr>
      <tr><td>MPU6050 Temp</td><td id='MPU6050Temp'>-</td></tr>
      <tr><td>Acc X</td><td id='AccX'>-</td></tr>
      <tr><td>Acc Y</td><td id='AccY'>-</td></tr>
      <tr><td>Acc Z</td><td id='AccZ'>-</td></tr>
      <tr><td>Gyroscope</td><td id='Gyroscope'>-</td></tr>
      <tr><td>Local Pressure</td><td id='LocalPressure'>-</td></tr>
      <tr><td>Default Sea-Level Pressure</td><td id='DefaultSeaLevelPressure'>-</td></tr>
      <tr><td>Pressure Source</td><td id='PressureSource'>-</td></tr>
    </table>
    <table class="data-table">
      <caption class="data-table-caption">System / Status</caption>
      <tr><th>Parameter</th><th>Value</th></tr>
      <tr><td>Parachute Status</td><td id='ParachuteStatus'>-</td></tr>
      <tr><td>Sensors Calibrated</td><td id="SensorsCalibrated">-</td></tr>
      <tr><td>Trigger Logic</td><td id='TriggerLogic'>-</td></tr>
      <tr><td>Acc X Threshold</td><td id='AccXThreshold'>-</td></tr>
      <tr><td>Acc Y Threshold</td><td id='AccYThreshold'>-</td></tr>
      <tr><td>Acc Z Threshold</td><td id='AccZThreshold'>-</td></tr>
      <tr><td>Sensor Mode Flight</td><td id='SensorModeFlight'>-</td></tr>
      <tr><td>Total Space (MB)</td><td id='TotalSpace'>-</td></tr>
      <tr><td>Used Space (MB)</td><td id='UsedSpace'>-</td></tr>
      <tr><td>SPIFFS Total Space (KB)</td><td id='SPIFFSTotalSpace'>-</td></tr>
      <tr><td>SPIFFS Used Space (KB)</td><td id='SPIFFSUsedSpace'>-</td></tr>
    </table>
  </div>

  <!-- Button bar - all main actions in a single row -->
  <div class="button-bar">
    <button id='BTN_ARM' onclick="button_arm()">Arm Parachute</button>
    <button id='BTN_RELEASE' onclick="button_release()">Release Parachute</button>
    <button id='BTN_CALIBRATE' onclick="button_calibrate()">Calibrate Sensors</button>
    <button id='BTN_UNARM' onclick="button_unarm()">Unarm Parachute</button>
    <button id='BTN_OTA' onclick="window.open('/update','_blank')">OTA Upgrade</button>
    <button id='BTN_FILES' onclick="window.open('/files','_blank')">Files</button>
    <button id='BTN_VIS' onclick="window.open('/visualization','_blank')">Visualization</button>
  </div>

  <!-- Controls below -->
  <div class="controls-row">
    <label for="latitude">Latitude:</label>
    <input type="number" id="latitude" step="0.000001" value="52.03323004349591" placeholder="Latitude">
    <label for="longitude">Longitude:</label>
    <input type="number" id="longitude" step="0.000001" value="4.36483383178711" placeholder="Longitude">
    <button id="BTN_SET_LOCATION" onclick="button_update_location()">Set Location</button>
  </div>
<div class="controls-row">
  <label for='newAltThreshold'>Altitude Drop:</label>
  <input type='number' step='0.1' id='newAltThreshold' placeholder='Altitude Drop Threshold' value='0.8'>
  <label for='newAccX'>Acc X:</label>
  <input type='number' step='0.1' id='newAccX' placeholder='Acc X Threshold' value='15.0'>
  <label for='newAccY'>Acc Y:</label>
  <input type='number' step='0.1' id='newAccY' placeholder='Acc Y Threshold' value='15.0'>
  <label for='newAccZ'>Acc Z:</label>
  <input type='number' step='0.1' id='newAccZ' placeholder='Acc Z Threshold' value='15.0'>
  <label for="triggerAbs">Trigger on both + and - Acc</label>
  <input type="checkbox" id="triggerAbs" checked>
  <label for='triggerLogic'>Trigger Logic:</label>
  <select id='triggerLogic'><option value='OR'>OR</option><option value='AND'>AND</option></select>
  <button id='BTN_UPDATE_TRIGGERS' onclick="button_update_triggers()">Update Triggers Thresholds</button>
</div>
  <div class="controls-row">
    <label for='axisConfig'>Axis Mapping Configuration:</label>
    <select id='axisConfig'>
      <option value='0'>0 - Swap x>X and y>Z and z>Y</option>
      <option value='1'>1 - Swap x>X and y>Y and z>Z</option>
      <option value='2'>2 - Swap x>Y and y>X and z>Z</option>
      <option value='3'>3 - Swap x>Y and y>Z and z>X</option>
      <option value='4'>4 - Swap x>Z and y>Y and z>X</option>
      <option value='5'>5 - Swap x>Z and y>X and z>Y</option>
    </select>
    <button onclick="button_update_axis()">Update Axis Mapping</button>
  </div>


</body>
</html>


  <script>
    var Socket;
    function init(){
      Socket = new WebSocket('ws://' + window.location.hostname + ':81/');
      Socket.onmessage = function(event){ processCommand(event); };
    }
    function button_arm(){ Socket.send(JSON.stringify({ parachute: 'Armed' })); }
    function button_release(){ Socket.send(JSON.stringify({ parachute: 'Released' })); }
    function button_calibrate(){ Socket.send(JSON.stringify({ calibrateSensors: true })); }
    function button_unarm() { Socket.send(JSON.stringify({ parachute: 'Unarmed' }));}
    function button_update_location(){
      var lat = parseFloat(document.getElementById('latitude').value);
      var lon = parseFloat(document.getElementById('longitude').value);
      Socket.send(JSON.stringify({ latitude: lat, longitude: lon }));
    }
function button_update_triggers(){
  var altVal = parseFloat(document.getElementById('newAltThreshold').value);
  var accXVal = parseFloat(document.getElementById('newAccX').value);
  var accYVal = parseFloat(document.getElementById('newAccY').value);
  var accZVal = parseFloat(document.getElementById('newAccZ').value);
  var logicVal = document.getElementById('triggerLogic').value;
  var absVal = document.getElementById('triggerAbs').checked;
  Socket.send(JSON.stringify({
    newThreshold: altVal,
    newAccX: accXVal,
    newAccY: accYVal,
    newAccZ: accZVal,
    newTriggerLogic: logicVal,
    triggerAbs: absVal
  }));
}

    function button_update_axis(){
      var v = parseInt(document.getElementById('axisConfig').value);
      Socket.send(JSON.stringify({ axisConfig: v }));
    }
    function processCommand(event){
      var obj = JSON.parse(event.data);
      document.getElementById('AbsoluteAltitude').innerHTML = obj.AbsoluteAltitude||'-';
      document.getElementById('RelativeAltitude').innerHTML = obj.RelativeAltitude||'-';
      document.getElementById('AltitudeDrop').innerHTML = obj.AltitudeDrop||'-';
      document.getElementById('BMP280Temp').innerHTML = obj.BMP280Temp||'-';
      document.getElementById('BMP280Pressure').innerHTML = obj.BMP280Pressure||'-';
      document.getElementById('MPU6050Temp').innerHTML = obj.MPU6050Temp||'-';
      document.getElementById('AccX').innerHTML = obj.AccX||'-';
      document.getElementById('AccY').innerHTML = obj.AccY||'-';
      document.getElementById('AccZ').innerHTML = obj.AccZ||'-';
      document.getElementById('Gyroscope').innerHTML = obj.Gyroscope||'-';
      document.getElementById('ParachuteStatus').innerHTML = obj.ParachuteStatus||'-';
      document.getElementById('LocalPressure').innerHTML = obj.LocalPressure||'-';
      document.getElementById('DefaultSeaLevelPressure').innerHTML = obj.DefaultSeaLevelPressure||'-';
      document.getElementById('PressureSource').innerHTML = obj.PressureSource||'-';
      document.getElementById('MaxAbsAltitude').innerHTML = obj.MaxAbsAltitude||'-';
      document.getElementById('MinAbsAltitude').innerHTML = obj.MinAbsAltitude||'-';
      document.getElementById('MaxRelAltitude').innerHTML = obj.MaxRelAltitude||'-';
      document.getElementById('MinRelAltitude').innerHTML = obj.MinRelAltitude||'-';
      document.getElementById('MaxAltDrop').innerHTML = obj.MaxAltDrop||'-';
      document.getElementById('MinAltDrop').innerHTML = obj.MinAltDrop||'-';
      document.getElementById('AltDropThreshold').innerHTML = obj.AltDropThreshold||'-';
      document.getElementById('AccXThreshold').innerHTML = obj.AccXThreshold||'-';
      document.getElementById('AccYThreshold').innerHTML = obj.AccYThreshold||'-';
      document.getElementById('AccZThreshold').innerHTML = obj.AccZThreshold||'-';
      document.getElementById('TriggerLogic').innerHTML = obj.TriggerLogic||'-';
      document.getElementById('TotalSpace').innerHTML = obj.TotalSpace||'-';
      document.getElementById('UsedSpace').innerHTML = obj.UsedSpace||'-';
      document.getElementById('SPIFFSTotalSpace').innerHTML = obj.SPIFFSTotalSpace || '-';
      document.getElementById('SPIFFSUsedSpace').innerHTML = obj.SPIFFSUsedSpace || '-';
      document.getElementById('SensorModeFlight').innerHTML = obj.SensorModeFlight||'-';
      document.getElementById('currentLatitude').innerHTML  = (obj.Latitude  !== undefined) ? obj.Latitude  : '-';
      document.getElementById('currentLongitude').innerHTML = (obj.Longitude !== undefined) ? obj.Longitude : '-';
      document.getElementById('axisConfigDisplay').innerText = obj['Axis Config']||'-';
      document.getElementById('SensorsCalibrated').innerHTML = (obj.SensorsCalibrated === true) ? "Yes" : (obj.SensorsCalibrated === false) ? "No" : "-";

      if (obj.SpiffsWarning) {
        document.getElementById('spiffsWarningBox').style.display = '';
      } else {
        document.getElementById('spiffsWarningBox').style.display = 'none';
      }

      if (obj.SensorsCalibWarning) {
        document.getElementById('calibWarningBox').style.display = '';
      } else {
        document.getElementById('calibWarningBox').style.display = 'none';
      }
    }
    window.onload = init;
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
// Unified Upload Form with Radio Selection (uploads to SPIFFS or SD based on selection)
// ---------------------------------------------------------------------------
const char upload_form_combined[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
  <head>
    <title>ESP32 File Upload</title>
    <meta name='viewport' content='width=device-width, initial-scale=1'>
  </head>
  <body>
    <form id="uploadForm" method="POST" enctype="multipart/form-data">
      <label>Select File System:</label><br>
      <input type="radio" id="fsSpiffs" name="fs" value="SPIFFS" checked>
      <label for="fsSpiffs">SPIFFS</label><br>
      <input type="radio" id="fsSd" name="fs" value="SD">
      <label for="fsSd">SD Card</label><br><br>
      <div id="folderInput">
        <label for="targetFolder">Target Folder:</label><br>
        <input type="text" id="targetFolder" name="targetFolder" placeholder="Target folder (e.g. /myfolder)" value="/"><br>
      </div>
      <input type="file" name="upload"><br>
      <input type="submit" value="Upload">
    </form>
    <script>
      // Show or hide folder input based on selected file system
      function updateFolderInputVisibility() {
         var fsOption = document.querySelector('input[name="fs"]:checked').value;
         document.getElementById("folderInput").style.display = (fsOption === "SPIFFS") ? "block" : "none";
      }
      document.querySelectorAll('input[name="fs"]').forEach(function(radio) {
         radio.addEventListener("change", updateFolderInputVisibility);
      });
      updateFolderInputVisibility(); // Set initial visibility
      
      // Handle form submission with AJAX
      document.getElementById("uploadForm").addEventListener("submit", function(event) {
         event.preventDefault();
         var fsOption = document.querySelector('input[name="fs"]:checked').value;
         var actionUrl;
         if (fsOption === "SPIFFS") {
            var folder = document.getElementById("targetFolder").value;
            actionUrl = "/upload?targetFolder=" + encodeURIComponent(folder);
         } else {
            actionUrl = "/uploadsd";
         }
         var formData = new FormData(this);
         var xhr = new XMLHttpRequest();
         xhr.open("POST", actionUrl, true);
         xhr.onload = function() {
           if(xhr.status == 200) {
             alert("Upload Successful");
             location.reload();
           } else {
             alert("Upload Failed: " + xhr.statusText);
           }
         };
         xhr.send(formData);
      });
    </script>
  </body>
</html>
)rawliteral";

// ---------------------------------------------------------------------------
// Combined File Manager Page (Lists files from SD_MMC and SPIFFS with delete and download options)
// ---------------------------------------------------------------------------
void handleFiles() {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'><title>File Manager</title>";
  html += "<style>"
          "body { font-family: Arial, sans-serif; padding: 20px; background-color: #EEEEEE; }"
          "h1, h2 { text-align: center; }"
          "ul { list-style: none; padding: 0; }"
          "li { margin-bottom: 10px; }"
          "</style></head><body>";
  html += "<h1>File Manager</h1>";

  // JavaScript function for file deletion (handles both file systems)
  html += "<script>"
          "function deleteFile(filename, fsType) {"
          "  if (confirm('Are you sure you want to delete ' + filename + '?')) {"
          "    var endpoint = (fsType === 'SPIFFS') ? '/deleteFileSPIFFS' : '/deleteFileSD';"
          "    var xhr = new XMLHttpRequest();"
          "    xhr.open('GET', endpoint + '?name=' + encodeURIComponent(filename), true);"
          "    xhr.onload = function() {"
          "      if (xhr.status == 200) {"
          "         alert('File deleted successfully');"
          "         location.reload();"
          "      } else {"
          "         alert('Deletion failed: ' + xhr.statusText);"
          "      }"
          "    };"
          "    xhr.send();"
          "  }"
          "}"
          "</script>";

  // Include upload form and list files from SD_MMC
  html += "<h2>Upload New Files</h2>";
  html += upload_form_combined;
  html += "<h2>SD Card Files</h2><ul>";
  File rootSD = SD_MMC.open("/");
  File file = rootSD.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      String filename = file.name();
      float fileSizeMB = file.size() / (1024.0 * 1024.0);
      String lastUpdated = "N/A";
#ifdef ESP32
      time_t t = file.getLastWrite();
      if (t > 0) {
        struct tm *tmInfo = localtime(&t);
        char timeStr[26];
        strftime(timeStr, 26, "%Y-%m-%d %H:%M:%S", tmInfo);
        lastUpdated = String(timeStr);
      }
#endif
      html += "<li>" + filename + " - " + String(fileSizeMB, 2) + " MB, Last Updated: " + lastUpdated;
      html += " <button onclick=\"deleteFile('" + filename + "', 'SD')\">Delete</button>";
      html += " <button onclick=\"window.open('/downloadFile?name=" + filename + "','_blank')\">Download</button></li>";
    }
    file = rootSD.openNextFile();
  }
  html += "</ul>";

  // List files from SPIFFS
  html += "<h2>SPIFFS Files</h2><ul>";
  File rootSPIFFS = SPIFFS.open("/");
  File spFile = rootSPIFFS.openNextFile();
  while (spFile) {
    String spFilename = spFile.name();
    size_t fileSizeKB = spFile.size() / 1024;
    String lastUpdated = "N/A";  // SPIFFS does not store modification timestamps.
    html += "<li>" + spFilename + " - " + String(fileSizeKB) + " KB, Last Updated: " + lastUpdated;
    html += " <button onclick=\"deleteFile('" + spFilename + "', 'SPIFFS')\">Delete</button>";
    html += " <button onclick=\"window.open('/downloadFile?name=" + spFilename + "','_blank')\">Download</button></li>";
    spFile = rootSPIFFS.openNextFile();
  }
  html += "</ul>";

  html += "<div style='text-align:center;'><a href='/'><button type='button'>Back</button></a></div>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// ---------------------------------------------------------------------------
// Combined File Download Handler (checks both SD_MMC and SPIFFS)
// ---------------------------------------------------------------------------
void handleCombinedFileDownload() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "File name not specified");
    return;
  }
  String filename = server.arg("name");
  if (filename.charAt(0) != '/') {
    filename = "/" + filename;
  }
  if (SD_MMC.exists(filename.c_str())) {
    File file = SD_MMC.open(filename.c_str(), "r");
    if (!file) {
      server.send(500, "text/plain", "Failed to open file");
      return;
    }
    server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
    server.streamFile(file, "application/octet-stream");
    file.close();
    return;
  } else if (SPIFFS.exists(filename.c_str())) {
    File file = SPIFFS.open(filename.c_str(), "r");
    if (!file) {
      server.send(500, "text/plain", "Failed to open file");
      return;
    }
    server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
    server.streamFile(file, "application/octet-stream");
    file.close();
    return;
  } else {
    server.send(404, "text/plain", "File not found");
    return;
  }
}

// ---------------------------------------------------------------------------
// Combined File Deletion Handler (tries both SD_MMC and SPIFFS)
// ---------------------------------------------------------------------------
void handleCombinedFileDelete() {
  if (!server.hasArg("name")) {
    Serial.println("Deletion error: no file name provided");
    server.send(400, "text/plain", "File name not specified");
    return;
  }
  String filename = server.arg("name");
  if (filename.charAt(0) != '/') {
    filename = "/" + filename;
  }
  Serial.println("Attempting to delete file: " + filename);
  bool deleted = false;
  if (SD_MMC.exists(filename.c_str())) {
    if (SD_MMC.remove(filename.c_str())) {
      deleted = true;
      Serial.println("Deleted from SD: " + filename);
    } else {
      Serial.println("Failed to delete from SD: " + filename);
    }
  }
  if (SPIFFS.exists(filename.c_str())) {
    if (SPIFFS.remove(filename.c_str())) {
      deleted = true;
      Serial.println("Deleted from SPIFFS: " + filename);
    } else {
      Serial.println("Failed to delete from SPIFFS: " + filename);
    }
  }
  if (deleted) {
    server.send(200, "text/plain", "File deleted successfully");
  } else {
    server.send(500, "text/plain", "Failed to delete file");
  }
}

// ---------------------------------------------------------------------------
// Updated SPIFFS File Upload Handler (uses 'targetFolder' parameter for path)
// ---------------------------------------------------------------------------
void handleFileUpload() {
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    // Get the target folder from the query string
    String folder = server.arg("targetFolder");
    String filename = upload.filename;
    if (!filename.startsWith("/")) {
      filename = "/" + filename;
    }
    // Prepend folder (if not root) to filename
    if (folder != "" && folder != "/") {
      if (!folder.endsWith("/")) {
        folder += "/";
      }
      filename = folder + filename;
    }
    Serial.printf("SPIFFS Upload Start: %s\n", filename.c_str());
    uploadFile = SPIFFS.open(filename, FILE_WRITE);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) {
      uploadFile.write(upload.buf, upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile) {
      uploadFile.close();
      Serial.printf("SPIFFS Upload End: %s, %u bytes\n", upload.filename.c_str(), upload.totalSize);
    }
  }
}

// ---------------------------------------------------------------------------
// SD Card File Upload Handler (uploads file to the SD card root)
// ---------------------------------------------------------------------------
void handleFileUploadSD() {
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) {
      filename = "/" + filename;
    }
    Serial.printf("SD Upload Start: %s\n", filename.c_str());
    File sdFile = SD_MMC.open(filename, FILE_WRITE);
    if (!sdFile) {
      Serial.println("Failed to open file on SD card");
      return;
    }
    uploadFile = sdFile;
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) {
      uploadFile.write(upload.buf, upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile) {
      uploadFile.close();
      Serial.printf("SD Upload End: %s, %u bytes\n", upload.filename.c_str(), upload.totalSize);
    }
  }
}

// ---------------------------------------------------------------------------
// SPIFFS and SD File Deletion Handlers (called from file manager page)
// ---------------------------------------------------------------------------
void handleSPIFFSFileDelete() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "File name not specified");
    return;
  }
  String filename = server.arg("name");
  if (filename.charAt(0) != '/') {
    filename = "/" + filename;
  }
  Serial.println("Deleting from SPIFFS: " + filename);
  if (SPIFFS.exists(filename.c_str())) {
    if (SPIFFS.remove(filename.c_str())) {
      Serial.println("Deleted from SPIFFS: " + filename);
      server.send(200, "text/plain", "SPIFFS file deleted successfully");
    } else {
      Serial.println("Failed to delete SPIFFS file: " + filename);
      server.send(500, "text/plain", "Failed to delete SPIFFS file");
    }
  } else {
    server.send(404, "text/plain", "SPIFFS file not found");
  }
}

void handleSDFileDelete() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "File name not specified");
    return;
  }
  String filename = server.arg("name");
  if (filename.charAt(0) != '/') {
    filename = "/" + filename;
  }
  Serial.println("Deleting from SD: " + filename);
  if (SD_MMC.exists(filename.c_str())) {
    if (SD_MMC.remove(filename.c_str())) {
      Serial.println("Deleted from SD: " + filename);
      server.send(200, "text/plain", "SD file deleted successfully");
    } else {
      Serial.println("Failed to delete SD file: " + filename);
      server.send(500, "text/plain", "Failed to delete SD file");
    }
  } else {
    server.send(404, "text/plain", "SD file not found");
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
      // Mapping: Swap x>X and y>Z and z>Y
      corrX = rawX;
      corrY = rawZ;
      corrZ = rawY;
      break;
    case 1:
      // Variant 1: Mapping: Swap x>X and y>Y and z>Z
      corrX = rawX;
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
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
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
  snprintf(eventLog, sizeof(eventLog), "Timestamp: %s, Event: Parachute Released!\n", eventTimestamp.c_str());

  appendFile(SD_MMC, "/log.csv", eventLog);
  if (spiffsLoggingAllowed) {
    appendFile(SPIFFS, "/log.csv", eventLog);
  }

  for (int i = 0; i < 2; i++) {
    parachuteservo.write(180);
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

  if (!sensorsCalibrated) {
    // Flash orange LEDs as a warning
    blinkColor(orangeColor, 15, 100);  // 15 times, 80 ms on/off (adjust as needed)
    Serial.println("Cannot arm: Sensors are not calibrated!");
    return;  // Don't arm the parachute
  }

  Serial.println("Arming parachute...");

  char eventLog[128];
  String eventTimestamp = getTimeStampString();
  snprintf(eventLog, sizeof(eventLog), "Timestamp: %s, Event: Parachute Armed!\n", eventTimestamp.c_str());
  appendFile(SD_MMC, "/log.csv", eventLog);

  checkSpiffsSpaceAndWarn();
  if (spiffsLoggingAllowed) {
    appendFile(SPIFFS, "/log.csv", eventLog);
  }

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
  parachuteservo.write(0);
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
  baselineCaptured = false;  // Optional: Clear baseline so it's recalculated next time
  if (!alreadyWarned) {
    setAllLEDs(blueColor);  // Blue = unarmed/safe
  } else {
    setWarningPatternLEDs();  // If SPIFFS warning active, keep warning pattern
  }
  Serial.println("Parachute is now UNARMED. Logging to SPIFFS will stop unless armed.");
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
      file.println("Timestamp,BMP Temp,Pressure,Absolute Altitude,Relative Altitude,Altitude Drop,MPU Temp,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,Parachute Status,Local Pressure,Default Sea-Level Pressure,API Status,Max Abs Altitude,Min Abs Altitude,Max Rel Altitude,Min Rel Altitude,Max Alt Drop,Min Alt Drop,Total Space,Used Space");
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
    sensorModeFlight = true;  // ← switch back into flight‐mode
    setAllLEDs(blueColor);    // ← (optional) restore flight-mode LED color
    server.send(200, "text/html", webpage);
  });

  // Inline visualization‐mode handler:
  server.on("/visualization", HTTP_GET, []() {
    sensorModeFlight = false;  // enter visualization mode
    setAllLEDs(purpleColor);   // paint LEDs purple
    // strip.show();             // only if your setAllLEDs() doesn't call show()
    server.sendHeader("Location", "/visualization/index.html");
    server.send(302, "text/plain", "Redirecting...");
  });

  // Serve the static visualization files from SPIFFS
  server.serveStatic("/visualization/", SPIFFS, "/");


  // File management endpoints: file listing, upload, deletion, and download for both SD and SPIFFS.
  server.on("/files", []() {
    setAllLEDs(orangeColor);  // ← (optional) set maintance color LED color
    handleFiles();
  });
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
  server.on(
    "/upload", HTTP_POST, []() {
      blinkColor(greenColor, 2, 250);
      server.send(200, "text/plain", "SPIFFS Upload Successful");
    },
    handleFileUpload);
  server.on(
    "/uploadsd", HTTP_POST, []() {
      blinkColor(greenColor, 2, 250);
      server.send(200, "text/plain", "SD Card Upload Successful");
    },
    handleFileUploadSD);

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

  // Update pressure from API if connected, or flag for future update if not
  if (WiFi.status() == WL_CONNECTED) {
    if (!apiPressureUpdated) {
      updatePressureFromAPI();
    }
  } else {
    apiPressureUpdated = false;
  }

  // Update SD card space statistics
  totalSpace = SD_MMC.totalBytes() / (1024 * 1024);
  usedSpace = SD_MMC.usedBytes() / (1024 * 1024);
  // Update SPIFF card space statistics
  size_t spiffsTotal = SPIFFS.totalBytes() / 1024;
  size_t spiffsUsed = SPIFFS.usedBytes() / 1024;



  // Read sensor values from BMP and MPU
  float bmpTemp = bmp.readTemperature();
  float absoluteAltitude = bmp.readAltitude(getLocalSeaLevelPressure());
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate relative altitude based on baseline if parachute is armed
  float relativeAltitude = (parachuteStatus == "unarmed") ? 0 : absoluteAltitude - baselineAltitude;
  if (absoluteAltitude > maxAbsoluteAltitude)
    maxAbsoluteAltitude = absoluteAltitude;
  if (absoluteAltitude < minAbsoluteAltitude)
    minAbsoluteAltitude = absoluteAltitude;
  if (relativeAltitude > maxRelativeAltitude)
    maxRelativeAltitude = relativeAltitude;
  if (relativeAltitude < minRelativeAltitude)
    minRelativeAltitude = relativeAltitude;

  // Calculate altitude drop based on armed state
  float altitudeDrop = (parachuteStatus == "armed") ? armedMaxRelativeAltitude - relativeAltitude : maxRelativeAltitude - relativeAltitude;
  if (altitudeDrop > maxAltitudeDrop)
    maxAltitudeDrop = altitudeDrop;
  if (altitudeDrop < minAltitudeDrop)
    minAltitudeDrop = altitudeDrop;

  // Calculate corrected accelerometer readings subtracting calibration offsets
  float corrAx, corrAy, corrAz;
  correctAxes(a.acceleration.x, a.acceleration.y, a.acceleration.z, corrAx, corrAy, corrAz);
  float relAccX = corrAx - accXOffset;
  float relAccY = corrAy - accYOffset;
  float relAccZ = corrAz - accZOffset;

  // Print detailed sensor readings to serial if enabled
  if (debugSerial) {
    Serial.print(getTimeStampString());
    Serial.print(" BMP280 Temp: ");
    Serial.print(bmpTemp);
    Serial.println(" *C");
    Serial.print(getTimeStampString());
    Serial.print(" BMP280 Pressure: ");
    Serial.print(bmp.readPressure() / 100.0F);
    Serial.println(" hPa");
    Serial.print(getTimeStampString());
    Serial.print(" Absolute Altitude: ");
    Serial.print(absoluteAltitude);
    Serial.println(" m");
    Serial.print(getTimeStampString());
    Serial.print(" Relative Altitude: ");
    Serial.print(relativeAltitude);
    Serial.println(" m");
    Serial.print(getTimeStampString());
    Serial.print(" Altitude Drop: ");
    Serial.print(altitudeDrop);
    Serial.println(" m");
    Serial.print(getTimeStampString());
    Serial.print(" MPU6050 Temp: ");
    Serial.print(temp.temperature);
    Serial.println(" *C");
    Serial.print(getTimeStampString());
    Serial.print(" Accelerometer (rel, corrected): ");
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
    Serial.print(lastLocalPressure);
    Serial.println(" hPa");
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
    Serial.println("--------------------");
  }

  // Log sensor data to SD card en SPIFF
  char dataString[512];
  String currentTimestamp = getTimeStampString();
  snprintf(dataString, sizeof(dataString),
           "\"%s\",%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,\"%s\",%.2f,1013.25,\"%s\",%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%llu,%llu\n",
           currentTimestamp.c_str(),
           bmpTemp,
           bmp.readPressure() / 100.0F,
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
           usedSpace,
           spiffsTotal,
           spiffsUsed,
           currentLatitude,
           currentLongitude);


  checkSpiffsSpaceAndWarn();
  // Only need this once:
  appendFile(SD_MMC, "/log.csv", dataString);
  if ((parachuteStatus == "armed" || parachuteStatus == "released") && spiffsLoggingAllowed) {
    appendFile(SPIFFS, "/log.csv", dataString);
  }


  // Check trigger conditions if the system is armed to possibly deploy the parachute
  if (parachuteStatus == "armed") {
    bool triggerAlt = (altitudeDrop >= altitudeDropThreshold);
bool triggerAccX, triggerAccY, triggerAccZ;
if (triggerAbs) {
  triggerAccX = (fabs(relAccX) >= accXThreshold);
  triggerAccY = (fabs(relAccY) >= accYThreshold);
  triggerAccZ = (fabs(relAccZ) >= accZThreshold);
} else {
  triggerAccX = (relAccX >= accXThreshold);
  triggerAccY = (relAccY >= accYThreshold);
  triggerAccZ = (relAccZ >= accZThreshold);
}

    bool triggerCondition = useAndLogic ? (triggerAlt && triggerAccX && triggerAccY && triggerAccZ)
                                        : (triggerAlt || triggerAccX || triggerAccY || triggerAccZ);
    if (triggerCondition) {
      parachuteRelease();
    }
  }

  // -----------------------
  // Send sensor data over WebSocket for real-time monitoring
  // -----------------------
  if (sensorModeFlight) {
    static unsigned long previousMillis = 0;
    int interval = 40;
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
      object["AccX"] = relAccX;
      object["AccY"] = relAccY;
      object["AccZ"] = relAccZ;
      {
        // --- BEGIN MODIFICATION 1 ---
        // Instead of sending the raw gyroscope values, apply the same axis correction as used in getGyroReadings().
        float rawGx = g.gyro.x, rawGy = g.gyro.y, rawGz = g.gyro.z;
        float corrGx, corrGy, corrGz;
        correctAxes(rawGx, rawGy, rawGz, corrGx, corrGy, corrGz);
        object["Gyroscope"] = String(corrGx) + "; " + String(corrGy) + "; " + String(corrGz);
        // --- END MODIFICATION 1 ---
      }
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
      object["SensorsCalibWarning"] = !sensorsCalibrated;  // true if warning should show
      serializeJson(doc, jsonString);
      webSocket.broadcastTXT(jsonString);
      previousMillis = nowMillis;
    }
  } else {
    // In visualization mode, send separate WebSocket messages for each sensor reading
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


