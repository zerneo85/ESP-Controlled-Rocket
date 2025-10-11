// ------------------------------
// Dashboard helpers
// ------------------------------

// Null-safe setter for table cells (innerText)
function setValue(id, value) {
  const el = document.getElementById(id);
  if (!el) return; // Cel bestaat (nog) niet → stilletjes overslaan
  el.innerText = (value !== undefined && value !== null) ? value : '-';
}

// --- UI lock helpers: voorkom dat live WS updates je invoer direct overschrijven
const _uiLockUntil = {};

/**
 * Lock a form field so incoming WS frames won't overwrite it for a short time.
 * @param {string} id  element id
 * @param {number} ms  lock duration in ms (default ~1.2s)
 */
function lockField(id, ms = 1200) {
  _uiLockUntil[id] = Date.now() + ms;
}

/**
 * Set an <input> value only if the element is idle (not focused and not locked).
 * Safely no-ops if the element is missing.
 */
function setInputIfIdle(id, value) {
  const el = document.getElementById(id);
  if (!el) return;
  const now = Date.now();
  const locked = _uiLockUntil[id] && now < _uiLockUntil[id];
  if (document.activeElement === el || locked) return; // respect user typing / short lock
  el.value = (value ?? '');
}

// Main updater: match JSON keys aan HTML IDs
function updateDashboard(data) {
  // Flight / Altitude
  setValue('AbsoluteAltitude', data.AbsoluteAltitude);
  setValue('RelativeAltitude', data.RelativeAltitude);
  setValue('AltitudeDrop', data.AltitudeDrop);
  setValue('MaxAbsAltitude', data.MaxAbsAltitude);
  setValue('MinAbsAltitude', data.MinAbsAltitude);
  setValue('MaxRelAltitude', data.MaxRelAltitude);
  setValue('MinRelAltitude', data.MinRelAltitude);
  setValue('MaxAltDrop', data.MaxAltDrop);
  setValue('MinAltDrop', data.MinAltDrop);
  setValue('AltDropThreshold', data.AltDropThreshold);

  // Nieuw: afgeleiden / apogee
  setValue('VerticalSpeed', data.VerticalSpeed);
  setValue('ApogeeDetected', data.ApogeeDetected);
  setValue('ApogeeAltitude', data.ApogeeAltitude);
  setValue('TriggeredBy', data.TriggeredBy);

  // Locatie + axis mapping
  setValue('currentLatitude', data.Latitude);
  setValue('currentLongitude', data.Longitude);
  setValue('axisConfigDisplay', data['Axis Config'] ?? data.axisConfig ?? data.axisConfigDisplay);

  // Sensor data
  setValue('BMP280Temp', data.BMP280Temp);
  setValue('BMP280Pressure', data.BMP280Pressure);
  setValue('MPU6050Temp', data.MPU6050Temp);
  setValue('AccX', data.AccX);
  setValue('AccY', data.AccY);
  setValue('AccZ', data.AccZ);
  setValue('Gyroscope', data.Gyroscope);
  setValue('LocalPressure', data.LocalPressure);
  setValue('DefaultSeaLevelPressure', data.DefaultSeaLevelPressure);
  setValue('PressureSource', data.PressureSource);

  // System / Status
  setValue('ParachuteStatus', data.ParachuteStatus);
  setValue('RocketStatus', data.RocketStatus); // ← nieuw veld
  setValue('SensorsCalibrated', data.SensorsCalibrated);
  setValue('TriggerLogic', data.TriggerLogic);
  setValue('AccXThreshold', data.AccXThreshold);
  setValue('AccYThreshold', data.AccYThreshold);
  setValue('AccZThreshold', data.AccZThreshold);
  setValue('SensorModeFlight', data.SensorModeFlight);
  setValue('TotalSpace', data.TotalSpace);
  setValue('UsedSpace', data.UsedSpace);
  setValue('SPIFFSTotalSpace', data.SPIFFSTotalSpace);
  setValue('SPIFFSUsedSpace', data.SPIFFSUsedSpace);
  setValue('LaunchAccThreshold', data.LaunchAccThreshold);
  setValue('MinApogeeAlt', data.MinApogeeAlt);
  setValue('NegVsConfirm', data.NegVsConfirm);
  setValue('NegCountConfirm', data.NegCountConfirm);
  setValue('GroundSettleMs', data.GroundSettleMs);

  // Warnings zichtbaar/verbergen (als ze bestaan)
  const warn1 = document.getElementById('spiffsWarningBox');
  if (warn1) warn1.style.display = data.SpiffsWarning ? '' : 'none';
  const warn2 = document.getElementById('calibWarningBox');
  if (warn2) warn2.style.display = data.SensorsCalibrated ? 'none' : '';

  // Checkbox UI sync (optioneel)
  const apogeeChk = document.getElementById('chkApogee');
  if (apogeeChk && typeof data.ApogeeTriggerEnabled !== 'undefined') {
    apogeeChk.checked = !!data.ApogeeTriggerEnabled;
  }

  // --- Status threshold inputs: alleen bijwerken als het veld idle is
  setInputIfIdle('minApogeeAlt',       data.MinApogeeAlt);
  setInputIfIdle('negVsConfirm',       data.NegVsConfirm);
  setInputIfIdle('negCountConfirm',    data.NegCountConfirm);
  setInputIfIdle('launchAccThreshold', data.LaunchAccThreshold);
  setInputIfIdle('groundSettleMs',     data.GroundSettleMs);
}

// ------------------------------
// WebSocket live updates
// ------------------------------
let ws;

function connectWS() {
  ws = new WebSocket("ws://" + location.hostname + ":81");

  ws.onopen = function () {
    console.log("WebSocket connected");
    // Vraag meteen een snapshot + zet stream aan
    try { ws.send(JSON.stringify({ request: "telemetry" })); } catch (e) {}
    try { ws.send(JSON.stringify({ sensorModeFlight: true })); } catch (e) {}
  };

  ws.onclose = function () {
    console.log("WebSocket disconnected, retrying...");
    setTimeout(connectWS, 1500);
  };

  ws.onmessage = function (event) {
    try {
      const data = JSON.parse(event.data);
      // Altijd updaten – berichten kunnen deelsets bevatten
      updateDashboard(data);
    } catch (e) {
      // non-JSON → negeren
    }
  };
}

connectWS();

// ------------------------------
// Buttons (acties naar ESP32)
// ------------------------------
function button_arm() {
  ws.send(JSON.stringify({ parachute: "Armed" }));
}
function button_release() {
  ws.send(JSON.stringify({ parachute: "Released" }));
}
function button_unarm() {
  ws.send(JSON.stringify({ parachute: "Unarmed" }));
}
function button_calibrate() {
  ws.send(JSON.stringify({ calibrateSensors: true }));
}

function button_update_location() {
  const latitude = parseFloat(document.getElementById('latitude').value);
  const longitude = parseFloat(document.getElementById('longitude').value);
  ws.send(JSON.stringify({ latitude, longitude }));
}

function button_update_triggers() {
  ws.send(JSON.stringify({
    newThreshold: parseFloat(document.getElementById('newAltThreshold').value),
    enAltDrop: document.getElementById('chkAltDrop').checked,
    newAccX: parseFloat(document.getElementById('newAccX').value),
    enAccX: document.getElementById('chkAccX').checked,
    newAccY: parseFloat(document.getElementById('newAccY').value),
    enAccY: document.getElementById('chkAccY').checked,
    newAccZ: parseFloat(document.getElementById('newAccZ').value),
    enAccZ: document.getElementById('chkAccZ').checked,
    // Nieuw:
    enApogee: document.getElementById('chkApogee') ? document.getElementById('chkApogee').checked : true,
    triggerAbs: document.getElementById('triggerAbs').checked,
    newTriggerLogic: document.getElementById('triggerLogic').value
  }));
}

function button_update_status_thresholds() {
  const minApogeeAlt       = parseFloat(document.getElementById('minApogeeAlt').value);
  const negVsConfirm       = parseFloat(document.getElementById('negVsConfirm').value);
  const negCountConfirm    = parseInt(document.getElementById('negCountConfirm').value, 10);
  const launchAccThreshold = parseFloat(document.getElementById('launchAccThreshold').value);
  const groundSettleMs     = parseInt(document.getElementById('groundSettleMs').value, 10);

  // Kort locken zodat binnenkomende WS frames de UI niet meteen terugzetten
  ['minApogeeAlt','negVsConfirm','negCountConfirm','launchAccThreshold','groundSettleMs']
    .forEach(id => lockField(id, 1500));

  ws.send(JSON.stringify({
    minApogeeAlt, negVsConfirm, negCountConfirm,
    launchAccThreshold, groundSettleMs
  }));
}

function button_update_axis() {
  const config = parseInt(document.getElementById('axisConfig').value, 10);
  ws.send(JSON.stringify({ axisConfig: config }));
}
