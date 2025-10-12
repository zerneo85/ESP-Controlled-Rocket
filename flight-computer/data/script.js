// ===== DASHBOARD SCRIPT (served at /script.js) =====

// --- helpers ---
const $ = (id) => document.getElementById(id);

// Track which inputs the user is currently editing
const editing = new Set();

// Throttle telemetry UI updates
let lastPaint = 0;
const PAINT_INTERVAL_MS = 200; // ~5 fps is plenty for UI

// Wire focus/blur handlers once so telemetry won't overwrite while the user types
function registerEditableInputs() {
  const editableIds = [
    "latitude", "longitude",
    "newAccX", "newAccY", "newAccZ",
    "newAltThreshold",
    "axisConfig",
    "triggerLogic",
    "minApogeeAlt", "negVsConfirm", "negCountConfirm",
    "launchAccThreshold", "groundSettleMs",

    // checkboxes + triggerAbs
    "chkApogee", "chkAccX", "chkAccY", "chkAccZ", "chkAltDrop",
    "triggerAbs"
  ];
  editableIds.forEach(id => {
    const el = $(id);
    if (!el) return;
    el.addEventListener("mousedown", () => editing.add(id)); // click starts edit
    el.addEventListener("focusin",  () => editing.add(id));
    el.addEventListener("input",    () => editing.add(id));
    el.addEventListener("focusout", () => editing.delete(id));
  });
}

// Safe setters: only update if not being edited right now
function setTextIfIdle(id, value) {
  const el = $(id);
  if (!el) return;
  // These IDs map to plain <td> cells, not inputs — safe to always set.
  el.textContent = value;
}

function setNumberInputIfIdle(id, value, digits = 2) {
  const el = $(id);
  if (!el) return;
  if (editing.has(id)) return;
  const num = (typeof value === "number") ? value : parseFloat(value);
  if (!Number.isNaN(num)) el.value = num.toFixed(digits);
}

function setStringInputIfIdle(id, value) {
  const el = $(id);
  if (!el) return;
  if (editing.has(id)) return;
  el.value = value;
}

function setCheckboxIfIdle(id, checked) {
  const el = $(id);
  if (!el) return;
  if (editing.has(id)) return;
  el.checked = !!checked;
}

function setSelectIfIdle(id, value) {
  const el = $(id);
  if (!el) return;
  if (editing.has(id)) return;
  el.value = String(value);
}

// --- buttons / actions already used by your HTML ---
function button_arm()      { wsSend({ parachute: "Armed" }); }
function button_release()  { wsSend({ parachute: "Released" }); }
function button_unarm()    { wsSend({ parachute: "Unarmed" }); }
function button_calibrate(){ wsSend({ calibrateSensors: true }); }

function button_update_location() {
  const latitude  = parseFloat($("latitude").value);
  const longitude = parseFloat($("longitude").value);
  if (Number.isFinite(latitude) && Number.isFinite(longitude)) {
    wsSend({ latitude, longitude });
  }
}

function button_update_triggers() {
  const idsToLock = [
    "chkApogee","chkAccX","chkAccY","chkAccZ","chkAltDrop",
    "triggerAbs","triggerLogic","newAccX","newAccY","newAccZ","newAltThreshold"
  ];
  // mark as editing and hold for a moment
  idsToLock.forEach(id => editing.add(id));
  setTimeout(() => idsToLock.forEach(id => editing.delete(id)), 4000);

  wsSend({
    enApogee:   $("chkApogee").checked,
    enAccX:     $("chkAccX").checked,
    enAccY:     $("chkAccY").checked,
    enAccZ:     $("chkAccZ").checked,
    enAltDrop:  $("chkAltDrop").checked,

    triggerAbs: $("triggerAbs").checked,
    newTriggerLogic: $("triggerLogic").value,

    newAccX: parseFloat($("newAccX").value),
    newAccY: parseFloat($("newAccY").value),
    newAccZ: parseFloat($("newAccZ").value),
    newThreshold: parseFloat($("newAltThreshold").value),
  });
}

function button_update_axis() {
  wsSend({ axisConfig: parseInt($("axisConfig").value, 10) });
}

function button_update_status_thresholds() {
  const idsToLock = ["minApogeeAlt","negVsConfirm","negCountConfirm","launchAccThreshold","groundSettleMs"];
  idsToLock.forEach(id => editing.add(id));
  setTimeout(() => idsToLock.forEach(id => editing.delete(id)), 4000);

  wsSend({
    minApogeeAlt:       parseFloat($("minApogeeAlt").value),
    negVsConfirm:       parseFloat($("negVsConfirm").value),
    negCountConfirm:    parseInt($("negCountConfirm").value, 10),
    launchAccThreshold: parseFloat($("launchAccThreshold").value),
    groundSettleMs:     parseInt($("groundSettleMs").value, 10),
  });
}

// --- websocket wiring ---
let ws;
function wsSend(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  }
}

function startWS() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.host}:81/`);

  ws.addEventListener("open", () => {
    // Ask for an initial snapshot
    wsSend({ request: "telemetry" });
  });

  ws.addEventListener("message", (evt) => {
    // Throttle paints
    const now = performance.now();
    if (now - lastPaint < PAINT_INTERVAL_MS) return;
    lastPaint = now;

    let data;
    try { data = JSON.parse(evt.data); } catch { return; }

    // Guard every assignment — use the safe setters above.
    if ("AbsoluteAltitude" in data)    setTextIfIdle("AbsoluteAltitude", data.AbsoluteAltitude.toFixed(2));
    if ("RelativeAltitude" in data)    setTextIfIdle("RelativeAltitude", data.RelativeAltitude.toFixed(2));
    if ("VerticalSpeed" in data)       setTextIfIdle("VerticalSpeed", data.VerticalSpeed.toFixed(3));
    if ("ApogeeDetected" in data)      setTextIfIdle("ApogeeDetected", data.ApogeeDetected ? "Yes" : "No");
    if ("ApogeeAltitude" in data)      setTextIfIdle("ApogeeAltitude", data.ApogeeAltitude.toFixed(2));
    if ("AltitudeDrop" in data)        setTextIfIdle("AltitudeDrop", data.AltitudeDrop.toFixed(2));
    if ("MaxAbsAltitude" in data)      setTextIfIdle("MaxAbsAltitude", data.MaxAbsAltitude.toFixed(2));
    if ("MinAbsAltitude" in data)      setTextIfIdle("MinAbsAltitude", data.MinAbsAltitude.toFixed(2));
    if ("MaxRelAltitude" in data)      setTextIfIdle("MaxRelAltitude", data.MaxRelAltitude.toFixed(2));
    if ("MinRelAltitude" in data)      setTextIfIdle("MinRelAltitude", data.MinRelAltitude.toFixed(2));
    if ("MaxAltDrop" in data)          setTextIfIdle("MaxAltDrop", data.MaxAltDrop.toFixed(2));
    if ("MinAltDrop" in data)          setTextIfIdle("MinAltDrop", data.MinAltDrop.toFixed(2));

    if ("BMP280Temp" in data)          setTextIfIdle("BMP280Temp", data.BMP280Temp.toFixed(2));
    if ("BMP280Pressure" in data)      setTextIfIdle("BMP280Pressure", data.BMP280Pressure.toFixed(2));
    if ("MPU6050Temp" in data)         setTextIfIdle("MPU6050Temp", data.MPU6050Temp.toFixed(2));
    if ("AccX" in data)                setTextIfIdle("AccX", data.AccX.toFixed(2));
    if ("AccY" in data)                setTextIfIdle("AccY", data.AccY.toFixed(2));
    if ("AccZ" in data)                setTextIfIdle("AccZ", data.AccZ.toFixed(2));
    if ("Gyroscope" in data)           setTextIfIdle("Gyroscope", data.Gyroscope);

    if ("LocalPressure" in data)       setTextIfIdle("LocalPressure", data.LocalPressure.toFixed(2));
    if ("DefaultSeaLevelPressure" in data) setTextIfIdle("DefaultSeaLevelPressure", data.DefaultSeaLevelPressure);
    if ("PressureSource" in data)      setTextIfIdle("PressureSource", data.PressureSource);

    if ("ParachuteStatus" in data)     setTextIfIdle("ParachuteStatus", data.ParachuteStatus);
    if ("RocketStatus" in data)        setTextIfIdle("RocketStatus", data.RocketStatus);
    if ("TriggeredBy" in data)         setTextIfIdle("TriggeredBy", data.TriggeredBy);
    if ("SensorsCalibrated" in data)   setTextIfIdle("SensorsCalibrated", data.SensorsCalibrated ? "Yes" : "No");
    if ("TriggerLogic" in data)        setTextIfIdle("TriggerLogic", data.TriggerLogic);

    if ("AccXThreshold" in data)       setTextIfIdle("AccXThreshold", data.AccXThreshold.toFixed(2));
    if ("AccYThreshold" in data)       setTextIfIdle("AccYThreshold", data.AccYThreshold.toFixed(2));
    if ("AccZThreshold" in data)       setTextIfIdle("AccZThreshold", data.AccZThreshold.toFixed(2));
    if ("AltDropThreshold" in data)    setTextIfIdle("AltDropThreshold", data.AltDropThreshold.toFixed(2));

    if ("TotalSpace" in data)          setTextIfIdle("TotalSpace", data.TotalSpace);
    if ("UsedSpace" in data)           setTextIfIdle("UsedSpace", data.UsedSpace);
    if ("SPIFFSTotalSpace" in data)    setTextIfIdle("SPIFFSTotalSpace", data.SPIFFSTotalSpace);
    if ("SPIFFSUsedSpace" in data)     setTextIfIdle("SPIFFSUsedSpace", data.SPIFFSUsedSpace);
    if ("SensorModeFlight" in data)    setTextIfIdle("SensorModeFlight", data.SensorModeFlight ? "ON" : "OFF");
    if ("axisConfig" in data)          setTextIfIdle("axisConfigDisplay", data.axisConfig);
    if ("Axis Config" in data)         setTextIfIdle("axisConfigDisplay", data["Axis Config"]);

    if ("Latitude" in data)            setNumberInputIfIdle("latitude", data.Latitude, 6);
    if ("Longitude" in data)           setNumberInputIfIdle("longitude", data.Longitude, 6);

    // Reflect triggers/enables back into controls if user isn’t editing
    if ("EnAltDrop" in data)           setCheckboxIfIdle("chkAltDrop", data.EnAltDrop);
    if ("EnAccX" in data)              setCheckboxIfIdle("chkAccX", data.EnAccX);
    if ("EnAccY" in data)              setCheckboxIfIdle("chkAccY", data.EnAccY);
    if ("EnAccZ" in data)              setCheckboxIfIdle("chkAccZ", data.EnAccZ);
    if ("ApogeeTriggerEnabled" in data) setCheckboxIfIdle("chkApogee", data.ApogeeTriggerEnabled);

    if ("LaunchAccThreshold" in data)  setNumberInputIfIdle("launchAccThreshold", data.LaunchAccThreshold, 1);
    if ("MinApogeeAlt" in data)        setNumberInputIfIdle("minApogeeAlt", data.MinApogeeAlt, 1);
    if ("NegVsConfirm" in data)        setNumberInputIfIdle("negVsConfirm", Math.abs(data.NegVsConfirm), 2);
    if ("NegCountConfirm" in data)     setNumberInputIfIdle("negCountConfirm", data.NegCountConfirm, 0);
    if ("GroundSettleMs" in data)      setNumberInputIfIdle("groundSettleMs", data.GroundSettleMs, 0);

    if ("TriggerLogic" in data)        setSelectIfIdle("triggerLogic", data.TriggerLogic);
    if ("Axis Config" in data)         setSelectIfIdle("axisConfig", data["Axis Config"]);

    // --- System / Status values (table cells) ---
if ("LaunchAccThreshold" in data)
  setTextIfIdle("LaunchAccThreshold", Number(data.LaunchAccThreshold).toFixed(2));
if ("MinApogeeAlt" in data)
  setTextIfIdle("MinApogeeAlt", Number(data.MinApogeeAlt).toFixed(1));
if ("NegVsConfirm" in data)
  setTextIfIdle("NegVsConfirm", Math.abs(Number(data.NegVsConfirm)).toFixed(2));
if ("NegCountConfirm" in data)
  setTextIfIdle("NegCountConfirm", data.NegCountConfirm);
if ("GroundSettleMs" in data)
  setTextIfIdle("GroundSettleMs", data.GroundSettleMs);

// Mirror coordinates into the table cells as well
if ("Latitude" in data)
  setTextIfIdle("currentLatitude", Number(data.Latitude).toFixed(6));
if ("Longitude" in data)
  setTextIfIdle("currentLongitude", Number(data.Longitude).toFixed(6));

  });

  ws.addEventListener("close", () => {
    // retry after a bit
    setTimeout(startWS, 1000);
  });
}

window.addEventListener("DOMContentLoaded", () => {
  registerEditableInputs();
  startWS();
});

// Expose button handlers used by your HTML
window.button_arm = button_arm;
window.button_release = button_release;
window.button_unarm = button_unarm;
window.button_calibrate = button_calibrate;
window.button_update_location = button_update_location;
window.button_update_triggers = button_update_triggers;
window.button_update_axis = button_update_axis;
window.button_update_status_thresholds = button_update_status_thresholds;
