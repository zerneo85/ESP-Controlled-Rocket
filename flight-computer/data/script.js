// Utility: Update dashboard with current data
function updateDashboard(data) {
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

  // Match field names to HTML IDs!
  setValue('currentLatitude', data.Latitude);
  setValue('currentLongitude', data.Longitude);
  setValue('axisConfigDisplay', data['Axis Config'] || data.axisConfig || data.axisConfigDisplay);

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

  setValue('ParachuteStatus', data.ParachuteStatus);
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

  document.getElementById('spiffsWarningBox').style.display =
    data.SpiffsWarning ? '' : 'none';
  document.getElementById('calibWarningBox').style.display =
    !data.SensorsCalibrated ? '' : 'none';
}

// Utility: set innerText or fallback to '-'
function setValue(id, value) {
  document.getElementById(id).innerText = value !== undefined ? value : '-';
}

// --- WebSocket live updates ---
let ws;
function connectWS() {
  ws = new WebSocket("ws://" + location.hostname + ":81");
  ws.onopen = function() {
    console.log("WebSocket connected");
  };
  ws.onclose = function() {
    console.log("WebSocket disconnected, retrying...");
    setTimeout(connectWS, 1500);
  };
  ws.onmessage = function(event) {
    try {
      const data = JSON.parse(event.data);
      // The WebSocket sends live flight values
      if (data.AbsoluteAltitude !== undefined) {
        updateDashboard(data);
      }
    } catch (e) {
      // Non-JSON, ignore
    }
  };
}
connectWS();


// BUTTON HANDLERS
function button_arm() {
  ws.send(JSON.stringify({ parachute: "Armed" }));
}
function button_release() {
  ws.send(JSON.stringify({ parachute: "Released" }));
}
function button_calibrate() {
  ws.send(JSON.stringify({ calibrateSensors: true }));
}
function button_unarm() {
  ws.send(JSON.stringify({ parachute: "Unarmed" }));
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
    triggerAbs: document.getElementById('triggerAbs').checked,
    newTriggerLogic: document.getElementById('triggerLogic').value
  }));
}
function button_update_axis() {
  const config = parseInt(document.getElementById('axisConfig').value);
  ws.send(JSON.stringify({ axisConfig: config }));
}