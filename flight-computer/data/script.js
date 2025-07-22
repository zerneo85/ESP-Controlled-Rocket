/*
  Modified to display a rocket-shaped 3D object representing the MPU6050 sensor orientation.
  The object now uses a cylinder for the body and a cone for the nose.
  Lighting has been added for a smoother, more realistic appearance.
  This version supports dynamic switching of the axis mapping via an "axisConfig" parameter.
  Mapping variants from 0 to 5 are supported as follows:

    Variant 0 (Default):
      - Mapping: Swap x>-X and y>Z and z>Y
   
    Variant 1:
      - Mapping: Swap x>-X and y>Y and z>Z
  
    Variant 2:
      - Mapping: Swap x>Y and y>X and z>Z

    Variant 3:
      - Mapping: Swap x>Y and y>Z and z>X

    Variant 4:
      - Mapping: Swap x>Z and y>Y and z>X

    Variant 5:
      - Mapping: Swap x>Z and y>X and z>Y


  If an unrecognized configuration is received, the default (Variant 0) is applied.
*/

// Global variable for axis mapping (default to variant 0)
let currentAxisConfig = 0;

let scene, camera, renderer, rocket;

function parentWidth(elem) {
  return elem.parentElement.clientWidth;
}

function parentHeight(elem) {
  return elem.parentElement.clientHeight;
}

function init3D() {
  // Create a new scene with a white background.
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xffffff);
  
  // Create a perspective camera.
  camera = new THREE.PerspectiveCamera(
    75,
    parentWidth(document.getElementById("3Dcube")) / parentHeight(document.getElementById("3Dcube")),
    0.1,
    1000
  );
  
  // Initialize the WebGL renderer.
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(parentWidth(document.getElementById("3Dcube")), parentHeight(document.getElementById("3Dcube")));
  document.getElementById("3Dcube").appendChild(renderer.domElement);

  // Add ambient light for overall illumination.
  const ambientLight = new THREE.AmbientLight(0x404040, 2);
  scene.add(ambientLight);
  
  // Add a directional light for shading.
  const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
  directionalLight.position.set(5, 10, 7.5);
  scene.add(directionalLight);

  // Rocket Geometry
  // Define dimensions:
  const bodyHeight = 4;    // Height of the rocket body (cylinder)
  const bodyRadius = 1;    // Radius of the cylinder
  const coneHeight = 2;    // Height of the nose (cone)
  const coneRadius = 1;    // Base radius of the cone
  
  // Create cylinder geometry for the rocket body.
  const cylinderGeometry = new THREE.CylinderGeometry(
    bodyRadius, bodyRadius, bodyHeight, 32, 1, true
  );
  
  // Create cone geometry for the rocket nose.
  const coneGeometry = new THREE.ConeGeometry(
    coneRadius, coneHeight, 32
  );
  
  // Use MeshPhongMaterial to obtain smooth shading.
  const bodyMaterial = new THREE.MeshPhongMaterial({ color: 0x023e8a });
  const noseMaterial = new THREE.MeshPhongMaterial({ color: 0x03045e });
  
  // Create the mesh objects.
  const bodyMesh = new THREE.Mesh(cylinderGeometry, bodyMaterial);
  const coneMesh = new THREE.Mesh(coneGeometry, noseMaterial);

  // Position the cone so that its base sits exactly at the top of the cylinder.
  // The cylinder spans from -bodyHeight/2 to +bodyHeight/2, so its top is at +bodyHeight/2.
  coneMesh.position.y = (bodyHeight / 2) + (coneHeight / 2);

  // Group the body and the cone together to form one composite rocket.
  rocket = new THREE.Group();
  rocket.add(bodyMesh);
  rocket.add(coneMesh);
  scene.add(rocket);

  // Position the camera so that the rocket is clearly visible.
  camera.position.z = 8;
  renderer.render(scene, camera);
}

// Update renderer and camera on window resize.
function onWindowResize() {
  camera.aspect = parentWidth(document.getElementById("3Dcube")) / parentHeight(document.getElementById("3Dcube"));
  camera.updateProjectionMatrix();
  renderer.setSize(parentWidth(document.getElementById("3Dcube")), parentHeight(document.getElementById("3Dcube")));
}
window.addEventListener("resize", onWindowResize, false);

// Initialize the 3D visualization.
init3D();

/**
 * Maps the incoming gyroscope data to rotation values based on the axis mapping configuration.
 * @param {Object} data - The gyroscope data object, containing gyroX, gyroY, and gyroZ.
 * @param {number} axisConfig - The current configuration (0-5).
 * @returns {Object} - An object with properties {rotX, rotY, rotZ} representing rotation.
 */
function mapGyroToRotation(data, axisConfig) {
  let rot = { rotX: 0, rotY: 0, rotZ: 0 };
  switch(axisConfig) {
    case 0:
    // Variant 0 (Default):
    //   - Mapping: Swap x>-X and y>Z and z>Y
      rot.rotX = -data.gyroX;
      rot.rotY = data.gyroZ;
      rot.rotZ = data.gyroY;
      break;
   
      case 1:
    // Variant 1:
    //   - Mapping: Swap x>X and y>Y and z>Z
      rot.rotX = -data.gyroX;
      rot.rotY = data.gyroY;
      rot.rotZ = data.gyroZ;
      break;
   
     case 2:
    // Variant 2:
    //   - Mapping: Swap x>Y and y>X and z>Z
      rot.rotX = data.gyroY;
      rot.rotY = data.gyroX;
      rot.rotZ = data.gyroZ;
      break;
    
      case 3:
    // Variant 3:
    //   - Mapping: Swap x>Y and y>Z and z>X
      rot.rotX = data.gyroY;
      rot.rotY = data.gyroZ;
      rot.rotZ = data.gyroX;
      break;

    case 4:
      // Variant 4:
    //   - Mapping: Swap x>Z and y>Y and z>X
      rot.rotX = -data.gyroZ;
      rot.rotY = -data.gyroY;
      rot.rotZ = -data.gyroX;
      break;
    
      case 5:
      // Variant 5:
    //   - Mapping: Swap x>Z and y>X and z>Y
      rot.rotX = data.gyroZ;
      rot.rotY = data.gyroX;
      rot.rotZ = data.gyroZ;
      break;
    default:
      // Fallback to default mapping.
      rot.rotX = data.gyroX;
      rot.rotY = data.gyroY;
      rot.rotZ = data.gyroZ;
      break;
  }
  return rot;
}

// Create a WebSocket connection (note the port 81).
var socket = new WebSocket("ws://" + window.location.hostname + ":81");

socket.onopen = function (event) {
  console.log("WebSocket Connected");
};

socket.onerror = function (error) {
  console.log("WebSocket Error: ", error);
};

// Listen for messages from the WebSocket.
socket.onmessage = function (event) {
  let msg = JSON.parse(event.data);
  
  // If the firmware sends an axisConfig field, update the client-side mapping configuration.
  if (msg.axisConfig !== undefined) {
    currentAxisConfig = msg.axisConfig;
  }
  
  if (msg.event === "gyro_readings") {
    document.getElementById("gyroX").innerHTML = msg.data.gyroX;
    document.getElementById("gyroY").innerHTML = msg.data.gyroY;
    document.getElementById("gyroZ").innerHTML = msg.data.gyroZ;
    
    // Map the received gyroscope data to rocket rotation based on the currentAxisConfig.
    let rotation = mapGyroToRotation(msg.data, currentAxisConfig);
    rocket.rotation.x = rotation.rotX;
    rocket.rotation.y = rotation.rotY;
    rocket.rotation.z = rotation.rotZ;
    
    renderer.render(scene, camera);
  } else if (msg.event === "accelerometer_readings") {
    document.getElementById("accX").innerHTML = msg.data.accX;
    document.getElementById("accY").innerHTML = msg.data.accY;
    document.getElementById("accZ").innerHTML = msg.data.accZ;
  } else if (msg.event === "temperature_reading") {
    document.getElementById("temp").innerHTML = msg.data;
  }
};

// Reset position function triggered by buttons.
function resetPosition(element) {
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/" + element.id, true);
  console.log("Reset: " + element.id);
  xhr.send();
}


// go back to the main site
document.getElementById('back').addEventListener('click', () => {
  sensorModeFlight = true;      // switch to sensor-mode
  window.location.href = '/';   // navigate home
});