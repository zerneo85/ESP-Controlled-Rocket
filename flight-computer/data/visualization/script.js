/*
  Modified to work with the WebSocketsServer implementation.
*/

let scene, camera, renderer, rocket;

function parentWidth(elem) {
  return elem.parentElement.clientWidth;
}

function parentHeight(elem) {
  return elem.parentElement.clientHeight;
}

function init3D() {
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xffffff);
  camera = new THREE.PerspectiveCamera(
    75,
    parentWidth(document.getElementById("3Dcube")) / parentHeight(document.getElementById("3Dcube")),
    0.1,
    1000
  );
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(parentWidth(document.getElementById("3Dcube")), parentHeight(document.getElementById("3Dcube")));
  document.getElementById("3Dcube").appendChild(renderer.domElement);

  const ambientLight = new THREE.AmbientLight(0x404040, 2);
  scene.add(ambientLight);
  const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
  directionalLight.position.set(5, 10, 7.5);
  scene.add(directionalLight);

  // Rocket Geometry
  const bodyHeight = 4;
  const bodyRadius = 1;
  const coneHeight = 2;
  const coneRadius = 1;
  const cylinderGeometry = new THREE.CylinderGeometry(bodyRadius, bodyRadius, bodyHeight, 32, 1, true);
  const coneGeometry = new THREE.ConeGeometry(coneRadius, coneHeight, 32);
  const bodyMaterial = new THREE.MeshPhongMaterial({ color: 0x023e8a });
  const noseMaterial = new THREE.MeshPhongMaterial({ color: 0x03045e });
  const bodyMesh = new THREE.Mesh(cylinderGeometry, bodyMaterial);
  const coneMesh = new THREE.Mesh(coneGeometry, noseMaterial);

  coneMesh.position.y = (bodyHeight / 2) + (coneHeight / 2);
  rocket = new THREE.Group();
  rocket.add(bodyMesh);
  rocket.add(coneMesh);
  scene.add(rocket);

  camera.position.z = 8;
  renderer.render(scene, camera);
}

function onWindowResize() {
  camera.aspect = parentWidth(document.getElementById("3Dcube")) / parentHeight(document.getElementById("3Dcube"));
  camera.updateProjectionMatrix();
  renderer.setSize(parentWidth(document.getElementById("3Dcube")), parentHeight(document.getElementById("3Dcube")));
}
window.addEventListener("resize", onWindowResize, false);
init3D();

// Create a WebSocket connection (note the port 81).
var socket = new WebSocket("ws://" + window.location.hostname + ":81");

socket.onopen = function (event) {
  console.log("WebSocket Connected");
};

socket.onerror = function (error) {
  console.log("WebSocket Error: ", error);
};

socket.onmessage = function (event) {
  let msg = JSON.parse(event.data);
  if (msg.event === "gyro_readings") {
    document.getElementById("gyroX").innerHTML = msg.data.gyroX;
    document.getElementById("gyroY").innerHTML = msg.data.gyroY;
    document.getElementById("gyroZ").innerHTML = msg.data.gyroZ;
    // Update rocket rotation
    rocket.rotation.x = msg.data.gyroY;
    rocket.rotation.y = msg.data.gyroZ;
    rocket.rotation.z = msg.data.gyroX;
    renderer.render(scene, camera);
  } else if (msg.event === "accelerometer_readings") {
    document.getElementById("accX").innerHTML = msg.data.accX;
    document.getElementById("accY").innerHTML = msg.data.accY;
    document.getElementById("accZ").innerHTML = msg.data.accZ;
  } else if (msg.event === "temperature_reading") {
    document.getElementById("temp").innerHTML = msg.data;
  }
};

function resetPosition(element) {
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/" + element.id, true);
  console.log("Reset: " + element.id);
  xhr.send();
}
