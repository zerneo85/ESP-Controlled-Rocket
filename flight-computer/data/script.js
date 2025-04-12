// 3D Visualization & SSE Sensor Data for Visualization Page

let scene, camera, renderer, rocket;

function parentWidth(elem) { return elem.parentElement.clientWidth; }
function parentHeight(elem) { return elem.parentElement.clientHeight; }

function init3D() {
  // Create a scene with white background
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xffffff);
  
  // Set up camera
  camera = new THREE.PerspectiveCamera(
    75,
    parentWidth(document.getElementById("3Dcube")) / parentHeight(document.getElementById("3Dcube")),
    0.1,
    1000
  );
  
  // Renderer
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(parentWidth(document.getElementById("3Dcube")), parentHeight(document.getElementById("3Dcube")));
  document.getElementById("3Dcube").appendChild(renderer.domElement);
  
  // Lights
  const ambientLight = new THREE.AmbientLight(0x404040, 2);
  scene.add(ambientLight);
  const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
  directionalLight.position.set(5, 10, 7.5);
  scene.add(directionalLight);
  
  // Rocket geometry
  const bodyHeight = 4, bodyRadius = 1, coneHeight = 2, coneRadius = 1;
  const cylinderGeometry = new THREE.CylinderGeometry(bodyRadius, bodyRadius, bodyHeight, 32, 1, true);
  const coneGeometry = new THREE.ConeGeometry(coneRadius, coneHeight, 32);
  const bodyMaterial = new THREE.MeshPhongMaterial({ color: 0x023e8a });
  const noseMaterial = new THREE.MeshPhongMaterial({ color: 0x03045e });
  const bodyMesh = new THREE.Mesh(cylinderGeometry, bodyMaterial);
  const coneMesh = new THREE.Mesh(coneGeometry, noseMaterial);
  // Position the cone on top of the cylinder
  coneMesh.position.y = (bodyHeight / 2) + (coneHeight / 2);
  
  // Create a group for rocket and add meshes
  rocket = new THREE.Group();
  rocket.add(bodyMesh);
  rocket.add(coneMesh);
  scene.add(rocket);
  
  // Set camera position
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

// Establish SSE connection to receive sensor data from the "/events" endpoint
if (!!window.EventSource) {
  var source = new EventSource("/events");

  source.addEventListener('open', function(e) {
    console.log("SSE Connected");
  }, false);

  source.addEventListener('error', function(e) {
    if(e.target.readyState !== EventSource.OPEN) {
      console.log("SSE Disconnected");
    }
  }, false);

  // Listen for sensor_data events and update the display and 3D rocket
  source.addEventListener('sensor_data', function(e) {
    let obj = JSON.parse(e.data);
    // Update sensor display elements
    document.getElementById("gyroX").innerHTML = obj.gyroX || "-";
    document.getElementById("gyroY").innerHTML = obj.gyroY || "-";
    document.getElementById("gyroZ").innerHTML = obj.gyroZ || "-";
    document.getElementById("accX").innerHTML = obj.accX || "-";
    document.getElementById("accY").innerHTML = obj.accY || "-";
    document.getElementById("accZ").innerHTML = obj.accZ || "-";
    document.getElementById("temp").innerHTML = obj.MPU6050Temp || "-";
    
    // Apply sensor readings to update rocket rotation
    rocket.rotation.x = obj.gyroY || 0;
    rocket.rotation.y = obj.gyroZ || 0;
    rocket.rotation.z = obj.gyroX || 0;
    renderer.render(scene, camera);
  }, false);
}

// Reset position function triggered by buttons
function resetPosition(element) {
  let xhr = new XMLHttpRequest();
  xhr.open("GET", "/" + element.id, true);
  xhr.send();
}
