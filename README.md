# ESP Controlled Rocket

The ESP Controlled Rocket repository is a comprehensive project that brings together an advanced flight computer system along with detailed 3D printing designs to build and launch a rocket. The project includes embedded firmware for real-time telemetry, sensor integration, parachute deployment, OTA updates, and a complete set of 3D models for printing the rocket parts.

---

## Table of Contents

- [Introduction](#introduction)
- [Flight Computer](#flight-computer)
  - [Features](#features)
  - [Code Overview](#code-overview)
  - ![Flight Computer Screenshot](media/flight_computer_dashboard.png)
- [3D Designs](#3d-designs)
  - [Overview of 3D Models](#overview-of-3d-models)
  - ![3D Design Overview](media/3d_design_overview.png)
- [Assembly & Build](#assembly--build)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

---

## Introduction

The **ESP Controlled Rocket** project combines state-of-the-art embedded firmware with practical, ready-to-print 3D designs to create a complete rocket system. The flight computer manages sensor data, performs real-time logging and telemetry, and controls a parachute deployment mechanism, while the 3D designs guide you in printing and assembling the physical components of the rocket.

---

## Flight Computer

This section covers the firmware running on the ESP32, responsible for flight data acquisition, parachute deployment, OTA updates, and web-based telemetry.

### Features

- **Sensor Integration:**  
  - **BMP280 Sensor:** Reads temperature, pressure, and computes altitude.
  - **MPU6050 Sensor:** Provides accelerometer, gyroscope data, and sensor temperature.
  
- **Real-Time Telemetry:**  
  - Web server with WebSocket support serves a live dashboard.
  - Displays parameters like absolute/relative altitude, temperature, pressure, inertial data, and parachute status.
  
- **Parachute Deployment:**  
  - Monitors altitude drop and triggers a servo to release the parachute when a threshold is exceeded.
  
- **SD Card Logging:**  
  - All telemetry and key events are logged to an SD card for post-flight analysis.
  
- **WiFi Connectivity & OTA Updates:**  
  - Automatically connects to a predefined network or sets up an access point if needed.
  - Supports remote firmware updates via an HTTP OTA endpoint.
  
- **Time Synchronization:**  
  - Uses NTP for accurate timestamping of logs and events.
  
- **API Integration:**  
  - Retrieves local sea-level pressure from OpenWeatherMap to refine altitude calculations.

### Code Overview

The flight computer code is divided into several modules:

- **Library Inclusions:**  
  The code uses libraries for sensor management, SD card operations, WiFi, OTA updates, web server functionalities, and JSON handling.

- **Sensor Initialization & Data Acquisition:**  
  Two I2C buses are used to connect the BMP280 and MPU6050 sensors. Data from these sensors is processed to compute absolute and relative altitude, as well as to monitor environmental and inertial parameters.

- **Parachute Control Logic:**  
  The firmware captures a baseline altitude when arming the parachute. A significant altitude drop triggers the parachute release via a servo mechanism.

- **Web Server & Telemetry Dashboard:**  
  A lightweight web server provides a dashboard to monitor live flight data. WebSockets broadcast JSON-formatted telemetry to the client browser.

- **Logging & OTA Updates:**  
  All flight data and events are logged on an SD card, and the firmware supports OTA updates using mDNS for simplified network access.

For more details on the flight computer code, please refer to the [Flight Computer Code Documentation](docs/flight_computer.md).

![Flight Computer Screenshot](media/flight_computer_dashboard.png)

---

## 3D Designs

In addition to the firmware, this repository contains 3D printing designs for the rocket's structure and components. The models are optimized for easy printing and assembly.

### Overview of 3D Models

- **Rocket Body:**  
  The main body design includes compartments for the electronics, payload, and fuel system.

- **Nose Cone & Fins:**  
  Detailed designs for the aerodynamic nose cone and stabilizing fins ensure optimal flight performance.

- **Assembly Parts:**  
  Custom brackets and mounting fixtures to securely integrate the flight computer and sensors into the rocket.

For a visual overview of the designs, please see the screenshot below:

![3D Design Overview](media/3d_design_overview.png)

Detailed 3D model files (.STL) and assembly instructions can be found in the [3d_designs](3d_designs) folder.

---

## Assembly & Build

This chapter outlines the steps to assemble your ESP Controlled Rocket:

1. **3D Print the Components:**  
   - Use the provided STL files to print the rocket body, nose cone, fins, and mounting parts.
   - Ensure proper print settings for high quality and durability.

2. **Hardware Integration:**  
   - Install the ESP32 flight computer into the designated compartment.
   - Connect the sensors (BMP280 and MPU6050), SD card module, servo, and other peripherals following the wiring diagrams.

3. **Final Assembly:**  
   - Assemble the printed components.
   - Secure all electronics and ensure proper alignment for aerodynamic stability.

For detailed build instructions, please refer to the [Assembly Guide](docs/assembly_guide.md).

---

## Usage

### Setting Up the Flight Computer

1. **WiFi Configuration:**  
   Update the network credentials in the firmware:
   ```cpp
   const char* ssid = "TDGC-Rocket";
   const char* wifiPassword = "Rocket2022!";
## WiFi Connection Fallback

If the connection fails, the device will switch to access point mode.

2. **OTA Updates:**

Access the OTA update page by navigating to [http://esp32-webupdate.local/update](http://esp32-webupdate.local/update) once the device is connected to WiFi.

3. **Running the Code**

Compile and upload the code to your ESP32 using the Arduino IDE or PlatformIO.

## Monitoring Flight Data

- Use any web browser to access the live telemetry dashboard served by the ESP32.
- The dashboard displays real-time flight data including:
  - Altitude
  - Temperature
  - Inertial measurements
  - Parachute status
- For further details on usage, check out the [User Manual](docs/user_manual.md).

## Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository.
2. Create a new branch for your feature or bugfix.
3. Commit your changes with clear messages.
4. Submit a pull request with a detailed description of your changes.


