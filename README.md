# ESP Controlled Rocket

The ESP Controlled Rocket repository is a comprehensive project that brings together an advanced flight computer system along with detailed 3D printing designs to build and launch a rocket. The project includes embedded firmware for real-time telemetry, sensor integration, parachute deployment, and a complete set of 3D models for printing the rocket parts.

---

## Table of Contents

- [Introduction](#introduction)
- [Flight Computer](#flight-computer)
  - [Features](#features)
  - [Code Overview](#code-overview)
  - [Flight Computer Screenshot](#flight-computer-screenshot)
- [3D Designs](#3d-designs)
  - [Overview of 3D Models](#overview-of-3d-models)
- [Assembly & Build](#assembly--build)
- [Usage](#usage)
- [Rocket Performance Calculations](#rocket-performance-calculations)
- [Contributing](#contributing)

---

## Introduction

The **ESP Controlled Rocket** project combines state-of-the-art embedded firmware with ready-to-print 3D designs to create a complete rocket system. The flight computer manages sensor data, logs flight events, and controls the parachute deployment mechanism while providing real-time telemetry via a web interface. New additions include status indication via RGB LED effects and a manual release function accessible from the web interface.

---

## Flight Computer

### Features

- **Sensor Integration:**  
  - **BMP280 Sensor:** Reads temperature, pressure, and computes altitude.
  - **MPU6050 Sensor:** Provides accelerometer, gyroscope, and sensor temperature data.
- **Real-Time Telemetry:**  
  - A web server with WebSocket support streams live flight data including altitude, temperature, inertial measurements, and parachute status.
- **Parachute Deployment:**  
  - Monitors a corrected altitude drop calculation to trigger the servo-controlled parachute release automatically.
  - Includes a manual release function via the web interface for testing.
- **Enhanced SD Card Logging:**  
  - Logs flight data and key events with accurate timestamps.
- **RGB LED Status Indicators:**  
  - An LED ring provides visual feedback for network connectivity, parachute arming, and release states.
- **WiFi Connectivity & OTA Updates:**  
  - Connects to a predefined network or automatically sets up an access point.
  - Supports remote firmware updates through an HTTP OTA endpoint.
- **Time Synchronization:**  
  - Uses NTP to provide accurate timestamps for logging.

### Code Overview

The flight computer firmware is organized into modules that handle:
- **Sensor Management:** Initialization and continuous reading of the BMP280 and MPU6050 sensors.
- **Data Processing:** Calculation of absolute altitude, relative altitude (with baseline captured at arming), and a corrected altitude drop.
- **Parachute Control:** Both automatic deployment (triggered by an altitude drop exceeding a threshold) and manual release via a web interface.
- **LED Status Indicators:** Control of an RGB LED ring that reflects current system and network status.
- **Communication:** A lightweight web server and WebSocket system that broadcast real-time flight data.
- **Logging:** Timestamped recording of flight events and sensor readings on an SD card.
- **OTA Updates:** Remote firmware update functionality via HTTP OTA and mDNS service.

### Flight Computer Screenshot

![Flight Computer Screenshot](media/flight_computer_dashboard_4.png)

---

## 3D Designs

The repository includes 3D printing designs for the rocket's structure and components. The models are optimized for easy printing and assembly.

### Overview of 3D Models

- **Rocket Body:**  
  Main structure with compartments for the electronics, payload, and fuel system.
- **Nose Cone & Fins:**  
  Aerodynamic designs to ensure optimal flight performance.
- **Assembly Parts:**  
  Custom brackets and mounting fixtures to integrate the flight computer and sensors securely.

![3D Design Overview](media/3d_design_overview.png)

Detailed 3D model files (.STL) and assembly instructions can be found in the [3d_designs](3d_designs) folder.

---

## Assembly & Build

For detailed build instructions, refer to the [Assembly Guide](docs/assembly_guide.md). The guide covers component assembly, wiring, and final system integration including wiring for the new RGB LED indicators and updated parachute control.

---

## Usage

### Setting Up the Flight Computer

1. **WiFi Configuration:**  
   Update the network credentials in the firmware.  
   *Note: The WiFi configuration code has been omitted from this document for security reasons.*  
   If the connection fails, the device will switch to access point mode and indicate its status via LED blinks.

2. **OTA Updates:**  
   Access the OTA update page by navigating to [http://esp32-webupdate.local/update](http://esp32-webupdate.local/update) once the device is connected to WiFi.

3. **Manual Parachute Release:**  
   The web dashboard now includes two buttons:
   - **Arm Parachute:** Arms the parachute and sets the baseline altitude.
   - **Release Parachute:** Immediately triggers the parachute release for testing.

4. **Running the Code:**  
   Compile and upload the code to your ESP32 using the Arduino IDE or PlatformIO.

### Debugging and Logging

The firmware includes various configuration options to control debugging output and logging behavior, such as sensor logs, SD card logging, and LED status indications.

---

## Rocket Performance Calculations

The following table shows a simplified performance calculation for different tube lengths (assuming a 60 mm diameter tube) based on these assumptions:

- **Internal Volume:**  
  \( V = \pi \times (0.03\,m)^2 \times L \) (with \(L\) in meters)
- **Optimal Water:**  
  Approximately 42% of the internal volume (in liters) is used.
- **Total Mass:**  
  Dry mass is constant at 0.7 kg; water mass is added (1 L water ≈ 1 kg).
- **Thrust Time:**  
  \( T = \dfrac{V_{water}}{A \times v_e} \) with nozzle area \(A \approx 7.85 \times 10^{-5}\,m^2\) and effective exit velocity \(v_e \approx 50\,m/s\).
- **Δv:**  
  \(\Delta v = v_e \ln\left(\dfrac{m_{initial}}{m_{dry}}\right)\).
- **Burnout Speed:**  
  \( v_{burn} \approx \Delta v - \tfrac{1}{2}gT \) with \( g = 9.81\,m/s^2 \).
- **Coasting Height:**  
  \( h_{coast} = \dfrac{v_{burn}^2}{2g} \).
- **Thrust Height:**  
  \( h_{thrust} \approx \dfrac{v_{burn}}{2} \times T \).
- **Maximum Altitude:**  
  \( h_{max} = h_{coast} + h_{thrust} \).
- **Flight Duration:**  
  Ascent time is approximately \( T + \dfrac{v_{burn}}{g} \) and descent (free-fall) is similar, yielding a total flight duration of about \( T + 2 \times \dfrac{v_{burn}}{g} \).
- **Impact Speed:**  
  Approximately equal to the burnout speed (ignoring air resistance).

```markdown
| Tube Length (cm) | Internal Volume (L) | Optimal Water (L) | Total Mass (kg) | Thrust Time (s) | Δv (m/s) | Burnout Speed (m/s) | Maximum Altitude (m) | Flight Duration (s) | Impact Speed (m/s) |
|------------------|---------------------|-------------------|-----------------|-----------------|----------|---------------------|----------------------|---------------------|--------------------|
| 75               | 2.12                | 0.90              | 1.60            | 0.23            | 41.3     | 40.2                | 87.0                 | 8.43                | 40.2               |
| 100              | 2.83                | 1.19              | 1.89            | 0.30            | 49.7     | 48.2                | 125.6                | 10.12               | 48.2               |
| 125              | 3.53                | 1.48              | 2.18            | 0.38            | 56.9     | 55.0                | 164.7                | 11.60               | 55.0               |
| 150              | 4.24                | 1.78              | 2.48            | 0.45            | 63.4     | 61.2                | 204.8                | 12.92               | 61.2               |
| 175              | 4.95                | 2.08              | 2.78            | 0.53            | 69.1     | 66.5                | 242.8                | 14.09               | 66.5               |
| 200              | 5.65                | 2.37              | 3.07            | 0.60            | 74.0     | 71.1                | 278.9                | 15.08               | 71.1               |

## Contributing
Contributions are welcome! Please follow these guidelines:
- Fork the repository.
- Create a new branch for your feature or bugfix.
- Commit your changes with clear messages.
- Submit a pull request with a detailed description of your changes.
