# Flight Computer Documentation

This document provides a detailed explanation of the flight computer firmware used in the ESP Controlled Rocket project. It covers the system architecture, sensor integration, core functionalities, and software modules used to ensure reliable telemetry and safe parachute deployment.

> **Note:** Camera and timelapse functionality have been completely removed. New features include RGB LED status indicators, a corrected altitude drop calculation, and an extra web interface command to manually release the parachute.

## Table of Contents

- [Introduction](#introduction)
- [System Architecture](#system-architecture)
- [Sensor Integration](#sensor-integration)
- [Core Functionalities](#core-functionalities)
  - [Data Acquisition](#data-acquisition)
  - [Parachute Control Logic](#parachute-control-logic)
  - [Web Server and Real-Time Telemetry](#web-server-and-real-time-telemetry)
  - [Logging and Data Storage](#logging-and-data-storage)
  - [OTA Updates and Network Connectivity](#ota-updates-and-network-connectivity)
- [Software Modules](#software-modules)
- [Code Structure Overview](#code-structure-overview)
- [Recent Improvements](#recent-improvements)
- [Troubleshooting](#troubleshooting)
- [References](#references)

---

## Introduction

The flight computer firmware runs on an ESP32 and is designed to handle sensor data acquisition, parachute deployment, telemetry logging, and remote firmware updates. This version focuses on robust sensor integration, precise altitude drop calculations, visual feedback via an RGB LED ring, and a web interface that now includes manual commands for arming and releasing the parachute.

---

## System Architecture

The system comprises:

- **Microcontroller:** ESP32 with SD_MMC support.
- **Sensors:**  
  - **BMP280:** Measures temperature, pressure, and calculates altitude.
  - **MPU6050:** Provides accelerometer and gyroscope data.
- **Actuator:** Servo motor for parachute deployment.
- **Status Indicators:**  
  - **RGB LED Ring:** Provides visual feedback for system status and network connectivity.
- **Communication Modules:**  
  - **WiFi:** Enables network connectivity and hosts a web server.
  - **WebSockets:** Streams live flight telemetry data.
  - **HTTP OTA:** Facilitates remote firmware updates.
- **Storage:** SD card for logging flight events and sensor data.

A block diagram of the system is shown below (replace with your diagram file):

![System Architecture](media/flight_computer_architecture.png)

---

## Sensor Integration

### BMP280 Sensor

- **Purpose:**  
  Measures temperature and pressure, and computes altitude.
- **Configuration:**  
  Operates in normal mode with defined sampling settings.
- **Altitude Correction:**  
  Altitude is calculated using the local sea-level pressure, updated via the OpenWeatherMap API.

### MPU6050 Sensor

- **Purpose:**  
  Provides accelerometer and gyroscope data.
- **Configuration:**  
  Initialized on a dedicated I2C bus and configured with proper ranges for accurate measurements.

---

## Core Functionalities

### Data Acquisition

- Reads data from BMP280 and MPU6050 sensors.
- Computes key parameters:
  - **Absolute Altitude:** Based on BMP280 readings.
  - **Relative Altitude:** Calculated relative to a baseline captured during parachute arming.
  - **Altitude Drop:** Correctly computed from the difference between the maximum relative altitude and the current relative altitude.

### Parachute Control Logic

- **Arming the Parachute:**  
  - A command via the web interface (WebSocket) with `"Armed"` triggers the arming process.
  - The current altitude is captured as a baseline and relative altitude tracking is reset.
- **Parachute Release:**  
  - Automatic release is triggered when a significant altitude drop is detected.
  - A new web interface command (`"Released"`) is available to manually trigger the release for testing.
  - The servo motor actuates the parachute mechanism.
  
### Web Server and Real-Time Telemetry

- **Dashboard:**  
  A lightweight web server hosts a real-time dashboard displaying flight parameters (altitude, temperature, inertial data, parachute status, etc.).
- **WebSockets:**  
  JSON-formatted telemetry data is broadcast for live updates.  
  New buttons allow for manual arming and releasing of the parachute.

### Logging and Data Storage

- **SD Card Logging:**  
  Logs sensor readings and key events (e.g., parachute arming/release) with accurate timestamps.
- **Serial Monitoring:**  
  Provides real-time debugging output over the serial interface.

### OTA Updates and Network Connectivity

- **WiFi Operation:**  
  The firmware attempts to connect to a predefined network; if unsuccessful, it sets up an access point.
- **OTA Update:**  
  A built-in HTTP OTA update endpoint allows for remote firmware updates.
- **RGB LED Status:**  
  The LED ring indicates network status (green for connected, red for AP mode) and displays system events.

---

## Software Modules

The firmware is divided into several modules:

- **Sensor Management:**  
  Initialization and continuous reading of the BMP280 and MPU6050 sensors.
- **Data Processing:**  
  Calculation of altitude values, including the corrected altitude drop.
- **Parachute Control:**  
  Functions to arm and release the parachute automatically based on sensor data or manually via the web interface.
- **LED Status Indicators:**  
  Control of the RGB LED ring for visual feedback.
- **Communication:**  
  A web server and WebSocket system that broadcast live telemetry.
- **Logging:**  
  Recording sensor data and flight events to an SD card.
- **Utility Functions:**  
  Time synchronization via NTP and formatting for log entries.

---

## Code Structure Overview

The code is structured as follows:

- **Library Inclusions & Global Variables:**  
  All necessary libraries are included and global variables (e.g., sensor thresholds, WiFi credentials, LED colors) are defined.
- **Setup Function:**  
  Initializes sensors, WiFi, OTA update service, SD card, web server, WebSocket, servo, and LED strip.
- **Main Loop:**  
  Continuously reads sensor data, processes flight parameters, checks for parachute conditions, and updates the telemetry dashboard.
- **Helper Functions:**  
  Functions such as `getTimeStampString()`, `parachuteArmed()`, `parachuteRelease()`, and `webSocketEvent()` manage tasks like time formatting and event handling.

---

## Recent Improvements

- **RGB LED Status Indicators:**  
  Added an RGB LED ring for visual system feedback (network status, parachute arming, and release states).
- **Corrected Altitude Drop Calculation:**  
  The relative altitude and altitude drop are now calculated more accurately by resetting the tracking values upon arming.
- **Manual Parachute Release via Web Interface:**  
  A new "Release Parachute" button has been added to the web dashboard so that the parachute can be released manually for testing purposes.
- **Removed Camera Functionality:**  
  All camera and timelapse imaging functionality has been removed to streamline the flight computer firmware.

---

## Troubleshooting

- **Sensor Issues:**  
  - Verify wiring for the BMP280 and MPU6050.
  - Use an I2C scanner to check sensor addresses.
- **WiFi and OTA Connectivity:**  
  - Ensure correct network credentials.
  - Check that the ESP32 is within range.
- **Logging Issues:**  
  - Confirm the SD card is formatted correctly.
  - Verify that the SD_MMC interface initializes without errors.
- **LED or Parachute Behavior:**  
  - Ensure sufficient power supply for both the servo and LED ring.
  - Check that manual commands on the web interface trigger the correct functions.

---

## References

- [Adafruit BMP280 Library Documentation](https://github.com/adafruit/Adafruit_BMP280_Library)
- [Adafruit MPU6050 Library Documentation](https://github.com/adafruit/Adafruit_MPU6050)
- [Arduino OTA Update Library](https://github.com/esp8266/Arduino)
- [OpenWeatherMap API](https://openweathermap.org/api)