# Flight Computer Documentation

This document provides a detailed explanation of the flight computer firmware used in the ESP Controlled Rocket project. It covers the system architecture, sensor integration, core functionalities, and software modules used to ensure reliable telemetry and safe parachute deployment.

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
- [Future Improvements](#future-improvements)
- [Troubleshooting](#troubleshooting)
- [References](#references)

---

## Introduction

The flight computer firmware runs on an ESP32 and is designed to handle sensor data acquisition, parachute deployment, telemetry logging, and remote firmware updates. It integrates multiple sensors and communication protocols to provide a robust system for rocket flight monitoring.

## System Architecture

The system is composed of:

- **Microcontroller:** ESP32 with built-in SD_MMC support.
- **Sensors:**  
  - **BMP280:** Measures temperature, atmospheric pressure, and computes altitude.
  - **MPU6050:** Provides accelerometer, gyroscope, and sensor temperature data.
- **Actuator:** Servo motor for parachute deployment.
- **Communication Modules:**  
  - **WiFi:** For network connectivity and hosting a web server.
  - **WebSockets:** For real-time data streaming.
  - **HTTP OTA:** For remote firmware updates.
- **Storage:** SD card module for logging telemetry data.

A block diagram of the system is shown below (replace with your diagram file):

![System Architecture](media/flight_computer_architecture.png)

## Sensor Integration

### BMP280 Sensor

- **Purpose:** Measures temperature, pressure, and calculates altitude.
- **Library:** `Adafruit_BMP280.h`
- **Configuration:**
  - Operates in normal mode with defined sampling settings.
  - Altitude calculation is refined using local sea-level pressure fetched via the OpenWeatherMap API.

### MPU6050 Sensor

- **Purpose:** Provides accelerometer and gyroscope data along with sensor temperature.
- **Library:** `Adafruit_MPU6050.h`
- **Configuration:**
  - Initialized on a dedicated I2C bus.
  - Configured with proper ranges and filter bandwidth for accurate measurements.

## Core Functionalities

### Data Acquisition

- Reads data from BMP280 and MPU6050 sensors.
- Computes key parameters:
  - **Absolute Altitude:** Calculated using sensor data and corrected with API-provided sea-level pressure.
  - **Relative Altitude:** Determined relative to a baseline set when the parachute is armed.
  - **Altitude Drop:** Monitored to detect rapid decreases in altitude and trigger parachute deployment.

### Parachute Control Logic

- **Arming the Parachute:**  
  - A command via WebSocket (with the value `"Armed"`) triggers the arming process.
  - The current altitude is captured as a baseline for future relative altitude calculations.
- **Parachute Release:**  
  - When a significant altitude drop is detected (exceeding a predefined threshold), the servo motor is actuated to release the parachute.
  - The event is logged to both the serial monitor and the SD card.

### Web Server and Real-Time Telemetry

- **Dashboard:**  
  - A lightweight web server hosts a real-time dashboard displaying flight parameters such as altitude, temperature, inertial data, and parachute status.
- **WebSockets:**  
  - JSON-formatted telemetry data is broadcast to connected clients for live updates.

### Logging and Data Storage

- **SD Card Logging:**  
  - All sensor readings and significant events (e.g., parachute arming/release) are timestamped and recorded on an SD card.
- **Serial Monitoring:**  
  - Data is also output via the serial port for real-time debugging and monitoring.

### OTA Updates and Network Connectivity

- **WiFi Operation:**  
  - The firmware attempts to connect to a pre-configured WiFi network; if unsuccessful, it automatically sets up an Access Point.
- **OTA Update:**  
  - An HTTP OTA update endpoint is available, simplifying remote firmware updates.
  - mDNS is used for network discovery, allowing the device to be accessed via a friendly hostname.

## Software Modules

The firmware is divided into several key modules:

- **Sensor Management:**  
  - Initializes and reads from the BMP280 and MPU6050 sensors.
- **Data Processing:**  
  - Calculates altitude values and monitors flight conditions.
- **Communication:**  
  - Manages the web server, WebSocket communication, and OTA update mechanisms.
- **Logging:**  
  - Handles data recording to the SD card.
- **Utility Functions:**  
  - Provides time synchronization (via NTP) and timestamp generation for log entries.

## Code Structure Overview

The primary structure of the code includes:

- **Library Inclusions & Global Variables:**  
  - Importing necessary libraries and defining configuration variables (e.g., sensor thresholds, WiFi credentials).
- **Setup Function:**  
  - Initializes sensors, WiFi, OTA update service, SD card, and web server.
- **Main Loop:**  
  - Continuously reads sensor data, processes flight parameters, checks for parachute deployment conditions, and updates the telemetry dashboard.
- **Helper Functions:**  
  - Functions such as `getTimeStampString()`, `parachuteArmed()`, `parachuteRelease()`, and `webSocketEvent()` manage specific tasks like time formatting and event handling.

## Future Improvements

- **Enhanced Telemetry:** Incorporate additional sensors (e.g., GPS) for improved navigation.
- **Robust Error Handling:** Develop more advanced error recovery strategies for sensor and network failures.
- **Data Analytics:** Create post-flight analysis tools to visualize sensor data trends and flight performance.
- **Modular Firmware Updates:** Refine the OTA process to reduce downtime during updates.

## Troubleshooting

- **Sensor Initialization Failures:**  
  - Verify that all sensor wiring is secure.
  - Use an I2C scanner tool to confirm correct addresses.
- **WiFi/OTA Connectivity Issues:**  
  - Ensure the correct network credentials are set.
  - Check that the ESP32 is within adequate range of the WiFi network.
- **Logging Errors:**  
  - Confirm that the SD card is properly formatted and connected.
  - Verify that the SD_MMC interface is correctly initialized.

## References

- [Adafruit BMP280 Library Documentation](https://github.com/adafruit/Adafruit_BMP280_Library)
- [Adafruit MPU6050 Library Documentation](https://github.com/adafruit/Adafruit_MPU6050)
- [Arduino OTA Update Library](https://github.com/esp8266/Arduino)
- [OpenWeatherMap API](https://openweathermap.org/api)

*For further assistance or to report issues, please open an issue in the GitHub repository.*
