# Flight Computer Documentation

This document provides a detailed explanation of the flight computer firmware used in the ESP Controlled Rocket project. It covers architecture, sensor integration, core functionalities, and software modules.

> **Note:** Version **3.4.0** adds web-based file management, optional 3D visualization, axis configuration, and live location updates. Camera/timelapse functionality is removed.

## Table of Contents

- [Introduction](#introduction)  
- [System Architecture](#system-architecture)  
- [Sensor Integration](#sensor-integration)  
- [Core Functionalities](#core-functionalities)  
- [Software Modules](#software-modules)  
- [Code Structure Overview](#code-structure-overview)  
- [Recent Improvements](#recent-improvements)  
- [Troubleshooting](#troubleshooting)  
- [References](#references)

---

## Introduction

Runs on ESP32; handles sensor data, parachute control, telemetry, logging, and OTA updates.

---

## System Architecture

- **ESP32** with SD_MMC  
- **BMP280** & **MPU6050** via I2C  
- **Servo** for parachute  
- **RGB LED Ring** for status  
- **WiFi/WebSocket** for dashboard & commands  
- **SPIFFS & SD** for storage

![Architecture](media/flight_computer_architecture.png)

---

## Sensor Integration

### BMP280  
- Altitude via barometric formula; uses live location override for reference pressure.

### MPU6050  
- IMU data; supports runtime axis remapping via web UI.

---

## Core Functionalities

### Data Acquisition  
- Absolute & relative altitude, corrected altitude drop.

### Parachute Control  
- Arm via web; automatic/manual release; logs events.

### Web Server & Telemetry  
- Dashboard, WebSockets, axis config, location override, optional 3D viz data.

### Logging & Storage  
- Web file manager (upload/download/delete SPIFFS & SD), SD logging, serial debug.

### OTA & Connectivity  
- Station/AP mode, HTTP OTA (`/update`), mDNS, LED status patterns.

---

## Software Modules

- Sensor Management  
- Data Processing  
- Parachute Control  
- LED Indicators  
- Communication (HTTP & WS)  
- Logging  
- Utilities

---

## Code Structure Overview

- **Globals & Includes**  
- **setup()**: init WiFi, SD, sensors, server, WS, LED ring  
- **loop()**: handleClient, webSocket.loop, read sensors, compute, log, broadcast  
- **webSocketEvent()**: process commands (arm/release, axis config, location)  
- **Helper functions**: armParachute(), releaseParachute(), updateOrientation(), updateLocation(), setLEDStatus()

---

## Recent Improvements

- **Web File Manager** (SPIFFS & SD)  
- **Optional 3D Visualization** data stream  
- **Runtime Axis Configuration**  
- **Live Location Updates**  
- **Removed Camera/Timelapse**  
- **Altitude Drop Refinement**  
- **RGB LED Enhancements**

---

## Troubleshooting

- Check I2C wiring & axis settings  
- Verify WiFi/AP connectivity & IP/mDNS  
- Ensure SD card format & wiring  
- Test servo power & movement  

---

## References

- Adafruit BMP280 & MPU6050 libraries  
- ESP32 WebServer & WebSocket libraries  
- Arduino OTA docs  
- OpenWeatherMap API  
