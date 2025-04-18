# Assembly Guide for ESP Controlled Rocket

This guide provides detailed instructions to help you assemble your ESP Controlled Rocket from printed parts and electronic components. Follow each section carefully to ensure a successful build.

*(Note: The design files for all 3D printed components are still in development and will be released soon.)*

## Table of Contents

- [Introduction](#introduction)  
- [Pre-Assembly Checklist](#pre-assembly-checklist)  
- [3D Printing the Components](#3d-printing-the-components)  
- [Hardware Components Overview](#hardware-components-overview)  
- [Wiring and Electronics Integration](#wiring-and-electronics-integration)  
- [Mechanical Assembly](#mechanical-assembly)  
- [Final Assembly and Testing](#final-assembly-and-testing)  
- [Troubleshooting](#troubleshooting)  
- [Additional Resources](#additional-resources)

---

## Introduction

The ESP Controlled Rocket combines a custom flight computer with carefully designed 3D printed parts to create a robust rocket system. This guide outlines each step of the assembly process—from printing the parts to integrating the electronics and performing final tests.

## Pre-Assembly Checklist

Before beginning assembly, ensure you have:

- **3D Printed Parts:** *(STL files coming soon)*  
  - Rocket Body  
  - Nose Cone  
  - Fins  
  - Mounting Brackets  
- **Electronics:**  
  - ESP32 (SD_MMC support)  
  - BMP280 & MPU6050 Sensors  
  - Servo Motor  
  - SD Card Module  
  - Wiring & Connectors  
- **Tools:**  
  - 3D Printer (PLA recommended)  
  - Soldering Iron  
  - Screwdrivers  
  - Multimeter  
- **Software:**  
  - Arduino IDE / PlatformIO  
  - Repository code & docs

*(Camera module removed as of v3.3.0.)*

## 3D Printing the Components

1. **STL Files:** in `3D_Designs` folder (coming soon)  
2. **Print Settings:**  
   - PLA, 0.2 mm layer, 20–30% infill  
   - Supports as needed  
3. **Post-Processing:**  
   - Remove supports, sand edges, test-fit

## Hardware Components Overview

- **ESP32**  
- **BMP280 & MPU6050**  
- **Servo Motor**  
- **SD Card Module**  
- **Cables & Connectors**

## Wiring and Electronics Integration

1. **Sensors (I2C):**  
   - BMP280: SDA → GPIO 42, SCL → GPIO 37  
   - MPU6050: SDA/SCL on separate I2C bus if available  
2. **Servo:** Signal → GPIO 14, Power → 5 V, GND → common  
3. **SD Card (SD_MMC):**  
   - CMD → GPIO 38, CLK → GPIO 39, D0 → GPIO 40  
4. **Power:** Stable 3.3 V/5 V supply, common ground  

*See [Wiring Diagram](wiring_diagram.md) for visuals.*

## Mechanical Assembly

1. **Rocket Body:** Join printed parts, align compartments  
2. **Mount Electronics:** Secure ESP32 & sensors in brackets  
3. **Parachute Mechanism:** Mount servo and release latch  
4. **Final Integration:** Attach nose cone & fins, ensure rigidity

## Final Assembly and Testing

1. **Pre-Flight Check:** Verify wiring, mounts, sensors  
2. **Power On & Diagnostics:** Upload firmware, monitor serial  
3. **Ground Testing:** Arm/release parachute, check telemetry  
4. **Final Assembly:** Close compartments, insert SD card

## Troubleshooting

- **Sensor Issues:** Check I2C wiring & axis config  
- **WiFi/OTA:** Verify credentials & AP mode  
- **SD Card:** FAT32 format & proper wiring  
- **Servo:** Test separately & ensure power

*(Camera/timelapse removed.)*

## Additional Resources

- [Flight Computer Docs](flight_computer.md)  
- [Wiring Diagram](wiring_diagram.md)

*Happy building!*
