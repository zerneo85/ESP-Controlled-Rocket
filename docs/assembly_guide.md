# Assembly Guide for ESP Controlled Rocket

This guide provides detailed instructions to help you assemble your ESP Controlled Rocket from printed parts and electronic components. Follow each section carefully to ensure a successful build.

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

Before beginning the assembly, ensure you have the following:

- **3D Printed Parts:**
  - Rocket Body
  - Nose Cone
  - Fins
  - Mounting Brackets and Fixtures
- **Electronics:**
  - ESP32 (with SD_MMC support)
  - BMP280 Sensor
  - MPU6050 Sensor
  - Servo Motor (for parachute deployment)
  - SD Card Module
  - Wiring and Connectors
- **Tools:**
  - 3D Printer (with proper filament)
  - Soldering iron and solder
  - Screwdrivers and small tools
  - Multimeter (for testing connections)
- **Software:**
  - Arduino IDE or PlatformIO for firmware uploads
  - Access to the repository's code and documentation

## 3D Printing the Components

1. **STL Files:**  
   The 3D model files are located in the `3D_Designs` folder. Key files include:
   - `rocket_body.stl`
   - `nose_cone.stl`
   - `fins.stl`
   - `mounting_brackets.stl`

2. **Print Settings:**  
   - **Material:** PLA or ABS (PLA is recommended for ease of printing)
   - **Layer Height:** 0.2 mm
   - **Infill:** 20-30% for a balance between strength and weight
   - **Supports:** Use supports as needed based on the design

3. **Post-Processing:**  
   - Remove supports carefully.
   - Sand or trim any rough edges if necessary.
   - Test-fit parts together before final assembly.

## Hardware Components Overview

Ensure that all electronic components are available and in good condition:
- **ESP32:** The central flight computer.
- **BMP280 & MPU6050 Sensors:** For altitude and inertial measurements.
- **Servo Motor:** Used to actuate the parachute release mechanism.
- **SD Card Module:** For logging flight data.
- **Cables and Connectors:** Use appropriate gauge wires for secure connections.

## Wiring and Electronics Integration

1. **Sensor Connections:**
   - **BMP280:** Connect to the I2C bus defined on pins (SDA_1, SCL_1).
   - **MPU6050:** Connect to a separate I2C bus (SDA_2, SCL_2) for isolation.
2. **Servo Motor:**
   - Connect the signal wire to the designated servo pin on the ESP32.
   - Ensure proper power supply as per servo specifications.
3. **SD Card Module:**
   - Follow the pin definitions (SD_MMC_CMD, SD_MMC_CLK, SD_MMC_D0) for proper connection.
4. **Power Management:**
   - Ensure that all components are powered by a stable supply.
   - Check voltage levels to avoid damage to sensors and the ESP32.

*Refer to the wiring diagram provided in the `docs` folder for detailed pin connections and layout.*

## Mechanical Assembly

1. **Assemble the Rocket Body:**
   - Begin by joining the printed parts of the rocket body.
   - Ensure proper alignment for the electronics compartment.
2. **Mounting the Electronics:**
   - Secure the ESP32 and peripheral modules using the printed mounting brackets.
   - Route wires neatly to avoid interference and potential disconnections.
3. **Installing the Parachute Mechanism:**
   - Attach the servo motor in the designated compartment.
   - Verify that the servo movement is unobstructed.
4. **Final Integration:**
   - Fit the nose cone and fins into place.
   - Confirm that all parts are securely fastened.

## Final Assembly and Testing

1. **Pre-Flight Check:**
   - Double-check all wiring and mounting.
   - Ensure that the sensors are correctly aligned and unobstructed.
2. **Power On and Diagnostics:**
   - Upload the flight computer firmware and power on the system.
   - Use the serial monitor to verify sensor readings and connectivity.
3. **Ground Testing:**
   - Test the parachute mechanism by simulating an altitude drop.
   - Check the web dashboard for real-time telemetry.
4. **Final Assembly:**
   - Once diagnostics are complete, assemble all parts into the final rocket configuration.
   - Secure the rocket components for safe transport and launch.

## Troubleshooting

- **Sensor Errors:**  
  - Check wiring connections if sensors do not initialize.
  - Verify the I2C addresses using a scanner tool.
- **WiFi Connectivity:**  
  - Ensure that network credentials are correct.
  - Verify that the ESP32 is within range of the WiFi network.
- **Servo Malfunction:**  
  - Test the servo separately to ensure proper operation.
  - Check power supply and signal integrity.
- **SD Card Logging Issues:**  
  - Confirm that the SD card is properly formatted.
  - Verify pin connections as per the SD_MMC interface definitions.

## Additional Resources

- [Flight Computer Documentation](docs/flight_computer.md)
- [Wiring Diagram](docs/wiring_diagram.pdf)

*Happy building and best of luck with your launch!*
