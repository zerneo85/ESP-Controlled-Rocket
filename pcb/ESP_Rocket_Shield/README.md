# ESP Rocket Shield — KiCad Starter Project (Rev A)

This is a **starter** KiCad 8 project for your ESP32‑S3 rocket shield. It includes a schematic with all blocks you requested (power with L7805, I²C sensors (BMP280/MPU6050 class), LED pads, piezo buzzer, and dual female headers for the ESP32‑S3 dev board). 
The PCB file contains the board outline and class definitions; import/update from the schematic to populate footprints as you finalize.

> **Note**: This scaffold is intentionally minimal so you can adjust footprints to your exact parts. Nets, labels, and blocks are provided; please run **Tools → Annotate**, then **Assign Footprints**, then **Update PCB from Schematic**.

## Quick steps
1. Open `ESP_Rocket_Shield.kicad_pro` in KiCad 8+.
2. Open the schematic. Check/replace sensor symbols if you choose BMP388/DPS310/ICM‑42688 instead of BMP280/MPU6050.
3. Use **Assign Footprints** to bind footprints that match your inventory (headers, LGA packages, buzzer, etc.).
4. Run **Update PCB from Schematic** to bring parts into PCB.
5. Route, pour GND, add stitching vias, and finalize thermals on L7805.
6. Generate Gerbers.

## Nets / pins (matches your firmware & diagram)
- LED data: **GPIO17** → `LED_DATA` → series **330 Ω** → LED pad.
- I²C: **GPIO42= SDA**, **GPIO37= SCL**, with **4.7 kΩ** pull‑ups to **3V3**.
- Buzzer: **GPIO21** → 1 kΩ → N‑MOSFET/NPN → Active buzzer to **+5V**.
- Servo (optional): **GPIO14** → 3‑pin header (+5V / PWM / GND).
- Power: **VIN** (2S Li‑ion) → Fuse → Schottky (reverse) → **L7805** → **+5V** rail.
- ESP32 gets power via **5V** dev board pin; sensors on **3V3** from the dev board.

## Library picks
- L7805: `Regulator_Linear:L7805` symbol; footprint TO‑220 or TO‑252.
- Baro: `Sensor_Pressure:BMP280` (swap to BMP388/DPS310 later if you prefer).
- IMU: `Sensor_Motion:MPU-6050` (swap to ICM‑42688‑P if you prefer).
- Headers: `Connector_Generic:Conn_01x19_Female` left/right for ESP32‑S3; `Conn_01x03` for LED and Servo.
- Buzzer driver: `Device:Q_NMOS_GSD` (2N7002) or `Device:Q_NPN_BEC` (MMBT2222A).

## Board outline
The PCB is initialized as 90×40 mm with 4× M2.5 mounting holes. Adjust to fit your airframe.

## Notes
- Place the IMU and barometer near the board center, away from the buzzer and power edges.
- Keep a vent hole under the barometer and cover with open‑cell foam after assembly.
- If LED current > ~0.6–0.8 A and VIN ≈ 2S (7–8.4 V), consider swapping L7805 for a buck converter footprint to reduce heat.
