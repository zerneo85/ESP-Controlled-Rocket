+-------------------------------------------------------+
|          Freenove ESP32-WROVER CAM Board              |
|                (Microcontroller)                    |
+-------------------------------------------------------+
          |             |             |      
          |             |             |      
       [3.3V]        [5V/USB]       [GND]
          |             |             |      
          |             |             |      
          |             |             |
          +-------------+-------------+
                        |
       +----------------------------------+
       |  Integrated Camera Module         |
       |  (Camera sensor uses built-in pins |
       |   via SCCB/I2C interface as defined |
       |   in "camera_pins.h")               |
       |                                    |
       |  (Optional external I2C config:    |
       |     SDA: Pin 4, SCL: Pin 5)          |
       +----------------------------------+
                        |
                        |
                        |
+-----------------------+-------------------------------+
|                   I2C Bus for Sensors                   |
|                (BMP280 & MPU6050)                         |
+-----------------------+-------------------------------+
                        |
             SDA (Sensors) -> Pin **42**
             SCL (Sensors) -> Pin **37**
                        |
         +--------------+--------------+
         |                             |
+------------------+         +------------------+
|   BMP280 Sensor  |         |   MPU6050 Sensor |
| (Temp, Pressure, |         | (Accel, Gyro,    |
|  Altitude)       |         |  Temperature)    |
+------------------+         +------------------+
         |                             |
         +--------------+--------------+
                        |
                        |
+-------------------------------------------------------+
|                SD_MMC (SD Card Module)                |
+-------------------------------------------------------+
                        |
         CMD  -> Pin **38**
         CLK  -> Pin **39**
         D0   -> Pin **40**
                        |
               [SD Card Socket/Module]
                        |
                        |
+-------------------------------------------------------+
|                Servo Motor (Parachute)                |
+-------------------------------------------------------+
                        |
       Signal (Control) -> Pin **14**
                        |
       VCC and GND should be connected to an 
       appropriate external power supply (e.g., 5V)
       with common ground to the ESP32 board.
