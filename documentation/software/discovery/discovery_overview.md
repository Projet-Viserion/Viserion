# Overview of the Discovery software architecture

## Board connectors

The Discovery board controls the ultrasonic sensors and the IMU. The user manual of the board can be found [here](https://www.st.com/content/ccc/resource/technical/document/user_manual/8a/56/97/63/8d/56/41/73/DM00063382.pdf/files/DM00063382.pdf/jcr:content/translations/en.DM00063382.pdf). The IMU uses the ST MEMS E-compass and Gyroscope embedded on the board. The LSM303DLHC is an ultra-compact low-power system-in-package featuring a 3D digital linear acceleration sensor and a 3D digital magnetic sensor. It includes a sensing element and an IC interface able to provide the measured acceleration to the external world through an I2C serial interface. The L3GD20 is an ultra-compact, low-power, three-axis-angular-rate sensor (gyroscope). The STM32F303VCT6 MCU controls this motion sensor through the SPI interface.

The ports used to connect the board are described in the tables below.

### IMU Sensors
| Port | Configuration | Remap    | Description                 |
|------|---------------|----------|-----------------------------|
| PA5  | AF Output PP  | SPI (SPC)| L3GD20 Gyroscope            |
| PA6  | AF Output PP  | SPI (SDO)| L3GD20 Gyroscope            |
| PA7  | AF Output PP  | SPI (SDI)| L3GD20 Gyroscope            |
| PB6  | AF Output PP  | I2C (SCL)| LSM303DLHC Accelerometer    |
| PB6  | AF Output PP  | I2C (SDA)| LSM303DLHC Accelerometer    |

### Ultrasonic Sensors
| Port | Configuration | Remap    | Description                 |
|------|---------------|----------|-----------------------------|
|      |               |          |                             |

### CAN bus
| Port | Configuration | Remap    | Description                 |
|------|---------------|----------|-----------------------------|
|      |               |          | Rx CAN controler            |
|      |               |          | Tx CAN controler            |

## How to manage sensors and actuators

### AHRS

The code to compute the orientation is inspired of the project [https://github.com/dccharacter/AHRS](https://github.com/dccharacter/AHRS).

The computation uses the fusion code either of:

 - Madgwick algorythm;
 - Mahony algorythm.

The data ouput can be Quaternion or Euler angles. By default the values send through the CAN bus are Euler angles in radian with a fixed-point representation s:1:22.


