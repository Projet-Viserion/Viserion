# Overview of the Nucleo software architecture

## Connexion

The Nucleo board controls the vehicle's engines and steering, as well as the battery level. The ports used to connect the board are described in the diagram of the [electronic board](../../hardware/electronic/electronic_overview.md)


### Power switch
| Port | Configuration | Remap | Description                     |
|------|---------------|-------|---------------------------------|
| PA6  | Output PP     | -     |Control the power                |


### Sensors
| Port | Configuration | Remap | Description                     |
|------|---------------|-------|---------------------------------|
| PA0  | Analog Input  | ADC1 IN0 |Battery Level             |
| PA1  | Analog Input  | ADC1 IN1 | Steering wheel angle     |
| PB10 | AF Output PP  | TIM2 CH3 | Position left motor |
| PB8 | AF Output PP  | TIM4 CH3 | Position right motor   |

### Left wheel motor
| Port | Configuration | Remap | Description                  |
|------|---------------|-------|------------------------------|
| PC10 | Output PP 10MHz| -        | Enable left motor         |
| PA8  | AF Output PP  | TIM1 CH1 |  Control left motor (IN1) |
| PA7  | AF Output PP  | TIM1 CH1N|  Control left motor (IN2) |
| PA4  | Analog Input  | ADC1 IN4 | Current left motor        |

### Right wheel motor
| Port | Configuration | Remap | Description                  |
|------|---------------|-------|------------------------------|
| PC11 | Output PP 10MHz| -        | Enable right motor         |
| PA9  | AF Output PP  | TIM1 CH2 |  Control right motor (IN1) |
| PB0  | AF Output PP  | TIM1 CH2N|  Control right motor (IN2) |
| PA5  | Analog Input  | ADC1 IN5 | Current right motor        |

### Steering wheel manual buttons
| Port | Configuration | Remap | Description                  |
|------|---------------|-------|------------------------------|
| PB14 | Input Pullup  | —     | Status of the right button | 
| PB15 | Input Pullup  | —     | Status of the right button | 

### Steering wheel motor
| Port | Configuration | Remap | Description                  |
|------|---------------|-------|------------------------------|
| PC12 | Output PP 10MHz| -       | Enable steering motor         |
| PA10 | AF Output PP  | TIM1 CH3 |  Control steering motor (IN1) |
| PB1  | AF Output PP  | TIM1 CH3N|  Control steering motor (IN2) |
| PC0  | Analog Input  | ADC1 IN10 | Current steering motor      |

### CAN bus
| Port | Configuration | Remap | Description                  |
|------|---------------|-------|------------------------------|
| PA11 | AF Output PP  | CAN RX | CAN Rx bus                  |
| PA12 | AF Output PP  | CAN TX | CN TX bus                   |

The full description of the port configuration is provided in the file [electronic board](../../../nucleo/voiture-elec.txt).

## How to manage sensors and actuators

### Bootstrap power

To maintain the power it is necessary to set PA6. If the pin is reset the power of the car is off.

Functions `power_bootstrap` and  `power_shutdown`can be used to control the state of the PA6.

> [!IMPORTANT]
> When launching the DEBUG or LOAD of the STM32 card, it is necessary to manually press and hold the START button (ST-LINK imposes a reset that cuts off the latch circuit).

### Voltage measurements

All voltage measurements (battery, steering angle, current left rear motor, current right rear motor and current front motor)  are made in continuous mode by the ADC. The measurements are stored in a circular buffer in the DMA.

The battery level can be obtained by calling the function `int get_battery_level(void)` and the position angle of the steering by `int get_steering_angle(void)`. Battery and steering angle are raw data.

The value for the battery is between 0 and 0xFFF. The battery level U (V) can be computed by U = (4095 / Bat\_mes) * (3.3 / 0.2). The nominal operation of the battery has to be between 11 and 14 V. 

The steering angle has to be calibrate.

### Wheel motor speed

The speed of the wheel motors is computed by mesuring the duration between two edges of a coder. This value can be obtained by using the function `int void get_speed_right_motor()` or `int void get_speed_left_motor()`. The value is given in *0.01 rpm.


### Motor Commands

All engines are speed controlled, even the steering wheel engine. 

The command of a motor is encoded by 8-bit data. The bit 7 is used to enable or disable the motor and bits 6-0 are used to code the value of the command. The values are between 0 and 100 with 50 to stop the motor. To avoid problems some software thresholds were implemented (25-75 for the wheels and 40-60 for the steering).

**Warning:** All motors operate in open loop. Thus, the steering  is not controled in angle.

The motor control of the wheels is done by calling the function `wheels_set_speed`. The motor control of the steering is done by calling `steering_set_speed`. These functions are periodically called in the main file.


## How to use the CAN bus

Sending a message on the CAN bus is done by calling the function `void CAN_Send (unint8_t* data, unint32_t id)`. 

The reception of a message is done in the callback function of the CAN interruption `HAL_CAN_RxCpltCallback` in file `can.c`. Motor controls (wheel and steering) are shared via global variables. These variables are updated on receipt of the CAN message. The actuators are updated periodically in the main file.

## Scheduling

### Motor speed
The speed is calculated under interruption at each edge of the coder of each wheel. The clock used to calculate the speed is the timer TIM2 for the left wheel and TIM4 for the right wheel. This timer is reset at each interruption and give directly the speed. if the car does not move then the speed is set to zero after 100ms.

### Voltage measurements
Voltage measurements are continuously updated by the ADC, so there is no scheduling for these values. They can be read asynchronously. The freshness of the data is less than 1ms.

### Motor Commands
The motor consigns are updated each time a CAN message is received. The actuators are updated periodically in the loop of the main file. The base period is 20ms. It can be changed by changing the value of the `PERIOD\_UPDATE\_CMD` constant in the `Inc/main.h` file. The maximum response time to react to a command is approximatively egals to the time transmission of the CAN MCM message, plus the period of the update loop.

### Sending data sensors
The battery level, the steering angle, and the speeds of the left and right motor are periodically send with a period egals to 100ms. It can be changed by changing the value of the `PERIOD\_CAN\_SEND` constant in the `Inc/main.h` file.

