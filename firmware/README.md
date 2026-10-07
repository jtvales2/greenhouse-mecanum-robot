# STM32 Chassis Firmware

Embedded firmware for the four-wheel mecanum chassis used in the greenhouse mobile robot.

## Hardware

- MCU: STM32F407ZGT6
- Four independent wheel motors
- Wheel encoders
- WIT IMU
- UART communication with the ROS 2 onboard computer

## Main Modules

- `chassis_ctrl` — mecanum kinematics and wheel-speed closed-loop control
- `chassis_hw` — chassis hardware abstraction
- `robot_app` — application state and robot status output
- `ros_serial_proto` — STM32 / ROS 2 serial protocol
- `imu_yaw` — yaw integration and gyro-bias handling
- `wit_imu_hal` — WIT IMU hardware integration
- `MotorHAL` — motor hardware interface
- `EncoderHAL` — encoder interface

## Development Environment

- STM32CubeMX
- Keil MDK-ARM
- STM32 HAL / CMSIS

The CubeMX project file is included as:

`stm32/dfh1.0.ioc`

## Third-Party Dependencies

STM32 HAL and CMSIS are provided by STMicroelectronics under their respective licenses.

The project also interfaces with the WIT Motion IMU SDK. Vendor SDK source files are not redistributed in this repository. Obtain the required SDK from WIT Motion separately.

### WIT Motion SDK Files Required for Build

The original Keil project references the WIT Motion SDK.

Before building the complete firmware, obtain the corresponding WIT Motion SDK
from the vendor and provide the following files:

```text
firmware/stm32/Core/Inc/wit_c_sdk.h
firmware/stm32/Core/Inc/REG.h
firmware/stm32/Core/Src/wit_c_sdk.c

## Validation

This firmware was used on the real four-wheel mecanum robot and validated together with the ROS 2 / Nav2 system.
