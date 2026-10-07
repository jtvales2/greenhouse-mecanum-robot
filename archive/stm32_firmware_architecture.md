# STM32 Firmware Architecture

The STM32F407 firmware implements the low-level motion-control layer of the greenhouse mecanum robot.

It is responsible for motor control, encoder feedback, mecanum kinematics, odometry, IMU yaw processing and communication with the ROS 2 onboard computer.

---

## 1. Firmware Overview

```text
ROS 2 Onboard Computer
        |
        | UART velocity command
        v
+--------------------------+
|      ros_serial_proto    |
+------------+-------------+
             |
             v
+--------------------------+
|        robot_app         |
+------------+-------------+
             |
             v
+--------------------------+
|       chassis_ctrl       |
|                         |
| Mecanum Kinematics      |
| Wheel Speed Control     |
| Odometry                |
+------------+-------------+
             |
             v
+--------------------------+
|        chassis_hw        |
+------------+-------------+
             |
      +------+------+
      |             |
      v             v
   Motors        Encoders
```

The IMU provides an independent yaw measurement path:

```text
WIT IMU
   |
   v
wit_imu_hal
   |
   v
imu_yaw
   |
   v
Odometry / Robot State
```

---

## 2. Main Modules

### `chassis_ctrl`

Core chassis-control module.

Responsibilities:

- mecanum-wheel inverse kinematics
- body velocity command handling
- individual wheel-speed setpoints
- wheel-speed closed-loop control
- chassis velocity estimation
- odometry integration
- emergency wheel-output stop

Coordinate convention:

```text
+vx = forward
+vy = left
+wz  = counter-clockwise
```

---

### `chassis_hw`

Hardware abstraction layer for the chassis.

Responsibilities include interaction with:

- motor drivers
- wheel encoders
- timers
- low-level actuator interfaces

The control layer therefore does not need to directly manipulate peripheral registers.

---

### `MotorHAL`

Motor-control hardware interface.

It provides the low-level interface used by the chassis-control layer to command individual wheel motors.

---

### `EncoderHAL`

Wheel-encoder interface.

Encoder feedback is used by the chassis controller for wheel-speed feedback and odometry estimation.

---

### `imu_yaw`

Processes yaw-rate information from the IMU and maintains the robot yaw estimate.

The real vehicle exposed long-duration gyro bias during development, so the yaw path was explicitly separated from the chassis-control logic to make calibration and drift handling easier to debug.

---

### `wit_imu_hal`

Integration layer between the firmware and the WIT IMU.

The vendor WIT Motion SDK itself is treated as an external dependency and is not redistributed by this repository.

---

### `ros_serial_proto`

Serial protocol between STM32 and the Raspberry Pi ROS 2 computer.

The onboard computer sends body-velocity commands to the STM32, while the STM32 publishes chassis state back to ROS 2.

The original system used commands including:

```text
V vx vy wz
STOP
RESET_ODOM
PING
```

The returned robot state was consumed by the ROS 2 serial bridge and odometry node.

---

### `robot_app`

Top-level robot application logic.

Responsibilities include:

- system startup
- command dispatch
- chassis-state packaging
- application-level state management
- interaction between communication and motion-control modules

---

## 3. Control Flow

The main control path is:

```text
ROS 2 /cmd_vel_robot
        |
        v
base_serial_bridge
        |
        | UART
        v
ros_serial_proto
        |
        v
robot_app
        |
        v
chassis_ctrl
        |
        +----> mecanum inverse kinematics
        |
        +----> wheel-speed controller
        |
        v
chassis_hw / MotorHAL
        |
        v
Four Wheel Motors
```

Feedback path:

```text
Wheel Encoders
      |
      v
EncoderHAL
      |
      v
chassis_ctrl
      |
      +----> wheel velocity
      +----> body velocity
      +----> odometry
      |
      v
robot_app
      |
      v
ros_serial_proto
      |
      | UART
      v
ROS 2 onboard computer
```

---

## 4. Odometry and Yaw

The robot combines:

- four wheel encoders
- mecanum forward kinematics
- WIT IMU yaw information

to produce the chassis motion state used by the ROS 2 odometry layer.

The ROS 2 side publishes:

```text
odom -> base_link
```

while AMCL provides:

```text
map -> odom
```

This separates local wheel/IMU motion estimation from global map localization.

---

## 5. Firmware Project

MCU:

```text
STM32F407ZGT6
```

Development tools:

- STM32CubeMX
- Keil MDK-ARM
- STM32 HAL
- CMSIS

CubeMX project:

```text
firmware/stm32/dfh1.0.ioc
```

Keil project:

```text
firmware/stm32/MDK-ARM/dfh1.0.uvprojx
```

---

## 6. Repository Notes

Generated binaries and IDE-specific user files are intentionally excluded from the repository.

The firmware source tree contains both project-authored code and STMicroelectronics HAL/CMSIS components.

The WIT Motion vendor SDK is an external dependency and is not redistributed in this repository.

See:

[Third-Party Notices](../THIRD_PARTY_NOTICES.md)
