# STM32 Firmware Architecture

This document describes the low-level STM32F407 firmware architecture used by
the greenhouse mecanum robot.

The firmware implements the real-time chassis-control layer and provides:

- mecanum kinematics;
- wheel-speed closed-loop control;
- motor and encoder interfaces;
- IMU yaw processing;
- robot-state management;
- UART communication with the ROS 2 onboard computer.

---

## 1. Firmware Position in the System

The STM32 sits between the ROS 2 onboard computer and the physical mecanum
chassis.

```text
ROS 2 Onboard Computer
        |
        | UART ASCII protocol
        v
+--------------------------+
|       STM32F407          |
|                          |
|  ros_serial_proto        |
|          |               |
|      robot_app           |
|          |               |
|     chassis_ctrl         |
|          |               |
|     chassis_hw           |
|       /      \           |
| MotorHAL   EncoderHAL    |
+-----|----------|---------+
      |          |
      v          v
   Motors      Encoders
```

The IMU path is:

```text
WIT IMU
   |
   v
imu_uart_hal
   |
   v
wit_imu_hal
   |
   v
imu_yaw
   |
   v
robot_app / chassis state
```

---

## 2. Development Platform

Target MCU:

```text
STM32F407ZGT6
```

Development environment:

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

Startup file:

```text
firmware/stm32/MDK-ARM/startup_stm32f407xx.s
```

---

## 3. Main Source Modules

The main project-specific firmware modules are located under:

```text
firmware/stm32/Core/
```

Important modules include:

```text
robot_app
chassis_ctrl
chassis_hw
MotorHAL
EncoderHAL
ros_serial_proto
imu_yaw
wit_imu_hal
imu_uart_hal
motion_command
mechanism_ctrl
yaw_ctrl
```

---

## 4. `robot_app`

`robot_app` acts as the application-level coordination layer.

Its responsibilities include:

- robot startup state;
- high-level command dispatch;
- chassis command forwarding;
- robot-state packaging;
- odometry reset handling;
- communication between protocol and control modules.

Conceptually:

```text
serial command
      |
      v
ros_serial_proto
      |
      v
robot_app
      |
      v
chassis_ctrl
```

The application layer prevents the communication module from directly
controlling hardware.

---

## 5. `chassis_ctrl`

`chassis_ctrl` is the main chassis-control module.

Its responsibilities include:

- body velocity command handling;
- mecanum inverse kinematics;
- wheel-speed target generation;
- wheel-speed closed-loop control;
- chassis velocity estimation;
- odometry-related state updates;
- stop / safety handling.

Body-frame convention:

```text
+vx = forward
+vy = left
+wz  = counter-clockwise
```

The ROS 2 system therefore commands a full omnidirectional chassis state:

```text
vx
vy
wz
```

rather than only forward velocity and yaw rate.

---

## 6. Mecanum Kinematics

The four-wheel mecanum chassis converts body velocity into four independent
wheel commands.

Conceptually:

```text
        vx / vy / wz
             |
             v
    Mecanum inverse kinematics
             |
     +-------+-------+-------+
     |       |       |       |
     v       v       v       v
   wheel0  wheel1  wheel2  wheel3
```

The wheel commands are then handled by the low-level wheel-control path.

The same wheel feedback is used to estimate chassis motion.

---

## 7. Wheel-Speed Closed Loop

The low-level motor-control chain is:

```text
wheel target
    |
    v
chassis_ctrl
    |
    v
motor control output
    |
    v
MotorHAL
    |
    v
motor driver
    |
    v
wheel
    |
    v
encoder
    |
    v
EncoderHAL
    |
    v
measured wheel speed
    |
    +----------> chassis_ctrl
```

This keeps the real-time wheel loop inside the STM32 rather than depending on
the Raspberry Pi ROS 2 scheduler.

---

## 8. `chassis_hw`

`chassis_hw` provides chassis hardware abstraction.

It separates:

```text
control logic
```

from:

```text
MCU peripheral / motor / encoder details
```

This allows the control layer to work with hardware through a smaller and more
consistent interface.

---

## 9. `MotorHAL`

`MotorHAL` is the motor hardware abstraction layer.

It provides the low-level interface used to command the physical wheel motors.

Its role is below the chassis-control layer:

```text
chassis_ctrl
     |
     v
MotorHAL
     |
     v
timer / GPIO / motor driver
```

This separation keeps high-level mecanum control independent from direct
peripheral manipulation.

---

## 10. `EncoderHAL`

`EncoderHAL` provides wheel-encoder access.

Encoder information is used for:

- wheel-speed feedback;
- chassis velocity estimation;
- odometry-related calculations.

The feedback chain is:

```text
wheel encoder
     |
     v
EncoderHAL
     |
     v
chassis_ctrl
```

---

## 11. IMU Architecture

The robot uses a WIT IMU.

The firmware separates the IMU path into several layers.

```text
UART peripheral
      |
      v
imu_uart_hal
      |
      v
wit_imu_hal
      |
      v
imu_yaw
      |
      v
robot state
```

### `imu_uart_hal`

Provides the low-level UART interface for the IMU.

### `wit_imu_hal`

Provides the project-specific integration layer between the WIT IMU interface
and the robot firmware.

### `imu_yaw`

Handles:

- gyro angular-rate processing;
- yaw integration;
- zero-bias / deadband handling;
- yaw-state maintenance.

---

## 12. Yaw Drift Handling

During real-vehicle testing, long-duration yaw drift was observed.

The practical final gyro zero deadband used during the project was:

```text
ROBOT_GYRO_ZERO_DEADBAND_DPS = 0.50f
```

This value was selected after real-vehicle tests.

It reduced the integration of small residual gyro bias while the robot was
stationary.

This was treated as a practical engineering baseline rather than the ideal
long-term IMU solution.

A future more advanced implementation should use:

```text
stationary detection
+
online gyro-bias estimation
+
stationary bias correction
```

rather than continuously increasing a fixed deadband.

---

## 13. Odometry-Related Parameters

The final documented STM32 baseline included approximately:

```text
ROBOT_AUTO_LAUNCH_ENABLE = 1

ROBOT_ODOM_VX_SCALE = 1.0250
ROBOT_ODOM_VY_SCALE = 0.9400

ROBOT_YAW_RADIUS_CM = 40.72
ROBOT_WZ_MAX_DEG_S = 180.0

ROBOT_GYRO_ZERO_DEADBAND_DPS = 0.50
```

These values were frozen after the final real-vehicle tuning stage.

They should not be changed without new test evidence.

---

## 14. `ros_serial_proto`

`ros_serial_proto` implements the UART communication interface between the STM32
and the Raspberry Pi.

The protocol uses a lightweight human-readable ASCII line format.

Example velocity command:

```text
V vx vy wz
```

Other commands include:

```text
STOP
RESET_ODOM
PING
```

The STM32 also publishes periodic robot-state feedback.

Detailed protocol documentation is available in:

```text
docs/serial_protocol.md
```

---

## 15. Command Flow

The normal autonomous command path is:

```text
Nav2
  |
  v
/cmd_vel_robot
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
  v
chassis_hw / MotorHAL
  |
  v
Four motors
```

This separates:

```text
high-level navigation
```

from:

```text
real-time wheel control
```

---

## 16. Feedback Flow

The main feedback path is:

```text
Wheel Encoders
      |
      v
EncoderHAL
      |
      v
chassis_ctrl
      |
      +----> wheel velocities
      +----> body velocity
      +----> local motion state
      |
      v
robot_app
      |
      v
ros_serial_proto
      |
      | UART
      v
base_serial_bridge
      |
      v
/base/state
      |
      v
base_odometry
      |
      v
/odom
```

The IMU yaw path contributes angular state to the same robot-state chain.

---

## 17. Communication Safety

The STM32 communication layer includes command timeout handling.

If a valid host command is not refreshed for the configured timeout window,
the firmware stops the robot rather than continuing to execute stale velocity
commands.

This provides a simple dead-man behavior between:

```text
Raspberry Pi
```

and:

```text
STM32
```

The detailed timeout and protocol behavior is documented in:

```text
docs/serial_protocol.md
```

---

## 18. Mechanism Control

The firmware also contains mechanism-related modules such as:

```text
mechanism_ctrl
motion_command
```

These functions are kept separate from the core mecanum chassis controller.

This maintains a clearer architecture between:

```text
mobile base control
```

and:

```text
additional mechanism commands
```

---

## 19. Why the STM32 Owns the Wheel Loop

The Raspberry Pi runs:

- ROS 2;
- Nav2;
- AMCL;
- MPPI;
- lidar processing;
- visualization / networking;
- additional sensor nodes.

Its scheduling load is not deterministic enough to directly close the
high-frequency wheel-control loop.

The STM32 therefore owns:

```text
wheel-speed closed loop
```

while the Raspberry Pi supplies:

```text
body-level velocity commands
```

This architecture isolates real-time motor control from Linux / ROS 2 load
variation.

---

## 20. Repository Layout

Relevant firmware structure:

```text
firmware/stm32/
├── Core/
│   ├── Inc/
│   │   ├── chassis_ctrl.h
│   │   ├── chassis_hw.h
│   │   ├── EncoderHAL.h
│   │   ├── MotorHAL.h
│   │   ├── imu_yaw.h
│   │   ├── robot_app.h
│   │   ├── ros_serial_proto.h
│   │   └── ...
│   │
│   └── Src/
│       ├── chassis_ctrl.c
│       ├── chassis_hw.c
│       ├── EncoderHAL.c
│       ├── MotorHAL.c
│       ├── imu_yaw.c
│       ├── robot_app.c
│       ├── ros_serial_proto.c
│       └── ...
│
├── Drivers/
├── MDK-ARM/
└── dfh1.0.ioc
```

---

## 21. Third-Party Components

The firmware repository includes STMicroelectronics HAL and CMSIS components.

These remain subject to their original STMicroelectronics license terms.

The project also interfaces with a WIT Motion IMU.

The original WIT Motion vendor SDK is treated as an external dependency and is
not redistributed as project-authored MIT code.

See:

```text
THIRD_PARTY_NOTICES.md
```

---

## 22. Real-Vehicle Validation

The STM32 firmware was used on the physical four-wheel mecanum robot.

Validated functions included:

- four-wheel velocity closed-loop control;
- forward / backward mecanum motion;
- lateral motion;
- rotation;
- encoder feedback;
- IMU yaw integration;
- serial command reception;
- robot-state feedback;
- odometry-related state generation;
- integration with the ROS 2 / Nav2 stack.

The firmware therefore represents the real low-level control implementation
used during vehicle testing, rather than a demonstration-only example.

---

## 23. Related Documentation

- [System Architecture](system_architecture.md)
- [STM32 / ROS 2 Serial Protocol](serial_protocol.md)
- [Odometry and TF Engineering](odometry_and_tf.md)
- [Real-Vehicle Validation](field_validation.md)
- [Fault Analysis and Debugging](fault_analysis.md)