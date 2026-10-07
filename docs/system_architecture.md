# System Architecture

This document provides a system-level overview of the greenhouse mecanum robot.

The robot integrates:

- STM32 low-level chassis control;
- Raspberry Pi 4 running ROS 2 Jazzy;
- wheel encoder and IMU based odometry;
- RPLIDAR A1;
- AMCL localization;
- Nav2 MPPI Omni navigation;
- velocity smoothing and collision monitoring;
- high-priority gamepad takeover;
- RS485 environmental sensing;
- IMX219 video streaming;
- persistent USB device identification;
- Wi-Fi operation and fixed Ethernet maintenance access.

The final engineering baseline was validated on a physical four-wheel mecanum
robot.

---

## 1. System Overview

The complete robot can be divided into five major layers:

```text
+--------------------------------------------------+
|                 User / Operator                  |
| Goal Pose / Gamepad / Monitoring / Camera       |
+-------------------------+------------------------+
                          |
                          v
+--------------------------------------------------+
|            ROS 2 Navigation Layer                |
| Nav2 / AMCL / MPPI / Costmaps / Lifecycle       |
+-------------------------+------------------------+
                          |
                          v
+--------------------------------------------------+
|          ROS 2 Robot Interface Layer             |
| Serial Bridge / Odometry / twist_mux / Sensors  |
+-------------------------+------------------------+
                          |
                          v
+--------------------------------------------------+
|             STM32 Control Layer                  |
| Kinematics / Wheel Control / IMU / Odometry      |
+-------------------------+------------------------+
                          |
                          v
+--------------------------------------------------+
|                Physical Robot                    |
| Motors / Encoders / Mecanum Wheels / Sensors    |
+--------------------------------------------------+
```

---

## 2. Hardware Architecture

Main hardware:

```text
Raspberry Pi 4
STM32F407ZGT6
Four mecanum wheels
Four wheel encoders
RPLIDAR A1
WIT IMU
IMX219 camera
RS485 environmental sensor
USB gamepad receiver
```

The Raspberry Pi handled high-level robotics tasks.

The STM32 handled deterministic low-level chassis control.

---

## 3. High-Level Control Architecture

The validated autonomous navigation path was:

```text
Foxglove / Goal Pose
        |
        v
bt_navigator
        |
        v
planner_server
        |
        v
MPPI Controller
motion_model = Omni
        |
        v
/cmd_vel_nav
        |
        v
velocity_smoother
        |
        v
/cmd_vel_smoothed
        |
        v
collision_monitor
        |
        v
/cmd_vel_nav_safe
        |
        v
twist_mux  <------ Gamepad /cmd_vel_teleop
        |
        v
/cmd_vel_robot
        |
        v
base_serial_bridge
        |
        | UART ASCII protocol
        v
STM32F407
        |
        v
Four-wheel velocity closed loop
        |
        v
Physical mecanum chassis
```

The gamepad command had higher priority than Nav2.

This allowed manual takeover during autonomous operation.

---

## 4. Why the Navigation Model Is Omni

The chassis is a four-wheel mecanum platform.

It can generate:

```text
vx
vy
wz
```

independently.

Therefore the Nav2 controller uses:

```text
motion_model = Omni
```

rather than a differential-drive model.

The final documented velocity limits were approximately:

```text
vx_max =  0.30 m/s
vx_min = -0.20 m/s

vy_max =  0.20 m/s

wz_max =  0.70 rad/s
```

This enables:

- forward / backward motion;
- lateral translation;
- diagonal motion;
- simultaneous translation and rotation.

---

## 5. ROS 2 Navigation Layer

The final navigation architecture retained seven main managed components:

```text
controller_server
smoother_server
planner_server
behavior_server
velocity_smoother
collision_monitor
bt_navigator
```

These were launched using the final composed entry:

```text
navigation_pi4_composed.launch.py
```

and managed by:

```text
lifecycle_manager_navigation
```

The final runtime lifecycle bond timeout was:

```text
10.0 s
```

The reduced composed architecture replaced the heavier default Nav2 launch as
the formal operating baseline.

---

## 6. Planner and Controller

Global planner:

```text
nav2_navfn_planner::NavfnPlanner
```

Local controller:

```text
nav2_mppi_controller::MPPIController
```

Motion model:

```text
Omni
```

The final Pi 4 MPPI baseline was:

```text
controller_frequency = 10 Hz

time_steps = 30
model_dt   = 0.10
batch_size = 1000

visualize = false
```

The system intentionally prioritized stable real-time execution over a higher
nominal control frequency.

---

## 7. Costmap Architecture

The final local costmap used:

```text
frame: odom
rolling window: approximately 3 m × 3 m
layers:
  ObstacleLayer
  InflationLayer
```

The final global costmap used:

```text
frame: map
layers:
  StaticLayer
  ObstacleLayer
  InflationLayer
```

The final baseline used:

```text
always_send_full_costmap = false
```

to reduce Raspberry Pi communication and processing load.

---

## 8. Robot Footprint

The physical chassis was approximately:

```text
0.70 m × 0.30 m
```

The Nav2 rectangular footprint used approximately:

```text
x = ±0.35 m
y = ±0.15 m
```

This footprint was used for collision checking during navigation.

---

## 9. Localization Architecture

The final TF chain was:

```text
map
 |
 | AMCL
 v
odom
 |
 | base_odometry
 v
base_link
 |
 | static transform
 v
laser
```

Responsibilities:

### `map -> odom`

Published by:

```text
AMCL
```

This provides map-relative localization.

### `odom -> base_link`

Published by the original:

```text
base_odometry
```

ROS 2 node.

This represents continuous local robot motion.

### `base_link -> laser`

Static transform corresponding to the physical lidar installation.

---

## 10. AMCL

AMCL used an omnidirectional robot model:

```text
nav2_amcl::OmniMotionModel
```

This matches the mecanum chassis motion characteristics.

After startup, the operator supplied:

```text
/initialpose
```

before autonomous navigation.

The expected localization sequence was:

```text
start localization
      |
      v
publish /initialpose
      |
      v
confirm map -> base_link
      |
      v
start / verify navigation
```

---

## 11. Final Laser Transform

The final mechanical installation was represented as:

```text
base_link -> laser

x     = +0.27 m
y     =  0.00 m
z     = +0.42 m

roll  = 0
pitch = 0
yaw   = 0
```

Earlier temporary transforms used during bringup were replaced with this
measured installation value.

---

## 12. ROS 2 Robot Interface Layer

The original Raspberry Pi ROS 2 workspace contained several project-specific
packages.

### `base_serial_bridge`

Responsibilities:

```text
subscribe /cmd_vel_robot
convert ROS velocity command
send STM32 UART command
receive STM32 state
publish /base/state
```

The bridge communicated with the STM32 using a lightweight ASCII protocol.

---

### `base_odometry`

Responsibilities:

```text
subscribe /base/state
integrate chassis motion
publish /odom
publish odom -> base_link
```

Body-frame velocity was rotated using current yaw before integration into the
odom frame.

---

### `gamepad_teleop`

Responsibilities:

```text
read USB gamepad
LB deadman / takeover
publish /cmd_vel_teleop
```

---

### `twist_mux`

The final arbitration concept was approximately:

```text
Nav2 priority     ≈ 50
Teleop priority   ≈ 100
```

so manual control could override autonomous navigation.

---

## 13. STM32 Communication Interface

The Raspberry Pi communicated with the STM32 over UART.

Velocity command:

```text
V vx vy wz
```

Other supported commands included:

```text
STOP
RESET_ODOM
PING
```

The STM32 returned an ASCII robot-state message.

Detailed protocol documentation is available in:

```text
docs/serial_protocol.md
```

---

## 14. STM32 Firmware Architecture

Important firmware modules include:

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
```

Key responsibilities:

### `chassis_ctrl`

- mecanum kinematics;
- wheel-speed control;
- chassis motion state;
- odometry-related calculations.

### `chassis_hw`

- low-level chassis hardware abstraction.

### `ros_serial_proto`

- STM32 / Raspberry Pi ASCII communication.

### `imu_yaw`

- gyro processing;
- yaw integration;
- gyro-bias handling.

### `robot_app`

- application-level coordination;
- robot-state packaging;
- command handling.

Detailed firmware architecture:

```text
docs/stm32_firmware_architecture.md
```

---

## 15. Feedback Architecture

The low-level feedback path was conceptually:

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
      +----> local state
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
      |
      v
odom -> base_link
```

The WIT IMU provided the yaw / angular-rate path used by the low-level system.

---

## 16. Environmental Sensor Architecture

The environmental subsystem used:

```text
RS485 environmental sensor
        |
        | Modbus-RTU
        v
PL2303 USB-RS485
        |
        v
Raspberry Pi
        |
        v
environment_sensor ROS 2 package
        |
        v
/environment/*
```

The validated measurements were:

```text
temperature
humidity
noise
PM2.5
PM10
```

The final poll rate was approximately:

```text
1 Hz
```

The sensor did not provide:

```text
CO2
barometric pressure
light intensity
```

and those quantities should not be claimed.

---

## 17. Camera Architecture

The final camera path was:

```text
IMX219
   |
   v
libcamera / GStreamer
   |
   v
v4l2h264enc
Pi 4 hardware H.264
   |
   v
h264parse
   |
   v
go2rtc
   |
   +----> Browser
   +----> RTSP
   +----> WebRTC / MSE
```

Final operating baseline:

```text
640 × 360
10 fps
CBR 600 kbps
I-frame period = 10
```

---

## 18. USB Device Architecture

The final system avoided dynamic device identities such as:

```text
/dev/ttyUSB0
/dev/ttyUSB1
/dev/input/js0
```

Persistent identification was used instead.

### RPLIDAR

```text
/dev/serial/by-id/...
```

### Environmental sensor

```text
/dev/serial/by-id/...
```

### Gamepad

```text
device_name = Controller
```

This reduced failures caused by USB enumeration-order changes.

---

## 19. Automatic Startup

The final system used:

```text
robot-base.service
```

to automatically start the base-control chain.

The original service launched:

```text
base_serial_bridge
joy_node
gamepad_teleop
twist_mux
```

The camera system used:

```text
robot-camera.service
```

The final operation procedure intentionally avoided manually launching duplicate
instances of these nodes.

---

## 20. Final Localization Entry

The formal localization launch file was:

```text
greenhouse_localization_final.launch.py
```

It started the original system's:

```text
base_odometry
RPLIDAR
base_link -> laser static TF
map_server
AMCL
localization lifecycle manager
```

---

## 21. Final Navigation Entry

The formal navigation launch file was:

```text
navigation_pi4_composed.launch.py
```

The final system no longer used:

```text
default full navigation_launch.py
```

as its formal daily navigation entry.

An intermediate separate-process lightweight architecture was also abandoned.

---

## 22. Network Architecture

Normal operation:

```text
wlan0 = DHCP
```

Maintenance interface:

```text
eth0 = 192.168.50.2/24
```

The Ethernet interface intentionally had:

```text
no default gateway
no DNS
```

Its purpose was to provide predictable maintenance access even when Wi-Fi was
unavailable.

---

## 23. Complete Data Flow

The complete navigation and control data flow can be summarized as:

```text
                         USER
                          |
                  Goal / Gamepad
                          |
                          v
                 +----------------+
                 |    ROS 2       |
                 | Navigation     |
                 +-------+--------+
                         |
                         v
                  Nav2 Planner
                         |
                         v
                  MPPI Omni
                         |
                         v
                velocity_smoother
                         |
                         v
                collision_monitor
                         |
                         v
       Gamepad ------> twist_mux
                         |
                         v
                base_serial_bridge
                         |
                       UART
                         |
                         v
                      STM32
                         |
              +----------+----------+
              |                     |
              v                     v
      Mecanum Wheel Control      IMU / State
              |                     |
              v                     |
         Physical Robot             |
              |                     |
              +------ Encoders -----+
                         |
                         v
                  Robot State
                         |
                       UART
                         |
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
                         |
                         v
                odom -> base_link
                         |
                         v
                       AMCL
                         |
                         v
                    map -> odom
```

---

## 24. Final Engineering Baseline

The final system formed the following validated chain:

```text
STM32 low-level closed-loop control
        |
        v
ROS 2 chassis bridge
        |
        v
odometry / TF
        |
        v
RPLIDAR / AMCL
        |
        v
Nav2 MPPI Omni
        |
        v
velocity smoothing
        |
        v
collision monitoring
        |
        v
twist_mux
        |
        v
physical robot
```

The project also completed:

```text
gamepad takeover
RS485 environmental sensing
persistent USB device identity
IMX219 browser streaming
automatic startup
Wi-Fi operation
fixed Ethernet maintenance access
```

This architecture represents the final 2026 greenhouse mecanum robot
engineering baseline.

---

## 25. Repository Source Availability

The STM32 firmware source is preserved in this repository.

The original Raspberry Pi ROS 2 application-layer source tree was not fully
preserved.

Therefore components such as:

```text
base_serial_bridge
base_odometry
gamepad_teleop
environment_sensor
custom launch files
```

are documented from the original engineering records but should not be
presented as preserved original source.

Any future recreation should be labeled as reconstructed until revalidated on
hardware.

---

## 26. Related Documentation

- [STM32 Firmware Architecture](stm32_firmware_architecture.md)
- [STM32 / ROS 2 Serial Protocol](serial_protocol.md)
- [Nav2 / MPPI Tuning and Stability](nav2_tuning.md)
- [Odometry and TF Engineering](odometry_and_tf.md)
- [Real-Vehicle Validation](field_validation.md)
- [Bringup and Operation Guide](bringup_and_operation.md)
- [Fault Analysis and Debugging](fault_analysis.md)
