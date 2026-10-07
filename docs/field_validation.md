# Real-Vehicle Validation

This document summarizes the functions that were actually validated on the
physical greenhouse mecanum robot.

The purpose of this document is to clearly separate:

- real-vehicle validated functions;
- preserved source code;
- preserved configuration snapshots;
- engineering records;
- components whose original ROS 2 source code was not fully preserved.

No functionality is presented here as vehicle-tested unless it was recorded as
such in the original development log.

---

## 1. Final System Chain

The completed robot formed the following closed-loop system:

```text
STM32 Chassis Control
        |
        v
ROS 2 Serial Bridge
        |
        v
Odometry / TF
        |
        v
RPLIDAR + AMCL
        |
        v
Nav2 MPPI Omni
        |
        v
Velocity Smoother
        |
        v
Collision Monitor
        |
        v
twist_mux
        |
        v
ROS 2 Serial Bridge
        |
        v
STM32
        |
        v
Physical Mecanum Robot
```

Additional subsystems included:

```text
Gamepad manual takeover
RS485 environmental sensing
IMX219 camera streaming
USB persistent device identification
systemd automatic startup
Wi-Fi + wired maintenance networking
```

---

## 2. Validation Summary

| Function | Result |
|---|---|
| STM32 chassis firmware | PASS |
| Four-wheel mecanum motion | PASS |
| Wheel encoder feedback | PASS |
| IMU yaw integration | PASS |
| STM32 / Raspberry Pi serial communication | PASS |
| ROS 2 odometry | PASS |
| `odom -> base_link` TF | PASS |
| RPLIDAR A1 integration | PASS |
| AMCL localization | PASS |
| Nav2 `NavigateToPose` | PASS |
| MPPI Omni controller | PASS |
| Final goal orientation | PASS |
| Automatic stop at goal | PASS |
| Gamepad high-priority takeover | PASS |
| Cold-start recovery procedure | PASS |
| RS485 environment sensor | PASS |
| Persistent USB device identification | PASS |
| IMX219 browser video stream | PASS |
| systemd base startup | PASS |
| Wi-Fi automatic connection | PASS |
| Fixed Ethernet maintenance interface | PASS |

---

## 3. Nav2 Real-Vehicle Validation

The final navigation stack used a reduced composed Nav2 architecture.

The managed navigation components were:

```text
controller_server
smoother_server
planner_server
behavior_server
velocity_smoother
collision_monitor
bt_navigator
```

All seven managed components remained active during the final stability test.

The earlier failure mode in which a lifecycle heartbeat timeout caused the
navigation stack to reset was no longer observed in the final baseline.

---

## 4. NavigateToPose

The physical robot successfully completed:

```text
NavigateToPose
```

on the real mecanum platform.

The validated behavior included:

```text
goal received
      |
      v
global path generated
      |
      v
MPPI Omni trajectory execution
      |
      v
vehicle approaches goal
      |
      v
final heading adjustment
      |
      v
automatic stop
```

The navigation test was therefore not limited to planner visualization or
simulation.

The complete command chain reached the physical vehicle.

---

## 5. Final Goal Orientation

The final system used:

```text
SimpleGoalChecker
+
MPPI GoalAngleCritic
```

The orientation of the target pose represented the desired final vehicle
heading.

The robot was validated to:

1. drive to the target position;
2. continue rotating when necessary;
3. reach the requested final orientation;
4. stop automatically.

This confirmed that the goal was treated as a full pose:

```text
x
y
yaw
```

rather than position only.

---

## 6. Gamepad High-Priority Takeover

Manual takeover remained available while Nav2 was running.

The command arbitration chain was:

```text
Nav2 command
      |
      v
      +--------+
               |
               v
           twist_mux
               ^
               |
      +--------+
      |
Gamepad command
```

The gamepad deadman / takeover input had higher priority.

The following behavior was validated on the real robot:

```text
Nav2 driving
      |
      v
press LB
      |
      v
manual command takes priority
      |
      v
operator controls robot
```

This provided a practical operator intervention path during autonomous testing.

---

## 7. Raspberry Pi 4 Stability Validation

The final navigation baseline used:

```text
controller_frequency = 10 Hz

MPPI:
time_steps = 30
model_dt   = 0.10
batch_size = 1000
visualize  = false

velocity_smoother = 10 Hz
```

The final tolerance settings included:

```text
costmap_update_timeout = 0.50 s
failure_tolerance      = 1.00 s
transform_tolerance    = 0.30 s
```

Lifecycle management used:

```text
bond_timeout = 10.0 s
```

These values formed the final documented Raspberry Pi 4 stability baseline.

---

## 8. Costmap Validation

The final local costmap used:

```text
ObstacleLayer
+
InflationLayer
```

rather than the earlier VoxelLayer configuration.

The global costmap used:

```text
StaticLayer
+
ObstacleLayer
+
InflationLayer
```

Both were configured with:

```text
always_send_full_costmap = false
```

The change reduced unnecessary processing and communication load on the
Raspberry Pi 4.

---

## 9. Localization and TF

The final localization chain was:

```text
map
 |
 | AMCL
 v
odom
 |
 | local odometry
 v
base_link
 |
 | static transform
 v
laser
```

The final physical laser transform was:

```text
base_link -> laser

x     = +0.27 m
y     =  0.00 m
z     = +0.42 m

roll  = 0
pitch = 0
yaw   = 0
```

This replaced temporary transform values used during earlier development.

---

## 10. Chassis and Odometry Validation

The STM32 firmware provided:

- mecanum chassis control;
- wheel velocity feedback;
- body velocity state;
- odometry-related state;
- IMU yaw processing;
- serial communication with the Raspberry Pi.

One final motion test included approximately:

```text
forward travel ≈ 0.55 m
rotation       ≈ 26.65°
```

After stopping, the STM32 state was approximately:

```text
x    ≈ 54.75 cm
y    ≈ 0.25 cm
yaw  ≈ 26.65°

vx = 0
vy = 0
wz = 0
```

The ROS 2 odometry reported approximately:

```text
x    ≈ 0.542 m
y    ≈ 0.006 m
yaw  ≈ 26.650°
```

This was used as part of the final odometry / yaw validation.

---

## 11. Cold-Start Validation

The final project was not validated only from an already-running development
session.

A documented cold-start procedure was retained.

At startup:

```text
robot-base.service
```

was expected to automatically start the base control chain.

This included the original system components:

```text
base_serial_bridge
joy_node
gamepad_teleop
twist_mux
```

The final development record explicitly warns against manually launching a
second copy of these nodes because duplicate publishers or nodes could produce
control conflicts.

---

## 12. Localization Startup Procedure

The final localization launch entry was:

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

After a cold start, the operator first supplied:

```text
/initialpose
```

and verified:

```text
map -> base_link
```

before starting the navigation stack.

---

## 13. Final Navigation Entry

The final navigation launch entry was:

```text
navigation_pi4_composed.launch.py
```

with the Nav2 parameter file supplied at launch.

The composed architecture became the formal navigation entry after stability
validation.

The earlier full default Nav2 launch and the abandoned separate-process
lightweight launch were no longer considered the final operating baseline.

---

## 14. Environmental Sensor Validation

The robot integrated an environmental sensor through:

```text
PL2303 USB-RS485
+
Modbus-RTU
```

The final communication settings were:

```text
4800 baud
8N1
slave ID = 1
```

The validated registers represented:

```text
R500 = temperature
R501 = humidity
R502 = noise
R503 = PM2.5
R504 = PM10
```

The ROS 2 environment node published five:

```text
/environment/*
```

topics at approximately:

```text
1 Hz
```

The final system passed both continuous-operation and reconnect testing.

No unsupported environmental quantities such as:

```text
CO2
barometric pressure
light intensity
```

should be claimed for this sensor integration.

---

## 15. USB Device Stability

Dynamic Linux device numbering such as:

```text
/dev/ttyUSB0
/dev/ttyUSB1
/dev/input/js0
```

was not used as the final device-identification strategy.

The final robot used persistent identification.

RPLIDAR and the environmental sensor used:

```text
/dev/serial/by-id
```

The gamepad was selected through:

```text
device_name = Controller
```

This prevented normal USB enumeration-order changes from breaking the startup
configuration.

---

## 16. Camera Validation

The robot used an:

```text
IMX219
```

camera.

The final browser-streaming chain was:

```text
IMX219
   |
   v
libcamera / GStreamer
   |
   v
hardware H.264 encoding
   |
   v
go2rtc
   |
   v
browser / RTSP / WebRTC client
```

The final operating configuration was:

```text
resolution: 640 × 360
frame rate: 10 fps
rate control: CBR
bitrate: 600 kbps
I-frame period: 10 frames
```

The camera streaming path passed final testing, including operation while the
robot was moving.

---

## 17. Network and Maintenance Validation

The final network strategy separated normal wireless use from maintenance.

### Wi-Fi

```text
wlan0 = DHCP
```

This allowed the robot to obtain an address from the current access point.

### Ethernet maintenance port

```text
eth0 = 192.168.50.2/24
```

The wired interface intentionally did not use:

```text
default gateway
DNS
```

so it could remain a predictable maintenance interface independent of the
current Wi-Fi network.

---

## 18. Final Frozen Engineering Baseline

The final development record froze the following major parameters.

### STM32

```text
AUTO_LAUNCH = 1

VX scale  = 1.0250
VY scale  = 0.9400

YAW_RADIUS = 40.72 cm
WZ_MAX     = 180 deg/s

GYRO_DEADBAND = 0.50 deg/s
```

### Nav2

```text
MPPI Omni
footprint = ±0.35 m × ±0.15 m
```

### Raspberry Pi 4

```text
controller = 10 Hz
velocity_smoother = 10 Hz

MPPI = 30 / 0.10 / 1000
visualize = false
```

### Costmap

```text
local:
ObstacleLayer + InflationLayer

global:
StaticLayer + ObstacleLayer + InflationLayer

always_send_full_costmap = false
```

### Lifecycle

```text
navigation_pi4_composed.launch.py
bond_timeout = 10.0 s
```

### Laser

```text
x = +0.27 m
y = 0
z = +0.42 m
yaw = 0
```

These values represent the final documented engineering baseline and should not
be changed without new test evidence.

---

## 19. What Is Preserved in This Repository

The repository preserves:

```text
STM32 firmware source
STM32CubeMX project
Keil project
Nav2 configuration snapshots
Nav2 tuning history
engineering documentation
real-vehicle photos
demonstration material
```

The STM32 firmware is therefore directly inspectable.

---

## 20. What Is Not Fully Preserved

The original Raspberry Pi ROS 2 application-layer source tree was not fully
preserved.

This includes original source for components such as:

```text
base_serial_bridge
base_odometry
gamepad_teleop
custom launch files
environment_sensor
```

Their interfaces, architecture, startup process and validated behavior are
documented from the original development records.

The repository does not present reconstructed implementations as the original
vehicle-tested source.

---

## 21. Configuration Preservation Limitation

The exact final Nav2 YAML file used during the September 10–11 final vehicle
validation was not preserved.

Historical configuration snapshots remain available for engineering reference.

Therefore the repository intentionally distinguishes between:

```text
preserved historical configuration
```

and:

```text
documented final runtime values
```

A future reconstructed YAML must be labeled as reconstructed until it has been
validated again on hardware.

---

## 22. Validation Evidence Philosophy

This repository follows three labels.

### Source available

The original source code is present and can be inspected.

### Vehicle validated

The behavior was recorded as successfully tested on the physical robot.

### Documented from engineering records

The original implementation may no longer be available, but its interfaces,
configuration and observed behavior were preserved in the engineering log.

These labels should not be treated as interchangeable.

For example:

```text
STM32 firmware
    =
source available
+
vehicle validated
```

while part of the original Raspberry Pi ROS 2 application layer is:

```text
original source not fully preserved
+
vehicle behavior validated
+
architecture documented from engineering records
```

This distinction is intentional.

---

## 23. Final Project State

At final handoff, the project had completed the engineering integration of:

```text
STM32 low-level control
        |
ROS 2 chassis bridge
        |
odometry / TF
        |
RPLIDAR / AMCL
        |
Nav2 MPPI Omni
        |
velocity smoothing
        |
collision monitoring
        |
twist_mux
        |
physical vehicle
```

together with:

```text
gamepad takeover
environment sensing
persistent USB identification
camera streaming
automatic startup
Wi-Fi connectivity
fixed Ethernet maintenance
```

The final project state was therefore treated as the 2026 greenhouse mecanum
robot engineering baseline.
