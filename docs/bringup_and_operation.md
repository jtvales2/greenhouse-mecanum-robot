# Bringup and Operation Guide

This document describes the final daily startup and operating procedure for the
greenhouse mecanum robot.

The procedure is reconstructed directly from the final engineering handoff
records.

The original Raspberry Pi ROS 2 application-layer source tree was not fully
preserved, so the launch files and services described here document the
original validated system rather than providing a complete currently
rebuildable Raspberry Pi workspace.

---

## 1. Final Startup Architecture

The final robot startup sequence was divided into three layers:

```text
Power On
   |
   v
systemd base services
   |
   v
Localization
   |
   v
Initial Pose / TF verification
   |
   v
Nav2 Navigation
   |
   v
NavigateToPose
```

The base-control chain was automatically started by:

```text
robot-base.service
```

The camera subsystem was managed by:

```text
robot-camera.service
```

---

## 2. Important Rule

Do not manually launch duplicate copies of the base-control nodes after
`robot-base.service` is already active.

The final system automatically started the original components:

```text
base_serial_bridge
joy_node
gamepad_teleop
twist_mux
```

Starting another copy manually may create:

- duplicate publishers;
- duplicate nodes;
- command conflicts;
- difficult-to-diagnose control behavior.

Always check the systemd service first.

---

## 3. Step 1 — Check Base Services

After powering on the Raspberry Pi, check:

```bash
systemctl is-active robot-base.service
```

Expected result:

```text
active
```

Also check the camera service:

```bash
systemctl is-active robot-camera.service
```

Expected result:

```text
active
```

If `robot-base.service` is active, do not manually start:

```text
base_serial_bridge
joy_node
gamepad_teleop
twist_mux
```

again.

---

## 4. Step 2 — Start Localization

Open Terminal A.

Source ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

Source the robot workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

Start the final localization launch:

```bash
ros2 launch ~/ros2_ws/launch/greenhouse_localization_final.launch.py
```

The original launch started:

```text
base_odometry
RPLIDAR
base_link -> laser static TF
map_server
AMCL
localization lifecycle manager
```

The formal laser transform was:

```text
base_link -> laser

x     = +0.27 m
y     =  0.00 m
z     = +0.42 m

roll  = 0
pitch = 0
yaw   = 0
```

---

## 5. Step 3 — Set Initial Pose

After localization starts, the robot must receive an initial pose.

The final workflow used Foxglove to publish:

```text
/initialpose
```

The initial pose should correspond to the robot's actual location and heading
on the map.

Do not immediately start autonomous navigation before the localization state is
confirmed.

---

## 6. Step 4 — Verify TF

After setting `/initialpose`, verify that:

```text
map -> base_link
```

is continuously available.

A useful command is:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

The transform should update continuously without repeatedly disappearing.

The expected localization chain is:

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
 | static TF
 v
laser
```

Only after this chain is healthy should the operator continue to Nav2.

---

## 7. Step 5 — Optional Foxglove Session

Open Terminal B.

Source the environment:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Start Foxglove Bridge:

```bash
ros2 run foxglove_bridge foxglove_bridge
```

During normal Pi 4 operation, visualization should remain minimal.

Useful displays include:

```text
/map
TF
/plan
```

After `/initialpose` is set, Foxglove can be stopped if it is no longer needed.

This reduces Raspberry Pi 4 load.

If Foxglove is still required for publishing:

```text
/goal_pose
```

keep only the necessary panels active.

---

## 8. Step 6 — Start Final Navigation

After localization and TF are confirmed, start the final composed Nav2 stack.

Source the environment if necessary:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Launch:

```bash
ros2 launch ~/ros2_ws/launch/navigation_pi4_composed.launch.py \
  params_file:=/home/ubt/ros2_ws/config/nav2_params_mecanum.yaml
```

This was the final formal navigation entry.

The project no longer used the default full:

```text
navigation_launch.py
```

as the final operating entry.

The earlier separate-process lightweight launch was also abandoned.

---

## 9. Final Managed Navigation Components

The final composed navigation stack retained:

```text
controller_server
smoother_server
planner_server
behavior_server
velocity_smoother
collision_monitor
bt_navigator
```

These components were managed by:

```text
lifecycle_manager_navigation
```

The final lifecycle bond timeout was:

```text
10.0 s
```

---

## 10. Step 7 — Send a Goal

After Nav2 becomes active, send a:

```text
NavigateToPose
```

goal.

The final robot was validated to:

```text
plan path
   |
   v
generate MPPI Omni commands
   |
   v
drive toward target
   |
   v
adjust final orientation
   |
   v
stop automatically
```

The goal orientation is meaningful.

The robot may continue rotating after reaching the target position until the
requested final heading is reached.

---

## 11. Manual Takeover

The final robot retained gamepad control while Nav2 was active.

Command arbitration used:

```text
Nav2
  |
  v
twist_mux
  ^
  |
Gamepad
```

The gamepad LB deadman / takeover command had higher priority.

Validated behavior:

```text
Nav2 running
    |
press LB
    |
    v
manual control takes priority
```

This allows the operator to intervene during autonomous navigation.

---

## 12. Navigation Control Chain

The final command path was:

```text
Goal Pose
   |
   v
bt_navigator
   |
   v
planner_server
   |
   v
MPPI Controller
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
twist_mux
   |
   v
/cmd_vel_robot
   |
   v
base_serial_bridge
   |
   | UART
   v
STM32
   |
   v
Four-wheel mecanum chassis
```

---

## 13. Final Navigation Baseline

The final documented Pi 4 baseline included:

```text
controller_frequency = 10 Hz
velocity_smoother    = 10 Hz
```

MPPI:

```text
time_steps = 30
model_dt   = 0.10
batch_size = 1000
visualize  = false
```

Controller / TF tolerances:

```text
costmap_update_timeout = 0.50 s
failure_tolerance      = 1.00 s
transform_tolerance    = 0.30 s
```

Lifecycle:

```text
bond_timeout = 10.0 s
```

These values should not be changed during normal bringup without new test
evidence.

---

## 14. Final Costmap Baseline

Local costmap:

```text
ObstacleLayer
+
InflationLayer
```

Global costmap:

```text
StaticLayer
+
ObstacleLayer
+
InflationLayer
```

Both used:

```text
always_send_full_costmap = false
```

The earlier local VoxelLayer configuration was not part of the final baseline.

---

## 15. Camera Access

The camera subsystem used:

```text
IMX219
+
GStreamer / libcamera
+
hardware H.264
+
go2rtc
```

The final stream configuration was:

```text
640 × 360
10 fps
CBR 600 kbps
I-frame period = 10
```

The camera service remained resident, while the camera / GStreamer pipeline
could be started when a viewing client connected.

---

## 16. Wireless Access

The Wi-Fi interface used:

```text
wlan0 = DHCP
```

SSH access:

```bash
ssh ubt@<wlan0-IP>
```

Camera access:

```text
http://<wlan0-IP>:1984/stream.html?src=robot_cam
```

The actual wireless IP depends on the network DHCP server.

---

## 17. Wired Maintenance Access

A fixed Ethernet maintenance interface was retained:

```text
eth0 = 192.168.50.2/24
```

The maintenance computer should use an address such as:

```text
192.168.50.1/24
```

SSH:

```bash
ssh ubt@192.168.50.2
```

Camera:

```text
http://192.168.50.2:1984/stream.html?src=robot_cam
```

The Ethernet interface intentionally did not use a default gateway or DNS
configuration.

Its role was to provide a predictable recovery connection independent of the
current Wi-Fi network.

---

## 18. USB Device Identification

The final system did not depend on dynamic Linux names such as:

```text
ttyUSB0
ttyUSB1
js0
```

The RPLIDAR and environmental sensor used:

```text
/dev/serial/by-id
```

The gamepad used:

```text
device_name = Controller
```

This should be preserved during normal maintenance.

---

## 19. Recommended Startup Checklist

Before sending an autonomous goal, verify the following sequence:

```text
[1] Raspberry Pi powered on

[2] robot-base.service = active

[3] robot-camera.service = active

[4] Localization launch running

[5] RPLIDAR active

[6] Map loaded

[7] /initialpose supplied

[8] map -> base_link TF continuously available

[9] Final composed Nav2 launch running

[10] Navigation lifecycle nodes active

[11] Gamepad takeover available

[12] Send NavigateToPose goal
```

Do not skip the localization / TF verification step.

---

## 20. Common Operational Mistakes

### Starting duplicate base nodes

Do not manually start another:

```text
base_serial_bridge
joy_node
gamepad_teleop
twist_mux
```

when `robot-base.service` is already active.

---

### Starting Nav2 before localization is ready

Do not send autonomous goals before:

```text
/initialpose
```

has been supplied and:

```text
map -> base_link
```

has been confirmed.

---

### Running excessive visualization

The Raspberry Pi 4 previously experienced navigation instability under high
system load.

Keep visualization minimal during normal autonomous operation.

---

### Returning to the full default Nav2 launch

The final project baseline used:

```text
navigation_pi4_composed.launch.py
```

Do not replace it with the full default Nav2 launch without a specific reason
and new validation.

---

### Depending on ttyUSB numbering

Do not replace persistent USB identities with:

```text
ttyUSB0
ttyUSB1
```

because enumeration order can change after reconnect or reboot.

---

## 21. Recovery Philosophy

If the robot does not navigate correctly, debug from the lowest confirmed layer
upward.

Recommended sequence:

```text
systemd services
        |
        v
STM32 / serial bridge
        |
        v
odometry
        |
        v
TF
        |
        v
RPLIDAR
        |
        v
AMCL
        |
        v
Nav2 lifecycle
        |
        v
planner
        |
        v
controller
        |
        v
velocity command chain
```

Do not immediately retune Nav2 parameters when the actual failure is in a lower
layer.

---

## 22. Repository Limitation

The launch files and Raspberry Pi ROS 2 application source referenced in this
guide were part of the original validated system.

Their complete original source tree was not preserved in this repository.

Therefore this document is an operational and architectural record of the
vehicle-tested system.

Any future recreation of those files should be marked as reconstructed until
revalidated on the physical robot.
