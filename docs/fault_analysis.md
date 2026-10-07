# Fault Analysis and Engineering Debugging

This document summarizes several real faults encountered during development of
the greenhouse mecanum robot.

The goal is not only to list the final fixes, but to preserve the debugging
process:

```text
symptom
  ↓
evidence
  ↓
fault isolation
  ↓
root cause
  ↓
fix
  ↓
validation
```

The project involved STM32 firmware, ROS 2, TF, Nav2, USB devices, camera
streaming and Raspberry Pi system-load constraints.

Many failures therefore required system-level debugging rather than changing a
single parameter.

---

# 1. Raspberry Pi 4 Navigation Overload

## Symptom

During Nav2 testing, the Raspberry Pi 4 entered a high-load condition.

One recorded load average reached approximately:

```text
11.39 / 7.31 / 5.40
```

on a four-core Raspberry Pi 4.

At the same time, the controller repeatedly reported:

```text
Control loop missed its desired rate of 20 Hz
```

The real controller rate sometimes dropped to approximately:

```text
5 ~ 18 Hz
```

Other symptoms appeared at the same time:

```text
TF future extrapolation
collision_monitor invalid source
MPPI optimizer reset / failure
Failed to make progress
lifecycle heartbeat timeout
```

---

## Evidence

Example CPU contributors included:

```text
foxglove_bridge       ≈ 34.8%
base_serial_bridge    ≈ 23.6%
controller_server     ≈ 12.5%
```

with AMCL, joystick handling and other Nav2 processes also consuming CPU.

After Foxglove was closed, the system showed approximately:

```text
64 ~ 69% CPU idle
0% iowait
0 swap activity
```

This confirmed that visualization was a significant load contributor.

However, closing Foxglove did not automatically repair an already damaged
lifecycle state.

Therefore Foxglove was:

```text
a load contributor
```

but not the only system-level cause.

---

## Fix

The final Pi 4 baseline reduced computational load.

Controller:

```text
20 Hz -> 10 Hz
```

Velocity smoother:

```text
20 Hz -> 10 Hz
```

MPPI:

```text
time_steps:
56 -> 30

model_dt:
0.05 -> 0.10

batch_size:
2000 -> 1000

visualize:
true -> false
```

The prediction horizon remained approximately:

```text
30 × 0.10 = 3.0 s
```

which was sufficient for the low-speed robot.

---

## Result

The final system used the reduced 10 Hz baseline rather than attempting to
maintain a nominal 20 Hz controller frequency that the complete Pi 4 workload
could not execute reliably.

---

## Engineering Lesson

A nominally higher control frequency is not automatically better.

```text
stable 10 Hz
```

is preferable to:

```text
requested 20 Hz
+
repeated missed deadlines
+
TF delay
+
controller instability
```

---

# 2. Nav2 Lifecycle Heartbeat Reset

## Symptom

At one stage the Nav2 node states appeared inconsistent.

For example:

```text
controller_server -> active
planner_server    -> inactive
bt_navigator      -> unconfigured
collision_monitor -> active
```

At first glance this could look like several independent node-startup
failures.

---

## Evidence

Detailed logs showed that the default Navigation stack had previously completed:

```text
Configure
+
Activate
```

successfully.

The logs included:

```text
Managed nodes are active
Creating bond timer
```

Therefore the nodes were not simply failing to start.

After approximately:

```text
194 s
```

the lifecycle manager reported that it had not received a heartbeat from:

```text
behavior_server
```

The default approximately:

```text
4000 ms
```

bond timeout was reached.

The lifecycle manager then entered:

```text
CRITICAL FAILURE
```

and began:

```text
Shutting down related nodes
Resetting managed nodes
```

The mixed lifecycle states were therefore observations made while the system
was already in the middle of a managed reset.

---

## Failed Debugging Direction

Repeated manual commands such as:

```text
configure
activate
```

were attempted during some earlier investigations.

This made the lifecycle state more difficult to interpret.

The project therefore stopped using repeated manual lifecycle transitions as a
recovery strategy.

---

## Parameter Verification Lesson

A value was once appended to YAML:

```text
bond_timeout = 10.0
```

but runtime inspection still returned:

```text
4.0
```

This demonstrated an important rule:

> A parameter written in YAML is not necessarily the parameter being used at
> runtime.

Launch-file parameter injection and precedence must also be considered.

---

## Final Fix

The project moved to a reduced composed Navigation architecture containing only:

```text
controller_server
smoother_server
planner_server
behavior_server
velocity_smoother
collision_monitor
bt_navigator
```

The final runtime bond timeout was:

```text
10.0 s
```

and was verified as part of the final configuration.

---

## Engineering Lesson

When lifecycle states appear inconsistent:

```text
do not immediately assume
multiple nodes failed independently
```

First check whether:

```text
lifecycle manager
        |
        v
heartbeat failure
        |
        v
managed reset
```

is already in progress.

---

# 3. FollowPath Abort Before Velocity Output

## Symptom

During one navigation-debugging stage:

```text
ComputePathToPose
```

succeeded, but:

```text
FollowPath
```

aborted.

No useful:

```text
/cmd_vel_nav
```

output was observed in the failing samples.

---

## Fault Isolation

The control chain was:

```text
Planner
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
collision_monitor
  |
  v
twist_mux
  |
  v
STM32
```

If:

```text
planner succeeded
```

but:

```text
FollowPath aborted
```

before any valid:

```text
/cmd_vel_nav
```

was produced, then the fault was upstream of:

```text
velocity_smoother
collision_monitor
twist_mux
STM32
```

The investigation therefore remained focused on:

```text
controller_server / MPPI
```

rather than modifying the chassis firmware.

---

## Action Status Trap

The action status topic contained historical UUIDs.

Therefore seeing:

```text
status = 4
```

somewhere in the topic did not automatically mean the latest goal had
succeeded.

Relevant status meanings included:

```text
4 = SUCCEEDED
5 = CANCELED
6 = ABORTED
```

The latest goal UUID had to be inspected.

---

## Engineering Lesson

Always identify:

```text
the first failing layer
```

before changing downstream systems.

For this case:

```text
global path exists
+
FollowPath fails
+
no velocity command generated
```

meant:

```text
do not debug STM32 yet
```

---

# 4. Long-Term Odometry Yaw Drift

## Symptom

After approximately:

```text
1 hour 12 minutes
```

of continuous odometry operation:

```text
odom -> base_link
```

had drifted to approximately:

```text
-93° ~ -94°
```

even though the physical robot had not rotated by that amount.

---

## Initial Evidence

The original gyro deadband was:

```text
0.05 deg/s
```

While stationary, residual gyro values included approximately:

```text
-0.22
-0.16
-0.10
+0.14
+0.21
-0.28 deg/s
```

These values were individually small.

However:

```text
small constant bias
        |
        v
continuous integration
        |
        v
large long-term yaw error
```

---

## First Fix

The deadband was increased to:

```text
0.35 deg/s
```

This improved stationary behavior.

After actual driving and rotation, however, residual yaw drift of approximately:

```text
0.03 ~ 0.05 deg/s
```

was still observed.

---

## TF-Based Fault Isolation

The transforms were inspected independently.

Observed:

```text
map -> odom
```

remained approximately stable around:

```text
-139.191°
```

while:

```text
odom -> base_link
```

continued changing from approximately:

```text
-2.65°
```

toward:

```text
-3.39°
```

Therefore:

```text
map -> odom stable
+
odom -> base_link drifting
```

strongly indicated:

```text
local odometry / IMU
```

rather than:

```text
AMCL
```

as the source.

---

## Final Practical Fix

The deadband was changed to:

```text
0.50 deg/s
```

Firmware value:

```text
ROBOT_GYRO_ZERO_DEADBAND_DPS = 0.50f
```

---

## Validation

The robot then performed approximately:

```text
0.55 m forward
+
26.65° rotation
+
stop
```

The STM32 settled at approximately:

```text
x    = 54.75 cm
y    = 0.25 cm
yaw  = 26.65°

vx = 0
vy = 0
wz = 0
```

ROS odometry reported approximately:

```text
x    = 0.542 m
y    = 0.006 m
yaw  = 26.650°
```

Yaw remained stable for tens of seconds.

---

## Limitation

The `0.50 deg/s` deadband was a practical engineering solution for this project
stage.

It was not considered the ideal long-term IMU architecture.

A more advanced design should use:

```text
stationary detection
+
online gyro-bias estimation
+
stationary bias correction / yaw freezing
```

instead of continuously increasing the deadband.

---

# 5. USB Dynamic Device Numbering

## Symptom

Linux device names such as:

```text
/dev/ttyUSB0
/dev/ttyUSB1
/dev/input/js0
```

could change when:

- devices were unplugged;
- USB ports changed;
- connection order changed;
- the system rebooted.

This could cause a correctly configured ROS 2 node to connect to the wrong
device or fail to open the expected port.

---

## Devices

The robot included:

```text
RPLIDAR A1 / CH340
environment sensor / PL2303
DragonRise gamepad
```

A single boot might assign:

```text
RPLIDAR      -> /dev/ttyUSB0
environment  -> /dev/ttyUSB1
gamepad      -> /dev/input/js0
```

but those names were not treated as permanent identities.

---

## Final Fix

The RPLIDAR used:

```text
/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

The environmental sensor used its PL2303:

```text
/dev/serial/by-id/...
```

identity.

The gamepad used:

```text
device_name = Controller
```

instead of relying only on:

```text
/dev/input/js0
```

---

## Important Maintenance Issue

The RPLIDAR Jazzy launch file had been modified to use the persistent CH340
`by-id` path.

Because this launch file was under:

```text
/opt/ros/jazzy/
```

a ROS / apt package upgrade could overwrite that modification.

Therefore, if the lidar suddenly returns to:

```text
/dev/ttyUSB0
```

after a system upgrade, the first debugging step should be to inspect the launch
file rather than retune the lidar.

---

## Engineering Lesson

Device identity should describe:

```text
which device it is
```

not merely:

```text
which number Linux assigned during this boot
```

---

# 6. Camera Stream Freezing During Motion

## Symptom

The browser video stream behaved normally while the robot was stationary.

When the vehicle started moving, the video could freeze.

At the same time:

```text
IMX219 / CSI kernel logs showed no disconnect
gst-launch process remained alive
Pi4 load was not abnormally high
```

This suggested the camera hardware itself had not simply disconnected.

---

## First Hypothesis

The camera service originally used scheduling restrictions including:

```text
Nice = 10
CPUWeight = 20
IOSchedulingClass = idle
```

A hypothesis was that Nav2 was starving the camera process.

These restrictions were removed and normal scheduling priority was restored.

The problem still occurred.

Therefore:

```text
camera process starvation
```

was not accepted as the final root cause.

---

## Encoder Investigation

The hardware H.264 encoder was then inspected.

Default values included approximately:

```text
video_bitrate = 10,000,000 bit/s
rate mode     = VBR
I-frame period = 60
```

The camera used a:

```text
160° wide-angle view
```

During vehicle movement, a large percentage of the image changed every frame.

This created significantly more encoded scene motion and recovery pressure than
a stationary image.

---

## Final Fix

The final stream was changed to:

```text
640 × 360
10 fps

CBR
600 kbps

I-frame period = 10
```

At 10 fps:

```text
I-frame every ≈ 1 s
```

The robot was then tested while moving.

The browser video remained stable.

The issue was considered resolved.

---

## Engineering Lesson

The first plausible explanation is not always the root cause.

The debugging sequence was:

```text
motion causes freeze
      |
      v
check CSI / camera disconnect
      |
      v
check process survival
      |
      v
check Pi4 CPU load
      |
      v
test scheduling hypothesis
      |
      v
problem remains
      |
      v
inspect encoder parameters
      |
      v
reduce bitrate + CBR + shorter GOP
      |
      v
real-vehicle validation
```

---

# 7. AMCL Initial Pose and TF Timing

## Symptom

After a cold localization startup, AMCL repeatedly reported that it could not
publish a pose until an initial pose had been provided.

Before `/initialpose`:

```text
map -> odom
```

did not exist.

Therefore:

```text
map -> base_link
```

was also unavailable.

---

## Additional Timing Problem

During high-load testing, republishing `/initialpose` sometimes produced:

```text
Failed to transform initial pose in time
```

or:

```text
Lookup would require extrapolation into the future
```

At one high-load point, `/odom` was approximately:

```text
9 ~ 10 Hz
```

and a maximum observed interval was approximately:

```text
0.275 s
```

This suggested a TF / scheduling timing boundary rather than automatically
indicating that AMCL itself was fundamentally broken.

---

## Final Operating Rule

The startup sequence was formalized as:

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
start / verify Navigation
      |
      v
send goal
```

The system should not immediately send repeated goals before localization is
confirmed.

---

# 8. Environmental Sensor Serial Failures

## Final Interface

The environmental sensor used:

```text
Modbus-RTU
slave = 1
4800 baud
8N1
```

with registers:

```text
500 temperature
501 humidity
502 noise
503 PM2.5
504 PM10
```

---

## Serial Open Failure

If the ROS 2 node could not open the sensor port, the first checks were:

```text
PL2303 by-id exists?
sensor powered?
user has serial permissions?
```

The register map was not the first thing to change.

---

## Serial Busy

Only one program can own the same serial port.

Therefore the final rule was:

```text
do not run
environment_sensor
and
raw Python read_sensor.py
at the same time
```

---

## No Response

If the port opened but the device returned no data, the diagnostic order was:

```text
power
A/B wiring
slave ID
4800 8N1
```

The already validated register addresses were not repeatedly guessed again.

---

# 9. System-Level Debugging Strategy

The final project adopted a layered debugging order:

```text
Physical device identity
        |
        v
systemd service
        |
        v
ROS 2 node
        |
        v
serial / sensor interface
        |
        v
odometry
        |
        v
TF
        |
        v
localization
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
        |
        v
STM32 / actuator
```

The main principle was:

> Do not modify a lower layer unless evidence indicates the fault has reached
> that layer.

---

# 10. Engineering Rules Preserved From Development

## One goal at a time

Do not rapidly send multiple `/goal_pose` commands.

Multiple goals can introduce:

```text
preemption
recovery
mixed action history
additional CPU load
```

---

## Check the latest action UUID

Do not infer current navigation success from an old status entry.

---

## Do not manually repair a resetting lifecycle repeatedly

If the lifecycle manager is already resetting nodes, repeated manual
`configure` / `activate` commands make the state harder to diagnose.

---

## Verify runtime parameters

Do not assume YAML values are active.

Use live parameter inspection when parameter precedence matters.

---

## Do not retune already validated subsystems without evidence

During the final stage, the following were treated as frozen unless new
reproducible evidence appeared:

```text
STM32 chassis parameters
Omni motion model
RPLIDAR protocol / transform
gyro deadband
environment sensor register map
camera encoder baseline
USB persistent identities
```

---

# 11. Main Engineering Takeaway

The most important lesson from this project was that a robotics failure often
crosses several software and hardware layers.

For example:

```text
Robot does not move toward goal
```

does not directly imply:

```text
motor-control problem
```

The actual fault may be:

```text
planner
controller
TF
lifecycle
CPU overload
command arbitration
serial bridge
STM32
```

The project therefore evolved toward evidence-driven debugging:

```text
observe
  ↓
identify first failing layer
  ↓
change one variable
  ↓
retest
  ↓
freeze validated result
```

This approach was used throughout the final integration and vehicle-validation
stage.
