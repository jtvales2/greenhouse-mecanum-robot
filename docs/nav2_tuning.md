# Nav2 / MPPI Tuning and Stability Engineering

This document summarizes the real-vehicle Nav2 tuning process used on the
greenhouse mecanum robot.

The purpose of the tuning process was not to maximize theoretical controller
performance. The final objective was to obtain a stable, reproducible
navigation stack on a Raspberry Pi 4 while preserving omnidirectional mecanum
motion.

---

## 1. Navigation Platform

### Hardware

- Raspberry Pi 4
- STM32F407ZGT6 chassis controller
- Four-wheel mecanum chassis
- RPLIDAR A1
- WIT IMU
- Wheel encoders

### Software

- Ubuntu 24.04
- ROS 2 Jazzy
- Nav2
- AMCL
- MPPI Controller
- Omni motion model

The final navigation control chain was:

```text
Goal Pose
    |
    v
Nav2 Planner
    |
    v
MPPI Omni Controller
    |
    v
velocity_smoother
    |
    v
collision_monitor
    |
    v
twist_mux  <----- Gamepad Override
    |
    v
base_serial_bridge
    |
    v
STM32
    |
    v
Four-wheel mecanum chassis
```

---

## 2. Why Omni Motion Model

The robot uses a four-wheel mecanum chassis.

Unlike a differential-drive robot, the chassis can generate:

```text
vx
vy
wz
```

simultaneously.

Therefore the navigation controller must support lateral motion.

The final Nav2 configuration uses:

```text
motion_model: Omni
```

This allows the MPPI controller to generate:

- forward / backward motion;
- lateral motion;
- diagonal motion;
- simultaneous translation and rotation.

Historical configuration snapshots are preserved under:

```text
config/tuning_history/
```

including an earlier pre-Omni configuration for comparison.

---

## 3. Initial Performance Problem

During early real-vehicle testing, the Raspberry Pi 4 experienced significant
system load while running the complete navigation stack.

Observed symptoms included:

- controller loop missing the requested rate;
- control frequency dropping below the requested 20 Hz;
- TF future-extrapolation warnings;
- collision-monitor source timing problems;
- MPPI optimizer reset / failure;
- lifecycle heartbeat timeout;
- eventual shutdown / reset of Nav2 managed nodes.

One recorded high-load condition reached approximately:

```text
load average:
11.39 / 7.31 / 5.40
```

on the four-core Raspberry Pi 4.

The debugging process showed that the problem could not be treated simply as
an MPPI tuning issue.

CPU scheduling, visualization load, TF timing and lifecycle management were all
part of the system-level failure chain.

---

## 4. Raspberry Pi 4 Load Reduction

The initial controller frequency was:

```text
20 Hz
```

The final validated value was reduced to:

```text
10 Hz
```

The MPPI configuration was also reduced.

### MPPI evolution

Earlier configuration:

```text
time_steps: 56
model_dt: 0.05
batch_size: 2000
visualize: true
```

Final Pi 4 baseline:

```text
time_steps: 30
model_dt: 0.10
batch_size: 1000
visualize: false
```

This reduced:

- trajectory sampling cost;
- visualization overhead;
- controller CPU demand;
- communication / rendering pressure.

The final prediction horizon remained approximately:

```text
30 × 0.10 s = 3.0 s
```

which was sufficient for the low-speed greenhouse robot.

---

## 5. Controller Frequency Alignment

The final operating frequencies were aligned around the Pi 4 processing
capability.

```text
controller_frequency: 10 Hz
velocity_smoother:    10 Hz
```

This avoided running the smoother significantly faster than the actual
controller loop.

The objective was deterministic real-vehicle behavior rather than maximizing
nominal loop frequency.

---

## 6. Progress Checker Tuning

Low-speed mecanum motion caused the original progress checker to sometimes
judge the robot too aggressively.

The tuning evolved from approximately:

```text
required_movement_radius:
0.5 m -> 0.2 m

movement_time_allowance:
10 s -> 15 s
```

The robot operated at relatively low navigation speeds, so smaller valid
progress over a longer time window was expected.

This reduced unnecessary navigation aborts during slow real-world motion.

---

## 7. Controller and TF Tolerance

Real testing showed scheduling jitter and TF delays on the Raspberry Pi.

Three important tolerances were increased.

### Costmap update timeout

```text
0.30 s -> 0.50 s
```

### Controller failure tolerance

```text
0.30 s -> 1.00 s
```

### MPPI transform tolerance

```text
0.10 s -> 0.30 s
```

These changes were not intended to hide persistent failures.

Their purpose was to prevent short scheduling or TF delays from immediately
causing `FollowPath` to abort.

---

## 8. FollowPath Fault Isolation

During debugging, the system was analyzed layer by layer.

A typical failure pattern was:

```text
ComputePathToPose = SUCCEEDED
FollowPath        = ABORTED
/cmd_vel_nav      = no valid output
```

This is an important diagnostic distinction.

If global planning succeeds but no controller velocity is generated, debugging
should remain inside:

```text
controller_server / MPPI
```

instead of immediately investigating:

```text
velocity_smoother
collision_monitor
twist_mux
STM32
```

This layer-by-layer method prevented unrelated parts of the system from being
reconfigured during controller debugging.

---

## 9. Lifecycle Heartbeat Failure

One of the most important stability problems was initially mistaken for a
general Nav2 startup problem.

The full default Navigation stack actually configured and activated
successfully.

After running for a period of time, the lifecycle manager failed to receive a
heartbeat from:

```text
behavior_server
```

The default approximately:

```text
4 s
```

bond timeout then caused the lifecycle manager to reset the managed navigation
nodes.

This produced apparently inconsistent lifecycle states such as:

```text
controller_server -> active
planner_server    -> inactive
bt_navigator      -> unconfigured
```

These mixed states were a consequence of the lifecycle reset process rather
than independent node failures.

---

## 10. Why the Default Nav2 Launch Was Replaced

The standard Nav2 navigation launch included components that were not required
for this robot's main `NavigateToPose` task.

Examples included:

- route server;
- waypoint follower;
- docking server.

These components added additional process, DDS and computation overhead on the
Raspberry Pi 4.

The project therefore moved to a reduced Navigation stack.

---

## 11. Failed Lightweight Architecture

An intermediate experiment launched the Nav2 nodes as separate lightweight
processes.

This approach was expected to reduce complexity.

In practice, it increased:

- process count;
- DDS communication overhead;
- lifecycle / parameter service latency.

Lifecycle and parameter CLI calls became less reliable.

The architecture was therefore abandoned.

This is an important engineering lesson:

> fewer functions does not automatically mean lower system overhead if the
> process and middleware architecture becomes more expensive.

---

## 12. Final Composed Navigation Architecture

The final solution used:

```text
rclcpp_components/component_container_isolated
```

Only the required navigation components were retained:

```text
controller_server
smoother_server
planner_server
behavior_server
velocity_smoother
collision_monitor
bt_navigator
```

The lifecycle manager handled these seven components.

The final bond timeout was:

```text
10.0 s
```

This value was verified at runtime rather than assumed from the YAML file.

---

## 13. Costmap Optimization

The local costmap originally used a VoxelLayer.

For the final 2D navigation task, the additional voxel-processing overhead was
not required.

The local costmap was simplified to:

```text
ObstacleLayer
+
InflationLayer
```

The global costmap remained:

```text
StaticLayer
+
ObstacleLayer
+
InflationLayer
```

The final configuration also used:

```text
always_send_full_costmap: false
```

to reduce unnecessary costmap communication and processing.

---

## 14. Visualization Load

Visualization was useful during debugging, but it also contributed
significantly to system load.

During controller tuning, Foxglove was restricted to only the required views.

Typical minimum visualization set:

```text
/map
TF
/plan
```

High-bandwidth displays such as:

```text
/scan
MPPI trajectory visualization
multiple plots
```

were disabled unless specifically required for diagnosis.

MPPI visualization remained:

```text
visualize: false
```

in the final operating baseline.

---

## 15. Final Motion Limits

The final navigation configuration used relatively conservative real-vehicle
limits.

```text
vx_max:  0.30 m/s
vx_min: -0.20 m/s
vy_max:  0.20 m/s
wz_max:  0.70 rad/s
```

These values prioritized:

- indoor safety;
- repeatability;
- localization stability;
- predictable controller behavior.

The objective was not maximum chassis speed.

---

## 16. Robot Footprint

The physical chassis was approximately:

```text
0.70 m × 0.30 m
```

The Nav2 footprint used the corresponding rectangular approximation:

```text
x: ±0.35 m
y: ±0.15 m
```

Using the actual chassis scale was important for collision checking and path
execution.

---

## 17. Final Laser Transform

The final RPLIDAR installation transform was:

```text
base_link -> laser

x     = +0.27 m
y     =  0.00 m
z     = +0.42 m

roll  = 0
pitch = 0
yaw   = 0
```

Earlier temporary navigation tests used a simplified transform.

The final values correspond to the actual mechanical installation.

---

## 18. Final Navigation Baseline

The final documented Pi 4 navigation baseline was:

```text
Controller frequency:
10 Hz

MPPI:
time_steps = 30
model_dt   = 0.10
batch_size = 1000
visualize  = false

Progress checker:
movement_radius         = 0.2 m
movement_time_allowance = 15 s

Controller tolerance:
costmap_update_timeout = 0.50 s
failure_tolerance      = 1.00 s

MPPI:
transform_tolerance = 0.30 s

Lifecycle:
bond_timeout = 10.0 s

Local costmap:
ObstacleLayer + InflationLayer

Global costmap:
StaticLayer + ObstacleLayer + InflationLayer

always_send_full_costmap = false
```

---

## 19. Real-Vehicle Validation

The final system completed real-vehicle tests including:

- `NavigateToPose`;
- MPPI Omni trajectory execution;
- target-position approach;
- final goal-orientation adjustment;
- automatic stop;
- gamepad high-priority takeover;
- cold-start recovery using the documented startup procedure.

The final control chain therefore represented a real closed loop:

```text
Goal
 ↓
Planner
 ↓
MPPI
 ↓
Velocity Smoother
 ↓
Collision Monitor
 ↓
twist_mux
 ↓
ROS 2 Serial Bridge
 ↓
STM32
 ↓
Physical Robot
```

---

## 20. Preserved Configuration Warning

The repository contains historical configuration snapshots under:

```text
config/
config/tuning_history/
```

However, the exact final YAML file used during the September 10–11 final
vehicle validation was not preserved.

Therefore:

- preserved YAML files are treated as historical engineering snapshots;
- documented final values come from the original development records;
- historical files are not falsely labeled as the exact final production
  configuration.

A future reconstructed configuration should be clearly named, for example:

```text
nav2_params_mecanum_reconstructed.yaml
```

and should not be presented as the original vehicle-tested file until it has
been revalidated.

---

## 21. Engineering Lessons

### Tune the system, not only the controller

A controller failure may actually originate from:

```text
CPU scheduling
TF timing
DDS overhead
visualization
lifecycle management
costmap processing
```

rather than controller gains alone.

### Debug by layer

When:

```text
planner succeeds
controller fails
```

do not immediately modify the STM32 or lower-level motor controller.

Find the first failing layer and remain there until evidence points elsewhere.

### Runtime verification matters

A parameter written into YAML does not necessarily mean it is the value being
used at runtime.

Important values such as:

```text
bond_timeout
```

were verified using live ROS 2 parameter queries.

### Stable beats theoretically faster

Reducing the controller from 20 Hz to 10 Hz was beneficial because the actual
hardware could execute the lower-rate loop consistently.

A stable 10 Hz loop was more useful than a nominal 20 Hz loop that repeatedly
missed deadlines.

### Preserve known-good baselines

Each major tuning stage was backed up before the next change.

This made it possible to distinguish:

```text
motion-model changes
Pi 4 performance changes
TF tolerance changes
lifecycle experiments
```

rather than accumulating unexplained parameter modifications.
