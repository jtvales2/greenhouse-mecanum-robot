# Odometry and TF Engineering

This document describes the odometry and TF architecture used by the
greenhouse mecanum robot, together with the real-vehicle yaw-drift issue that
was diagnosed and corrected during development.

---

## 1. TF Architecture

The final localization chain used:

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

Responsibilities were separated as follows:

- `map -> odom`
  - published by AMCL;
  - provides global localization correction.

- `odom -> base_link`
  - published by the ROS 2 `base_odometry` node;
  - represents locally integrated robot motion.

- `base_link -> laser`
  - static transform;
  - represents the physical installation of the RPLIDAR.

This separation made it possible to diagnose whether a localization problem
originated from:

```text
global localization
or
local odometry
```

instead of treating the whole TF tree as one black box.

---

## 2. Odometry Inputs

The local odometry layer used:

- four wheel encoders;
- mecanum chassis motion estimation;
- WIT IMU yaw information.

The STM32 provided chassis-state information to the Raspberry Pi through the
serial protocol.

The ROS 2 `base_odometry` node then published:

```text
/odom
```

and:

```text
odom -> base_link
```

The chassis body-frame velocities were rotated using the current yaw before
being integrated into the odom frame.

Conceptually:

```text
Wheel Encoders
      |
      v
STM32 chassis state
      |
      +------> vx
      +------> vy
      +------> yaw / wz
      |
      v
ROS 2 base_odometry
      |
      v
/odom
      |
      v
odom -> base_link
```

---

## 3. Original Yaw Drift Problem

During long-duration testing, the robot exposed a significant yaw-drift issue.

After approximately:

```text
1 hour 12 minutes
```

of continuous odometry operation, the reported:

```text
odom -> base_link
```

yaw had drifted to approximately:

```text
-93° to -94°
```

even though the actual robot heading had not changed by that amount.

This caused:

```text
map -> base_link
```

to become visibly inconsistent with the real vehicle heading.

---

## 4. Initial Gyro Deadband

The original gyro zero deadband was:

```text
0.05 deg/s
```

During standstill, the WIT gyro still produced residual values such as:

```text
-0.22
-0.16
-0.10
+0.14
+0.21
-0.28 deg/s
```

These values were small, but continuous integration caused yaw error to
accumulate over time.

Conceptually:

```text
small gyro bias
      |
      v
continuous integration
      |
      v
slow yaw drift
      |
      v
large long-term heading error
```

The problem therefore did not require a large instantaneous sensor error.

A small persistent bias was sufficient.

---

## 5. First Adjustment

The deadband was increased from:

```text
0.05 deg/s
```

to:

```text
0.35 deg/s
```

This significantly improved standstill behavior.

However, after actual forward motion and rotation, the system still exhibited
approximately:

```text
0.03 ~ 0.05 deg/s
```

of one-direction yaw drift.

This showed that the first adjustment reduced the problem but did not fully
remove the residual gyro-bias effect.

---

## 6. TF-Based Fault Isolation

A critical part of the debugging process was separating the TF chain.

Instead of assuming AMCL was responsible for the heading error, the two major
transforms were inspected independently.

Observed behavior:

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

This provided an important diagnostic result:

```text
map -> odom stable
        +
odom -> base_link drifting
        =
local odometry / IMU problem
```

Therefore the fault was not attributed to AMCL.

This prevented unnecessary changes to:

- AMCL parameters;
- map configuration;
- laser transform;
- RPLIDAR driver settings.

---

## 7. Final Practical Deadband

The final practical value used during this project stage was:

```text
0.50 deg/s
```

Firmware parameter:

```text
ROBOT_GYRO_ZERO_DEADBAND_DPS = 0.50f
```

This value was selected from real-vehicle behavior rather than chosen
arbitrarily.

---

## 8. Real-Vehicle Validation

After applying the final deadband value, the robot was tested with:

```text
forward motion ≈ 0.55 m
+
rotation ≈ 26.65°
+
stop
```

The STM32 state settled to approximately:

```text
x    ≈ 54.75 cm
y    ≈ 0.25 cm
yaw  = 26.65°

vx = 0
vy = 0
wz = 0
```

The ROS 2 odometry settled to approximately:

```text
x    ≈ 0.542 m
y    ≈ 0.006 m
yaw  = 26.650°
```

The yaw remained stable for tens of seconds without the previously visible
one-direction drift.

The STM32 displacement and ROS 2 odometry were also in the same expected
magnitude.

---

## 9. Why This Was Not Treated as an AMCL Problem

The important evidence was:

```text
map -> odom
```

remained stable while:

```text
odom -> base_link
```

continued drifting.

Because AMCL publishes:

```text
map -> odom
```

and the local odometry layer publishes:

```text
odom -> base_link
```

the source of the drift could be isolated to the local odometry / IMU side.

This is a useful debugging rule:

> inspect each TF edge independently before changing the global localization
> stack.

---

## 10. Final Laser Transform

The final physical RPLIDAR installation was represented by:

```text
base_link -> laser

x     = +0.27 m
y     =  0.00 m
z     = +0.42 m

roll  = 0
pitch = 0
yaw   = 0
```

Earlier development used temporary values such as:

```text
x = 0
z = 0.20 m
```

for basic navigation verification.

Those temporary values were later replaced with the measured mechanical
installation values.

---

## 11. Relationship Between Local and Global Localization

The complete localization architecture was:

```text
                  AMCL
map ------------------------------> odom
                                      |
                                      |
                                      | local odometry
                                      v
                                  base_link
                                      |
                                      |
                                      | static TF
                                      v
                                    laser
```

The roles were intentionally different.

### Local odometry

Responsible for:

- short-term robot motion;
- wheel encoder integration;
- yaw evolution;
- continuous local motion state.

### AMCL

Responsible for:

- map-relative localization;
- correcting global pose relative to the static map.

This architecture allows local motion to remain continuous while AMCL provides
global localization.

---

## 12. Engineering Limitation

The final:

```text
0.50 deg/s
```

deadband was a practical engineering fix for this project stage.

It should not be interpreted as the ideal long-term IMU solution.

The development record explicitly notes that if long-duration thermal drift
appears again, the better solution would be:

```text
stationary detection
        +
online gyro-bias tracking
        +
stationary yaw freezing / bias correction
```

rather than continuously increasing the deadband.

This distinction is important.

A large deadband can suppress noise and bias, but it may also remove genuine
low-rate rotation information.

Therefore the final project value was treated as a validated practical
baseline, not a universal IMU-calibration strategy.

---

## 13. Debugging Method

The yaw-drift investigation followed a useful system-debugging sequence.

### Step 1 — Reproduce the symptom

Observe long-duration heading drift.

### Step 2 — Check raw / low-level state

Inspect:

```text
/base/state
```

especially:

```text
wz
yaw
```

### Step 3 — Split the TF chain

Compare:

```text
map -> odom
```

with:

```text
odom -> base_link
```

### Step 4 — Identify the first drifting layer

If:

```text
map -> odom stable
odom -> base_link drifting
```

then remain in the odometry / IMU layer.

### Step 5 — Change one parameter

Adjust the gyro zero deadband only.

### Step 6 — Perform motion + stop validation

Do not validate only while the robot is stationary from boot.

The sensor must also be tested after real motion and rotation.

### Step 7 — Freeze the validated baseline

Once the practical value passed the real-vehicle test, stop changing unrelated
parameters without new evidence.

---

## 14. Useful ROS 2 Diagnostics

The project used TF inspection commands such as:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

and:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

These allow the local and complete localization chains to be inspected
independently.

The main odometry and state interfaces included:

```text
/base/state
/odom
TF: odom -> base_link
TF: map -> odom
```

---

## 15. Final Engineering Lessons

### Small bias becomes large error after integration

An error of only a fraction of a degree per second can become a very large yaw
error when continuously integrated.

### Split transforms before tuning

A wrong final robot heading does not automatically mean AMCL is wrong.

Inspect:

```text
map -> odom
```

and:

```text
odom -> base_link
```

independently.

### Validate after motion

A sensor may appear stable after boot but exhibit residual bias after movement,
temperature change or vibration.

### Change one layer at a time

Once evidence identified the problem as local odometry / IMU drift, unrelated
Nav2, AMCL and laser settings were not changed.

### Practical fixes and fundamental fixes are different

The project used:

```text
gyro deadband = 0.50 deg/s
```

as a practical validated solution.

A more advanced future implementation should use online bias estimation and
stationary detection instead of relying only on a fixed deadband.
