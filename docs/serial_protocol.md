# STM32 / ROS 2 Serial Protocol

This document describes the serial communication interface between the
Raspberry Pi ROS 2 onboard computer and the STM32F407 chassis controller.

The protocol is implemented in:

```text
firmware/stm32/Core/Src/ros_serial_proto.c
firmware/stm32/Core/Inc/ros_serial_proto.h
```

---

## 1. Overview

The Raspberry Pi sends high-level chassis commands to the STM32 through UART.

The STM32 performs:

- command parsing
- command limiting
- mecanum chassis control
- communication timeout handling
- robot-state feedback

The communication format is a lightweight ASCII line protocol.

```text
Raspberry Pi / ROS 2
        |
        | ASCII command
        | UART
        v
+----------------------+
|   ros_serial_proto   |
+----------+-----------+
           |
           v
+----------------------+
|      robot_app       |
+----------+-----------+
           |
           v
+----------------------+
|     chassis_ctrl     |
+----------------------+
```

Each command is terminated by:

```text
\n
```

or:

```text
\r
```

---

## 2. Velocity Command

Format:

```text
V <vx> <vy> <wz>
```

Example:

```text
V 20.0 0.0 10.0
```

Units:

| Field | Meaning | Unit |
|---|---|---|
| `vx` | longitudinal body velocity | cm/s |
| `vy` | lateral body velocity | cm/s |
| `wz` | yaw angular velocity | deg/s |

Coordinate convention:

```text
+vx = forward
+vy = left
+wz  = counter-clockwise
```

The STM32 performs an additional safety clamp before passing the command to
the chassis controller.

Current firmware limits are:

```text
vx: -300 ... +300 cm/s
vy: -300 ... +300 cm/s
wz: -360 ... +360 deg/s
```

The command is eventually passed to:

```text
RobotApp_SetBodyVelocity(vx, vy, wz)
```

---

## 3. Stop Command

Format:

```text
STOP
```

The command immediately calls:

```text
RobotApp_Stop()
```

Lowercase command variants are also accepted.

---

## 4. Mechanism Commands

The firmware also supports mechanism-control commands.

### Link A

```text
MECH A
```

### Link B

```text
MECH B
```

### Stop mechanism

```text
MECH STOP
```

These commands are handled through the application layer rather than directly
controlling hardware from the communication module.

---

## 5. Odometry Reset

Format:

```text
RESET_ODOM
```

The firmware calls:

```text
RobotApp_ResetOdometry()
```

This resets the chassis odometry state.

---

## 6. Link Check

The host can verify communication using:

```text
PING
```

The STM32 replies:

```text
PONG
```

This provides a simple communication-health check without issuing a motion
command.

---

## 7. Robot State Feedback

The STM32 periodically transmits a robot-state message.

Format:

```text
S <launched> <x> <y> <yaw> <vx> <vy> <wz> <wheel0> <wheel1> <wheel2> <wheel3>
```

Example structure:

```text
S 1 54.75 0.25 26.65 0.00 0.00 0.00 0.00 0.00 0.00 0.00
```

Fields:

| Field | Meaning | Unit |
|---|---|---|
| `launched` | chassis/application enabled state | boolean/integer |
| `x` | local odometry X | cm |
| `y` | local odometry Y | cm |
| `yaw` | robot yaw | deg |
| `vx` | estimated longitudinal velocity | cm/s |
| `vy` | estimated lateral velocity | cm/s |
| `wz` | estimated yaw rate | deg/s |
| `wheel0` | wheel 0 velocity | cm/s |
| `wheel1` | wheel 1 velocity | cm/s |
| `wheel2` | wheel 2 velocity | cm/s |
| `wheel3` | wheel 3 velocity | cm/s |

The current firmware publishes this status every:

```text
100 ms
```

corresponding to approximately:

```text
10 Hz
```

---

## 8. Communication Timeout

A motion-control interface must not continue executing an old velocity command
if the ROS 2 computer or serial link fails.

The firmware therefore tracks the time of the most recent valid command.

Current timeout:

```text
300 ms
```

If no valid command is received for longer than this period:

```text
RobotApp_Stop()
```

is executed and the serial link is marked offline.

Conceptually:

```text
Valid command
     |
     v
update last_cmd_ms
     |
     v
command age <= 300 ms ?
     |
   yes
     |
continue
     
command age > 300 ms
     |
     v
RobotApp_Stop()
     |
     v
serial link offline
```

This creates a simple dead-man behavior between the Raspberry Pi and STM32.

---

## 9. Receive Architecture

UART reception uses interrupt-driven single-byte reception.

```text
UART RX interrupt
      |
      v
receive one byte
      |
      +---- newline / carriage return?
      |              |
      |             yes
      |              |
      |              v
      |        mark line ready
      |
      +---- otherwise
                     |
                     v
               append to buffer
```

The RX line buffer size is:

```text
96 bytes
```

When a complete line is available, parsing is performed outside the receive
callback by `RosSerial_Service()`.

This keeps the UART interrupt callback relatively small and separates byte
collection from application-level command processing.

---

## 10. Buffer Overflow Handling

If the receive buffer reaches its maximum length before a line terminator is
received, the current receive line is discarded and an overflow flag is set.

The buffer is then restarted for the next input.

This prevents an invalid or excessively long serial message from continuing to
overwrite the receive buffer.

---

## 11. UART Error Recovery

The firmware includes a UART error callback.

On an RX error it:

1. aborts the current interrupt receive operation;
2. clears supported UART error flags;
3. resets the current receive length;
4. restarts interrupt reception.

This allows the communication layer to recover from transient UART receive
errors without requiring a complete MCU restart.

---

## 12. ROS 2 Integration

The original Raspberry Pi application used a ROS 2 serial bridge.

The command path was:

```text
Nav2 / teleoperation
        |
        v
/cmd_vel_robot
        |
        v
base_serial_bridge
        |
        | unit conversion
        | ASCII protocol
        v
UART
        |
        v
STM32
```

The feedback path was:

```text
STM32
  |
  | S status message
  v
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
/odom + odom -> base_link
```

The original Raspberry Pi application-layer source code was not fully
preserved, so this repository documents the validated interface while the
STM32 implementation remains available as source.

---

## 13. Design Notes

The protocol intentionally uses a simple human-readable ASCII format.

Advantages during development included:

- easy inspection with a serial terminal;
- easy command injection during bring-up;
- simple debugging of ROS 2 / STM32 integration;
- no dependency on a generated message library;
- straightforward fault isolation.

Trade-offs include:

- higher bandwidth than a binary protocol;
- text parsing overhead;
- no packet-level CRC in the current protocol;
- no explicit sequence number.

For the bandwidth and control rates used by this robot, the ASCII protocol was
sufficient for the validated platform.

A future production-oriented revision could introduce:

- framed binary packets;
- CRC;
- sequence numbers;
- protocol versioning;
- explicit acknowledgement for safety-critical commands.
