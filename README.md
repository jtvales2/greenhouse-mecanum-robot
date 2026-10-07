# Greenhouse Mecanum Robot

A greenhouse mobile robot based on **STM32F407 + Raspberry Pi 4 + ROS 2 Jazzy + Nav2 MPPI Omni**.

This project was developed and validated on a real four-wheel mecanum robot for greenhouse navigation, manual takeover, environmental sensing and remote monitoring.

> **Repository Scope**
>
> This repository contains the open-source STM32 chassis firmware, Nav2 configuration, tuning history, engineering documentation and field-test materials.
>
> The original Raspberry Pi ROS 2 application-layer source code was not fully preserved. Its architecture, package responsibilities and validated behavior are documented from the original engineering records.

---

## Demo

### Real Robot Platform

<p align="center">
  <img src="media/f1ed5129057b6bf8a711b6330355f722.jpg" width="48%">
  <img src="media/ed0fc33912e08596cc2c8be37d58b40b.jpg" width="48%">
</p>

<p align="center">
  <em>Four-wheel mecanum greenhouse mobile robot used for real-world navigation and system integration tests.</em>
</p>

The robot was validated on a real four-wheel mecanum platform using ROS 2 Jazzy, Nav2, AMCL and the MPPI Omni controller.

---

## Hardware

- STM32F407ZGT6
- Raspberry Pi 4
- Four-wheel mecanum chassis
- RPLIDAR A1
- WIT IMU
- Wheel encoders
- IMX219 camera
- RS485 environmental sensor

---

## Software Stack

- Ubuntu 24.04
- ROS 2 Jazzy
- Nav2
- AMCL
- MPPI Controller
- Omni motion model
- velocity_smoother
- collision_monitor
- twist_mux

---

## System Architecture

```text
Goal Pose
   ↓
Nav2 Planner
   ↓
MPPI Omni Controller
   ↓
Velocity Smoother
   ↓
Collision Monitor
   ↓
twist_mux  ←  Gamepad Override
   ↓
ROS 2 Serial Bridge
   ↓
STM32 Chassis Controller
   ↓
Mecanum Wheel Velocity Control
