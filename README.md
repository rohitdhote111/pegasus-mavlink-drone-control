# Pegasus MAVLink Drone Control

A simple keyboard-based controller for operating a drone in the [Isaac Sim Pegasus Simulator](https://nvlabs.github.io/pegasus) using MAVLink commands.

## Overview

This project enables you to control a simulated drone in Isaac Sim via keyboard inputs. It communicates with the drone through the PX4 autopilot using MAVLink and Offboard mode.

## Features

- Arm/disarm the drone
- Toggle Offboard mode
- Takeoff and land
- Control drone motion in all directions (3D)
- Keyboard-based real-time interaction

## Keyboard Controls

| Key | Action             |
|-----|--------------------|
| `Space` | Arm / Disarm the drone |
| `o` | Toggle Offboard mode |
| `t` | Takeoff             |
| `l` | Land                |
| `w` | Move forward        |
| `s` | Move backward       |
| `a` | Move left           |
| `d` | Move right          |
| `q` | Yaw left (rotate CCW) |
| `e` | Yaw right (rotate CW) |
| `r` | Move up             |
| `f` | Move down           |
| `x` | Quit the controller |

## Requirements

- [Isaac Sim](https://developer.nvidia.com/isaac-sim)
- Pegasus Simulator
- MAVSDK-Python
- PX4 Autopilot setup

## Getting Started

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/pegasus-mavlink-drone-control.git
   cd pegasus-mavlink-drone-control
