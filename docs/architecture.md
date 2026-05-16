# Architecture

## Overview

Two-process system communicating exclusively via ROS 2 topics over DDS:

- **Isaac Sim process** (Python 3.11) — runs the H1 walking policy, simulates
  physics, publishes/subscribes via OmniGraph
- **ROS 2 controller process** (Python 3.12) — runs teleop and waypoint nodes
  using system rclpy
┌─────────────────────────────────────────┐    ┌──────────────────────────┐
│  Isaac Sim (Python 3.11)                │    │  ROS 2 (Python 3.12)     │
│  ┌──────────────────────────────────┐   │    │  ┌────────────────────┐  │
│  │ OmniGraph                        │   │    │  │ teleop_node        │  │
│  │  OnPlaybackTick                  │   │    │  │ waypoint_controller│  │
│  │  ├─ SubscribeTwist  /h1/cmd_vel ─┼───┼────┼──┤ path_logger        │  │
│  │  ├─ ComputeOdometry              │   │    │  └────────────────────┘  │
│  │  └─ PublishOdometry /h1/odom ────┼───┼────┼─────►                    │
│  └──────────────┬───────────────────┘   │    └──────────────────────────┘
│                 │                       │                ↑
│  ┌──────────────▼───────────────────┐   │                │ DDS
│  │ Physics callback (200 Hz)        │   │                │
│  │  read OmniGraph attrs (cmd_vel)  │   │
│  │  → H1FlatTerrainPolicy.forward() │   │
│  └──────────────────────────────────┘   │
└─────────────────────────────────────────┘
## Why OmniGraph for the ROS interface

The natural design — `import rclpy` inside the Isaac Sim standalone script —
does not work reliably in Isaac Sim 5.1:

- Isaac Sim ships Python 3.11; system ROS 2 Jazzy is built for Python 3.12,
  so system `rclpy` cannot be imported into Isaac Sim's interpreter.
- Isaac Sim's bundled `rclpy` (under `exts/isaacsim.ros2.bridge/jazzy/rclpy/`)
  imports but crashes inside `Node.__init__` due to an internal assertion
  in `rcl_interfaces__msg__parameter_event__convert_from_py`.

The supported and stable pattern is to perform all ROS 2 publish/subscribe
inside OmniGraph nodes (which use Isaac Sim's internal C++ rclcpp directly)
and read/write OmniGraph attributes from the Python physics callback. This
keeps Python-side code independent of any ROS Python ABI.

## Why bridge-in-sim-process

The bridge logic lives in the same process as the simulation rather than as
a separate node. This matches how real humanoids deploy their locomotion
controllers — tightly coupled to the actuator interface, exposing a clean
ROS API for higher-level control. External controllers (teleop, waypoint
follower) live in their own process and talk to the bridge over DDS,
identical to talking to a real robot.

## Interface contract

See [interface.md](interface.md) for topic specifications, units, limits,
and watchdog behavior.
