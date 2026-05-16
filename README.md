# H1 Humanoid Control

A ROS 2 wrapper around NVIDIA Isaac Sim's pre-trained Unitree H1 flat-terrain walking policy.
Demonstrates a clean two-process architecture (simulator ↔ ROS 2 controller) that decouples
locomotion from high-level control, transferable from wheeled robots (Jetbot) to humanoids.

## Architecture

Two independent processes communicating only via ROS 2 topics:

- **Isaac Sim bridge** (`isaac_sim/h1_ros_bridge.py`) — wraps NVIDIA's H1 policy,
  subscribes to `/h1/cmd_vel`, publishes `/h1/odom`
- **ROS 2 controllers** (`ros2_ws/src/h1_controller/`) — teleop and closed-loop
  waypoint navigation, both publish `/h1/cmd_vel`

See [`docs/interface.md`](docs/interface.md) for the topic contract and
[`docs/architecture.md`](docs/architecture.md) for the system diagram.

## Status

- [x] Stage 0 — Baseline H1 policy verified in Isaac Sim
- [x] Stage 1 — Interface contract defined
- [x] Stage 2 — Isaac Sim ↔ ROS 2 bridge
- [ ] Stage 3 — Teleop controller
- [ ] Stage 4 — Closed-loop waypoint controller
- [ ] Stage 5 — Validation and documentation
- [ ] Stage 6 — Stretch: gesture control

## Requirements

- Ubuntu 24.04
- NVIDIA Isaac Sim 5.0
- ROS 2 Jazzy
- NVIDIA driver 580+ (open kernel modules supported)

## Quick Start

See [`docs/setup.md`](docs/setup.md).

## License

Apache-2.0
