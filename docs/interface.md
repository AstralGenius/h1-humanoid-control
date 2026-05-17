# H1 Humanoid Control — Interface Contract

**Version:** 1.1 (Stage 2 complete)

Status tags throughout:

- 🟢 **implemented** — live in the running system
- 🟡 **partial** — implemented with known gaps (noted inline)
- ⚪ **planned** — defined for a later stage, not yet wired

---

## System Architecture

Two independent processes communicating only via ROS 2 topics:

- **Isaac Sim process** (Python 3.11) — runs physics, the H1 walking policy, and an OmniGraph that handles all ROS 2 pub/sub.
- **ROS 2 controller process** (Python 3.12) — runs teleop, waypoint follower, and logging nodes using system `rclpy`.

No shared memory. No file-based IPC. All communication is ROS 2 over DDS on localhost.

---

## Physics Parameters 🟢

| Parameter             | Value             | Source                                      |
| --------------------- | ----------------- | ------------------------------------------- |
| Physics step (dt)     | 0.005 s (200 Hz)  | `bridge_config.PHYSICS_DT`                  |
| Rendering step        | 0.04 s (25 Hz)    | `bridge_config.RENDERING_DT`                |
| Policy inference rate | 200 Hz            | runs every physics step in `on_physics_step` |

---

## Topics

### `/h1/cmd_vel` 🟢

- **Direction:** ROS 2 controller → Isaac Sim bridge
- **Type:** `geometry_msgs/Twist`
- **QoS:** default reliable, depth 10
- **Source inside Isaac Sim:** OmniGraph `ROS2SubscribeTwist` node

Fields used:

- `linear.x` — forward velocity (m/s), clamped to `[0.0, MAX_LIN_X]`
- `angular.z` — yaw rate (rad/s), clamped to `[-MAX_ANG_Z, MAX_ANG_Z]`

Fields ignored:

- `linear.y` — H1 flat-terrain policy does not support lateral motion
- `linear.z`, `angular.x`, `angular.y` — not physically meaningful

### `/h1/odom` 🟢

- **Direction:** Isaac Sim bridge → ROS 2 controller
- **Type:** `nav_msgs/Odometry`
- **Publisher rate:** ~30 Hz (tied to OmniGraph render tick)
- **QoS:** default reliable, depth 10
- **Source inside Isaac Sim:** OmniGraph `IsaacComputeOdometry` → `ROS2PublishOdometry`

Fields published: position, orientation, linear velocity, and angular velocity of the chassis prim (`/World/H1`).

Frame IDs:

- `header.frame_id` = `odom`
- `child_frame_id` = `base_link`

Quaternion ordering is handled internally by the OmniGraph publish node — no manual `[w,x,y,z]` → `[x,y,z,w]` conversion is needed at the application layer.

### `/clock` 🟢

- **Type:** `rosgraph_msgs/Clock`
- **Purpose:** simulated time for downstream nodes that need synchronised stamps

---

## Coordinate Frames (REP-105)

- `world` — Isaac Sim global frame, Z up, X forward
- `odom` — identical to `world` in this project (no drift model)
- `base_link` — robot chassis prim, Z up, X forward

---

## Units

| Quantity         | Unit               |
| ---------------- | ------------------ |
| Position         | metres             |
| Linear velocity  | metres per second  |
| Angle            | radians            |
| Angular velocity | radians per second |
| Time             | seconds (ROS Time) |

---

## Policy Constraints (Isaac Sim H1 flat-terrain policy)

| Constraint               | Value      | Notes                                      |
| ------------------------ | ---------- | ------------------------------------------ |
| Max forward velocity     | 0.75 m/s   | exceeding causes fall                      |
| Min forward velocity     | 0.0 m/s    | reverse not supported                      |
| Max yaw rate magnitude   | 0.75 rad/s |                                            |
| Lateral velocity support | none       | `base_command[1]` ignored by policy        |
| Initial spawn height     | 1.05 m     | `bridge_config.ROBOT_SPAWN_HEIGHT`         |

The bridge clamps incoming `/h1/cmd_vel` to these limits. Out-of-range values are silently clipped without logging.

---

## Failure Modes & Watchdogs ⚪

Planned for Stage 5 hardening — not yet implemented.

| Condition                | Detection                  | Response                                  |
| ------------------------ | -------------------------- | ----------------------------------------- |
| No cmd_vel for > 0.5 s   | bridge timestamp check     | zero command                              |
| No odom for > 1.0 s      | controller timestamp check | controller stops, logs error              |
| Robot falls (Z < 0.5 m)  | bridge polls pose          | zero command, log error                   |
| Sim process dies         | controller missing odom    | launchfile restart policy                 |

---

## Waypoint Interface (Stage 4) ⚪

### `/h1/waypoints`

- **Direction:** operator → waypoint controller
- **Type:** `geometry_msgs/PoseArray`
- **QoS:** transient_local (latched), depth 1
- **Frame:** `odom`

### `/h1/waypoint_status`

- **Direction:** waypoint controller → operator
- **Type:** `std_msgs/String`
- **Values:** `IDLE` | `ROTATING` | `WALKING` | `REACHED` | `COMPLETE` | `ERROR`

---

## Versioning

| Version | Change                                                                                                                                                                                |
| ------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1.0     | Initial spec, written before implementation                                                                                                                                            |
| 1.1     | Updated to reflect OmniGraph bridge in Isaac Sim 5.1; marked watchdogs and fall detection as planned-not-implemented; removed stale claims about Python-side quaternion conversion     |

Any change to topic names, message types, units, or limits requires bumping the version and updating both the bridge and all controllers in lockstep.
