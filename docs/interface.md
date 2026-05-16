# H1 Humanoid Control — Interface Contract

## System Architecture

Two independent processes communicating only via ROS 2 topics:

- **Isaac Sim process** — runs physics, H1 flat-terrain policy inference, rendering
- **ROS 2 process** — runs controller nodes (teleop, waypoint follower)

No shared memory. No file-based IPC. All communication is ROS 2 over localhost.

---

## Physics Parameters

| Parameter | Value | Source |
|---|---|---|
| Physics step (dt) | 0.005 s (200 Hz) | h1_standalone.py line 55: `physics_dt=1/200` |
| Rendering step | 0.04 s (25 Hz) | h1_standalone.py line 55: `rendering_dt=8/200` |
| Policy inference rate | 200 Hz | runs every physics step via `on_physics_step` |

---

## Topics

### /h1/cmd_vel

- **Direction:** ROS 2 controller → Isaac Sim bridge
- **Type:** `geometry_msgs/Twist`
- **Publisher rate:** 50 Hz (controller decides)
- **QoS:** default reliable, depth 10
- **Fields used:**
  - `linear.x` — forward velocity (m/s) → maps to `base_command[0]`
  - `angular.z` — yaw rate (rad/s) → maps to `base_command[2]`
- **Fields ignored:**
  - `linear.y` — H1 flat-terrain policy does not support lateral motion
  - `linear.z`, `angular.x`, `angular.y` — not physically meaningful here

### /h1/odom

- **Direction:** Isaac Sim bridge → ROS 2 controller
- **Type:** `nav_msgs/Odometry`
- **Publisher rate:** 200 Hz (published from `on_physics_step`)
- **QoS:** default reliable, depth 10
- **Fields published:**
  - `pose.pose.position` — from `h1.robot.get_world_pose()[0]`
  - `pose.pose.orientation` — from `h1.robot.get_world_pose()[1]`, **converted from Isaac `[w,x,y,z]` to ROS `[x,y,z,w]`**
  - `twist.twist.linear` — from `h1.robot.get_linear_velocity()`
  - `twist.twist.angular` — from `h1.robot.get_angular_velocity()`
- **Frame IDs:**
  - `header.frame_id` = `"odom"`
  - `child_frame_id` = `"base_link"`

---

## Coordinate Frames (REP-105 convention)

- `world` — Isaac Sim global frame, Z up, X forward
- `odom` — identical to world in this project (no drift model)
- `base_link` — robot torso centre, Z up, X forward

Quaternion convention: Isaac Sim returns `[w, x, y, z]`. ROS expects `[x, y, z, w]`. The bridge must reorder before publishing.

---

## Units

| Quantity | Unit |
|---|---|
| Position | metres |
| Linear velocity | m/s |
| Angle | radians |
| Angular velocity | rad/s |
| Time | seconds (ROS Time) |

---

## Policy Constraints (Isaac Sim H1 flat-terrain policy)

| Constraint | Value | Source |
|---|---|---|
| Max forward velocity | 0.75 m/s | Isaac Sim docs, exceeding causes fall |
| Min forward velocity | 0.0 m/s | reverse not supported |
| Max yaw rate magnitude | 0.75 rad/s | Isaac Sim docs |
| Lateral velocity support | none | `base_command[1]` ignored by policy |
| Initial spawn height | 1.05 m | h1_standalone.py line 70 |

The bridge clamps incoming `/h1/cmd_vel` to these limits. Out-of-range values are silently clamped (not rejected) and logged at warn level.

---

## Failure Modes & Watchdogs

| Condition | Detection | Response |
|---|---|---|
| No cmd_vel for >0.5 s | bridge timestamp check | bridge zeroes `base_command` |
| No odom for >1.0 s | controller timestamp check | controller publishes zero cmd_vel, logs error |
| Robot falls (Z < 0.5 m) | bridge polls pose | bridge zeroes command, logs error |
| Sim process dies | controller missing odom | systemd/launchfile restart policy |

---

## Waypoint Interface (Stage 4)

### /h1/waypoints
- **Direction:** operator → waypoint controller
- **Type:** `geometry_msgs/PoseArray`
- **QoS:** transient_local (latched), depth 1
- **Frame:** `odom`

### /h1/waypoint_status
- **Direction:** waypoint controller → operator
- **Type:** `std_msgs/String`
- **Values:** `IDLE` | `ROTATING` | `WALKING` | `REACHED` | `COMPLETE` | `ERROR`

---

## Versioning

This contract is **v1.0**. Any change to topic names, message types, units, or limits requires bumping the version and updating both the bridge and all controllers in lockstep.