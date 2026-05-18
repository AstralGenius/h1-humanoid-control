# SPDX-License-Identifier: Apache-2.0
"""Closed-loop waypoint navigation for the H1 humanoid.

Reads /h1/odom, publishes /h1/cmd_vel, drives the robot through a list of
(x, y) goals using a heading-first control law:

  ROTATING -> face the goal (in place)
  WALKING  -> walk toward it with gentle heading correction
  REACHED  -> within tolerance, advance to next waypoint

The algorithm assumes a forward-only kinematic (matches the H1 flat-terrain
policy: no reverse, no lateral motion). See docs/interface.md for limits.

Goals come from a YAML config file specified by --waypoints-file, or fall
back to a built-in square path for quick testing.
"""

import math
import os
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import yaml


# Policy limits — match docs/interface.md
MAX_LIN_SPEED = 0.5      # m/s — below 0.75 ceiling for safety margin
MAX_ANG_SPEED = 0.6      # rad/s — below 0.75 ceiling for safety margin

# Default goal tolerances
DEFAULT_POSITION_TOLERANCE = 0.25     # m
DEFAULT_HEADING_TOLERANCE = 0.15      # rad (~8.6 deg)
DEFAULT_REENTER_ROTATE_THRESHOLD = 0.35  # rad — drift this much, re-rotate

# Default control gains
DEFAULT_KP_LIN = 0.6
DEFAULT_KP_ANG = 1.2
DEFAULT_WALK_ANG_GAIN = 0.3   # multiplier on heading error while walking

# Loop rate
CONTROL_RATE_HZ = 20.0

# Default fallback path: a 2m square
DEFAULT_WAYPOINTS = [
    (2.0, 0.0),
    (2.0, 2.0),
    (0.0, 2.0),
    (0.0, 0.0),
]


class NavState(Enum):
    IDLE = auto()
    ROTATING = auto()
    WALKING = auto()
    REACHED = auto()
    COMPLETE = auto()


def wrap_to_pi(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw (rotation about Z) from a quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointController(Node):
    """Drives the H1 through a sequence of (x, y) goals."""

    def __init__(self) -> None:
        super().__init__("waypoint_controller")

        # Parameters
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("position_tolerance", DEFAULT_POSITION_TOLERANCE)
        self.declare_parameter("heading_tolerance", DEFAULT_HEADING_TOLERANCE)
        self.declare_parameter("kp_lin", DEFAULT_KP_LIN)
        self.declare_parameter("kp_ang", DEFAULT_KP_ANG)
        self.declare_parameter("loop_forever", False)

        self._pos_tol = float(self.get_parameter("position_tolerance").value)
        self._head_tol = float(self.get_parameter("heading_tolerance").value)
        self._kp_lin = float(self.get_parameter("kp_lin").value)
        self._kp_ang = float(self.get_parameter("kp_ang").value)
        self._loop = bool(self.get_parameter("loop_forever").value)

        waypoints_file = self.get_parameter("waypoints_file").value
        self._waypoints = self._load_waypoints(waypoints_file)
        self._goal_index = 0

        # State
        self._state = NavState.IDLE
        self._have_odom = False
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # Pub / sub
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._cmd_pub = self.create_publisher(Twist, "/h1/cmd_vel", qos)
        self._status_pub = self.create_publisher(String, "/h1/waypoint_status", qos)
        self.create_subscription(Odometry, "/h1/odom", self._on_odom, qos)

        # Control loop
        self._timer = self.create_timer(1.0 / CONTROL_RATE_HZ, self._tick)

        self.get_logger().info(
            f"Waypoint controller ready. {len(self._waypoints)} goals loaded. "
            f"pos_tol={self._pos_tol:.2f}m head_tol={math.degrees(self._head_tol):.1f}deg"
        )
        self._publish_status("IDLE")

    # -- I/O ------------------------------------------------------------

    def _load_waypoints(self, path: str) -> list:
        """Load waypoints from YAML, or fall back to the default square."""
        if not path:
            self.get_logger().info("No waypoints file given; using default square path")
            return list(DEFAULT_WAYPOINTS)
        if not os.path.exists(path):
            self.get_logger().error(f"Waypoints file not found: {path}; using default")
            return list(DEFAULT_WAYPOINTS)
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f)
            pts = [(float(p["x"]), float(p["y"])) for p in data["waypoints"]]
            self.get_logger().info(f"Loaded {len(pts)} waypoints from {path}")
            return pts
        except Exception as exc:
            self.get_logger().error(f"Failed to parse {path}: {exc}; using default")
            return list(DEFAULT_WAYPOINTS)

    def _on_odom(self, msg: Odometry) -> None:
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        if not self._have_odom:
            self._have_odom = True
            self._state = NavState.ROTATING
            self.get_logger().info("First odom received; starting navigation")

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def _publish_cmd(self, lin: float, ang: float) -> None:
        lin = max(0.0, min(MAX_LIN_SPEED, lin))
        ang = max(-MAX_ANG_SPEED, min(MAX_ANG_SPEED, ang))
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self._cmd_pub.publish(msg)

    def _stop(self) -> None:
        self._publish_cmd(0.0, 0.0)

    # -- Control loop ---------------------------------------------------

    def _tick(self) -> None:
        if not self._have_odom:
            return

        if self._state == NavState.COMPLETE:
            self._stop()
            return

        if self._goal_index >= len(self._waypoints):
            if self._loop:
                self._goal_index = 0
                self._state = NavState.ROTATING
                self.get_logger().info("Lap complete, restarting")
            else:
                self._state = NavState.COMPLETE
                self._publish_status("COMPLETE")
                self.get_logger().info("All waypoints reached. Stopping.")
                self._stop()
            return

        gx, gy = self._waypoints[self._goal_index]
        dx, dy = gx - self._x, gy - self._y
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_error = wrap_to_pi(target_heading - self._yaw)

        # Goal reached?
        if distance < self._pos_tol:
            self.get_logger().info(
                f"Waypoint {self._goal_index} reached "
                f"(x={self._x:.2f}, y={self._y:.2f})"
            )
            self._goal_index += 1
            self._state = NavState.REACHED
            self._publish_status("REACHED")
            self._stop()
            return

        # State transitions
        if self._state == NavState.REACHED:
            self._state = NavState.ROTATING

        if self._state == NavState.ROTATING:
            self._publish_status("ROTATING")
            if abs(heading_error) < self._head_tol:
                self._state = NavState.WALKING
            else:
                self._publish_cmd(0.0, self._kp_ang * heading_error)
                return

        if self._state == NavState.WALKING:
            self._publish_status("WALKING")
            if abs(heading_error) > DEFAULT_REENTER_ROTATE_THRESHOLD:
                self._state = NavState.ROTATING
                self._stop()
                return

            lin = min(self._kp_lin * distance, MAX_LIN_SPEED)
            ang = self._kp_ang * heading_error * DEFAULT_WALK_ANG_GAIN
            self._publish_cmd(lin, ang)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node._cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()