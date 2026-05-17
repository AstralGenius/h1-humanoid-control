# SPDX-License-Identifier: Apache-2.0
"""Keyboard teleoperation for the H1 humanoid.

Push-to-go semantics: the robot moves while keys are held, stops when released.

Key bindings:
    W / Up arrow    forward
    A / Left arrow  turn left (positive yaw)
    D / Right arrow turn right (negative yaw)
    S / Space       explicit stop
    Q / Esc         quit

Publishes at 20 Hz on /h1/cmd_vel (geometry_msgs/Twist).
All values clamped to the interface contract limits before publishing.
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard


# Defaults — could move to a parameter file later
DEFAULT_LIN_SPEED = 0.4    # m/s, below the 0.75 policy max
DEFAULT_ANG_SPEED = 0.5    # rad/s
MAX_LIN_SPEED = 0.75
MAX_ANG_SPEED = 0.75
PUBLISH_RATE_HZ = 20.0


class H1Teleop(Node):
    """ROS 2 node that publishes Twist commands from keyboard input."""

    def __init__(self) -> None:
        super().__init__("h1_teleop")

        # Declare parameters so they can be overridden via launch / CLI
        self.declare_parameter("linear_speed", DEFAULT_LIN_SPEED)
        self.declare_parameter("angular_speed", DEFAULT_ANG_SPEED)
        self.declare_parameter("topic", "/h1/cmd_vel")

        self._lin_speed = float(self.get_parameter("linear_speed").value)
        self._ang_speed = float(self.get_parameter("angular_speed").value)
        topic = self.get_parameter("topic").value

        # Clamp configured speeds to policy limits — defence in depth
        if self._lin_speed > MAX_LIN_SPEED:
            self.get_logger().warn(
                f"linear_speed {self._lin_speed} > {MAX_LIN_SPEED}, clamping"
            )
            self._lin_speed = MAX_LIN_SPEED
        if self._ang_speed > MAX_ANG_SPEED:
            self.get_logger().warn(
                f"angular_speed {self._ang_speed} > {MAX_ANG_SPEED}, clamping"
            )
            self._ang_speed = MAX_ANG_SPEED

        # Currently pressed keys — protected by lock since pynput uses its own thread
        self._pressed = set()
        self._lock = threading.Lock()
        self._should_quit = False

        self._pub = self.create_publisher(Twist, topic, 10)
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

        self._print_help()

        # Start the keyboard listener in a background thread
        self._listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self._listener.start()

    # -- ROS callbacks --------------------------------------------------

    def _tick(self) -> None:
        """Periodic publish: build a Twist from currently-held keys."""
        with self._lock:
            keys = set(self._pressed)

        msg = Twist()

        # Forward — only one direction supported by policy
        if "forward" in keys:
            msg.linear.x = self._lin_speed

        # Yaw — left positive, right negative; cancel if both held
        ang = 0.0
        if "left" in keys:
            ang += self._ang_speed
        if "right" in keys:
            ang -= self._ang_speed
        msg.angular.z = ang

        # Explicit stop overrides everything
        if "stop" in keys:
            msg = Twist()  # all zeros

        self._pub.publish(msg)

    # -- Keyboard callbacks ---------------------------------------------

    def _on_press(self, key) -> None:
        action = self._key_to_action(key)
        if action == "quit":
            self._should_quit = True
            return False  # tells pynput listener to stop
        if action is not None:
            with self._lock:
                self._pressed.add(action)

    def _on_release(self, key) -> None:
        action = self._key_to_action(key)
        if action is not None and action != "quit":
            with self._lock:
                self._pressed.discard(action)

    @staticmethod
    def _key_to_action(key) -> str | None:
        """Map a pynput key to one of: forward, left, right, stop, quit."""
        # Letter keys
        try:
            char = key.char.lower() if key.char else None
        except AttributeError:
            char = None

        if char == "w":
            return "forward"
        if char == "a":
            return "left"
        if char == "d":
            return "right"
        if char == "s":
            return "stop"
        if char == "q":
            return "quit"

        # Special keys
        if key == keyboard.Key.up:
            return "forward"
        if key == keyboard.Key.left:
            return "left"
        if key == keyboard.Key.right:
            return "right"
        if key == keyboard.Key.space:
            return "stop"
        if key == keyboard.Key.esc:
            return "quit"

        return None

    # -- Misc -----------------------------------------------------------

    def _print_help(self) -> None:
        self.get_logger().info("H1 teleop ready. Keys:")
        self.get_logger().info("  W / Up        forward")
        self.get_logger().info("  A / Left      turn left")
        self.get_logger().info("  D / Right     turn right")
        self.get_logger().info("  S / Space     stop")
        self.get_logger().info("  Q / Esc       quit")
        self.get_logger().info(
            f"Speeds: lin={self._lin_speed:.2f} m/s, ang={self._ang_speed:.2f} rad/s"
        )

    def should_quit(self) -> bool:
        return self._should_quit


def main(args=None) -> None:
    rclpy.init(args=args)
    node = H1Teleop()
    try:
        while rclpy.ok() and not node.should_quit():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish one final stop so the robot doesn't keep walking
        node._pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()