# SPDX-License-Identifier: Apache-2.0
"""Path logger for the H1 humanoid.

Subscribes to /h1/odom and /h1/waypoint_status, writes one CSV row per odom
message:

    wall_time, sim_time, x, y, yaw, status

Output path is configurable; defaults to /tmp/h1_path_<timestamp>.csv so a
default run never overwrites a previous one.

Usage:
    ros2 run h1_controller path_logger
    ros2 run h1_controller path_logger --ros-args -p output:=/tmp/run1.csv
"""

import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class PathLogger(Node):
    """Logs robot pose and navigation state to CSV."""

    def __init__(self) -> None:
        super().__init__("path_logger")

        default_path = f"/tmp/h1_path_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.declare_parameter("output", default_path)
        self._output_path = str(self.get_parameter("output").value)

        # Latest status — joined to every odom row
        self._status = "UNKNOWN"

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, "/h1/odom", self._on_odom, qos)
        self.create_subscription(String, "/h1/waypoint_status", self._on_status, qos)

        # Open file and write header
        self._file = open(self._output_path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(["wall_time", "sim_time", "x", "y", "yaw", "status"])
        self._file.flush()

        self._row_count = 0
        self.get_logger().info(f"Logging odom to {self._output_path}")

    def _on_status(self, msg: String) -> None:
        self._status = msg.data

    def _on_odom(self, msg: Odometry) -> None:
        wall = time.time()
        stamp = msg.header.stamp
        sim = stamp.sec + stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self._writer.writerow([f"{wall:.6f}", f"{sim:.6f}", f"{x:.4f}", f"{y:.4f}",
                               f"{yaw:.4f}", self._status])
        self._row_count += 1

        # Flush periodically so a crash doesn't lose recent data
        if self._row_count % 50 == 0:
            self._file.flush()

    def close(self) -> None:
        if self._file and not self._file.closed:
            self._file.flush()
            self._file.close()
            self.get_logger().info(
                f"Wrote {self._row_count} rows to {self._output_path}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()