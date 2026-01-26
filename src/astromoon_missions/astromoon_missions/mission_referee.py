#!/usr/bin/env python3

import os
import math
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class MissionReferee(Node):
    def __init__(self):
        super().__init__("mission_referee")

        self.declare_parameter("mission_file", "")
        mission_file = self.get_parameter("mission_file").value

        if not mission_file or not os.path.exists(mission_file):
            raise RuntimeError("mission_file invalid")

        with open(mission_file, "r") as f:
            self.mission = yaml.safe_load(f)

        self.time_limit = self.mission.get("time_limit_s", 300)

        wp_cfg = self.mission["waypoints"]
        self.tol = wp_cfg.get("tolerance_xy_m", 0.3)
        self.waypoints = [p["xy_yaw"] for p in wp_cfg["points"]]

        self.current_wp = 0
        self.start_time = self.get_clock().now()

        self.result_pub = self.create_publisher(String, "/mission/result", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        self.get_logger().info(
            f"[Referee] Monitoring {len(self.waypoints)} waypoints"
        )

        self.timer = self.create_timer(0.5, self._check_timeout)

    def _on_odom(self, msg: Odometry):
        if self.current_wp >= len(self.waypoints):
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        wx, wy, _ = self.waypoints[self.current_wp]
        dist = math.hypot(x - wx, y - wy)

        if dist < self.tol:
            self.get_logger().info(
                f"[Referee] Waypoint {self.current_wp+1} reached"
            )
            self.current_wp += 1

            if self.current_wp == len(self.waypoints):
                self._finish("SUCCESS")

    def _check_timeout(self):
        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1e9

        if elapsed > self.time_limit:
            self.get_logger().warn("[Referee] Timeout")
            self._finish("FAIL")

    def _finish(self, result: str):
        msg = String()
        msg.data = result
        self.result_pub.publish(msg)
        self.get_logger().info(f"[Referee] Mission result: {result}")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = MissionReferee()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
