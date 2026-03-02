#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool


class CmdVelMux(Node):
    def __init__(self):
        super().__init__("cmd_vel_mux")

        self.declare_parameter("default_mode", "auto")

        default_mode = str(self.get_parameter("default_mode").value).strip().lower()
        self._mode = "manual" if default_mode == "manual" else "auto"

        self._out_pub = self.create_publisher(Twist, "/cmd_vel_muxed", 10)
        self._mode_pub = self.create_publisher(String, "/cmd_vel_mux/mode", 10)

        self.create_subscription(Twist, "/cmd_vel", self._on_auto_cmd, 10)
        self.create_subscription(Twist, "/cmd_vel_manual", self._on_manual_cmd, 10)

        self.create_service(SetBool, "/cmd_vel_mux/set_manual_mode", self._on_set_manual_mode)

        self._publish_mode()
        self.get_logger().info(
            "cmd_vel mux ready: use /cmd_vel in AUTO, /cmd_vel_manual in MANUAL"
        )

    def _publish_mode(self):
        msg = String()
        msg.data = self._mode
        self._mode_pub.publish(msg)

    def _publish_stop(self):
        self._out_pub.publish(Twist())

    def _set_mode(self, mode: str):
        if mode == self._mode:
            return
        self._mode = mode
        self._publish_stop()
        self._publish_mode()
        self.get_logger().info(f"Switched cmd_vel mode to {self._mode}")

    def _on_set_manual_mode(self, request: SetBool.Request, response: SetBool.Response):
        self._set_mode("manual" if request.data else "auto")
        response.success = True
        response.message = f"cmd_vel mode: {self._mode}"
        return response

    def _on_auto_cmd(self, msg: Twist):
        if self._mode == "auto":
            self._out_pub.publish(msg)

    def _on_manual_cmd(self, msg: Twist):
        if self._mode == "manual":
            self._out_pub.publish(msg)


def main():
    rclpy.init()
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
