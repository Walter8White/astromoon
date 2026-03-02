#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class MapToOdomBroadcaster(Node):
    """
    Publishes map -> odom using Gazebo ground-truth + /odom.

    Assumptions (fits your situation):
    - /odom pose is expressed in frame 'rover/odom' and child is 'rover/base_link'
    - /gz/pose_tf provides a transform with child_frame_id == 'rover' that corresponds to the rover pose in WORLD.
      (header.frame_id is empty in your bridged TFMessage, so we force parent to `world_frame` parameter.)
    - world->map is an identity static transform (so map frame is aligned with Gazebo world).
    """

    def __init__(self):
        super().__init__("map_to_odom_broadcaster")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("pose_tf_topic", "/gz/pose_tf")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "rover/odom")
        self.declare_parameter("base_frame", "rover/base_link")
        self.declare_parameter("world_frame", "world")

        # Which child_frame_id in /gz/pose_tf corresponds to the rover model pose
        self.declare_parameter("gz_child_name", "rover")

        self.odom_topic = self.get_parameter("odom_topic").value
        self.pose_tf_topic = self.get_parameter("pose_tf_topic").value

        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.world_frame = self.get_parameter("world_frame").value
        self.gz_child_name = self.get_parameter("gz_child_name").value

        qos_be = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.br = TransformBroadcaster(self)

        # Latest odom pose (odom->base) in 2D
        self.last_odom = None  # (x, y, yaw)
        self.last_odom_stamp = None


        # Latest world pose (world->base) in 2D (from Gazebo)
        self.last_world = None  # (x, y, yaw)

        self.create_subscription(Odometry, self.odom_topic, self.cb_odom, qos_be)
        self.create_subscription(TFMessage, self.pose_tf_topic, self.cb_gz_tf, qos_be)

        self.get_logger().info(
            f"Publishing TF {self.map_frame} -> {self.odom_frame} using {self.pose_tf_topic} (child='{self.gz_child_name}') and {self.odom_topic}"
        )

    def cb_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.last_odom = (x, y, yaw)
        self.try_publish()
        self.last_odom = (x, y, yaw)
        self.last_odom_stamp = msg.header.stamp


    def cb_gz_tf(self, msg: TFMessage):
        # Find the transform for the rover model
        for tr in msg.transforms:
            if tr.child_frame_id == self.gz_child_name:
                x = tr.transform.translation.x
                y = tr.transform.translation.y
                yaw = yaw_from_quat(tr.transform.rotation)
                self.last_world = (x, y, yaw)
                self.try_publish()
                return

    def try_publish(self):
        if self.last_odom_stamp is None:
            return

        if self.last_odom is None or self.last_world is None:
            return

        ox, oy, oyaw = self.last_odom
        wx, wy, wyaw = self.last_world

        # We want: T_map_odom = T_map_base * inv(T_odom_base)
        # Here map == world (world->map identity), and we use Gazebo as world->base truth:
        # T_world_odom = T_world_base * inv(T_odom_base)
        dyaw = wrap_pi(wyaw - oyaw)
        cos_t = math.cos(dyaw)
        sin_t = math.sin(dyaw)

        # translation: w_p_o = w_p_b - R(dyaw) * o_p_b
        dx = wx - (cos_t * ox - sin_t * oy)
        dy = wy - (sin_t * ox + cos_t * oy)

        t = TransformStamped()
        # Use current ROS time (Gazebo TFMessage stamp was 0 in your output)
        t.header.stamp = self.last_odom_stamp

        t.header.frame_id = self.map_frame
        t.child_frame_id = "rover/odom"

        t.transform.translation.x = float(dx)
        t.transform.translation.y = float(dy)
        t.transform.translation.z = 0.0

        qx, qy, qz, qw = quat_from_yaw(dyaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = MapToOdomBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
