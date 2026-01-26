#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.action._follow_waypoints import FollowWaypoints_Result
from lifecycle_msgs.srv import GetState



def yaw_to_quat(yaw: float):
    # z = sin(yaw/2), w = cos(yaw/2)
    import math
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")

        self.declare_parameter("mission_file", "")
        mission_file = self.get_parameter("mission_file").value
        if not mission_file or not os.path.exists(mission_file):
            raise RuntimeError(f"mission_file invalid: {mission_file}")

        with open(mission_file, "r") as f:
            self.mission = yaml.safe_load(f)

        self.event_pub = self.create_publisher(String, "/mission/events", 10)

        exec_cfg = self.mission.get("execution", {})
        self.loop = bool(exec_cfg.get("loop", False))
        self.loop_delay_s = float(exec_cfg.get("loop_delay_s", 0.0))

        wp_cfg = self.mission["waypoints"]
        self.frame_id = wp_cfg.get("frame_id", "odom")

        self.poses = self._build_poses()

        self.client = ActionClient(self, FollowWaypoints, "/follow_waypoints")

        self._publish_event("MISSION_STARTED")
        self._publish_event(f"FOLLOW_WAYPOINTS_READY n={len(self.poses)} loop={self.loop}")

        self._wait_nav2_ready()

    
    def _wait_nav2_ready(self):
        # Check /waypoint_follower lifecycle state every 0.5s
        self.get_logger().info("Waiting for /waypoint_follower to become ACTIVE...")
        self._ready_timer = self.create_timer(0.5, self._check_waypoint_follower_state)

    def _check_waypoint_follower_state(self):
        client = self.create_client(GetState, "/waypoint_follower/get_state")
        if not client.wait_for_service(timeout_sec=0.2):
            return

        req = GetState.Request()
        future = client.call_async(req)
        future.add_done_callback(self._on_waypoint_follower_state)

    def _on_waypoint_follower_state(self, future):
        try:
            resp = future.result()
        except Exception:
            return

        # ACTIVE == 3
        if resp.current_state.id == 3:
            self.get_logger().info("Nav2 waypoint_follower is ACTIVE. Sending FollowWaypoints goal.")
            self._publish_event("NAV2_READY")
            # stop timer
            self._ready_timer.cancel()
            self._send_goal()


    def _publish_event(self, text: str):
        msg = String()
        msg.data = text
        self.event_pub.publish(msg)

    def _build_poses(self):
        poses = []

        for p in self.mission["waypoints"]["points"]:
            x, y, yaw = p["xy_yaw"]
            poses.append(self._make_pose(x, y, yaw))

        home_cfg = self.mission.get("home", {})
        if bool(home_cfg.get("enabled", False)):
            hx, hy, hyaw = home_cfg.get("xy_yaw", [0.0, 0.0, 0.0])
            home_frame = home_cfg.get("frame_id", self.frame_id)
            poses.append(self._make_pose(hx, hy, hyaw, frame_id=home_frame))

        return poses

    def _make_pose(self, x, y, yaw, frame_id=None):
        ps = PoseStamped()
        ps.header.frame_id = frame_id if frame_id is not None else self.frame_id
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(float(yaw))
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps

    def _send_goal(self):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("FollowWaypoints action server not available (/follow_waypoints)")
            self._publish_event("FOLLOW_WAYPOINTS_SERVER_MISSING")
            return

        goal = FollowWaypoints.Goal()
        # stamp is optional; Nav2 mostly uses frame_id + tf
        goal.poses = self.poses

        self._publish_event("FOLLOW_WAYPOINTS_GOAL_SENT")
        self.get_logger().info(f"Sending FollowWaypoints with {len(self.poses)} poses")

        send_future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, feedback_msg):
        # feedback.current_waypoint is common
        try:
            idx = feedback_msg.feedback.current_waypoint
            self._publish_event(f"FOLLOW_WAYPOINTS_FEEDBACK current_waypoint={idx}")
        except Exception:
            pass

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("FollowWaypoints goal rejected")
            self._publish_event("FOLLOW_WAYPOINTS_REJECTED")
            return

        self._publish_event("FOLLOW_WAYPOINTS_ACCEPTED")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        res = future.result()
        status = res.status
        result: FollowWaypoints_Result = res.result

        self.get_logger().info(f"FollowWaypoints finished status={status}")
        self._publish_event(f"FOLLOW_WAYPOINTS_DONE status={status}")

        # If looping, send again
        if self.loop:
            if self.loop_delay_s > 0.0:
                self.get_logger().info(f"Looping after {self.loop_delay_s}s")
                self.create_timer(self.loop_delay_s, self._loop_once)
            else:
                self._send_goal()
        else:
            # One-shot: let referee decide success/fail; manager can just idle
            self.get_logger().info("Loop disabled: manager idle")

    def _loop_once(self):
        # timer callback; send goal then cancel timer by doing nothing else
        self._send_goal()


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
