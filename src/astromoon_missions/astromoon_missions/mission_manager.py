#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
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

        # --- state / guards ---
        self._goal_in_progress = False
        self._active_goal_handle = None
        self._loop_timer = None
        self._ready_timer = None

        # --- logging state (only for "wait until ready") ---
        self._logged_once = set()
        self._last_wp_idx = None

        self._publish_event("MISSION_STARTED")
        self._publish_event(f"FOLLOW_WAYPOINTS_READY n={len(self.poses)} loop={self.loop}")

        self._wait_nav2_ready()

    def _log_once(self, key: str, level: str, msg: str):
        if key in self._logged_once:
            return
        self._logged_once.add(key)
        getattr(self.get_logger(), level)(msg)

    def _wait_nav2_ready(self):
        self._log_once("wait_nav2", "info", "Waiting for /waypoint_follower to become ACTIVE...")
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
            self._log_once("nav2_ready", "info", "Nav2 waypoint_follower is ACTIVE. Sending FollowWaypoints goal.")
            self._publish_event("NAV2_READY")
            if self._ready_timer is not None:
                self._ready_timer.cancel()
                self._ready_timer = None

            # Start looping timer once (if configured)
            if self.loop and self.loop_delay_s > 0.0 and self._loop_timer is None:
                self.get_logger().info(f"Loop enabled: trying to start every {self.loop_delay_s}s when idle")
                self._loop_timer = self.create_timer(self.loop_delay_s, self._loop_once)

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
        ps.header.stamp = self.get_clock().now().to_msg()
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
        # Guard: don't spam / preempt ourselves
        if self._goal_in_progress:
            return

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("FollowWaypoints action server not available (/follow_waypoints)")
            self._publish_event("FOLLOW_WAYPOINTS_SERVER_MISSING")
            return

        goal = FollowWaypoints.Goal()
        goal.poses = self.poses

        self._publish_event("FOLLOW_WAYPOINTS_GOAL_SENT")
        self.get_logger().info(f"Sending FollowWaypoints with {len(self.poses)} poses")

        send_future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

        # Mark in progress once we've initiated the send (prevents concurrent sends)
        self._goal_in_progress = True

    def _on_feedback(self, feedback_msg):
        try:
            idx = feedback_msg.feedback.current_waypoint
        except Exception:
            return

        if idx != self._last_wp_idx:
            self._last_wp_idx = idx
            self.get_logger().info(f"Following waypoints: current_waypoint={idx}")
            self._publish_event(f"FOLLOW_WAYPOINTS_FEEDBACK current_waypoint={idx}")

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self._goal_in_progress = False
            self.get_logger().error(f"FollowWaypoints goal response failed: {e}")
            self._publish_event("FOLLOW_WAYPOINTS_GOAL_RESPONSE_FAILED")
            return

        if not goal_handle.accepted:
            self._goal_in_progress = False
            self._active_goal_handle = None
            self.get_logger().error("FollowWaypoints goal rejected")
            self._publish_event("FOLLOW_WAYPOINTS_REJECTED")
            return

        self._active_goal_handle = goal_handle
        self._publish_event("FOLLOW_WAYPOINTS_ACCEPTED")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"FollowWaypoints result failed: {e}")
            self._publish_event("FOLLOW_WAYPOINTS_RESULT_FAILED")
            self._goal_in_progress = False
            self._active_goal_handle = None
            return

        status = res.status
        self.get_logger().info(f"FollowWaypoints finished status={status}")
        self._publish_event(f"FOLLOW_WAYPOINTS_DONE status={status}")

        # Mark idle so timer (or immediate loop) can relaunch
        self._goal_in_progress = False
        self._active_goal_handle = None
        self._last_wp_idx = None

        # If loop_delay_s == 0, immediately relaunch on completion (if looping enabled)
        if self.loop and self.loop_delay_s <= 0.0:
            self.get_logger().info("Loop enabled with delay=0.0: restarting immediately")
            self._send_goal()

        if not self.loop:
            self.get_logger().info("Loop disabled: manager idle")

    def _loop_once(self):
        # Periodic timer: only send if idle
        if not self.loop:
            return
        if self._goal_in_progress:
            return
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
