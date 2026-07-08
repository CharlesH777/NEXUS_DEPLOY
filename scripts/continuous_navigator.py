#!/usr/bin/env python3
"""
Continuous navigator: eliminates stop-and-go between exploration goals.

Directly orchestrates planner + controller (bypasses BT navigator
goal handling) so the robot never stops between waypoints:

  1. /goal_pose -> plan path (compute_path_to_pose action)
  2. send path to controller (follow_path action)
  3. WHILE robot is driving, pre-plan the NEXT path
  4. when current path finishes, immediately fire the pre-planned path
  5. zero gap — robot never stops

Fully async — no spin_until_future_complete inside callbacks.
"""
import math
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose, FollowPath
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


def make_pose(frame, x, y, yaw=0.0):
    m = PoseStamped()
    m.header.frame_id = frame
    m.header.stamp = rclpy.clock.Clock().now().to_msg()
    m.pose.position.x = float(x)
    m.pose.position.y = float(y)
    m.pose.orientation.z = math.sin(0.5 * yaw)
    m.pose.orientation.w = math.cos(0.5 * yaw)
    return m


class ContinuousNavigator(Node):
    def __init__(self):
        super().__init__("continuous_navigator")

        self.declare_parameter("global_frame", "world")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("preplan_distance", 3.0)
        self.declare_parameter("switch_distance", 1.5)
        self.declare_parameter("coast_velocity", 0.3)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.preplan_dist = float(self.get_parameter("preplan_distance").value)
        self.coast_vel = float(self.get_parameter("coast_velocity").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.switch_dist = float(self.get_parameter("switch_distance").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.planner_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.controller_client = ActionClient(self, FollowPath, "follow_path")
        self.cmd_vel_pub = None  # no coasting — unsafe near obstacles

        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.on_goal, 20)

        self.lock = threading.Lock()
        self.pending_goals = []
        self.current_goal = None        # (x, y)
        self.current_handle = None      # follow_path goal handle
        self.preplanned_path = None     # Path ready to fire
        self.preplanning = False
        self.planning_current = False
        # True between calling _send_path and the matching _on_drive_response
        # callback. Prevents tick() Branch 1 from re-firing and stealing the
        # next goal during the async gap where current_handle is still None.
        self.pending_drive = False
        self.robot_xy = None
        self.last_path = None           # cached path for instant re-send

        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(
            "continuous_navigator ready (preplan=%.1fm switch=%.1fm)"
            % (self.preplan_dist, self.switch_dist))

    # ── robot pose ──────────────────────────────────────────────────
    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
        except TransformException:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    # ── goal input ──────────────────────────────────────────────────
    def on_goal(self, msg: PoseStamped):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        with self.lock:
            if self.current_goal:
                if abs(gx - self.current_goal[0]) < 0.5 and abs(gy - self.current_goal[1]) < 0.5:
                    return
            if self.pending_goals:
                last = self.pending_goals[-1]
                if abs(gx - last[0]) < 0.5 and abs(gy - last[1]) < 0.5:
                    return
            self.pending_goals.append((gx, gy))
        self.get_logger().info(
            "Goal queued: (%.1f, %.1f)" % (gx, gy))

    # ── main loop ───────────────────────────────────────────────────
    def tick(self):
        self.robot_xy = self.get_robot_pose()
        if self.robot_xy is None:
            return

        # No active goal → start one. If no goals available, do nothing
        # (robot slows down near last goal via MPPI — safe, costmap-aware).
        if self.current_handle is None and not self.planning_current and not self.pending_drive:
            with self.lock:
                if self.pending_goals:
                    self.current_goal = self.pending_goals.pop(0)
                else:
                    return  # wait for explorer — no coasting (unsafe)
            self.planning_current = True
            gx, gy = self.current_goal
            self.get_logger().info("Planning path to (%.1f, %.1f)" % (gx, gy))
            self._plan_and_drive(gx, gy)
            return

        # Active goal → pre-plan NEXT goal immediately (not waiting until
        # close). This is "提前规划": the next path is ready before the
        # robot reaches the current goal.
        if (self.current_handle is not None
                and not self.preplanning
                and self.preplanned_path is None):
            with self.lock:
                if not self.pending_goals:
                    return
                next_goal = self.pending_goals[0]
            # Start pre-planning immediately — don't wait for proximity
            self.preplanning = True
            gx, gy = next_goal
            self.get_logger().info(
                "Pre-planning next (%.1f, %.1f)" % (gx, gy))
            self._plan_path_async(gx, gy, self._on_preplan_done)

        # PROACTIVE SWITCH: if pre-planned path is ready and robot is close
        # enough to current goal, cancel current FollowPath and immediately
        # fire the next one. Robot never stops.
        if (self.current_handle is not None
                and self.preplanned_path is not None
                and self.current_goal is not None):
            dist = math.hypot(
                self.robot_xy[0] - self.current_goal[0],
                self.robot_xy[1] - self.current_goal[1])
            if dist < self.switch_dist:
                self.get_logger().info(
                    "Proactive switch [dist=%.1fm]" % dist)
                # Cancel current FollowPath (will trigger _on_drive_result
                # but we ignore it since we're already switching)
                old_handle = self.current_handle
                self.current_handle = None  # prevent double-switch
                with self.lock:
                    self.current_goal = self.pending_goals.pop(0) if self.pending_goals else None
                    path = self.preplanned_path
                    self.preplanned_path = None
                if old_handle is not None:
                    cancel_future = old_handle.cancel_goal_async()
                    cancel_future.add_done_callback(lambda _: None)
                if path is not None and self.current_goal is not None:
                    self._send_path(path)

    # ── async path planning ─────────────────────────────────────────
    def _plan_path_async(self, gx, gy, callback):
        if not self.planner_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("Planner not ready")
            callback(None)
            return

        goal = ComputePathToPose.Goal()
        goal.goal = make_pose(self.global_frame, gx, gy)
        if self.robot_xy:
            goal.start = make_pose(self.global_frame, self.robot_xy[0], self.robot_xy[1])
        goal.use_start = False

        future = self.planner_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_plan_goal_response(f, callback))

    def _on_plan_goal_response(self, future, callback):
        handle = future.result()
        if handle is None or not handle.accepted:
            callback(None)
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_plan_result(f, callback))

    def _on_plan_result(self, future, callback):
        result = future.result()
        if result is None:
            callback(None)
            return
        callback(result.result.path)

    # ── pre-plan callback ───────────────────────────────────────────
    def _on_preplan_done(self, path):
        self.preplanning = False
        if path is not None:
            with self.lock:
                self.preplanned_path = path
            self.get_logger().info("Pre-plan ready")
        else:
            self.get_logger().warn("Pre-plan failed")

    # ── plan + drive (current goal) ─────────────────────────────────
    def _plan_and_drive(self, gx, gy):
        # If we have a pre-planned path, use it
        with self.lock:
            path = self.preplanned_path
            self.preplanned_path = None

        if path is not None:
            self.planning_current = False
            self.get_logger().info("Using pre-planned path")
            self._send_path(path)
            return

        # Otherwise plan now
        self._plan_path_async(gx, gy, self._on_current_plan_done)

    def _on_current_plan_done(self, path):
        self.planning_current = False
        if path is None:
            self.get_logger().warn("Planning failed — dropping goal")
            with self.lock:
                self.current_goal = None
            return
        self._send_path(path)

    # ── send path to controller ─────────────────────────────────────
    def _send_path(self, path: Path):
        if not self.controller_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("Controller not ready")
            self.pending_drive = False
            with self.lock:
                self.current_goal = None
            return

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = ""
        goal.goal_checker_id = ""

        self.last_path = path  # cache for instant re-send
        # Guard the async gap: _on_drive_response will clear this once the
        # server accepts/rejects the goal.
        self.pending_drive = True

        future = self.controller_client.send_goal_async(
            goal, feedback_callback=lambda _fb: None)
        future.add_done_callback(self._on_drive_response)

    def _on_drive_response(self, future):
        self.pending_drive = False
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn("FollowPath rejected")
            with self.lock:
                self.current_handle = None
                self.current_goal = None
            return
        self.current_handle = handle
        self.get_logger().info("Following path")
        result_future = handle.get_result_async()
        # Capture the handle so _on_drive_result can verify the result belongs
        # to the currently active goal and not a cancelled/switched-away one.
        result_future.add_done_callback(
            lambda f, h=handle: self._on_drive_result(f, h))

    def _on_drive_result(self, future, handle):
        """FollowPath completed or was cancelled.

        Only clear state if this result belongs to the currently active
        handle. A proactive switch cancels the old FollowPath; its result
        callback must NOT clobber the newly-switched goal's state.
        """
        if handle is not self.current_handle:
            return  # stale result from a cancelled / switched-away goal
        self.current_handle = None
        self.current_goal = None

    # ── (coast removed — unsafe near obstacles) ─────────────────────
def main():
    rclpy.init()
    node = ContinuousNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
