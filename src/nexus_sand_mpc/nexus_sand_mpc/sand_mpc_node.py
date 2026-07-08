from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node

from .sand_mpc_controller import SandMpcControllerMIMO, SandMpcObservation


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class SandMpcCompensatorNode(Node):
    """ROS wrapper for the sand-slip MIMO MPC command compensator."""

    def __init__(self) -> None:
        super().__init__("sand_mpc_compensator")
        self.declare_parameters(
            "",
            [
                ("input_cmd_topic", "/mppi/cmd_vel_raw"),
                ("output_cmd_topic", "/cmd_vel"),
                ("odom_topic", "/odom"),
                ("control_rate", 20.0),
                ("command_timeout_sec", 0.30),
                ("odom_timeout_sec", 0.6),
                ("translational_deadband", 0.02),
                ("preserve_lateral_direction", True),
                ("passthrough_on_missing_odom", True),
                ("publish_zero_on_timeout", True),
                ("horizon", 10),
                ("dt_nominal", 0.05),
                ("cmd_delay", 0.05),
                ("drive_tau", 0.08),
                ("turn_tau", 0.08),
                ("q_v", 120.0),
                ("q_int_v", 12.0),
                ("r_du_v", 0.30),
                ("q_w", 80.0),
                ("q_int_w", 6.0),
                ("r_du_w", 0.50),
                ("slip_alpha", 0.20),
                ("slip_init", 0.20),
                ("correction_gain", 0.30),
                ("v_max", 1.50),
                ("w_max", 1.40),
            ],
        )

        self.input_cmd_topic = str(self.get_parameter("input_cmd_topic").value)
        self.output_cmd_topic = str(self.get_parameter("output_cmd_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.command_timeout = float(self.get_parameter("command_timeout_sec").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout_sec").value)
        self.translational_deadband = float(self.get_parameter("translational_deadband").value)
        self.preserve_lateral_direction = bool(self.get_parameter("preserve_lateral_direction").value)
        self.passthrough_on_missing_odom = bool(self.get_parameter("passthrough_on_missing_odom").value)
        self.publish_zero_on_timeout = bool(self.get_parameter("publish_zero_on_timeout").value)

        try:
            self.controller = SandMpcControllerMIMO(
                horizon=int(self.get_parameter("horizon").value),
                dt_nominal=float(self.get_parameter("dt_nominal").value),
                cmd_delay=float(self.get_parameter("cmd_delay").value),
                drive_tau=float(self.get_parameter("drive_tau").value),
                turn_tau=float(self.get_parameter("turn_tau").value),
                q_v=float(self.get_parameter("q_v").value),
                q_int_v=float(self.get_parameter("q_int_v").value),
                r_du_v=float(self.get_parameter("r_du_v").value),
                q_w=float(self.get_parameter("q_w").value),
                q_int_w=float(self.get_parameter("q_int_w").value),
                r_du_w=float(self.get_parameter("r_du_w").value),
                slip_alpha=float(self.get_parameter("slip_alpha").value),
                slip_init=float(self.get_parameter("slip_init").value),
                correction_gain=float(self.get_parameter("correction_gain").value),
                v_max=float(self.get_parameter("v_max").value),
                w_max=float(self.get_parameter("w_max").value),
            )
        except RuntimeError as exc:
            self.get_logger().fatal(str(exc))
            raise

        self.time_origin: Optional[float] = None
        self.latest_cmd = Twist()
        self.latest_cmd_stamp: Optional[rclpy.time.Time] = None
        self.latest_odom_stamp: Optional[rclpy.time.Time] = None
        self.latest_direction = (1.0, 0.0)
        self.last_warn_missing_odom_sec = -1.0
        self.last_warn_compute_sec = -1.0

        self.cmd_sub = self.create_subscription(Twist, self.input_cmd_topic, self.on_cmd, 20)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)

        control_rate = max(float(self.get_parameter("control_rate").value), 1.0)
        self.timer = self.create_timer(1.0 / control_rate, self.on_timer)
        self.get_logger().info(
            "sand_mpc_compensator ready: %s -> %s, odom=%s, rate=%.1f Hz"
            % (self.input_cmd_topic, self.output_cmd_topic, self.odom_topic, control_rate)
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def relative_time(self, absolute_sec: float) -> float:
        if self.time_origin is None:
            self.time_origin = absolute_sec
        return max(0.0, absolute_sec - self.time_origin)

    def relative_now(self) -> float:
        return self.relative_time(self.now_sec())

    def message_time_or_now(self, stamp) -> float:
        t = stamp_to_sec(stamp)
        if t <= 0.0:
            t = self.now_sec()
        return self.relative_time(t)

    def on_cmd(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.latest_cmd_stamp = self.get_clock().now()

        vx = float(msg.linear.x)
        vy = float(msg.linear.y) if self.preserve_lateral_direction else 0.0
        v_ref = math.hypot(vx, vy)
        if v_ref > self.translational_deadband:
            self.latest_direction = (vx / v_ref, vy / v_ref)
        else:
            v_ref = 0.0
        self.controller.set_reference(v_ref, float(msg.angular.z))

    def on_odom(self, msg: Odometry) -> None:
        self.latest_odom_stamp = self.get_clock().now()
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y) if self.preserve_lateral_direction else 0.0
        v_actual = math.hypot(vx, vy)
        w_actual = float(msg.twist.twist.angular.z)
        self.controller.receive_observation(
            SandMpcObservation(
                timestamp=self.message_time_or_now(msg.header.stamp),
                v=v_actual,
                w=w_actual,
            )
        )

    def has_fresh_cmd(self) -> bool:
        if self.latest_cmd_stamp is None:
            return False
        return self.get_clock().now() - self.latest_cmd_stamp <= Duration(seconds=self.command_timeout)

    def has_fresh_odom(self) -> bool:
        if self.latest_odom_stamp is None:
            return False
        return self.get_clock().now() - self.latest_odom_stamp <= Duration(seconds=self.odom_timeout)

    def publish_stop(self) -> None:
        self.controller.set_reference(0.0, 0.0)
        self.cmd_pub.publish(Twist())

    def publish_passthrough(self) -> None:
        self.cmd_pub.publish(self.latest_cmd)

    def on_timer(self) -> None:
        if not self.has_fresh_cmd():
            if self.publish_zero_on_timeout:
                self.publish_stop()
            return

        if self.passthrough_on_missing_odom and not self.has_fresh_odom():
            now = self.now_sec()
            if now - self.last_warn_missing_odom_sec > 2.0:
                self.get_logger().warning(
                    "sand MPC is passing through raw cmd because odom is missing or stale."
                )
                self.last_warn_missing_odom_sec = now
            self.publish_passthrough()
            return

        t_now = self.relative_now()
        try:
            v_cmd, w_cmd = self.controller.compute(t_now)
        except Exception as exc:
            # Solver failure (infeasible QP, IPOPT non-convergence, NaN in
            # state) must never leave the /cmd_vel hardware boundary silent.
            # Publish a safe stop and let the next tick retry.
            now = self.now_sec()
            if now - self.last_warn_compute_sec > 2.0:
                self.get_logger().error(
                    "sand MPC compute failed (%s); publishing stop." % exc
                )
                self.last_warn_compute_sec = now
            self.publish_stop()
            return
        raw_v_ref = math.hypot(
            float(self.latest_cmd.linear.x),
            float(self.latest_cmd.linear.y) if self.preserve_lateral_direction else 0.0,
        )

        out = Twist()
        if raw_v_ref > self.translational_deadband:
            out.linear.x = self.latest_direction[0] * v_cmd
            out.linear.y = self.latest_direction[1] * v_cmd if self.preserve_lateral_direction else 0.0
        out.angular.z = w_cmd
        self.cmd_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SandMpcCompensatorNode()
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
