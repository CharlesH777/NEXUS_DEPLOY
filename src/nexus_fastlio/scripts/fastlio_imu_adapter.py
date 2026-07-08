#!/usr/bin/env python3
import copy
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu


class FastlioImuAdapter(Node):
    def __init__(self):
        super().__init__("fastlio_imu_adapter")

        self.declare_parameter("input_topic", "/livox/imu")
        self.declare_parameter("output_topic", "/imu_fastlio")
        self.declare_parameter("linear_accel_scale", 1.0)
        self.declare_parameter("rotation_pitch_deg", 0.0)
        self.declare_parameter("target_frame_id", "base_link")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.linear_accel_scale = float(self.get_parameter("linear_accel_scale").value)
        self.rotation_pitch_deg = float(self.get_parameter("rotation_pitch_deg").value)
        self.target_frame_id = str(self.get_parameter("target_frame_id").value)

        theta = math.radians(self.rotation_pitch_deg)
        self.cos_t = math.cos(theta)
        self.sin_t = math.sin(theta)

        self.sub = self.create_subscription(Imu, self.input_topic, self.on_imu, 50)
        self.pub = self.create_publisher(Imu, self.output_topic, 50)

        self.get_logger().info(
            f"FAST-LIO IMU adapter: {self.input_topic} -> {self.output_topic}, "
            f"linear_accel_scale={self.linear_accel_scale}, "
            f"rotation_pitch_deg={self.rotation_pitch_deg}, target_frame_id={self.target_frame_id}"
        )

    def on_imu(self, msg: Imu):
        out = copy.deepcopy(msg)
        if self.target_frame_id:
            out.header.frame_id = self.target_frame_id

        ax = out.linear_acceleration.x
        ay = out.linear_acceleration.y
        az = out.linear_acceleration.z
        out.linear_acceleration.x = (self.cos_t * ax + self.sin_t * az) * self.linear_accel_scale
        out.linear_acceleration.y = ay * self.linear_accel_scale
        out.linear_acceleration.z = (-self.sin_t * ax + self.cos_t * az) * self.linear_accel_scale

        gx = out.angular_velocity.x
        gy = out.angular_velocity.y
        gz = out.angular_velocity.z
        out.angular_velocity.x = self.cos_t * gx + self.sin_t * gz
        out.angular_velocity.y = gy
        out.angular_velocity.z = -self.sin_t * gx + self.cos_t * gz
        self.pub.publish(out)


def main():
    rclpy.init()
    node = FastlioImuAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
