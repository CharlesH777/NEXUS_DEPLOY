#!/usr/bin/env python3
import copy
import math

import rclpy
from rclpy.node import Node

from livox_ros_driver2.msg import CustomMsg


class FastlioLidarAdapter(Node):
    def __init__(self):
        super().__init__("fastlio_lidar_adapter")

        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/lidar_fastlio")
        self.declare_parameter("rotation_pitch_deg", 0.0)
        self.declare_parameter("target_frame_id", "base_link")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.rotation_pitch_deg = float(self.get_parameter("rotation_pitch_deg").value)
        self.target_frame_id = str(self.get_parameter("target_frame_id").value)

        theta = math.radians(self.rotation_pitch_deg)
        self.cos_t = math.cos(theta)
        self.sin_t = math.sin(theta)

        self.sub = self.create_subscription(CustomMsg, self.input_topic, self.on_lidar, 20)
        self.pub = self.create_publisher(CustomMsg, self.output_topic, 20)

        self.get_logger().info(
            f"FAST-LIO lidar adapter: {self.input_topic} -> {self.output_topic}, "
            f"rotation_pitch_deg={self.rotation_pitch_deg}, target_frame_id={self.target_frame_id}"
        )

    def on_lidar(self, msg: CustomMsg):
        out = copy.deepcopy(msg)
        if self.target_frame_id:
            out.header.frame_id = self.target_frame_id

        for pt in out.points:
            x = pt.x
            y = pt.y
            z = pt.z
            pt.x = self.cos_t * x + self.sin_t * z
            pt.y = y
            pt.z = -self.sin_t * x + self.cos_t * z

        self.pub.publish(out)


def main():
    rclpy.init()
    node = FastlioLidarAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
