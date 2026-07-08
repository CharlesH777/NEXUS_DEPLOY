#!/usr/bin/env python3
"""Bridge FAST-LIO2 odometry into the deployment stack topics."""

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FastlioOdomBridge(Node):
    def __init__(self):
        super().__init__("fastlio_odom_bridge")

        self.declare_parameter("input_odom_topic", "/fastlio2/lio_odom")
        self.declare_parameter("output_odom_topic", "/odom")
        self.declare_parameter("output_pose_topic", "/pose")
        self.declare_parameter("output_frame_id", "map")
        self.declare_parameter("child_frame_id", "base_footprint")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_pose", True)

        self.input_odom_topic = str(self.get_parameter("input_odom_topic").value)
        self.output_odom_topic = str(self.get_parameter("output_odom_topic").value)
        self.output_pose_topic = str(self.get_parameter("output_pose_topic").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.publish_pose = bool(self.get_parameter("publish_pose").value)

        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 20)
        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.input_odom_topic,
            self.on_odom,
            50,
        )

        self.get_logger().info(
            "FAST-LIO odom bridge: "
            f"{self.input_odom_topic} -> {self.output_odom_topic}, "
            f"pose={self.output_pose_topic}, frame={self.output_frame_id}, "
            f"child={self.child_frame_id}, publish_tf={self.publish_tf}"
        )

    def on_odom(self, msg: Odometry):
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.output_frame_id or msg.header.frame_id
        out.child_frame_id = self.child_frame_id or msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        self.odom_pub.publish(out)

        if self.publish_pose:
            pose = PoseStamped()
            pose.header.stamp = out.header.stamp
            pose.header.frame_id = out.header.frame_id
            pose.pose = out.pose.pose
            self.pose_pub.publish(pose)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = out.header.stamp
            tf_msg.header.frame_id = out.header.frame_id
            tf_msg.child_frame_id = out.child_frame_id
            tf_msg.transform.translation.x = out.pose.pose.position.x
            tf_msg.transform.translation.y = out.pose.pose.position.y
            tf_msg.transform.translation.z = out.pose.pose.position.z
            tf_msg.transform.rotation = out.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = FastlioOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
