#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from tf2_ros import TransformBroadcaster


class PoseToTfBridge(Node):
    def __init__(self) -> None:
        super().__init__("pose_to_tf_bridge")

        self.declare_parameter("pose_topic", "/pose")
        self.declare_parameter("parent_frame", "world")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 0.0)

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.child_frame = str(self.get_parameter("child_frame").value)
        self.offset_x = float(self.get_parameter("offset_x").value)
        self.offset_y = float(self.get_parameter("offset_y").value)
        self.offset_z = float(self.get_parameter("offset_z").value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 20)

        self.get_logger().info(
            f"Bridging {self.pose_topic} -> TF {self.parent_frame} -> {self.child_frame} "
            f"with offset=({self.offset_x}, {self.offset_y}, {self.offset_z})"
        )

    @staticmethod
    def rotate_vector_by_quaternion(vector: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
        q_xyz = quaternion[:3]
        q_w = float(quaternion[3])
        t = 2.0 * np.cross(q_xyz, vector)
        return vector + q_w * t + np.cross(q_xyz, t)

    def pose_callback(self, msg: PoseStamped) -> None:
        tf_msg = TransformStamped()
        tf_msg.header = msg.header
        tf_msg.header.frame_id = msg.header.frame_id or self.parent_frame
        tf_msg.child_frame_id = self.child_frame
        quaternion = np.array(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ],
            dtype=np.float64,
        )
        offset = self.rotate_vector_by_quaternion(
            np.array([self.offset_x, self.offset_y, self.offset_z], dtype=np.float64),
            quaternion,
        )
        tf_msg.transform.translation.x = msg.pose.position.x + float(offset[0])
        tf_msg.transform.translation.y = msg.pose.position.y + float(offset[1])
        tf_msg.transform.translation.z = msg.pose.position.z + float(offset[2])
        tf_msg.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def main() -> None:
    rclpy.init()
    node = PoseToTfBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
