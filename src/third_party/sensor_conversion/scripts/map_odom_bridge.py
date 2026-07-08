#!/usr/bin/env python3
"""
TF 桥接：发布 map -> odom，连接 NEXUS 的 TF 树
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class MapOdomBridge(Node):
    def __init__(self):
        super().__init__('map_odom_bridge')

        self.tf_broadcaster = TransformBroadcaster(self)

        # 定时发布 map -> odom (静态，身份变换)
        self.timer = self.create_timer(0.1, self.publish_tf)

        self.get_logger().info('Map-Odom TF bridge started')

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        # 身份变换（map 和 odom 重合）
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = MapOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
