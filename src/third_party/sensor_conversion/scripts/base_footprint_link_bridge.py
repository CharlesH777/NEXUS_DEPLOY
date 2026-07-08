#!/usr/bin/env python3
"""
TF 桥接：发布 base_footprint -> base_link
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class BaseFootprintLinkBridge(Node):
    def __init__(self):
        super().__init__('base_footprint_link_bridge')

        self.tf_broadcaster = TransformBroadcaster(self)

        # 定时发布 base_footprint -> base_link
        self.timer = self.create_timer(0.1, self.publish_tf)

        self.get_logger().info('Base_footprint-base_link TF bridge started')

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'

        # base_link 在 base_footprint 上方一点点
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0  # 如果需要高度偏移，调整这个值
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = BaseFootprintLinkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
