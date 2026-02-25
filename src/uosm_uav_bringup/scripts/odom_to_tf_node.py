#!/usr/bin/env python3
"""
Lightweight node that subscribes to an Odometry topic and broadcasts
the corresponding odom -> base_link TF transform.

Used in bag playback mode where the perception/MSF node is not running.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf_node')

        self.declare_parameter('odom_topic', '/mavros/odometry/out')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.parent_frame_ = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame_ = self.get_parameter('child_frame').get_parameter_value().string_value

        self.tf_broadcaster_ = TransformBroadcaster(self)
        self.sub_ = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )

        self.get_logger().info(
            f'Broadcasting TF: {self.parent_frame_} -> {self.child_frame_} from {odom_topic}'
        )

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame_
        t.child_frame_id = self.child_frame_
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster_.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
