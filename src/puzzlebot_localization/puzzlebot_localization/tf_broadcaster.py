#!/usr/bin/env python3
"""
Subscribes to /odom (nav_msgs/Odometry) and broadcasts the dynamic
transform odom -> base_footprint so that RViz and the TF tree reflect
the robot's current estimated pose.

This is kept separate from the localisation node so the TF source can
be replaced (e.g., by an EKF) without modifying the odometry publisher.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class TFBroadcasterNode(Node):

    def __init__(self):
        super().__init__('tf_broadcaster')

        self.br_ = TransformBroadcaster(self)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info('TF Broadcaster started — odom -> base_footprint')

    def odom_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation

        self.br_.sendTransform(t)


def main():
    rclpy.init()
    node = TFBroadcasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
