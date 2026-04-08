#!/usr/bin/env python3
"""Relays Twist messages to TwistStamped for the controller."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistRelay(Node):

    def __init__(self):
        super().__init__("twist_relay")
        self.twist_sub_ = self.create_subscription(
            Twist, "cmd_vel", self.twistCallback, 10)
        self.twist_pub_ = self.create_publisher(
            TwistStamped, "puzzlebot_controller/cmd_vel", 10)

    def twistCallback(self, msg):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.twist = msg
        self.twist_pub_.publish(twist_stamped)


def main():
    rclpy.init()
    twist_relay = TwistRelay()
    rclpy.spin(twist_relay)
    twist_relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
