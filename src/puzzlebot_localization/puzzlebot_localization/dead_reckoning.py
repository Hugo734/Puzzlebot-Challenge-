#!/usr/bin/env python3
"""
Subscribes to /wr and /wl (std_msgs/Float32, rad/s) and integrates the
robot pose using Euler's method at a fixed update rate.

Forward kinematics:
    v     = r * (omega_r + omega_l) / 2
    omega = r * (omega_r - omega_l) / L

Euler integration (dt from timer period):
    theta += omega * dt
    x     += v * cos(theta) * dt
    y     += v * sin(theta) * dt

Publishes /odom (nav_msgs/Odometry).
Quaternion computed with NumPy — no tf_transformations needed.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry


class Localisation(Node):

    def __init__(self):
        super().__init__('localisation')

        # Parameters
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.19)
        self.declare_parameter('update_rate', 20.0)         # Hz

        self.r_ = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.L_ = self.get_parameter('wheel_separation').get_parameter_value().double_value
        rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.dt_ = 1.0 / rate

        self.get_logger().info(
            f'Localisation started — r={self.r_} m, L={self.L_} m, dt={self.dt_:.3f} s')

        # Robot state
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # Latest wheel speeds (updated by subscribers)
        self.wr_ = 0.0
        self.wl_ = 0.0

        # Subscribers
        self.create_subscription(Float32, '/wr', self.wr_callback, 10)
        self.create_subscription(Float32, '/wl', self.wl_callback, 10)

        # Publisher
        self.odom_pub_ = self.create_publisher(Odometry, '/odom', 10)

        # Integration timer
        self.create_timer(self.dt_, self.timer_callback)

    def wr_callback(self, msg):
        self.wr_ = msg.data

    def wl_callback(self, msg):
        self.wl_ = msg.data

    def timer_callback(self):
        # Forward kinematics
        v = self.r_ * (self.wr_ + self.wl_) / 2.0
        omega = self.r_ * (self.wr_ - self.wl_) / self.L_

        # Euler integration
        self.theta_ += omega * self.dt_
        self.x_ += v * np.cos(self.theta_) * self.dt_
        self.y_ += v * np.sin(self.theta_) * self.dt_

        # Wrap theta to [-pi, pi]
        self.theta_ = float((self.theta_ + np.pi) % (2.0 * np.pi) - np.pi)

        # Quaternion from yaw (pure rotation around Z)
        qz = float(np.sin(self.theta_ / 2.0))
        qw = float(np.cos(self.theta_ / 2.0))

        # Build odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x_
        odom.pose.pose.position.y = self.y_
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.odom_pub_.publish(odom)


def main():
    rclpy.init()
    node = Localisation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
