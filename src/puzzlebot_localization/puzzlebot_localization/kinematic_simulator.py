#!/usr/bin/env python3
"""
Subscribes to /cmd_vel (geometry_msgs/Twist) and converts the desired
linear velocity v and angular velocity omega into individual wheel angular
speeds using the differential-drive inverse kinematics:

    omega_r = (2*v + omega*L) / (2*r)
    omega_l = (2*v - omega*L) / (2*r)

Publishes /wr and /wl (std_msgs/Float32) in rad/s for dead-reckoning.
Gazebo wheel commands are handled by the simple_controller node.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class KinematicSimulator(Node):

    def __init__(self):
        super().__init__('kinematic_simulator')

        # Robot physical parameters
        self.declare_parameter('wheel_radius', 0.05)        # r [m]
        self.declare_parameter('wheel_separation', 0.19)    # L [m]

        self.r_ = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.L_ = self.get_parameter('wheel_separation').get_parameter_value().double_value

        self.get_logger().info(
            f'Kinematic Simulator started — r={self.r_} m, L={self.L_} m')

        # Publishers for dead-reckoning
        self.wr_pub_ = self.create_publisher(Float32, '/wr', 10)
        self.wl_pub_ = self.create_publisher(Float32, '/wl', 10)

        # Subscriber
        self.cmd_sub_ = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        v = msg.linear.x        # linear velocity  [m/s]
        omega = msg.angular.z   # angular velocity [rad/s]

        # Inverse kinematics: (v, omega) -> (omega_r, omega_l)
        wr = float((2.0 * v + omega * self.L_) / (2.0 * self.r_))
        wl = float((2.0 * v - omega * self.L_) / (2.0 * self.r_))

        # Publish wheel velocities for dead-reckoning
        msg_r = Float32()
        msg_l = Float32()
        msg_r.data = wr
        msg_l.data = wl
        self.wr_pub_.publish(msg_r)
        self.wl_pub_.publish(msg_l)


def main():
    rclpy.init()
    node = KinematicSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
