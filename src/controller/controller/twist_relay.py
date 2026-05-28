#!/usr/bin/env python3
"""Velocity smoother: rate-limits acceleration to prevent current spikes on the
hackerboard when driving from a power bank.

Publishers send desired velocity to /cmd_vel_in; this node ramps the actual
command at max_linear_accel (m/s²) and max_angular_accel (rad/s²) before
forwarding it to the hardware and simulation paths.

Outputs:
  /cmd_vel                        Twist       → hackerboard micro_ros agent
  puzzlebot_controller/cmd_vel    TwistStamped → simple_controller (sim)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TwistStamped


class VelSmoother(Node):

    def __init__(self):
        super().__init__("twist_relay")

        self.declare_parameter("max_linear_accel",  0.3)   # m/s²
        self.declare_parameter("max_angular_accel", 0.5)   # rad/s²
        self.declare_parameter("rate",             50.0)   # Hz

        max_lin = self.get_parameter("max_linear_accel").value
        max_ang = self.get_parameter("max_angular_accel").value
        rate    = self.get_parameter("rate").value

        self._max_lin_accel = max_lin
        self._max_ang_accel = max_ang
        self._dt = 1.0 / rate

        self._cur_lin = 0.0
        self._cur_ang = 0.0
        self._tgt_lin = 0.0
        self._tgt_ang = 0.0

        # BEST_EFFORT so we can receive from any publisher QoS (perception
        # nodes use BEST_EFFORT; navigation/dashboard use RELIABLE — both work).
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub = self.create_subscription(
            Twist, "cmd_vel_in", self._cmd_cb, sub_qos)

        # Match the QoS the perception/navigation nodes used before so the
        # hackerboard micro_ros agent sees identical message characteristics.
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Real hardware: hackerboard micro_ros subscribes to /cmd_vel (Twist)
        self._twist_pub = self.create_publisher(Twist, "cmd_vel", out_qos)
        # Simulation: simple_controller expects TwistStamped
        self._stamped_pub = self.create_publisher(
            TwistStamped, "puzzlebot_controller/cmd_vel", 10)

        self.create_timer(self._dt, self._update)

        self.get_logger().info(
            f"VelSmoother — max_lin={max_lin} m/s²  "
            f"max_ang={max_ang} rad/s²  rate={rate} Hz")

    def _cmd_cb(self, msg: Twist) -> None:
        self._tgt_lin = msg.linear.x
        self._tgt_ang = msg.angular.z

    @staticmethod
    def _ramp(current: float, target: float, max_delta: float) -> float:
        delta = target - current
        if delta > max_delta:
            delta = max_delta
        elif delta < -max_delta:
            delta = -max_delta
        return current + delta

    def _update(self) -> None:
        self._cur_lin = self._ramp(
            self._cur_lin, self._tgt_lin, self._max_lin_accel * self._dt)
        self._cur_ang = self._ramp(
            self._cur_ang, self._tgt_ang, self._max_ang_accel * self._dt)

        t = Twist()
        t.linear.x  = self._cur_lin
        t.angular.z = self._cur_ang
        self._twist_pub.publish(t)

        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.twist = t
        self._stamped_pub.publish(ts)


def main():
    rclpy.init()
    node = VelSmoother()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
