#!/usr/bin/env python3
"""
Republishes /scan with local clock timestamp.

Fixes clock-offset issues between the robot (lidar source) and the local
PC running SLAM + RViz: the lidar header.stamp is overwritten with now()
so TF lookups always use the local timeline.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan


_qos_in = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE,
)
_qos_out = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE,
)


class ScanTimestampFix(Node):
    def __init__(self):
        super().__init__('scan_timestamp_fix')
        self._pub = self.create_publisher(LaserScan, '/scan_fixed', _qos_out)
        self._sub = self.create_subscription(LaserScan, '/scan', self._cb, _qos_in)
        self.get_logger().info('scan_timestamp_fix ready — /scan → /scan_fixed with local stamp')

    def _cb(self, msg: LaserScan):
        # Stamp 200 ms in the past.  Needs to exceed the worst-case
        # gap between consecutive map→odom TF publishes (SLAM's
        # heartbeat + scan-callback variance), otherwise RViz throws
        # "extrapolation into the future" and drops the scan.  200 ms
        # comfortably covers a 30 Hz heartbeat with margin while keeping
        # the visual lag tolerable during rotation.
        now = self.get_clock().now()
        msg.header.stamp = (now - Duration(nanoseconds=200_000_000)).to_msg()
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = ScanTimestampFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
