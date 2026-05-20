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
        # Stamp 300 ms in the past so the TF buffer always has a valid entry
        # at the scan's timestamp even when RViz's TF buffer lags by ~140 ms.
        now = self.get_clock().now()
        msg.header.stamp = (now - Duration(nanoseconds=300_000_000)).to_msg()
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = ScanTimestampFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
