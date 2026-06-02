"""ROS2 node that controls the FPGA lifter via a GPIO HAL.

Subscriptions
-------------
/lifter_level  (std_msgs/UInt8)
    Target level 0-7.  Values outside [0, 7] are clamped before forwarding.

Publications
------------
/lifter_status  (std_msgs/UInt8)
    Current level, published at 1 Hz.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8


class LiftingNode(Node):
    """Bridges /lifter_level commands to GPIO hardware via a selectable HAL."""

    def __init__(self) -> None:
        super().__init__('lifting_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('use_mock_gpio', True)
        self.declare_parameter('pin_bit0', 11)
        self.declare_parameter('pin_bit1', 13)
        self.declare_parameter('pin_bit2', 15)
        self.declare_parameter('level_rest', 0)
        self.declare_parameter('level_transport', 1)
        self.declare_parameter('level_pick_floor', 3)
        self.declare_parameter('level_carry', 4)
        self.declare_parameter('level_pick_rack2', 5)
        self.declare_parameter('level_max', 7)

        use_mock: bool = self.get_parameter('use_mock_gpio').get_parameter_value().bool_value

        # ── HAL selection ─────────────────────────────────────────────
        if use_mock:
            from lifting.hal.mock import MockGpioDriver
            self._driver = MockGpioDriver()
            self.get_logger().info('LiftingNode: using MockGpioDriver')
        else:
            from lifting.hal.jetson import JetsonGpioDriver
            self._driver = JetsonGpioDriver()
            self.get_logger().info('LiftingNode: using JetsonGpioDriver')

        self._driver.setup()
        self._current_level: int = 0

        # ── ROS interfaces ────────────────────────────────────────────
        self._sub = self.create_subscription(
            UInt8,
            '/lifter_level',
            self._on_lifter_level,
            10,
        )

        self._status_pub = self.create_publisher(UInt8, '/lifter_status', 10)

        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info('LiftingNode ready — subscribed to /lifter_level')

    # ── Callbacks ─────────────────────────────────────────────────────

    def _on_lifter_level(self, msg: UInt8) -> None:
        """Clamp level to [0, 7] and forward to the GPIO driver."""
        raw_level: int = int(msg.data)
        level = max(0, min(7, raw_level))

        if raw_level != level:
            self.get_logger().warn(
                'Received out-of-range level %d; clamped to %d', raw_level, level
            )

        self._driver.set_level(level)
        self._current_level = level
        self.get_logger().debug('Lifter set to level %d (binary: %s)', level, format(level, '03b'))

    def _publish_status(self) -> None:
        """Publish the current lifter level at 1 Hz."""
        msg = UInt8()
        msg.data = self._current_level
        self._status_pub.publish(msg)

    # ── Lifecycle ─────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self._driver.cleanup()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LiftingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
