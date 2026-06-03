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
from std_msgs.msg import Float32


class VelSmoother(Node):

    def __init__(self):
        super().__init__("twist_relay")

        self.declare_parameter("max_linear_accel",  0.3)   # m/s²
        self.declare_parameter("max_angular_accel", 0.5)   # rad/s²
        self.declare_parameter("rate",             50.0)   # Hz

        # ── Stall guard ──────────────────────────────────────────────────
        # If we're commanding a linear move but the wheels aren't turning
        # (robot blocked, e.g. pushing into a pallet), the motors draw locked-
        # rotor current that can brown out the Jetson on the power bank. This
        # cuts the linear command the instant a stall is detected (local to the
        # Jetson at `rate` Hz — no cross-host latency) and keeps it cut until the
        # commander backs off or reverses. Angular is left untouched.
        self.declare_parameter("stall_guard",        True)
        self.declare_parameter("stall_lin_thresh",   0.03)  # m/s — only guard real moves
        self.declare_parameter("stall_wheel_thresh", 0.4)   # rad/s — absolute floor for "stalled"
        self.declare_parameter("stall_time",         0.08)  # s of stall before cutting (fast)
        self.declare_parameter("stall_release",      0.02)  # m/s target below which we unblock
        # Partial-stall detection: pushing a pallet often lets the wheels CREEP
        # (turn slow but not zero) while drawing high current — the absolute
        # floor above misses that. So we also flag a stall when the wheels turn
        # at less than `stall_fraction` of the speed the commanded velocity
        # implies (expected = |cur_lin| / wheel_radius rad/s).
        self.declare_parameter("wheel_radius",       0.05)  # m
        self.declare_parameter("stall_fraction",     0.5)   # cut if wheel < this × expected

        max_lin = self.get_parameter("max_linear_accel").value
        max_ang = self.get_parameter("max_angular_accel").value
        rate    = self.get_parameter("rate").value

        self._max_lin_accel = max_lin
        self._max_ang_accel = max_ang
        self._dt = 1.0 / rate

        self._stall_guard     = bool(self.get_parameter("stall_guard").value)
        self._stall_lin_thr   = float(self.get_parameter("stall_lin_thresh").value)
        self._stall_wheel_thr = float(self.get_parameter("stall_wheel_thresh").value)
        self._stall_time      = float(self.get_parameter("stall_time").value)
        self._stall_release   = float(self.get_parameter("stall_release").value)
        self._wheel_radius    = float(self.get_parameter("wheel_radius").value)
        self._stall_fraction  = float(self.get_parameter("stall_fraction").value)
        self._wl = 0.0
        self._wr = 0.0
        self._stall_t = 0.0
        self._blocked = False
        self._stall_dir = 1.0

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

        # Wheel encoder speeds (rad/s) for the stall guard — published locally
        # on the Jetson by velocity_bridge, so this stays low-latency.
        self.create_subscription(Float32, "/wl", self._wl_cb, sub_qos)
        self.create_subscription(Float32, "/wr", self._wr_cb, sub_qos)

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

    def _wl_cb(self, msg: Float32) -> None:
        self._wl = float(msg.data)

    def _wr_cb(self, msg: Float32) -> None:
        self._wr = float(msg.data)

    def _update_stall_guard(self) -> None:
        """Latch _blocked when commanding linear motion but the wheels are
        stalled; release when the commander backs off or reverses."""
        if not self._stall_guard:
            return
        wheel = max(abs(self._wl), abs(self._wr))
        driving = abs(self._cur_lin) > self._stall_lin_thr
        # Stall = wheels below the absolute floor OR below a fraction of the
        # speed the command implies (catches "creeping under heavy load").
        expected = abs(self._cur_lin) / self._wheel_radius if self._wheel_radius > 0 else 0.0
        stall_floor = max(self._stall_wheel_thr, self._stall_fraction * expected)
        if driving and wheel < stall_floor:
            self._stall_t += self._dt
            if self._stall_t >= self._stall_time and not self._blocked:
                self._blocked = True
                self._stall_dir = 1.0 if self._cur_lin >= 0.0 else -1.0
                self.get_logger().warn(
                    f"Stall guard: wheels loaded/blocked (wheel={wheel:.2f} < "
                    f"{stall_floor:.2f} rad/s) — cutting linear cmd.")
        else:
            self._stall_t = 0.0
        # Release when the commander stops pushing in the stalled direction.
        if self._blocked and (abs(self._tgt_lin) < self._stall_release
                              or self._tgt_lin * self._stall_dir < 0.0):
            self._blocked = False
            self._stall_t = 0.0
            self.get_logger().info("Stall guard: released.")

    @staticmethod
    def _ramp(current: float, target: float, max_delta: float) -> float:
        delta = target - current
        if delta > max_delta:
            delta = max_delta
        elif delta < -max_delta:
            delta = -max_delta
        return current + delta

    def _update(self) -> None:
        self._update_stall_guard()

        if self._blocked:
            self._cur_lin = 0.0   # hard cut — fastest relief from stall current
        else:
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
