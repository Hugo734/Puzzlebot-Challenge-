import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

_qos_scan = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE,
)

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid as OccupancyGridMsg
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.time import Time
from rclpy.duration import Duration

from slam.occupancy_grid import OccupancyGrid
from slam.icp import scan_to_points, transform_points, icp


def _yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _se2_compose(a, b):
    """Compose SE(2) transforms a ⊕ b. Both are (x, y, theta). Returns (x, y, theta)."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by,
            ay + s * bx + c * by,
            _wrap(at + bt))


def _se2_inverse(a):
    """Inverse of SE(2) transform a = (x, y, theta)."""
    ax, ay, at = a
    c, s = math.cos(at), math.sin(at)
    return (-c * ax - s * ay,
             s * ax - c * ay,
            _wrap(-at))


class SLAMNode(Node):

    def __init__(self):
        super().__init__('slam_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('resolution',         0.05)
        self.declare_parameter('map_width',          400)
        self.declare_parameter('map_height',         400)
        self.declare_parameter('l_occ',              0.85)
        self.declare_parameter('l_free',            -0.40)
        self.declare_parameter('l_min',             -5.0)
        self.declare_parameter('l_max',              5.0)
        self.declare_parameter('use_icp',            False)
        self.declare_parameter('icp_max_iter',       20)
        self.declare_parameter('icp_tolerance',      1e-4)
        self.declare_parameter('icp_reject_dist',    0.3)
        self.declare_parameter('icp_min_points',     20)
        self.declare_parameter('icp_max_fitness',    0.2)
        self.declare_parameter('range_min',          0.12)
        self.declare_parameter('range_max',          10.0)
        self.declare_parameter('map_publish_every',  5)
        self.declare_parameter('base_frame',         'base_link')
        self.declare_parameter('ray_step',           1)
        self.declare_parameter('min_delta_xy',       0.02)
        self.declare_parameter('min_delta_theta',    0.01)
        self.declare_parameter('display_l_occ',      1.0)
        self.declare_parameter('display_l_free',    -0.5)
        self.declare_parameter('laser_yaw_trim',     0.0)
        self.declare_parameter('map_initial_heading', 0.0)

        res    = self.get_parameter('resolution').value
        w      = self.get_parameter('map_width').value
        h      = self.get_parameter('map_height').value

        # ── Occupancy grid ────────────────────────────────────────────
        self.grid = OccupancyGrid(
            width=w, height=h, resolution=res,
            l_occ=self.get_parameter('l_occ').value,
            l_free=self.get_parameter('l_free').value,
            l_min=self.get_parameter('l_min').value,
            l_max=self.get_parameter('l_max').value,
            display_l_occ=self.get_parameter('display_l_occ').value,
            display_l_free=self.get_parameter('display_l_free').value,
        )

        # ── State ─────────────────────────────────────────────────────
        # SLAM pose (map frame)
        self._sx = 0.0
        self._sy = 0.0
        self._stheta = 0.0

        # Last odometry pose (odom frame)
        self._ox = 0.0
        self._oy = 0.0
        self._otheta = 0.0
        self._odom_ready = False

        # Buffer of recent odom samples — (t_sec, x, y, theta) — used to
        # interpolate the robot pose at the scan's capture timestamp instead
        # of using the latest (newer) odom, which fans walls during rotation.
        self._odom_buf = deque(maxlen=200)   # ~10 s at 20 Hz

        # map→odom TF offset — seeded with map_initial_heading so the map
        # frame is pre-rotated to match the room orientation at launch.
        self._tf_x     = 0.0
        self._tf_y     = 0.0
        self._tf_theta = self.get_parameter('map_initial_heading').value

        # Keep previous scan already projected in WORLD frame so use_icp=True
        # matches against a correct historical reference (was: base-frame, then
        # re-projected at the *current* slam pose — which baked in zero motion).
        self._prev_cloud_world = None
        self._scan_count = 0

        # Last pose at which the occupancy grid was updated — used for the
        # minimum-movement threshold to suppress stationary LiDAR jitter.
        self._last_map_x     = 0.0
        self._last_map_y     = 0.0
        self._last_map_theta = 0.0

        # TF: cached base_frame → laser_frame static offset
        self.base_frame    = self.get_parameter('base_frame').value
        self._tf_buffer    = Buffer()
        self._tf_listener  = TransformListener(self._tf_buffer, self)
        self._laser_offset = None    # (x, y, yaw) — lazy-resolved on first scan

        # ── ROS interfaces ────────────────────────────────────────────
        self._map_pub  = self.create_publisher(OccupancyGridMsg, '/map', 1)
        self._pose_pub = self.create_publisher(PoseStamped, '/slam_pose', 10)
        self._tf_br    = TransformBroadcaster(self)

        # Publish map→odom at 10 Hz from the very start so RViz can display
        # the robot model before the first scan is processed.  The scan
        # callback keeps updating the same transform once data arrives.
        self.create_timer(0.1, self._tf_timer_cb)

        self.create_subscription(Odometry, '/odom',
                                 self._odom_callback, 10)
        self.create_subscription(LaserScan, '/scan',
                                 self._scan_callback, _qos_scan)

        self.get_logger().info(
            f'SLAM node started — {w}x{h} cells @ {res} m/cell '
            f'({w*res:.1f} x {h*res:.1f} m)')

    # ── Startup TF heartbeat ───────────────────────────────────────────

    def _tf_timer_cb(self):
        """Publish map→odom at 10 Hz regardless of scan state."""
        now = self.get_clock().now().to_msg()
        self._publish_tf(now)

    # ── Odometry callback ──────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry):
        self._ox     = msg.pose.pose.position.x
        self._oy     = msg.pose.pose.position.y
        self._otheta = _yaw_from_quaternion(msg.pose.pose.orientation)
        self._odom_ready = True

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._odom_buf.append((t, self._ox, self._oy, self._otheta))

    def _odom_at(self, t_query):
        """
        Linearly interpolate odom pose at ROS time `t_query` (float seconds).
        Falls back to the nearest sample if t_query is outside the buffered range.
        """
        if not self._odom_buf:
            return self._ox, self._oy, self._otheta

        # Clamp to buffer ends — extrapolation is more harmful than the latest
        # sample when scans arrive faster than the buffer can grow.
        if t_query <= self._odom_buf[0][0]:
            _, x, y, th = self._odom_buf[0]
            return x, y, th
        if t_query >= self._odom_buf[-1][0]:
            _, x, y, th = self._odom_buf[-1]
            return x, y, th

        # Walk back to find the bracketing pair (cheap — ≤200 entries)
        prev = self._odom_buf[0]
        for sample in self._odom_buf:
            if sample[0] >= t_query:
                t0, x0, y0, th0 = prev
                t1, x1, y1, th1 = sample
                a = (t_query - t0) / (t1 - t0) if t1 > t0 else 0.0
                # Wrap-safe yaw interpolation
                dth = _wrap(th1 - th0)
                return (x0 + a * (x1 - x0),
                        y0 + a * (y1 - y0),
                        _wrap(th0 + a * dth))
            prev = sample
        _, x, y, th = self._odom_buf[-1]
        return x, y, th

    # ── Scan callback ──────────────────────────────────────────────────

    def _resolve_laser_offset(self, laser_frame):
        """Lookup base_frame → laser_frame once and cache. Returns False if not yet available."""
        if self._laser_offset is not None:
            return True
        try:
            tf = self._tf_buffer.lookup_transform(
                self.base_frame, laser_frame, Time(),
                timeout=Duration(seconds=0.0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False
        t = tf.transform.translation
        q = tf.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw  = math.atan2(siny, cosy)
        trim = self.get_parameter('laser_yaw_trim').value
        yaw  = _wrap(yaw + trim)
        self._laser_offset = (float(t.x), float(t.y), float(yaw))
        self.get_logger().info(
            f'Laser→{self.base_frame} offset cached: '
            f'x={t.x:.3f} y={t.y:.3f} yaw={math.degrees(yaw):.1f}° '
            f'(trim={math.degrees(trim):.2f}°)')
        return True

    def _scan_callback(self, msg: LaserScan):
        if not self._odom_ready:
            return

        if not self._resolve_laser_offset(msg.header.frame_id):
            self.get_logger().warn(
                f'Waiting for TF {self.base_frame}→{msg.header.frame_id}',
                throttle_duration_sec=2.0)
            return

        # Two odom poses are needed:
        #   - scan_pose: odom at the scan's capture time → used to project rays
        #     into the map.  Without this, walls fan outward during rotation
        #     because the EKF has moved on by the time this callback fires.
        #   - live_pose: latest odom → used for TF and /slam_pose so the robot
        #     icon (driven by the live odom→base_footprint TF) stays aligned
        #     with the pose marker RViz renders.
        scan_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        scan_ox, scan_oy, scan_otheta = self._odom_at(scan_t)
        # _ox/_oy/_otheta keep the latest odom (untouched).

        lx, ly, lyaw = self._laser_offset
        range_min = self.get_parameter('range_min').value
        range_max = self.get_parameter('range_max').value

        # Subsample rays: every `ray_step`-th ray reduces CPU load on the
        # Jetson without meaningfully degrading map or ICP quality.
        ray_step  = self.get_parameter('ray_step').value
        if ray_step > 1:
            ranges_sub = np.array(msg.ranges, dtype=np.float64)[::ray_step]
            angle_inc  = msg.angle_increment * ray_step
        else:
            ranges_sub = msg.ranges
            angle_inc  = msg.angle_increment

        # Current scan as point cloud in base_link frame (laser offset applied)
        cloud = scan_to_points(
            ranges_sub, msg.angle_min, angle_inc,
            range_min, range_max,
            laser_x=lx, laser_y=ly, laser_yaw=lyaw)

        if len(cloud) < self.get_parameter('icp_min_points').value:
            return

        # ── Pose estimation: ICP (optional) or pure odometry ──────────
        # SLAM pose is always tracked at the LATEST odom (so /slam_pose and
        # the live TF stay synchronised).  The scan_pose is only used as the
        # ray origin for the map update below.
        self._propagate_from_odom()

        # If SLAM's own ICP is enabled, refine the latest pose with it.
        use_icp = self.get_parameter('use_icp').value
        if use_icp and self._prev_cloud_world is not None and \
                len(self._prev_cloud_world) >= self.get_parameter('icp_min_points').value:

            curr_world = transform_points(
                cloud, self._sx, self._sy, self._stheta)

            dx, dy, dtheta, fitness, converged = icp(
                source=curr_world,
                target=self._prev_cloud_world,
                init_dx=0.0, init_dy=0.0, init_dtheta=0.0,
                max_iter=self.get_parameter('icp_max_iter').value,
                tolerance=self.get_parameter('icp_tolerance').value,
                reject_dist=self.get_parameter('icp_reject_dist').value,
                min_points=self.get_parameter('icp_min_points').value,
            )
            if converged and fitness < self.get_parameter('icp_max_fitness').value:
                # SE(2) composition: T_icp ⊕ T_slam
                self._sx, self._sy, self._stheta = _se2_compose(
                    (dx, dy, dtheta), (self._sx, self._sy, self._stheta))

        # ── Map update — use SCAN-time pose so rotation doesn't smear walls
        # The scan-time SLAM pose is built from scan-time odom + the same
        # map→odom offset that produced the live SLAM pose.
        scan_slam_x, scan_slam_y, scan_slam_theta = _se2_compose(
            (self._tf_x, self._tf_y, self._tf_theta),
            (scan_ox, scan_oy, scan_otheta))

        # Within-scan rotation deskew: estimate omega across the scan period
        # from the odom buffer and let occupancy_grid spread per-ray heading.
        scan_dur = msg.scan_time if msg.scan_time > 0.0 else 0.1
        _, _, theta_end = self._odom_at(scan_t + scan_dur)
        scan_omega = _wrap(theta_end - scan_otheta) / scan_dur if scan_dur > 0 else 0.0

        # Minimum-movement gate: skip the (expensive) map update when the
        # robot is nearly stationary — stationary LiDAR jitter adds noise
        # without contributing new information.
        min_xy    = self.get_parameter('min_delta_xy').value
        min_theta = self.get_parameter('min_delta_theta').value
        moved = (
            math.hypot(scan_slam_x - self._last_map_x,
                       scan_slam_y - self._last_map_y) >= min_xy or
            abs(_wrap(scan_slam_theta - self._last_map_theta)) >= min_theta
        )
        if moved:
            self.grid.update_scan(
                scan_slam_x, scan_slam_y, scan_slam_theta,
                ranges_sub, msg.angle_min, angle_inc,
                range_min, range_max,
                laser_x=lx, laser_y=ly, laser_yaw=lyaw,
                scan_omega=scan_omega, scan_time=scan_dur)
            self._last_map_x     = scan_slam_x
            self._last_map_y     = scan_slam_y
            self._last_map_theta = scan_slam_theta

        # ── Recompute map→odom TF from the (possibly ICP-refined) SLAM pose
        # T_map_odom = T_map_base ⊕ T_odom_base⁻¹
        self._tf_x, self._tf_y, self._tf_theta = _se2_compose(
            (self._sx, self._sy, self._stheta),
            _se2_inverse((self._ox, self._oy, self._otheta)))

        # ── Publish ────────────────────────────────────────────────────
        now = self.get_clock().now().to_msg()
        self._publish_pose(now)
        self._publish_tf(now)

        self._scan_count += 1
        if self._scan_count % self.get_parameter('map_publish_every').value == 0:
            self._publish_map(now)

        # Cache the current scan as a WORLD-frame cloud at the current SLAM
        # pose so the next iteration's ICP can match against an absolute
        # reference (not re-projected with zero displacement).
        self._prev_cloud_world = transform_points(
            cloud, self._sx, self._sy, self._stheta)

    def _propagate_from_odom(self):
        """Use raw odometry as the SLAM pose (no ICP correction)."""
        # T_map_base = T_map_odom ⊕ T_odom_base (full SE(2) — works at any yaw)
        self._sx, self._sy, self._stheta = _se2_compose(
            (self._tf_x, self._tf_y, self._tf_theta),
            (self._ox, self._oy, self._otheta))

    # ── Publishers ─────────────────────────────────────────────────────

    def _publish_pose(self, stamp):
        msg = PoseStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'map'
        msg.pose.position.x = self._sx
        msg.pose.position.y = self._sy
        msg.pose.position.z = 0.0
        half = self._stheta / 2.0
        msg.pose.orientation.z = math.sin(half)
        msg.pose.orientation.w = math.cos(half)
        self._pose_pub.publish(msg)

    def _publish_tf(self, stamp):
        tf = TransformStamped()
        tf.header.stamp    = stamp
        tf.header.frame_id = 'map'
        tf.child_frame_id  = 'odom'
        tf.transform.translation.x = self._tf_x
        tf.transform.translation.y = self._tf_y
        tf.transform.translation.z = 0.0
        half = self._tf_theta / 2.0
        tf.transform.rotation.z = math.sin(half)
        tf.transform.rotation.w = math.cos(half)
        self._tf_br.sendTransform(tf)

    def _publish_map(self, stamp):
        msg = OccupancyGridMsg()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'map'

        msg.info.resolution = self.grid.resolution
        msg.info.width      = self.grid.width
        msg.info.height     = self.grid.height
        msg.info.origin.position.x = self.grid.origin_x
        msg.info.origin.position.y = self.grid.origin_y
        msg.info.origin.orientation.w = 1.0

        msg.data = self.grid.to_ros_data()
        self._map_pub.publish(msg)

        self.get_logger().info(
            f'Map published — pose=({self._sx:.2f}, {self._sy:.2f}, '
            f'{math.degrees(self._stheta):.1f}°)',
            throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = SLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
