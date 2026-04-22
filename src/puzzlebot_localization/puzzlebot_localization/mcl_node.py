#!/usr/bin/env python3
"""
mcl_node.py — Basic Monte Carlo Localization

Core MCL algorithm:
  1. Sample N particles in free space
  2. Score particles against /scan using likelihood field
  3. Filter: keep top survivors
  4. Estimate pose from survivors
  5. Propagate particles with odometry + noise
  6. Repeat
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from puzzlebot_localization.map_utils import (
    MAP_RESOLUTION,
    MAP_SIZE_PX,
    generate_obstacles_world_map,
    build_likelihood_field,
    sample_free_particles,
    score_particles,
    filter_particles,
    estimate_pose,
    propagate_particles,
)

WHEEL_RADIUS = 0.05    # metres
WHEEL_BASE   = 0.19    # metres


class MCLNode(Node):

    def __init__(self):
        super().__init__('mcl_node')

        # Parameters
        self.declare_parameter('n_particles',   500)
        self.declare_parameter('sigma_field_m', 0.2)
        self.declare_parameter('ray_step',        5)
        self.declare_parameter('keep_fraction',  0.5)
        self.declare_parameter('sigma_xy',      0.005)
        self.declare_parameter('sigma_theta',   0.01)

        n_particles   = self.get_parameter('n_particles').value
        sigma_field_m = self.get_parameter('sigma_field_m').value
        self.ray_step      = self.get_parameter('ray_step').value
        self.keep_fraction = self.get_parameter('keep_fraction').value
        self.sigma_xy      = self.get_parameter('sigma_xy').value
        self.sigma_theta   = self.get_parameter('sigma_theta').value

        # Load known map and build likelihood field
        self.get_logger().info('Loading obstacles world map...')
        self.world_map = generate_obstacles_world_map()

        self.get_logger().info(f'Building likelihood field (sigma={sigma_field_m} m)...')
        self.likelihood = build_likelihood_field(self.world_map, sigma_m=sigma_field_m)

        # Sample initial particle cloud from free space
        self.get_logger().info(f'Sampling {n_particles} particles...')
        self.particles = sample_free_particles(self.world_map, n_particles)
        self._n_particles = n_particles
        self.scores = np.zeros(n_particles)

        # Kidnap recovery
        self._zero_score_count  = 0
        self._KIDNAP_THRESHOLD  = 30

        # Odometry accumulators
        self._wr = 0.0
        self._wl = 0.0
        self._delta_s     = 0.0
        self._delta_theta = 0.0

        # Subscriptions
        self.create_subscription(LaserScan, '/scan',
                                 self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(Float32, '/wr', self._wr_callback, 10)
        self.create_subscription(Float32, '/wl', self._wl_callback, 10)

        # Publishers
        self.particle_pub = self.create_publisher(PoseArray,   '/particles', 10)
        self.pose_pub     = self.create_publisher(PoseStamped, '/mcl_pose',  10)

        # Particle cloud at 2 Hz
        self.create_timer(0.5, self._publish_particles)

        # 20 Hz odometry integration
        self._odom_dt = 1.0 / 20.0
        self.create_timer(self._odom_dt, self._integrate_odometry)

        self.get_logger().info(
            'MCLNode ready — /scan | /wr /wl → /particles /mcl_pose'
        )

    def _wr_callback(self, msg: Float32):
        self._wr = float(msg.data)

    def _wl_callback(self, msg: Float32):
        self._wl = float(msg.data)

    def _integrate_odometry(self):
        """20 Hz dead-reckoning accumulator."""
        v     = WHEEL_RADIUS * (self._wr + self._wl) / 2.0
        omega = WHEEL_RADIUS * (self._wr - self._wl) / WHEEL_BASE
        self._delta_s     += v     * self._odom_dt
        self._delta_theta += omega * self._odom_dt

    def _scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float64)

        # Count valid rays
        step_idx = np.arange(0, len(ranges), self.ray_step)
        r_sub    = ranges[step_idx]
        valid    = np.isfinite(r_sub) & (r_sub >= msg.range_min) & (r_sub <= msg.range_max)
        n_valid  = int(valid.sum())

        # Score particles against likelihood field
        self.scores = score_particles(
            self.particles, ranges,
            msg.angle_min, msg.angle_increment,
            self.likelihood,
            range_min=msg.range_min,
            range_max=msg.range_max,
            ray_step=self.ray_step,
        )

        best_score = float(self.scores.max()) if len(self.scores) > 0 else 0.0

        self.get_logger().info(
            f'[MCL] valid_rays={n_valid}  best={best_score:.0f}  '
            f'zero_count={self._zero_score_count}',
            throttle_duration_sec=2.0,
        )

        # Kidnap recovery
        if best_score <= 0.0:
            self._zero_score_count += 1
            if self._zero_score_count >= self._KIDNAP_THRESHOLD:
                self.get_logger().warn(
                    f'[MCL] Kidnap recovery — reinitialising particles'
                )
                self.particles = sample_free_particles(self.world_map, self._n_particles)
                self.scores    = np.zeros(self._n_particles)
                self._delta_s     = 0.0
                self._delta_theta = 0.0
                self._zero_score_count = 0
            return
        else:
            self._zero_score_count = 0

        # Filter survivors
        survivors, surv_scores = filter_particles(
            self.particles, self.scores, keep_fraction=self.keep_fraction)

        # Estimate pose
        x, y, theta = estimate_pose(survivors, surv_scores)
        self._publish_pose(x, y, theta, msg.header.stamp)

        # Reset odometry deltas
        delta_s, delta_theta = self._delta_s, self._delta_theta
        self._delta_s     = 0.0
        self._delta_theta = 0.0

        # Propagate particles
        self.particles = propagate_particles(
            survivors, surv_scores,
            delta_s, delta_theta,
            sigma_xy=self.sigma_xy,
            sigma_theta=self.sigma_theta,
            n_out=self._n_particles,
        )
        self.scores = np.zeros(len(self.particles))

    def _publish_pose(self, x, y, theta, stamp):
        msg = PoseStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = float(np.sin(theta / 2.0))
        msg.pose.orientation.w = float(np.cos(theta / 2.0))
        self.pose_pub.publish(msg)

    def _publish_particles(self):
        msg = PoseArray()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for x, y, theta in self.particles:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = 0.0
            p.orientation.z = float(np.sin(theta / 2.0))
            p.orientation.w = float(np.cos(theta / 2.0))
            msg.poses.append(p)
        self.particle_pub.publish(msg)


def main():
    rclpy.init()
    node = MCLNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
