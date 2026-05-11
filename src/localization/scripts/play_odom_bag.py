#!/usr/bin/env python3
"""
Play /odom messages from a ROS2 bag file with a live 2D trajectory plot.

Usage:
  python3 play_odom_bag.py <bag_path>
  python3 play_odom_bag.py ./my_bag --rate 2.0   # double speed
  python3 play_odom_bag.py ./my_bag --no-publish  # plot only, no ROS publishing

Press Ctrl+C to stop early.
"""

import sys
import time
import math
import argparse

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

import rclpy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
import rosbag2_py


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def setup_plot():
    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_title('EKF Odometry — Live Trajectory', fontsize=13)
    ax.set_xlabel('x  [m]')
    ax.set_ylabel('y  [m]')
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.5)

    trail,   = ax.plot([], [], color='royalblue', linewidth=1.5, label='trajectory')
    dot,     = ax.plot([], [], 'o', color='royalblue', markersize=7)
    arrow    = ax.annotate('', xy=(0, 0), xytext=(0, 0),
                           arrowprops=dict(arrowstyle='->', color='tomato', lw=2.5))
    info_txt = ax.text(0.02, 0.97, '', transform=ax.transAxes,
                       fontsize=9, verticalalignment='top',
                       fontfamily='monospace',
                       bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))

    ax.legend(loc='lower right')
    fig.tight_layout()
    return fig, ax, trail, dot, arrow, info_txt


def update_plot(ax, trail, dot, arrow, info_txt, xs, ys, yaws, count, stamp):
    x, y, yaw = xs[-1], ys[-1], yaws[-1]
    arrow_len  = 0.15

    trail.set_data(xs, ys)
    dot.set_data([x], [y])

    # Redraw orientation arrow
    arrow.xy     = (x + arrow_len * math.cos(yaw), y + arrow_len * math.sin(yaw))
    arrow.xytext = (x, y)

    info_txt.set_text(
        f'msg : {count}\n'
        f'x   : {x:+.3f} m\n'
        f'y   : {y:+.3f} m\n'
        f'yaw : {math.degrees(yaw):+.1f}°\n'
        f't   : {stamp:.2f} s'
    )

    # Auto-scale with a small margin
    margin = 0.5
    ax.set_xlim(min(xs) - margin, max(xs) + margin)
    ax.set_ylim(min(ys) - margin, max(ys) + margin)

    plt.pause(0.001)


def main():
    parser = argparse.ArgumentParser(description='Play /odom from a ROS2 bag with live plot')
    parser.add_argument('bag_path', help='Path to the bag folder')
    parser.add_argument('--rate', type=float, default=1.0,
                        help='Playback speed multiplier (default: 1.0)')
    parser.add_argument('--no-publish', action='store_true',
                        help='Plot only — do not republish /odom to ROS')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('odom_bag_player')
    pub  = None if args.no_publish else node.create_publisher(Odometry, '/odom', 10)

    # Open bag
    storage_options   = rosbag2_py.StorageOptions(uri=args.bag_path, storage_id='')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if '/odom' not in topics:
        node.get_logger().error(
            f'/odom not found in bag. Available: {list(topics.keys())}')
        rclpy.shutdown()
        sys.exit(1)

    node.get_logger().info(
        f"Playing '{args.bag_path}' at {args.rate}x — "
        f"{'plot only' if args.no_publish else 'publishing /odom'}")

    fig, ax, trail, dot, arrow, info_txt = setup_plot()

    xs, ys, yaws = [], [], []
    count      = 0
    prev_stamp = None
    prev_wall  = None
    t0_bag     = None

    try:
        while reader.has_next():
            topic, data, stamp_ns = reader.read_next()
            if topic != '/odom':
                continue

            # Timing
            if prev_stamp is not None:
                delta_bag  = (stamp_ns - prev_stamp) / 1e9 / args.rate
                delta_wall = time.monotonic() - prev_wall
                sleep_time = delta_bag - delta_wall
                if sleep_time > 0:
                    time.sleep(sleep_time)

            msg = deserialize_message(data, Odometry)
            if pub:
                pub.publish(msg)

            if t0_bag is None:
                t0_bag = stamp_ns

            x   = msg.pose.pose.position.x
            y   = msg.pose.pose.position.y
            yaw = yaw_from_quaternion(msg.pose.pose.orientation)

            xs.append(x)
            ys.append(y)
            yaws.append(yaw)
            count     += 1
            prev_stamp = stamp_ns
            prev_wall  = time.monotonic()

            update_plot(ax, trail, dot, arrow, info_txt,
                        xs, ys, yaws, count,
                        (stamp_ns - t0_bag) / 1e9)

            node.get_logger().info(
                f'[{count}] x={x:.3f}  y={y:.3f}  yaw={math.degrees(yaw):.1f}°',
                throttle_duration_sec=1.0,
            )

    except KeyboardInterrupt:
        pass

    node.get_logger().info(f'Done — {count} messages. Close the plot window to exit.')
    plt.ioff()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
