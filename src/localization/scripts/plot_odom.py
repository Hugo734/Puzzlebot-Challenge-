#!/usr/bin/env python3
"""
Plot the /odom trajectory from a recorded ROS 2 bag.

Usage:
    python3 plot_odom.py [bag_directory]

Saves the plot to src/img/odom_trajectory.png.

Launch the dead-reckoning stack with:
    ros2 launch puzzlebot_localization dead_reckoning.launch.py

Record the bag with:
    ros2 bag record /odom -o odom_square
"""
import glob
import sqlite3
import sys

import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

bag_dir = sys.argv[1] if len(sys.argv) > 1 else 'odom_square'

db_files = glob.glob(f'{bag_dir}/*.db3')
if not db_files:
    sys.exit(f'No .db3 file found in "{bag_dir}". Did you record the bag there?')

conn = sqlite3.connect(db_files[0])
rows = conn.execute(
    "SELECT data FROM messages m "
    "JOIN topics t ON m.topic_id = t.id "
    "WHERE t.name = '/odom'"
).fetchall()
conn.close()

if not rows:
    sys.exit('No /odom messages found in the bag.')

xs, ys = [], []
for (data,) in rows:
    msg = deserialize_message(bytes(data), Odometry)
    xs.append(msg.pose.pose.position.x)
    ys.append(msg.pose.pose.position.y)

fig, ax = plt.subplots(figsize=(5, 5))
ax.plot(xs, ys, 'b-', linewidth=1.5, label='Dead-reckoning odometry (/odom)')
ax.plot(xs[0],  ys[0],  'go', markersize=8, label='Start')
ax.plot(xs[-1], ys[-1], 'rs', markersize=8, label='End')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_title('Estimated Trajectory from Dead-Reckoning Odometry')
ax.legend()
ax.set_aspect('equal')
ax.grid(True)
fig.tight_layout()

plt.show()
