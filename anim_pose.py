import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Path to the CSV file
csv_path = '/home/jsouzasoar/robot_pose.csv'

# Load poses from CSV
poses = []
with open(csv_path, newline='') as file:
    reader = csv.DictReader(file)
    for row in reader:
        timestamp_ns = int(row['timestamp_ns'])
        x = float(row['x'])
        y = float(row['y'])
        yaw = float(row['yaw_rad'])
        poses.append((timestamp_ns, x, y, yaw))

# Sort by time (just in case)
poses.sort()

timestamps, xs, ys, yaws = zip(*poses)

t0 = timestamps[0]
times = [(t - t0) / 1e9 for t in timestamps]

# Setup animation
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.grid()
ax.set_title('Robot Pose Animation')
robot_dot, = ax.plot([], [], 'bo')  # blue dot for robot position

arrow_patch = None  # placeholder for the arrow

def init():
    ax.set_xlim(min(xs) - 1, max(xs) + 1)
    ax.set_ylim(min(ys) - 1, max(ys) + 1)
    return robot_dot,

def update(i):
    global arrow_patch
    x, y, yaw = xs[i], ys[i], yaws[i]
    robot_dot.set_data([x], [y])  # Set as sequence

    # Remove previous arrow if exists
    if arrow_patch is not None:
        arrow_patch.remove()

    dx = 0.5 * np.cos(yaw)
    dy = 0.5 * np.sin(yaw)
    arrow_patch = ax.arrow(x, y, dx, dy, head_width=0.1, color='r')

    return robot_dot, arrow_patch

ani = animation.FuncAnimation(
    fig, update, frames=len(xs),
    init_func=init, blit=True, interval=50
)

# Save to GIF using Pillow
from matplotlib.animation import PillowWriter
output_path = '/home/jsouzasoar/robot_pose_animation.gif'
ani.save(output_path, writer=PillowWriter(fps=20))
print(f"Animation saved as {output_path}")
