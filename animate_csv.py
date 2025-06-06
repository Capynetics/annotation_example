import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Load CSV file
df = pd.read_csv("/home/jsouzasoar/annotation_example/13_annotated/scene_13_positions.csv")

# Extract robot data
robot_x = df["robot_x"].values
robot_y = df["robot_y"].values
robot_yaw = df["robot_yaw_rad"].values

# Extract object coordinates
object_coords = []
for i in range(1, 6):
    x = df[f"x{i}"].values
    y = df[f"y{i}"].values
    object_coords.append((x, y))

# Create the figure and axes
fig, ax = plt.subplots()
robot_body, = ax.plot([], [], 'bo', label="Robot")
robot_heading, = ax.plot([], [], 'b-', lw=1)
objects_plot = [ax.plot([], [], 'rx')[0] for _ in range(5)]

def init():
    ax.set_xlim(0, 12)
    ax.set_ylim(-7, 3)
    ax.set_aspect('equal')
    ax.set_title("Robot and Objects Animation")
    ax.legend()
    return [robot_body, robot_heading] + objects_plot

def update(frame):
    x = robot_x[frame]
    y = robot_y[frame]
    yaw = robot_yaw[frame]

    # Robot position (must be sequences)
    robot_body.set_data([x], [y])

    # Robot heading line
    dx = 0.5 * np.cos(yaw)
    dy = 0.5 * np.sin(yaw)
    robot_heading.set_data([x, x + dx], [y, y + dy])

    # Object positions (must also be sequences)
    for i, (ox, oy) in enumerate(object_coords):
        objects_plot[i].set_data([ox[frame]], [oy[frame]])

    return [robot_body, robot_heading] + objects_plot


ani = animation.FuncAnimation(fig, update, frames=len(df), init_func=init, blit=True, interval=50)

# Uncomment to display in a notebook or GUI
plt.show()

# Save to file (optional)
# ani.save("robot_scene_animation.mp4", writer='ffmpeg', fps=20)
