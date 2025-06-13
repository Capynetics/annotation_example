import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import json
import matplotlib

matplotlib.rcParams['animation.ffmpeg_path'] = '/usr/bin/ffmpeg'  # Adjust if your ffmpeg is elsewhere

scene = 1
# Load CSV file
df = pd.read_csv(f"/home/jsouzasoar/annotation_example/{scene}_poses/{scene}_robot_and_participants.csv")

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

# Load occupancy points
with open(f"/home/jsouzasoar/annotation_example/{scene}_poses/{scene}_occupancy_xy_points.json", "r") as f:
    occupancy_data = json.load(f)

# Extract occupancy points
occupancy_x = [point["x"] for point in occupancy_data]
occupancy_y = [point["y"] for point in occupancy_data]

# Create the figure and axes
fig, ax = plt.subplots()
# Robot as a red triangle (Polygon)
robot_triangle = plt.Polygon([[0, 0], [0, 0], [0, 0]], closed=True, color='red', label="Robot")
ax.add_patch(robot_triangle)
# Participants as blue dots
objects_plot = [ax.plot([], [], 'bo', markersize=8, label="Participant" if i == 0 else "")[0] for i in range(5)]
occupancy_plot = ax.plot(occupancy_x, occupancy_y, 'k.', markersize=1, label="Occupancy")[0]

def get_triangle_coords(x, y, yaw, base=0.3, height=0.5):
    # Returns coordinates for an isosceles triangle centered at (x, y) and oriented by yaw
    # Triangle points: tip, left base, right base
    tip = np.array([x + height * np.cos(yaw), y + height * np.sin(yaw)])
    left = np.array([
        x + base/2 * np.cos(yaw + np.pi/2),
        y + base/2 * np.sin(yaw + np.pi/2)
    ])
    right = np.array([
        x + base/2 * np.cos(yaw - np.pi/2),
        y + base/2 * np.sin(yaw - np.pi/2)
    ])
    return np.array([tip, left, right])

def init():
    ax.set_xlim(0, 12)
    ax.set_ylim(-7, 3)
    ax.set_aspect('equal')
    ax.set_title("Robot and Participants position over time")
    ax.set_xlabel("x (meters)")
    ax.set_ylabel("y (meters)")
    #ax.legend()
    # Initialize robot triangle
    coords = get_triangle_coords(robot_x[0], robot_y[0], robot_yaw[0])
    robot_triangle.set_xy(coords)
    # Initialize participants
    for i, (ox, oy) in enumerate(object_coords):
        objects_plot[i].set_data([ox[0]], [oy[0]])
    return [robot_triangle, occupancy_plot] + objects_plot

def update(frame):
    x = robot_x[frame]
    y = robot_y[frame]
    yaw = robot_yaw[frame]
    # Update robot triangle
    coords = get_triangle_coords(x, y, yaw)
    robot_triangle.set_xy(coords)
    # Update participants
    for i, (ox, oy) in enumerate(object_coords):
        objects_plot[i].set_data([ox[frame]], [oy[frame]])
    return [robot_triangle, occupancy_plot] + objects_plot

ani = animation.FuncAnimation(fig, update, frames=len(df), init_func=init, blit=True, interval=50)

plt.show()

# Save to file (optional)
#ani.save("robot_scene_animation.mp4", writer='ffmpeg', fps=20)
