import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read CSV
df = pd.read_csv('/home/jsouzasoar/annotation_example/13_annotated/scene_13_positions.csv')

# Plot robot trajectory
plt.figure(figsize=(10, 8))
plt.plot(df['robot_x'], df['robot_y'], label='Robot Trajectory', color='blue')

# Plot robot orientation as arrows
arrow_scale = 0.3
for x, y, yaw in zip(df['robot_x'], df['robot_y'], df['robot_yaw_rad']):
    plt.arrow(x, y, arrow_scale*np.cos(yaw), arrow_scale*np.sin(yaw),
              head_width=0.15, head_length=0.2, fc='blue', ec='blue', alpha=0.5)

# Plot the 5 points as scatter points (use different colors/markers)
for i in range(1, 6):
    plt.scatter(df[f'x{i}'], df[f'y{i}'], label=f'Point {i}', s=40)

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Robot Trajectory and Points')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.tight_layout()
plt.show()