import os
import json
import glob
import numpy as np
import cv2

# Set of scenes requiring HSR offsets and pose
hsr_scene_numbers = {1, 2, 3, 4, 5, 6, 8, 9, 10, 12, 13, 27, 28, 29, 30, 31, 32, 34, 35, 36, 38, 39, 46, 52}

# Static transforms for image alignment
image_pose_hsr = {'x': 1.80, 'y': -16.70, 'z': -2.10, 'yaw': np.radians(4.0)}
image_pose_jackal = {'x': -0.30, 'y': -16.80, 'z': -2.20, 'yaw': np.radians(-3.0)}

# Default image (same for all, but reused for transform)
default_image_name = "static_obstacle_map_cropped.png"

def extract_black_xy_from_image(image_path, pose, resolution=0.05):
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"⚠️ Failed to read image: {image_path}")
        return []

    black_indices = np.where(img == 0)
    if len(black_indices[0]) == 0:
        return []

    ys_pix, xs_pix = black_indices
    xs = xs_pix * resolution
    ys = ys_pix * resolution
    zs = np.zeros_like(xs)

    points = np.stack([xs, ys, zs], axis=1)

    yaw = pose['yaw']
    rotation = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0, 0, 1]
    ])
    translation = np.array([pose['x'], pose['y'], pose['z']])
    transformed = (rotation @ points.T).T + translation

    return [{"x": float(x), "y": float(y)} for x, y, _ in transformed]

def process_all_scenes():
    annotated_dirs = sorted(glob.glob("*_annotated"))

    for folder in annotated_dirs:
        try:
            scene_number = int(folder.split("_")[0])
        except ValueError:
            continue

        poses_folder = f"{scene_number}_poses"
        if not os.path.exists(poses_folder):
            print(f"⚠️ Skipping scene {scene_number}: poses folder not found.")
            continue

        # Use correct pose
        image_pose = image_pose_hsr if scene_number in hsr_scene_numbers else image_pose_jackal

        if not os.path.exists(default_image_name):
            print(f"❌ Missing occupancy map image: {default_image_name}")
            break

        print(f"🔄 Processing scene {scene_number}...")

        xy_points = extract_black_xy_from_image(default_image_name, image_pose)
        if not xy_points:
            print(f"  ⚠️ No black pixels found for scene {scene_number}")
            continue

        out_filename = f"{scene_number}_occupancy_xy_points.json"
        out_paths = [
            os.path.join(folder, out_filename),
            os.path.join(poses_folder, out_filename)
        ]

        for path in out_paths:
            with open(path, "w") as f:
                json.dump(xy_points, f, indent=2)
            print(f"  ✅ Saved to {path} ({len(xy_points)} points)")

if __name__ == "__main__":
    process_all_scenes()
