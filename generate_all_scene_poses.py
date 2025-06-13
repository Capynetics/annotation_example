import glob
import json
import os
import numpy as np
import pandas as pd
import bisect

# Set of HSR scenes with known time offset
hsr_scene_numbers = {1, 2, 3, 4, 5, 6, 8, 9, 10, 12, 13, 27, 28, 29, 30, 31, 32, 34, 35, 36, 38, 39, 46, 52}

# Helper functions
def extract_timestamp_from_filename(filename):
    ts_str = os.path.basename(filename).rsplit('.', 2)
    if len(ts_str) >= 2:
        full_ts = '.'.join(ts_str[:2])
        return int(float(full_ts) * 1e9)
    return None

def find_closest_pose_idx(pose_times, ts):
    i = bisect.bisect_left(pose_times, ts)
    if i > 0 and (i == len(pose_times) or abs(ts - pose_times[i - 1]) < abs(ts - pose_times[i])):
        return i - 1
    return i

def load_participant_centers(json_path):
    if not os.path.exists(json_path):
        return []
    with open(json_path, 'r') as f:
        data = json.load(f)

    centers = []
    for fig in data.get("figures", []):
        if fig.get("geometryType") == "cuboid_3d":
            pos = fig["geometry"]["position"]
            centers.append((pos["x"], pos["y"]))
    return centers[:5]  # Limit to 5

# Process each annotated scene
for folder in sorted(glob.glob("*_annotated")):
    try:
        scene_number = int(folder.split("_")[0])
    except ValueError:
        continue

    print(f"Processing scene {scene_number}...")

    pose_csv_path = f"{folder}/{scene_number}_robot_pose.csv"
    offset_json_path = f"{folder}/{scene_number}_grs_to_bot_offset.json"
    pcd_files = sorted(glob.glob(f"{folder}/scene_{scene_number}/pointcloud/*.pcd"))
    ann_files = sorted(glob.glob(f"{folder}/scene_{scene_number}/ann/*.json"))

    if not os.path.exists(pose_csv_path) or not os.path.exists(offset_json_path):
        print(f"  Skipping scene {scene_number} (missing pose or offset files).")
        continue

    try:
        pose_df = pd.read_csv(pose_csv_path)
        pose_times = pose_df['timestamp_ns'].values

        with open(offset_json_path, 'r') as f:
            offset_data = json.load(f)

        POSE_OFFSET = np.array([
            offset_data.get('x', 0.0),
            offset_data.get('y', 0.0),
            offset_data.get('z', 0.0)
        ])
        YAW_OFFSET_RAD = np.radians(offset_data.get('yaw_deg', 0.0))

        TIME_OFFSET_NS = 15_500_000_000 if scene_number in hsr_scene_numbers else 0

        rows = []
        for pcd_path, ann_path in zip(pcd_files, ann_files):
            ts = extract_timestamp_from_filename(pcd_path)
            if ts is None:
                continue

            adjusted_ts = ts + TIME_OFFSET_NS
            pose_idx = find_closest_pose_idx(pose_times, adjusted_ts)

            if pose_idx >= len(pose_df):
                continue

            pose_row = pose_df.iloc[pose_idx]
            rx = pose_row['x'] + POSE_OFFSET[0]
            ry = pose_row['y'] + POSE_OFFSET[1]
            r_yaw = pose_row['yaw_rad'] + YAW_OFFSET_RAD

            participant_positions = load_participant_centers(ann_path)
            flat_participants = [coord for pair in participant_positions for coord in pair]
            while len(flat_participants) < 10:
                flat_participants.extend([None, None])

            rows.append([adjusted_ts] + flat_participants + [rx, ry, r_yaw])

        # Save to CSV inside its own output folder
        output_dir = f"{scene_number}_poses"
        os.makedirs(output_dir, exist_ok=True)
        output_path = f"{output_dir}/{scene_number}_robot_and_participants.csv"

        columns = ['Timestamp'] + [f'x{i}' for i in range(1, 6)] + [f'y{i}' for i in range(1, 6)] + ['rx', 'ry', 'r_yaw']
        df_out = pd.DataFrame(rows, columns=columns)
        df_out.to_csv(output_path, index=False)
        print(f"  Saved to {output_path}")

    except Exception as e:
        print(f"  Failed to process scene {scene_number}: {e}")
