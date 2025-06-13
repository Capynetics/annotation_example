import os
import glob
import json
import numpy as np
import pandas as pd
import bisect

# Scene numbers needing time offset
hsr_scene_numbers = [1, 2, 3, 4, 5, 6, 8, 9, 10, 12, 13, 27, 28, 29, 30, 31, 32, 34, 35, 36, 38, 39, 46, 52]

def extract_timestamp_from_filename(filename):
    ts_str = os.path.basename(filename).rsplit('.', 2)
    if len(ts_str) >= 2:
        try:
            return int(float('.'.join(ts_str[:2])) * 1e9)
        except ValueError:
            return None
    return None

def find_closest_pose_idx(ts, pose_times):
    i = bisect.bisect_left(pose_times, ts)
    if i > 0 and (i == len(pose_times) or abs(ts - pose_times[i - 1]) < abs(ts - pose_times[i])):
        return i - 1
    return i

def process_scene(scene_number):
    annotated_folder = f"{scene_number}_annotated"
    poses_folder = f"{scene_number}_poses"
    output_csv = os.path.join(poses_folder, f"{scene_number}_robot_and_participants.csv")

    pointcloud_dir = os.path.join(annotated_folder, f"scene_{scene_number}", "pointcloud")
    ann_dir = os.path.join(annotated_folder, f"scene_{scene_number}", "ann")
    pose_csv_path = os.path.join(annotated_folder, f"{scene_number}_robot_pose.csv")
    offset_json_path = os.path.join(annotated_folder, f"{scene_number}_grs_to_bot_offset.json")

    if not all([os.path.exists(p) for p in [pointcloud_dir, ann_dir, pose_csv_path, offset_json_path]]):
        print(f"[!] Skipping scene {scene_number}: missing required files.")
        return

    TIME_OFFSET_NS = 15_500_000_000 if scene_number in hsr_scene_numbers else 0

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

    pcds = sorted(glob.glob(os.path.join(pointcloud_dir, '*.pcd')))
    anns = sorted(glob.glob(os.path.join(ann_dir, '*.json')))

    rows = []
    for idx, pcd_file in enumerate(pcds):
        ts = extract_timestamp_from_filename(pcd_file)
        if ts is None:
            continue
        adjusted_ts = ts + TIME_OFFSET_NS
        pose_idx = find_closest_pose_idx(adjusted_ts, pose_times)
        pose_row = pose_df.iloc[pose_idx]

        # Transformation: T and R
        base_x, base_y, base_yaw = pose_row['x'], pose_row['y'], pose_row['yaw_rad']
        T = np.eye(4)
        T[:2, :2] = [[np.cos(base_yaw), -np.sin(base_yaw)],
                     [np.sin(base_yaw),  np.cos(base_yaw)]]
        T[:3, 3] = np.array([base_x, base_y, 0.0]) + POSE_OFFSET

        R = np.eye(4)
        R[:2, :2] = [[np.cos(YAW_OFFSET_RAD), -np.sin(YAW_OFFSET_RAD)],
                     [np.sin(YAW_OFFSET_RAD),  np.cos(YAW_OFFSET_RAD)]]

        final_transform = R @ T
        robot_frame_x = final_transform[0, 3]
        robot_frame_y = final_transform[1, 3]
        robot_frame_yaw = np.arctan2(final_transform[1, 0], final_transform[0, 0])

        # Participants
        part_positions = []
        if idx < len(anns) and os.path.exists(anns[idx]):
            with open(anns[idx], 'r') as f:
                ann_data = json.load(f)
            for figure in ann_data.get("figures", []):
                if figure.get("geometryType") == "cuboid_3d":
                    pos = figure["geometry"]["position"]
                    part_positions.extend([pos['x'], pos['y']])
        while len(part_positions) < 10:
            part_positions.append('')

        row = [ts, robot_frame_x, robot_frame_y, robot_frame_yaw] + part_positions
        rows.append(row)

    if not os.path.exists(poses_folder):
        os.makedirs(poses_folder)

    header = ['timestamp', 'robot_x', 'robot_y', 'robot_yaw_rad',
              'x1', 'y1', 'x2', 'y2', 'x3', 'y3', 'x4', 'y4', 'x5', 'y5']
    df = pd.DataFrame(rows, columns=header)
    df.to_csv(output_csv, index=False)
    print(f"[✓] Scene {scene_number}: saved {len(rows)} frames to {output_csv}")

if __name__ == '__main__':
    # Loop through all folders ending in _annotated
    folders = sorted([d for d in os.listdir('.') if d.endswith('_annotated')])
    for folder in folders:
        try:
            scene_number = int(folder.split('_')[0])
            process_scene(scene_number)
        except Exception as e:
            print(f"[!] Failed processing {folder}: {e}")
