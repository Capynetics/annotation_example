import open3d as o3d
import glob
import numpy as np
import json
import os
import pandas as pd
import bisect


scene_number = 13  # Change this to the desired scene number

hsr_scene_numbers = [1, 2, 3, 4, 5, 6, 8, 9, 10, 12, 13, 27, 28, 29, 30, 31, 32, 34, 35, 36, 38, 39]
TIME_OFFSET_NS = 0

# Adjust time offset for specific scenes
if scene_number in hsr_scene_numbers or scene_number in [46, 52]:
    TIME_OFFSET_NS = 15_500_000_000

def vis_dict(directory):
    pcds = sorted(glob.glob(f'{directory}/*.pcd'))
    anns = sorted(glob.glob(f'{directory.replace("pointcloud", "ann")}/*.json'))

    if not pcds:
        print("No .pcd files found in the directory.")
        return

    print(f"Found {len(pcds)} .pcd files with annotations")

    # Load robot pose CSV
    pose_df = pd.read_csv(f'{scene_number}_annotated/{scene_number}_robot_pose.csv')
    pose_times = pose_df['timestamp_ns'].values

    # Load spatial and yaw offset from JSON
    with open(f'{scene_number}_annotated/{scene_number}_grs_to_bot_offset.json', 'r') as f:
        offset_data = json.load(f)

    POSE_OFFSET = np.array([
        offset_data.get('x', 0.0),
        offset_data.get('y', 0.0),
        offset_data.get('z', 0.0)
    ])
    YAW_OFFSET_RAD = np.radians(offset_data.get('yaw_deg', 0.0))
    
    print(f"\nLoaded offset from JSON:")
    print(f"→ POSE_OFFSET      : {POSE_OFFSET}")
    print(f"→ YAW_OFFSET_RAD   : {YAW_OFFSET_RAD:.3f} rad ({np.degrees(YAW_OFFSET_RAD):.1f}°)\n")

    def extract_timestamp_from_filename(filename):
        ts_str = os.path.basename(filename).rsplit('.', 2)
        if len(ts_str) >= 2:
            full_ts = '.'.join(ts_str[:2])
            return int(float(full_ts) * 1e9)
        return None

    def find_closest_pose_idx(ts):
        i = bisect.bisect_left(pose_times, ts)
        if i > 0 and (i == len(pose_times) or abs(ts - pose_times[i - 1]) < abs(ts - pose_times[i])):
            return i - 1
        return i

    def create_robot_frame(x, y, yaw, offset=np.zeros(3), yaw_offset=0.0, size=0.3):
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
        T = np.eye(4)
        T[:2, :2] = [[np.cos(yaw), -np.sin(yaw)],
                     [np.sin(yaw),  np.cos(yaw)]]
        T[:3, 3] = np.array([x, y, 0.0]) + offset

        R = np.eye(4)
        R[:2, :2] = [[np.cos(yaw_offset), -np.sin(yaw_offset)],
                     [np.sin(yaw_offset),  np.cos(yaw_offset)]]

        final_transform = R @ T
        frame.transform(final_transform)
        return frame

    def set_initial_camera(view_ctrl):
        eye = np.array([5.0, 0.0, 5.0])        # Camera position
        lookat = np.array([5.0, 0.0, 10.0])     # Point to look at (directly below)
        up = np.array([0.0, 1.0, 0.0])         # Y axis is "up" in the image

        front = (lookat - eye)
        front /= np.linalg.norm(front)

        view_ctrl.set_lookat(lookat)
        view_ctrl.set_front(front)
        view_ctrl.set_up(up)
        view_ctrl.set_zoom(0.5)  # Adjust zoom level if needed

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
    idx = 0

    set_initial_camera(vis.get_view_control())  # Set camera view

    def load_pcd(filepath):
        pcd = o3d.io.read_point_cloud(filepath)
        points = np.asarray(pcd.points)
        if points.size == 0:
            print(f"Warning: Empty point cloud in {filepath}, adding a dummy point.")
            points = np.array([[1.0, 0.0, 1.0]])
        pcd.points = o3d.utility.Vector3dVector(points)
        return pcd

    def load_annotations(json_file):
        if not os.path.exists(json_file):
            return []
        with open(json_file, 'r') as f:
            ann_data = json.load(f)
        cuboids = []
        for figure in ann_data.get("figures", []):
            if figure.get("geometryType") == "cuboid_3d":
                bbox = figure["geometry"]
                center = np.array([bbox["position"]["x"], bbox["position"]["y"], bbox["position"]["z"]])
                print(f"Participant position: x={center[0]:.3f}, y={center[1]:.3f}")
                cuboids.append(draw_bbox(bbox))
        return cuboids

    def draw_bbox(bbox):
        center = np.array([bbox["position"]["x"], bbox["position"]["y"], bbox["position"]["z"]])
        size = np.array([bbox["dimensions"]["x"], bbox["dimensions"]["y"], bbox["dimensions"]["z"]])
        rotation = np.array([bbox["rotation"]["x"], bbox["rotation"]["y"], bbox["rotation"]["z"]])
        bbox_o3d = o3d.geometry.OrientedBoundingBox(center,
                                                    o3d.geometry.get_rotation_matrix_from_xyz(rotation),
                                                    size)
        bbox_o3d.color = (1, 0, 0)
        return bbox_o3d
    
    def collect_all_data_and_save_csv():
        print("Collecting participant and robot positions for all frames...")
        frame_rows = []
        max_participants = 0

        for frame in all_frame_data:
            # Extract timestamp from pcd_file (before .pcd) and remove decimal point
            ts_str = frame["pcd_file"].split('.pcd')[0]
            ts_int_str = ts_str.replace('.', '')
            row = {
                "timestamp": ts_int_str,
                "robot_x": frame["robot_x"],
                "robot_y": frame["robot_y"],
                "robot_yaw_rad": frame["robot_yaw_rad"],
            }
            # Add participant coordinates as x1, y1, x2, y2, ...
            for i, pos in enumerate(frame["participant_positions"]):
                row[f"x{i+1}"] = pos[0]
                row[f"y{i+1}"] = pos[1]
            max_participants = max(max_participants, len(frame["participant_positions"]))
            frame_rows.append(row)

        # Ensure all rows have the same columns
        columns = ["timestamp", "robot_x", "robot_y", "robot_yaw_rad"]
        for i in range(1, max_participants+1):
            columns += [f"x{i}", f"y{i}"]

        df = pd.DataFrame(frame_rows)
        df = df.reindex(columns=columns)
        out_csv = f"{scene_number}_annotated/scene_{scene_number}_participants_and_robot_flat.csv"
        df.to_csv(out_csv, index=False)
        print(f"Saved participant and robot positions to {out_csv}")

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

    all_frame_data = []  # Collect data for all frames

    def update_view(vis, new_idx, collect_data=False):
        nonlocal idx
        if 0 <= new_idx < len(pcds):
            idx = new_idx
            pcd_file = pcds[idx]
            ts = extract_timestamp_from_filename(pcd_file)
            adjusted_ts = ts + TIME_OFFSET_NS
            pose_idx = find_closest_pose_idx(adjusted_ts)
            pose_row = pose_df.iloc[pose_idx]
            pose_ts = pose_row['timestamp_ns']

            if collect_data:
                # Collect participant positions for this frame
                participant_positions = []
                if anns and os.path.exists(anns[idx]):
                    with open(anns[idx], 'r') as f:
                        ann_data = json.load(f)
                    for figure in ann_data.get("figures", []):
                        if figure.get("geometryType") == "cuboid_3d":
                            bbox = figure["geometry"]
                            center = np.array([bbox["position"]["x"], bbox["position"]["y"], bbox["position"]["z"]])
                            participant_positions.append(center)
                all_frame_data.append({
                    "frame_idx": idx,
                    "pcd_file": os.path.basename(pcd_file),
                    "robot_x": pose_row['x'],
                    "robot_y": pose_row['y'],
                    "robot_yaw_rad": pose_row['yaw_rad'],
                    "participant_positions": participant_positions
                })

            print("\n=== Frame Debug Info ===")
            print(f"[{idx+1}/{len(pcds)}] File              : {os.path.basename(pcd_file)}")
            print(f"→ Raw PCD timestamp      : {ts}")
            print(f"→ Adjusted PCD timestamp : {adjusted_ts}")
            print(f"→ Closest pose timestamp : {pose_ts}")
            print(f"→ Time diff              : {(adjusted_ts - pose_ts)/1e6:.2f} ms")
            print(f"→ Pose x                 : {pose_row['x']:.3f}")
            print(f"→ Pose y                 : {pose_row['y']:.3f}")
            print(f"→ Pose yaw_rad           : {pose_row['yaw_rad']:.3f}")
            print(f"→ POSE_OFFSET            : {POSE_OFFSET}")
            print(f"→ YAW_OFFSET_RAD         : {YAW_OFFSET_RAD:.3f} rad ({np.degrees(YAW_OFFSET_RAD):.1f}°)")
            print(f"Robot position: x={pose_row['x']:.3f}, y={pose_row['y']:.3f}, yaw={pose_row['yaw_rad']:.3f} rad")
            print("========================\n")

            ctr = vis.get_view_control()
            cam_params = ctr.convert_to_pinhole_camera_parameters()

            vis.clear_geometries()
            pcd = load_pcd(pcd_file)
            bbox_objs = load_annotations(anns[idx]) if anns else []
            robot_frame = create_robot_frame(
                pose_row['x'],
                pose_row['y'],
                pose_row['yaw_rad'],
                offset=POSE_OFFSET,
                yaw_offset=YAW_OFFSET_RAD
            )

            vis.add_geometry(pcd)
            #vis.add_geometry(axis)
            for bbox in bbox_objs:
                vis.add_geometry(bbox)
            vis.add_geometry(robot_frame)

            vis.update_renderer()
            ctr.convert_from_pinhole_camera_parameters(cam_params)

    def right_click(vis): update_view(vis, idx + 1)
    def left_click(vis): update_view(vis, idx - 1)
    def exit_key(vis): print("Exiting visualization."); vis.destroy_window()

    vis.register_key_callback(262, right_click)
    vis.register_key_callback(263, left_click)
    vis.register_key_callback(32, exit_key)

    update_view(vis, idx)
    
    # Collect data for all frames before saving CSV
    for i in range(len(pcds)):
        update_view(vis, i, collect_data=True)
    collect_all_data_and_save_csv()

    vis.run()

if __name__ == '__main__':
    vis_dict(f'{scene_number}_annotated/scene_{scene_number}/pointcloud')
