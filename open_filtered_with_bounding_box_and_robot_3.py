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

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

    def update_view(vis, new_idx):
        nonlocal idx
        if 0 <= new_idx < len(pcds):
            idx = new_idx
            pcd_file = pcds[idx]
            ts = extract_timestamp_from_filename(pcd_file)
            adjusted_ts = ts + TIME_OFFSET_NS
            pose_idx = find_closest_pose_idx(adjusted_ts)
            pose_row = pose_df.iloc[pose_idx]
            pose_ts = pose_row['timestamp_ns']

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
            # Print participant positions
            if anns and os.path.exists(anns[idx]):
                with open(anns[idx], 'r') as f:
                    ann_data = json.load(f)
                print("Participants' positions (x, y):")
                for figure in ann_data.get("figures", []):
                    if figure.get("geometryType") == "cuboid_3d":
                        pos = figure["geometry"]["position"]
                        print(f"  x={pos['x']:.3f}, y={pos['y']:.3f}")

            # Print robot frame position (with offset and yaw offset)
            base_x, base_y, base_yaw = pose_row['x'], pose_row['y'], pose_row['yaw_rad']
            c, s = np.cos(base_yaw), np.sin(base_yaw)
            offset_rot = [
                c * POSE_OFFSET[0] - s * POSE_OFFSET[1],
                s * POSE_OFFSET[0] + c * POSE_OFFSET[1]
            ]
            robot_frame_x = base_x + offset_rot[0]
            robot_frame_y = base_y + offset_rot[1]
            robot_frame_yaw = base_yaw + YAW_OFFSET_RAD


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
    vis.run()

def save_positions_csv(directory, output_csv):
    pcds = sorted(glob.glob(f'{directory}/*.pcd'))
    anns = sorted(glob.glob(f'{directory.replace("pointcloud", "ann")}/*.json'))

    if not pcds:
        print("No .pcd files found in the directory.")
        return

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

    rows = []
    for idx, pcd_file in enumerate(pcds):
        ts = extract_timestamp_from_filename(pcd_file)
        adjusted_ts = ts + TIME_OFFSET_NS
        pose_idx = find_closest_pose_idx(adjusted_ts)
        pose_row = pose_df.iloc[pose_idx]

        # Robot frame position (with offset and yaw offset) -- FIXED
        base_x, base_y, base_yaw = pose_row['x'], pose_row['y'], pose_row['yaw_rad']

        # Build T
        T = np.eye(4)
        T[:2, :2] = [[np.cos(base_yaw), -np.sin(base_yaw)],
                     [np.sin(base_yaw),  np.cos(base_yaw)]]
        T[:3, 3] = np.array([base_x, base_y, 0.0]) + POSE_OFFSET

        # Build R
        R = np.eye(4)
        R[:2, :2] = [[np.cos(YAW_OFFSET_RAD), -np.sin(YAW_OFFSET_RAD)],
                     [np.sin(YAW_OFFSET_RAD),  np.cos(YAW_OFFSET_RAD)]]

        final_transform = R @ T
        robot_frame_x = final_transform[0, 3]
        robot_frame_y = final_transform[1, 3]
        robot_frame_yaw = np.arctan2(final_transform[1, 0], final_transform[0, 0])

        # Get participant positions
        part_positions = []
        if anns and os.path.exists(anns[idx]):
            with open(anns[idx], 'r') as f:
                ann_data = json.load(f)
            for figure in ann_data.get("figures", []):
                if figure.get("geometryType") == "cuboid_3d":
                    pos = figure["geometry"]["position"]
                    part_positions.extend([pos['x'], pos['y']])
        # Pad to 10 values (for up to 5 participants)
        while len(part_positions) < 10:
            part_positions.append('')

        row = [ts, robot_frame_x, robot_frame_y, robot_frame_yaw] + part_positions
        rows.append(row)

    # Write to CSV
    header = ['timestamp', 'robot_x', 'robot_y', 'robot_yaw_rad',
              'x1', 'y1', 'x2', 'y2', 'x3', 'y3', 'x4', 'y4', 'x5', 'y5']
    df = pd.DataFrame(rows, columns=header)
    df.to_csv(output_csv, index=False)
    print(f"Saved positions to {output_csv}")

if __name__ == '__main__':
    # Uncomment to run the CSV export:
    save_positions_csv(f'{scene_number}_annotated/scene_{scene_number}/pointcloud',
                       f'{scene_number}_annotated/scene_{scene_number}_positions.csv')
    # vis_dict(f'{scene_number}_annotated/scene_{scene_number}/pointcloud')
