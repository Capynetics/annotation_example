
import os
import rosbag
import numpy as np
import tf.transformations as tft
import csv
from tqdm import tqdm

# List of HSR scene numbers
hsr_scene_numbers = [15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
                     41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52]

# Base directory where bag files are located
base_path = "/media/jsouzasoar/EXTERNAL_USB"
output_dir = os.path.join(base_path, "hsr_robot_poses")
os.makedirs(output_dir, exist_ok=True)

def transform_to_matrix(transform):
    trans = transform.transform.translation
    rot = transform.transform.rotation
    trans_vec = [trans.x, trans.y, trans.z]
    quat = [rot.x, rot.y, rot.z, rot.w]
    mat = tft.quaternion_matrix(quat)
    mat[0:3, 3] = trans_vec
    return mat

for scene_num in tqdm(hsr_scene_numbers, desc="Processing HSR Scenes"):
    bag_file = f"{scene_num}_robot.bag"
    bag_path = os.path.join(base_path, bag_file)
    csv_path = os.path.join(output_dir, f"{scene_num}_robot_pose.csv")
    poses = []

    print(f"\n--- Scene {scene_num} ---")
    print(f"Reading: {bag_path}")

    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            tf_data = {}

            for topic, msg, t in bag.read_messages(topics=['/tf', '/tf_static']):
                for transform in msg.transforms:
                    key = (transform.header.frame_id, transform.child_frame_id)
                    tf_data.setdefault(key, []).append(transform)

        if ('odom', 'base_link') not in tf_data or ('map', 'odom') not in tf_data:
            print("Missing required TF frames. Skipping this scene.")
            continue

        for tf_odom_base in tf_data.get(('odom', 'base_link'), []):
            stamp = tf_odom_base.header.stamp
            stamp_ns = stamp.secs * 1_000_000_000 + stamp.nsecs

            tf_map_odom_list = tf_data.get(('map', 'odom'), [])
            tf_map_odom = next(
                (t for t in tf_map_odom_list if abs((t.header.stamp.to_nsec() - stamp_ns)) < 50_000_000),
                None
            )

            if tf_map_odom is None:
                print(f"Missing map->odom at {stamp_ns}")
                continue

            T_map_odom = transform_to_matrix(tf_map_odom)
            T_odom_base = transform_to_matrix(tf_odom_base)
            T_map_base = np.dot(T_map_odom, T_odom_base)

            x = T_map_base[0, 3]
            y = T_map_base[1, 3]
            _, _, yaw = tft.euler_from_matrix(T_map_base)

            poses.append((stamp_ns, x, y, yaw))

        print(f"Extracted {len(poses)} poses.")

        with open(csv_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['timestamp_ns', 'x', 'y', 'yaw_rad'])
            writer.writerows(poses)

        print(f"Saved to {csv_path}")

    except Exception as e:
        print(f"Error processing scene {scene_num}: {str(e)}")
