
import os
import rosbag
import numpy as np
import tf.transformations as tft
import csv

# List of HSR scene numbers
hsr_scene_numbers = [1, 2, 3, 4, 5, 6, 8, 9, 10, 12, 13, 27, 28, 29, 30, 31, 32, 34, 35, 36, 38, 39]

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

for scene_num in hsr_scene_numbers:
    bag_file = f"{scene_num}_robot.bag"
    bag_path = os.path.join(base_path, bag_file)
    csv_path = os.path.join(output_dir, f"{scene_num}_robot_pose.csv")
    poses = []

    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            tf_data = {}

            for topic, msg, t in bag.read_messages(topics=['/tf', '/tf_static']):
                for transform in msg.transforms:
                    key = (transform.header.frame_id, transform.child_frame_id)
                    tf_data.setdefault(key, []).append(transform)

            for tf_odom_base in tf_data.get(('odom', 'base_footprint'), []):
                stamp = tf_odom_base.header.stamp
                stamp_ns = stamp.secs * 1_000_000_000 + stamp.nsecs

                tf_map_odom_list = tf_data.get(('map', 'odom'), [])
                tf_map_odom = next(
                    (t for t in tf_map_odom_list if abs((t.header.stamp.to_nsec() - stamp_ns)) < 50_000_000),
                    None
                )

                if tf_map_odom is None:
                    continue

                T_map_odom = transform_to_matrix(tf_map_odom)
                T_odom_base = transform_to_matrix(tf_odom_base)
                T_map_base = np.dot(T_map_odom, T_odom_base)

                x = T_map_base[0, 3]
                y = T_map_base[1, 3]
                _, _, yaw = tft.euler_from_matrix(T_map_base)

                poses.append((stamp_ns, x, y, yaw))

        with open(csv_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['timestamp_ns', 'x', 'y', 'yaw_rad'])
            writer.writerows(poses)

        print(f"Saved poses for Scene {scene_num} to {csv_path}")

    except Exception as e:
        print(f"Failed to process Scene {scene_num}: {str(e)}")
