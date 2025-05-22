import os
import shutil

# Define your paths
annotations_src_dir = "/home/jsouzasoar/Downloads/Annotations/"
to_be_annotated_dir = "/home/jsouzasoar/Downloads/To_be_annotated"
output_dir = "/home/jsouzasoar/annotation_example"

# Loop from x = 3 onward
for x in range(27, 28):
    print(f"\n=== Processing index {x} ===")

    # Step 1: Copy x_annotation → x_annotated
    src_annotation = os.path.join(annotations_src_dir, f"{x}_annotation")
    dst_annotation = os.path.join(output_dir, f"{x}_annotated")

    if os.path.exists(src_annotation):
        if not os.path.exists(dst_annotation):
            shutil.copytree(src_annotation, dst_annotation)
            print(f"[✓] Copied {src_annotation} → {dst_annotation}")
        else:
            print(f"[!] Skipped: {dst_annotation} already exists")
            continue
    else:
        print(f"[X] Missing annotation folder: {src_annotation}")
        continue

    # Step 2: Rename ds0 → scene_x
    ds0_path = os.path.join(dst_annotation, "ds0")
    scene_path = os.path.join(dst_annotation, f"scene_{x}")
    if os.path.exists(ds0_path):
        os.rename(ds0_path, scene_path)
        print(f"[✓] Renamed {ds0_path} → {scene_path}")
    else:
        print(f"[X] Missing ds0 folder in {dst_annotation}")
        continue

    # Step 3: Copy x_grs_pcds_filtered → scene_x/pointcloud
    src_pcds_folder = os.path.join(to_be_annotated_dir, f"{x}_grs_pcds_filtered")
    dst_pointcloud_folder = os.path.join(scene_path, "pointcloud")

    if os.path.exists(src_pcds_folder):
        if not os.path.exists(dst_pointcloud_folder):
            shutil.copytree(src_pcds_folder, dst_pointcloud_folder)
            print(f"[✓] Copied {src_pcds_folder} → {dst_pointcloud_folder}")
        else:
            print(f"[!] Skipped: pointcloud folder already exists at {dst_pointcloud_folder}")
    else:
        print(f"[X] Missing PCD folder: {src_pcds_folder}")
