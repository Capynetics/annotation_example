import open3d as o3d
import glob
import numpy as np
import json
import os

def vis_dict(directory):
    pcds = sorted(glob.glob('{}/*.pcd'.format(directory)))
    anns = sorted(glob.glob('{}/*.json'.format(directory.replace('pointcloud', 'ann'))))

    if not pcds:
        print("No .pcd files found in the directory.")
        return

    print(f"Found {len(pcds)} .pcd files with annotations")

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()

    idx = 0

    def load_pcd(filepath):
        """Loads a PCD file and ensures it has at least one point."""
        pcd = o3d.io.read_point_cloud(filepath)
        points = np.asarray(pcd.points)

        if points.size == 0:
            print(f"Warning: Empty point cloud in {filepath}, adding a dummy point.")
            points = np.array([[1.0, 0.0, 1.0]])  # Dummy point to avoid empty cloud
        
        pcd.points = o3d.utility.Vector3dVector(points)
        return pcd

    def load_annotations(json_file):
        """Loads Supervisely annotations and extracts cuboid bounding boxes."""
        if not os.path.exists(json_file):
            return []

        with open(json_file, 'r') as f:
            ann_data = json.load(f)

        cuboids = []
        
        # Iterate over "figures" where the geometric annotations are stored
        for figure in ann_data.get("figures", []):
            if figure.get("geometryType") == "cuboid_3d":
                bbox = figure["geometry"]
                cuboids.append(draw_bbox(bbox))
        
        return cuboids
        
        return cuboids

    def draw_bbox(bbox):
        """Creates an Open3D wireframe box from a Supervisely bounding box."""
        center = np.array([bbox["position"]["x"], bbox["position"]["y"], bbox["position"]["z"]])
        size = np.array([bbox["dimensions"]["x"], bbox["dimensions"]["y"], bbox["dimensions"]["z"]])
        rotation = np.array([bbox["rotation"]["x"], bbox["rotation"]["y"], bbox["rotation"]["z"]])

        # Create a wireframe bounding box
        bbox_o3d = o3d.geometry.OrientedBoundingBox(center, o3d.geometry.get_rotation_matrix_from_xyz(rotation), size)
        bbox_o3d.color = (1, 0, 0)  # Red color for visibility
        
        return bbox_o3d

    pcd = load_pcd(pcds[idx])
    bbox_objs = load_annotations(anns[idx]) if anns else []
    
    # Create XYZ reference frame (size=0.5)
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

    vis.add_geometry(pcd)
    vis.add_geometry(axis)
    for bbox in bbox_objs:
        vis.add_geometry(bbox)

    def update_view(vis, new_idx):
        nonlocal idx
        if 0 <= new_idx < len(pcds):
            print(f"Displaying: {pcds[new_idx]}")
            idx = new_idx

            # Save current camera parameters
            ctr = vis.get_view_control()
            cam_params = ctr.convert_to_pinhole_camera_parameters()

            # Clear and reload geometries
            vis.clear_geometries()
            pcd = load_pcd(pcds[idx])
            bbox_objs = load_annotations(anns[idx]) if anns else []
            
            vis.add_geometry(pcd)
            vis.add_geometry(axis)
            for bbox in bbox_objs:
                vis.add_geometry(bbox)

            # Restore previous camera parameters
            vis.update_renderer()
            ctr.convert_from_pinhole_camera_parameters(cam_params)

    def right_click(vis):
        update_view(vis, idx + 1)

    def left_click(vis):
        update_view(vis, idx - 1)

    def exit_key(vis):
        print("Exiting visualization.")
        vis.destroy_window()

    vis.register_key_callback(262, right_click)  # Right arrow key
    vis.register_key_callback(263, left_click)   # Left arrow key
    vis.register_key_callback(32, exit_key)      # Space bar

    vis.run()

if __name__ == '__main__':
    vis_dict('2_annotated/scene_2/pointcloud')  # Adjust path accordingly
