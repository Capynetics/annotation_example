# Point Cloud Annotation Viewer

## Overview
This project provides a visualization tool for annotated point cloud data using [Open3D](http://www.open3d.org/). The script loads 3D point clouds (`.pcd` files) along with their corresponding bounding box annotations (stored in `.json` files following the Supervisely format). It allows users to navigate through different annotated point clouds interactively.

## Features
- Loads `.pcd` files and their corresponding `.json` annotations
- Displays point clouds with 3D cuboid bounding boxes
- Allows navigation through multiple annotated files using keyboard controls
- Ensures valid visualization even if some `.pcd` files are empty by adding a dummy point
- Maintains camera view parameters while switching between files

## Dependencies
Ensure you have the following dependencies installed:

```sh
pip install open3d numpy
```

## Usage

### Running the Script
To run the visualization script, use the following command:

```sh
python open_filtered_with_bounding_box.py
```

By default, the script is set to look for point cloud data in:

```
/home/user/annotated/6_annotated/scene_6/pointcloud
```

Modify the `vis_dict()` function call at the end of `open_filtered_with_bounding_box.py` to match the correct directory where your `.pcd` and `.json` files are stored.

### Navigation Controls
- **Right Arrow (→)**: Move to the next point cloud file
- **Left Arrow (←)**: Move to the previous point cloud file
- **Space Bar**: Exit the visualization

## Directory Structure
Ensure your dataset follows this structure:
```
annotation_example/
│── open_filtered_with_bounding_box.py
│── README.md
│── 6_annotated/
│   ├── scene_6/
│   │   ├── pointcloud/
│   │   │   ├── file_1.pcd
│   │   │   ├── file_2.pcd
│   │   ├── ann/
│   │   │   ├── file_1.json
│   │   │   ├── file_2.json
│── meta.json
│── key_id_map.json
```

## How It Works
1. The script searches for `.pcd` files in the `pointcloud` directory.
2. It looks for corresponding `.json` annotation files in the `ann` directory.
3. The point clouds and their bounding boxes are visualized using Open3D.
4. The user can navigate through the dataset using keyboard shortcuts.

## Annotation Format
The script supports annotation files in the [Supervisely](https://supervisely.com/) format. The bounding boxes are extracted from the `figures` field in the JSON files, where:
- The `geometryType` should be `cuboid_3d`
- The bounding box includes `position`, `dimensions`, and `rotation` values

Example JSON annotation:
```json
{
  "figures": [
    {
      "geometryType": "cuboid_3d",
      "geometry": {
        "position": {"x": 1.0, "y": 2.0, "z": 3.0},
        "dimensions": {"x": 1.5, "y": 2.0, "z": 1.0},
        "rotation": {"x": 0.0, "y": 1.57, "z": 0.0}
      }
    }
  ]
}
```

## License
This project is open-source and can be used freely. Contributions and improvements are welcome!

## Author
Developed by Johnata Brayan.

