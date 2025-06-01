# Point Cloud + Robot Pose Viewer

A lightweight visualization tool that overlays annotated 3-D point-clouds **and** the mobile robot’s coordinate frame in real-time, powered by [Open3D](http://www.open3d.org/).

---

## ✨ What’s New?

* **Robot pose overlay**
  Reads `<scene>_robot_pose.csv` and visualises the robot’s coordinate frame for each point cloud.
* **Scene-specific time alignment**
  Optional `TIME_OFFSET_NS` (15.5 s for HSR & selected scenes) guarantees point-cloud ↔️ pose synchronisation.
* **Spatial & yaw offsets**
  Reads `<scene>_grs_to_bot_offset.json` so you can compensate for GNSS↔️base-link offsets without touching code.
* **Script renamed**
  The main entry point is now **`open_filtered_with_bounding_box_and_robot.py`**.
* **Additional dependencies**
  `pandas` for fast timestamp search.

---

## Features

* Load `.pcd` point-clouds and matching Supervisely-style `.json` annotations
* Draw 3-D cuboid bounding boxes
* Overlay a robot coordinate frame at the closest‐in‐time pose
* Keyboard navigation (← / → to step, Space to quit)
* Automatic dummy point insertion for empty `.pcd` files
* Camera viewpoint persists while you browse

---

## Installation

```bash
pip install open3d numpy pandas
```

---

## Directory Layout

```
<project_root>/
│── open_filtered_with_bounding_box_and_robot.py
│── README.md
│── 52_annotated/
│   ├── scene_52/
│   │   ├── pointcloud/
│   │   │   ├── 1717330243.123456789.pcd
│   │   │   └── 1717330244.123456789.pcd
│   │   └── ann/
│   │       ├── 1717330243.123456789.json
│   │       └── 1717330244.123456789.json
│   ├── 52_robot_pose.csv
│   └── 52_grs_to_bot_offset.json
└── meta.json
└── key_id_map.json
```

**Filename rule:** `*.pcd`, `*.json`, and the robot-pose row share the same nanosecond timestamp in their filenames / CSV to ensure correct pairing.

---

## CSV & JSON Formats

### `52_robot_pose.csv`

|       timestamp\_ns |  x (m) |  y (m) | yaw\_rad |
| ------------------: | -----: | -----: | -------: |
| 1717330243123456789 | 12.345 | -4.210 |     1.57 |

### `52_grs_to_bot_offset.json`

```json
{
  "x": 0.045,
  "y": -0.030,
  "z": 0.000,
  "yaw_deg": -1.5
}
```

The offset is applied **after** the pose, letting you shift from GNSS / GRS frame to base-link.

---

## Usage

### 1. Pick a scene

Open the script and change the `scene_number` at the top.

```python
scene_number = 52  # ← set your scene here
```

A few scenes use a hard-coded 15.5 s offset; they’re listed in `hsr_scene_numbers`.

### 2. Run

```bash
python open_filtered_with_bounding_box_and_robot.py
```

The script automatically looks in
`<scene>_annotated/scene_<scene>/pointcloud`

> **Need a different folder?**
> Call `vis_dict("my/custom/pointcloud")` manually at the bottom.

### 3. Navigation

| Key   | Action         |
| ----- | -------------- |
| →     | Next frame     |
| ←     | Previous frame |
| Space | Quit viewer    |

---

## Annotation Format (recap)

Each bounding box lives in `figures[*].geometry`. Required fields:

```json
{
  "position":   {"x": 1.0, "y": 2.0, "z": 3.0},
  "dimensions": {"x": 1.5, "y": 2.0, "z": 1.0},
  "rotation":   {"x": 0.0, "y": 1.57, "z": 0.0}
}
```

*Rotation is in radians using XYZ-Euler convention.*

---

## Troubleshooting

| Symptom                       | Fix                                                                                           |
| ----------------------------- | --------------------------------------------------------------------------------------------- |
| Black window / no points      | Verify `.pcd` is non-empty; dummy points will trigger a console warning.                      |
| Robot frame drifts from boxes | Check timestamps in filenames vs CSV. Adjust `TIME_OFFSET_NS` if your sensor logs are skewed. |
| Pose frame flipped            | Correct `yaw_deg` in `<scene>_grs_to_bot_offset.json`.                                        |

---

## License

MIT. PRs welcome! 🙌

---

## Author

Developed by **Johnata Brayan**.
