# LV-DOT Jazzy Workspace

![ROS2](https://img.shields.io/badge/ROS-2%20Jazzy-green) ![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-blue) ![Linux](https://img.shields.io/badge/Platform-Linux-orange) ![License](https://img.shields.io/badge/License-MIT-blue)

Dockerized ROS 2 Jazzy workspace for **LV-DOT**, a LiDAR + depth camera dynamic obstacle detection and tracking system.

Original project: https://github.com/Zhefan-Xu/LV-DOT  
Jazzy fork: https://github.com/ducciopet/Docker_LV-DOT_Jazzy  
Previous Humble version: https://github.com/ducciopet/Docker_LV-DOT

---

## What it does

The system fuses three sensing modalities to detect and track dynamic obstacles in 3D:

- **Depth camera** — dense short-range 3D point cloud extraction and UV-based obstacle detection
- **LiDAR (Velodyne)** — sparse long-range DBSCAN clustering
- **YOLOv11** — 2D human detection used to tag 3D bboxes as dynamic candidates

A separate **wall detector** estimates ground/roof height and detects planar walls, feeding those results back into the main detector to suppress static structure from both the point cloud and the bbox pipeline.

A one-shot **ICP calibration** node refines the LiDAR-to-camera extrinsic transform before detection starts.

For a complete technical description of every stage see [FRAMEWORK_ARCHITECTURE.md](FRAMEWORK_ARCHITECTURE.md).

---

## Repository structure

```
Docker_LV-DOT_Jazzy/
├── compose.yaml
├── compose_build.bash
├── entrypoint.sh
├── DockerFiles/Dockerfile
├── bags/
├── FRAMEWORK_ARCHITECTURE.md
├── README.md
└── src/
    └── onboard_detector/
        ├── cfg/
        │   └── detector_param_jo_zotac.yaml
        ├── include/onboard_detector/
        │   ├── dynamicDetector.cpp / .h
        │   ├── uvDetector.cpp / .h
        │   ├── lidarDetector.cpp / .h
        │   ├── dbscan.cpp / .h
        │   ├── kalmanFilter.cpp / .h
        │   └── utils.h
        ├── launch/
        │   ├── run_detector.launch.py
        │   └── run_calibration_icp.launch.py
        ├── rviz/
        ├── scripts/
        │   └── yolo_detector/
        │       ├── yolov11_detector_node.py
        │       ├── yolov11_detector.py
        │       ├── weights/
        │       └── config/
        ├── src/
        │   ├── detector_node.cpp
        │   ├── calibration_icp_node.cpp
        │   └── wall_detector_node.cpp
        └── srv/
            └── GetDynamicObstacles.srv
```

---

## Runtime nodes

| Node | Language | Role |
|------|----------|------|
| `calibration_icp_node` | C++ | One-shot LiDAR-to-camera extrinsic refinement via ICP |
| `wall_detector_node` | C++ | Ground/roof height estimation + planar wall OBB detection |
| `detector_node` | C++ | Main fusion, tracking, and dynamic classification |
| `yolov11_detector_node.py` | Python | 2D human detection with YOLOv11 |
| `rviz2` | — | Visualization |

---

## TF chain

```
map
└── base_link
    └── imu_link
        └── velodyne
            ├── camera_initial_guess    (static, published at launch)
            └── camera_refined          (static, published by calibration_icp_node)
```

The main detector and the wall detector both use `camera_refined`. The detector gates all processing on a successful TF lookup for `velodyne → camera_refined`.

---

## Key input topics

| Topic | Consumer |
|-------|----------|
| `/velodyne_points` | detector, wall\_detector, calibration |
| `/front_camera/camera/depth/image_rect_raw` | detector, wall\_detector, calibration |
| color image topic (configured in YAML) | detector, YOLO |
| `/clock` (bag playback) | all nodes |

---

## Key output topics

| Topic | Description |
|-------|-------------|
| `/onboard_detector/tracked_bboxes` | Confirmed tracked bboxes (yellow in RViz) |
| `/onboard_detector/dynamic_bboxes` | Bboxes classified as dynamic (blue) |
| `/onboard_detector/filtered_bboxes` | Post-fusion bboxes before tracking |
| `/onboard_detector/filtered_point_cloud` | Fused obstacle points |
| `/onboard_detector/dynamic_point_cloud` | Points belonging to dynamic bboxes |
| `/onboard_detector/history_trajectories` | Track history lines |
| `/onboard_detector/velocity_visualizaton` | Velocity arrows |
| `/onboard_detector/yolo_points` | 3D foreground points extracted from YOLO ROI |
| `/wall_detector/wall_markers` | Wall oriented bounding boxes |
| `/wall_detector/ground_height` | `[ground_z, roof_z, offset]` |
| `yolo_detector/detected_bounding_boxes` | YOLO 2D detections |

---

## Service

```
onboard_detector/get_dynamic_obstacles
  request:  current_position (Vector3), range (float)
  response: position[], velocity[], size[]
```

Returns dynamic obstacles within `range` metres, sorted by distance.

---

## Setup and build

```bash
git clone https://github.com/ducciopet/Docker_LV-DOT_Jazzy.git
cd Docker_LV-DOT_Jazzy

# Build Docker image
./compose_build.bash

# Start container
docker compose up -d
docker compose exec ros2_jazzy_sim bash

# Build workspace inside container
cd ~/ros2_ws
colcon build --packages-select onboard_detector
source install/setup.bash
```

---

## Running

```bash
# Full runtime
ros2 launch onboard_detector run_detector.launch.py

# Play a bag (in a separate terminal)
ros2 bag play <bag_path> -s mcap --clock

# Calibration debug only
ros2 launch onboard_detector run_calibration_icp.launch.py
```

---

## Launch sequence (`run_detector.launch.py`)

1. Sets `PYTHONPATH` for YOLO scripts
2. Publishes static TF `velodyne → camera_initial_guess` (initial guess for calibration)
3. Publishes static TF `velodyne → camera_refined` (overridden by ICP when it converges)
4. Starts `calibration_icp_node`
5. Starts `detector_node`
6. Starts `wall_detector_node`
7. Starts `yolov11_detector_node.py`
8. Starts `rviz2`

The detector does not process data until `lookupTfMatrix("velodyne", "camera_refined")` succeeds — this flag (`lidarToDepthCamOk_`) is checked at the top of every timer callback.

---

## Troubleshooting

**Detector processes nothing** — `lidarToDepthCamOk_` is false. Check:
```bash
ros2 topic echo /tf_static --qos-durability transient_local
```
The `calibration_icp_node` must receive a synchronized LiDAR + depth pair before it can publish the refined TF.

**Stale static TF publishers from a previous run:**
```bash
pkill -f static_transform_publisher
```

---

## Configuration

All parameters are in `cfg/detector_param_jo_zotac.yaml`. Each parameter is commented. The file has four sections: `detector_node`, `wall_detector_node`, `calibration_icp_node`, `yolov11_detector_node`.

---

## Citation

```bibtex
@article{xu2022lvdot,
  title={LV-DOT: Light-Weight Visual Dynamic Obstacle Tracker},
  author={Xu, Zhefan and others},
  journal={IEEE Robotics and Automation Letters},
  year={2022}
}
```

## License

MIT.
