# LV-DOT Jazzy Workspace

![ROS2](https://img.shields.io/badge/ROS-2%20Jazzy-green) ![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-blue) ![Linux](https://img.shields.io/badge/Platform-Linux-orange) ![License](https://img.shields.io/badge/License-MIT-blue)

This repository contains a Dockerized ROS 2 Jazzy workspace for LV-DOT, a lightweight visual and LiDAR-based dynamic obstacle detection and tracking framework.

Original project:
- https://github.com/Zhefan-Xu/LV-DOT

Repository variants:
- Jazzy version: https://github.com/ducciopet/Docker_LV-DOT_Jazzy
- Previous Humble version: https://github.com/ducciopet/Docker_LV-DOT

## Overview

The workspace combines:
- a C++ detector pipeline for depth, LiDAR, fusion, and tracking
- a Python YOLO node for 2D detections
- an ICP-based calibration node for refining `velodyne -> rs1_link_refined`
- RViz configurations for runtime and debugging
- Docker support for a reproducible ROS 2 Jazzy environment

The main runtime idea is:
1. publish the base TF chain
2. run ICP calibration to obtain `velodyne -> rs1_link_refined`
3. start the main detector only after the refined transform exists
4. fuse depth, LiDAR, and YOLO detections for dynamic obstacle tracking

---

## Top-level repository structure

```text
Docker_LV-DOT_Jazzy/
├── compose.yaml
├── compose_build.bash
├── entrypoint.sh
├── DockerFiles/
│   └── Dockerfile
├── bags/
├── media/
├── FRAMEWORK_ARCHITECTURE.md
├── README.md
└── src/
    ├── CMakeLists.txt
    └── onboard_detector/
```

### Top-level files

- `compose.yaml`  
  Docker Compose definition for the Jazzy runtime container.

- `compose_build.bash`  
  Convenience script to build the Docker image.

- `entrypoint.sh`  
  Container startup script.

- `DockerFiles/Dockerfile`  
  Base image and system dependencies.

- `bags/`  
  Local ROS 2 bag storage used during testing.

- `FRAMEWORK_ARCHITECTURE.md`  
  Detailed notes about the detection, fusion, and tracking pipeline.

---

## ROS package structure

The main package is `src/onboard_detector`.

```text
src/onboard_detector/
├── CMakeLists.txt
├── package.xml
├── cfg/
├── include/onboard_detector/
├── launch/
├── rviz/
├── scripts/
├── src/
└── srv/
```

### `cfg/`

Configuration files for runtime parameters.

Main files:
- `detector_param_jo_zotac.yaml` — main active Jazzy config
- `detector_param.yaml` — generic/default config

`detector_param_jo_zotac.yaml` contains two important parameter groups:
- `detector_node: ros__parameters:`
- `calibration_icp_node: ros__parameters:`

These parameters control:
- topic names
- TF frame names
- camera intrinsics
- ICP thresholds
- clustering thresholds
- tracking parameters

### `include/onboard_detector/`

Core detector implementation:

- `dynamicDetector.cpp` / `dynamicDetector.h`  
  Main fusion and tracking logic.

- `uvDetector.cpp` / `uvDetector.h`  
  UV/depth based detection path.

- `lidarDetector.cpp` / `lidarDetector.h`  
  LiDAR clustering and box extraction.

- `dbscan.cpp` / `dbscan.h`  
  DBSCAN implementation.

- `kalmanFilter.cpp` / `kalmanFilter.h`  
  Tracking filter logic.

- `utils.h`  
  Shared helpers.

### `src/`

ROS executables:

- `detector_node.cpp`  
  Main detector executable.

- `calibration_icp_node.cpp`  
  One-shot ICP calibration executable. It computes the refined transform and publishes calibration debug clouds.

- `calibration_node.cpp`  
  Legacy / alternate calibration executable retained in the package.

### `scripts/`

Python code and helper scripts.

Important contents:
- `create_bag_with_pose.py`
- `yolo_detector/`

Inside `scripts/yolo_detector/`:
- `yolov11_detector_node.py` — ROS 2 node wrapper for YOLOv11
- `yolov11_detector.py` — main inference logic
- `yolo_detector_node.py`, `yolo_detector.py` — older YOLO code paths
- `config/` — labels and model config
- `module/` — helper modules and custom layers
- `weights/` — local model weights

### `launch/`

Main launch files:
- `run_detector.launch.py` — full runtime launch
- `run_calibration_icp.launch.py` — calibration-only launch with RViz
- `run_calibration.launch.py` — older calibration launch flow

### `rviz/`

RViz presets for runtime and debugging:
- `detector_working_jo_zotac.rviz`
- `detector_working.rviz`
- `detector_debug.rviz`
- `detector.rviz`

### `srv/`

- `GetDynamicObstacles.srv`  
  Service definition exported by the package.

---

## Build system structure

### `CMakeLists.txt`

The package builds:
- the shared detector library
- `detector_node`
- `calibration_node`
- `calibration_icp_node`
- the `GetDynamicObstacles.srv` interface

Main external dependencies include:
- `rclcpp`
- `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `visualization_msgs`
- `message_filters`
- `cv_bridge`
- `PCL`
- `Open3D`
- `tf2`, `tf2_geometry_msgs`

### `package.xml`

Declares the package metadata and the ROS dependency list used by the executables and the generated service interface.

---

## Runtime architecture

### Calibration path

`calibration_icp_node`:
- subscribes to synchronized LiDAR and depth streams
- reads the initial TF guess from `velodyne -> rs1_link`
- runs one-shot ICP refinement
- publishes:
  - `/calibration/velodyne_points`
  - `/calibration/depth_cloud_in_velodyne`
  - static transform `velodyne -> rs1_link_refined`

### Detector path

`detector_node`:
- loads parameters from `detector_param_jo_zotac.yaml`
- uses refined camera/depth frames (`rs1_link_refined`)
- combines depth, LiDAR, and YOLO outputs
- performs association and tracking

### YOLO path

`yolov11_detector_node.py`:
- runs 2D human detection
- publishes bounding boxes and debug image topics

---

## Current launch behavior

### `run_detector.launch.py`

This is the main runtime launch.

It currently:
1. sets `PYTHONPATH` for the YOLO scripts
2. publishes static TFs:
   - `map -> base_link`
   - `base_link -> imu_link`
   - `imu_link -> velodyne`
   - `velodyne -> rs1_link`
3. starts `calibration_icp_node`
4. starts `yolov11_detector_node.py`
5. starts `rviz2`
6. waits for `velodyne -> rs1_link_refined`
7. starts `detector_node` only after the refined transform is available

This avoids starting the detector before the refined camera frame exists.

### `run_calibration_icp.launch.py`

This launch is intended for calibration debugging.

It starts:
- `calibration_icp_node`
- `rviz2`
- the initial static transform `velodyne -> rs1_link`

All calibration parameters are loaded from `detector_param_jo_zotac.yaml`.

---

## Frame structure

Important frames in this workspace:
- `map`
- `base_link`
- `imu_link`
- `velodyne`
- `rs1_link`
- `rs1_link_refined`

### Intended frame logic

- `rs1_link` is the initial camera frame used by the initial guess transform
- `rs1_link_refined` is produced by `calibration_icp_node`
- detector-side depth and color TF parameters are configured to use `rs1_link_refined`

---

## Setup and build

### Clone the repository

```bash
git clone https://github.com/ducciopet/Docker_LV-DOT_Jazzy.git
cd Docker_LV-DOT_Jazzy
```

### Build the Docker image

```bash
./compose_build.bash
```

or:

```bash
docker compose build
```

### Start the container

```bash
docker compose up -d
```

### Enter the container

```bash
docker compose exec ros2_jazzy_sim bash
```

### Build the workspace inside the container

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Main commands

### Full runtime

```bash
ros2 launch onboard_detector run_detector.launch.py
```

### Calibration-only debug

```bash
ros2 launch onboard_detector run_calibration_icp.launch.py
```

### Bag playback

Use simulated time when replaying data:

```bash
ros2 bag play <bag_path> -s mcap --clock
```

---

## Required input topics

Typical required runtime inputs include:
- `/velodyne_points`
- `/camera1/rs1/depth/image_rect_raw`
- the configured color image topic used by YOLO
- `/clock` during bag playback

If your topic names differ, update `detector_param_jo_zotac.yaml`.

---

## Important outputs

### Calibration outputs

- `/calibration/velodyne_points`
- `/calibration/depth_cloud_in_velodyne`
- refined static TF `velodyne -> rs1_link_refined`

### Detector outputs

The detector publishes obstacle and tracking results, including bounding boxes, debug images, and point-cloud outputs depending on configuration.

### YOLO outputs

The YOLO node publishes 2D detections, debug images, and timing-related topics.

---

## Troubleshooting

### `detector_node` does not start

`run_detector.launch.py` waits for `velodyne -> rs1_link_refined`.

If `detector_node` does not appear, verify:
- `calibration_icp_node` is running
- synchronized LiDAR and depth data are arriving
- the refined TF was actually published

### Refined TF seems missing

The calibration node publishes the refined transform as a static transform.

Check with:

```bash
ros2 topic echo /tf_static --qos-durability transient_local
```

### Calibration debug clouds are empty

The calibration node only publishes them after it receives synchronized LiDAR and depth data and completes ICP.

### Old TF publishers interfere with startup

If stale `static_transform_publisher` processes are still running, they can interfere with TF-dependent launch logic.

Useful cleanup:

```bash
pkill -f static_transform_publisher
```

---

## Additional documentation

- `FRAMEWORK_ARCHITECTURE.md` contains a more detailed explanation of the internal LV-DOT pipeline.

---

## Citation

If you use LV-DOT academically, please cite the original work.

```bibtex
@article{xu2022lvdot,
  title={LV-DOT: Light-Weight Visual Dynamic Obstacle Tracker},
  author={Xu, Zhefan and others},
  journal={IEEE Robotics and Automation Letters},
  year={2022}
}
```

## License

MIT, following the package metadata in this repository.
