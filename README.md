# onboard_detector

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-34C759?style=flat-square&logo=ros)](https://docs.ros.org/)
[![Docker](https://img.shields.io/badge/Docker-Supported-2496ED?style=flat-square&logo=docker)](https://www.docker.com/)

A ROS 2 package for real-time 3D dynamic obstacle detection and tracking on the Jo robot platform. Fuses depth camera, LiDAR, and YOLO 2D detections to produce tracked, classified 3D bounding boxes suitable for downstream navigation and collision avoidance.

---

## Architecture overview

```
                 ┌──────────────┐
                 │  depth camera │  (RGB + depth)
                 └──────┬───────┘
                        │
           ┌────────────▼────────────┐
           │   yolov11_detector_node  │  2D class detections
           └────────────┬────────────┘
                        │ /yolo_detector/detected_bounding_boxes
  ┌─────────────────────▼───────────────────────────────────┐
  │                    detector_node                         │
  │                                                         │
  │  depth → UV boxes ─┐                                    │
  │  depth → DBSCAN ───┼──► fuse ──► YOLO assoc ──► KF     │
  │  lidar → clusters ─┘            classify                │
  └──────────────────────────────────────────────────────────┘
           ▲                  ▲
           │ ground/walls     │ odometry
  ┌────────┴────────┐  ┌──────┴──────────────┐
  │ wall_detector   │  │ odometry_tf_publisher│
  │    _node        │  │    _node (optional)  │
  └─────────────────┘  └──────────────────────┘
           ▲
  ┌────────┴────────┐
  │calibration_icp  │  publishes camera_refined TF
  │    _node        │  (falls back to initial guess
  └─────────────────┘   after 3 s in outdoor mode)
```

See [docs/framework_architecture.md](docs/framework_architecture.md) for a detailed description of each processing stage.

---

## Nodes

### `detector_node`

The main perception node. Runs the full detection, tracking, and classification pipeline.

**Subscribed topics**

| Topic | Type | Description |
|---|---|---|
| `depth_image_topic` | `sensor_msgs/Image` | 16-bit depth image |
| `color_image_topic` | `sensor_msgs/Image` | RGB color image |
| `lidar_pointcloud_topic` | `sensor_msgs/PointCloud2` | 3D LiDAR scan |
| `odom_topic` / `pose_topic` | `nav_msgs/Odometry` or `geometry_msgs/PoseStamped` | Robot localization |
| `yolo_detector/detected_bounding_boxes` | `vision_msgs/Detection2DArray` | 2D YOLO detections |
| `/onboard_detector/ground_height` | `std_msgs/Float64` | Ground height from wall_detector |

**Published topics**

| Topic | Type | Description |
|---|---|---|
| `/onboard_detector/filtered_bboxes` | `visualization_msgs/MarkerArray` | All tracked 3D boxes |
| `/onboard_detector/dynamic_bboxes` | `visualization_msgs/MarkerArray` | Confirmed dynamic obstacles (red) |
| `/onboard_detector/potentially_dynamic_bboxes` | `visualization_msgs/MarkerArray` | YOLO-identified but near-stationary (green) |
| `/onboard_detector/dynamic_obstacles` | `onboard_detector/DynamicObstacleArray` | Dynamic obstacles as typed message |
| `/onboard_detector/predicted_bboxes` | `visualization_msgs/MarkerArray` | KF-predicted positions |
| `/yolo_points` | `sensor_msgs/PointCloud2` | Depth points inside YOLO detections (debug) |
| `/filtered_before_yolo_bboxes` | `visualization_msgs/MarkerArray` | 3D boxes before YOLO association (debug) |

---

### `wall_detector_node`

Runs RANSAC on the LiDAR scan to detect vertical planes (walls) and estimates the ground height from the depth image. Results are sent to `detector_node` to update its ground/roof bounds dynamically.

**Subscribed topics:** LiDAR point cloud, depth image, odometry TF

**Published topics:** ground height, wall OBB registry, LiDAR/depth debug clouds

---

### `calibration_icp_node`

One-shot extrinsic calibration between LiDAR and depth camera. Collects N synchronized scene pairs, runs point-to-plane ICP on each, averages valid results, and publishes the refined transform as a static TF.

**Outdoor mode (`is_indoor: false`):** if ICP does not complete within **3 seconds**, the node falls back to the initial-guess TF so detection can start immediately.

**Published TF:** `velodyne` → `camera_refined`

---

### `yolov11_detector_node.py`

Runs YOLOv11 inference on the color image and publishes 2D bounding boxes with class labels. Targets a configurable set of dynamic classes (person, car, bus, truck, …).

---

### `odometry_tf_publisher_node` _(optional)_

Republishes `/odometry/filtered` as a `odom` → `base_link` TF transform. Only needed when the EKF node (from `jo_navigation`) is not running alongside the detector.

Launch with `odom_pub:=true`.

---

## Launch

```bash
ros2 launch onboard_detector run_detector.launch.py
```

Optional arguments:

| Argument | Default | Description |
|---|---|---|
| `odom_pub` | `false` | Start the odometry TF publisher |

---

## Configuration

Parameters are loaded from a single YAML file. Two profiles are provided:

| File | Use case |
|---|---|
| `cfg/detector_param_jo_zotac_outdoor.yaml` | Outdoor operation — wider LiDAR range, tuned for cars, no depth background filter |
| `cfg/detector_param_jo_zotac_indoor.yaml` | Indoor operation — tighter range, 10th-percentile depth background filter active |

The active config is selected inside `run_detector.launch.py` (edit `config_file`).

### Key parameter groups

| Group | Relevant params |
|---|---|
| Detection ranges | `local_lidar_range`, `local_sensor_range` |
| LiDAR DBSCAN | `lidar_DBSCAN_epsilon`, `lidar_DBSCAN_min_points` |
| Visual DBSCAN | `dbscan_search_range_epsilon`, `dbscan_min_points_cluster` |
| Kalman filter | `kalman_filter_v2_param` [eP, eQPos, eQVel, eQAcc, eRPos] |
| Association | `max_match_range`, `max_match_speed`, scoring weights |
| YOLO matching | `yolo_point_fraction_threshold`, `yolo_2d_iou_threshold`, `yolo_x_y_resize` |
| Classification | `dynamic_velocity_threshold`, `dynamic_voting_threshold` |
| Environment | `is_indoor` (affects depth filtering and ICP timeout) |

---

## Custom messages

### `DynamicObstacle.msg`
Per-obstacle message: 3D position, velocity, bounding box dimensions, class label, YOLO flag.

### `DynamicObstacleArray.msg`
Stamped array of `DynamicObstacle`.

### `GetDynamicObstacles.srv`
Service to query the current dynamic obstacle list on demand.

---

## Building

```bash
# Inside the Docker container
cd ~/src
colcon build --symlink-install --install-base ~/local_install --packages-select onboard_detector
source ~/local_install/setup.bash
```
