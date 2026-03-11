# LV-DOT Jazzy Architecture

## Overview

This document describes the current runtime architecture of the Jazzy workspace in this repository.

The active system is built around four main runtime pieces:
- `calibration_icp_node`
- `detector_node`
- `yolov11_detector_node.py`
- `rviz2`

The current launch flow is:
1. publish the static TF chain up to `velodyne -> rs1_link`
2. run one-shot ICP calibration
3. publish the refined transform `velodyne -> rs1_link_refined`
4. start `detector_node` only after the refined transform exists
5. run YOLO and RViz in parallel

---

## Workspace structure

```text
Docker_LV-DOT_Jazzy/
├── compose.yaml
├── compose_build.bash
├── entrypoint.sh
├── DockerFiles/
├── bags/
├── FRAMEWORK_ARCHITECTURE.md
├── README.md
└── src/
    └── onboard_detector/
```

Main ROS package:

```text
src/onboard_detector/
├── cfg/
├── include/onboard_detector/
├── launch/
├── rviz/
├── scripts/
├── src/
└── srv/
```

---

## Runtime nodes

## 1. `calibration_icp_node`

### Role

`calibration_icp_node` computes a refined LiDAR-to-camera transform from:
- LiDAR point cloud input
- depth image input
- initial TF guess `velodyne -> rs1_link`

### Current behavior

The current implementation is **one-shot**, not service-based.

That means:
- the node subscribes to synchronized LiDAR and depth messages
- on the first valid synchronized pair, it runs ICP
- after success, it publishes debug outputs and the refined transform
- after success, it does not recalibrate again in the same run

### Inputs

Configured from YAML under `calibration_icp_node: ros__parameters:`:
- `depth_topic`
- `velodyne_topic`
- `lidar_frame`
- `camera_frame`
- `refined_camera_frame`
- ICP parameters
- depth intrinsics and range limits

Typical topic names in this repository:
- `/camera1/rs1/depth/image_rect_raw`
- `/velodyne_points`

### Outputs

Calibration publishes:
- `/calibration/velodyne_points`
- `/calibration/depth_cloud_in_velodyne`
- static TF `velodyne -> rs1_link_refined`

### Important note

The refined TF is published through a `StaticTransformBroadcaster`, so it appears on `/tf_static`, not `/tf`.

---

## 2. `detector_node`

### Role

`detector_node` is the main C++ LV-DOT runtime.

It combines:
- depth-based detection
- LiDAR-based detection
- visual/LiDAR fusion
- YOLO-assisted human association
- tracking and dynamic/static classification

### Internal detector stages

The main logic lives in:
- `include/onboard_detector/dynamicDetector.cpp`
- `include/onboard_detector/dynamicDetector.h`

The detector pipeline is organized around these steps:

1. **Depth / UV detection**
   - obstacle extraction from depth imagery

2. **DBSCAN on depth-derived cloud**
   - visual clustering and 3D box estimation

3. **LiDAR clustering**
   - LiDAR DBSCAN and box extraction

4. **Visual-LiDAR fusion**
   - fusion of visual and LiDAR boxes

5. **YOLO integration**
   - associates 2D human detections with 3D boxes
   - marks human detections as dynamic candidates

6. **Tracking**
   - box association and temporal tracking

7. **Dynamic classification**
   - classifies tracked objects as dynamic/static

8. **Visualization publishing**
   - debug images, markers, and point clouds

### Frame usage

Detector-side depth and color TF parameters are configured to use:
- `tf_depth_frame: rs1_link_refined`
- `tf_color_frame: rs1_link_refined`

So the main detector is intended to run only after calibration has produced `rs1_link_refined`.

---

## 3. `yolov11_detector_node.py`

### Role

This Python node performs YOLO-based 2D detection.

### Location

Main scripts are under:
- `src/onboard_detector/scripts/yolo_detector/`

Important files:
- `yolov11_detector_node.py`
- `yolov11_detector.py`
- `weights/`
- `config/`

### Function in the pipeline

YOLO is asynchronous relative to the C++ detector.

The detector stores the latest YOLO detections and uses them during the fusion stage to:
- improve human detection
- label certain 3D detections as dynamic
- split large fused boxes when multiple humans are visible

---

## 4. `rviz2`

### Role

RViz is used for:
- debugging calibration outputs
- verifying TF trees
- visualizing obstacle markers and point clouds
- checking detector output alignment

Key configs in this repository:
- `rviz/detector_jo_zotac.rviz`
- `rviz/detector_debug.rviz`

---

## Launch architecture

## `run_detector.launch.py`

This is the main runtime launch.

### What it starts immediately

- static TF publisher: `map -> base_link`
- static TF publisher: `base_link -> imu_link`
- static TF publisher: `imu_link -> velodyne`
- static TF publisher: `velodyne -> rs1_link`
- `calibration_icp_node`
- `yolov11_detector_node.py`
- `rviz2`

### Detector gating logic

`detector_node` is not started immediately.

Instead, the launch runs a small TF waiter process that checks for:
- `velodyne -> rs1_link_refined`

If that transform appears within the timeout window:
- `detector_node` is started

If it does not appear:
- the launch shuts down

### Why this is needed

The detector uses `rs1_link_refined` for camera/depth frame references. Starting the detector before calibration would make the camera-side cloud/frame logic inconsistent.

---

## `run_calibration_icp.launch.py`

This launch is intended for calibration-only debugging.

It starts:
- `calibration_icp_node`
- `rviz2`
- the initial static transform `velodyne -> rs1_link`

This is useful when validating:
- aligned calibration clouds
- TF publication
- ICP tuning

---

## TF architecture

The core TF chain is:

```text
map
└── base_link
    └── imu_link
        └── velodyne
            ├── rs1_link
            └── rs1_link_refined
```

### Meaning of the frames

- `rs1_link`  
  Initial camera frame used as the input guess for ICP

- `rs1_link_refined`  
  Refined camera frame produced by calibration

### Important distinction

- `velodyne -> rs1_link` is the initial guess
- `velodyne -> rs1_link_refined` is the calibration result

The detector should use the refined frame.

---

## Configuration architecture

Main config file:

```text
src/onboard_detector/cfg/detector_param_jo_zotac.yaml
```

This file contains two runtime sections:

### `detector_node: ros__parameters:`

Contains detector-specific settings such as:
- detector topics
- map/lidar/depth/color TF frames
- depth intrinsics
- clustering thresholds
- tracking parameters

### `calibration_icp_node: ros__parameters:`

Contains calibration-specific settings such as:
- input topics
- initial and refined frame names
- ICP thresholds
- overlap parameters
- depth intrinsics and filtering parameters

### Design choice

Launch files are intended to load parameters from YAML rather than duplicating topic or TF values inline.

---

## Topics

## Main input topics

Typical runtime inputs include:
- `/velodyne_points`
- `/camera1/rs1/depth/image_rect_raw`
- color image topic used by YOLO
- `/clock` for bag playback with simulated time

## Calibration outputs

- `/calibration/velodyne_points`
- `/calibration/depth_cloud_in_velodyne`
- `/tf_static` entry for `velodyne -> rs1_link_refined`

## Detector outputs

The detector publishes obstacle, tracking, and debug outputs. Exact topic names are determined by the detector implementation and YAML configuration.

## YOLO outputs

The YOLO node publishes 2D detections and debug imagery.

---

## Build architecture

The package build is defined by:
- `src/onboard_detector/CMakeLists.txt`
- `src/onboard_detector/package.xml`

The package builds:
- detector library
- `detector_node`
- `calibration_node`
- `calibration_icp_node`
- generated interface for `GetDynamicObstacles.srv`

The current calibration ICP build does **not** depend on `std_srvs`; the service-based variant was removed from the active code.

---

## Common runtime scenarios

## Scenario 1: Full runtime

1. launch `run_detector.launch.py`
2. play a bag with `--clock`
3. calibration runs automatically on the first valid synchronized data
4. refined TF is published
5. detector starts
6. YOLO and RViz continue running

## Scenario 2: Calibration debugging

1. launch `run_calibration_icp.launch.py`
2. play a bag with synchronized depth + LiDAR
3. observe:
   - calibration debug clouds
   - refined TF publication
   - alignment quality in RViz

---

## Known failure modes

### 1. `detector_node` never starts

Likely causes:
- calibration never received synchronized data
- ICP failed before publishing `rs1_link_refined`
- old TF publishers interfered with the TF tree

### 2. Calibration clouds look aligned but refined TF seems missing

Possible explanation:
- the debug clouds are both published in the LiDAR frame after explicit transformation, so they can appear aligned even if the TF check was done incorrectly
- the refined transform is on `/tf_static`, not `/tf`

### 3. Old TF publishers affect launch behavior

Stale `static_transform_publisher` processes can satisfy TF checks unexpectedly.

Useful cleanup:

```bash
pkill -f static_transform_publisher
```

---

## File responsibilities summary

### Core runtime files

- `src/onboard_detector/launch/run_detector.launch.py`  
  Main runtime orchestration

- `src/onboard_detector/launch/run_calibration_icp.launch.py`  
  Calibration debugging launch

- `src/onboard_detector/src/calibration_icp_node.cpp`  
  One-shot ICP calibration and refined TF publication

- `src/onboard_detector/include/onboard_detector/dynamicDetector.cpp`  
  Main detector implementation

- `src/onboard_detector/scripts/yolo_detector/yolov11_detector_node.py`  
  YOLO runtime node

- `src/onboard_detector/cfg/detector_param_jo_zotac.yaml`  
  Main runtime configuration

---

## Related documentation

For user-oriented setup and repository layout, see:
- `README.md`

For the original project and paper context, see:
- https://github.com/Zhefan-Xu/LV-DOT
