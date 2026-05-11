# onboard_detector — Framework Architecture

This document describes the internal processing pipeline of `detector_node` (`dynamicDetector` class), from raw sensor data to classified and tracked 3D bounding boxes.

---

## High-level pipeline

```
Depth image ──────────────────────────────┐
                                          ▼
                                  uvDetect()           UV bounding boxes
                                  dbscanDetect()       DBSCAN depth clusters
                                          │
LiDAR scan ──► lidarDetect() ────────────►│
                                          ▼
                                  filterLVBBoxes()
                                  ├─ fuse UV + DBSCAN
                                  ├─ fuse with LiDAR
                                  └─ YOLO association
                                          │
Odometry ────────────────────────────────►│
                                          ▼
                              kalmanFilterAndUpdateHist()
                                  ├─ predict
                                  ├─ associate detections → tracks
                                  └─ update / create / drop tracks
                                          │
                                          ▼
                              classificationCB()
                                  ├─ PATH 1 (YOLO candidate)
                                  │   ├─ speed ≥ dynaVelThresh → dynamic
                                  │   └─ speed < dynaVelThresh → potentially dynamic
                                  ├─ PATH 2 (in FOV, non-YOLO)
                                  │   └─ point-cloud motion voting
                                  └─ PATH 3 (outside FOV)
                                          └─ net displacement + straightness
                                          │
                                          ▼
                                      visCB()
                              publish dynamic / potentially dynamic / filtered bboxes
```

---

## Detection stages

### 1. UV detection (`uvDetect`)

Processes the depth image through a UV occupancy map:
1. Project depth pixels into a 2D UV grid (bird's-eye view)
2. Find occupied cells and cluster them
3. Back-project clusters to 3D bounding boxes

The UV detector is designed to capture objects with a compact footprint (pedestrians, small obstacles).

---

### 2. Visual DBSCAN (`dbscanDetect`)

Clusters the voxel-downsampled depth point cloud using DBSCAN in 3D world space:
- `dbscan_search_range_epsilon` — neighbourhood radius
- `dbscan_min_points_cluster` — minimum cluster size

Optionally applies recursive cluster splitting (`dbscan_refinement_enable`) to break apart large merged blobs.

---

### 3. LiDAR detection (`lidarDetect`)

Runs a separate DBSCAN on the LiDAR point cloud (after projection into the local sensing volume). Uses dedicated parameters (`lidar_DBSCAN_epsilon`, `lidar_DBSCAN_min_points`) tuned for the lower point density of spinning LiDARs at range.

---

### 4. Fusion and YOLO association (`filterLVBBoxes`)

This is the core fusion function. It runs in three phases.

#### Phase A — Sensor fusion

1. **UV ↔ DBSCAN merge** (`mergeNestedGroup`): boxes from UV detection and visual DBSCAN are merged using mutual IoU and IoV thresholds.
2. **Visual ↔ LiDAR merge**: the merged visual boxes are further fused with LiDAR clusters. Configurable via `lidar_visual_merging_flag` (`"bigger"` / `"smaller"` / `"none"`).
3. **Final merge pass** (`final_merge_flag`): optional extra nesting pass before YOLO association.

#### Phase B — YOLO association

For each YOLO 2D detection of a target dynamic class:

**STEP 1 — Build depth point cloud inside YOLO rect**
- Map the YOLO 2D rect from color image to depth image coordinates.
- Apply an **inscribed ellipse mask** to discard corner pixels (which typically contain background returns from adjacent objects).
- Project valid depth pixels to 3D world-frame points → `yoloPoints`.

**STEP 2 — Background filtering**
- **Outdoor (`is_indoor: false`):** height bounds only (`groundHeight_` to `roofHeight_`).
- **Indoor (`is_indoor: true`):** additionally applies a 10th-percentile depth filter: only points within `yoloDepthTolerance_` of the foreground depth are kept.
- Result: `filteredYoloPoints`.

**STEP 3 — Match to 3D bboxes (Case 1 — depth available)**
- For each fused 3D box, count how many `filteredYoloPoints` fall inside it.
- If the fraction ≥ `yolo_point_fraction_threshold` → set `is_yolo_candidate = true`.
- Optionally correct box height: if the Z range of inside points differs from the 3D box height by ≥ `yolo_height_correction_threshold`, override `z_width` and `z`.
- If `yolo_x_y_resize: true`, also resize x/y to fit inside points and re-filter the cluster.

**Case 2 — No depth points (LiDAR-only / beyond depth range)**
- `tryIouFallback()`: project the 8 corners of each 3D bbox onto the color image.
- Compute 2D IoU between the projected AABB and the YOLO rect.
- If IoU ≥ `yolo_2d_iou_threshold` → set `is_yolo_candidate = true`.
- Multiple 3D boxes can match the same YOLO detection (large object split into clusters by DBSCAN).

---

### 5. Kalman filter and track management (`kalmanFilterAndUpdateHist`)

Each detection from `filterLVBBoxes` is either assigned to an existing track or used to create a new one. The association uses a composite score:

```
score = pos_weight    * Δposition
      + size_weight   * Δsize
      + iou2d_weight  * 2D IoU
      + vel_dir_weight * velocity direction error
      + yolo_class_weight * class consistency penalty
      + prev_obs_pos_weight * prev observation distance
      + confirmed_bonus (if track is confirmed)
      + yolo_bonus (if detection is YOLO-flagged)
```

Scores below `min_match_score` are rejected. The best valid assignment is used.

**Track lifecycle:**

```
  New detection ──► UNCONFIRMED
                         │ N consecutive hits (min_confirm_hits)
                         ▼
                    CONFIRMED ◄─────── re-detected each frame
                         │
                         │ missed detections
                         ▼
                    COASTING (up to max_missed_frames)
                         │
                         ▼
                      DELETED
```

YOLO-confirmed tracks use `max_missed_frames_yolo` (extended lifetime) instead of `max_missed_frames`.

The Kalman filter (V2) uses a constant-acceleration motion model with state `[x, y, z, vx, vy, vz, ax, ay, az]`. The position-only observation model allows the filter to estimate velocity and acceleration purely from successive position measurements.

Parameters `[eP, eQPos, eQVel, eQAcc, eRPos]` control initial covariance, process noise on position/velocity/acceleration, and measurement noise respectively.

---

### 6. Classification (`classificationCB`)

Each confirmed track is classified on every frame through one of three paths.

#### PATH 1 — YOLO candidate

A track marked `is_yolo_candidate` (from YOLO association) bypasses motion analysis:
- KF speed ≥ `dynamic_velocity_threshold` → **dynamic** (red box)
- KF speed < `dynamic_velocity_threshold` → **potentially dynamic** (green box) — YOLO-identified object that is currently stationary

#### PATH 2 — In camera FOV, non-YOLO

Motion is assessed by comparing current and historical 3D point cloud positions:
- A per-point displacement vote determines if the cluster is moving.
- `dynamic_voting_threshold` sets the fraction of consistent moving votes required.
- A KF velocity confidence gate (`dynamic_kf_vel_std_ratio`) filters noisy estimates.
- `dynamic_consistency_threshold` consecutive candidate frames are needed before the dynamic label is assigned.
- A sticky label mechanism (`frames_force_dynamic`, `frames_force_dynamic_check_range`) keeps the track dynamic even through brief occlusions.

#### PATH 3 — Outside camera FOV

For tracks outside the camera field of view, motion analysis relies on KF-estimated kinematics only:
- Requires minimum observed speed (`min_outside_fov_obs_speed`) over a sliding window.
- Checks net displacement, straightness of the trajectory, and step-by-step speed consistency.
- Tunable via the `outside_fov_class_*` parameter group.

---

## Track association outside camera FOV

When a track moves outside the camera FOV, normal IoU-based association is unavailable. A dedicated outside-FOV association pass runs with relaxed thresholds:
- `max_match_range_outside_fov` (positional gate)
- `max_relative_size_diff_outside_fov` (size consistency)
- `max_natural_innovation_outside_fov` (Kalman innovation gate)
- `outside_fov_turn_*` parameters allow confirmed, mature tracks to be associated even through abrupt direction changes (e.g., a car turning at an intersection), provided the track age exceeds `outside_fov_turn_min_track_age`.

---

## Ground and wall estimation (wall_detector_node)

The wall detector runs independently and feeds back into the detector:

1. **Ground estimation:** RANSAC on the bottom fraction of the depth image finds the dominant horizontal plane. The result is published as ground height and consumed by `detector_node` to update its height-based filters.
2. **Wall detection:** RANSAC on the LiDAR cloud extracts vertical planes. Each detected wall is registered in an OBB registry with EMA-based merging and expiry.

Ground height is updated via an EMA filter (`ground_ema_alpha`) to smooth out sensor noise.

---

## Indoor vs outdoor mode

The `is_indoor` flag switches several behaviors:

| Behaviour | Indoor (`true`) | Outdoor (`false`) |
|---|---|---|
| Depth background filter | 10th-percentile depth filter active | Height bounds only |
| ICP calibration timeout | No timeout (waits for full ICP) | Falls back to initial guess after 3 s |
| Typical YOLO association range | Depth camera fully covers scene | Mostly LiDAR-only (IoU fallback) |
