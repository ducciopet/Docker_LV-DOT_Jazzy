# onboard_detector — Framework Architecture

This document describes the internal logic of every node and processing module in `onboard_detector`, from raw sensor data to classified and tracked 3D bounding boxes published to the navigation stack.

---

## Table of contents

1. [System-level data flow](#1-system-level-data-flow)
2. [calibration_icp_node](#2-calibration_icp_node)
3. [wall_detector_node](#3-wall_detector_node)
4. [detector_node — overview](#4-detector_node--overview)
5. [UV detection (uvDetect)](#5-uv-detection-uvdetect)
6. [Visual DBSCAN (dbscanDetect)](#6-visual-dbscan-dbscandetect)
7. [LiDAR detection (lidarDetect)](#7-lidar-detection-lidardetect)
8. [Fusion and YOLO association (filterLVBBoxes)](#8-fusion-and-yolo-association-filterlvbboxes)
9. [Kalman filter and track management](#9-kalman-filter-and-track-management)
10. [Dynamic classification (classificationCB)](#10-dynamic-classification-classificationcb)
11. [Outside-FOV association and classification](#11-outside-fov-association-and-classification)
12. [yolov11_detector_node](#12-yolov11_detector_node)
13. [odometry_tf_publisher_node](#13-odometry_tf_publisher_node)
14. [Indoor vs outdoor mode](#14-indoor-vs-outdoor-mode)

---

## 1. System-level data flow

The pipeline processes one detection frame per incoming synchronized (depth + LiDAR) message pair. The full sequence within a single frame is:

```
[Frame N arrives]
        │
        ├─ depth image ──────────────────────────────────────────────────────┐
        │                                                                    │
        │   uvDetect()          build UV occupancy map, cluster, back-project│
        │   dbscanDetect()      voxel-filter depth cloud, DBSCAN, split      │
        │                                                                    │
        ├─ LiDAR scan ───────────────────────────────────────────────────────┤
        │                                                                    │
        │   lidarDetect()       project to local volume, DBSCAN             │
        │                                                                    │
        ├─ YOLO 2D detections (async, latest available) ─────────────────────┤
        │                                                                    │
        └──────────────────────────────────────────────────────────────────► filterLVBBoxes()
                                                                             │
                                                                             │  Phase A: UV ↔ DBSCAN merge
                                                                             │  Phase B: visual ↔ LiDAR merge
                                                                             │  Phase C: YOLO association
                                                                             │
                                                                             ▼
                                                              kalmanFilterAndUpdateHist()
                                                                             │
                                                                             │  predict tracks
                                                                             │  associate detections → tracks
                                                                             │  update / create / drop tracks
                                                                             │
                                                                             ▼
                                                              classificationCB()
                                                                             │
                                                                             ├─ PATH 1: YOLO candidate → speed gate
                                                                             ├─ PATH 2: in FOV → point-cloud motion voting
                                                                             └─ PATH 3: outside FOV → kinematics-only
                                                                             │
                                                                             ▼
                                                                          visCB()
                                                              publish MarkerArrays + ObstacleArray
```

Throughout this pipeline the robot's current pose (from odometry or TF) is used to transform all sensor data into a consistent world frame before any geometry comparison is made.

---

## 2. calibration_icp_node

### Purpose

The depth camera and LiDAR have mechanically independent coordinate frames. `calibration_icp_node` computes the rigid transform **velodyne → camera_refined** by aligning the depth point cloud (expressed in the LiDAR frame via an initial guess) with the LiDAR scan itself. This transform is published as a static TF and used by all other nodes for the lifetime of the session.

### Initialization

At startup the node reads the initial-guess TF `velodyne → camera_initial_guess` from the TF tree. This transform is published by a `static_transform_publisher` in the launch file and should encode a rough manually-measured extrinsic (e.g., from a previous ICP run or a physical measurement). All ICP runs use this as the starting point for iterative refinement.

### Per-scene ICP pipeline

Each time a synchronized (LiDAR scan, depth image) pair arrives:

1. **Depth → point cloud.** The depth image is projected to 3D using the pinhole camera model and depth intrinsics. Pixels with depth outside `[depth_min_value, depth_max_value]` are discarded. Every `depth_skip_pixel`-th pixel is used to reduce computation.

2. **LiDAR overlap extraction.** The full LiDAR cloud is filtered to keep only points that project into the depth camera's field of view (with a margin of `overlap_fov_margin_px` pixels). If fewer than `min_overlap_points` survive, the full LiDAR cloud is used as the ICP target to prevent degenerate registration.

3. **Voxel down-sampling.** Both the depth cloud (source) and the LiDAR overlap (target) are voxel-filtered at `icp_voxel_size` to normalize point density.

4. **Normal estimation (point-to-plane mode).** When `icp_use_point_to_plane: true`, surface normals are estimated on the target cloud using a KD-tree search within radius `icp_normal_radius` and at most `icp_normal_max_nn` neighbours. Larger values are needed for outdoor scenes where the point density is lower.

5. **ICP registration.** Open3D's `RegistrationICP` runs for up to `icp_max_iter` iterations with a maximum correspondence distance of `icp_max_correspondence_distance`. The initial transformation is the current estimate of `T_lidar_camera_init_` (starts from the initial-guess TF, updated after each accepted scene).

6. **Fitness gate.** The ICP fitness score (fraction of source points with a correspondence below the distance threshold) is checked against `icp_fitness_threshold`. Scenes that fail are discarded. Accepted scenes contribute their transform and RPY to the accumulator.

### Multi-scene averaging

After `icp_max_runs` scenes have been processed, the node computes the final extrinsic:

- **Translation:** arithmetic mean of accepted translation vectors.
- **Rotation:** mean-of-angles using `atan2(mean_sin, mean_cos)` per roll/pitch/yaw axis, which correctly handles angle wraparound.

The averaged transform is then published as the static TF `velodyne → camera_refined` and logged in full.

If no scene ever passes the fitness threshold (all discarded), the accumulator is reset and the node retries from the next incoming frame batch.

### Outdoor timeout and fallback

When `is_indoor: false`, a wall timer fires after `icp_timeout_sec` seconds:

- **If ≥ 1 valid ICP result exists:** `publishOnTimeout()` calls `finalizeCalibration()`, which publishes the partial average as `camera_refined`. Detection starts with a coarser but real ICP estimate.
- **If 0 valid results exist:** the initial-guess TF is published directly as `camera_refined`. Detection starts immediately with no refinement.

In both cases `processed_ = true` is set, which silences all further callbacks. A nested retry timer at 1-second intervals handles the case where the TF lookup fails transiently at timeout time (e.g., the clock is not yet running with `use_sim_time`).

The indoor path has no timeout: the node waits indefinitely for a full set of `icp_max_runs` valid scenes.

### Why ICP is harder outdoors

Outdoor environments typically have:
- **Fewer planar surfaces** for point-to-plane ICP to anchor onto (no walls/floor/ceiling forming a complete geometric cage).
- **Sparser depth returns** from sunlight interference or long-range objects.
- **Less overlap** between the LiDAR and depth camera FOVs when the scene is open.

The outdoor config compensates by relaxing `icp_fitness_threshold` (0.15 vs 0.5), widening `icp_max_correspondence_distance` (0.35 m vs 0.2 m), using larger voxels (0.08 m) and a larger normal estimation radius (0.25 m).

---

## 3. wall_detector_node

### Purpose

`wall_detector_node` has two independent responsibilities that run in parallel:

1. **Ground height estimation** from the depth camera — feeds `detector_node` so its height-based filters always track the true ground level even on sloped or uneven terrain.
2. **Wall detection** from the LiDAR — maintains a registry of vertical planar surfaces so `detector_node` can filter out static wall detections from its obstacle output.

### Calibration gate

Both functions are gated on the `camera_refined` TF being present in the TF tree (published by `calibration_icp_node`). Until that TF appears, the node silently drops all incoming messages. This prevents using a stale or zero extrinsic during the calibration phase.

### Ground height estimation (depth RANSAC)

**Why from depth and not LiDAR?** The depth camera gives a dense, textured image of the floor surface in front of the robot and is less affected by specular reflections from flat surfaces. LiDAR often under-samples the ground at close range due to the scan angle.

**Algorithm:**

1. **ROI selection.** Only the bottom `1/ground_estim_bottom_fraction` rows of the depth image are used. These rows correspond to the ground plane in front of the robot at typical mounting heights.
2. **3D projection.** Each valid depth pixel in the ROI is unprojected to a 3D point in the depth camera frame using the pinhole model.
3. **Transform to world frame.** Points are transformed using the current `camera_refined` pose (from TF).
4. **RANSAC plane fit.** Three random points define a candidate plane. Inliers are all points within a distance threshold of the plane. The best plane (most inliers) after `ransac_max_iterations` is accepted if it has at least `ground_estim_min_inliers` points.
5. **Ground height update.** The median Z of inlier points plus `ground_conservative_offset` is used as the new ground height estimate. This conservative offset ensures the ground is never estimated too high (which would cause ground-level obstacles to be missed).
6. **EMA smoothing.** The published ground height is smoothed with an EMA filter: `g_new = α * g_raw + (1−α) * g_prev`. A small `ground_ema_alpha` (e.g., 0.05) prevents sudden jumps from individual noisy depth frames.

The roof height is always `ground_height + ground_roof_offset`.

### Wall detection (LiDAR RANSAC)

**Algorithm:**

1. **Voxel filtering.** The incoming LiDAR cloud is voxel-filtered at `voxel_resolution` to normalize point density and reduce computation.
2. **Iterative plane extraction.** RANSAC is run repeatedly on the remaining cloud until `max_planes` have been found or no more planes meet the minimum inlier count:
   a. Three random points define a candidate plane via `fitPlane()`.
   b. All points within `ransac_inlier_threshold` of that plane are counted as inliers.
   c. After `ransac_max_iterations` samples, the best plane is accepted if it has ≥ `ransac_min_inliers` inliers.
   d. Inlier points are removed from the cloud before the next iteration.
3. **Wall classification.** A plane is classified as a wall if its normal vector is sufficiently horizontal, i.e., the angle between the normal and the XY plane is ≤ `wall_vertical_angle_deg` degrees. Ground and ceiling planes (near-horizontal normals) are discarded.
4. **Contiguous cluster extraction.** The inlier points of each wall plane are further clustered spatially to extract the largest contiguous segment. This removes stray isolated inlier points that are geometrically on the plane but physically disconnected.
5. **OBB fitting.** An oriented bounding box is fitted to the contiguous inlier cluster using PCA on the inlier point set: the eigenvectors of the covariance matrix define the OBB axes, and the extents are the projections along those axes. The OBB is discarded if its aspect ratio (long/short axis) exceeds `wall_bbox_max_aspect_ratio` (which would indicate a degenerate plane fit, not a real wall).

### WallBBoxRegistry — persistent wall tracking

Individual LiDAR scans are noisy and may miss walls due to occlusion or scan angle. The `WallBBoxRegistry` maintains a persistent set of wall OBBs across frames:

- **Matching.** Each newly detected wall OBB is matched to the closest existing registry entry based on: (a) IoV (intersection-over-volume of the smaller box) ≥ `overlap_threshold`, (b) surface normal dot product ≥ `min_normal_dot`, and (c) center distance ≤ `max_center_distance`. All three conditions must hold.
- **Merging.** Matched entries are updated with an EMA: `existing = (1−w) * existing + w * incoming` where `w = merge_weight`. This prevents abrupt jumps while still allowing the registry to adapt to small changes.
- **New entries.** Unmatched detections are added to the registry as new entries.
- **Pose compensation.** Between frames, all existing registry OBBs are transformed by the odometry delta pose (current pose − previous pose) to keep them aligned with the world frame even as the robot moves.
- **Expiry.** Entries not matched for `max_missed_frames` consecutive frames are removed. This allows walls to disappear when the robot moves past them or a doorway opens.

---

## 4. detector_node — overview

`detector_node` instantiates `dynamicDetector` and wires its inputs to ROS subscriptions. The main callback runs on every synchronized (depth + LiDAR) message pair and executes the full pipeline:

```
depthAndCloudCB()
    ├── uvDetect()
    ├── dbscanDetect()
    ├── lidarDetect()
    ├── filterLVBBoxes()        // fusion + YOLO association
    ├── kalmanFilterAndUpdateHist()
    ├── classificationCB()
    └── visCB()                 // publish
```

The robot's pose is read at the start of each callback either from the latest odometry message (`localization_mode: 1`) or by looking up the `odom → base_link` TF chain. All detection functions receive the current pose so they can transform sensor data into the world frame.

---

## 5. UV detection (uvDetect)

### Concept

The UV detector builds a **bird's-eye-view occupancy grid** from the depth image. Each cell in the grid represents a column of 3D space above a fixed XY footprint. Occupied cells are clustered and their 3D bounding boxes are extracted. This is fast and works well for compact objects (pedestrians, boxes) because it avoids 3D point cloud processing entirely.

### Algorithm

1. **Depth unprojection.** Each valid depth pixel is projected to 3D using the pinhole model. Points outside `[depth_min_value, depth_max_value]` and outside `local_sensor_range` are discarded.
2. **Ground/roof filtering.** Points below `ground_height + margin` or above `roof_height` are discarded. The ground height comes from `wall_detector_node` (or falls back to the parameter default).
3. **UV projection.** Each 3D point `(X, Y, Z)` in the world frame is mapped to a 2D UV cell `(u, v)` by discretizing X and Y at a fixed resolution. The cell is marked occupied.
4. **2D connected-component clustering.** Occupied UV cells are clustered using 4-connectivity or DBSCAN on the 2D grid.
5. **3D back-projection.** For each cluster, the bounding box is computed from the 3D points that fell into its cells: min/max X, Y, Z give the `box3D` geometry. The center and dimensions are stored in a `box3D` struct.

The UV detector is complementary to DBSCAN: it tends to give tighter horizontal bounds but can merge vertically separated objects (stacked boxes) into one cluster.

---

## 6. Visual DBSCAN (dbscanDetect)

### Concept

The visual DBSCAN detector clusters the depth point cloud directly in 3D world space. It captures elongated or irregular shapes that the UV grid would merge or split incorrectly, and provides per-point assignment to clusters.

### Algorithm

1. **Depth cloud construction.** Same projection and filtering as UV detection. The resulting 3D point cloud is expressed in the world frame.
2. **Voxel down-sampling.** The cloud is voxel-filtered at `voxel_size` to normalize density. The occupied voxel threshold `voxel_occupied_thresh` discards voxels with too few raw points (noise rejection).
3. **DBSCAN.** The `dbscan` class runs standard DBSCAN with:
   - `dbscan_search_range_epsilon` — neighbourhood radius in 3D world space
   - `dbscan_min_points_cluster` — minimum cluster size
   Each cluster is assigned a unique label; noise points get label -1.
4. **Bounding box extraction.** For each cluster, axis-aligned bounding box min/max XYZ are computed.
5. **Cluster refinement (optional).** When `dbscan_refinement_enable: true`, large clusters (diagonal > `dbscan_refine_max_diagonal`) are recursively split if their density is below `dbscan_refine_min_density`. Splitting uses a second DBSCAN pass with `dbscan_refine_split_eps` and discards sub-clusters smaller than `dbscan_refine_min_subcluster_pts`. This breaks apart merged blobs (e.g., two people standing close together) at the cost of increased computation.

---

## 7. LiDAR detection (lidarDetect)

### Concept

The LiDAR provides a 360° dense scan at longer range than the depth camera. Its DBSCAN operates in the LiDAR's local frame (projected to the robot-centred `local_lidar_range` volume) and uses wider parameters than the visual DBSCAN because spinning LiDARs have lower point density per unit area at range.

### Algorithm

1. **Range filtering.** Points outside `local_lidar_range` (a box ±X, ±Y, ±Z around the robot) are discarded.
2. **Ground/roof filtering.** Same height gates as the visual pipeline.
3. **DBSCAN.** Parameters: `lidar_DBSCAN_epsilon` (neighbourhood radius, typically 0.25 m to bridge point gaps on car surfaces at range) and `lidar_DBSCAN_min_points`.
4. **Bounding box extraction.** Axis-aligned bounding boxes are computed per cluster.

LiDAR clusters are the backbone of the far-field detection pipeline. They are typically larger and noisier than depth clusters but cover objects well beyond the depth camera's 5 m range.

---

## 8. Fusion and YOLO association (filterLVBBoxes)

This function receives three independent lists of `box3D` (UV, DBSCAN, LiDAR) plus the latest YOLO 2D detections and produces a single fused, YOLO-annotated list.

### Phase A — UV ↔ DBSCAN merge

`mergeNestedGroup(uvBoxes, dbBoxes, samegroupIOU_threshold, samegroupIOV_threshold)`:

- For each pair (UV box, DBSCAN box), compute mutual 3D IoU and IoV (intersection volume / volume of the smaller box).
- Pairs above `samegroupIOU_threshold` (peers) or `samegroupIOV_threshold` (one contained in the other) are merged.
- The merge result depends on `visual_merging_flag`: `"smaller"` = tight box containing both centers; `"bigger"` = union; `"none"` = keep both.
- UV boxes with no DBSCAN match are kept if `uv_unmerged_flag: true`; DBSCAN boxes with no UV match are kept if `db_unmerged_flag: true`.

The result is a single list of **visual boxes** (combining UV and DBSCAN evidence).

### Phase B — Visual ↔ LiDAR merge

`mergeNestedGroup(visualBoxes, lidarBoxes, lidar_visual_filtering_BBox_IOU_threshold, lidar_visual_filtering_BBox_IOV_threshold)`:

Same procedure as Phase A but across the visual and LiDAR lists. Controlled by `lidar_visual_merging_flag`, `visual_unmerged_flag`, `lidar_unmerged_flag`. In outdoor mode `lidar_unmerged_flag: true` is critical because most objects beyond 5 m have no depth return and exist only as LiDAR clusters.

An optional **final merge pass** (`final_merge_flag`) runs an extra `mergeNestedGroup` call on the fused list to collapse any remaining overlapping boxes before YOLO association.

### Phase C — YOLO association

For each YOLO 2D detection whose class is in `yolo_dynamic_classes`:

#### STEP 1 — Build YOLO depth point cloud

The YOLO 2D bounding rect (in color image coordinates) is mapped to depth image coordinates. An **inscribed ellipse mask** is applied: pixels in the corners of the rectangle (outside the ellipse inscribed in the rect) are discarded. This is important because corners of YOLO rectangles often contain background pixels from adjacent objects, especially at object boundaries.

Valid depth pixels inside the ellipse are projected to 3D world-frame points → `yoloPoints`.

#### STEP 2 — Background filtering

- **Outdoor (`is_indoor: false`):** only height bounds are applied (`ground_height` to `roof_height`). All depth points in the right height range are kept. This is appropriate outdoors where background separation by depth percentile is unreliable (open sky, large depth variance).
- **Indoor (`is_indoor: true`):** an additional **10th-percentile depth filter** is applied. The 10th percentile of the Z values of `yoloPoints` is the foreground depth. Only points within `yolo_depth_tolerance` meters beyond this percentile are kept. This aggressively separates the foreground object from background walls/floors that happen to project into the YOLO rect.

Result: `filteredYoloPoints` — a tight 3D point cloud representing the foreground object seen by YOLO.

#### STEP 3A — Match with depth points (Case 1)

For each fused 3D box, count how many `filteredYoloPoints` fall inside it. If `inside_count / total_yolo_points ≥ yolo_point_fraction_threshold` → set `is_yolo_candidate = true` on that box.

**Height correction:** if the Z range of inside points differs from the box's Z width by ≥ `yolo_height_correction_threshold`, the box height and Z center are overridden with the YOLO-derived values. YOLO gives an accurate silhouette of the object which often constrains height better than LiDAR clusters (which over-extend vertically due to ground reflections or multi-layer scan lines).

**XY resize (`yolo_x_y_resize: true`):** the box X/Y extents are also shrunk to fit the inside points, and the cluster is re-filtered using only those points. This tightens the bounding box to the true object footprint as seen by depth, at the cost of some robustness to depth holes.

#### STEP 3B — IoU fallback (Case 2, no depth points)

When `filteredYoloPoints` is empty (object is beyond depth camera range, or the depth image has no returns), `tryIouFallback()` projects the 8 corners of each 3D box onto the color image using the camera intrinsics and the `camera_refined` extrinsic TF. The axis-aligned 2D bounding box of those projected corners is compared to the YOLO rect. If 2D IoU ≥ `yolo_2d_iou_threshold` → `is_yolo_candidate = true`.

This fallback allows YOLO to annotate LiDAR-only clusters of cars and persons that are beyond the depth camera's effective range (~5 m).

---

## 9. Kalman filter and track management

### Motion model

The Kalman filter uses a **constant-acceleration** model with 9-dimensional state:

```
x = [px, py, pz, vx, vy, vz, ax, ay, az]ᵀ
```

The state transition matrix `A` is parametrized by `dt` (the time elapsed since the last frame):

```
A = I₉ + dt * B_vel + (dt²/2) * B_acc
```

where `B_vel` and `B_acc` couple velocity and acceleration into the position terms. Only the 3D position `[px, py, pz]` is observed directly (observation matrix `H` is a 3×9 identity-like matrix picking the first three states).

This means velocity and acceleration are never measured directly — they are estimated purely from the sequence of position observations. The filter learns to predict where a moving object will be in the next frame.

**Noise parameters** (`kalman_filter_v2_param: [eP, eQPos, eQVel, eQAcc, eRPos]`):
- `eP` — initial state covariance (uncertainty at track creation).
- `eQPos/eQVel/eQAcc` — process noise on position/velocity/acceleration. Higher `eQVel`/`eQAcc` makes the filter respond faster to acceleration changes (appropriate for cars).
- `eRPos` — measurement noise on position. Higher values make the filter trust measurements less and rely more on the prediction.

### Association scoring

Each frame, the Kalman filter first **predicts** each track's position (no measurement needed). Then incoming detections are matched to existing tracks using a composite score:

```
score = match_pos_score_weight            × Δposition(predicted, detection)
      + match_size_score_weight           × |size_diff| / mean_size
      + match_iou2d_score_weight          × (1 - 2D_IoU)
      + match_velocity_direction_weight   × velocity_direction_error [rad]
      + match_yolo_class_weight           × class_consistency_penalty
      + match_prev_obs_pos_weight         × Δposition(last_obs, detection)
      + match_prev_obs_iou2d_weight       × (1 - 2D_IoU_vs_last_obs)
      − confirmed_track_assoc_bonus       (if track is confirmed)
      − yolo_track_assoc_bonus            (if detection is YOLO-flagged)
```

Lower score = better match. Assignments with score > `min_match_score` are rejected outright. Confirmed tracks use the stricter `min_match_score_confirmed` threshold; dynamic tracks use `min_match_score_dynamic`. The best-scoring valid assignment is used (greedy, single-pass).

The velocity direction error term penalizes assignments where the implied motion direction (from predicted position to detection position) is inconsistent with the track's current velocity direction. This is particularly effective at preventing ID swaps when two objects cross paths.

A **speed gate** additionally rejects assignments where the implied speed exceeds `max_match_speed` (25 m/s by default, designed to handle fast cars).

### Track lifecycle

```
New detection with no match
        │
        ▼
   UNCONFIRMED
        │
        │  N consecutive hits without interruption
        │  (N = min_confirm_hits)
        ▼
   CONFIRMED ◄─────────────── re-detected each frame (reset missed counter)
        │
        │  consecutive misses
        ▼
   COASTING (Kalman prediction only, no observation update)
        │
        │  misses > max_missed_frames (or max_missed_frames_yolo for YOLO tracks)
        ▼
     DELETED
```

**Natural motion gates** prevent noise from inflating track age. For an unconfirmed track to progress toward confirmation, each frame's update must satisfy:
- Displacement ≥ `min_natural_motion_dist` (track is not frozen)
- Displacement ≤ `max_natural_motion_dist` (track is not jumping)
- Kalman innovation ≤ `max_natural_innovation` (measurement agrees with prediction)
- Velocity direction error ≤ `max_velocity_direction_error_confirm`

Confirmed tracks use looser gates (`max_natural_innovation_confirmed`, `max_velocity_direction_error_tracked`), and dynamic tracks use even looser ones (`max_natural_innovation_dynamic`, `max_velocity_direction_error_tracked_dynamic`) to tolerate the higher speed and acceleration of fast-moving objects.

### Duplicate suppression

After the main association pass, a deduplication step removes tracks that overlap with another track beyond thresholds (`duplicate_track_dist_threshold`, `duplicate_track_iou_threshold`, `duplicate_size_rel_threshold`). The younger/less-confirmed track is removed.

---

## 10. Dynamic classification (classificationCB)

Each confirmed track is reclassified every frame through one of three exclusive paths.

### PATH 1 — YOLO candidate

A track is on PATH 1 if its `is_yolo_candidate` flag was set during YOLO association in the current or a recent frame (the flag is sticky for `max_non_yolo_in_fov_frames` frames inside the FOV, longer outside it).

Classification is purely speed-based using the Kalman-estimated velocity:

- `‖v‖ ≥ dynamic_velocity_threshold` → **dynamic** (`is_dynamic = true`). The track is flagged as a confirmed moving obstacle and published on `/dynamic_bboxes`.
- `‖v‖ < dynamic_velocity_threshold` → **potentially dynamic** (`is_potentially_dynamic = true`). The object is recognized by YOLO as a dynamic class (e.g., a parked car or a standing person) but is currently stationary. Published on `/potentially_dynamic_bboxes`.

YOLO-candidate tracks bypass motion voting entirely. This prevents a stationary person from being wrongly classified as a static obstacle just because the point cloud isn't moving.

### PATH 2 — In camera FOV, non-YOLO

Tracks inside the camera FOV that are not YOLO candidates are assessed by **point-cloud motion voting**:

1. **Frame comparison.** The current frame's point cloud for the track is compared with the point cloud from `frame_skip` frames ago. Each point in the current frame finds its nearest neighbour in the historical cloud and computes the displacement vector.
2. **Per-point vote.** A point casts a "moving" vote if its displacement ≥ `dynamic_velocity_threshold × dt × frame_skip`.
3. **Voting threshold.** If the fraction of moving votes ≥ `dynamic_voting_threshold` AND the Kalman velocity estimate has low standard deviation (coefficient of variation ≤ `dynamic_kf_vel_std_ratio`), the track is a **dynamic candidate**.
4. **Consistency gate.** The track must be a dynamic candidate for `dynamic_consistency_threshold` consecutive frames before the `is_dynamic` flag is set. This prevents single noisy frames from triggering false positives.
5. **Sticky label.** Once confirmed dynamic, the label is maintained if at least `frames_force_dynamic` out of the last `frames_force_dynamic_check_range` frames show dynamic evidence. This keeps the track dynamic during brief occlusions.

### PATH 3 — Outside camera FOV

For tracks that have moved out of the camera FOV, depth-based motion voting is unavailable. Classification uses Kalman-estimated kinematics:

1. **Speed window.** The observed speeds (Kalman-estimated `‖v‖`) over the last `outside_fov_class_window` frames are averaged.
2. **Net displacement and straightness.** The displacement from the oldest to the newest position in the window is compared to the sum of step-by-step distances. A high ratio (close to 1.0) means the object is moving in a straight line; a low ratio means it's oscillating (likely noise or a stationary object jitter).
3. **Conditions for dynamic:**
   - Mean speed ≥ `outside_fov_class_min_net_speed`
   - Net displacement ≥ `outside_fov_class_min_net_disp`
   - Straightness ≥ `outside_fov_class_min_straightness`
   - Per-step speed ≤ `outside_fov_class_max_step_speed` (eliminates spurious jumps)
   - Track age ≥ `outside_fov_class_min_track_age`

This conservative set of conditions prevents noise-induced Kalman velocity from falsely promoting static objects to dynamic.

---

## 11. Outside-FOV association and classification

When a track exits the camera FOV, normal 3D IoU-based association may fail because the box shapes differ between LiDAR-only (less precise) and LiDAR+depth (tighter) detection modes. A dedicated outside-FOV association pass runs with:

- **Wider positional gate:** `max_match_range_outside_fov` (e.g., 1.2 m vs 4.0 m in-FOV).
- **Size tolerance:** `max_relative_size_diff_outside_fov` — LiDAR-only clusters are typically smaller than their LiDAR+depth equivalents (depth fills in between scan lines), so a moderate size difference is tolerated.
- **Innovation gate:** `max_natural_innovation_outside_fov` — Kalman residual threshold.
- **Abrupt-turn association:** confirmed, mature tracks (`track_age ≥ outside_fov_turn_min_track_age`) can be matched even when the implied direction change is large (e.g., a car turning at a corner), provided the track is moving and the size difference is small. Tuned by `outside_fov_turn_*` parameters.

**YOLO ID-swap guard:** after a YOLO detection on the object, the `yolo_base_size` is recorded. If the track's bounding box grows by more than `yolo_base_mismatch_thresh` (e.g., 150%) relative to that baseline for `max_yolo_base_mismatch_frames` consecutive frames, the `is_yolo_candidate` flag is cleared. This prevents LiDAR noise or a nearby wall from inheriting a pedestrian's YOLO label after an ID swap outside the FOV.

---

## 12. yolov11_detector_node

A pure-Python ROS 2 node that runs YOLOv11 inference using the `ultralytics` library. It subscribes to the color image topic, runs inference at `timer_period_sec` intervals (~30 Hz), and publishes a `vision_msgs/Detection2DArray` with 2D bounding boxes and class labels.

Only classes listed in `target_classes` are forwarded. All other detections are silently discarded before publishing to avoid loading the `detector_node` association step with irrelevant boxes (e.g., traffic lights, furniture).

The inference size (`inference_size: 640`) controls the resolution at which YOLO processes the image. Higher values improve detection at long range (better for cars) at the cost of latency.

---

## 13. odometry_tf_publisher_node

A thin C++ node that subscribes to `/odometry/filtered` (EKF output from `robot_localization`) and re-broadcasts it as a `odom → base_link` TF transform at the same timestamp. This is only needed when running the detector **without** the full `jo_navigation` stack (e.g., playing a bag without starting EKF). When EKF is running, it publishes the TF itself and this node is redundant (disable with `odom_pub:=false`, which is the default).

---

## 14. Indoor vs outdoor mode

The `is_indoor` flag in the config file controls several pipeline behaviours simultaneously:

| Behaviour | Indoor (`true`) | Outdoor (`false`) |
|---|---|---|
| **Depth background filter in YOLO association** | 10th-percentile depth filter removes background returns within each YOLO rect | Height bounds only — no depth percentile filtering |
| **ICP calibration timeout** | No timeout — node waits indefinitely for `icp_max_runs` valid scenes | After `icp_timeout_sec` seconds, publishes partial average or initial guess |
| **Typical YOLO association mode** | Case 1 (depth available) dominant — depth camera fully covers scene | Case 2 (IoU fallback) common — many objects beyond depth range |
| **Recommended `icp_fitness_threshold`** | 0.5 (strict — indoor geometry constrains ICP well) | 0.15 (relaxed — fewer planar surfaces) |
| **Detection range** | `local_sensor_range: [5, 5, 5]` m — tight, no far-field | `local_lidar_range: [15, 15, 5]` m — wide, LiDAR dominant beyond 5 m |

In outdoor mode, the pipeline relies much more heavily on LiDAR clusters (Phase B and Case 2 of YOLO association) because the depth camera is effectively limited to ~4–5 m in most outdoor conditions. The classification pipeline must therefore depend more on Kalman kinematics (PATH 3) than on point-cloud motion voting (PATH 2).
