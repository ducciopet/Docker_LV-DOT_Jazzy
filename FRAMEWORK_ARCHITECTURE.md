# LV-DOT Framework Architecture

This document describes in detail the internal architecture of the detection, fusion, tracking, and classification pipeline implemented in this repository.

---

## Table of contents

1. [System overview](#1-system-overview)
2. [Node: calibration\_icp\_node](#2-node-calibration_icp_node)
3. [Node: wall\_detector\_node](#3-node-wall_detector_node)
4. [Node: yolov11\_detector\_node](#4-node-yolov11_detector_node)
5. [Node: detector\_node — timer architecture](#5-node-detector_node--timer-architecture)
6. [Stage 1 — LiDAR point cloud preprocessing](#6-stage-1--lidar-point-cloud-preprocessing)
7. [Stage 2 — Depth camera detection (UV + DBSCAN)](#7-stage-2--depth-camera-detection-uv--dbscan)
8. [Stage 3 — LiDAR DBSCAN clustering](#8-stage-3--lidar-dbscan-clustering)
9. [Stage 4 — Bbox fusion (filterLVBBoxes)](#9-stage-4--bbox-fusion-filterlvbboxes)
10. [Stage 5 — YOLO association and bbox resizing](#10-stage-5--yolo-association-and-bbox-resizing)
11. [Stage 6 — Tracking (zmin gate + box association + Kalman filter)](#11-stage-6--tracking-zmin-gate--box-association--kalman-filter)
12. [Stage 7 — Dynamic classification](#12-stage-7--dynamic-classification)
13. [Track lifecycle summary](#13-track-lifecycle-summary)
14. [Key data structures](#14-key-data-structures)
15. [Configuration reference](#15-configuration-reference)

---

## 1. System overview

```
                   ┌─────────────────────────────────────┐
  /velodyne_points │        calibration_icp_node          │──► /tf_static (camera_refined)
  /depth image     │  one-shot ICP LiDAR↔camera refinement│──► debug clouds
                   └─────────────────────────────────────┘
                                    │ camera_refined TF
                                    ▼
  /velodyne_points ──────────────────────────────────────────────────────────────────┐
  /depth image     ──────────────────────────────────────────────────────────────────┤
  /pose or /odom   ──────────────────────────────────────────────────────────────────┤
                                                                                     │
                   ┌─────────────────────────────────────┐                          │
  /velodyne_points │         wall_detector_node           │──► /wall_detector/       │
  /depth image     │  RANSAC walls + ground/roof height   │    wall_markers          │
                   └─────────────────────────────────────┘    ground_height         │
                                    │                               │                │
                                    └───────────────────────────────┼────────────────┤
                                                                    ▼                ▼
                   ┌──────────────────────────────────────────────────────────────────┐
                   │                      detector_node                               │
                   │                                                                  │
                   │  LiDAR preprocessing ──► LiDAR DBSCAN                           │
                   │  Depth unprojection  ──► UV detection ──► visual DBSCAN         │
                   │                                                    │             │
                   │              BboxesMerger (UV + visual DBSCAN)     │             │
                   │              BboxesMerger (visual + LiDAR)         │             │
                   │                                │                                 │
                   │  YOLO 2D detection ────────────►  YOLO association + resize      │
                   │                                │                                 │
                   │                         zmin gate                               │
                   │                         boxAssociation (Kalman predict)         │
                   │                         kalmanFilterAndUpdateHist               │
                   │                         classificationCB                        │
                   │                                │                                 │
                   └────────────────────────────────┼─────────────────────────────────┘
                                                    │
                              tracked_bboxes ◄───────┤
                              dynamic_bboxes ◄───────┘

  /color image ──► yolov11_detector_node ──► yolo_detector/detected_bounding_boxes ──► detector_node
```

---

## 2. Node: calibration\_icp\_node

### Purpose

Computes a refined extrinsic transform between the LiDAR (`velodyne`) and the depth camera frame (`camera_refined`) using ICP point-to-point alignment. This refinement is performed once at startup and its result is broadcast as a static TF.

### Why it matters

All depth-based detections and the wall detector rely on transforming depth points into the map frame using the `velodyne → camera_refined` TF. An inaccurate initial guess causes misalignment between LiDAR clusters and depth-derived clusters in the fusion stage, producing spurious merged bboxes.

### Behavior

- Subscribes to synchronized LiDAR and depth streams using `ApproximateTime`.
- On the first valid pair, projects the depth image into 3D (camera frame), transforms both clouds to a common frame, and runs Open3D ICP.
- If ICP converges (fitness score below threshold, RMSE below threshold), publishes:
  - static TF `velodyne → camera_refined` via `StaticTransformBroadcaster`
  - `/calibration/velodyne_points` — LiDAR cloud for debug visualization
  - `/calibration/depth_cloud_in_velodyne` — depth cloud aligned in LiDAR frame
- After success the node does **not** recalibrate. It is one-shot.

### Gating of the main detector

The main detector calls `lookupTfMatrix("velodyne", "camera_refined")` periodically. Until this lookup succeeds, the member `lidarToDepthCamOk_` remains false and all detector timer callbacks return immediately. This prevents detection from running with a stale or absent extrinsic.

---

## 3. Node: wall\_detector\_node

### Purpose

Detects planar walls from LiDAR and estimates the floor and ceiling height from the depth camera. Its outputs directly constrain the main detector:

- **Ground and roof height** (`/wall_detector/ground_height`) — published as `[ground_z, roof_z, offset]` — used by the main detector to filter depth points and LiDAR points vertically.
- **Wall OBBs** (`/wall_detector/wall_markers`) — oriented bounding boxes of detected walls, consumed by the main detector to remove wall points from both the depth cloud and the LiDAR cloud before clustering.

### Why it is a separate node

Wall detection is computationally independent from the detection pipeline. Running it separately allows it to operate at its own frequency and avoids blocking the main detector loop. The main detector consumes its outputs asynchronously via topic subscriptions.

### Ground and roof estimation

The node reads the latest depth image. It takes the bottom `1/ground_estim_bottom_fraction` rows of the image (configurable), unprojects each pixel to 3D in the map frame, and runs RANSAC to fit a horizontal plane. The estimated plane height is smoothed with an EMA filter (alpha = `ground_ema_alpha`). The roof height is derived as `ground_height + ground_roof_offset`.

This estimate is what the main detector uses to:
- clip the depth point cloud vertically before voxel filtering
- clip the LiDAR cloud vertically before DBSCAN
- clip YOLO foreground points before bbox matching

### Wall detection

The node runs multi-plane RANSAC on the incoming LiDAR cloud (after voxel downsampling). For each detected plane:
1. Checks that the plane normal is nearly vertical (`wall_vertical_angle_deg` threshold), confirming it is a wall and not floor/ceiling.
2. Extracts the largest spatially contiguous cluster of inlier points.
3. Builds an oriented bounding box (OBB) aligned with the plane normal.
4. Checks the aspect ratio of the OBB (`wall_bbox_max_aspect_ratio`) to reject thin slivers.
5. Removes inliers from the cloud and iterates for the next plane (up to `max_planes`).

Detected wall OBBs are passed through a persistent registry (`WallBBoxRegistry`) that:
- tracks OBBs across frames using IOU/IOV overlap and normal similarity
- applies EMA smoothing (`merge_weight`) to smooth out jitter
- expires OBBs that have not been observed for `max_missed_frames` frames
- compensates for robot motion by transforming existing OBBs by the delta pose each frame

### How wall OBBs are used in the main detector

The main detector subscribes to `/wall_detector/wall_markers`. On each callback it deserializes the marker array into `std::vector<WallOBB>` (center, rotation, half-size). The function `isInsideAnyWall(point)` is then called:
- during depth point voxel filtering — to discard depth points inside any wall OBB
- during LiDAR preprocessing — to discard LiDAR points inside any wall OBB before DBSCAN

This prevents walls from generating spurious clusters in either sensor path.

---

## 4. Node: yolov11\_detector\_node

### Purpose

Runs YOLOv11 inference on the color image stream and publishes 2D bounding boxes for target classes (by default: `person`). The main detector uses these 2D detections to:
- identify which 3D fused bboxes correspond to humans
- resize those bboxes to tightly fit the human silhouette
- mark them as `is_yolo_candidate = true`, which activates privileged tracking and classification logic

### Implementation

The Python node (`yolov11_detector.py`) is a `rclpy.Node` subclass that:
- subscribes to the color image topic
- runs YOLO inference on a timer (`timer_period_sec`, typically ~33 ms)
- publishes `vision_msgs/Detection2DArray` to `yolo_detector/detected_bounding_boxes`

The bounding box format encodes the top-left corner in `results[0].pose.pose.position.{x,y}` and the size in `bbox.size_x / size_y`. The class label is in `results[0].hypothesis.class_id`.

### Asynchronous operation

YOLO runs independently at its own rate. The main detector stores the latest `Detection2DArray` in `yoloDetectionResults_` on each callback and consumes it once per detection cycle inside `filterLVBBoxes`. After consumption the array is cleared so stale detections are not reused on subsequent cycles.

### Device selection

YOLO runs on CUDA if available, otherwise on CPU. Half-precision (`float16`) is used automatically when CUDA is detected.

---

## 5. Node: detector\_node — timer architecture

The main detector (`dynamicDetector`) runs five independent ROS 2 wall timers, all at the same period (`dt`, default 50 ms = 20 Hz):

| Timer | Callback | What it does |
|-------|----------|--------------|
| `detectionTimer_` | `detectionCB` | Depth detection: `dbscanDetect` → `uvDetect` → `filterLVBBoxes` |
| `lidarDetectionTimer_` | `lidarDetectionCB` | LiDAR detection: `lidarDetect` |
| `trackingTimer_` | `trackingCB` | zmin gate → `boxAssociation` → `kalmanFilterAndUpdateHist` |
| `classificationTimer_` | `classificationCB` | Dynamic/static classification |
| `visTimer_` | `visCB` | Publish all visualization topics |

All callbacks check `lidarToDepthCamOk_` and return immediately if false.

Data flow between timers uses shared member variables (`filteredBBoxes_`, `lidarBBoxes_`, `boxHist_`, etc.) without explicit synchronization. The timers fire at the same rate and in the same ROS 2 executor thread, so ordering is deterministic within a single spin cycle.

---

## 6. Stage 1 — LiDAR point cloud preprocessing

**Triggered by:** `lidarPoseCB` / `lidarOdomCB` (on every incoming LiDAR message), then `lidarDetectionCB` timer.

### Steps

1. **Pass-through range filter** — keeps only points within `local_lidar_range` in X and Y (in the LiDAR frame).

2. **Gaussian distance-based downsampling** — for each point at distance `d` from the sensor, the probability of keeping it is `p = exp(-d² / (2σ²))` where `σ = gaussian_downsample_rate`. This mimics the real density of a rotating LiDAR: close points are denser and are kept more; far points are sparser and are thinned further.

3. **Transform to map frame** — using the current `orientationLidar_` / `positionLidar_` from the TF lookup.

4. **Vertical clip** — pass-through filter on Z: `[groundHeight_, roofHeight_]`. Ground and roof heights come from the wall detector.

5. **Wall point removal** — each point is tested against every wall OBB received from the wall detector (`isInsideAnyWall`). Points inside any wall are discarded.

6. **Adaptive voxel downsampling** — PCL `VoxelGrid` starting at 0.1 m leaf size, iteratively increasing the leaf size until the point count falls below `downsample_thresh`.

The result is stored in `lidarCloud_` and passed to `lidarDetect`.

---

## 7. Stage 2 — Depth camera detection (UV + DBSCAN)

**Triggered by:** `depthPoseCB` / `depthOdomCB` (stores depth image and camera pose), then `detectionCB` timer calls `dbscanDetect` and `uvDetect`.

### 7.1 Depth image unprojection (`projectDepthImage`)

Each pixel `(u, v)` is sampled at stride `skip_pixel` (skipping `depth_filter_margin` border pixels). Valid pixels (non-zero, in `[depth_min_value, depth_max_value]`) are unprojected to 3D using the depth camera intrinsics and transformed to the map frame:

```
ptCam = [(u - cx) * d / fx,  (v - cy) * d / fy,  d]
ptMap = orientationDepth * ptCam + positionDepth
```

Out-of-range pixels are treated as `raycast_max_length + 0.1` (flagged as "free space").

### 7.2 Voxel filtering of depth points (`voxelFilter`)

The unprojected point cloud is filtered using a 3D occupancy grid with resolution `voxel_size`. A point is included in the filtered cloud only when its voxel has accumulated at least `voxel_occ_thresh` hits. This removes single-pixel noise while preserving real obstacles.

Additional constraints applied at this stage:
- point must be above `groundHeight_`
- point must not be inside any wall OBB

The result is `filteredDepthPoints_`.

### 7.3 UV detection (`uvDetect`)

The UV detector (`uvDetector.cpp`) builds a 2D histogram of depth points projected onto the U (horizontal) and V (vertical) axes. Connected components in this histogram form candidate obstacle ROIs. For each ROI it estimates a 3D bounding box in the camera frame. `transformUVBBoxes` then converts these boxes to the map frame using the depth camera pose.

Result: `uvBBoxes_`.

### 7.4 Visual DBSCAN (`dbscanDetect`)

`filteredDepthPoints_` is clustered using DBSCAN with parameters `db_min_points_cluster` (minimum cluster size) and `db_epsilon` (neighborhood radius). Each cluster is fitted with an AABB. Clusters exceeding `max_object_size` in any dimension are discarded.

Optional DBSCAN refinement (`dbscan_refinement_enable`): large or low-density clusters can be recursively split along their longest axis using sub-clustering with tighter DBSCAN parameters. This separates adjacent people or vehicles that DBSCAN merged into one cluster.

Result: `dbBBoxes_` with associated `pcClustersVisual_`.

---

## 8. Stage 3 — LiDAR DBSCAN clustering

**Triggered by:** `lidarDetectionCB` timer calling `lidarDetect`.

The preprocessed `lidarCloud_` is clustered using the LiDAR-specific DBSCAN parameters (`lidar_db_min_points`, `lidar_db_epsilon`). Each cluster is fitted with an AABB. Clusters exceeding `max_object_size` are discarded.

Result: `lidarBBoxes_` with associated `lidarClusters_`.

---

## 9. Stage 4 — Bbox fusion (`filterLVBBoxes`)

This is the most complex stage. It runs inside `detectionCB` after depth detection and produces `filteredBBoxes_`, which feeds the tracker.

### Fusion tools: IOU and IOV

Two overlap metrics are used throughout:

- **IOU** (Intersection over Union) = `vol_intersection / vol_union` — measures how much two bboxes overlap symmetrically.
- **IOV** (Intersection over Volume) = `vol_intersection / vol_box1` — measures how much of `box1` is contained inside `box2`. This is asymmetric and detects nested/containment relationships even when IOU is low.

### BboxesMerger

`BboxesMerger` is the core merging function. It takes two groups of bboxes (group1 and group2), builds a bipartite overlap graph using IOU and IOV, and resolves it according to a merging strategy:

- **`"bigger"`** strategy: each connected subtree (parent + all children) is merged into one bbox whose AABB spans all constituent boxes. Used when a LiDAR cluster and a visual cluster represent the same object — you want the union.
- **`"smaller"`** strategy: only leaf nodes (the smallest boxes in a nested relationship) are kept. Used when UV detection produces a coarse box and DBSCAN produces a finer one — you prefer the finer one.

The `leaf_only` flag further restricts merging to cases where no additional parent-child chain exists.

Bboxes from either group that do not participate in any overlap relationship are passed through unchanged (leftover handling).

### `mergeNestedGroup`

Before cross-sensor merging, same-sensor bboxes are cleaned up. `mergeNestedGroup` runs inside each sensor branch to collapse boxes that are mutually overlapping (by IOU) or nested (by IOV) within the same detection frame. This prevents the same physical object from spawning two bboxes in one sensor.

The algorithm:
1. Compute pairwise IOU matrix.
2. Mutually best-matching pairs with IOU > threshold are merged immediately.
3. Remaining boxes are checked for IOV-based containment; a directed graph is built where the larger box is the parent.
4. Each connected tree is merged into a single AABB using `mergeBoxesSet`.
5. Isolated boxes pass through unchanged.

### Full fusion pipeline (step by step)

**Step 1 — UV vs visual DBSCAN merge**

UV bboxes (group1) and DBSCAN visual bboxes (group2) are merged with `BboxesMerger` using strategy `visual_merging_flag` (typically `"smaller"`) and thresholds `visual_box_iou_thresh` / `visual_box_iov_thresh`.

The flags `uv_unmerged` and `db_unmerged` control whether unmatched bboxes from each group are passed through. If `uv_unmerged = true`, UV detections that have no matching DBSCAN bbox are kept as-is (useful when the DBSCAN misses a close object). If `db_unmerged = true`, DBSCAN bboxes with no UV match are kept.

Result: `visualBBoxes_`.

**Step 2 — LiDAR nested merge**

`mergeNestedGroup` is applied to `lidarBBoxes_` to collapse redundant LiDAR clusters before cross-sensor fusion.

Result: `lidarBBoxesFiltered`.

**Step 3 — Visual vs LiDAR merge**

`visualBBoxes_` (group1) and `lidarBBoxesFiltered` (group2) are merged with `BboxesMerger` using strategy `lidar_visual_merging_flag` (typically `"bigger"`) and thresholds `lidar_visual_box_iou_thresh` / `lidar_visual_box_iov_thresh`.

Flags `visual_unmerged` and `lidar_unmerged` control passthrough of unmatched detections. `lidar_visual_merger_leaf_only` limits merging to leaf-only relationships.

Result: `filteredBBoxesTemp`.

**Step 4 (optional) — Final nested merge**

If `final_merge_flag = true`, a final `mergeNestedGroup` pass is applied to `filteredBBoxesTemp` to collapse any residual duplicates that survived the previous merge stages. This is useful when both sensor paths independently detected the same object and the cross-sensor merger did not collapse them (e.g. IOU was just below threshold).

Result is stored in `filteredBBoxesBeforeYolo_` for visualization, then proceeds to YOLO association.

---

## 10. Stage 5 — YOLO association and bbox resizing

Still inside `filterLVBBoxes`, after the fusion stages.

### Purpose

Match 2D YOLO detections to 3D fused bboxes and mark them as `is_yolo_candidate`. This flag activates privileged tracking (immediate confirmation, extended miss tolerance) and classification (always dynamic).

### Algorithm

For each YOLO detection of a target class:

1. **Map 2D ROI to depth image** — the YOLO rect is in color image coordinates. It is remapped to depth image coordinates using the depth/color intrinsic ratio:
   ```
   u_depth = fx/fxC * (u_color - cxC) + cx
   v_depth = fy/fyC * (v_color - cyC) + cy
   ```

2. **Build 3D point cloud from ROI** — each depth pixel inside the remapped ROI is unprojected to 3D world frame. Only pixels within `[depth_min_value, depth_max_value]` are used.

3. **Foreground filtering** — depths are sorted; the 10th-percentile depth is taken as the foreground reference. Only points within `yolo_depth_tolerance` metres of this reference are kept. This removes background pixels (e.g. wall behind the person) that happen to fall inside the 2D YOLO rect.

4. **Point-in-bbox check** — for each 3D fused bbox, count how many foreground YOLO points fall inside it. If the fraction exceeds `yolo_point_fraction_thresh`, the bbox is tagged `is_yolo_candidate = true`.

5. **Bbox resize** — the bbox base (x, y, x\_width, y\_width) is replaced with the tight AABB of the YOLO foreground points that fell inside. The z center and z\_width are preserved. The associated point cluster is re-filtered to stay inside the resized box.

This resize step is important: the original fused bbox often covers a wider area (LiDAR clusters are not tight around a human body). The YOLO foreground cloud gives a much tighter horizontal footprint. The resized bbox is what the tracker uses as the observation.

### YOLO result consumption

After processing, `yoloDetectionResults_` is cleared. The `is_yolo_candidate` flag is propagated forward by the tracker (sticky), so future frames do not need a fresh YOLO detection to maintain the human label.

---

## 11. Stage 6 — Tracking (zmin gate + box association + Kalman filter)

**Triggered by:** `trackingCB` timer.

### 11.1 Zmin gate

Before association, bboxes whose bottom face is too high above the ground are discarded:

```
zmin = bbox.z - bbox.z_width / 2
if zmin > groundHeight_ + track_max_zmin_above_ground  →  discard
```

This removes floating detections caused by reflections or LiDAR noise that appear well above floor level.

### 11.2 Box association (`boxAssociation`)

If no tracks exist yet, all current detections become new tracks. Otherwise `boxAssociationHelper` is called.

`boxAssociationHelper`:
1. Calls `getPredictedBBoxesFromFilters` — runs a Kalman predict step on each existing track to extrapolate its expected position.
2. Calls `getPreviousObservedBBoxes` — retrieves the last observed bbox from each track's history.
3. Calls `findBestMatch`.

### 11.3 Association score (`findBestMatch` / `computeAssociationScore`)

For each pair (current detection `i`, existing track `j`) a composite score is computed:

```
score = - w_pos      * predPosDist / (maxMatchRange * dt)
        - w_size     * relSizeDiff
        - w_iou2d    * (1 - predIou2d)
        - w_prevpos  * prevObsPosDist / (maxMatchRange * dt)
        - w_previou  * (1 - prevObsIou2d)
        - w_vel_dir  * velocityDirectionError
        + confirmed_track_assoc_bonus   (if track is confirmed)
        + yolo_track_assoc_bonus        (if track has YOLO evidence)
        - yolo_class_consistency_weight (if track is YOLO but detection is not)
```

Where:
- `predPosDist` — Euclidean distance between the KF-predicted position and the current detection center
- `relSizeDiff` — relative size difference between predicted and current bbox: `max(|Δx/x|, |Δy/y|, |Δz/z|)`
- `predIou2d` — 2D IoU (top-view) between predicted bbox and current detection
- `prevObsPosDist` — distance between the last *observed* (not predicted) position and the current detection
- `prevObsIou2d` — 2D IoU between last observed bbox and current detection
- `velocityDirectionError` — cosine dissimilarity between observed velocity direction and KF-predicted velocity direction

Hard gates applied before the score is computed:
- `predPosDist > maxMatchRange` → candidate rejected
- `relSizeDiff > maxRelativeSizeDiffMatch` → candidate rejected
- `!isNaturalMotion(j, currBBox)` → candidate rejected (see below)
- `!passesVelocityDirectionGate(j, currBBox)` → candidate rejected

The minimum acceptable score depends on track state:
- YOLO track: `min_match_score_dynamic` (most permissive)
- Confirmed non-YOLO track: `min_match_score_confirmed`
- Tentative track: `min_match_score`

Candidates are sorted by score and assigned greedily (one-to-one).

### 11.4 Natural motion gate (`isNaturalMotion`)

Checks that the displacement from the last observation to the current detection is physically plausible:

1. Observed displacement `obsDist` must be `>= min_natural_motion_dist` (rejects stationary clutter when `track_steady_objects = false`).
2. `obsDist` must be `<= max_natural_motion_dist` (rejects teleportations).
3. KF innovation (distance from KF prediction to current detection) must be `<= maxInnovation`, where:
   - YOLO tracks: `max_natural_innovation_dynamic`
   - Confirmed tracks: `max_natural_innovation_confirmed`
   - Tentative tracks: `max_natural_innovation`

### 11.5 Velocity direction gate (`passesVelocityDirectionGate`)

When the track has a meaningful velocity (above `stationary_speed_thresh`), the direction of the observed velocity must not deviate too far from the KF-predicted direction. Error is computed as `1 - cos(angle)`:

- Dynamic / YOLO tracks: `max_velocity_direction_error_confirm_dynamic` / `max_velocity_direction_error_tracked_dynamic` (very permissive)
- Confirmed non-YOLO tracks: `max_velocity_direction_error_tracked`
- Tentative tracks: `max_velocity_direction_error_confirm`

If both observed and predicted speeds are below `stationary_speed_thresh`, the error is 0 (gate passes).

### 11.6 Kalman filter update (`kalmanFilterAndUpdateHist`)

The Kalman filter used is **KF V2**: a constant-acceleration model with position-only observation.

State vector (6D): `[x, y, vx, vy, ax, ay]`

Transition matrix A (with timestep `dt`):
```
A = | 1  0  dt  0   dt²/2   0   |
    | 0  1   0  dt    0    dt²/2 |
    | 0  0   1   0   dt      0   |
    | 0  0   0   1    0     dt   |
    | 0  0   0   0    1      0   |
    | 0  0   0   0    0      1   |
```

Observation matrix H (2×6): observes only position.
```
H = | 1  0  0  0  0  0 |
    | 0  1  0  0  0  0 |
```

This means velocity and acceleration are estimated internally by the filter from position measurements only. No finite-difference velocity is injected as an observation (avoids double-counting noisy velocity estimates).

Noise parameters (from YAML `kalman_filter_v2_param: [eP, eQPos, eQVel, eQAcc, eRPos]`):
- `eP` — initial state covariance scalar
- `eQPos`, `eQVel`, `eQAcc` — process noise for position, velocity, acceleration
- `eRPos` — observation noise for position

**Matched tracks** (detection found): KF `estimate` (predict + update) is called. The KF output provides `(x, y, Vx, Vy, Ax, Ay)`. The bbox geometry (width, height, z) comes from the current detection, not from the KF (KF only tracks 2D position + velocity).

**Unmatched tracks** (no detection): KF `predict` is called. The predicted bbox is stored in history and used for future association, but is **never** added to the output `trackedBBoxes_`. No coasting bboxes appear in the output — only real detections that matched a track.

**Unmatched detections** (no track): a new track is spawned if the detection is not too close to an existing track (`isCloseToExistingTrack`) and not a duplicate of another bbox already added this frame (`isDuplicateOfTrackedThisFrame`).

### 11.7 Track confirmation (`shouldConfirmTrack`)

A track enters the output `trackedBBoxes_` only once it is confirmed. Confirmation rules:

- **YOLO detection**: confirmed immediately on the first frame. No consecutive hits required.
- **Non-YOLO detection**: requires `min_confirm_hits` consecutive matched frames AND:
  - observed speed ≥ `stationary_speed_thresh` (unless `track_steady_objects = true`)
  - passes `passesVelocityDirectionGate`
  - if outside camera FOV: requires the mean Kalman speed over history ≥ `dynaVelThresh` AND coefficient of variation ≤ `dynaKfVelStdRatio` (rejects LiDAR jitter that produces apparent single-frame motion)

Once confirmed, a track **stays confirmed** until it dies from exceeding `maxMissedFrames`.

### 11.8 Stationary suppression

A confirmed non-YOLO track that is stationary (KF speed < `stationary_speed_thresh`) is suppressed from `trackedBBoxes_`. It remains alive internally (history and KF are maintained) so it can re-enter the output immediately when it starts moving. YOLO tracks bypass this suppression (people standing still are still obstacles).

### 11.9 YOLO flag lifecycle

The `is_yolo_candidate` flag on a track follows these rules:

1. **Set**: when `filterLVBBoxes` detects enough YOLO foreground points inside the bbox.
2. **Sticky propagation**: each frame `newEstimatedBBox.is_yolo_candidate = currDetected.is_yolo_candidate || prev_yolo_cand`. This keeps the flag alive between YOLO frames.
3. **FOV decay** (`nonYoloInFovStreak`): if the track is inside the camera FOV and matched a non-YOLO detection for `max_non_yolo_in_fov_frames` consecutive frames, the flag is cleared. This prevents a wall or static object from permanently inheriting a YOLO label from a coincidental overlap.
4. **Size mismatch decay** (`yoloBaseMismatchStreak`): only active **outside** the camera FOV. The current bbox width is compared to the maximum YOLO-derived width seen along the track. If the bbox has grown by more than `yolo_base_mismatch_thresh` (150% by default) for `max_yolo_base_mismatch_frames` consecutive frames, the flag is cleared. Only growth is checked, not shrinkage — outside the FOV only LiDAR is available, so bboxes are naturally smaller than when depth camera data is also included.

### 11.10 Track termination

- Non-YOLO tracks: dropped after `max_missed_frames` consecutive unmatched frames.
- YOLO tracks: dropped after `max_missed_frames_yolo` consecutive unmatched frames (typically 3× longer).

---

## 12. Stage 7 — Dynamic classification

**Triggered by:** `classificationCB` timer, runs after tracking.

Classification runs only on tracks that appear in `trackedBBoxes_` (confirmed, non-suppressed). This ensures every dynamic bbox corresponds to a visible tracked bbox.

### Case I — YOLO-certified tracks

If `boxHist_[i][0].is_yolo_candidate` is true: the track is always classified dynamic. `is_dynamic = true` is written to history so the force-dynamic check (Case III) can count it.

### Case II — Insufficient history

If the track has fewer than `frame_skip + 1` history frames, the available frames are used instead.

### Case III — Force dynamic

If the track was classified dynamic for ≥ `frames_force_dynamic` out of the last `frames_force_dynamic_check_range` frames AND the current Kalman speed ≥ `dynamic_velocity_threshold`, the track is forced dynamic. This maintains the dynamic label for an object that has been consistently moving even if the current frame's point cloud evidence is weak (e.g. briefly occluded).

A non-YOLO track that stops moving loses the force-dynamic label: the velocity check prevents a parked car from being classified dynamic just because it was moving earlier.

### Case IV — Point cloud voting

The standard case. For each point in the current cluster, its velocity relative to the cluster `frame_skip` frames ago is computed. If the speed exceeds `dynamic_velocity_threshold`, it casts a "dynamic" vote. If the vote fraction exceeds `dynamic_voting_threshold`, the track is classified dynamic.

Additionally, a **Kalman velocity quality check** is applied: if the Kalman filter has consistently estimated speed above threshold with low relative standard deviation (`dynaKfVelStdRatio`) over the last `frame_skip` frames, the dynamic label is granted even without sufficient point votes. This handles the outside-FOV case where the LiDAR point cloud is sparse and point voting is unreliable.

### Output

`dynamicBBoxes_` contains bboxes classified as dynamic this frame. These are a strict subset of `trackedBBoxes_`.

---

## 13. Track lifecycle summary

```
Detection arrives in filteredBBoxes_
       │
       ▼
No existing track?  ──► New track spawned (tentative, hit=1)
       │                 YOLO detection: immediately confirmed + in output
       │                 Non-YOLO: tentative, not in output yet
       │
       ▼
Existing track found (best match above score threshold)?
       │
       ├── YES ──► KF estimate (predict + update)
       │           hitStreak++, missedFrames = 0
       │           shouldConfirmTrack?
       │             YES: add to trackedBBoxes_ (unless stationary non-YOLO)
       │             NO:  keep tentative
       │
       └── NO  ──► KF predict only (no observation)
                   missedFrames++
                   > maxMissedFrames (or maxMissedFramesYolo)?
                     YES: track deleted
                     NO:  track kept alive internally, not in output

trackedBBoxes_ → classificationCB → dynamicBBoxes_
```

---

## 14. Key data structures

### `box3D`

Core bbox type used throughout:

| Field | Description |
|-------|-------------|
| `x, y, z` | Center position in map frame |
| `x_width, y_width, z_width` | Full side lengths |
| `Vx, Vy` | Velocity from KF (map frame) |
| `Ax, Ay` | Acceleration from KF |
| `id` | Unique track ID (monotonically increasing) |
| `is_yolo_candidate` | True if YOLO has confirmed this as a human |
| `is_dynamic` | True if classificationCB classified it as dynamic |

### `boxHist_`

`std::vector<std::deque<box3D>>` — one deque per track, most recent frame at front. Maximum depth = `hist_size`. Used for classification (point voting over time), force-dynamic counting, and natural motion gating.

### `pcHist_`, `pcCenterHist_`

Parallel histories of the raw point cloud cluster and cluster centroid for each track. Used for point-level dynamic classification.

### `filters_`

`std::vector<kalman_filter>` — one KF per track. Each filter holds state `[x, y, vx, vy, ax, ay]` and its covariance matrix P.

### `missedFrames_`, `hitStreak_`, `confirmedTracks_`

Per-track scalars maintaining the confirmation and termination logic.

### `yoloXWidth_`, `yoloYWidth_`, `yoloBaseMismatchStreak_`

Per-track YOLO size baseline (maximum YOLO-derived dimensions seen along the track) and consecutive mismatch counter for the ID-swap guard.

---

## 15. Configuration reference

All parameters live in `cfg/detector_param_jo_zotac.yaml`. Key groups:

### Sensor setup
```yaml
depth_image_topic, color_image_topic, lidar_pointcloud_topic
tf_map_frame, tf_lidar_frame, tf_depth_frame, tf_color_frame
depth_intrinsics: [fx, fy, cx, cy]
color_intrinsics: [fxC, fyC, cxC, cyC]
depth_scale_factor, depth_min_value, depth_max_value
```

### Point cloud filtering
```yaml
voxel_size              # depth voxel grid resolution
voxel_occ_thresh        # min hits per voxel to keep point
depth_filter_margin     # border pixels to skip
skip_pixel              # pixel stride for depth unprojection
local_sensor_range      # [x, y, z] max range for depth points
local_lidar_range       # [x, y] max range for LiDAR
gaussian_downsample_rate # sigma for LiDAR distance downsampling
downsample_thresh        # max LiDAR points after downsampling
```

### DBSCAN
```yaml
db_min_points_cluster   # depth DBSCAN min points
db_epsilon              # depth DBSCAN neighborhood radius
lidar_db_min_points     # LiDAR DBSCAN min points
lidar_db_epsilon        # LiDAR DBSCAN neighborhood radius
dbscan_refinement_enable / visual_dbscan_refinement_enable
```

### Fusion
```yaml
visual_box_iou_thresh, visual_box_iov_thresh
lidar_visual_box_iou_thresh, lidar_visual_box_iov_thresh
samegroup_iou_thresh, samegroup_iov_thresh
visual_merging_flag     # "smaller" or "bigger"
lidar_visual_merging_flag
uv_unmerged, db_unmerged, visual_unmerged, lidar_unmerged
final_merge_flag
```

### YOLO
```yaml
yolo_point_fraction_thresh   # fraction of YOLO points needed inside bbox
yolo_depth_tolerance         # background removal tolerance (metres)
yolo_dynamic_classes         # list of classes treated as dynamic
```

### Tracking
```yaml
kalman_filter_v2_param: [eP, eQPos, eQVel, eQAcc, eRPos]
max_missed_frames            # frames before non-YOLO track dies
max_missed_frames_yolo       # frames before YOLO track dies
max_match_range              # max association distance
min_match_score              # score threshold for tentative tracks
min_match_score_confirmed    # score threshold for confirmed tracks
min_match_score_dynamic      # score threshold for YOLO tracks
min_natural_motion_dist      # min displacement to count as motion
max_natural_motion_dist      # max displacement (teleportation gate)
max_natural_innovation       # max KF innovation for tentative tracks
max_natural_innovation_confirmed
max_natural_innovation_dynamic
stationary_speed_thresh      # speed below which track is considered still
track_steady_objects         # if false, stationary detections are rejected
min_confirm_hits             # consecutive hits required to confirm non-YOLO
confirmed_track_assoc_bonus  # score bonus for matching confirmed track
yolo_track_assoc_bonus       # score bonus for matching YOLO track
track_max_zmin_above_ground  # max bbox bottom height above ground
```

### YOLO flag decay
```yaml
max_non_yolo_in_fov_frames   # FOV-based decay counter limit
max_yolo_base_mismatch_frames # size-mismatch decay counter limit
yolo_base_mismatch_thresh    # relative growth threshold for ID-swap detection
```

### Dynamic classification
```yaml
frame_skip                   # history gap for point velocity voting
dynamic_velocity_threshold   # min speed to count as dynamic vote
dynamic_voting_threshold     # min vote fraction to classify dynamic
dynamic_kf_vel_std_ratio     # max std/mean ratio for KF velocity quality check
frames_force_dynamic         # min dynamic frames in window to force dynamic
frames_force_dynamic_check_range # window size for force-dynamic check
dynamic_consistency_threshold
```
