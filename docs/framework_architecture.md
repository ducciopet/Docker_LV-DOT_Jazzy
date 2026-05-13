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

The pipeline is **timer-driven**, not callback-driven. The synchronized sensor callbacks only update shared state buffers; five independent wall timers at `time_step` (default 0.1 s = 10 Hz) consume those buffers and run the actual detection stages.

```
SENSOR CALLBACKS (update shared state, fire at sensor rate)
─────────────────────────────────────────────────────────────────────
ApproximateTime(depth, odom)  ──► depthOdomCB()
                                    store depthImage_, camera pose

ApproximateTime(LiDAR, odom)  ──► lidarOdomCB()
                                    range-filter, Gaussian downsample,
                                    transform to world frame, wall-filter
                                    → store lidarCloud_, lidar pose

color image  ──────────────────► colorImgCB()     store detectedColorImage_
YOLO 2D      ──────────────────► yoloDetectionCB() store yoloDetectionResults_
ground/walls ──────────────────► groundHeightCB()  store groundHeight_, roofHeight_, wallBBoxes_


TIMER PIPELINE (fire every time_step = 0.1 s)
─────────────────────────────────────────────────────────────────────
detectionTimer_      ──► detectionCB()
                             dbscanDetect()     depth → DBSCAN clusters
                             uvDetect()         depth → UV occupancy boxes
                             filterLVBBoxes()   fuse UV+DBSCAN+LiDAR + YOLO assoc

lidarDetectionTimer_ ──► lidarDetectionCB()
                             lidarDetect()      lidarCloud_ → DBSCAN clusters

trackingTimer_       ──► trackingCB()
                             kalmanFilterAndUpdateHist()
                               predict · associate · update / create / drop tracks

classificationTimer_ ──► classificationCB()
                             PATH 1: YOLO candidate → speed gate
                             PATH 2: in FOV → point-cloud motion voting
                             PATH 3: outside FOV → kinematics-only

visTimer_            ──► visCB()
                             publish MarkerArrays + ObstacleArray
```

The decoupling between sensor callbacks and processing timers means that the detection pipeline always operates on the **latest available** data from each sensor, without needing to triple-synchronize depth, LiDAR, and odometry in a single message filter. The odom/pose is paired independently with each sensor to ensure each buffer is stamped with the correct robot pose at the time of acquisition.

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

`detector_node` instantiates `dynamicDetector` and wires its inputs to ROS subscriptions and timers. There is no single monolithic callback: sensor data arrives through two independent `ApproximateTime` synchronizers and is buffered in shared members; five wall timers at `time_step` (0.1 s) then consume that data in pipeline order:

```
Sensor callbacks (update shared state only):
  depthOdomCB()        ← ApproximateTime(depth, odom)   → depthImage_, camera pose
  lidarOdomCB()        ← ApproximateTime(LiDAR, odom)   → lidarCloud_, lidar pose
  colorImgCB()         ← color image topic              → detectedColorImage_
  yoloDetectionCB()    ← YOLO detections topic          → yoloDetectionResults_
  groundHeightCB()     ← wall_detector topic            → groundHeight_, roofHeight_, wallBBoxes_

Timer callbacks (drive the pipeline at time_step):
  detectionCB()        → dbscanDetect() + uvDetect() + filterLVBBoxes()
  lidarDetectionCB()   → lidarDetect()
  trackingCB()         → kalmanFilterAndUpdateHist()
  classificationCB()   → PATH 1/2/3 classification
  visCB()              → publish MarkerArrays + ObstacleArray
```

The odom message is paired independently with each sensor so each buffer carries the correct robot pose at the time of acquisition, without needing to triple-synchronize depth, LiDAR, and odometry in a single message filter.

---

## 5. UV detection (uvDetect)

### Concept

The UV detector builds a **U-disparity map** from the depth image — a 2D histogram where each column corresponds to an image column range and each row corresponds to a depth bin. A cell is "hot" when many pixels in that image-column band have depth falling in that bin. Contiguous hot runs within a row are segmented and merged across rows to form image-space bounding boxes, which are then back-projected to 3D in the camera frame. The approach avoids full 3D point cloud construction; it operates entirely on the raw 16-bit depth image.

### Algorithm

1. **U-map construction (`extract_U_map`).** The depth image is scanned column by column. For each pixel with a valid depth in `[min_dist, max_dist]` mm, the depth is quantised into one of `histSize = depth.rows / row_downsample` bins, and the corresponding U-map cell `(col / col_scale, bin)` is incremented. A `(5, 9)` Gaussian blur (σ = 10) is applied to smooth the histogram.

2. **Bounding-box extraction (`extract_bb`).** Each row of the U-map is scanned left-to-right for runs of cells whose count exceeds `u_min = threshold_point × row_downsample`. Each run becomes a `UVbox` segment. Vertically adjacent segments in consecutive rows that overlap horizontally are merged into the same parent using union-find. After merging, boxes with area < 25 cells are discarded. The result is a list of `bounding_box_U` rectangles in U-map space `(col_start, col_end, bin_start, bin_end)`.

3. **3D bounding-box extraction (`extract_3Dbox`, called from `uvDetect`).** For each U-map bbox:
   - The depth range `[depth_near, depth_far]` is recovered from the bin indices.
   - The image columns `[x_left, x_right]` are recovered from the col indices (× `col_scale`).
   - The vertical extent `[y_up, y_down]` is found by scanning those depth-image columns for the topmost and bottommost pixels whose depth falls in `[depth_near, depth_far]`, using a forward-look of `num_check = 15` consecutive rows to suppress isolated noise.
   - The 3D center in the camera frame is computed via the pinhole back-projection: `X = (col_center − cx) × z / fx`, `Y = (row_center − cy) × z / fy`, `Z = (depth_near + depth_far) / 2`, with depth converted from mm to m.
   - Width and height are derived from the pixel extents scaled by `z / focal_length`.

4. **Bird's-eye visualization (`extract_bird_view`).** Converts U-map bboxes to top-down rectangles for RViz display. This output is **not** used by the 3D detection pipeline.

### Notes

- There is **no ground/roof filtering** inside the UV detector itself; height gating is applied later in `filterLVBBoxes`.
- The 3D boxes come out in the **camera frame**, not the world frame; `filterLVBBoxes` transforms them via the stored camera-to-world pose.
- The UV detector is complementary to DBSCAN: it is faster and works on the raw depth image, but can merge objects at similar depth that are separated horizontally, or split a single object that spans multiple depth bins.

---

## 6. Visual DBSCAN (dbscanDetect)

### Concept

The visual DBSCAN detector projects the depth image to a full 3D point cloud in world space and clusters it with DBSCAN. Unlike the UV detector, which only captures per-column depth histograms, DBSCAN works on individual 3D points and can resolve spatially separated objects at the same depth, distinguish objects with irregular or elongated shapes, and leverage 3D density information for post-processing refinement.

### Algorithm

1. **Depth projection (`projectDepthImage`).** Every valid depth pixel (in `[depth_min_value, depth_max_value]`) is back-projected to the camera frame using the pinhole model, then transformed to the world frame via the stored camera pose (`orientationDepth_`, `positionDepth_` from the last `depthOdomCB`). Points beyond `local_sensor_range` in X or Y are rejected early.

2. **Voxel filtering (`voxelFilter`).** The projected cloud is discretised into voxels of side `voxel_size`. A voxel is kept only when it accumulates at least `voxel_occupied_thresh` raw points — this simultaneously down-samples the cloud and rejects isolated noisy readings without discarding coherent but low-density returns.

3. **Ground/roof and wall filtering (`filterPoints`).** After voxel filtering, points outside `[groundHeight_, roofHeight_]` in Z are discarded, and points that fall inside any known wall OBB (`isInsideAnyWall`) are removed. Both thresholds are updated live from `wall_detector_node`.

4. **DBSCAN (`clusterPointsAndBBoxes`).** Standard DBSCAN runs on the filtered cloud:
   - `dbscan_search_range_epsilon` — neighbourhood radius in world-frame metres
   - `dbscan_min_points_cluster` — minimum points to form a core
   Points not assigned to any cluster (noise) are discarded.

5. **Cluster refinement (optional, `visual_dbscan_refinement_enable`).** After DBSCAN, each raw cluster is processed by `refineClusterRecursive`:
   - A cluster is a **split candidate** when its XY diagonal > `dbscan_refine_max_diagonal` **and** its point density (pts / volume) < `dbscan_refine_min_density`. The diagonal is computed only in XY to avoid height variation triggering false splits.
   - The split is first attempted with a tighter DBSCAN pass (`dbscan_refine_split_eps`, `dbscan_refine_split_min_pts`). If that yields fewer than 2 sub-clusters, the algorithm falls back to **axis-slicing**: the longest axis is divided into slices of width `dbscan_refine_axis_slice_width`; contiguous non-empty slices are merged into sub-clusters.
   - Sub-clusters below `dbscan_refine_min_subcluster_pts` points are discarded. If `dbscan_refine_recursive: true`, each surviving sub-cluster is evaluated again recursively up to `dbscan_refine_max_depth` levels.
   - Sub-clusters whose bounding-box volume is below `dbscan_refine_min_box_volume` are clamped to that minimum when computing density (prevents division by near-zero for flat clusters).

   **Why refinement matters indoors.** In indoor environments the depth camera's ~5 m range covers the entire room, and objects like furniture, people near walls, or clustered items are spatially close. With a broad epsilon (needed to bridge depth noise), DBSCAN readily merges adjacent objects into one large, sparse super-cluster. Refinement detects these "merged blobs" — large diagonal + low density — and cleanly splits them before they enter the tracker. Outdoors, objects are generally well-separated and the depth range is shorter than the scene, so the merging problem is much rarer; refinement can be disabled (`visual_dbscan_refinement_enable: false`) to save CPU.

6. **Bounding box extraction.** For each final cluster, the AABB (min/max XYZ) is computed and stored as a `box3D`.

---

## 7. LiDAR detection (lidarDetect)

### Concept

The LiDAR provides a 360° scan that extends far beyond the depth camera's range (typically 20–30 m vs 5 m). The LiDAR pipeline runs in two stages: a **preprocessing callback** (`lidarOdomCB`) that fires on every synchronised (LiDAR, odom) pair and stores a clean world-frame cloud, and a **detection function** (`lidarDetect`) called by the timer pipeline that clusters the stored cloud.

### Preprocessing in `lidarOdomCB`

1. **XY range filtering.** A `pcl::PassThrough` filter retains only points within `±local_lidar_range.x` and `±local_lidar_range.y` of the sensor origin. This caps the computational load and discards background returns beyond the region of interest.

2. **Gaussian distance-based downsampling.** Each point surviving the XY filter is accepted with probability `p = exp(−dist² / (2σ²))` where σ = `gaussian_down_sample_rate` and `dist` is the planar distance to the sensor. This keeps a dense representation near the robot (where small/fast objects need precise boundaries) while thinning the far-field (where DBSCAN only needs to capture the cluster's bulk). Without this step, the near field would have orders of magnitude more points than the far field, making a single epsilon value poorly suited across the full range.

3. **World-frame transform.** The downsampled cloud is transformed from the LiDAR frame to the world frame using `orientationLidar_` and `positionLidar_` (extracted from the synchronized odometry).

4. **Ground/roof and wall filtering.** A Z pass-through filter applies `[groundHeight_, roofHeight_]`, then `isInsideAnyWall` removes points inside known wall OBBs.

5. **Adaptive VoxelGrid downsampling.** A `pcl::VoxelGrid` is applied to the height- and wall-filtered cloud starting with a 0.1 m leaf size. If the point count still exceeds `downsample_threshold` (default 4000), the leaf size is multiplied by 1.1 and the filter is re-run, repeating until the count falls below the threshold. This is the most important computational safeguard in the preprocessing chain: it provides a hard upper bound on the number of points that DBSCAN will ever see, guaranteeing bounded execution time regardless of scene density. Without it, a dense near-range scan or a cluttered environment could inflate the cloud by an order of magnitude and stall real-time processing.

6. **Storage.** The resulting cloud is stored as `lidarCloud_` for the timer pipeline.

### Detection in `lidarDetect`

1. **DBSCAN.** The stored `lidarCloud_` is clustered with `lidar_DBSCAN_epsilon` and `lidar_DBSCAN_min_points`. A larger epsilon than the visual DBSCAN is typical (e.g. 0.25 m vs 0.15 m) because spinning LiDARs have sparse angular sampling at range — consecutive scan lines can be centimetres apart on a 10 m object.

2. **Cluster refinement (optional, `dbscan_refinement_enable`).** The same `refineClusterRecursive` logic used by the visual pipeline is applied here. LiDAR shares the full set of refinement parameters with the visual path; the only difference is that visual DBSCAN has an independent override flag (`visual_dbscan_refinement_enable`) while LiDAR is gated solely by `dbscan_refinement_enable`. Indoors, LiDAR refinement is particularly effective: the 360° scan captures both sides of a doorway, a room corner, or two people walking together, and DBSCAN's epsilon easily bridges their gap. Refinement splits these merged blobs before they corrupt track geometry. Outdoors, objects are more spread out and the far-field angular sparsity naturally prevents merging, so refinement adds little benefit at the cost of more computation.

3. **Max-object-size filter.** After DBSCAN + refinement, each cluster whose bounding box exceeds `max_object_size [x, y, z]` in any axis is discarded. This removes walls, trees, and other large static structures that DBSCAN merges into single clusters before any further stage can handle them.

3. **Bounding box storage.** Surviving clusters and their AABBs are stored as `lidarBBoxes_` and `lidarClusters_` for the fusion stage.

LiDAR clusters are the backbone of the far-field detection pipeline. They are typically sparser than depth clusters but provide reliable coverage of vehicles and pedestrians well beyond the depth camera's range.

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
