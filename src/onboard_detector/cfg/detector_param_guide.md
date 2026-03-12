# Parameter Configuration Guide for detector_param_jo_zotac.yaml

This guide explains each parameter in the configuration file and how to set it for your use case.

---

## detector_node > onboard_detector

### Localization & Topics
- **localization_mode**: 0 = use /pose, 1 = use /odom for localization.
- **depth_image_topic, color_image_topic, lidar_pointcloud_topic**: ROS topics for depth, color, and LiDAR data.
- **pose_topic, odom_topic**: Topics for robot pose and odometry.
- **use_tf_pose**: If true, use TF for pose; otherwise, use topic.
- **tf_map_frame, tf_lidar_frame, tf_depth_frame, tf_color_frame**: Frame names for TF tree.
- **urdf_camera_frame**: Camera frame from URDF.
- **camera_frame_initial_guess**: Initial guess frame for calibration.
- **output_detected_image_topic, output_detected_bboxes_topic, output_obstacles_topic**: Output topics for results.

### Camera Parameters
- **depth_intrinsics, color_intrinsics**: Camera calibration parameters [fx, fy, cx, cy].
- **depth_scale_factor**: Scale for converting depth units (e.g., 1000 for mm to m).
- **depth_min_value, depth_max_value**: Min/max valid depth (meters).
- **depth_filter_margin, depth_skip_pixel**: Filtering and downsampling for depth images.
- **image_cols, image_rows**: Image dimensions.

### System
- **time_step**: Processing time step (seconds).

### DBSCAN (Clustering)
- **ground_height, roof_height**: Remove points below/above these heights.
- **voxel_occupied_thresh**: Min points for a voxel to be considered occupied.
- **dbscan_min_points_cluster**: Min points for a cluster.
- **dbscan_search_range_epsilon**: DBSCAN search radius.
- **lidar_DBSCAN_min_points, lidar_DBSCAN_epsilon**: DBSCAN params for LiDAR.
- **downsample_threshold**: Point count threshold for downsampling.
- **gaussian_downsample_rate**: Downsampling rate.

### Voxel Filtering
- **voxel_size**: (Tunable) Size (in meters) of the voxel grid used for downsampling before DBSCAN clustering. Set this in the YAML to control the spatial resolution of the voxel filter. Lower values preserve more detail but increase computation. Typical: 0.05–0.2. Example: `voxel_size: 0.1`.

### LiDAR Visual Filtering
- **filtering_BBox_IOU_threshold**: IOU threshold for bounding box filtering.

### Tracking & Data Association
- **max_match_range, max_size_diff_range**: Max allowed difference for matching objects.
- **feature_weight**: Weights for position, size, and centroid in matching.
- **history_size**: Number of frames to keep in history.
- **fix_size_history_threshold, fix_size_dimension_threshold**: Thresholds for fixing object size.
- **kalman_filter_param, kalman_filter_averaging_frames**: Kalman filter settings.

### Classification
- **frame_skip**: Frames to skip when comparing point clouds.
- **dynamic_velocity_threshold**: Min velocity to consider object dynamic.
- **dynamic_voting_threshold**: Min score to classify as dynamic.
- **frames_force_dynamic, frames_force_dynamic_check_range**: History range for dynamic classification.
- **dynamic_consistency_threshold**: Frames required for consistent dynamic classification.

### Size Constraints
- **target_constrain_size**: Enable/disable size constraints.
- **target_object_size**: Expected object size [x, y, z] (meters). If you want to detect multiple object types (e.g., humans and cars), set this to the typical size of your primary target, or consider extending the code to support a list of sizes or class-specific constraints. For multi-class detection, you may need to relax this constraint or implement logic to handle different object sizes for each class.
- **max_object_size**: Max allowed object size [x, y, z] (meters).

---

## calibration_icp_node
- **depth_topic, velodyne_topic**: Input topics for depth and LiDAR.
- **lidar_frame, urdf_camera_frame, camera_frame_initial_guess, refined_camera_frame**: Frame names for calibration.
- **icp_max_correspondence_distance**: Max distance for ICP point matching.
- **icp_max_iteration**: Max ICP iterations.
- **icp_voxel_size**: Voxel size for downsampling.
- **icp_use_point_to_plane**: Use point-to-plane ICP.
- **icp_normal_radius, icp_normal_max_nn**: Normal estimation params.
- **min_overlap_points**: Min points for overlap.
- **overlap_fov_margin_px**: Margin in pixels for overlap.
- **onboard_detector**: Nested camera parameters (see above).

---

## yolov11_detector_node
- **image_topic**: Input image topic.
- **detected_image_topic, detected_bboxes_topic, inference_time_topic**: Output topics.
- **weights_path, class_names_path**: Paths to YOLO weights and class names.
- **target_classes**: List of classes to detect.
- **timer_period_sec**: Inference period (seconds).
- **inference_size**: Input size for YOLO model.
- **queue_size**: ROS message queue size.

---

# How to Configure
1. **Set topic and frame names** to match your robot setup.
2. **Tune clustering and filtering** for your environment (DBSCAN, voxel, filtering params).
3. **Adjust tracking and classification** thresholds for your use case.
4. **Set camera intrinsics** from calibration.
5. **For YOLO, set correct weights and class names.**

If you need more details on any parameter, ask for a deep dive on that section.
