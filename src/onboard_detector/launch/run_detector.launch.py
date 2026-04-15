#!/usr/bin/env python3

import os
import yaml

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory

# =============================================================================
# Shared sensor configuration — single source of truth.
# These values override whatever is written in the YAML file, so editing here
# is sufficient to update all nodes at once.
# =============================================================================
SHARED = {
    # Topics
    'depth_topic':       '/front_camera/camera/depth/image_rect_raw',
    'color_topic':       '/front_camera/camera/color/image_raw',
    'lidar_topic':       '/velodyne_points',
    # TF frames
    'map_frame':         'map',
    'lidar_frame':       'velodyne',
    'depth_frame':       'camera_refined',
    # Depth camera intrinsics & parameters
    'depth_intrinsics':  [436.9647521972656, 436.9647521972656, 431.9205627441406, 240.13380432128906],
    'depth_scale':       1000.0,
    'depth_min':         0.5,
    'depth_max':         4.5,
    'depth_skip':        2,
    # Ground estimation
    'ground_height':      -0.7,
    'ground_roof_offset':  5.0,
    # YOLO target classes (must match between detector_node and yolov11_detector_node)
    'yolo_classes': ['person', 'car', 'bus', 'truck', 'motorbike', 'bicycle',
                     'dog', 'cat', 'horse', 'cow', 'sheep'],
}


def _load_node_params(config_path, node_name):
    try:
        with open(config_path, 'r', encoding='utf-8') as cfg:
            content = yaml.safe_load(cfg) or {}
        return content.get(node_name, {}).get('ros__parameters', {})
    except Exception:
        return {}


def generate_launch_description():

    pkg_dir = get_package_share_directory('onboard_detector')
    config_file = os.path.join(pkg_dir, 'cfg', 'detector_param_jo_zotac.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'detector_jo_zotac.rviz')

    calibration_params = _load_node_params(config_file, 'calibration_icp_node')
    camera_frame = str(calibration_params.get('urdf_camera_frame', 'camera'))
    camera_frame_initial_guess = str(calibration_params.get('camera_frame_initial_guess', 'camera_initial_guess'))

    scripts_dir = os.path.join(pkg_dir, 'scripts')
    yolo_dir = os.path.join(scripts_dir, 'yolo_detector')

    pythonpath_action = SetEnvironmentVariable(
        'PYTHONPATH',
        yolo_dir + os.pathsep + scripts_dir + os.pathsep + os.environ.get('PYTHONPATH', '')
    )

    # Initial Guess TF Obtained manually from a previous calibration ICP node
    static_tf_velodyne_to_camera_link_initial_guess = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='calib_icp_tf_velodyne_to_camera_link_initial_guess',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '--x', '0.218304037',
                '--y', '0.107423631',
                '--z', '-0.020496928',
                '--roll', '-1.576042033',
                '--pitch', '-0.037013778',
                '--yaw', '-1.578200049',
                '--frame-id', SHARED['lidar_frame'],
                '--child-frame-id', camera_frame_initial_guess
            ]
    )

    # /tf from velodyne to urdf camera frame (camera_link)
    static_tf_velodyne_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='detector_tf_velodyne_to_camera_link',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--x', '0.27639',
            '--y', '-0.01446',
            '--z', '-0.35199',
            '--roll', '-1.5707963267948966',
            '--pitch', '0.0',
            '--yaw', '-1.5707963267948966',
            '--frame-id', SHARED['lidar_frame'],
            '--child-frame-id', camera_frame
        ]
    )

    calibration_node = Node(
        package='onboard_detector',
        executable='calibration_icp_node',
        name='calibration_icp_node',
        output='screen',
        parameters=[
            ParameterFile(config_file, allow_substs=True),
            {
                'depth_topic':                          SHARED['depth_topic'],
                'velodyne_topic':                       SHARED['lidar_topic'],
                'lidar_frame':                          SHARED['lidar_frame'],
                'onboard_detector.depth_intrinsics':    SHARED['depth_intrinsics'],
                'onboard_detector.depth_scale_factor':  SHARED['depth_scale'],
                'onboard_detector.depth_min_value':     SHARED['depth_min'],
                'onboard_detector.depth_skip_pixel':    SHARED['depth_skip'],
                # depth_max_value is intentionally NOT shared: calibration uses 5.0
                # (slightly larger range to catch edge points), detector uses 4.5
                'use_sim_time': True,
            },
        ],
    )

    detector_node = Node(
        package='onboard_detector',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[
            ParameterFile(config_file, allow_substs=True),
            {
                'onboard_detector.depth_image_topic':      SHARED['depth_topic'],
                'onboard_detector.color_image_topic':      SHARED['color_topic'],
                'onboard_detector.lidar_pointcloud_topic': SHARED['lidar_topic'],
                'onboard_detector.tf_map_frame':           SHARED['map_frame'],
                'onboard_detector.tf_lidar_frame':         SHARED['lidar_frame'],
                'onboard_detector.tf_depth_frame':         SHARED['depth_frame'],
                'onboard_detector.tf_color_frame':         SHARED['depth_frame'],
                'onboard_detector.depth_intrinsics':       SHARED['depth_intrinsics'],
                'onboard_detector.depth_scale_factor':     SHARED['depth_scale'],
                'onboard_detector.depth_min_value':        SHARED['depth_min'],
                'onboard_detector.depth_max_value':        SHARED['depth_max'],
                'onboard_detector.depth_skip_pixel':       SHARED['depth_skip'],
                'onboard_detector.ground_height':          SHARED['ground_height'],
                'onboard_detector.ground_roof_offset':     SHARED['ground_roof_offset'],
                'onboard_detector.yolo_dynamic_classes':   SHARED['yolo_classes'],
                'use_sim_time': True,
            },
        ],
    )

    wall_detector_node = Node(
        package='onboard_detector',
        executable='wall_detector_node',
        name='wall_detector_node',
        output='screen',
        parameters=[
            ParameterFile(config_file, allow_substs=True),
            {
                'wall_detector.lidar_topic':        SHARED['lidar_topic'],
                'wall_detector.map_frame':          SHARED['map_frame'],
                'wall_detector.lidar_frame':        SHARED['lidar_frame'],
                'wall_detector.depth_image_topic':  SHARED['depth_topic'],
                'wall_detector.depth_frame':        SHARED['depth_frame'],
                'wall_detector.depth_intrinsics':   SHARED['depth_intrinsics'],
                'wall_detector.depth_scale_factor': SHARED['depth_scale'],
                'wall_detector.depth_min_value':    SHARED['depth_min'],
                'wall_detector.depth_max_value':    SHARED['depth_max'],
                'wall_detector.depth_skip_pixel':   SHARED['depth_skip'],
                'wall_detector.ground_height':      SHARED['ground_height'],
                'wall_detector.ground_roof_offset': SHARED['ground_roof_offset'],
                'use_sim_time': True,
            },
        ],
    )

    yolo_node = Node(
        package='onboard_detector',
        executable='yolov11_detector_node.py',
        name='yolov11_detector_node',
        output='screen',
        parameters=[
            ParameterFile(config_file, allow_substs=True),
            {
                'image_topic':    SHARED['color_topic'],
                'target_classes': SHARED['yolo_classes'],
                'use_sim_time': True,
            },
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_detector',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        pythonpath_action,
        static_tf_velodyne_to_camera_link,
        static_tf_velodyne_to_camera_link_initial_guess,
        calibration_node,
        detector_node,
        wall_detector_node,
        yolo_node,
        rviz_node,
    ])
