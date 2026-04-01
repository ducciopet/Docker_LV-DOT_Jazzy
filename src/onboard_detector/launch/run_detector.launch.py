#!/usr/bin/env python3

import os
import yaml

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


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
    detector_params = _load_node_params(config_file, 'detector_node')
    detector_onboard_params = detector_params.get('onboard_detector', {}) if isinstance(detector_params, dict) else {}
    camera_frame = str(calibration_params.get('camera_frame', 'camera_link'))
    camera_frame_initial_guess = str(calibration_params.get('camera_frame_initial_guess', 'camera_initial_guess'))

    pose_topic = str(detector_onboard_params.get('pose_topic', '/glim_ros/lidar_pose'))
    map_frame = str(detector_onboard_params.get('tf_map_frame', 'map'))
    scripts_dir = os.path.join(pkg_dir, 'scripts')
    yolo_dir = os.path.join(scripts_dir, 'yolo_detector')

    pythonpath_action = SetEnvironmentVariable(
        'PYTHONPATH',
        yolo_dir + os.pathsep + scripts_dir + os.pathsep + os.environ.get('PYTHONPATH', '')
    )

    # /tf from velodyne to initial guess camera frame (camera_initial_guess) obtained from a


    calibration_node = Node(
        package='onboard_detector',
        executable='calibration_icp_node',
        name='calibration_icp_node',
        output='screen',
        parameters=[
            ParameterFile(config_file, allow_substs=True),
            {'use_sim_time': True},
        ],
    )

    # static_tf_base_to_imu = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='detector_tf_base_link_to_imu_link',
    #     arguments=[
    #         '--x','0.22172',
    #         '--y','0.02606',
    #         '--z','0.28202',
    #         '--roll','0.0',
    #         '--pitch','0.0',
    #         '--yaw','0.0',
    #         '--frame-id','base_link',
    #         '--child-frame-id','imu_link'
    #     ],
    # )

    # static_tf_imu_to_lidar = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='detector_tf_imu_link_to_velodyne',
    #     arguments=[
    #         '--x','-0.19172',
    #         '--y','0.00254',
    #         '--z','0.34798',
    #         '--roll','0.0',
    #         '--pitch','0.0',
    #         '--yaw','0.0',
    #         '--frame-id','imu_link',
    #         '--child-frame-id','velodyne'
    #     ]
    # )


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
                '--frame-id', 'velodyne',
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
            '--frame-id', 'velodyne',
            '--child-frame-id', camera_frame
        ]
    )


    detector_node = Node(
        package='onboard_detector',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[
            ParameterFile(config_file, allow_substs=True),
            {'use_sim_time': True},
        ],
    )

    yolo_node = Node(
        package='onboard_detector',
        executable='yolov11_detector_node.py',
        name='yolov11_detector_node',
        output='screen',
        parameters=[
            ParameterFile(config_file, allow_substs=True),
            {'use_sim_time': True},
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
        # static_tf_base_to_imu,
        # static_tf_imu_to_lidar,
        static_tf_velodyne_to_camera_link,
        static_tf_velodyne_to_camera_link_initial_guess,
        calibration_node,
        detector_node,
        yolo_node,
        rviz_node,
    ])