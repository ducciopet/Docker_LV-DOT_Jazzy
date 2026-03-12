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
    camera_frame = str(calibration_params.get('camera_frame', 'rs1_link'))
    pose_topic = str(detector_onboard_params.get('pose_topic', '/pose'))
    map_frame = str(detector_onboard_params.get('tf_map_frame', 'map'))
    scripts_dir = os.path.join(pkg_dir, 'scripts')
    yolo_dir = os.path.join(scripts_dir, 'yolo_detector')

    pythonpath_action = SetEnvironmentVariable(
        'PYTHONPATH',
        yolo_dir + os.pathsep + scripts_dir + os.pathsep + os.environ.get('PYTHONPATH', '')
    )

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



    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='detector_tf_base_link_to_imu_link',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'base_link', '--child-frame-id', 'imu_link']
    )

    static_tf_imu_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='detector_tf_imu_link_to_velodyne',
        arguments=['--x', '-0.1', '--y', '0.0', '--z', '0.30', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'imu_link', '--child-frame-id', 'velodyne']
    )

    initial_guess_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='detector_tf_velodyne_to_camera_initial_guess',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--x', '0.2',
            '--y', '0.0',
            '--z', '-0.65',
            '--roll', '-1.4661566652919380',
            '--pitch', '0.0',
            '--yaw', '-1.8500490070139893',
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
        static_tf_base_to_imu,
        static_tf_imu_to_lidar,
        initial_guess_tf,
        calibration_node,
        detector_node,
        yolo_node,
        rviz_node,
    ])