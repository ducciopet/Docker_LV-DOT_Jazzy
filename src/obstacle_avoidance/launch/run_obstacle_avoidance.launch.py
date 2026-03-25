#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Static TF publishers (copied from run_detector.launch.py)
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='detector_tf_base_link_to_imu_link',
        arguments=[
            '--x','0.22172',
            '--y','0.02606',
            '--z','0.28202',
            '--roll','0.0',
            '--pitch','0.0',
            '--yaw','0.0',
            '--frame-id','base_link',
            '--child-frame-id','imu_link'
        ],
    )

    static_tf_imu_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='detector_tf_imu_link_to_velodyne',
        arguments=[
            '--x','-0.19172',
            '--y','0.00254',
            '--z','0.34798',
            '--roll','0.0',
            '--pitch','0.0',
            '--yaw','0.0',
            '--frame-id','imu_link',
            '--child-frame-id','velodyne'
        ]
    )

    static_tf_velodyne_to_rs1_link_initial_guess = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='calib_icp_tf_velodyne_to_rs1_link_initial_guess',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--x', '0.218304037',
            '--y', '0.107423631',
            '--z', '-0.020496928',
            '--roll', '-1.576042033',
            '--pitch', '-0.037013778',
            '--yaw', '-1.578200049',
            '--frame-id', 'velodyne',
            '--child-frame-id', 'rs1_link_initial_guess'
        ]
    )

    static_tf_velodyne_to_rs1_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='detector_tf_velodyne_to_rs1_link',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--x', '0.27639',
            '--y', '-0.01446',
            '--z', '-0.35199',
            '--roll', '-1.5707963267948966',
            '--pitch', '0.0',
            '--yaw', '-1.5707963267948966',
            '--frame-id', 'velodyne',
            '--child-frame-id', 'rs1_link'
        ]
    )
    # Onboard Detector package paths
    onboard_pkg_dir = get_package_share_directory('onboard_detector')
    onboard_config = os.path.join(onboard_pkg_dir, 'cfg', 'detector_param_jo_zotac.yaml')
    onboard_rviz = os.path.join(onboard_pkg_dir, 'rviz', 'detector_jo_zotac.rviz')

    # Obstacle Avoidance package paths
    avoidance_pkg_dir = get_package_share_directory('obstacle_avoidance')
    avoidance_config = os.path.join(avoidance_pkg_dir, 'cfg', 'obstacle_avoidance_params.yaml')
    avoidance_rviz = os.path.join(avoidance_pkg_dir, 'rviz', 'detector_jo_zotac.rviz')

    # Set PYTHONPATH for onboard_detector scripts
    scripts_dir = os.path.join(onboard_pkg_dir, 'scripts')
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
        parameters=[onboard_config, {'use_sim_time': True}],
    )

    detector_node = Node(
        package='onboard_detector',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[onboard_config, {'use_sim_time': True}],
    )

    yolo_node = Node(
        package='onboard_detector',
        executable='yolov11_detector_node.py',
        name='yolov11_detector_node',
        output='screen',
        parameters=[onboard_config, {'use_sim_time': True}],
    )

    obstacle_avoidance_node = Node(
        package='obstacle_avoidance',
        executable='obstacle_avoidance_node',
        name='obstacle_avoidance_node',
        output='screen',
        parameters=[avoidance_config],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_all',
        output='screen',
        arguments=['-d', avoidance_rviz],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        pythonpath_action,
        static_tf_base_to_imu,
        static_tf_imu_to_lidar,
        static_tf_velodyne_to_rs1_link_initial_guess,
        static_tf_velodyne_to_rs1_link,
        calibration_node,
        detector_node,
        yolo_node,
        obstacle_avoidance_node,
        rviz_node,
    ])
