#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    pkg_dir = get_package_share_directory('onboard_detector')

    config_file = os.path.join(pkg_dir, 'cfg', 'detector_param_jo_zotac.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'detector_debug.rviz')

    return LaunchDescription([
        Node(
            package='onboard_detector',
            executable='calibration_icp_node',
            name='calibration_icp_node',
            output='screen',
            parameters=[
                ParameterFile(config_file, allow_substs=True),
                {'use_sim_time': True},
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '--x', '0.2',
                '--y', '0.0',
                '--z', '-0.65',
                '--roll', '-1.4661566652919380',
                '--pitch', '0.0',
                '--yaw', '-1.8500490070139893',
                '--frame-id', 'velodyne',
                '--child-frame-id', 'rs1_link'
            ]
        ),
    ])
