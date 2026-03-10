#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterFile
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from pathlib import Path

def generate_launch_description():
    pkg_dir = get_package_share_directory('onboard_detector')
    
    # Path to parameters file
    config_file = os.path.join(pkg_dir, 'cfg', 'detector_param_jo_zotac.yaml')
    
    # Path to rviz config file
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'detector_working_jo_zotac.rviz')
    
    # Add scripts directory to Python path for yolo imports
    scripts_dir = os.path.join(pkg_dir, 'scripts')
    yolo_dir = os.path.join(scripts_dir, 'yolo_detector')
    
    # Set ROS_DOMAIN_ID if needed and PYTHONPATH
    pythonpath_action = SetEnvironmentVariable(
        'PYTHONPATH',
        yolo_dir + os.pathsep + scripts_dir + os.pathsep + os.environ.get('PYTHONPATH', '')
    )
    
    return LaunchDescription([
        pythonpath_action,
        
        # Load parameters from YAML file
        Node(
            package='onboard_detector',
            executable='detector_node',
            name='detector_node',
            output='screen',
            parameters=[
                ParameterFile(config_file, allow_substs=True),
                {'use_sim_time': True},
            ],
        ),
        
        # YOLO v11 detector node

        Node(
            package='onboard_detector',
            executable='yolov11_detector_node.py',
            name='yolov11_detector_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        
        # RViz visualization (optional - comment out if not needed)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),

        # Static TF transforms

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',                        
            arguments=[
                '--x','0.0',
                '--y','0.0',
                '--z','0.0',
                '--roll','0.0',
                '--pitch','0.0',
                '--yaw','0.0',
                '--frame-id','map', 
                '--child-frame-id','base_link'
            ]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',                        
            arguments=[
                '--x','0.0',
                '--y','0.0',
                '--z','0.0',
                '--roll','0.0',
                '--pitch','0.0',
                '--yaw','0.0',
                '--frame-id','base_link', 
                '--child-frame-id','imu_link'
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',                        
            arguments=[
                '--x','-0.1',
                '--y','0.0',
                '--z','0.30',
                '--roll','0.0',
                '--pitch','0.0',
                '--yaw','0.0',
                '--frame-id','imu_link', 
                '--child-frame-id','velodyne'
            ]
        ),

        # Old static transform: velodyne -> rs1_link (used as initial guess for ICP)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '--x','0.2',
                '--y','0.0',
                '--z','-0.65',
                '--roll','-1.4661566652919380',
                '--pitch','0.0',
                '--yaw','-1.8500490070139893',
                '--frame-id','velodyne',
                '--child-frame-id','rs1_link'
            ]
        ),

        # ICP Refined Transform (velodyne -> rs1_link):
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '--x', '0.225',
                '--y', '0.062',
                '--z', '-0.193',
                '--roll', '-1.537',
                '--pitch', '-0.042',
                '--yaw', '-1.561',
                '--frame-id', 'velodyne',
                '--child-frame-id', 'rs1_link_refined'
            ]
        ),
    ])
