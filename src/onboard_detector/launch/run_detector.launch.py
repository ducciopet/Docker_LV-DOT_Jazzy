#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('onboard_detector')
    config_file = os.path.join(pkg_dir, 'cfg', 'detector_param_jo_zotac.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'detector_working_jo_zotac.rviz')

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

    static_tf_map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'map', '--child-frame-id', 'base_link']
    )

    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'base_link', '--child-frame-id', 'imu_link']
    )

    static_tf_imu_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '-0.1', '--y', '0.0', '--z', '0.30', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'imu_link', '--child-frame-id', 'velodyne']
    )

    initial_guess_tf = Node(
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
        parameters=[{'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    tf_wait_script = """
import sys
import time
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

TIMEOUT_SEC = 300.0
CHECK_PERIOD_SEC = 0.5
TARGET_FRAME = 'velodyne'
SOURCE_FRAME = 'rs1_link_refined'

rclpy.init(args=None)
node = Node('wait_for_refined_tf')
buffer = Buffer()
listener = TransformListener(buffer, node, spin_thread=True)
deadline = time.monotonic() + TIMEOUT_SEC

try:
    while time.monotonic() < deadline and rclpy.ok():
        if buffer.can_transform(TARGET_FRAME, SOURCE_FRAME, Time(), timeout=Duration(seconds=0.2)):
            print('[refined_tf_waiter] Detected TF velodyne -> rs1_link_refined')
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
        time.sleep(CHECK_PERIOD_SEC)

    print('[refined_tf_waiter] Timeout waiting for TF velodyne -> rs1_link_refined', file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)
except Exception as exc:
    print(f'[refined_tf_waiter] Error while waiting for TF: {exc}', file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)
"""

    tf_waiter = ExecuteProcess(
        cmd=['python3', '-c', tf_wait_script],
        output='screen',
        name='wait_for_refined_tf',
    )

    detector_start_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=tf_waiter,
            on_exit=lambda event, context: [
                LogInfo(msg='Refined TF published. Starting detector_node.'),
                detector_node,
            ] if event.returncode == 0 else [
                LogInfo(msg='Refined TF was not published in time. Shutting down.'),
                EmitEvent(event=Shutdown(reason='Calibration TF unavailable')),
            ],
        )
    )

    return LaunchDescription([
        pythonpath_action,
        static_tf_map_to_base,
        static_tf_base_to_imu,
        static_tf_imu_to_lidar,
        initial_guess_tf,
        calibration_node,
        yolo_node,
        rviz_node,
        tf_waiter,
        detector_start_handler,
    ])
