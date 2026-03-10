#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def _run_calibration_then_start_detector(context, *args, **kwargs):
    import rclpy
    from rclpy.node import Node as RclpyNode
    from std_srvs.srv import Trigger

    timeout_wait_service_sec = 60.0
    timeout_call_sec = 180.0

    rclpy.init(args=None)
    helper = RclpyNode('calibration_startup_client')
    client = helper.create_client(Trigger, '/calibration_icp_node/run_calibration')

    waited = 0.0
    while not client.wait_for_service(timeout_sec=1.0):
        waited += 1.0
        if waited >= timeout_wait_service_sec:
            helper.get_logger().error('Timeout waiting for /calibration_icp_node/run_calibration service')
            helper.destroy_node()
            rclpy.shutdown()
            return [
                LogInfo(msg='Calibration service not available. Detector startup aborted.'),
                EmitEvent(event=Shutdown(reason='Calibration service unavailable')),
            ]

    request = Trigger.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(helper, future, timeout_sec=timeout_call_sec)

    if not future.done() or future.result() is None:
        helper.get_logger().error('Calibration service call timed out or failed')
        helper.destroy_node()
        rclpy.shutdown()
        return [
            LogInfo(msg='Calibration request failed or timed out. Detector startup aborted.'),
            EmitEvent(event=Shutdown(reason='Calibration failed')),
        ]

    response = future.result()
    helper.get_logger().info(f'Calibration response: success={response.success}, message={response.message}')
    helper.destroy_node()
    rclpy.shutdown()

    if not response.success:
        return [
            LogInfo(msg=f'Calibration failed: {response.message}'),
            EmitEvent(event=Shutdown(reason='Calibration returned failure')),
        ]

    pkg_dir = get_package_share_directory('onboard_detector')
    config_file = os.path.join(pkg_dir, 'cfg', 'detector_param_jo_zotac.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'detector_working_jo_zotac.rviz')

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

    return [
        LogInfo(msg='Calibration successful. Starting detector, YOLO, and RViz.'),
        detector_node,
        yolo_node,
        rviz_node,
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory('onboard_detector')
    config_file = os.path.join(pkg_dir, 'cfg', 'detector_param_jo_zotac.yaml')

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

    return LaunchDescription([
        pythonpath_action,
        static_tf_map_to_base,
        static_tf_base_to_imu,
        static_tf_imu_to_lidar,
        initial_guess_tf,
        calibration_node,
        OpaqueFunction(function=_run_calibration_then_start_detector),
    ])
