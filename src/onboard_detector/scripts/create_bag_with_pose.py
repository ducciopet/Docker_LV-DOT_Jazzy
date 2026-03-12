#!/usr/bin/env python3
"""
Create a new bag with /pose topic representing 0.6m translation along z-axis
from map to base_link, using timestamps from /imu topic.
"""

import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import PoseStamped
import os
from datetime import datetime


def create_bag_with_pose(input_bag_path, output_bag_path):
    """
    Read input bag, copy all topics, and add /pose topic with timestamps from /imu.
    
    Args:
        input_bag_path: Path to source bag directory
        output_bag_path: Path to output bag directory
    """
    
    # Setup reader
    storage_options_read = StorageOptions(uri=input_bag_path, storage_id='mcap')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = SequentialReader()
    reader.open(storage_options_read, converter_options)
    
    # Get topics and types
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    # Setup writer
    storage_options_write = StorageOptions(uri=output_bag_path, storage_id='mcap')
    writer = SequentialWriter()
    writer.open(storage_options_write, converter_options)
    
    # Create all existing topics in output bag
    for topic_info in topic_types:
        writer.create_topic(topic_info)
    
    # Add /pose topic
    from rosbag2_py import TopicMetadata
    pose_topic = TopicMetadata(
        id=0,
        name='/pose',
        type='geometry_msgs/msg/PoseStamped',
        serialization_format='cdr'
    )
    writer.create_topic(pose_topic)
    
    print(f"Reading from: {input_bag_path}")
    print(f"Writing to: {output_bag_path}")
    print(f"Found {len(topic_types)} topics")
    
    imu_topic_candidates = ['/imu', '/imu/data']
    imu_topic = next((t for t in imu_topic_candidates if t in type_map), None)
    if imu_topic is None:
        raise RuntimeError(f"No IMU topic found. Tried {imu_topic_candidates}, available topics: {list(type_map.keys())}")

    # Track IMU timestamps for pose generation
    imu_timestamps = []
    
    # First pass: collect IMU timestamps and copy all messages
    message_count = 0
    imu_count = 0
    
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        # Write original message
        writer.write(topic, data, timestamp)
        message_count += 1
        
        # Collect IMU timestamps
        if topic == imu_topic:
            imu_timestamps.append(timestamp)
            imu_count += 1
        
        if message_count % 1000 == 0:
            print(f"Processed {message_count} messages, collected {imu_count} IMU timestamps...")
    
    print(f"\nCopied {message_count} messages from original bag")
    print(f"Found {imu_count} IMU messages on topic {imu_topic}")
    
    # Create and write pose messages using IMU timestamps
    print(f"\nGenerating {len(imu_timestamps)} pose messages...")
    
    for idx, timestamp in enumerate(imu_timestamps):
        pose_msg = PoseStamped()
        
        # Set header with map frame
        pose_msg.header.frame_id = 'map'
        
        # Convert nanoseconds timestamp to seconds and nanoseconds
        sec = timestamp // 1_000_000_000
        nanosec = timestamp % 1_000_000_000
        pose_msg.header.stamp.sec = int(sec)
        pose_msg.header.stamp.nanosec = int(nanosec)
        
        # Set position: 0.75m translation along z-axis (map to base_link)
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.75
        
        # Set orientation: identity quaternion (no rotation)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        
        # Serialize and write
        serialized_msg = serialize_message(pose_msg)
        writer.write('/pose', serialized_msg, timestamp)
        
        if (idx + 1) % 100 == 0:
            print(f"Written {idx + 1}/{len(imu_timestamps)} pose messages...")
    
    print(f"\nSuccessfully created bag with {len(imu_timestamps)} pose messages")
    print(f"Output: {output_bag_path}")
    
    # Clean up
    del reader
    del writer


def main():
    # Paths relative to workspace bags directory
    bags_dir = '/home/ros/ros2_ws/bags'
    input_bag = os.path.join(bags_dir, 'duccio_test')
    output_bag = os.path.join(bags_dir, 'duccio_test_ok')
    
    # Check if input bag exists
    if not os.path.exists(input_bag):
        print(f"Error: Input bag not found: {input_bag}")
        return 1
    
    # Create output directory if it doesn't exist
    if os.path.exists(output_bag):
        print(f"Warning: Output bag already exists: {output_bag}")
        response = input("Overwrite? (y/n): ")
        if response.lower() != 'y':
            print("Aborted.")
            return 0
        import shutil
        shutil.rmtree(output_bag)
    
    # Create the bag
    try:
        create_bag_with_pose(input_bag, output_bag)

        return 0
        
    except Exception as e:
        print(f"Error creating bag: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    exit(main())
