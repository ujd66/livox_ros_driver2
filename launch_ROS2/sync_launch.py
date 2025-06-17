import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Sync node for camera and lidar synchronization
    sync_node = Node(
        package='livox_ros_driver2',
        executable='sync_node_ros2',
        name='sync_node',
        output='screen',
        parameters=[{
            'image_topic': '/camera/color/image_raw',  # D435i camera topic
            'lidar_topic': '/livox/lidar',             # MID360 lidar topic
            'synced_image_topic': '/synced_image',     # Synchronized image output
            'synced_lidar_topic': '/synced_lidar',     # Synchronized lidar output
            'publish_rate': 10.0                       # 10Hz synchronization rate
        }]
    )

    return LaunchDescription([
        sync_node
    ])