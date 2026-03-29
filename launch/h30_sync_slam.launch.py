"""Launch ORB-SLAM3 Monocular-Inertial for pre-synchronised topics.

This launch file starts ONLY the SLAM node and optional visualisation.
It does NOT start the camera or IMU drivers — those should be started
separately via the cam_imu_sync package:

    # Terminal 1: start camera + IMU + sync
    ros2 launch cam_imu_sync cam_imu_sync.launch.py

    # Terminal 2: start ORB-SLAM3 (this launch file)
    ros2 launch orb_slam3_ros2 h30_sync_slam.launch.py

The SLAM node subscribes to:
  - /sync/image_raw  (synchronised image)
  - /sync/imu        (synchronised IMU with interpolated boundaries)

Since synchronisation is handled externally by cam_imu_sync,
cam_imu_time_offset is set to 0.0.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_pangolin', default_value='true'),
        DeclareLaunchArgument(
            'image_topic', default_value='/sync/image_raw',
            description='Synchronised image topic'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/sync/imu',
            description='Synchronised IMU topic'),
        DeclareLaunchArgument(
            'settings_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('orb_slam3_ros2'),
                'config', 'Monocular-Inertial', 'rmoncam_h30.yaml'])),

        # ── ORB-SLAM3 Monocular-Inertial ──
        Node(
            package='orb_slam3_ros2',
            executable='imu_mono_node_cpp',
            output='screen',
            parameters=[{
                'sensor_type': 'imu-monocular',
                'use_pangolin': LaunchConfiguration('use_pangolin'),
                'image_topic': LaunchConfiguration('image_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'settings_file': LaunchConfiguration('settings_file'),
                'cam_imu_time_offset': 0.0,  # sync done externally
            }],
        ),

        # ── Static TF ──
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        ),

        # ── RViz ──
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('orb_slam3_ros2'),
                'config', 'point_cloud.rviz'])],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
