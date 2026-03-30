"""Launch ORB-SLAM3 for bag replay testing.

Usage:
    # Terminal 1: Start SLAM
    ros2 launch orb_slam3_ros2 h30_bag_test.launch.py

    # Terminal 2: Play bag
    ros2 bag play /path/to/bag \\
        --qos-profile-overrides-path \\
        $(ros2 pkg prefix orb_slam3_ros2)/share/orb_slam3_ros2/config/bag_qos_override.yaml

Subscribes to /sync/image_raw and /sync/imu (pre-synchronised).
cam_imu_time_offset is set to 0.0 since sync is done externally.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('orb_slam3_ros2')

    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_pangolin', default_value='true'),
        DeclareLaunchArgument(
            'image_topic', default_value='/sync/image_raw'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/sync/imu'),
        DeclareLaunchArgument(
            'settings_file',
            default_value=PathJoinSubstitution([
                pkg_share,
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
                'cam_imu_time_offset': 0.0,
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
                pkg_share, 'config', 'point_cloud.rviz'])],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
