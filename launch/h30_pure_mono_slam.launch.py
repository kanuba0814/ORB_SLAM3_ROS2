"""Launch ORB-SLAM3 H30 pure Monocular with terminal diagnostics enabled."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('orb_slam3_ros2')

    return LaunchDescription([
        DeclareLaunchArgument('use_pangolin', default_value='true'),
        DeclareLaunchArgument(
            'settings_file',
            default_value=PathJoinSubstitution([
                pkg_share,
                'config', 'Monocular', 'rmoncam.yaml'])),
        DeclareLaunchArgument('debug_enable', default_value='true'),
        DeclareLaunchArgument(
            'cam_imu_time_offset', default_value='0.0',
            description='Pure Monocular mode does not apply an IMU time offset'),
        DeclareLaunchArgument(
            'orb_time_offset_mode', default_value='legacy_warn'),
        DeclareLaunchArgument(
            'debug_expected_imu_rate_hz', default_value='0.0'),

        # USB camera only. This launch intentionally does not start H30 IMU.
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video-rmoncam',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'yuyv',
                'camera_info_url': 'file:///home/gxxl/slam_ws_v2/extra_ws/'
                    'src/ORB_SLAM3_ROS2/config/Monocular-Inertial/'
                    'rmoncam_camera_info.yaml',
            }],
        ),

        Node(
            package='orb_slam3_ros2',
            executable='imu_mono_node_cpp',
            output='screen',
            parameters=[{
                'sensor_type': 'monocular',
                'use_pangolin': LaunchConfiguration('use_pangolin'),
                'image_topic': '/image_raw',
                'imu_topic': '',
                'settings_file': LaunchConfiguration('settings_file'),
                'cam_imu_time_offset':
                    LaunchConfiguration('cam_imu_time_offset'),
                'debug_enable': LaunchConfiguration('debug_enable'),
                'orb_time_offset_mode':
                    LaunchConfiguration('orb_time_offset_mode'),
                'debug_expected_imu_rate_hz':
                    LaunchConfiguration('debug_expected_imu_rate_hz'),
            }],
        ),
    ])
