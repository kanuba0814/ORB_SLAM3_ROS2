from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_pangolin', default_value='true'),
        DeclareLaunchArgument(
            'cam_imu_time_offset', default_value='0.2745',
            description='Kalibr timeshift_cam_imu [s]'),
        DeclareLaunchArgument(
            'settings_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('orb_slam3_ros2'),
                'config', 'Monocular-Inertial', 'rmoncam_h30.yaml'])),

        # ── USB Camera ──
        # publishes: /image_raw (sensor_msgs/Image, RELIABLE QoS)
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
                    'ORB_SLAM3_ROS2/config/Monocular-Inertial/'
                    'rmoncam_camera_info.yaml',
            }],
        ),

        # ── Yesense H30 IMU ──
        # publishes: /imu/data_raw (sensor_msgs/Imu, RELIABLE QoS)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('yesense_std_ros2'),
                    'launch', 'yesense_node.launch.py'])),
        ),

        # ── ORB-SLAM3 Monocular-Inertial ──
        Node(
            package='orb_slam3_ros2',
            executable='imu_mono_node_cpp',
            output='screen',
            parameters=[{
                'sensor_type': 'imu-monocular',
                'use_pangolin': LaunchConfiguration('use_pangolin'),
                'image_topic': '/image_raw',
                'imu_topic': '/imu/data_raw',
                'settings_file': LaunchConfiguration('settings_file'),
                'cam_imu_time_offset':
                    LaunchConfiguration('cam_imu_time_offset'),
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
