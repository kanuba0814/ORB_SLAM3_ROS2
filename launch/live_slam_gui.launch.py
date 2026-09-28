"""
One-shot live launch: H30 camera + IMU + sync + ORB-SLAM3 (with InitGate)
+ RViz2 + Pangolin Frame Viewer + Pangolin Map Viewer.

Composition:
  - IncludeLaunchDescription(cam_imu_sync/launch/cam_imu_sync.launch.py)
      Starts usb_cam + yesense_std_ros2 + cam_imu_sync_node.
      Produces /sync/image_raw and /sync/imu (Kalibr-corrected).
  - imu_mono_node_cpp (ORB-SLAM3 wrapper)
      Subscribes to /sync/*. cam_imu_time_offset is forced to 0.0
      because the sync node already applies the Kalibr timeshift —
      avoid double-correcting. orb_time_offset_mode=zero_for_sync.
      InitGate hardcoded ON (init_gate_enabled=True); the gate's
      motion threshold / sustained duration / await timeout fall
      back to the C++ defaults (3.0 / 0.3 / 20.0).
  - static_transform_publisher world -> map
  - GUI = three windows, all on by default:
      * RViz2 with config/point_cloud.rviz
      * Pangolin "ORB-SLAM3: Current Frame" (image + ORB features)
      * Pangolin "ORB-SLAM3: Map Viewer"   (3D map / keyframes)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('orb_slam3_ros2')

    return LaunchDescription([
        # ── GUI toggles ─────────────────────────────────────────────
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz2 with point_cloud.rviz config.'),
        DeclareLaunchArgument(
            'use_pangolin', default_value='true',
            description='Enable ORB-SLAM3 Pangolin Frame Viewer + Map Viewer.'),

        # ── Hardware + sync passthrough ─────────────────────────────
        DeclareLaunchArgument(
            'cam_imu_time_offset', default_value='0.2745',
            description='Kalibr timeshift_cam_imu [s] passed to the '
                        'sync node only. The SLAM node sees /sync/* '
                        'already time-corrected, so cam_imu_time_offset '
                        'inside the SLAM node is hardcoded to 0.'),
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video-rmoncam',
            description='V4L2 device path for the H30 camera.'),
        DeclareLaunchArgument(
            'camera_info_url', default_value='',
            description='Optional camera_info YAML URL.'),
        DeclareLaunchArgument(
            'enable_interpolation', default_value='true',
            description='cam_imu_sync linear interpolation at frame edges.'),

        # ── SLAM node knobs ─────────────────────────────────────────
        DeclareLaunchArgument(
            'image_topic', default_value='/sync/image_raw'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/sync/imu'),
        DeclareLaunchArgument(
            'settings_file',
            default_value=PathJoinSubstitution([
                pkg_share, 'config', 'Monocular-Inertial',
                'rmoncam_h30.yaml'])),
        DeclareLaunchArgument('debug_enable', default_value='false'),
        DeclareLaunchArgument(
            'orb_time_offset_mode', default_value='zero_for_sync'),
        DeclareLaunchArgument(
            'debug_expected_imu_rate_hz', default_value='0.0'),
        DeclareLaunchArgument('imu_accel_scale_x', default_value='1.0'),
        DeclareLaunchArgument('imu_accel_scale_y', default_value='1.0'),
        DeclareLaunchArgument('imu_accel_scale_z', default_value='1.0'),
        DeclareLaunchArgument('imu_gyro_scale_x', default_value='1.0'),
        DeclareLaunchArgument('imu_gyro_scale_y', default_value='1.0'),
        DeclareLaunchArgument('imu_gyro_scale_z', default_value='1.0'),
        DeclareLaunchArgument('imu_accel_axis_order', default_value='xyz'),
        DeclareLaunchArgument('imu_gyro_axis_order', default_value='xyz'),

        # ── Hardware + sync (usb_cam + yesense + cam_imu_sync_node) ─
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('cam_imu_sync'),
                    'launch', 'cam_imu_sync.launch.py'])),
            launch_arguments={
                'cam_imu_time_offset':
                    LaunchConfiguration('cam_imu_time_offset'),
                'video_device':
                    LaunchConfiguration('video_device'),
                'camera_info_url':
                    LaunchConfiguration('camera_info_url'),
                'enable_interpolation':
                    LaunchConfiguration('enable_interpolation'),
            }.items(),
        ),

        # ── ORB-SLAM3 Monocular-Inertial ────────────────────────────
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
                # Sync node already applied the Kalibr offset; do NOT
                # apply it a second time inside the SLAM node.
                'cam_imu_time_offset': 0.0,
                'debug_enable': LaunchConfiguration('debug_enable'),
                'orb_time_offset_mode':
                    LaunchConfiguration('orb_time_offset_mode'),
                'debug_expected_imu_rate_hz':
                    LaunchConfiguration('debug_expected_imu_rate_hz'),
                'imu_accel_scale_x': ParameterValue(
                    LaunchConfiguration('imu_accel_scale_x'),
                    value_type=float),
                'imu_accel_scale_y': ParameterValue(
                    LaunchConfiguration('imu_accel_scale_y'),
                    value_type=float),
                'imu_accel_scale_z': ParameterValue(
                    LaunchConfiguration('imu_accel_scale_z'),
                    value_type=float),
                'imu_gyro_scale_x': ParameterValue(
                    LaunchConfiguration('imu_gyro_scale_x'),
                    value_type=float),
                'imu_gyro_scale_y': ParameterValue(
                    LaunchConfiguration('imu_gyro_scale_y'),
                    value_type=float),
                'imu_gyro_scale_z': ParameterValue(
                    LaunchConfiguration('imu_gyro_scale_z'),
                    value_type=float),
                'imu_accel_axis_order': ParameterValue(
                    LaunchConfiguration('imu_accel_axis_order'),
                    value_type=str),
                'imu_gyro_axis_order': ParameterValue(
                    LaunchConfiguration('imu_gyro_axis_order'),
                    value_type=str),
                # InitGate hardcoded ON; threshold / sustained /
                # await timeout intentionally omitted so the C++
                # node defaults (3.0 / 0.3 / 20.0) apply.
                'init_gate_enabled': True,
            }],
        ),

        # ── Static TF ───────────────────────────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        ),

        # ── RViz ────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                pkg_share, 'config', 'point_cloud.rviz'])],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
