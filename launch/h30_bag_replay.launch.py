# flake8: noqa
"""
Replay an H30 rosbag into the ORB-SLAM3 wrapper only.

This launch file intentionally does not start camera, IMU, or cam_imu_sync
drivers.  It starts the ORB wrapper and, when requested, a ros2 bag player.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _bag_actions(context, *args, **kwargs):
    del args, kwargs
    play_bag = LaunchConfiguration("play_bag").perform(context).lower()
    if play_bag not in ("1", "true", "yes", "on"):
        return []

    bag_path = LaunchConfiguration("bag_path").perform(context)
    rate = LaunchConfiguration("rate").perform(context)
    start_offset = LaunchConfiguration("start_offset").perform(context)
    duration = LaunchConfiguration("duration").perform(context)
    play_delay = LaunchConfiguration("play_delay").perform(context)
    qos_profile_overrides_path = LaunchConfiguration(
        "qos_profile_overrides_path").perform(context)

    cmd = [
        "ros2",
        "bag",
        "play",
        bag_path,
        "--clock",
        "--rate",
        rate,
        "--start-offset",
        start_offset,
    ]
    if qos_profile_overrides_path:
        cmd.extend([
            "--qos-profile-overrides-path",
            qos_profile_overrides_path,
        ])

    shutdown_timer = []
    try:
        duration_value = float(duration)
        rate_value = max(float(rate), 1e-6)
        play_delay_value = max(float(play_delay), 0.0)
    except ValueError:
        duration_value = 0.0
        rate_value = 1.0
        play_delay_value = 0.0

    if duration_value > 0.0:
        shutdown_timer = [
            TimerAction(
                period=play_delay_value + (duration_value / rate_value) + 8.0,
                actions=[Shutdown(reason="h30_bag_replay duration elapsed")],
            )
        ]

    bag_player = ExecuteProcess(
        cmd=cmd,
        output="screen",
    )

    return [
        RegisterEventHandler(
            OnProcessExit(
                target_action=bag_player,
                on_exit=[Shutdown(reason="h30_bag_replay bag play exited")],
            )
        ),
        TimerAction(period=play_delay_value, actions=[bag_player]),
        *shutdown_timer,
    ]


def generate_launch_description():
    pkg_share = FindPackageShare("orb_slam3_ros2")

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag_path",
            default_value="",
            description="Directory path of the rosbag2 bag to replay.",
        ),
        DeclareLaunchArgument(
            "play_bag",
            default_value="false",
            description="Start ros2 bag play from this launch file.",
        ),
        DeclareLaunchArgument(
            "sensor_type",
            default_value="imu-monocular",
            description="ORB-SLAM3 sensor mode: monocular or imu-monocular.",
        ),
        DeclareLaunchArgument("image_topic", default_value="/sync/image_raw"),
        DeclareLaunchArgument("imu_topic", default_value="/sync/imu"),
        DeclareLaunchArgument(
            "settings_file",
            default_value=PathJoinSubstitution([
                pkg_share,
                "config",
                "Monocular-Inertial",
                "rmoncam_h30.yaml",
            ]),
        ),
        DeclareLaunchArgument("cam_imu_time_offset", default_value="0.0"),
        DeclareLaunchArgument(
            "orb_time_offset_mode",
            default_value="zero_for_sync",
            description="Use zero_for_sync for already synchronized topics.",
        ),
        DeclareLaunchArgument("debug_enable", default_value="true"),
        DeclareLaunchArgument("debug_runtime_metrics", default_value="true"),
        DeclareLaunchArgument("enable_video_recording", default_value="false"),
        DeclareLaunchArgument(
            "publish_pose_only_when_tracking_ok",
            default_value="true",
        ),
        DeclareLaunchArgument("max_image_queue_size", default_value="2"),
        DeclareLaunchArgument("max_imu_queue_size", default_value="500"),
        DeclareLaunchArgument("drop_stale_images", default_value="true"),
        DeclareLaunchArgument("max_image_age_ms", default_value="200.0"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_pangolin", default_value="false"),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("start_offset", default_value="0.0"),
        DeclareLaunchArgument(
            "play_delay",
            default_value="2.0",
            description="Seconds to wait before starting ros2 bag play.",
        ),
        DeclareLaunchArgument("imu_accel_scale_x", default_value="1.0"),
        DeclareLaunchArgument("imu_accel_scale_y", default_value="1.0"),
        DeclareLaunchArgument("imu_accel_scale_z", default_value="1.0"),
        DeclareLaunchArgument("imu_gyro_scale_x", default_value="1.0"),
        DeclareLaunchArgument("imu_gyro_scale_y", default_value="1.0"),
        DeclareLaunchArgument("imu_gyro_scale_z", default_value="1.0"),
        DeclareLaunchArgument("imu_accel_axis_order", default_value="xyz"),
        DeclareLaunchArgument("imu_gyro_axis_order", default_value="xyz"),
        DeclareLaunchArgument(
            "duration",
            default_value="0.0",
            description="Playback duration in bag time. 0 means full bag.",
        ),
        DeclareLaunchArgument(
            "qos_profile_overrides_path",
            default_value=PathJoinSubstitution([
                pkg_share,
                "config",
                "bag_qos_override.yaml",
            ]),
            description="ros2 bag play QoS override YAML path.",
        ),

        Node(
            package="orb_slam3_ros2",
            executable="imu_mono_node_cpp",
            output="screen",
            parameters=[{
                "sensor_type": ParameterValue(
                    LaunchConfiguration("sensor_type"),
                    value_type=str,
                ),
                "use_pangolin": ParameterValue(
                    LaunchConfiguration("use_pangolin"),
                    value_type=bool,
                ),
                "image_topic": ParameterValue(
                    LaunchConfiguration("image_topic"),
                    value_type=str,
                ),
                "imu_topic": ParameterValue(
                    LaunchConfiguration("imu_topic"),
                    value_type=str,
                ),
                "settings_file": ParameterValue(
                    LaunchConfiguration("settings_file"),
                    value_type=str,
                ),
                "cam_imu_time_offset":
                    ParameterValue(
                        LaunchConfiguration("cam_imu_time_offset"),
                        value_type=float,
                    ),
                "debug_enable": ParameterValue(
                    LaunchConfiguration("debug_enable"),
                    value_type=bool,
                ),
                "debug_runtime_metrics": ParameterValue(
                    LaunchConfiguration("debug_runtime_metrics"),
                    value_type=bool,
                ),
                "enable_video_recording": ParameterValue(
                    LaunchConfiguration("enable_video_recording"),
                    value_type=bool,
                ),
                "publish_pose_only_when_tracking_ok": ParameterValue(
                    LaunchConfiguration("publish_pose_only_when_tracking_ok"),
                    value_type=bool,
                ),
                "max_image_queue_size": ParameterValue(
                    LaunchConfiguration("max_image_queue_size"),
                    value_type=int,
                ),
                "max_imu_queue_size": ParameterValue(
                    LaunchConfiguration("max_imu_queue_size"),
                    value_type=int,
                ),
                "drop_stale_images": ParameterValue(
                    LaunchConfiguration("drop_stale_images"),
                    value_type=bool,
                ),
                "max_image_age_ms": ParameterValue(
                    LaunchConfiguration("max_image_age_ms"),
                    value_type=float,
                ),
                "orb_time_offset_mode":
                    ParameterValue(
                        LaunchConfiguration("orb_time_offset_mode"),
                        value_type=str,
                    ),
                "imu_accel_scale_x": ParameterValue(
                    LaunchConfiguration("imu_accel_scale_x"),
                    value_type=float,
                ),
                "imu_accel_scale_y": ParameterValue(
                    LaunchConfiguration("imu_accel_scale_y"),
                    value_type=float,
                ),
                "imu_accel_scale_z": ParameterValue(
                    LaunchConfiguration("imu_accel_scale_z"),
                    value_type=float,
                ),
                "imu_gyro_scale_x": ParameterValue(
                    LaunchConfiguration("imu_gyro_scale_x"),
                    value_type=float,
                ),
                "imu_gyro_scale_y": ParameterValue(
                    LaunchConfiguration("imu_gyro_scale_y"),
                    value_type=float,
                ),
                "imu_gyro_scale_z": ParameterValue(
                    LaunchConfiguration("imu_gyro_scale_z"),
                    value_type=float,
                ),
                "imu_accel_axis_order": ParameterValue(
                    LaunchConfiguration("imu_accel_axis_order"),
                    value_type=str,
                ),
                "imu_gyro_axis_order": ParameterValue(
                    LaunchConfiguration("imu_gyro_axis_order"),
                    value_type=str,
                ),
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([
                pkg_share,
                "config",
                "point_cloud.rviz",
            ])],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),

        OpaqueFunction(function=_bag_actions),
    ])
