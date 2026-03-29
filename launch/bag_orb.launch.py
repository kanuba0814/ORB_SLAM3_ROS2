from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value="",
                description="Absolute path to rosbag2 directory.",
            ),
            DeclareLaunchArgument(
                "play_bag",
                default_value="true",
                description="Whether to play rosbag2 in this launch.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Whether to launch RViz.",
            ),
            DeclareLaunchArgument(
                "sensor_type",
                default_value="imu-monocular",
                description="ORB_SLAM3 sensor mode: monocular or imu-monocular.",
            ),
            DeclareLaunchArgument(
                "use_pangolin",
                default_value="true",
                description="Whether to show Pangolin viewer.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/sensing/camera/CAM_FRONT/image_raw",
                description="Image topic to subscribe.",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/sensing/imu/imu_data",
                description="IMU topic to subscribe.",
            ),
            DeclareLaunchArgument(
                "settings_file",
                default_value="",
                description="Override ORB_SLAM3 settings YAML path (absolute). Empty = auto-select by sensor_type.",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    LaunchConfiguration("bag_path"),
                    "--clock",
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("play_bag")),
            ),
            Node(
                package="orb_slam3_ros2",
                executable="imu_mono_node_cpp",
                output="screen",
                parameters=[
                    {
                        "sensor_type": LaunchConfiguration("sensor_type"),
                        "use_pangolin": LaunchConfiguration("use_pangolin"),
                        "use_sim_time": True,
                        "image_topic": LaunchConfiguration("image_topic"),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                        "settings_file": LaunchConfiguration("settings_file"),
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("orb_slam3_ros2"),
                            "config",
                            "point_cloud.rviz",
                        ]
                    ),
                ],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
