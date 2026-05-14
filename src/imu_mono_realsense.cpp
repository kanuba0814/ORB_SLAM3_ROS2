#include <pcl/PCLPointCloud2.h>
#include <pcl/cloud_iterator.h>
#include <pcl/common/centroid.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/logging.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#ifdef ORB_SLAM3_ROS2_HAS_NAV2_MAP_SERVER
#include "nav2_map_server/map_io.hpp"
#endif

#include <chrono>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <vector>
#include <atomic>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/persistence.hpp>

// this is orb_slam3
#include "System.h"

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

static const int kSyncSleepMs = 1;  // SyncWithImu polling interval

static int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

static double ns_to_ms(int64_t ns)
{
  return static_cast<double>(ns) / 1e6;
}

static std::string resolve_default_vocabulary_path()
{
  const char *vocab_env = std::getenv("ORB_SLAM3_VOCAB_PATH");
  if (vocab_env != nullptr && std::filesystem::exists(vocab_env)) {
    return std::string(vocab_env);
  }

  const char *root_env = std::getenv("ORB_SLAM3_ROOT_DIR");
  std::vector<std::string> candidates = {
    std::string(PROJECT_PATH) + "/ORB_SLAM3/Vocabulary/ORBvoc.txt",
    std::string(PROJECT_PATH) + "/Vocabulary/ORBvoc.txt",
    "/home/gxxl/slam_ws_v2/libs/ORB_SLAM3/Vocabulary/ORBvoc.txt",
    "/usr/local/share/ORB_SLAM3/Vocabulary/ORBvoc.txt",
    "/usr/share/ORB_SLAM3/Vocabulary/ORBvoc.txt",
  };

  if (root_env != nullptr) {
    candidates.insert(candidates.begin(), std::string(root_env) + "/Vocabulary/ORBvoc.txt");
  }

  for (const auto &path : candidates) {
    if (std::filesystem::exists(path)) {
      return path;
    }
  }

  return std::string(PROJECT_PATH) + "/ORB_SLAM3/Vocabulary/ORBvoc.txt";
}

class ImuMonoRealSense : public rclcpp::Node {
public:
  ~ImuMonoRealSense()
  {
    sync_running_ = false;
    if (sync_thread_.joinable()) sync_thread_.join();
  }

  ImuMonoRealSense()
    : Node("imu_mono_realsense"),
      vocabulary_file_path(resolve_default_vocabulary_path()),
      inertial_ba1_(false), inertial_ba2_(false),
      sync_running_(true)
  {

    // declare parameters
    declare_parameter("sensor_type", "imu-monocular");
    declare_parameter("use_pangolin", true);
    declare_parameter("image_topic", "camera/camera/color/image_raw");
    declare_parameter("imu_topic", "camera/camera/imu");
    declare_parameter("settings_file", "");
    declare_parameter("cam_imu_time_offset", 0.0);
    declare_parameter("debug_enable", false);
    declare_parameter("debug_runtime_metrics", true);
    declare_parameter("enable_video_recording", false);
    declare_parameter("publish_pose_only_when_tracking_ok", true);
    declare_parameter("max_image_queue_size", 2);
    declare_parameter("max_imu_queue_size", 500);
    declare_parameter("drop_stale_images", true);
    declare_parameter("max_image_age_ms", 200.0);
    declare_parameter("orb_time_offset_mode", "legacy_warn");
    declare_parameter("debug_expected_imu_rate_hz", 0.0);
    declare_parameter("imu_accel_scale_x", 1.0);
    declare_parameter("imu_accel_scale_y", 1.0);
    declare_parameter("imu_accel_scale_z", 1.0);
    declare_parameter("imu_gyro_scale_x", 1.0);
    declare_parameter("imu_gyro_scale_y", 1.0);
    declare_parameter("imu_gyro_scale_z", 1.0);
    declare_parameter("imu_accel_axis_order", "xyz");
    declare_parameter("imu_gyro_axis_order", "xyz");

    // get parameters
    sensor_type_param = get_parameter("sensor_type").as_string();
    use_pangolin = get_parameter("use_pangolin").as_bool();
    image_topic_ = get_parameter("image_topic").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();
    cam_imu_time_offset_ = get_parameter("cam_imu_time_offset").as_double();
    debug_enable_ = get_parameter("debug_enable").as_bool();
    debug_runtime_metrics_ =
      get_parameter("debug_runtime_metrics").as_bool();
    enable_video_recording_ =
      get_parameter("enable_video_recording").as_bool();
    publish_pose_only_when_tracking_ok_ =
      get_parameter("publish_pose_only_when_tracking_ok").as_bool();
    max_image_queue_size_ =
      get_parameter("max_image_queue_size").as_int();
    max_imu_queue_size_ =
      get_parameter("max_imu_queue_size").as_int();
    drop_stale_images_ = get_parameter("drop_stale_images").as_bool();
    max_image_age_ms_ = get_parameter("max_image_age_ms").as_double();
    orb_time_offset_mode_ = get_parameter("orb_time_offset_mode").as_string();
    debug_expected_imu_rate_hz_ =
      get_parameter("debug_expected_imu_rate_hz").as_double();
    imu_accel_scale_[0] = get_parameter("imu_accel_scale_x").as_double();
    imu_accel_scale_[1] = get_parameter("imu_accel_scale_y").as_double();
    imu_accel_scale_[2] = get_parameter("imu_accel_scale_z").as_double();
    imu_gyro_scale_[0] = get_parameter("imu_gyro_scale_x").as_double();
    imu_gyro_scale_[1] = get_parameter("imu_gyro_scale_y").as_double();
    imu_gyro_scale_[2] = get_parameter("imu_gyro_scale_z").as_double();
    imu_accel_axis_order_ = get_parameter("imu_accel_axis_order").as_string();
    imu_gyro_axis_order_ = get_parameter("imu_gyro_axis_order").as_string();
    imu_accel_axis_order_ = load_axis_order(
      imu_accel_axis_order_, imu_accel_axis_index_, "imu_accel_axis_order");
    imu_gyro_axis_order_ = load_axis_order(
      imu_gyro_axis_order_, imu_gyro_axis_index_, "imu_gyro_axis_order");
    RCLCPP_INFO(get_logger(),
                "Camera-IMU time offset: %.6f s (t_imu = t_cam + offset)",
                cam_imu_time_offset_);
    RCLCPP_INFO(
      get_logger(),
      "IMU input transform: acc_order=%s acc_scale=(%.3f, %.3f, %.3f), "
      "gyro_order=%s gyro_scale=(%.3f, %.3f, %.3f)",
      imu_accel_axis_order_.c_str(),
      imu_accel_scale_[0], imu_accel_scale_[1], imu_accel_scale_[2],
      imu_gyro_axis_order_.c_str(),
      imu_gyro_scale_[0], imu_gyro_scale_[1], imu_gyro_scale_[2]);
    RCLCPP_INFO(
      get_logger(),
      "Runtime controls: debug_runtime_metrics=%d enable_video_recording=%d "
      "publish_pose_only_when_tracking_ok=%d max_image_queue_size=%ld "
      "max_imu_queue_size=%ld drop_stale_images=%d max_image_age_ms=%.3f",
      debug_runtime_metrics_ ? 1 : 0,
      enable_video_recording_ ? 1 : 0,
      publish_pose_only_when_tracking_ok_ ? 1 : 0,
      max_image_queue_size_, max_imu_queue_size_,
      drop_stale_images_ ? 1 : 0, max_image_age_ms_);

    // define callback groups
    image_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    imu_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    slam_service_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions image_options;
    image_options.callback_group = image_callback_group_;
    rclcpp::SubscriptionOptions imu_options;
    imu_options.callback_group = imu_callback_group_;

    // set the sensor type based on parameter
    std::string settings_file_param = get_parameter("settings_file").as_string();

    ORB_SLAM3::System::eSensor sensor_type;
    if (sensor_type_param == "monocular") {
      sensor_type = ORB_SLAM3::System::MONOCULAR;
      settings_file_path = settings_file_param.empty()
        ? std::string(PROJECT_PATH) + "/config/Monocular/RealSense_D435i.yaml"
        : settings_file_param;
    } else if (sensor_type_param == "imu-monocular") {
      sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
      settings_file_path = settings_file_param.empty()
        ? std::string(PROJECT_PATH) + "/config/Monocular-Inertial/RealSense_D435i.yaml"
        : settings_file_param;
    } else {
      RCLCPP_ERROR(get_logger(), "Sensor type not recognized");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(get_logger(), "Settings file: %s", settings_file_path.c_str());
    load_yaml_debug_config();
    init_debug_output();
    warn_if_sync_offset_double_applied();

    RCLCPP_INFO_STREAM(get_logger(),
                       "vocabulary_file_path: " << vocabulary_file_path);

    // setup orb slam object
    orb_slam3_system_ = std::make_shared<ORB_SLAM3::System>(
      vocabulary_file_path, settings_file_path, sensor_type, use_pangolin, 0);

    // create publishers
    live_point_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("live_point_cloud", 10);
    pose_array_publisher_ =
      create_publisher<geometry_msgs::msg::PoseArray>("pose_array", 10);
    live_occupancy_grid_publisher_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("live_occupancy_grid", 10);
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("orb_odom", 10);
    orb_image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>("/orb_camera/image", 10);
    imu_publisher_ =
      create_publisher<sensor_msgs::msg::Imu>("/orb_camera/imu", 10);

    // create subscriptions
    rclcpp::QoS image_qos(rclcpp::KeepLast(10));
    image_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    image_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    image_sub = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, image_qos,
      std::bind(&ImuMonoRealSense::image_callback, this, _1), image_options);

    if (sensor_type_param != "monocular") {
      rclcpp::QoS imu_qos(rclcpp::KeepLast(10));
      imu_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
      imu_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
      imu_sub = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, imu_qos,
        std::bind(&ImuMonoRealSense::imu_callback, this, _1), imu_options);
    } else {
      RCLCPP_INFO(get_logger(),
                  "Pure Monocular mode selected; IMU subscription disabled");
    }

    // tf broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // create timer
    timer = create_wall_timer(
      100ms, std::bind(&ImuMonoRealSense::timer_callback, this),
      timer_callback_group_);

    timestamp_ = generate_timestamp_string();

    std::string path = std::string(PROJECT_PATH) + "/output/" + timestamp_;
    if (!std::filesystem::create_directory(path)) {
      std::cout << "Failed to create output directory" << std::endl;
      return;
    }
    if (!std::filesystem::create_directory(path + "/cloud")) {
      std::cout << "Failed to create cloud directory" << std::endl;
      return;
    }
    if (!std::filesystem::create_directory(path + "/grid")) {
      std::cout << "Failed to create grid directory" << std::endl;
      return;
    }
    if (enable_video_recording_) {
      if (!std::filesystem::create_directory(path + "/video")) {
        std::cout << "Failed to create images directory" << std::endl;
        return;
      }
    }

    rclcpp::on_shutdown([this]() {
      video_writer_.release();
      if (!live_pcl_cloud_.empty()) {
        try {
          pcl::io::savePCDFileBinary(
            std::string(PROJECT_PATH) + "/output/" + timestamp_ + "/cloud/" +
              timestamp_ + ".pcd",
            live_pcl_cloud_);
        } catch (const pcl::IOException &e) {
          RCLCPP_WARN(get_logger(), "Skipping PCD save: %s", e.what());
        }
      } else {
        RCLCPP_INFO(get_logger(), "Skipping PCD save: point cloud is empty");
      }
      #ifdef ORB_SLAM3_ROS2_HAS_NAV2_MAP_SERVER
      nav2_map_server::SaveParameters save_params;
      save_params.map_file_name = std::string(PROJECT_PATH) + "/output/" +
                                  timestamp_ + "/grid/" + timestamp_;
      save_params.image_format = "pgm";
      save_params.free_thresh = 0.196;
      save_params.occupied_thresh = 0.65;
      nav2_map_server::saveMapToFile(*live_occupancy_grid_, save_params);
      #else
      RCLCPP_WARN(get_logger(),
                  "nav2_map_server not found at build time; skipping occupancy grid save");
      #endif
    });

    initialize_variables();

    // Start the synchronisation thread (matches official ORB_SLAM3 ROS wrapper)
    sync_thread_ = std::thread(&ImuMonoRealSense::SyncWithImu, this);
    RCLCPP_INFO(get_logger(), "SyncWithImu thread started");

    if (enable_video_recording_) {
      std::string orb_slam_video_path = std::string(PROJECT_PATH) + "/output/" +
                                        timestamp_ + "/video/" + timestamp_ +
                                        ".mp4";
      RCLCPP_INFO_STREAM(get_logger(), "Video path: " << orb_slam_video_path);
      video_writer_.open(orb_slam_video_path,
                         cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30,
                         cv::Size(640, 500));

      if (!video_writer_.isOpened()) {
        RCLCPP_ERROR(get_logger(), "Error opening video writer");
        rclcpp::shutdown();
      }
    } else {
      RCLCPP_INFO(get_logger(), "Debug video recording disabled");
    }
  }

private:
  struct QueuedImage
  {
    sensor_msgs::msg::Image::SharedPtr msg;
    int64_t receive_steady_ns{0};
  };

  struct QueuedImu
  {
    sensor_msgs::msg::Imu::SharedPtr msg;
    int64_t receive_steady_ns{0};
  };

  struct FrameRuntimeMetrics
  {
    int64_t receive_steady_ns{0};
    int64_t track_begin_steady_ns{0};
    int64_t track_end_steady_ns{0};
    double cv_bridge_ms{0.0};
    double wait_for_imu_ms{0.0};
    double track_ms{0.0};
    double video_write_ms{0.0};
    double total_frame_ms{0.0};
    double image_age_at_track_begin_ms{0.0};
    double image_age_at_track_end_ms{0.0};
    double processed_image_age_ms{0.0};
    double first_imu_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double last_imu_time_sec{std::numeric_limits<double>::quiet_NaN()};
    double imu_span_ms{0.0};
    double image_to_first_imu_ms{std::numeric_limits<double>::quiet_NaN()};
    double image_to_last_imu_ms{std::numeric_limits<double>::quiet_NaN()};
  };

  pcl::PointCloud<pcl::PointXYZ>::Ptr
  filter_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    // statistical outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(100);
    sor.setStddevMulThresh(0.1);
    sor.filter(*sor_cloud);

    // radius outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr radius_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier;
    radius_outlier.setInputCloud(sor_cloud);
    radius_outlier.setRadiusSearch(
      0.1); // Adjust based on spacing in the point cloud
    radius_outlier.setMinNeighborsInRadius(
      5); // Increase for more aggressive outlier removal
    radius_outlier.filter(*radius_cloud);

    return radius_cloud;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr
  point_cloud_to_occupancy_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    // calculate the centroid
    Eigen::Matrix<float, 4, 1> centroid;
    pcl::ConstCloudIterator<pcl::PointXYZ> cloud_iterator(*cloud);
    pcl::compute3DCentroid(cloud_iterator, centroid);

    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();

    for (const auto &point : cloud->points) {
      if (point.x > max_x) {
        max_x = point.x;
      }
      if (point.y > max_y) {
        max_y = point.y;
      }
      if (point.x < min_x) {
        min_x = point.x;
      }
      if (point.y < min_y) {
        min_y = point.y;
      }
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid =
      std::make_shared<nav_msgs::msg::OccupancyGrid>();
    cloud->width = cloud->points.size();
    occupancy_grid->header.frame_id = "live_map";
    occupancy_grid->header.stamp = get_clock()->now();
    occupancy_grid->info.resolution = 0.05;
    occupancy_grid->info.width =
      std::abs(max_x - min_x) / occupancy_grid->info.resolution + 1;
    occupancy_grid->info.height =
      std::abs(max_y - min_y) / occupancy_grid->info.resolution + 1;
    occupancy_grid->info.origin.position.x = min_x;
    occupancy_grid->info.origin.position.y = min_y;
    occupancy_grid->info.origin.position.z = 0;
    occupancy_grid->info.origin.orientation.x = 0;
    occupancy_grid->info.origin.orientation.y = 0;
    occupancy_grid->info.origin.orientation.z = 0;
    occupancy_grid->info.origin.orientation.w = 1;
    occupancy_grid->data.resize(
      occupancy_grid->info.width * occupancy_grid->info.height, 0);
    for (const auto &point : cloud->points) {
      int x = (point.x - min_x) / occupancy_grid->info.resolution;
      int y = (point.y - min_y) / occupancy_grid->info.resolution;
      int index = y * occupancy_grid->info.width + x;
      occupancy_grid->data.at(index) = 100;
    }
    return occupancy_grid;
  }

  void initialize_variables()
  {
    pose_array_ = geometry_msgs::msg::PoseArray();
    pose_array_.header.frame_id = "live_map";

    live_pcl_cloud_msg_ = sensor_msgs::msg::PointCloud2();
    live_pcl_cloud_msg_.header.frame_id = "live_map";

    live_occupancy_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  }

  std::string generate_timestamp_string()
  {
    std::time_t now = std::time(nullptr);
    std::tm *ptm = std::localtime(&now);

    std::ostringstream oss;

    oss << std::put_time(ptm, "%Y-%m-%d_%H-%M-%S");

    return oss.str();
  }

  cv::Mat get_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0) {
      return cv_ptr->image.clone();
    } else {
      std::cerr << "Error image type" << std::endl;
      return cv_ptr->image.clone();
    }
  }

  void load_yaml_debug_config()
  {
    yaml_imu_frequency_ = 0.0;
    try {
      cv::FileStorage fs(settings_file_path, cv::FileStorage::READ);
      if (fs.isOpened() && !fs["IMU.Frequency"].empty()) {
        fs["IMU.Frequency"] >> yaml_imu_frequency_;
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Failed to read IMU.Frequency from %s: %s",
                  settings_file_path.c_str(), e.what());
    }
  }

  void init_debug_output()
  {
    if (!debug_enable_) return;
    RCLCPP_INFO(
      get_logger(),
      "[orb_meta] node=imu_mono_realsense sensor_type=%s "
      "image_topic=%s imu_topic=%s "
      "settings_file=%s cam_imu_time_offset=%.9f orb_time_offset_mode=%s "
      "yaml_imu_frequency=%.3f debug_expected_imu_rate_hz=%.3f "
      "debug_runtime_metrics=%d enable_video_recording=%d "
      "publish_pose_only_when_tracking_ok=%d max_image_queue_size=%ld "
      "max_imu_queue_size=%ld drop_stale_images=%d max_image_age_ms=%.3f "
      "imu_accel_axis_order=%s imu_accel_scale=(%.3f,%.3f,%.3f) "
      "imu_gyro_axis_order=%s imu_gyro_scale=(%.3f,%.3f,%.3f)",
      sensor_type_param.c_str(), image_topic_.c_str(), imu_topic_.c_str(),
      settings_file_path.c_str(), cam_imu_time_offset_,
      orb_time_offset_mode_.c_str(), yaml_imu_frequency_,
      debug_expected_imu_rate_hz_, debug_runtime_metrics_ ? 1 : 0,
      enable_video_recording_ ? 1 : 0,
      publish_pose_only_when_tracking_ok_ ? 1 : 0,
      max_image_queue_size_, max_imu_queue_size_,
      drop_stale_images_ ? 1 : 0, max_image_age_ms_,
      imu_accel_axis_order_.c_str(),
      imu_accel_scale_[0], imu_accel_scale_[1], imu_accel_scale_[2],
      imu_gyro_axis_order_.c_str(), imu_gyro_scale_[0], imu_gyro_scale_[1],
      imu_gyro_scale_[2]);
  }

  int axis_index(char axis) const
  {
    switch (axis) {
      case 'x': return 0;
      case 'y': return 1;
      case 'z': return 2;
      default: return -1;
    }
  }

  std::string load_axis_order(
    const std::string &order, int output[3], const char *name)
  {
    bool seen[3] = {false, false, false};
    bool valid = order.size() == 3;
    for (size_t index = 0; index < 3 && valid; ++index) {
      const int axis = axis_index(order[index]);
      if (axis < 0 || seen[axis]) {
        valid = false;
        break;
      }
      seen[axis] = true;
      output[index] = axis;
    }
    if (!valid) {
      RCLCPP_WARN(
        get_logger(),
        "Invalid %s='%s'; expected a permutation of xyz. Falling back to xyz.",
        name, order.c_str());
      output[0] = 0;
      output[1] = 1;
      output[2] = 2;
      return "xyz";
    }
    return order;
  }

  double effective_image_time(double image_time_sec) const
  {
    bool sync_input = image_topic_.find("/sync/") != std::string::npos;
    if (orb_time_offset_mode_ == "zero_for_sync" && sync_input) {
      return image_time_sec;
    }
    return image_time_sec + cam_imu_time_offset_;
  }

  double applied_image_offset() const
  {
    bool sync_input = image_topic_.find("/sync/") != std::string::npos;
    if (orb_time_offset_mode_ == "zero_for_sync" && sync_input) {
      return 0.0;
    }
    return cam_imu_time_offset_;
  }

  void warn_if_sync_offset_double_applied()
  {
    bool sync_input = image_topic_.find("/sync/") != std::string::npos;
    if (sync_input && std::abs(cam_imu_time_offset_) > 1e-9 &&
        orb_time_offset_mode_ == "legacy_warn") {
      RCLCPP_WARN(
        get_logger(),
        "ORB is subscribing to synchronized image topic '%s' and still applying "
        "cam_imu_time_offset=%.6f s before TrackMonocular. This may be a "
        "second offset; debug logger records image_time_sec and track_time_sec.",
        image_topic_.c_str(), cam_imu_time_offset_);
    }
  }

  std::string tracking_state_to_string(int state) const
  {
    switch (state) {
      case -1: return "SYSTEM_NOT_READY";
      case 0: return "NO_IMAGES_YET";
      case 1: return "NOT_INITIALIZED";
      case 2: return "OK";
      case 3: return "RECENTLY_LOST";
      case 4: return "LOST";
      case 5: return "OK_KLT";
      default: return "UNKNOWN";
    }
  }

  double nan_if_unavailable(bool available, float value) const
  {
    return available ? static_cast<double>(value)
                     : std::numeric_limits<double>::quiet_NaN();
  }

  void check_imu_rate_yaml_consistency()
  {
    if (imu_yaml_warned_ || observed_imu_rate_hz_ <= 0.0 ||
        yaml_imu_frequency_ <= 0.0) {
      return;
    }
    double diff = std::abs(observed_imu_rate_hz_ - yaml_imu_frequency_);
    double threshold = std::max(5.0, yaml_imu_frequency_ * 0.10);
    if (diff > threshold) {
      RCLCPP_WARN(
        get_logger(),
        "Observed IMU rate %.1f Hz does not match ORB YAML IMU.Frequency %.1f Hz",
        observed_imu_rate_hz_, yaml_imu_frequency_);
      imu_yaml_warned_ = true;
    }
  }

  bool is_tracking_ok_state(int state) const
  {
    return state == 2 || state == 5;
  }

  void write_debug_queue_1hz(double processed_image_age_ms)
  {
    if (!debug_enable_ || !debug_runtime_metrics_) return;

    int64_t now = steady_now_ns();
    if (debug_last_queue_ns_ != 0 &&
        now - debug_last_queue_ns_ < 1'000'000'000LL) {
      if (processed_image_age_ms >= 0.0) {
        debug_last_processed_image_age_ms_ = processed_image_age_ms;
      }
      return;
    }

    size_t img_buf_size = 0;
    size_t imu_buf_size = 0;
    double oldest_img_age_ms = 0.0;
    double newest_img_age_ms = 0.0;
    {
      std::lock_guard<std::mutex> lock(buf_mutex_img_);
      img_buf_size = img_buf_.size();
      if (!img_buf_.empty()) {
        oldest_img_age_ms = ns_to_ms(now - img_buf_.front().receive_steady_ns);
        newest_img_age_ms = ns_to_ms(now - img_buf_.back().receive_steady_ns);
      }
    }
    {
      std::lock_guard<std::mutex> lock(buf_mutex_imu_);
      imu_buf_size = imu_buf_.size();
    }

    const uint64_t image_total = input_image_count_total_.load();
    const uint64_t imu_total = input_imu_count_total_.load();
    const uint64_t orb_total = debug_orb_frame_count_.load();
    const double elapsed = debug_last_queue_ns_ == 0
      ? 0.0 : (now - debug_last_queue_ns_) / 1e9;
    const double image_fps = elapsed > 0.0
      ? (image_total - debug_last_queue_image_count_) / elapsed : 0.0;
    const double imu_fps = elapsed > 0.0
      ? (imu_total - debug_last_queue_imu_count_) / elapsed : 0.0;
    const double orb_fps = elapsed > 0.0
      ? (orb_total - debug_last_queue_orb_count_) / elapsed : 0.0;

    if (processed_image_age_ms >= 0.0) {
      debug_last_processed_image_age_ms_ = processed_image_age_ms;
    }
    debug_last_queue_ns_ = now;
    debug_last_queue_image_count_ = image_total;
    debug_last_queue_imu_count_ = imu_total;
    debug_last_queue_orb_count_ = orb_total;

    RCLCPP_INFO(
      get_logger(),
      "[orb_queue] steady_ns=%ld img_buf_size=%zu imu_buf_size=%zu "
      "dropped_img_count=%lu dropped_imu_count=%lu oldest_img_age_ms=%.3f "
      "newest_img_age_ms=%.3f processed_image_age_ms=%.3f "
      "input_image_fps_est=%.3f input_imu_fps_est=%.3f "
      "orb_processed_fps_est=%.3f",
      now, img_buf_size, imu_buf_size, dropped_img_count_.load(),
      dropped_imu_count_.load(), oldest_img_age_ms, newest_img_age_ms,
      debug_last_processed_image_age_ms_, image_fps, imu_fps, orb_fps);
  }

  void write_debug_timing(
    uint64_t frame_id, int64_t image_header_ns,
    const FrameRuntimeMetrics &metrics, bool track_called,
    const std::string &skip_reason)
  {
    if (!debug_enable_ || !debug_runtime_metrics_) return;
    RCLCPP_INFO(
      get_logger(),
      "[orb_timing] frame_id=%lu image_stamp=%ld "
      "receive_steady_ns=%ld track_begin_steady_ns=%ld "
      "track_end_steady_ns=%ld cv_bridge_ms=%.3f wait_for_imu_ms=%.3f "
      "track_ms=%.3f video_write_ms=%.3f total_frame_ms=%.3f "
      "image_age_at_track_begin_ms=%.3f image_age_at_track_end_ms=%.3f "
      "processed_image_age_ms=%.3f track_called=%d skip_reason=%s",
      frame_id, image_header_ns, metrics.receive_steady_ns,
      metrics.track_begin_steady_ns, metrics.track_end_steady_ns,
      metrics.cv_bridge_ms, metrics.wait_for_imu_ms, metrics.track_ms,
      metrics.video_write_ms, metrics.total_frame_ms,
      metrics.image_age_at_track_begin_ms, metrics.image_age_at_track_end_ms,
      metrics.processed_image_age_ms, track_called ? 1 : 0,
      skip_reason.empty() ? "none" : skip_reason.c_str());
  }

  void write_debug_imu_window(
    uint64_t frame_id, double image_time_sec, double track_time_sec,
    size_t imu_count, const FrameRuntimeMetrics &metrics)
  {
    if (!debug_enable_ || !debug_runtime_metrics_) return;
    RCLCPP_INFO(
      get_logger(),
      "[orb_imu_window] frame_id=%lu image_time_sec=%.9f "
      "track_time_sec=%.9f imu_count=%zu first_imu_time_sec=%.9f "
      "last_imu_time_sec=%.9f imu_span_ms=%.3f "
      "image_to_first_imu_ms=%.3f image_to_last_imu_ms=%.3f",
      frame_id, image_time_sec, track_time_sec, imu_count,
      metrics.first_imu_time_sec, metrics.last_imu_time_sec,
      metrics.imu_span_ms, metrics.image_to_first_imu_ms,
      metrics.image_to_last_imu_ms);
  }

  void write_debug_state_transition(const ORB_SLAM3::System::DebugState &state)
  {
    if (!debug_enable_ || !debug_runtime_metrics_) return;
    if (!debug_have_last_state_) {
      debug_have_last_state_ = true;
      debug_last_tracking_state_ = state.tracking_state;
      debug_last_imu_initialized_ = state.imu_initialized;
      debug_last_reset_count_ = state.reset_count;
      debug_last_map_id_ = state.map_id;
      return;
    }
    if (debug_last_tracking_state_ == state.tracking_state &&
        debug_last_imu_initialized_ == state.imu_initialized &&
        debug_last_reset_count_ == state.reset_count &&
        debug_last_map_id_ == state.map_id) {
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "[orb_state_transition] old_state=%s new_state=%s "
      "old_imu_initialized=%d new_imu_initialized=%d "
      "old_reset_count=%lu new_reset_count=%lu map_id=%ld frame_id=%lu "
      "last_reset_reason=%s",
      tracking_state_to_string(debug_last_tracking_state_).c_str(),
      tracking_state_to_string(state.tracking_state).c_str(),
      debug_last_imu_initialized_ ? 1 : 0, state.imu_initialized ? 1 : 0,
      debug_last_reset_count_, state.reset_count, state.map_id,
      state.frame_id, state.last_reset_reason.c_str());
    debug_last_tracking_state_ = state.tracking_state;
    debug_last_imu_initialized_ = state.imu_initialized;
    debug_last_reset_count_ = state.reset_count;
    debug_last_map_id_ = state.map_id;
  }

  void write_debug_visual_quality(
    const ORB_SLAM3::System::DebugState &state)
  {
    if (!debug_enable_ || !debug_runtime_metrics_) return;
    RCLCPP_INFO(
      get_logger(),
      "[orb_visual_quality] frame_id=%lu num_features=%d "
      "num_tracked_mappoints=%d num_inliers=%d num_keyframes=%lu "
      "num_matches=%d tracking_state=%s",
      state.frame_id, state.num_features, state.num_tracked_mappoints,
      state.num_inliers, state.num_keyframes, state.num_matches,
      tracking_state_to_string(state.tracking_state).c_str());
  }

  void write_debug_pose_gate(
    const ORB_SLAM3::System::DebugState &state,
    bool publish_pose, const std::string &reason)
  {
    if (!debug_enable_ || !debug_runtime_metrics_) return;
    RCLCPP_INFO(
      get_logger(),
      "[orb_pose_gate] tracking_state=%s imu_initialized=%d "
      "reset_count=%lu has_pose=%d publish_pose=%d reason=%s",
      tracking_state_to_string(state.tracking_state).c_str(),
      state.imu_initialized ? 1 : 0, state.reset_count,
      state.has_pose ? 1 : 0, publish_pose ? 1 : 0, reason.c_str());
  }

  void write_debug_track(
    int64_t image_header_ns, double image_time_sec, double track_time_sec,
    const std::vector<ORB_SLAM3::IMU::Point> &imu_meas,
    bool track_called, const std::string &skip_reason,
    const FrameRuntimeMetrics &metrics)
  {
    if (!debug_enable_) return;
    const auto state = orb_slam3_system_->GetDebugState();
    const auto events = orb_slam3_system_->PopDebugEvents();
    write_debug_state_transition(state);
    write_debug_timing(state.frame_id, image_header_ns, metrics, track_called,
                       skip_reason);
    write_debug_imu_window(state.frame_id, image_time_sec, track_time_sec,
                           imu_meas.size(), metrics);
    write_debug_visual_quality(state);
    const double pose_x = nan_if_unavailable(state.has_pose, state.pose.x());
    const double pose_y = nan_if_unavailable(state.has_pose, state.pose.y());
    const double pose_z = nan_if_unavailable(state.has_pose, state.pose.z());
    const double velocity_x =
      nan_if_unavailable(state.has_velocity, state.velocity.x());
    const double velocity_y =
      nan_if_unavailable(state.has_velocity, state.velocity.y());
    const double velocity_z =
      nan_if_unavailable(state.has_velocity, state.velocity.z());

    if (state.has_pose && !debug_pose_baseline_set_) {
      debug_pose_baseline_set_ = true;
      debug_pose_base_t_ = state.pose;
    }
    if (state.has_pose) {
      debug_last_pose_drift_m_ = (state.pose - debug_pose_base_t_).norm();
    }

    for (const auto &event : events) {
      const double parallax_margin =
        event.initialization_parallax - event.required_parallax;
      std::ostringstream event_oss;
      event_oss << std::fixed << std::setprecision(9)
                << "[orb_event] "
                << "event_code=" << event.event_code << ' '
                << "frame_id=" << event.frame_id << ' '
                << "map_id=" << event.map_id << ' '
                << "reason=" << event.reason << ' '
                << "num_matches=" << event.num_matches << ' '
                << "num_inliers=" << event.num_inliers << ' '
                << "median_parallax=" << event.median_parallax << ' '
                << "initialization_parallax=" << event.initialization_parallax << ' '
                << "required_parallax=" << event.required_parallax << ' '
                << "parallax_margin=" << parallax_margin << ' '
                << "estimated_scale=" << event.estimated_scale << ' '
                << "gravity_norm=" << event.gravity_norm << ' '
                << "bias_norm=" << event.bias_norm << ' '
                << "motion_dist_m=" << event.motion_dist_m << ' '
                << "required_motion_dist_m=" << event.required_motion_dist_m << ' '
                << "motion_time_sec=" << event.motion_time_sec << ' '
                << "required_motion_time_sec=" << event.required_motion_time_sec;
      std::lock_guard<std::mutex> lock(debug_mutex_);
      RCLCPP_INFO(get_logger(), "%s", event_oss.str().c_str());
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9)
        << "[orb_track] "
        << "frame_id=" << state.frame_id << ' '
        << "track_time_sec=" << track_time_sec << ' '
        << "imu_count=" << imu_meas.size() << ' '
        << "image_to_last_imu_ms=" << metrics.image_to_last_imu_ms << ' '
        << "tracking_state=" << tracking_state_to_string(state.tracking_state) << ' '
        << "imu_initialized=" << (state.imu_initialized ? 1 : 0) << ' '
        << "is_lost=" << (state.is_lost ? 1 : 0) << ' '
        << "recently_lost=" << (state.recently_lost ? 1 : 0) << ' '
        << "map_id=" << state.map_id << ' '
        << "reset_count=" << state.reset_count << ' '
        << "last_reset_reason=" << state.last_reset_reason << ' '
        << "num_features=" << state.num_features << ' '
        << "orb_init_reason_code=" << state.orb_init_reason_code << ' '
        << "num_matches=" << state.num_matches << ' '
        << "num_inliers=" << state.num_inliers << ' '
        << "num_tracked_mappoints=" << state.num_tracked_mappoints << ' '
        << "num_keyframes=" << state.num_keyframes << ' '
        << "median_parallax=" << state.median_parallax << ' '
        << "initialization_parallax=" << state.initialization_parallax << ' '
        << "required_parallax=" << state.required_parallax << ' '
        << "estimated_scale=" << state.estimated_scale << ' '
        << "gravity_norm=" << state.gravity_norm << ' '
        << "bias_norm=" << state.bias_norm << ' '
        << "bias_gx=" << state.bias_gyro.x() << ' '
        << "bias_gy=" << state.bias_gyro.y() << ' '
        << "bias_gz=" << state.bias_gyro.z() << ' '
        << "bias_ax=" << state.bias_acc.x() << ' '
        << "bias_ay=" << state.bias_acc.y() << ' '
        << "bias_az=" << state.bias_acc.z() << ' '
        << "velocity_x=" << velocity_x << ' '
        << "velocity_y=" << velocity_y << ' '
        << "velocity_z=" << velocity_z << ' '
        << "gravity_x=" << state.gravity.x() << ' '
        << "gravity_y=" << state.gravity.y() << ' '
        << "gravity_z=" << state.gravity.z() << ' '
        << "pose_x=" << pose_x << ' '
        << "pose_y=" << pose_y << ' '
        << "pose_z=" << pose_z << ' '
        << "pose_drift_m=" << debug_last_pose_drift_m_ << ' '
        << "track_called=" << (track_called ? 1 : 0) << ' '
        << "skip_reason=" << (skip_reason.empty() ? "none" : skip_reason) << ' '
        << "image_header_ns=" << image_header_ns << ' '
        << "image_time_sec=" << image_time_sec << ' '
        << "applied_offset_sec=" << applied_image_offset() << ' '
        << "offset_mode=" << orb_time_offset_mode_;
    {
      std::lock_guard<std::mutex> lock(debug_mutex_);
      RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
    }
  }

  void write_debug_pose(const Sophus::SE3f &Twc)
  {
    if (!debug_enable_) return;
    Eigen::Vector3f t = Twc.translation();
    double yaw = tf2::getYaw(tf2::Quaternion(
      Twc.unit_quaternion().x(), Twc.unit_quaternion().y(),
      Twc.unit_quaternion().z(), Twc.unit_quaternion().w()));
    if (!debug_pose_baseline_set_) {
      debug_pose_baseline_set_ = true;
      debug_pose_base_t_ = t;
      debug_pose_base_yaw_ = yaw;
    }
    debug_last_pose_drift_m_ = (t - debug_pose_base_t_).norm();
    debug_last_pose_yaw_drift_rad_ = yaw - debug_pose_base_yaw_;
    std::lock_guard<std::mutex> lock(debug_mutex_);
    RCLCPP_INFO(
      get_logger(),
      "[orb_pose] pose_index=%lu steady_ns=%ld x=%.9f y=%.9f z=%.9f "
      "qx=%.9f qy=%.9f qz=%.9f qw=%.9f drift_translation_m=%.9f "
      "drift_yaw_rad=%.9f",
      debug_pose_count_++, steady_now_ns(), t.x(), t.y(), t.z(),
      Twc.unit_quaternion().x(), Twc.unit_quaternion().y(),
      Twc.unit_quaternion().z(), Twc.unit_quaternion().w(),
      debug_last_pose_drift_m_, debug_last_pose_yaw_drift_rad_);
  }

  void write_debug_summary_1hz()
  {
    if (!debug_enable_) return;
    int64_t now = steady_now_ns();
    if (debug_last_summary_ns_ != 0 &&
        now - debug_last_summary_ns_ < 1'000'000'000LL) {
      return;
    }
    double elapsed = debug_last_summary_ns_ == 0
      ? 0.0 : (now - debug_last_summary_ns_) / 1e9;
    const uint64_t orb_count = debug_orb_frame_count_.load();
    double fps = elapsed > 0.0
      ? (orb_count - debug_last_summary_frame_count_) / elapsed
      : 0.0;
    debug_last_summary_ns_ = now;
    debug_last_summary_frame_count_ = orb_count;
    std::lock_guard<std::mutex> lock(debug_mutex_);
    RCLCPP_INFO(
      get_logger(),
      "[orb_summary] steady_ns=%ld orb_frames=%lu orb_fps=%.6f "
      "observed_imu_rate_hz=%.3f yaml_imu_frequency=%.3f "
      "debug_expected_imu_rate_hz=%.3f pose_drift_m=%.9f "
      "pose_yaw_drift_rad=%.9f "
      "gyro_mean=(%.5f,%.5f,%.5f) acc_mean=(%.5f,%.5f,%.5f)",
      now, orb_count, fps, observed_imu_rate_hz_,
      yaml_imu_frequency_, debug_expected_imu_rate_hz_,
      debug_last_pose_drift_m_, debug_last_pose_yaw_drift_rad_,
      debug_gyro_mean_[0], debug_gyro_mean_[1], debug_gyro_mean_[2],
      debug_acc_mean_[0], debug_acc_mean_[1], debug_acc_mean_[2]);
  }

  // ── image_callback: lightweight, only buffers ───────────────────
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    const int64_t receive_ns = steady_now_ns();
    input_image_count_total_++;

    // Republish with ROS time for external consumers
    rclcpp::Time time_now = get_clock()->now();
    sensor_msgs::msg::Image::SharedPtr msg_out =
      std::make_shared<sensor_msgs::msg::Image>(*msg);
    msg_out->header.stamp = time_now;
    msg_out->header.frame_id = "base_link";
    orb_image_publisher_->publish(*msg_out);

    // Push to buffer; SyncWithImu thread will consume
    {
      std::lock_guard<std::mutex> lock(buf_mutex_img_);
      if (max_image_queue_size_ > 0) {
        while (static_cast<int64_t>(img_buf_.size()) >= max_image_queue_size_) {
          img_buf_.pop();
          dropped_img_count_++;
        }
      }
      img_buf_.push(QueuedImage{msg, receive_ns});
    }
    write_debug_queue_1hz(-1.0);
  }

  // Dedicated tracking thread. IMU modes wait for IMU coverage; pure Monocular
  // mode consumes images directly.
  void SyncWithImu()
  {
    while (sync_running_ && rclcpp::ok()) {
      double tIm = 0;
      bool ready = false;
      const bool use_imu = sensor_type_param != "monocular";

      if (use_imu) {
        std::lock_guard<std::mutex> lock_img(buf_mutex_img_);
        std::lock_guard<std::mutex> lock_imu(buf_mutex_imu_);
        if (!img_buf_.empty() && !imu_buf_.empty()) {
          auto imgPtr = img_buf_.front().msg;
          double image_time =
            imgPtr->header.stamp.sec + imgPtr->header.stamp.nanosec * 1e-9;
          tIm = effective_image_time(image_time);

          // Wait until IMU data covers past the image timestamp
          double tIMU_back = imu_buf_.back().msg->header.stamp.sec
                           + imu_buf_.back().msg->header.stamp.nanosec * 1e-9;
          ready = (tIm <= tIMU_back);
        }
      } else {
        std::lock_guard<std::mutex> lock_img(buf_mutex_img_);
        ready = !img_buf_.empty();
      }

      if (!ready) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kSyncSleepMs));
        continue;
      }

      // Pop image
      QueuedImage queued_image;
      {
        std::lock_guard<std::mutex> lock(buf_mutex_img_);
        queued_image = img_buf_.front();
        img_buf_.pop();
      }
      auto imgPtr = queued_image.msg;

      FrameRuntimeMetrics metrics;
      metrics.receive_steady_ns = queued_image.receive_steady_ns;
      const int64_t frame_begin_ns = steady_now_ns();
      metrics.wait_for_imu_ms =
        ns_to_ms(frame_begin_ns - queued_image.receive_steady_ns);
      double image_time =
        imgPtr->header.stamp.sec + imgPtr->header.stamp.nanosec * 1e-9;
      int64_t image_header_ns =
        static_cast<int64_t>(imgPtr->header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(imgPtr->header.stamp.nanosec);
      tIm = effective_image_time(image_time);

      if (drop_stale_images_ &&
          metrics.wait_for_imu_ms > max_image_age_ms_) {
        dropped_img_count_++;
        metrics.track_begin_steady_ns = frame_begin_ns;
        metrics.track_end_steady_ns = frame_begin_ns;
        metrics.image_age_at_track_begin_ms = metrics.wait_for_imu_ms;
        metrics.image_age_at_track_end_ms = metrics.wait_for_imu_ms;
        metrics.processed_image_age_ms = metrics.wait_for_imu_ms;
        write_debug_track(image_header_ns, image_time, tIm, {},
                          false, "stale_image", metrics);
        write_debug_queue_1hz(metrics.processed_image_age_ms);
        continue;
      }

      const int64_t cv_begin_ns = steady_now_ns();
      cv::Mat imageFrame = get_image(imgPtr);
      const int64_t cv_end_ns = steady_now_ns();
      metrics.cv_bridge_ms = ns_to_ms(cv_end_ns - cv_begin_ns);

      // Collect all IMU measurements up to image timestamp
      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      if (use_imu) {
        std::lock_guard<std::mutex> lock(buf_mutex_imu_);
        while (!imu_buf_.empty()) {
          auto imuPtr = imu_buf_.front().msg;
          double tIMU =
            imuPtr->header.stamp.sec + imuPtr->header.stamp.nanosec * 1e-9;
          if (tIMU > tIm) break;
          imu_buf_.pop();
          const double raw_acc[3] = {
            imuPtr->linear_acceleration.x,
            imuPtr->linear_acceleration.y,
            imuPtr->linear_acceleration.z};
          const double raw_gyr[3] = {
            imuPtr->angular_velocity.x,
            imuPtr->angular_velocity.y,
            imuPtr->angular_velocity.z};
          cv::Point3f acc(
            raw_acc[imu_accel_axis_index_[0]] * imu_accel_scale_[0],
            raw_acc[imu_accel_axis_index_[1]] * imu_accel_scale_[1],
            raw_acc[imu_accel_axis_index_[2]] * imu_accel_scale_[2]);
          cv::Point3f gyr(
            raw_gyr[imu_gyro_axis_index_[0]] * imu_gyro_scale_[0],
            raw_gyr[imu_gyro_axis_index_[1]] * imu_gyro_scale_[1],
            raw_gyr[imu_gyro_axis_index_[2]] * imu_gyro_scale_[2]);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, tIMU));
        }
      }
      if (!vImuMeas.empty()) {
        metrics.first_imu_time_sec = vImuMeas.front().t;
        metrics.last_imu_time_sec = vImuMeas.back().t;
        metrics.imu_span_ms =
          (metrics.last_imu_time_sec - metrics.first_imu_time_sec) * 1000.0;
        metrics.image_to_first_imu_ms =
          (tIm - metrics.first_imu_time_sec) * 1000.0;
        metrics.image_to_last_imu_ms =
          (tIm - metrics.last_imu_time_sec) * 1000.0;
      }

      // Track
      bool track_called = false;
      std::string skip_reason;
      metrics.track_begin_steady_ns = steady_now_ns();
      metrics.image_age_at_track_begin_ms =
        ns_to_ms(metrics.track_begin_steady_ns - queued_image.receive_steady_ns);
      try {
        const int64_t track_call_begin_ns = steady_now_ns();
        if (!use_imu) {
          auto Tcw = orb_slam3_system_->TrackMonocular(imageFrame, tIm);
          {
            std::lock_guard<std::mutex> lock(orbslam3_mutex_);
            Tcw_.translation() = Tcw.translation();
            Tcw_.setQuaternion(Tcw.unit_quaternion());
          }
          track_called = true;
        } else if (vImuMeas.size() > 1) {
          auto Tcw =
            orb_slam3_system_->TrackMonocular(imageFrame, tIm, vImuMeas);
          {
            std::lock_guard<std::mutex> lock(orbslam3_mutex_);
            Tcw_.translation() = Tcw.translation();
            Tcw_.setQuaternion(Tcw.unit_quaternion());
          }
          track_called = true;
        } else {
          skip_reason = "insufficient_imu";
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Only %zu IMU meas for frame at t=%.3f, skipping",
            vImuMeas.size(), tIm);
        }
        metrics.track_ms = ns_to_ms(steady_now_ns() - track_call_begin_ns);
        if (enable_video_recording_ && video_writer_.isOpened()) {
          const int64_t video_begin_ns = steady_now_ns();
          video_writer_.write(imageFrame);
          metrics.video_write_ms = ns_to_ms(steady_now_ns() - video_begin_ns);
        }
      } catch (const std::exception &e) {
        skip_reason = "exception";
        RCLCPP_ERROR(get_logger(), "SLAM processing exception: %s", e.what());
      }
      if (track_called) {
        debug_orb_frame_count_++;
      }
      metrics.track_end_steady_ns = steady_now_ns();
      metrics.image_age_at_track_end_ms =
        ns_to_ms(metrics.track_end_steady_ns - queued_image.receive_steady_ns);
      metrics.processed_image_age_ms = metrics.image_age_at_track_end_ms;
      metrics.total_frame_ms = ns_to_ms(metrics.track_end_steady_ns - frame_begin_ns);
      write_debug_track(image_header_ns, image_time, tIm, vImuMeas,
                        track_called, skip_reason, metrics);
      write_debug_queue_1hz(metrics.processed_image_age_ms);

      std::this_thread::sleep_for(std::chrono::milliseconds(kSyncSleepMs));
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu &msg)
  {
    int64_t now_ns = steady_now_ns();
    input_imu_count_total_++;
    if (debug_enable_) {
      debug_imu_window_count_++;
      debug_imu_total_count_++;
      double n = static_cast<double>(debug_imu_total_count_);
      debug_gyro_mean_[0] += (msg.angular_velocity.x - debug_gyro_mean_[0]) / n;
      debug_gyro_mean_[1] += (msg.angular_velocity.y - debug_gyro_mean_[1]) / n;
      debug_gyro_mean_[2] += (msg.angular_velocity.z - debug_gyro_mean_[2]) / n;
      debug_acc_mean_[0] +=
        (msg.linear_acceleration.x - debug_acc_mean_[0]) / n;
      debug_acc_mean_[1] +=
        (msg.linear_acceleration.y - debug_acc_mean_[1]) / n;
      debug_acc_mean_[2] +=
        (msg.linear_acceleration.z - debug_acc_mean_[2]) / n;
      if (debug_imu_rate_window_start_ns_ == 0) {
        debug_imu_rate_window_start_ns_ = now_ns;
      } else if (now_ns - debug_imu_rate_window_start_ns_ >= 1000000000LL) {
        double elapsed = (now_ns - debug_imu_rate_window_start_ns_) / 1e9;
        observed_imu_rate_hz_ = debug_imu_window_count_ / elapsed;
        debug_imu_window_count_ = 0;
        debug_imu_rate_window_start_ns_ = now_ns;
        check_imu_rate_yaml_consistency();
      }
    }
    sensor_msgs::msg::Imu msg_out = msg;
    msg_out.header.stamp = get_clock()->now();
    msg_out.header.frame_id = "base_link";
    // imu_publisher_->publish(msg_out);
    if (!std::isnan(msg.linear_acceleration.x) &&
        !std::isnan(msg.linear_acceleration.y) &&
        !std::isnan(msg.linear_acceleration.z) &&
        !std::isnan(msg.angular_velocity.x) &&
        !std::isnan(msg.angular_velocity.y) &&
        !std::isnan(msg.angular_velocity.z)) {
      const sensor_msgs::msg::Imu::SharedPtr msg_ptr =
        std::make_shared<sensor_msgs::msg::Imu>(msg);
      {
        std::lock_guard<std::mutex> lock(buf_mutex_imu_);
        if (max_imu_queue_size_ > 0) {
          while (static_cast<int64_t>(imu_buf_.size()) >= max_imu_queue_size_) {
            imu_buf_.pop();
            dropped_imu_count_++;
          }
        }
        imu_buf_.push(QueuedImu{msg_ptr, now_ns});
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid IMU data - nan");
    }
    write_debug_queue_1hz(-1.0);
  }

  void timer_callback()
  {
    unique_lock<mutex> lock(orbslam3_mutex_);

    const auto state = orb_slam3_system_->GetDebugState();
    bool publish_pose = state.has_pose;
    std::string gate_reason = "ok";
    if (!state.has_pose) {
      publish_pose = false;
      gate_reason = "no_pose";
    } else if (publish_pose_only_when_tracking_ok_ &&
               !is_tracking_ok_state(state.tracking_state)) {
      publish_pose = false;
      gate_reason = "tracking_not_ok";
    }
    write_debug_pose_gate(state, publish_pose, gate_reason);
    if (!publish_pose) {
      write_debug_summary_1hz();
      return;
    }

    rclcpp::Time time_now = get_clock()->now();
    auto Twc = Tcw_.inverse();
    write_debug_pose(Twc);

    tf2::Quaternion q_orig(
      Twc.unit_quaternion().x(), Twc.unit_quaternion().y(),
      Twc.unit_quaternion().z(), Twc.unit_quaternion().w());

      tf2::Matrix3x3 m(q_orig);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      tf2::Quaternion q_yaw;
      q_yaw.setRPY(0, 0, yaw);

      tf2::Quaternion q_rot_x, q_rot_z;
      // q_rot_x.setRPY(M_PI / 2.0, 0, 0);
      q_rot_z.setRPY(0, 0, M_PI / 2.0);

      // change camera coordinate to map coordinates
      tf2::Quaternion q_combined = q_rot_z * q_yaw;
      q_combined.normalize();

      geometry_msgs::msg::TransformStamped odom_tf;
      odom_tf.header.stamp = time_now;
      odom_tf.header.frame_id = "odom";
      odom_tf.child_frame_id = "base_link";
      odom_tf.transform.translation.x = Twc.translation().x();
      odom_tf.transform.translation.y = Twc.translation().y();
      odom_tf.transform.translation.z = Twc.translation().z();
      odom_tf.transform.rotation.x = Twc.unit_quaternion().x();
      odom_tf.transform.rotation.y = Twc.unit_quaternion().y();
      odom_tf.transform.rotation.z = Twc.unit_quaternion().z();
      odom_tf.transform.rotation.w = Twc.unit_quaternion().w();
      tf_broadcaster->sendTransform(odom_tf);

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = time_now;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = Twc.translation().x();
      odom.pose.pose.position.y = Twc.translation().y();
      odom.pose.pose.position.z = Twc.translation().z();
      odom.pose.pose.orientation.x = Twc.unit_quaternion().x();
      odom.pose.pose.orientation.y = Twc.unit_quaternion().y();
      odom.pose.pose.orientation.z = Twc.unit_quaternion().z();
      odom.pose.pose.orientation.w = Twc.unit_quaternion().w();
      odom_publisher_->publish(odom);

      geometry_msgs::msg::Pose pose;
      pose.position.x = Twc.translation().x();
      pose.position.y = Twc.translation().y();
      // pose.position.z = Twc.translation().z();
      pose.orientation.x = q_combined.x();
      pose.orientation.y = q_combined.y();
      pose.orientation.z = q_combined.z();
      pose.orientation.w = q_combined.w();
      pose_array_.header.stamp = time_now;
      pose_array_.poses.push_back(pose);
      pose_array_publisher_->publish(pose_array_);

      // geometry_msgs::msg::TransformStamped base_link_tf;
      // base_link_tf.header.stamp = time_now;
      // base_link_tf.header.frame_id = "live_map";
      // base_link_tf.child_frame_id = "base_link";
      // base_link_tf.transform.translation.x = Twc.translation().x();
      // base_link_tf.transform.translation.y = Twc.translation().y();
      // base_link_tf.transform.rotation.x = q_combined.x();
      // base_link_tf.transform.rotation.y = q_combined.y();
      // base_link_tf.transform.rotation.z = q_combined.z();
      // base_link_tf.transform.rotation.w = q_combined.w();
      // tf_broadcaster->sendTransform(base_link_tf);

      geometry_msgs::msg::TransformStamped point_cloud_tf;
      point_cloud_tf.header.stamp = time_now;
      point_cloud_tf.header.frame_id = "map";
      point_cloud_tf.child_frame_id = "point_cloud";
      tf_broadcaster->sendTransform(point_cloud_tf);

      geometry_msgs::msg::TransformStamped live_map_tf;
      live_map_tf.header.stamp = time_now;
      live_map_tf.header.frame_id = "map";
      live_map_tf.child_frame_id = "live_map";
      tf_broadcaster->sendTransform(live_map_tf);

      // Some custom forks expose GetMapPCL(); vanilla ORB_SLAM3 does not.
      // Keep the node functional without live map cloud publishing.
      write_debug_summary_1hz();
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    live_point_cloud_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
    pose_array_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    live_occupancy_grid_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr orb_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::CallbackGroup::SharedPtr image_callback_group_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr slam_service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  sensor_msgs::msg::Imu imu_msg;
  geometry_msgs::msg::PoseArray pose_array_;

  std::string sensor_type_param;
  bool use_pangolin;
  std::string image_topic_;
  std::string imu_topic_;
  double cam_imu_time_offset_{0.0};
  bool debug_enable_{false};
  bool debug_runtime_metrics_{true};
  bool enable_video_recording_{false};
  bool publish_pose_only_when_tracking_ok_{true};
  int64_t max_image_queue_size_{2};
  int64_t max_imu_queue_size_{500};
  bool drop_stale_images_{true};
  double max_image_age_ms_{200.0};
  std::string orb_time_offset_mode_{"legacy_warn"};
  double debug_expected_imu_rate_hz_{0.0};
  double imu_accel_scale_[3]{1.0, 1.0, 1.0};
  double imu_gyro_scale_[3]{1.0, 1.0, 1.0};
  int imu_accel_axis_index_[3]{0, 1, 2};
  int imu_gyro_axis_index_[3]{0, 1, 2};
  std::string imu_accel_axis_order_{"xyz"};
  std::string imu_gyro_axis_order_{"xyz"};
  double yaml_imu_frequency_{0.0};
  double observed_imu_rate_hz_{0.0};
  bool imu_yaml_warned_{false};
  std::mutex debug_mutex_;
  std::atomic<uint64_t> debug_orb_frame_count_{0};
  uint64_t debug_last_summary_frame_count_{0};
  uint64_t debug_pose_count_{0};
  uint64_t debug_imu_total_count_{0};
  uint64_t debug_imu_window_count_{0};
  int64_t debug_imu_rate_window_start_ns_{0};
  int64_t debug_last_summary_ns_{0};
  int64_t debug_last_queue_ns_{0};
  uint64_t debug_last_queue_image_count_{0};
  uint64_t debug_last_queue_imu_count_{0};
  uint64_t debug_last_queue_orb_count_{0};
  double debug_last_processed_image_age_ms_{0.0};
  bool debug_have_last_state_{false};
  int debug_last_tracking_state_{-999};
  bool debug_last_imu_initialized_{false};
  unsigned long debug_last_reset_count_{0};
  long debug_last_map_id_{-1};
  std::atomic<uint64_t> input_image_count_total_{0};
  std::atomic<uint64_t> input_imu_count_total_{0};
  std::atomic<uint64_t> dropped_img_count_{0};
  std::atomic<uint64_t> dropped_imu_count_{0};
  double debug_gyro_mean_[3]{0.0, 0.0, 0.0};
  double debug_acc_mean_[3]{0.0, 0.0, 0.0};
  bool debug_pose_baseline_set_{false};
  Eigen::Vector3f debug_pose_base_t_{0.0f, 0.0f, 0.0f};
  double debug_pose_base_yaw_{0.0};
  double debug_last_pose_drift_m_{0.0};
  double debug_last_pose_yaw_drift_rad_{0.0};

  std::vector<geometry_msgs::msg::Vector3> vGyro;
  std::vector<double> vGyro_times;
  std::vector<geometry_msgs::msg::Vector3> vAccel;
  std::vector<double> vAccel_times;

  std::queue<QueuedImu> imu_buf_;
  std::queue<QueuedImage> img_buf_;
  std::mutex buf_mutex_imu_, buf_mutex_img_, orbslam3_mutex_,
    live_pcl_cloud_mutex_;

  std::shared_ptr<ORB_SLAM3::System> orb_slam3_system_;
  std::string vocabulary_file_path;
  std::string settings_file_path;

  sensor_msgs::msg::PointCloud2 live_pcl_cloud_msg_;
  pcl::PointCloud<pcl::PointXYZ> live_pcl_cloud_;
  nav_msgs::msg::OccupancyGrid::SharedPtr live_occupancy_grid_;

  bool inertial_ba1_;
  bool inertial_ba2_;
  Sophus::SE3f Tcw_;

  cv::VideoWriter video_writer_;
  std::string timestamp_;

  std::thread sync_thread_;
  std::atomic<bool> sync_running_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuMonoRealSense>();
  // Use MultiThreadedExecutor so IMU and image callbacks run in parallel
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
