#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
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
#include <tf2_ros/transform_broadcaster.h>

#ifdef ORB_SLAM3_ROS2_HAS_NAV2_MAP_SERVER
#include "nav2_map_server/map_io.hpp"
#endif

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <sstream>
#include <vector>

#include <cv_bridge/cv_bridge.h>

// this is orb_slam3
#include "System.h"

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

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

/// Lightweight IMU sample for interpolation
struct ImuSample {
  double t;
  cv::Point3f acc;
  cv::Point3f gyr;
};

/// Linear interpolation between two IMU samples at target time
static ORB_SLAM3::IMU::Point interpolate_imu(
  const ImuSample &s0, const ImuSample &s1, double t_target)
{
  double alpha = (s1.t > s0.t)
    ? std::max(0.0, std::min((t_target - s0.t) / (s1.t - s0.t), 1.0))
    : 0.0;
  float a = static_cast<float>(alpha);
  cv::Point3f acc = s0.acc * (1.0f - a) + s1.acc * a;
  cv::Point3f gyr = s0.gyr * (1.0f - a) + s1.gyr * a;
  return ORB_SLAM3::IMU::Point(acc, gyr, t_target);
}

/// Convert ROS Imu message to ImuSample
static ImuSample to_imu_sample(const sensor_msgs::msg::Imu::SharedPtr &msg)
{
  double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  cv::Point3f acc(static_cast<float>(msg->linear_acceleration.x),
                  static_cast<float>(msg->linear_acceleration.y),
                  static_cast<float>(msg->linear_acceleration.z));
  cv::Point3f gyr(static_cast<float>(msg->angular_velocity.x),
                  static_cast<float>(msg->angular_velocity.y),
                  static_cast<float>(msg->angular_velocity.z));
  return {t, acc, gyr};
}

class ImuMonoRealSense : public rclcpp::Node {
public:
  ImuMonoRealSense()
    : Node("imu_mono_realsense"),
      vocabulary_file_path(resolve_default_vocabulary_path()),
      inertial_ba1_(false), inertial_ba2_(false)
  {

    // declare parameters
    declare_parameter("sensor_type", "imu-monocular");
    declare_parameter("use_pangolin", true);
    declare_parameter("image_topic", "camera/camera/color/image_raw");
    declare_parameter("imu_topic", "camera/camera/imu");
    declare_parameter("settings_file", "");
    declare_parameter("cam_imu_time_offset", 0.0);

    // get parameters
    sensor_type_param = get_parameter("sensor_type").as_string();
    use_pangolin = get_parameter("use_pangolin").as_bool();
    image_topic_ = get_parameter("image_topic").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();
    cam_imu_time_offset_ = get_parameter("cam_imu_time_offset").as_double();
    RCLCPP_INFO(get_logger(),
                "Camera-IMU time offset: %.6f s (t_imu = t_cam + offset)",
                cam_imu_time_offset_);

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

    rclcpp::QoS imu_qos(rclcpp::KeepLast(10));
    imu_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    imu_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    imu_sub = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, imu_qos,
      std::bind(&ImuMonoRealSense::imu_callback, this, _1), imu_options);

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
    if (!std::filesystem::create_directory(path + "/video")) {
      std::cout << "Failed to create images directory" << std::endl;
      return;
    }

    rclcpp::on_shutdown([this]() {
      video_writer_.release();
      pcl::io::savePCDFileBinary(std::string(PROJECT_PATH) + "/output/" +
                                   timestamp_ + "/cloud/" + timestamp_ + ".pcd",
                                 live_pcl_cloud_);
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
  }

private:
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

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    rclcpp::Time time_now = get_clock()->now();

    sensor_msgs::msg::Image::SharedPtr msg_out =
      std::make_shared<sensor_msgs::msg::Image>(*msg);
    msg_out->header.stamp = time_now;
    msg_out->header.frame_id = "base_link";
    orb_image_publisher_->publish(*msg_out);

    img_buf_.push(msg);

    // Process all queued images
    while (!img_buf_.empty()) {
      auto imgPtr = img_buf_.front();
      img_buf_.pop();

      cv::Mat imageFrame = get_image(imgPtr);
      double tImage =
        imgPtr->header.stamp.sec + imgPtr->header.stamp.nanosec * 1e-9;
      tImage += cam_imu_time_offset_;

      // ── Interpolated IMU extraction ──────────────────────────
      vector<ORB_SLAM3::IMU::Point> vImuMeas;

      buf_mutex_imu_.lock();

      // (A) Collect raw IMU samples up to tImage
      std::vector<ImuSample> raw_samples;
      if (has_last_imu_) {
        raw_samples.push_back(last_imu_before_image_);
      }

      ImuSample next_imu{};
      bool has_next_imu = false;

      while (!imu_buf_.empty()) {
        auto imuPtr = imu_buf_.front();
        ImuSample s = to_imu_sample(imuPtr);
        if (s.t > tImage) {
          next_imu = s;       // peek but do NOT pop
          has_next_imu = true;
          break;
        }
        imu_buf_.pop();
        raw_samples.push_back(s);
      }

      buf_mutex_imu_.unlock();

      // (B) Need at least 2 samples for interpolation
      if (raw_samples.size() < 2 && sensor_type_param == "imu-monocular") {
        if (!raw_samples.empty()) {
          last_imu_before_image_ = raw_samples.back();
          has_last_imu_ = true;
        }
        RCLCPP_DEBUG(get_logger(),
          "Waiting for IMU data: only %zu samples at t=%.4f",
          raw_samples.size(), tImage);
        return;
      }

      // (C) Build interpolated IMU sequence covering [prev_image_time_, tImage]

      // C1: Interpolated start point at prev_image_time_ (skip on first frame)
      size_t start_idx = 0;
      if (prev_image_time_ > 0 && raw_samples.size() >= 2) {
        for (size_t i = 0; i + 1 < raw_samples.size(); ++i) {
          if (raw_samples[i].t <= prev_image_time_ &&
              raw_samples[i + 1].t > prev_image_time_) {
            vImuMeas.push_back(
              interpolate_imu(raw_samples[i], raw_samples[i + 1],
                              prev_image_time_));
            start_idx = i + 1;  // skip samples before prev_image_time_
            break;
          }
        }
      }

      // C2: Add original IMU samples in (prev_image_time_, tImage)
      for (size_t i = start_idx; i < raw_samples.size(); ++i) {
        const auto &s = raw_samples[i];
        if (s.t > prev_image_time_ && s.t < tImage) {
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(s.acc, s.gyr, s.t));
        }
      }

      // C3: Interpolated end point at tImage
      if (!raw_samples.empty() && has_next_imu) {
        vImuMeas.push_back(
          interpolate_imu(raw_samples.back(), next_imu, tImage));
      } else if (raw_samples.size() >= 2) {
        vImuMeas.push_back(
          interpolate_imu(raw_samples[raw_samples.size() - 2],
                          raw_samples.back(), tImage));
      }

      // (D) Save state for next frame
      prev_image_time_ = tImage;
      if (!raw_samples.empty()) {
        last_imu_before_image_ = raw_samples.back();
        has_last_imu_ = true;
      }

      RCLCPP_DEBUG(get_logger(),
        "Frame t=%.4f: %zu IMU pts, span [%.4f, %.4f]",
        tImage, vImuMeas.size(),
        vImuMeas.empty() ? 0.0 : vImuMeas.front().t,
        vImuMeas.empty() ? 0.0 : vImuMeas.back().t);

      // ── SLAM tracking ────────────────────────────────────────
      if (vImuMeas.empty() && sensor_type_param == "imu-monocular") {
        return;
      }

      try {
        if (sensor_type_param == "monocular") {
          auto Tcw = orb_slam3_system_->TrackMonocular(imageFrame, tImage);
          Tcw_.translation() = Tcw.translation();
          Tcw_.setQuaternion(Tcw.unit_quaternion());
        } else {
          if (vImuMeas.size() > 1) {
            auto Tcw =
              orb_slam3_system_->TrackMonocular(imageFrame, tImage, vImuMeas);
            Tcw_.translation() = Tcw.translation();
            Tcw_.setQuaternion(Tcw.unit_quaternion());
          }
        }
        video_writer_.write(imageFrame);

      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "SLAM processing exception: %s", e.what());
      }
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu &msg)
  {
    buf_mutex_imu_.lock();
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
      imu_buf_.push(msg_ptr);
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid IMU data - nan");
    }
    buf_mutex_imu_.unlock();
  }

  void timer_callback()
  {
    unique_lock<mutex> lock(orbslam3_mutex_);

    geometry_msgs::msg::Pose pose;
    // Some custom forks expose isImuInitialized(); vanilla ORB_SLAM3 does not.
    // Keep publishing pose based on latest successful tracking result.
    if (true) {
      rclcpp::Time time_now = get_clock()->now();
      auto Twc = Tcw_.inverse();

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
    } else {
      // RCLCPP_INFO_STREAM(get_logger(), "IMU not initialized");
      initialize_variables();
    }
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

  // Interpolation sync state
  double prev_image_time_{-1.0};
  ImuSample last_imu_before_image_{};
  bool has_last_imu_{false};

  queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf_;
  queue<sensor_msgs::msg::Image::SharedPtr> img_buf_;
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
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuMonoRealSense>());
  rclcpp::shutdown();
  return 0;
}
