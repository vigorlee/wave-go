// hdl localizaton 
#include <mutex>
#include <memory>
#include <iostream>
#include <atomic>
#include <filesystem>
#include <deque>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>
#include <fast_gicp/ndt/ndt_cuda.hpp>

#include <localization/pose_estimator.hpp>
#include <localization/static_imu_init.hpp>
#include <localization/mode_state.h>
#include <localization/global_localization.hpp>

#include <localization/msg/scan_matching_status.hpp>
#include <robots_dog_msgs/srv/load_map.hpp>
#include <robots_dog_msgs/srv/localization_state.hpp>
#include <robots_dog_msgs/msg/localization.hpp>

using namespace std;

namespace localization {

class HdlLocalizationNode : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  HdlLocalizationNode(const rclcpp::NodeOptions& options) : Node("localization", options) {
    tf_buffer      = std::make_unique<tf2_ros::Buffer>(get_clock());
    // tf_listener    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    robot_odom_frame_id              = declare_parameter<std::string>("robot_odom_frame_id", "map");
    odom_child_frame_id              = declare_parameter<std::string>("odom_child_frame_id", "livox_frame");
    send_tf_transforms               = declare_parameter<bool>("send_tf_transforms", false);
    cool_time_duration               = declare_parameter<double>("cool_time_duration", 0.5);
    reg_method                       = declare_parameter<std::string>("reg_method", "NDT_OMP");
    ndt_neighbor_search_method       = declare_parameter<std::string>("ndt_neighbor_search_method", "DIRECT7");
    ndt_neighbor_search_radius       = declare_parameter<double>("ndt_neighbor_search_radius", 2.0);
    ndt_resolution                   = declare_parameter<double>("ndt_resolution", 1.0);
    enable_robot_odometry_prediction = declare_parameter<bool>("enable_robot_odometry_prediction", false);

    use_imu     = declare_parameter<bool>("use_imu", true);
    invert_acc  = declare_parameter<bool>("invert_acc", false);
    invert_gyro = declare_parameter<bool>("invert_gyro", false);
    // imu static init params
    imu_init_time_         = static_cast<float>(declare_parameter<double>("imu_init_time", 3.0));
    imu_init_queue_size_   = declare_parameter<int>("imu_init_queue_size", 600);
    imu_init_max_gyro_var_ = static_cast<float>(declare_parameter<double>("imu_init_max_gyro_var", 0.05));
    imu_init_max_acce_var_ = static_cast<float>(declare_parameter<double>("imu_init_max_acce_var", 0.2));

    std::string imu_topic               = declare_parameter<std::string>("imu_topic", "/livox/imu");
    std::string points_topic            = declare_parameter<std::string>("points_topic", "/livox/lidar");
    std::string odom_topic              = declare_parameter<std::string>("odom_topic", "/odom/localization_odom");
    std::string aligned_points_topic    = declare_parameter<std::string>("aligned_points_topic", "/aligned_points");
    std::string status_topic            = declare_parameter<std::string>("status_topic", "/status");
    std::string localization_info_topic = declare_parameter<std::string>("localization_info_topic", "/localization_info");
    std::string global_map_points_topic = declare_parameter<std::string>("global_map_points_topic", "/global_map_points");

    // Load numeric parameters
    imu_data_filter_num_      = declare_parameter<int>("imu_data_filter_num", 5);
    globalmap_voxel_size_     = static_cast<float>(declare_parameter<double>("globalmap_voxel_size", 0.3));
    points_voxel_filter_size_ = static_cast<float>(declare_parameter<double>("points_voxel_filter_size", 0.2));
    
    min_valid_count_           = declare_parameter<int>("min_valid_count", 5);
    buffer_size_               = declare_parameter<int>("buffer_size", 10);
    localization_odom_frame_id = declare_parameter<std::string>("localization_odom_frame_id", "base_link");

    // IMU rotation matrix parameters 
    std::vector<double> init_imu_R;
    init_imu_R.reserve(9);
    declare_parameter<std::vector<double>>("init_R", std::vector<double>{});
    get_parameter("init_R", init_imu_R);
    
    // Gravity transform matrix parameters 
    std::vector<double> init_T;
    init_T.reserve(16);
    declare_parameter<std::vector<double>>("init_T", std::vector<double>{});
    get_parameter("init_T", init_T);
    
    // Initial pose initialization parameters 
    declare_parameter<int>("init_match_count_threshold", 5);
    get_parameter("init_match_count_threshold", init_match_count_threshold_);
    declare_parameter<float>("init_match_score_threshold", 0.15);
    get_parameter("init_match_score_threshold", init_match_score_threshold_);
    
    // Global localization parameters
    declare_parameter<bool>("use_global_localization_init", true);
    get_parameter("use_global_localization_init", use_global_localization_init_);
    
    declare_parameter<float>("init_pose_change_threshold", 0.01);
    get_parameter("init_pose_change_threshold", init_pose_change_threshold_);
    
    declare_parameter<float>("init_quat_change_threshold", 0.01);
    get_parameter("init_quat_change_threshold", init_quat_change_threshold_);
    
    declare_parameter<float>("global_localization_timeout", 10.0);
    get_parameter("global_localization_timeout", global_localization_timeout_);

    static_imu_init_.SetParam(imu_init_time_, imu_init_queue_size_, imu_init_max_gyro_var_, imu_init_max_acce_var_);
    
    if (!init_imu_R.empty()) {
      init_rotation_matrix_ << init_imu_R[0], init_imu_R[1], init_imu_R[2], 
                               init_imu_R[3], init_imu_R[4], init_imu_R[5], 
                               init_imu_R[6], init_imu_R[7], init_imu_R[8];
      RCLCPP_INFO(get_logger(), "IMU rotation matrix loaded from config");
    }
    if (!init_T.empty()) {
      gravity_transform_ << init_T[0], init_T[1], init_T[2], init_T[3], 
                           init_T[4], init_T[5], init_T[6], init_T[7], 
                           init_T[8], init_T[9], init_T[10], init_T[11], 
                           init_T[12], init_T[13], init_T[14], init_T[15];
      RCLCPP_INFO(get_logger(), "Gravity transform matrix loaded from config");
    }
    // Log loaded parameters for verification
    RCLCPP_INFO(get_logger(),
                "Loaded parameters:\n"
                "  robot_odom_frame_id: %s\n"
                "  odom_child_frame_id: %s\n"
                "  localization_odom_frame_id: %s\n"
                "  use_imu: %s\n"
                "  invert_acc: %s\n"
                "  invert_gyro: %s\n"
                "  imu_topic: %s\n"
                "  points_topic: %s\n"
                "  odom_topic: %s\n"
                "  aligned_points_topic: %s\n"
                "  status_topic: %s\n"
                "  localization_info_topic: %s\n"
                "  global_map_points_topic: %s\n"
                "  send_tf_transforms: %s\n"
                "  enable_robot_odometry_prediction: %s\n"
                "  reg_method: %s\n"
                "  ndt_neighbor_search_method: %s\n"
                "  ndt_neighbor_search_radius: %.3f\n"
                "  ndt_resolution: %.3f\n"
                "  imu_data_filter_num: %d\n"
                "  globalmap_voxel_size: %.3f\n"
                "  points_voxel_filter_size: %.3f\n"
                "  min_valid_count: %d\n"
                "  buffer_size: %d\n"
                "  imu_init_time: %.1f\n"
                "  imu_init_queue_size: %d\n"
                "  imu_init_max_gyro_var: %.3f\n"
                "  imu_init_max_acce_var: %.3f",
                robot_odom_frame_id.c_str(),
                odom_child_frame_id.c_str(),
                localization_odom_frame_id.c_str(),
                use_imu ? "true" : "false",
                invert_acc ? "true" : "false",
                invert_gyro ? "true" : "false",
                imu_topic.c_str(),
                points_topic.c_str(),
                odom_topic.c_str(),
                aligned_points_topic.c_str(),
                status_topic.c_str(),
                localization_info_topic.c_str(),
                global_map_points_topic.c_str(),
                send_tf_transforms ? "true" : "false",
                enable_robot_odometry_prediction ? "true" : "false",
                reg_method.c_str(),
                ndt_neighbor_search_method.c_str(),
                ndt_neighbor_search_radius,
                ndt_resolution,
                imu_data_filter_num_,
                globalmap_voxel_size_,
                points_voxel_filter_size_,
                min_valid_count_,
                buffer_size_,
                imu_init_time_,
                imu_init_queue_size_,
                imu_init_max_gyro_var_,
                imu_init_max_acce_var_);

    global_map_points_ptr_.reset(new pcl::PointCloud<PointT>());
    if (use_imu) {
      RCLCPP_INFO(get_logger(), "enable imu-based prediction");
      correct_imu_data_ptr_ = std::make_shared<sensor_msgs::msg::Imu>();
      imu_sub               = create_subscription<sensor_msgs::msg::Imu>(imu_topic, 256, std::bind(&HdlLocalizationNode::imu_callback, this, std::placeholders::_1));
    }
    points_sub      = create_subscription<sensor_msgs::msg::PointCloud2>(points_topic, 5, std::bind(&HdlLocalizationNode::points_callback, this, std::placeholders::_1));
    initialpose_sub =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 8, std::bind(&HdlLocalizationNode::initialpose_callback, this, std::placeholders::_1));

    localization_lidar_info_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100), // 10Hz 
                std::bind(&HdlLocalizationNode::PublishLidarLocalizationInfo, this));
    odom_publish_timer_            = this->create_wall_timer(
                std::chrono::milliseconds(50), // 20Hz
                std::bind(&HdlLocalizationNode::PublishOdomTimer, this));
    pose_pub    = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 5);
    aligned_pub = create_publisher<sensor_msgs::msg::PointCloud2>(aligned_points_topic, 5);
    status_pub  = create_publisher<localization::msg::ScanMatchingStatus>(status_topic, 5);

    localization_state_srv_ = this->create_service<robots_dog_msgs::srv::LocalizationState>(
            "/localization_state/service",
            std::bind(&HdlLocalizationNode::LocalizationStateCallback, this, std::placeholders::_1, std::placeholders::_2));
    load_map_service_ptr_   = create_service<robots_dog_msgs::srv::LoadMap>(
            "/load_map_service", std::bind(&HdlLocalizationNode::LoadMapCallBack, this, std::placeholders::_1, std::placeholders::_2));

    localization_info_pub_ = this->create_publisher<robots_dog_msgs::msg::Localization>(localization_info_topic, 10);
    global_map_pub_        = this->create_publisher<sensor_msgs::msg::PointCloud2>(global_map_points_topic, 1);

    initialize_params();
    raw_points_ptr_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
    // Initialize sensor data validity tracking
    last_lidar_data_time_ = get_clock()->now();
    last_imu_data_time_   = get_clock()->now();
    // Initialize buffer, mark all as invalid initially
    for (int i = 0; i < buffer_size_; i++) {
      lidar_status_buffer_.push_back(false);
      imu_status_buffer_.push_back(false);
    }
    // Initialize pose history
    last_valid_pose_time_   = get_clock()->now();
    has_valid_pose_history_ = false;
    
    // Initialize confidence management
    last_confidence_update_time_ = get_clock()->now();
    current_confidence_          = 0.0;
    is_extrapolating_            = false;
    
    log_counter_  = 0;
    log_interval_ = 10;
    
    RCLCPP_INFO(get_logger(), "Sensor data validity tracking initialized with buffer size: %d, min valid count: %d", 
                buffer_size_, min_valid_count_);
  }

private:
  localization::StaticIMUInit static_imu_init_;
  float imu_init_time_ = 3.0f;
  int imu_init_queue_size_ = 300;
  float imu_init_max_gyro_var_ = 0.05f;
  float imu_init_max_acce_var_ = 0.2f;
  // Initial pose initialization parameters
  int init_match_count_threshold_ = 5;
  float init_match_score_threshold_ = 0.2f;
  // Initial pose initialization state variables
  int init_match_count_ = 0;
  pcl::Registration<PointT, PointT>::Ptr create_registration() {
    if (reg_method == "NDT_OMP") {
      RCLCPP_INFO(get_logger(), "NDT_OMP is selected");
      pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon(0.01);
      ndt->setResolution(ndt_resolution);
      if (ndt_neighbor_search_method == "DIRECT1") {
        RCLCPP_INFO(get_logger(), "search_method DIRECT1 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else if (ndt_neighbor_search_method == "DIRECT7") {
        RCLCPP_INFO(get_logger(), "search_method DIRECT7 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      } else {
        if (ndt_neighbor_search_method == "KDTREE") {
          RCLCPP_INFO(get_logger(), "search_method KDTREE is selected");
        } else {
          RCLCPP_WARN(get_logger(), "invalid search method was given");
          RCLCPP_WARN(get_logger(), "default method is selected (KDTREE)");
        }
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      }
      return ndt;
    }
    RCLCPP_ERROR_STREAM(get_logger(), "unknown registration method:" << reg_method);
    return nullptr;
  }

  void initialize_params() {
    voxel_filter_ptr_->setLeafSize(points_voxel_filter_size_, points_voxel_filter_size_, points_voxel_filter_size_);
    registration = create_registration();

    // Initialize global localization
    if (use_global_localization_init_) {
      try {
        global_localization_ptr_ = std::make_shared<GlobalLocalization>();
        RCLCPP_INFO(get_logger(), "Global localization initialized successfully");
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Failed to initialize global localization: %s", e.what());
        use_global_localization_init_ = false;
      }
    }
    // initialize pose estimator
    specify_init_pose_ = declare_parameter<bool>("specify_init_pose", true);
    if (specify_init_pose_) {
      RCLCPP_INFO(get_logger(), "initialize pose estimator with specified parameters!!"); 
      init_pos_x_ = declare_parameter<double>("init_pos_x", 0.0);
      init_pos_y_ = declare_parameter<double>("init_pos_y", 0.0);
      init_pos_z_ = declare_parameter<double>("init_pos_z", 0.0);
      init_ori_w_ = declare_parameter<double>("init_ori_w", 1.0);
      init_ori_x_ = declare_parameter<double>("init_ori_x", 0.0);
      init_ori_y_ = declare_parameter<double>("init_ori_y", 0.0);
      init_ori_z_ = declare_parameter<double>("init_ori_z", 0.0);
  
      Eigen::Vector3f config_pos(init_pos_x_, init_pos_y_, init_pos_z_);
      Eigen::Quaternionf config_quat(init_ori_w_, init_ori_x_, init_ori_y_, init_ori_z_);
      last_init_pos_ = config_pos;
      last_init_quat_ = config_quat;
      has_set_init_pose_ = true;
      last_pose_source_ = "config";
      pose_estimator.reset(new localization::PoseEstimator(
        registration,
        get_clock()->now(),
        last_init_pos_,
        last_init_quat_,
        cool_time_duration
      ));
      RCLCPP_INFO(get_logger(), "Initial pose estimator created with config pose - Position: [%.3f, %.3f, %.3f]",
                   last_init_pos_.x(), last_init_pos_.y(), last_init_pos_.z());
    }
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    correct_imu_data_ptr_ = imu_msg;
    Eigen::Vector3f acceleration(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
    // Apply rotation matrix and gravity compensation 
    acceleration = init_rotation_matrix_ * acceleration * 9.81;
    correct_imu_data_ptr_->linear_acceleration.x = acceleration.x();
    correct_imu_data_ptr_->linear_acceleration.y = acceleration.y();
    correct_imu_data_ptr_->linear_acceleration.z = acceleration.z();
    Eigen::Vector3f angular_velocity(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    // Apply rotation matrix to angular velocity
    angular_velocity = init_rotation_matrix_ * angular_velocity;
    correct_imu_data_ptr_->angular_velocity.x = angular_velocity.x();
    correct_imu_data_ptr_->angular_velocity.y = angular_velocity.y();
    correct_imu_data_ptr_->angular_velocity.z = angular_velocity.z();
    // Update latest IMU angular velocity for extrapolation
    latest_angular_velocity_ = angular_velocity;
    updateImuStatus(true);
    if (!static_imu_init_.InitSuccess()) {
      static_imu_init_.AddIMUData(correct_imu_data_ptr_);
    } else {
      std::lock_guard<std::mutex> lock(imu_data_mutex);
      imu_data.push_back(correct_imu_data_ptr_);
    }
  }

  void points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr points_msg) {
    auto start = std::chrono::high_resolution_clock::now();
    updateLidarStatus(true);
    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex); 
    if (use_imu && !static_imu_init_.InitSuccess()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5.0, "Radar CallBack Waiting for IMU Initial !!!");
      publishExtrapolatedOdom(points_msg->header.stamp);
      return;
    }
    if (!pose_estimator) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5.0, "Radar CallBack Waiting for Initial Pose Input!!");
      pubDefaultLocalizationOdom(points_msg->header.stamp);
      return;
    }
    if (!global_map_points_ptr_ || global_map_points_ptr_->empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5.0, "Radar CallBack Waiting for Globalmap Input!!");
      pubDefaultLocalizationOdom(points_msg->header.stamp);
      return;
    }
    if (!isSensorDataValid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0, "Sensor data invalid, using extrapolation!");
      publishExtrapolatedOdom(points_msg->header.stamp);
      return;
    }
  
    const auto& stamp = points_msg->header.stamp;
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
    raw_points_ptr_->clear();
    pcl::fromROSMsg(*points_msg, *raw_points_ptr_);
                      
    if (raw_points_ptr_->empty()) {
      RCLCPP_ERROR(get_logger(), "cloud is empty!!");
      return;
    }

    auto filtered = downsample(raw_points_ptr_);
    TransformPoints(filtered, raw_points_ptr_);
    // last_scan = filtered;
    
    if (use_global_localization_init_ && gl_once_gate_ && global_localization_ptr_ && !is_init_success_) {
      RCLCPP_INFO(get_logger(), "Attempting global localization for better initial pose...");
      if (performGlobalLocalization(raw_points_ptr_)) {
        RCLCPP_INFO(get_logger(), "Global localization successful! Using new initial pose.");
        pose_estimator.reset(new localization::PoseEstimator(
          registration, get_clock()->now(), last_init_pos_, last_init_quat_, cool_time_duration));
        is_init_success_ = false;
        init_match_count_ = 0;
        localization_state_ = 1;
        gl_once_gate_ = false;
        RCLCPP_INFO(get_logger(), "Pose estimator recreated with global localization result");
      } else {
        RCLCPP_INFO(get_logger(), "Global localization failed, continuing with current pose.");
      }
    }
    
    if (!is_init_success_) {
      localization_state_ = 1;
      PoseEstimator::MatchResult init_result = pose_estimator->GetMatchState();
      RCLCPP_INFO(get_logger(), "init_result.is_converged_ : %d", init_result.is_converged_);
      RCLCPP_INFO(get_logger(), "init_result.fitness_score_ : %f", init_result.fitness_score_);
      // Check if current frame meets initialization criteria
      if (init_result.is_converged_ && init_result.fitness_score_ < init_match_score_threshold_) {
        init_match_count_++;
        RCLCPP_INFO(get_logger(), "Init match count: %d/%d (score: %.6f)", init_match_count_, init_match_count_threshold_, init_result.fitness_score_);
        // Check if we have enough consecutive successful matches
        if (init_match_count_ >= init_match_count_threshold_) {
          is_init_success_ = true;
          localization_state_ = 2;
          RCLCPP_INFO(get_logger(), "Init Pose Successful!!!");
          init_match_count_ = 0;  // Reset counter
        }
      } else {
        init_match_count_ = 0;
        RCLCPP_INFO(get_logger(), "Init match criteria not met, resetting counter");
      }
      RCLCPP_INFO(get_logger(), "Wait Init Pose!!! Current count: %d/%d", init_match_count_, init_match_count_threshold_);
    }

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());  
    // predict
    if (!use_imu) {
      pose_estimator->predict(stamp);
    } else {
      std::lock_guard<std::mutex> lock(imu_data_mutex);
      auto imu_iter = imu_data.begin();
      int num = 0;
      for (imu_iter; imu_iter != imu_data.end(); imu_iter++) {
        if (rclcpp::Time(stamp) < rclcpp::Time((*imu_iter)->header.stamp)) {
          break;
        }
        num++;
        if (!(num % imu_data_filter_num_)) {
          const auto& acc = (*imu_iter)->linear_acceleration;
          const auto& gyro = (*imu_iter)->angular_velocity;
          double acc_sign = invert_acc ? -1.0 : 1.0;
          double gyro_sign = invert_gyro ? -1.0 : 1.0;
          pose_estimator->predict((*imu_iter)->header.stamp, acc_sign * Eigen::Vector3f(acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
          publish_odometry((*imu_iter)->header.stamp, pose_estimator->matrix());
        }
      }
      imu_data.erase(imu_data.begin(), imu_iter);
    }

    // odometry-based prediction
    rclcpp::Time last_correction_time = pose_estimator->last_correction_time();
    if (enable_robot_odometry_prediction && last_correction_time != rclcpp::Time((int64_t)0, get_clock()->get_clock_type())) {
      geometry_msgs::msg::TransformStamped odom_delta;
      if (tf_buffer->canTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp, robot_odom_frame_id, rclcpp::Duration(std::chrono::milliseconds(100)))) {
        odom_delta =
          tf_buffer->lookupTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp, robot_odom_frame_id, rclcpp::Duration(std::chrono::milliseconds(0)));
      } else if (tf_buffer->canTransform(
                   odom_child_frame_id,
                   last_correction_time,
                   odom_child_frame_id,
                   rclcpp::Time((int64_t)0, get_clock()->get_clock_type()),
                   robot_odom_frame_id,
                   rclcpp::Duration(std::chrono::milliseconds(0)))) {
        odom_delta = tf_buffer->lookupTransform(
          odom_child_frame_id,
          last_correction_time,
          odom_child_frame_id,
          rclcpp::Time((int64_t)0, get_clock()->get_clock_type()),
          robot_odom_frame_id,
          rclcpp::Duration(std::chrono::milliseconds(0)));
      }
      if (odom_delta.header.stamp == rclcpp::Time((int64_t)0, get_clock()->get_clock_type())) {
        RCLCPP_WARN_STREAM(get_logger(), "failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id);
      } else {
        Eigen::Isometry3d delta = tf2::transformToEigen(odom_delta);
        pose_estimator->predict_odom(delta.cast<float>().matrix());
      }
    }
    // correct
    auto aligned = pose_estimator->correct(stamp, raw_points_ptr_);

    PoseEstimator::MatchResult match_result = pose_estimator->GetMatchState();
    if (match_result.is_converged_ && match_result.fitness_score_ < 0.5) {
      localization_state_ = 3;
      RCLCPP_INFO(get_logger(), "Continuous Localization Successful!!!");
    } else {
      localization_state_ = 4;
      RCLCPP_INFO(get_logger(), "Continuous Localization may not good!!!");
    }

    auto end = std::chrono::high_resolution_clock::now();
    last_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    if (last_timeout_ > 80) {
      RCLCPP_INFO(get_logger(), "!!!point cloud callback time cost > 80ms, = %d ms\n", last_timeout_);
    }

    if (aligned_pub->get_subscription_count()) {
      aligned->header.frame_id = "map";
      aligned->header.stamp = cloud->header.stamp;
      sensor_msgs::msg::PointCloud2 aligned_msg;
      pcl::toROSMsg(*aligned, aligned_msg);
      aligned_pub->publish(aligned_msg);
    }
    // Update pose history for extrapolation
    if (match_result.is_converged_ && match_result.fitness_score_ < 0.5) {
      Eigen::Matrix4f current_pose = pose_estimator->matrix();
      Eigen::Vector3f current_velocity = getCurrentVelocity(current_pose, points_msg->header.stamp);
      Eigen::Vector3f current_angular_velocity = getCurrentAngularVelocity(current_pose, points_msg->header.stamp);
      updatePoseHistory(current_pose, current_velocity, current_angular_velocity, points_msg->header.stamp);
      is_extrapolating_ = false;
      current_confidence_ = 1.0;
      last_confidence_update_time_ = points_msg->header.stamp;
    }
    publish_odometry(points_msg->header.stamp, pose_estimator->matrix());
  }

  /**
   * @brief callback for initial pose input 
   * @param pose_msg
   */
  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg) {
    RCLCPP_INFO(get_logger(), "initial pose received!!");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
  
    const auto& p = pose_msg->pose.pose.position;
    const auto& q = pose_msg->pose.pose.orientation;
    Eigen::Vector3f new_pos(p.x, p.y, p.z);
    Eigen::Quaternionf new_quat(q.w, q.x, q.y, q.z);
    
    bool pose_changed = false;
    if (!has_set_init_pose_) {
      pose_changed = true;
    } else {
      float pos_change = (new_pos - last_init_pos_).norm();
      float quat_change = std::abs(1.0f - std::abs(last_init_quat_.dot(new_quat))); 
      if (pos_change > init_pose_change_threshold_ || quat_change > init_quat_change_threshold_) {
        pose_changed = true;
        RCLCPP_INFO(get_logger(), "Pose change detected - Position: %.3f m, Orientation: %.3f", pos_change, quat_change);
      }
    }
    
    if (pose_changed) {
      last_init_pos_ = new_pos;
      last_init_quat_ = new_quat;
      has_set_init_pose_ = true;
      last_pose_source_ = "Callback";   
      pose_estimator.reset(new localization::PoseEstimator(
        registration, get_clock()->now(), last_init_pos_, last_init_quat_, cool_time_duration));
      // restart init verification and disable GL for this round
      is_init_success_ = false;
      init_match_count_ = 0;
      localization_state_ = 1;
      gl_once_gate_ = false;
      RCLCPP_INFO(get_logger(), "New initial pose set from RViz - Position: [%.3f, %.3f, %.3f], Quaternion: [%.3f, %.3f, %.3f, %.3f]",
                   last_init_pos_.x(), last_init_pos_.y(), last_init_pos_.z(), last_init_quat_.w(), last_init_quat_.x(), last_init_quat_.y(), last_init_quat_.z());
      RCLCPP_INFO(get_logger(), "Localization will restart with new pose");
    } else {
      RCLCPP_INFO(get_logger(), "Pose unchanged, no action needed");
    }
  }

  pcl::PointCloud<PointT>::Ptr downsample(const pcl::PointCloud<PointT>::Ptr& cloud) const {
    if (!voxel_filter_ptr_) {
      return cloud;
    }
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxel_filter_ptr_->setInputCloud(cloud);
    voxel_filter_ptr_->filter(*filtered);
    filtered->header = cloud->header;
    return filtered;
  }

  void publish_odometry(const rclcpp::Time& stamp, const Eigen::Matrix4f& pose) {
    RCLCPP_INFO(
      get_logger(),
      "[publish_odometry] stamp_ns=%ld now_ns=%ld send_tf_transforms=%s frame_id=%s child_frame_id=%s pose_xyz=[%.3f, %.3f, %.3f]",
      static_cast<long>(stamp.nanoseconds()),
      static_cast<long>(get_clock()->now().nanoseconds()),
      send_tf_transforms ? "true" : "false",
      robot_odom_frame_id.c_str(),
      localization_odom_frame_id.c_str(),
      pose(0, 3), pose(1, 3), pose(2, 3));
    if (send_tf_transforms) {
      if (tf_buffer->canTransform(robot_odom_frame_id, odom_child_frame_id, rclcpp::Time((int64_t)0, get_clock()->get_clock_type()))) {
        RCLCPP_INFO(
          get_logger(),
          "[publish_odometry] canTransform(%s <- %s) = true",
          robot_odom_frame_id.c_str(), odom_child_frame_id.c_str());
        geometry_msgs::msg::TransformStamped map_wrt_frame = tf2::eigenToTransform(Eigen::Isometry3d(pose.inverse().cast<double>()));
        map_wrt_frame.header.stamp = stamp;
        map_wrt_frame.header.frame_id = odom_child_frame_id;
        map_wrt_frame.child_frame_id = "map";

        geometry_msgs::msg::TransformStamped frame_wrt_odom = tf_buffer->lookupTransform(
          robot_odom_frame_id,
          odom_child_frame_id,
          rclcpp::Time((int64_t)0, get_clock()->get_clock_type()),
          rclcpp::Duration(std::chrono::milliseconds(100)));
        Eigen::Matrix4f frame2odom = tf2::transformToEigen(frame_wrt_odom).cast<float>().matrix();

        geometry_msgs::msg::TransformStamped map_wrt_odom;
        tf2::doTransform(map_wrt_frame, map_wrt_odom, frame_wrt_odom);

        tf2::Transform odom_wrt_map;
        tf2::fromMsg(map_wrt_odom.transform, odom_wrt_map);
        odom_wrt_map = odom_wrt_map.inverse();

        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.transform = tf2::toMsg(odom_wrt_map);
        odom_trans.header.stamp = stamp;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = robot_odom_frame_id;

        tf_broadcaster->sendTransform(odom_trans);
        RCLCPP_INFO(
          get_logger(),
          "[publish_odometry] broadcast TF map -> %s at stamp_ns=%ld",
          robot_odom_frame_id.c_str(), static_cast<long>(stamp.nanoseconds()));
      } else {
        RCLCPP_WARN(
          get_logger(),
          "[publish_odometry] canTransform(%s <- %s) = false, fallback broadcast map -> %s directly",
          robot_odom_frame_id.c_str(), odom_child_frame_id.c_str(), odom_child_frame_id.c_str());
        geometry_msgs::msg::TransformStamped odom_trans = tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>()));
        odom_trans.header.stamp = stamp;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = odom_child_frame_id;
        tf_broadcaster->sendTransform(odom_trans);
        RCLCPP_INFO(
          get_logger(),
          "[publish_odometry] broadcast fallback TF map -> %s at stamp_ns=%ld",
          odom_child_frame_id.c_str(), static_cast<long>(stamp.nanoseconds()));
      }
    }
    // publish the transform
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.pose.pose = tf2::toMsg(Eigen::Isometry3d(pose.cast<double>()));
    odom.child_frame_id = localization_odom_frame_id;
    // Use confidence value directly, 1.0 means completely reliable, 0.0 means completely unreliable
    double confidence = getCurrentConfidence();
    odom.pose.covariance[0] = confidence;  // Use confidence value directly
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;
    pose_pub->publish(odom);
    RCLCPP_INFO(
      get_logger(),
      "[publish_odometry] published odom header_ns=%ld frame_id=%s child_frame_id=%s",
      static_cast<long>(static_cast<long long>(odom.header.stamp.sec) * 1000000000LL + odom.header.stamp.nanosec),
      odom.header.frame_id.c_str(),
      odom.child_frame_id.c_str());
  }

  /**
   * @brief Publish default localization information (position is 0)
   * @param stamp timestamp
   */
  void pubDefaultLocalizationOdom(const rclcpp::Time& stamp) {
    is_extrapolating_ = false;
    current_confidence_ = 0.0;
    Eigen::Matrix4f default_pose = Eigen::Matrix4f::Identity();
    publish_odometry(stamp, default_pose);
  }

  /**
   * @brief perform global localization
   * @param current_cloud current point cloud
   * @return true if global localization is successful
   */
  bool performGlobalLocalization(const pcl::PointCloud<PointT>::Ptr& current_cloud) {
    if (!global_localization_ptr_ || !global_map_points_ptr_) {
      RCLCPP_WARN(get_logger(), "Global localization not available");
      return false;
    }
    global_localization_in_progress_ = true;
    global_localization_start_time_ = get_clock()->now();  
    RCLCPP_INFO(get_logger(), "Starting global localization..."); 
    try {
      Eigen::Vector3f init_pos;
      Eigen::Quaternionf init_quat;
      if (!getCurrentInitPose(init_pos, init_quat)) {
        RCLCPP_WARN(get_logger(), "Failed to get initial pose for global localization");
        global_localization_in_progress_ = false;
        return false;
      }
      
      Eigen::Matrix4d initial_trans = Eigen::Matrix4d::Identity();
      initial_trans.block<3, 1>(0, 3) = init_pos.cast<double>();
      initial_trans.block<3, 3>(0, 0) = init_quat.toRotationMatrix().cast<double>();
      
      Eigen::Matrix4d final_pose;
      bool localization_success = global_localization_ptr_->performGlobalLocalization(
        global_map_points_ptr_, current_cloud, initial_trans, final_pose);
      
      if (localization_success) {
        Eigen::Vector3f new_pos = final_pose.block<3, 1>(0, 3).cast<float>();
        Eigen::Quaternionf new_quat(final_pose.block<3, 3>(0, 0).cast<float>());
        if (pose_estimator) {
          last_init_pos_ = new_pos;
          last_init_quat_ = new_quat;
          has_set_init_pose_ = true;
          last_pose_source_ = "global_localization";
          RCLCPP_INFO(get_logger(), "Global localization successful! New pose: pos[%.3f, %.3f, %.3f], quat[%.3f, %.3f, %.3f, %.3f]",
                      new_pos.x(), new_pos.y(), new_pos.z(), new_quat.w(), new_quat.x(), new_quat.y(), new_quat.z());
          global_localization_in_progress_ = false;
          return true;
        }
      } else {
        RCLCPP_WARN(get_logger(), "Global localization failed");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Global localization exception: %s", e.what());
    }
    global_localization_in_progress_ = false;
    return false;
  }
  
  /**
   * @brief get current initial pose
   * @param pos output position
   * @param quat output orientation
   * @return true if success
   */
  bool getCurrentInitPose(Eigen::Vector3f& pos, Eigen::Quaternionf& quat) {
    if (has_set_init_pose_) {
      pos = last_init_pos_;
      quat = last_init_quat_;
      return true;
    }
    if (specify_init_pose_) {
      pos = Eigen::Vector3f(init_pos_x_, init_pos_y_, init_pos_z_);
      quat = Eigen::Quaternionf(init_ori_w_, init_ori_x_, init_ori_y_, init_ori_z_);
      return true;
    }
    pos = Eigen::Vector3f::Zero();
    quat = Eigen::Quaternionf::Identity();
    return true;
  }
  
  /**
   * @brief Check if lidar data is valid
   * @return true: lidar data is valid, false: lidar data is invalid
   */
  bool isLidarDataValid() {
    if (lidar_status_buffer_.size() < min_valid_count_) { return false; }
    int valid_count = std::count(lidar_status_buffer_.begin(), lidar_status_buffer_.end(), true);
    return valid_count >= min_valid_count_;
  }

  /**
   * @brief Check if IMU data is valid
   * @return true: IMU data is valid, false: IMU data is invalid
   */
  bool isImuDataValid() {
    if (!use_imu) { return false; }
    if (imu_status_buffer_.size() < min_valid_count_) { return false; }
    int valid_count = std::count(imu_status_buffer_.begin(), imu_status_buffer_.end(), true);
    return valid_count >= min_valid_count_;
  }

  /**
   * @brief Check if sensor data is valid (at least one of lidar or IMU is valid)
   * @return true: at least one sensor data is valid, false: all sensor data are invalid
   */
  bool isSensorDataValid() { return isLidarDataValid() || isImuDataValid(); }

  /**
   * @brief Update lidar data status
   * @param is_valid whether data is valid
   */
  void updateLidarStatus(bool is_valid) {
    lidar_status_buffer_.push_back(is_valid);
    if (lidar_status_buffer_.size() > buffer_size_) {
      lidar_status_buffer_.pop_front();
    }
    if (is_valid) {
      last_lidar_data_time_ = get_clock()->now();
    }
  }

  /**
   * @brief Update IMU data status
   * @param is_valid whether data is valid
   */
  void updateImuStatus(bool is_valid) {
    imu_status_buffer_.push_back(is_valid);
    if (imu_status_buffer_.size() > buffer_size_) {
      imu_status_buffer_.pop_front();
    }
    if (is_valid) {
      last_imu_data_time_ = get_clock()->now();
    }
  }
  
  // Transform points using gravity transformation matrix 
  void TransformPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out) {
      cloud_out->clear();
      int point_size = cloud_in->points.size();
      cloud_out->resize(point_size);
  #pragma omp parallel for
      for (int i = 0; i < cloud_in->points.size(); ++i) {
          pcl::PointXYZI  point;
          Eigen::Vector4f p_start(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z, 1.0);
          Eigen::Vector4f p_result(gravity_transform_ * p_start);
          point.x              = p_result(0);
          point.y              = p_result(1);
          point.z              = p_result(2);
          point.intensity      = cloud_in->points[i].intensity;
          cloud_out->points[i] = point;
      }
  }

  /**
   * @brief Update pose history for extrapolation
   * @param pose current pose matrix
   * @param velocity current velocity
   * @param angular_velocity current angular velocity
   * @param current_time current time
   */
  void updatePoseHistory(const Eigen::Matrix4f& pose, const Eigen::Vector3f& velocity, 
                         const Eigen::Vector3f& angular_velocity, const rclcpp::Time& current_time) {
    last_pose_ = pose;
    last_velocity_ = velocity;
    last_angular_velocity_ = angular_velocity;
    last_valid_pose_time_ = current_time;
    has_valid_pose_history_ = true;
  }

  /**
   * @brief Get current velocity (prioritize UKF state, extrapolation as backup)
   * @param current_pose current pose
   * @param current_time current time
   * @return current velocity
   */
  Eigen::Vector3f getCurrentVelocity(const Eigen::Matrix4f& current_pose, const rclcpp::Time& current_time) {
    if (pose_estimator) {
      return pose_estimator->vel();
    }
    if (has_valid_pose_history_) {
      double dt = (current_time - last_valid_pose_time_).seconds();
      if (dt > 0.0) {
        Eigen::Vector3f current_position = current_pose.block<3, 1>(0, 3);
        Eigen::Vector3f last_position = last_pose_.block<3, 1>(0, 3);
        Eigen::Vector3f position_delta = current_position - last_position;
        return position_delta / dt;
      }
    }
    return last_velocity_;
  }

  /**
   * @brief Get current angular velocity (prioritize IMU data, extrapolation as backup)
   * @param current_pose current pose
   * @param current_time current time
   * @return current angular velocity
   */
  Eigen::Vector3f getCurrentAngularVelocity(const Eigen::Matrix4f& current_pose, const rclcpp::Time& current_time) {
    if (latest_angular_velocity_.norm() > 0.0) {
      return latest_angular_velocity_;
    }
    if (has_valid_pose_history_) {
      double dt = (current_time - last_valid_pose_time_).seconds();
      if (dt > 0.0) {
        Eigen::Matrix3f current_rotation = current_pose.block<3, 3>(0, 0);
        Eigen::Matrix3f last_rotation = last_pose_.block<3, 3>(0, 0);
        Eigen::Matrix3f relative_rotation = current_rotation * last_rotation.transpose();
        Eigen::AngleAxisf angle_axis(relative_rotation);
        Eigen::Vector3f angular_velocity = angle_axis.axis() * angle_axis.angle() / dt;
        return angular_velocity;
      }
    }
    return last_angular_velocity_;
  }

  /**
   * @brief Get sensor status statistics information
   * @return string containing sensor status statistics information
   */
  std::string getSensorStatusInfo() const {
    std::stringstream ss;
    // Lidar status statistics
    int lidar_valid_count = std::count(lidar_status_buffer_.begin(), lidar_status_buffer_.end(), true);
    int lidar_total_count = lidar_status_buffer_.size();
    double lidar_valid_ratio = lidar_total_count > 0 ? (double)lidar_valid_count / lidar_total_count : 0.0; 
    // IMU status statistics
    int imu_valid_count = std::count(imu_status_buffer_.begin(), imu_status_buffer_.end(), true);
    int imu_total_count = imu_status_buffer_.size();
    double imu_valid_ratio = imu_total_count > 0 ? (double)imu_valid_count / imu_total_count : 0.0;
    
    ss << "Lidar: " << lidar_valid_count << "/" << lidar_total_count 
       << " (" << std::fixed << std::setprecision(1) << (lidar_valid_ratio * 100.0) << "%)"
       << " | IMU: " << imu_valid_count << "/" << imu_total_count 
       << " (" << std::fixed << std::setprecision(1) << (imu_valid_ratio * 100.0) << "%)"
       << " | Pose History: " << (has_valid_pose_history_ ? "Available" : "Not Available")
       << " | Confidence: " << std::fixed << std::setprecision(2) << current_confidence_; 
    return ss.str();
  }

  /**
   * @brief Update confidence
   * @param current_time current time
   */
  void updateConfidence(const rclcpp::Time& current_time) {
    if (!is_init_success_) {
      current_confidence_ = 0.0;
    } else if (is_extrapolating_) {
      // When extrapolating, confidence decays exponentially
      double dt = (current_time - last_confidence_update_time_).seconds();
      if (dt > 0.0) {
        current_confidence_ *= std::exp(-confidence_decay_rate_ * dt);
        if (current_confidence_ < 0.01) {
          current_confidence_ = 0.01;
        }
      }
    } else {
      current_confidence_ = 1.0;
    } 
    last_confidence_update_time_ = current_time;
  }

  /**
   * @brief Get current confidence
   * @return current confidence value
   */
  double getCurrentConfidence() const { return current_confidence_; }

  /**
   * @brief Control log printing frequency
   * @param message message to print
   * @param force_print whether to force print (ignore counter)
   */
  void controlledLogInfo(const std::string& message, bool force_print = false) {
    log_counter_++;
    if (force_print || log_counter_ % log_interval_ == 0) {
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
      log_counter_ = 0;  // Reset counter
    }
  }

  /**
   * @brief Convert quaternion to normalized Euler angles (ZYX order, consistent with Eigen)
   * @param quat quaternion (w, x, y, z)
   * @return normalized Euler angles [roll, pitch, yaw] (ZYX order)
   */
  Eigen::Vector3f quaternionToNormalizedRPY(const Eigen::Quaternionf& quat) {
    Eigen::Quaternionf normalized_quat = quat.normalized();
    float w = normalized_quat.w();
    float x = normalized_quat.x();
    float y = normalized_quat.y();
    float z = normalized_quat.z();
    // 0=Z-axis(yaw), 1=Y-axis(pitch), 2=X-axis(roll)
    float roll, pitch, yaw;
    // Calculate Roll 
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    // Calculate Pitch 
    float sinp = 2.0f * (w * y - x * z);
    if (std::abs(sinp) >= 1.0f) {
      // Handle gimbal lock case (pitch = ±90°)
      pitch = std::copysign(M_PI / 2.0f, sinp);
      roll = 0.0f;
      yaw = 2.0f * std::atan2(x, w);
    } else {
      pitch = std::asin(sinp);
      // Calculate Yaw 
      float siny_cosp = 2.0f * (w * z + x * y);
      float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
      yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    if (roll > M_PI / 2.0f) {
      roll -= M_PI;
      pitch = M_PI - pitch;
      yaw += M_PI;
    } else if (roll < -M_PI / 2.0f) {
      roll += M_PI;
      pitch = -M_PI - pitch;
      yaw += M_PI;
    }
    if (pitch > M_PI / 2.0f) {
      pitch = M_PI - pitch;
      roll += M_PI;
      yaw += M_PI;
    } else if (pitch < -M_PI / 2.0f) {
      pitch = -M_PI - pitch;
      roll += M_PI;
      yaw += M_PI;
    }
    if (yaw > M_PI) {
      yaw -= 2.0f * M_PI;
    } else if (yaw < -M_PI) {
      yaw += 2.0f * M_PI;
    }
    return Eigen::Vector3f(roll, pitch, yaw);
  }

  /**
   * @brief Extrapolate pose based on previous state
   * @param current_time current time
   * @return extrapolated pose matrix
   */
  Eigen::Matrix4f extrapolatePose(const rclcpp::Time& current_time) {
    if (!has_valid_pose_history_) {
      return Eigen::Matrix4f::Identity();
    }
    // Use fixed time step 0.05 seconds (20Hz)
    const double dt = 0.05;
    const double max_velocity = 1.0;        // Maximum velocity
    const double max_angular_velocity = 0.5; // Maximum angular velocity
    Eigen::Vector3f limited_velocity = last_velocity_;
    double velocity_norm = limited_velocity.norm();
    if (velocity_norm > max_velocity) {
      limited_velocity = limited_velocity.normalized() * max_velocity;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0, 
                           "Velocity limited from %.2f to %.2f m/s", velocity_norm, max_velocity);
    }
    
    // Limit angular velocity range
    Eigen::Vector3f limited_angular_velocity = last_angular_velocity_;
    double angular_velocity_norm = limited_angular_velocity.norm();
    if (angular_velocity_norm > max_angular_velocity) {
      limited_angular_velocity = limited_angular_velocity.normalized() * max_angular_velocity;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0, 
                           "Angular velocity limited from %.2f to %.2f rad/s", angular_velocity_norm, max_angular_velocity);
    }
    Eigen::Vector3f position_delta = limited_velocity * dt;
    Eigen::Vector3f last_position = last_pose_.block<3, 1>(0, 3);
    Eigen::Matrix3f last_rotation = last_pose_.block<3, 3>(0, 0);
    Eigen::Vector3f new_position = last_position + position_delta;
    Eigen::Vector3f angle_delta = limited_angular_velocity * dt;
    Eigen::Matrix3f delta_rotation = Eigen::Matrix3f::Identity();
    delta_rotation = Eigen::AngleAxisf(angle_delta.z(), Eigen::Vector3f::UnitZ()) *
                     Eigen::AngleAxisf(angle_delta.y(), Eigen::Vector3f::UnitY()) *
                     Eigen::AngleAxisf(angle_delta.x(), Eigen::Vector3f::UnitX());
    Eigen::Matrix3f new_rotation = last_rotation * delta_rotation;
    Eigen::Matrix4f extrapolated_pose = Eigen::Matrix4f::Identity();
    extrapolated_pose.block<3, 3>(0, 0) = new_rotation;
    extrapolated_pose.block<3, 1>(0, 3) = new_position;
    
    // Debug information
    RCLCPP_DEBUG(get_logger(), 
                 "Extrapolation: dt=%.3fs, vel=[%.2f,%.2f,%.2f], pos_delta=[%.2f,%.2f,%.2f], "
                 "new_pos=[%.2f,%.2f,%.2f]", 
                 dt, 
                 limited_velocity.x(), limited_velocity.y(), limited_velocity.z(),
                 position_delta.x(), position_delta.y(), position_delta.z(),
                 new_position.x(), new_position.y(), new_position.z());
    
    return extrapolated_pose;
  }

  /**
   * @brief Publish extrapolated localization information
   * @param stamp timestamp
   */
  void publishExtrapolatedOdom(const rclcpp::Time& stamp) {
    if (!has_valid_pose_history_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0, 
                           "No pose history available, publishing default pose");
      pubDefaultLocalizationOdom(stamp);
      return;
    }
    // Set extrapolation state to true, start confidence decay
    is_extrapolating_ = true;
    Eigen::Matrix4f extrapolated_pose = extrapolatePose(stamp);
    publish_odometry(stamp, extrapolated_pose);
    last_pose_ = extrapolated_pose;
    last_valid_pose_time_ = stamp; 
    double extrapolation_time = (stamp - last_valid_pose_time_).seconds();  
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1.0, 
                         "Published extrapolated pose and updated history (extrapolation time: %.2fs, "
                         "last velocity: [%.2f, %.2f, %.2f], "
                         "last angular velocity: [%.2f, %.2f, %.2f], "
                         "confidence: %.3f)", 
                         extrapolation_time,
                         last_velocity_.x(), last_velocity_.y(), last_velocity_.z(),
                         last_angular_velocity_.x(), last_angular_velocity_.y(), last_angular_velocity_.z(),
                         getCurrentConfidence());
  }

  void publish_scan_matching_status(const std_msgs::msg::Header& header, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned) {
    localization::msg::ScanMatchingStatus status;
    status.header = header;
    status.has_converged = registration->hasConverged();
    status.matching_error = registration->getFitnessScore();
    const double max_correspondence_dist = 0.5;

    int num_inliers = 0;
    std::vector<int> k_indices;
    std::vector<float> k_sq_dists;
    for (int i = 0; i < aligned->size(); i++) {
      const auto& pt = aligned->at(i);
      registration->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
      if (k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist) {
        num_inliers++;
      }
    }
    status.inlier_fraction = static_cast<float>(num_inliers) / aligned->size();
    status.relative_pose = tf2::eigenToTransform(Eigen::Isometry3d(registration->getFinalTransformation().cast<double>())).transform;
    status.prediction_labels.reserve(2);
    status.prediction_errors.reserve(2);
    std::vector<double> errors(6, 0.0);
    if (pose_estimator->wo_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::msg::String());
      status.prediction_labels.back().data = "without_pred";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->wo_prediction_error().get().cast<double>())).transform);
    }
    if (pose_estimator->imu_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::msg::String());
      status.prediction_labels.back().data = use_imu ? "imu" : "motion_model";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->imu_prediction_error().get().cast<double>())).transform);
    }
    if (pose_estimator->odom_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::msg::String());
      status.prediction_labels.back().data = "odom";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->odom_prediction_error().get().cast<double>())).transform);
    }
    status_pub->publish(status);
  }

  void PublishLidarLocalizationInfo() {
        auto current_time = this->get_clock()->now();
        updateConfidence(current_time);
        if (lidar_status_buffer_.size() > 0) {
            double lidar_time_diff = (current_time - last_lidar_data_time_).seconds();
            if (lidar_time_diff > sensor_timeout_threshold_) {
                updateLidarStatus(false);
            }
        }  
        if (imu_status_buffer_.size() > 0) {
            double imu_time_diff = (current_time - last_imu_data_time_).seconds();
            if (imu_time_diff > sensor_timeout_threshold_) {
                updateImuStatus(false);
            }
        }
        robots_dog_msgs::msg::Localization msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";
        msg.type = "loc_state";

        if (!is_init_success_) {
            msg.status = 0;
            msg.coord_type = is_use_map_coord_ ? 0 : 1;
            msg.pos.x = msg.pos.y = msg.pos.z = 0.0;
            msg.rpy.x = msg.rpy.y = msg.rpy.z = 0.0;
            msg.vel.x = msg.vel.y = msg.vel.z = 0.0;
            msg.acc.x = msg.acc.y = msg.acc.z = 0.0;
            msg.gyro.x = msg.gyro.y = msg.gyro.z = 0.0;
            msg.speed = 0.0;
            current_confidence_ = 0.0;
            localization_info_pub_->publish(msg);
            controlledLogInfo("Sensor Status: " + getSensorStatusInfo());
            return;
        }
        if (!isSensorDataValid()) {
            // Sensor data invalid, use extrapolated pose, set status to 4 (localization failed)
            msg.status = 4;
            msg.coord_type = is_use_map_coord_ ? 0 : 1;          
            if (has_valid_pose_history_) {
                Eigen::Matrix4f extrapolated_pose = extrapolatePose(current_time);
                Eigen::Vector3f position = extrapolated_pose.block<3, 1>(0, 3);
                Eigen::Matrix3f rotation = extrapolated_pose.block<3, 3>(0, 0);        
                if (msg.coord_type == 0) {
                    msg.pos.x = position.x();
                    msg.pos.y = position.y();
                    msg.pos.z = position.z();
                }
                Eigen::Quaternionf quat(rotation);
                Eigen::Vector3f rpy = quaternionToNormalizedRPY(quat);
                msg.rpy.x = rpy(0); // roll
                msg.rpy.y = rpy(1); // pitch
                msg.rpy.z = rpy(2); // yaw
            } else {
                msg.pos.x = msg.pos.y = msg.pos.z = 0.0;
                msg.rpy.y = msg.rpy.z = 0.0;
            }  
            msg.vel.x = msg.vel.y = msg.vel.z = 0.0;
            msg.acc.x = msg.acc.y = msg.acc.z = 0.0;
            msg.gyro.x = msg.gyro.y = msg.gyro.z = 0.0;
            msg.speed = 0.0;
            
            localization_info_pub_->publish(msg);
            controlledLogInfo("Sensor Status: " + getSensorStatusInfo() + " | Publishing extrapolated pose with status 4");
            return;
        }
        if (localization_state_ == 0) {
            msg.status = 0;
        } else if (localization_state_ == 1) {
            msg.status = 1;
        } else if (localization_state_ == 2) {
            msg.status = 2;
        } else if (localization_state_ == 4) {
            msg.status = 4;
        } else {
            msg.status = 3;
        }
        msg.coord_type = is_use_map_coord_ ? 0 : 1; // 0 map coordinate, 1 latitude and longitude coordinate

        Eigen::VectorXf state = pose_estimator->GetCurrentUkfState();
        if (msg.coord_type == 0) {
            msg.pos.x = state(0); // pos.x
            msg.pos.y = state(1); // pos.y
            msg.pos.z = state(2); // pos.z
        }
        {
            Eigen::Matrix3f rotation = pose_estimator->matrix().block<3, 3>(0, 0);
            Eigen::Quaternionf quat(rotation);
            Eigen::Vector3f rpy = quaternionToNormalizedRPY(quat);
            msg.rpy.x = rpy(0); // roll
            msg.rpy.y = rpy(1); // pitch
            msg.rpy.z = rpy(2); // yaw
        }
        if (correct_imu_data_ptr_) {
            const auto &imu = *correct_imu_data_ptr_;
            msg.acc.x = imu.linear_acceleration.x;
            msg.acc.y = imu.linear_acceleration.y;
            msg.acc.z = imu.linear_acceleration.z;

            msg.gyro.x = imu.angular_velocity.x;
            msg.gyro.y = imu.angular_velocity.y;
            msg.gyro.z = imu.angular_velocity.z;
        } else {
            msg.acc.x = msg.acc.y = msg.acc.z = 0.0;
            msg.gyro.x = msg.gyro.y = msg.gyro.z = 0.0;
        }
        const Eigen::Vector3f vel = pose_estimator->vel();
        msg.vel.x = vel(0);
        msg.vel.y = vel(1);
        msg.vel.z = vel(2);
        msg.speed = vel.norm();
        localization_info_pub_->publish(msg);
        controlledLogInfo("Sensor Status: " + getSensorStatusInfo());
    }

    /**
     * @brief Odometry publishing timer callback function
     * Only provides supplementary odometry publishing when sensors fail or not initialized, does not interfere with original logic
     */
    void PublishOdomTimer() {
        if (!is_init_success_) {
            RCLCPP_DEBUG(get_logger(), "Localization not initialized, publishing default pose");
            pubDefaultLocalizationOdom(this->get_clock()->now());
            return;
        }  

        if (!isSensorDataValid()) {
            RCLCPP_DEBUG(get_logger(), "Sensor data invalid, checking pose history...");
            auto current_time = this->get_clock()->now();
            if (has_valid_pose_history_) {
                double time_since_last_pose = (current_time - last_valid_pose_time_).seconds();
                RCLCPP_DEBUG(get_logger(), "Time since last pose: %.2fs, using extrapolation", time_since_last_pose);
                
                publishExtrapolatedOdom(current_time);
                return;
            } else {
                RCLCPP_DEBUG(get_logger(), "No valid pose history available, using default pose");
                pubDefaultLocalizationOdom(current_time);
                return;
            }
        }
        if (is_extrapolating_) {
            is_extrapolating_ = false;
            current_confidence_ = 1.0;
            RCLCPP_DEBUG(get_logger(), "Sensor data recovered, resetting extrapolation state, confidence: 1.0");
        }
        RCLCPP_DEBUG(get_logger(), "Sensor data valid, no supplementary odometry needed");
    }

    void LocalizationStateCallback(const std::shared_ptr<robots_dog_msgs::srv::LocalizationState::Request> request,
            std::shared_ptr<robots_dog_msgs::srv::LocalizationState::Response> response) {
         uint8_t receive_message = request->data;
        switch (receive_message) {
        case 0: 
            mode_state_.store(ModeState::INIT);
            response->success = true;
            response->message = "Initialized (Mode: INIT)";
            break;
        case 2:
            mode_state_.store(ModeState::READY);
            response->success = true;
            response->message = "Set READY State (will auto switch to ACTIVE when ready)";
            break;
        case 4: 
            mode_state_.store(ModeState::SUCCESS);
            response->success = true;
            response->message = "Localization stopped (Mode: SUCCESS)";
            break;
        default:
            response->success = false;
            response->message = "Invalid State";
            break;
        }
    }

    void LoadMapCallBack(robots_dog_msgs::srv::LoadMap::Request::SharedPtr request, 
            robots_dog_msgs::srv::LoadMap::Response::SharedPtr response) {
        update_map_flag_.store(true);
        std::string map_path = request->pcd_path;

        if (!std::filesystem::exists(std::filesystem::path(map_path))) {
            RCLCPP_ERROR(get_logger(), "Map file does not exist: %s", map_path.c_str());
            response->success = false;
            response->message = "Map file does not exist.";
            return;
        }
        global_map_points_ptr_->clear();
        pcl::io::loadPCDFile(map_path, *global_map_points_ptr_);
        RCLCPP_INFO(get_logger(), "Global map points size==: %zu", global_map_points_ptr_->points.size());
        if (!global_map_points_ptr_->empty()) {
            pcl::VoxelGrid<pcl::PointXYZI> voxel;
            voxel.setInputCloud(global_map_points_ptr_);
            voxel.setLeafSize(globalmap_voxel_size_, globalmap_voxel_size_, globalmap_voxel_size_);
            voxel.filter(*global_map_points_ptr_);
            if (global_map_points_ptr_->points.size() < 1000) {
                RCLCPP_ERROR(get_logger(), "Global map points size is too small: %zu", global_map_points_ptr_->points.size());
                response->success = false;
                response->message = "Global map points size is too small.";
                return;
            } else {
                if (global_map_pub_->get_subscription_count()) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
                    pcl::VoxelGrid<pcl::PointXYZI> publish_voxel;
                    publish_voxel.setInputCloud(global_map_points_ptr_);
                    publish_voxel.setLeafSize(0.5f, 0.5f, 0.5f); // Only for publishing
                    publish_voxel.filter(*global_map_downsampled);
                    sensor_msgs::msg::PointCloud2 global_map_msg;
                    pcl::toROSMsg(*global_map_downsampled, global_map_msg);
                    global_map_msg.header.frame_id = "map";
                    global_map_msg.header.stamp = this->get_clock()->now();
                    global_map_pub_->publish(global_map_msg);
                    RCLCPP_INFO(get_logger(), "Published downsampled global map to /global_map");
                }
                response->success = true;
                response->message = "Map update successfully.";
                RCLCPP_INFO(get_logger(), "Global map updated!!!!!!");
                registration->setInputTarget(global_map_points_ptr_);;
                Reset();
                update_map_flag_.store(false);
            }
        }
        else {
            RCLCPP_ERROR(get_logger(), "Loaded map is empty: %s", map_path.c_str());
            response->success = false;
            response->message = "Loaded map is empty.";
            update_map_flag_.store(false);
            return;
        }
        update_map_flag_.store(false);
    }

    void Reset() {
        is_init_success_ = false;
        localization_state_ = 0; 
        lidar_status_buffer_.clear();
        imu_status_buffer_.clear();
        for (int i = 0; i < buffer_size_; i++) {
          lidar_status_buffer_.push_back(false);
          imu_status_buffer_.push_back(false);
        }
        has_valid_pose_history_ = false;
    }

private:
  std::string robot_odom_frame_id;
  std::string odom_child_frame_id;
  std::string localization_odom_frame_id;
  bool send_tf_transforms;

  bool use_imu;
  bool invert_acc;
  bool invert_gyro;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                         imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr                 points_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr                 globalmap_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub;
  rclcpp::TimerBase::SharedPtr localization_lidar_info_timer_;
  rclcpp::TimerBase::SharedPtr odom_publish_timer_; 

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr               pose_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         aligned_pub;
  rclcpp::Publisher<localization::msg::ScanMatchingStatus>::SharedPtr status_pub;
  rclcpp::Publisher<robots_dog_msgs::msg::Localization>::SharedPtr    localization_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         global_map_pub_;

  std::unique_ptr<tf2_ros::Buffer>               tf_buffer;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // imu input buffer
  std::mutex imu_data_mutex;
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu_data;
  
  // transformation matrices 
  Eigen::Matrix3f init_rotation_matrix_ = Eigen::Matrix3f::Identity();
  Eigen::Matrix4f gravity_transform_    = Eigen::Matrix4f::Identity();
  
  // Global localization and pose management
  std::shared_ptr<GlobalLocalization> global_localization_ptr_;
  // Pose caching and change detection
  Eigen::Vector3f last_init_pos_     = Eigen::Vector3f::Zero();
  Eigen::Quaternionf last_init_quat_ = Eigen::Quaternionf::Identity();
  bool has_set_init_pose_            = false;
  std::string last_pose_source_      = "none";
  // Configuration parameters for initial pose
  bool specify_init_pose_ = true;
  double init_pos_x_ = 0.0;
  double init_pos_y_ = 0.0;
  double init_pos_z_ = 0.0;
  double init_ori_w_ = 1.0;
  double init_ori_x_ = 0.0;
  double init_ori_y_ = 0.0;
  double init_ori_z_ = 0.0;
  // Global localization parameters
  bool use_global_localization_init_ = true;
  float init_pose_change_threshold_ = 0.01f;      
  float init_quat_change_threshold_ = 0.01f;      
  float global_localization_timeout_ = 10.0f;     
  // Global localization state
  bool global_localization_in_progress_ = false;
  rclcpp::Time global_localization_start_time_;
  bool gl_once_gate_ = true;
  // Sensor data validity tracking
  rclcpp::Time last_lidar_data_time_;
  rclcpp::Time last_imu_data_time_;
  std::deque<bool> lidar_status_buffer_;
  std::deque<bool> imu_status_buffer_;
  int min_valid_count_;      
  int buffer_size_;          
  double sensor_timeout_threshold_ = 1.0; 

  // Store previous state for extrapolation
  Eigen::Vector3f last_velocity_{0.0, 0.0, 0.0};           // Previous velocity
  Eigen::Vector3f last_angular_velocity_{0.0, 0.0, 0.0};   // Previous angular velocity
  Eigen::Matrix4f last_pose_{Eigen::Matrix4f::Identity()};  // Previous pose matrix
  rclcpp::Time last_valid_pose_time_;                        // Time of last valid pose
  bool has_valid_pose_history_ = false;                      // Whether valid pose history exists
  
  // Store latest IMU data for extrapolation
  Eigen::Vector3f latest_angular_velocity_{0.0, 0.0, 0.0}; // Latest IMU angular velocity
  
  // Confidence management
  double current_confidence_ = 1.0;                    // Current confidence
  rclcpp::Time last_confidence_update_time_;           // Last confidence update time
  double confidence_decay_rate_ = 0.1;                 // Confidence decay rate (per second)
  bool is_extrapolating_ = false;                      // Whether currently extrapolating
  
  int log_counter_ = 0;                                // Log counter
  int log_interval_ = 10;                              // Log printing interval (print every 10 times)

  pcl::PointCloud<PointT>::Ptr globalmap;
  pcl::Registration<PointT, PointT>::Ptr registration;
  pcl::PointCloud<PointT>::Ptr global_map_points_ptr_;
  pcl::VoxelGrid<PointT>::Ptr  voxel_filter_ptr_ = pcl::VoxelGrid<PointT>::Ptr(new pcl::VoxelGrid<PointT>());
  pcl::PointCloud<PointT>::Ptr raw_points_ptr_ = nullptr;  ///< Raw point cloud pointer.
  // pose estimator
  std::mutex pose_estimator_mutex;
  std::unique_ptr<localization::PoseEstimator> pose_estimator;

  rclcpp::Service<robots_dog_msgs::srv::LocalizationState>::SharedPtr localization_state_srv_;
  rclcpp::Service<robots_dog_msgs::srv::LoadMap>::SharedPtr load_map_service_ptr_;
  // Parameters
  double cool_time_duration;
  std::string reg_method;
  std::string ndt_neighbor_search_method;
  double ndt_neighbor_search_radius;
  double ndt_resolution;
  bool enable_robot_odometry_prediction;
  int imu_data_filter_num_ = 5;  // Number of IMU data points to filter.

  bool is_init_success_ = false;
  bool is_use_map_coord_ = true;
  int localization_state_ = 0;   // 0: not init, 1: initing, 2: init success, 3: continuous localization, 4: continuous localization failed
  sensor_msgs::msg::Imu::SharedPtr correct_imu_data_ptr_;
  std::atomic<bool> update_map_flag_{false};
  float globalmap_voxel_size_ = 0.3;
  float points_voxel_filter_size_ = 0.2;
  int last_timeout_;
  std::atomic<ModeState>  mode_state_ = ModeState::INIT;
};
}  // namespace localization

int main(int argc, char** argv) {
  omp_set_num_threads(6);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<localization::HdlLocalizationNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
