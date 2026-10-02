/**
 * @file mapping_alg.h
 * @brief
 * @author Liuzhao Li (liliuzhao@jushenzhiren.com)
 * @version 1.0
 * @date 2025-07-31
 * @copyright Copyright (C) 2025 具身智人(北京)科技有限公司
 */

#pragma once
#include "common/state_mode.h"
#include "ikd_tree/ikd_tree.h"
#include "pcd2grid.h"
#include "process/imu_process.h"
#include "process/lidar_process.h"
#include "so3_math.h"

#include <Eigen/Core>
#include <chrono>
#include <csignal>
#include <fstream>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <math.h>
#include <mtk_iekf/esekfom/esekfom.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <robots_dog_msgs/srv/map_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include <unistd.h>
#include <visualization_msgs/msg/marker.hpp>
namespace robot::slam
{
    class MappingAlg : public rclcpp::Node
    {
    public:
        MappingAlg(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        ~MappingAlg();

        void run();

        void reset();

    private:
        double get_time_sec(const builtin_interfaces::msg::Time& time);

        rclcpp::Time get_ros_time(double timestamp);

        void init();

        void pointsBody2World(PointType const* const pi, PointType* const po);

        void pointsBody2Imu(PointType const* const pi, PointType* const po);

        void points_cache_collect();

        void lasermap_fov_segment();

        void lidarCallBack(const sensor_msgs::msg::PointCloud2::UniquePtr msg);

        void imuCallBack(const sensor_msgs::msg::Imu::UniquePtr msg_in);

        bool syncData(MeasureGroup& meas);

        void map_incremental();

        void pubWorldPoints(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull);

        void pubBodyPoints(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body);

        void pubMapPoints(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap);

        void stateCallBack(
            robots_dog_msgs::srv::MapState::Request::SharedPtr request, robots_dog_msgs::srv::MapState::Response::SharedPtr response);

        void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped,
            std::unique_ptr<tf2_ros::TransformBroadcaster>&                               tf_br);

        void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath);

        void map_publish_callback();

        void h_share_model(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data);

        template <typename T>
        void pointBodyToWorld(const Eigen::Matrix<T, 3, 1>& pi, Eigen::Matrix<T, 3, 1>& po)
        {
            Vec3d p_body(pi[0], pi[1], pi[2]);
            Vec3d p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

            po[0] = p_global(0);
            po[1] = p_global(1);
            po[2] = p_global(2);
        }

        template <typename T>
        void set_posestamp(T& out)
        {
            out.pose.position.x    = state_point.pos(0);
            out.pose.position.y    = state_point.pos(1);
            out.pose.position.z    = state_point.pos(2);
            out.pose.orientation.x = geoQuat.x;
            out.pose.orientation.y = geoQuat.y;
            out.pose.orientation.z = geoQuat.z;
            out.pose.orientation.w = geoQuat.w;
        }

        inline double QuaternionToYaw(double x, double y, double z, double w)
        {
            return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        }


    private:
        bool extrinsic_est_en = true, path_en = true;

        float       res_last[100000]       = { 0.0 };
        float       DET_RANGE              = 300.0f;
        const float MOV_THRESHOLD          = 1.5f;
        double      time_diff_lidar_to_imu = 0.0;

        std::mutex              mtx_buffer;
        std::condition_variable sig_buffer;
        std::string             root_dir_ = ROOT_DIR;
        std::string             lid_topic, imu_topic;
        std::string             data_path_;
        std::string             map_frame_ = "map";

        double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
        double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
        double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
        double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
        int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0;
        int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0;
        bool   point_selected_surf[100000] = { 0 };
        bool   lidar_pushed, flg_first_scan = true, flg_EKF_inited;
        bool   pub_world_points_flag_ = false, pub_body_points_flag_ = false;
        bool   is_first_lidar = true;

        Pcd2GridOptions           pcd2pgm_options_;
        std::shared_ptr<Pcd2Grid> pcd2grid_ptr_;

        std::vector<vector<int>>  pointSearchInd_surf;
        std::vector<BoxPointType> cub_needrm;
        std::vector<PointVector>  Nearest_Points;
        std::vector<double>       extrinT;
        std::vector<double>       extrinR;
        std::deque<double>        time_buffer;
        std::deque<CloudPtr>      lidar_buffer;
        std::deque<ImuMessagePtr> imu_buffer;

        CloudPtr featsFromMap     = CloudPtr(new PointCloudType());
        CloudPtr feats_undistort  = CloudPtr(new PointCloudType());
        CloudPtr feats_down_body  = CloudPtr(new PointCloudType());
        CloudPtr feats_down_world = CloudPtr(new PointCloudType());
        CloudPtr normvec          = CloudPtr(new PointCloudType(100000, 1));
        CloudPtr laserCloudOri    = CloudPtr(new PointCloudType(100000, 1));
        CloudPtr corr_normvect    = CloudPtr(new PointCloudType(100000, 1));
        CloudPtr _featsArray      = CloudPtr(new PointCloudType());

        pcl::VoxelGrid<PointType> downSizeFilterSurf;
        pcl::VoxelGrid<PointType> downSizeFilterMap;

        KD_TREE<PointType> ikdtree;

        Vec3d euler_cur;
        Vec3d position_last   = Zero3d;
        Vec3d Lidar_T_wrt_IMU = Zero3d;
        Mat3d Lidar_R_wrt_IMU = Eye3d;

        MeasureGroup                                 Measures;
        esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
        state_ikfom                                  state_point;
        vect3                                        pos_lid;

    public:
        void finish();

    private:
        std::shared_ptr<Preprocess> p_pre = std::make_shared<Preprocess>();
        std::shared_ptr<ImuProcess> p_imu = std::make_shared<ImuProcess>();

    private:
        nav_msgs::msg::Path             path;
        nav_msgs::msg::Odometry         odomAftMapped;
        geometry_msgs::msg::Quaternion  geoQuat;
        geometry_msgs::msg::PoseStamped msg_body_pose;

        BoxPointType LocalMap_Points;
        bool         Localmap_Initialized = false;

        double lidar_mean_scantime = 0.0;
        int    scan_num            = 0;

        PointCloudType::Ptr pcl_wait_pub  = PointCloudType::Ptr(new PointCloudType());
        PointCloudType::Ptr pcl_wait_save = PointCloudType::Ptr(new PointCloudType());

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pubLaserCloudFull_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pubLaserCloudFull_body_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pubLaserCloudMap_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          pubOdomAftMapped_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              pubPath_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         sub_imu_ptr_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_ptr_;

        rclcpp::Service<robots_dog_msgs::srv::MapState>::SharedPtr state_service_;

        std::atomic<SlamState> state_{ SlamState::STABLE };

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr                   map_pub_timer_;

        bool effect_pub_en = false, map_pub_en = false;
        bool archive_raw_map_ = true;
    };

}  // namespace robot::slam
