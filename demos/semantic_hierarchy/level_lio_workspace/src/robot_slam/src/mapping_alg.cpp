/**
 * @file mapping_alg.cpp
 * @brief
 * @author Liuzhao Li (liliuzhao@jushenzhiren.com)
 * @version 1.0
 * @date 2025-07-31
 * @copyright Copyright (C) 2025 具身智人(北京)科技有限公司
 */

#include "mapping_alg.h"

namespace robot::slam
{
    MappingAlg::MappingAlg(const rclcpp::NodeOptions& options)
        : Node("laser_mapping", options)
    {
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.map_en", false);
        archive_raw_map_ = this->declare_parameter<bool>("publish.archive_raw_map", true);
        this->declare_parameter<bool>("publish.world_points_en", true);
        this->declare_parameter<bool>("publish.body_points_en", true);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        gravity_prior_weight = this->declare_parameter<double>("mapping.gravity_prior_weight", 0.0);
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());

        this->declare_parameter<string>("pcd2pgm.file_name", "map");
        this->declare_parameter<double>("pcd2pgm.thre_z_min", 0.2);
        this->declare_parameter<double>("pcd2pgm.thre_z_max", 2.0);
        this->declare_parameter<int>("pcd2pgm.flag_pass_through", 0);
        this->declare_parameter<double>("pcd2pgm.map_resolution", 0.05);
        this->declare_parameter<double>("pcd2pgm.thre_radius", 0.1);
        this->declare_parameter<int>("pcd2pgm.thres_point_count", 10);

        this->get_parameter_or<string>("pcd2pgm.file_name", pcd2pgm_options_.file_name, "map");
        this->get_parameter_or<double>("pcd2pgm.thre_z_min", pcd2pgm_options_.thre_z_min, 0.2);
        this->get_parameter_or<double>("pcd2pgm.thre_z_max", pcd2pgm_options_.thre_z_max, 2.0);
        this->get_parameter_or<int>("pcd2pgm.flag_pass_through", pcd2pgm_options_.flag_pass_through, 0);
        this->get_parameter_or<double>("pcd2pgm.map_resolution", pcd2pgm_options_.map_resolution, 0.05);
        this->get_parameter_or<double>("pcd2pgm.thre_radius", pcd2pgm_options_.thre_radius, 0.1);
        this->get_parameter_or<int>("pcd2pgm.thres_point_count", pcd2pgm_options_.thres_point_count, 10);

        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.world_points_en", pub_world_points_flag_, true);
        this->get_parameter_or<bool>("publish.body_points_en", pub_body_points_flag_, true);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic, "/livox/imu");
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("filter_size_corner", filter_size_corner_min, 0.5);
        this->get_parameter_or<double>("filter_size_surf", filter_size_surf_min, 0.5);
        this->get_parameter_or<double>("filter_size_map", filter_size_map_min, 0.5);
        this->get_parameter_or<double>("cube_side_length", cube_len, 200.f);
        this->get_parameter_or<float>("mapping.det_range", DET_RANGE, 300.f);
        this->get_parameter_or<double>("mapping.fov_degree", fov_deg, 180.f);
        this->get_parameter_or<double>("mapping.gyr_cov", gyr_cov, 0.1);
        this->get_parameter_or<double>("mapping.acc_cov", acc_cov, 0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov", b_gyr_cov, 0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov", b_acc_cov, 0.0001);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, true);
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());

#ifdef ROOT_DIR
        data_path_ = std::string(ROOT_DIR) + "/map";
#else
        RCLCPP_INFO(this->get_logger(), "There is no macro definition of ROOT_DIR");
        data_path_ = "/home/user_name/.jszr/map";
#endif
        if (!checkDirExist(data_path_))
        {
            RCLCPP_INFO(this->get_logger(), "Create map directory failed!!!!!!.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        path.header.stamp    = this->get_clock()->now();
        path.header.frame_id = "map";

        FOV_DEG      = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG)*0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudType());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(Vec3d(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(Vec3d(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(Vec3d(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(Vec3d(b_acc_cov, b_acc_cov, b_acc_cov));

        init();
        pcd2grid_ptr_ = std::make_shared<Pcd2Grid>(pcd2pgm_options_);

        sub_lidar_ptr_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lid_topic, rclcpp::QoS(10).best_effort(), std::bind(&MappingAlg::lidarCallBack, this, std::placeholders::_1));

        sub_imu_ptr_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::QoS(200).best_effort(), std::bind(&MappingAlg::imuCallBack, this, std::placeholders::_1));
        pubLaserCloudFull_      = this->create_publisher<sensor_msgs::msg::PointCloud2>("/world_points", rclcpp::QoS(20).best_effort());
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/body_points", 20);
        pubLaserCloudMap_       = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_points", 20);
        pubOdomAftMapped_       = this->create_publisher<nav_msgs::msg::Odometry>("/slam_odom", 20);
        pubPath_                = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
        tf_broadcaster_         = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        state_service_ = this->create_service<robots_dog_msgs::srv::MapState>(
            "/slam_state_service", std::bind(&MappingAlg::stateCallBack, this, std::placeholders::_1, std::placeholders::_2));

        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&MappingAlg::map_publish_callback, this));

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    MappingAlg ::~MappingAlg() {}

    void MappingAlg::init()
    {
        std::vector<double> epsi(23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, std::bind(&MappingAlg::h_share_model, this, std::placeholders::_1, std::placeholders::_2),
            NUM_MAX_ITERATIONS, epsi.data());
    }

    void MappingAlg::stateCallBack(
        robots_dog_msgs::srv::MapState::Request::SharedPtr request, robots_dog_msgs::srv::MapState::Response::SharedPtr response)
    {
        uint8_t receive_message = request->data;
        switch (receive_message)
        {
            case 0:
                state_.store(SlamState::STABLE);
                response->success = true;
                response->message = "Set STABLE State!!!!!!";
                break;
            case 1:
                state_.store(SlamState::PASSIVE);
                response->success = true;
                response->message = "Set PASSIVE State!!!!!!";
                break;
            case 2:
                state_.store(SlamState::READY);
                response->success = true;
                response->message = "Set READY State!!!!!!";
                break;
            case 3:
                state_.store(SlamState::ACTIVE);
                reset();
                response->success = true;
                response->message = "Set ACTIVE State!!!!!!";
                break;
            case 4:
                state_.store(SlamState::ERROR);
                response->success = true;
                response->message = "Set ERROR State!!!!!!";
                break;
            case 5:
                state_.store(SlamState::SAVE);
                response->success = true;
                response->message = "Set SAVE State!!!!!!";
                break;
            default:
                response->success = false;
                response->message = "SLAM not this State Fail !!!!!!";
                break;
        }
    }

    void MappingAlg::reset()
    {
        time_buffer.clear();
        lidar_buffer.clear();
        imu_buffer.clear();
        is_first_lidar      = true;
        flg_first_scan      = true;
        scan_num            = 0;
        lidar_mean_scantime = 0.0;
        memset(point_selected_surf, true, sizeof(point_selected_surf));

        p_imu->reset();

        state_ikfom state_updated;
        state_updated.pos = Zero3d;
        state_updated.rot = Quatd(1.0, 0.0, 0.0, 0.0);
        state_point       = state_updated;  // 对state_point进行更新，state_point可视化用到
        kf.change_x(state_updated);
        Localmap_Initialized = false;
    }


    double MappingAlg::get_time_sec(const builtin_interfaces::msg::Time& time)
    {
        return rclcpp::Time(time).seconds();
    }

    rclcpp::Time MappingAlg::get_ros_time(double timestamp)
    {
        int32_t  sec       = std::floor(timestamp);
        auto     nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
        uint32_t nanosec   = nanosec_d;
        return rclcpp::Time(sec, nanosec);
    }

    void MappingAlg::pointsBody2World(PointType const* const pi, PointType* const po)
    {
        Vec3d p_body(pi->x, pi->y, pi->z);
        Vec3d p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

        po->x         = p_global(0);
        po->y         = p_global(1);
        po->z         = p_global(2);
        po->intensity = pi->intensity;
    }

    void MappingAlg::pointsBody2Imu(PointType const* const pi, PointType* const po)
    {
        Vec3d p_body_lidar(pi->x, pi->y, pi->z);
        Vec3d p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

        po->x         = p_body_imu(0);
        po->y         = p_body_imu(1);
        po->z         = p_body_imu(2);
        po->intensity = pi->intensity;
    }

    void MappingAlg::points_cache_collect()
    {
        PointVector points_history;
        ikdtree.acquire_removed_points(points_history);
    }

    void MappingAlg::lasermap_fov_segment()
    {
        cub_needrm.clear();
        int   kdtree_delete_counter = 0;
        Vec3d pos_LiD               = pos_lid;
        if (!Localmap_Initialized)
        {
            for (int i = 0; i < 3; i++)
            {
                LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
                LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
            }
            Localmap_Initialized = true;
            return;
        }
        float dist_to_map_edge[3][2];
        bool  need_move = false;
        for (int i = 0; i < 3; i++)
        {
            dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
            dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
                need_move = true;
        }
        if (!need_move)
            return;
        BoxPointType New_LocalMap_Points, tmp_boxpoints;
        New_LocalMap_Points = LocalMap_Points;
        float mov_dist      = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
        for (int i = 0; i < 3; i++)
        {
            tmp_boxpoints = LocalMap_Points;
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
            {
                New_LocalMap_Points.vertex_max[i] -= mov_dist;
                New_LocalMap_Points.vertex_min[i] -= mov_dist;
                tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
                cub_needrm.push_back(tmp_boxpoints);
            }
            else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            {
                New_LocalMap_Points.vertex_max[i] += mov_dist;
                New_LocalMap_Points.vertex_min[i] += mov_dist;
                tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
                cub_needrm.push_back(tmp_boxpoints);
            }
        }
        LocalMap_Points = New_LocalMap_Points;

        points_cache_collect();
        double delete_begin = omp_get_wtime();
        if (cub_needrm.size() > 0)
            kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    }

    void MappingAlg::lidarCallBack(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
    {
        mtx_buffer.lock();
        double cur_time              = get_time_sec(msg->header.stamp);
        double preprocess_start_time = omp_get_wtime();
        scan_count++;
        if (!is_first_lidar && cur_time < last_timestamp_lidar)
        {
            std::cerr << "lidar loop back, clear buffer" << std::endl;
            lidar_buffer.clear();
        }
        if (is_first_lidar)
        {
            is_first_lidar = false;
        }
        last_timestamp_lidar = cur_time;

        PointCloudType::Ptr ptr(new PointCloudType());

        pcl::PointCloud<livox_pcl::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);

        p_pre->process(pl_orig, ptr);
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(last_timestamp_lidar);

        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }

    void MappingAlg::imuCallBack(const sensor_msgs::msg::Imu::UniquePtr msg_in)
    {
        sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

        msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);

        double timestamp = get_time_sec(msg->header.stamp);

        mtx_buffer.lock();

        if (timestamp < last_timestamp_imu)
        {
            std::cerr << "lidar loop back, clear buffer" << std::endl;
            imu_buffer.clear();
        }

        last_timestamp_imu = timestamp;

        ImuMessagePtr imu_msg_ptr =
            std::make_shared<ImuMessage>(timestamp, Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));

        imu_buffer.push_back(imu_msg_ptr);
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }

    bool MappingAlg::syncData(MeasureGroup& meas)
    {
        if (lidar_buffer.empty() || imu_buffer.empty())
        {
            return false;
        }

        if (!lidar_pushed)
        {
            meas.lidar          = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            if (meas.lidar->points.size() <= 1)  // time too little
            {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
                std::cerr << "Too few input point cloud!\n";
            }
            else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
            {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            }
            else
            {
                scan_num++;
                lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
                lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
            }

            meas.lidar_end_time = lidar_end_time;

            lidar_pushed = true;
        }

        if (last_timestamp_imu < lidar_end_time)
        {
            return false;
        }

        double imu_time = imu_buffer.front()->timestamp;
        meas.imu.clear();
        while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
        {
            imu_time = imu_buffer.front()->timestamp;
            if (imu_time > lidar_end_time)
                break;
            meas.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }

        lidar_buffer.pop_front();
        time_buffer.pop_front();
        lidar_pushed = false;
        return true;
    }

    void MappingAlg::map_incremental()
    {
        PointVector PointToAdd;
        PointVector PointNoNeedDownsample;
        PointToAdd.reserve(feats_down_size);
        PointNoNeedDownsample.reserve(feats_down_size);
        for (int i = 0; i < feats_down_size; i++)
        {
            pointsBody2World(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
            if (!Nearest_Points[i].empty() && flg_EKF_inited)
            {
                const PointVector& points_near = Nearest_Points[i];
                bool               need_add    = true;
                BoxPointType       Box_of_Point;
                PointType          downsample_result, mid_point;
                mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
                mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
                mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
                float dist  = calc_dist(feats_down_world->points[i], mid_point);
                if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min
                    && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min
                    && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
                {
                    PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                    continue;
                }
                for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
                {
                    if (points_near.size() < NUM_MATCH_POINTS)
                        break;
                    if (calc_dist(points_near[readd_i], mid_point) < dist)
                    {
                        need_add = false;
                        break;
                    }
                }
                if (need_add)
                    PointToAdd.push_back(feats_down_world->points[i]);
            }
            else
            {
                PointToAdd.push_back(feats_down_world->points[i]);
            }
        }

        double st_time        = omp_get_wtime();
        int    add_point_size = ikdtree.Add_Points(PointToAdd, true);
        ikdtree.Add_Points(PointNoNeedDownsample, false);
        add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    }

    void MappingAlg::pubWorldPoints(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
    {

        PointCloudType::Ptr laserCloudFullRes(feats_undistort);
        int                 size = laserCloudFullRes->points.size();
        PointCloudType::Ptr laserCloudWorld(new PointCloudType(size, 1));

        for (int i = 0; i < size; i++)
        {
            pointsBody2World(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }
        // Online exploration keeps a bounded voxel map in its own node.
        // Preserve the original raw archive for the stock mapping workflow.
        if (archive_raw_map_)
            *pcl_wait_pub += *laserCloudWorld;
        if (pub_world_points_flag_)
        {
            sensor_msgs::msg::PointCloud2 laserCloudmsg;
            pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
            laserCloudmsg.header.stamp    = get_ros_time(lidar_end_time);
            laserCloudmsg.header.frame_id = "map";
            pubLaserCloudFull->publish(laserCloudmsg);
        }
    }

    void MappingAlg::pubBodyPoints(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
    {
        int                 size = feats_undistort->points.size();
        PointCloudType::Ptr laserCloudIMUBody(new PointCloudType(size, 1));

        for (int i = 0; i < size; i++)
        {
            pointsBody2Imu(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
        laserCloudmsg.header.stamp    = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "body";
        pubLaserCloudFull_body->publish(laserCloudmsg);
    }

    void MappingAlg::pubMapPoints(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
    {
        // PointCloudType::Ptr laserCloudFullRes(feats_down_body);
        // int                 size = laserCloudFullRes->points.size();
        // PointCloudType::Ptr laserCloudWorld(new PointCloudType(size, 1));

        // for (int i = 0; i < size; i++)
        // {
        //     pointsBody2World(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        // }
        // *pcl_wait_pub += *laserCloudWorld;

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
        laserCloudmsg.header.stamp    = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "map";
        pubLaserCloudMap->publish(laserCloudmsg);
    }

    void MappingAlg::publish_odometry(
        const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster>& tf_br)
    {
        odomAftMapped.header.frame_id = "map";
        odomAftMapped.child_frame_id  = "body";
        odomAftMapped.header.stamp    = get_ros_time(lidar_end_time);
        set_posestamp(odomAftMapped.pose);
        pubOdomAftMapped->publish(odomAftMapped);
        auto P = kf.get_P();
        for (int i = 0; i < 6; i++)
        {
            int k                                    = i < 3 ? i + 3 : i - 3;
            odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
            odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
            odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
            odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
            odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
            odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
        }

        geometry_msgs::msg::TransformStamped trans;
        trans.header.frame_id         = "map";
        trans.child_frame_id          = "body";
        trans.header.stamp            = get_ros_time(lidar_end_time);
        trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
        trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
        trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
        trans.transform.rotation.w    = odomAftMapped.pose.pose.orientation.w;
        trans.transform.rotation.x    = odomAftMapped.pose.pose.orientation.x;
        trans.transform.rotation.y    = odomAftMapped.pose.pose.orientation.y;
        trans.transform.rotation.z    = odomAftMapped.pose.pose.orientation.z;
        tf_br->sendTransform(trans);
    }

    void MappingAlg::publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
    {
        set_posestamp(msg_body_pose);
        msg_body_pose.header.stamp    = get_ros_time(lidar_end_time);  // ros::Time().fromSec(lidar_end_time);
        msg_body_pose.header.frame_id = "map";

        static int jjj = 0;
        jjj++;
        if (jjj % 10 == 0)
        {
            path.poses.push_back(msg_body_pose);
            pubPath->publish(path);
        }
    }

    void MappingAlg::h_share_model(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data)
    {
        laserCloudOri->clear();
        corr_normvect->clear();

#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
        for (int i = 0; i < feats_down_size; i++)
        {
            PointType& point_body = feats_down_body->points[i];
            PointType  point_world;

            Vec3d p_body(point_body.x, point_body.y, point_body.z);
            Vec3d p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
            point_world.x         = p_global(0);
            point_world.y         = p_global(1);
            point_world.z         = p_global(2);
            point_world.intensity = point_body.intensity;

            vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

            auto& points_near = Nearest_Points[i];

            if (ekfom_data.converge)
            {
                /** Find the closest surfaces in the map **/
                ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
                point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS        ? false
                                         : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                      : true;
            }

            if (!point_selected_surf[i])
                continue;

            // 添加额外的安全检查
            if (points_near.size() < NUM_MATCH_POINTS)
            {
                point_selected_surf[i] = false;
                continue;
            }

            VF(4) pabcd;
            point_selected_surf[i] = false;
            if (esti_plane<float>(pabcd, points_near, 0.1f))
            {
                float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
                float s   = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

                if (s > 0.9)
                {
                    point_selected_surf[i]       = true;
                    normvec->points[i].x         = pabcd(0);
                    normvec->points[i].y         = pabcd(1);
                    normvec->points[i].z         = pabcd(2);
                    normvec->points[i].intensity = pd2;
                    res_last[i]                  = abs(pd2);
                }
            }
        }

        effct_feat_num = 0;

        for (int i = 0; i < feats_down_size; i++)
        {
            if (point_selected_surf[i])
            {
                laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
                corr_normvect->points[effct_feat_num] = normvec->points[i];
                effct_feat_num++;
            }
        }

        if (effct_feat_num < 1)
        {
            ekfom_data.valid = false;
            std::cerr << "No Effective Points!" << std::endl;
            // ROS_WARN("No Effective Points! \n");
            return;
        }

        /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
        const bool use_gravity_prior = gravity_prior_weight > 0 && p_imu->gravity_reference_ready;
        const int rows = effct_feat_num + (use_gravity_prior ? 3 : 0);
        ekfom_data.h_x = Eigen::MatrixXd::Zero(rows, 12);
        ekfom_data.h.resize(rows);
        if (use_gravity_prior) {
            const Vec3d prediction = s.rot.conjugate() * Vec3d::UnitZ();
            Mat3d jacobian;
            jacobian << SKEW_SYM_MATRX(prediction);
            ekfom_data.h_x.block<3,3>(effct_feat_num,3) = gravity_prior_weight * jacobian;
            ekfom_data.h.tail<3>() = gravity_prior_weight * (p_imu->gravity_body_up - prediction);
        }

        for (int i = 0; i < effct_feat_num; i++)
        {
            const PointType& laser_p = laserCloudOri->points[i];
            Vec3d            point_this_be(laser_p.x, laser_p.y, laser_p.z);
            Mat3d            point_be_crossmat;
            point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
            Vec3d point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
            Mat3d point_crossmat;
            point_crossmat << SKEW_SYM_MATRX(point_this);

            const PointType& norm_p = corr_normvect->points[i];
            Vec3d            norm_vec(norm_p.x, norm_p.y, norm_p.z);

            Vec3d C(s.rot.conjugate() * norm_vec);
            Vec3d A(point_crossmat * C);
            if (extrinsic_est_en)
            {
                Vec3d B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);  // s.rot.conjugate()*norm_vec);
                ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
            }
            else
            {
                ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            }

            ekfom_data.h(i) = -norm_p.intensity;
        }
    }

    void MappingAlg::run()
    {
        if (state_.load() == SlamState::ACTIVE)
        {

            if (syncData(Measures))
            {
                if (flg_first_scan)
                {
                    first_lidar_time        = Measures.lidar_beg_time;
                    p_imu->first_lidar_time = first_lidar_time;
                    flg_first_scan          = false;
                    return;
                }

                p_imu->Process(Measures, kf, feats_undistort);
                state_point = kf.get_x();
                pos_lid     = state_point.pos + state_point.rot * state_point.offset_T_L_I;

                if (feats_undistort->empty())
                {
                    RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                    return;
                }

                flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
                lasermap_fov_segment();

                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
                feats_down_size = feats_down_body->points.size();
                if (ikdtree.Root_Node == nullptr)
                {
                    RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                    if (feats_down_size > 5)
                    {
                        ikdtree.set_downsample_param(filter_size_map_min);
                        feats_down_world->resize(feats_down_size);
                        for (int i = 0; i < feats_down_size; i++)
                        {
                            pointsBody2World(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                        }
                        ikdtree.Build(feats_down_world->points);
                    }
                    return;
                }
                if (feats_down_size < 5)
                {
                    RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                    return;
                }

                normvec->resize(feats_down_size);
                feats_down_world->resize(feats_down_size);

                Vec3d ext_euler = SO3ToEuler(state_point.offset_R_L_I);

                if (0)  // If you need to see map point, change to "if(1)"
                {
                    PointVector().swap(ikdtree.PCL_Storage);
                    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                    featsFromMap->clear();
                    featsFromMap->points = ikdtree.PCL_Storage;
                }

                pointSearchInd_surf.resize(feats_down_size);
                Nearest_Points.resize(feats_down_size);
                int  rematch_num       = 0;
                bool nearest_search_en = true;  //

                double solve_H_time = 0;
                kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
                state_point = kf.get_x();
                euler_cur   = SO3ToEuler(state_point.rot);
                pos_lid     = state_point.pos + state_point.rot * state_point.offset_T_L_I;
                geoQuat.x   = state_point.rot.coeffs()[0];
                geoQuat.y   = state_point.rot.coeffs()[1];
                geoQuat.z   = state_point.rot.coeffs()[2];
                geoQuat.w   = state_point.rot.coeffs()[3];
                map_incremental();

                publish_odometry(pubOdomAftMapped_, tf_broadcaster_);
                pubWorldPoints(pubLaserCloudFull_);

                if (path_en)
                    publish_path(pubPath_);
                if (pub_body_points_flag_)
                    pubBodyPoints(pubLaserCloudFull_body_);
            }
        }
        else if (state_.load() == SlamState::SAVE)
        {
            finish();
            state_.store(SlamState::READY);
        }
        else
        {
            return;
        }
    }

    void MappingAlg::map_publish_callback()
    {
        if (map_pub_en)
            pubMapPoints(pubLaserCloudMap_);
    }

    void MappingAlg::finish()
    {
        if (pcl_wait_pub->size() > 0)
        {
            string         file_name = data_path_ + string("/map.pcd");
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /data/" << file_name << endl;
            pcd_writer.writeBinary(file_name, *pcl_wait_pub);

            string pcd2grid_dir = data_path_ + "/" + pcd2pgm_options_.file_name;
            pcd2grid_ptr_->run(pcl_wait_pub, pcd2grid_dir);

            std::ofstream ofs;
            std::string   path_flie = data_path_ + "/map.txt";
            ofs.open(path_flie, std::ios::out | std::ios::trunc);
            if (!ofs.is_open())
            {
                std::cout << "Failed to open traj_file: " << path_flie << std::endl;
                return;
            }

            double theta;
            ofs << "# path" << std::endl;
            for (const auto& p : path.poses)
            {
                theta = QuaternionToYaw(p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
                ofs << std::fixed << std::setprecision(2) << p.pose.position.x << " " << p.pose.position.y << " " << theta << std::endl;
            }
            ofs.close();

            RCLCPP_INFO(get_logger(), "Save Map Success.");
        }
    }
}  // namespace robot::slam
