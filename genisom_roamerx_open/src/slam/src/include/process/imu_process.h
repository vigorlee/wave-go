/**
 * @file imu_process.h
 * @brief
 * @author Liuzhao Li (liliuzhao@jushenzhiren.com)
 * @version 1.0
 * @date 2025-07-31
 * @copyright Copyright (C) 2025 具身智人(北京)科技有限公司
 */

#pragma once
#include <deque>
#include <mutex>
#include "so3_math.h"
#include "common.h"
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "use_ikfom.h"

const bool time_list(robot::slam::PointType& x, robot::slam::PointType& y);

class ImuProcess
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();

    void reset();

    ~ImuProcess();

    void                          Reset();
    void                          set_extrinsic(const robot::slam::Vec3d& transl, const robot::slam::Mat3d& rot);
    void                          set_extrinsic(const robot::slam::Vec3d& transl);
    void                          set_extrinsic(const MD(4, 4) & T);
    void                          set_gyr_cov(const robot::slam::Vec3d& scaler);
    void                          set_acc_cov(const robot::slam::Vec3d& scaler);
    void                          set_gyr_bias_cov(const robot::slam::Vec3d& b_g);
    void                          set_acc_bias_cov(const robot::slam::Vec3d& b_a);
    Eigen::Matrix<double, 12, 12> Q;
    void                          Process(
                                 const robot::slam::MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, robot::slam::CloudPtr pcl_un_);

    robot::slam::Vec3d cov_acc;
    robot::slam::Vec3d cov_gyr;
    robot::slam::Vec3d cov_acc_scale;
    robot::slam::Vec3d cov_gyr_scale;
    robot::slam::Vec3d cov_bias_gyr;
    robot::slam::Vec3d cov_bias_acc;
    double             first_lidar_time;

private:
    void IMU_init(const robot::slam::MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, int& N);
    void UndistortPcl(const robot::slam::MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
        robot::slam::PointCloudType& pcl_in_out);

    robot::slam::CloudPtr            cur_pcl_un_;
    ImuMessagePtr                    last_imu_;
    std::deque<ImuMessagePtr>        v_imu_;
    std::vector<robot::slam::Pose6D> IMUpose;
    std::vector<robot::slam::Mat3d>  v_rot_pcl_;
    robot::slam::Mat3d               Lidar_R_wrt_IMU;
    robot::slam::Vec3d               Lidar_T_wrt_IMU;
    robot::slam::Vec3d               mean_acc;
    robot::slam::Vec3d               mean_gyr;
    robot::slam::Vec3d               angvel_last;
    robot::slam::Vec3d               acc_s_last;
    double                           start_timestamp_;
    double                           last_lidar_end_time_;
    int                              init_iter_num  = 1;
    bool                             b_first_frame_ = true;
    bool                             imu_need_init_ = true;
};