/**
 * @file custom_type.h
 * @brief
 * @author Liuzhao Li (liliuzhao@jushenzhiren.com)
 * @version 1.0
 * @date 2025-07-31
 * @copyright Copyright (C) 2025 具身智人(北京)科技有限公司
 */
#pragma once
#include "so3_math.h"
#include "common/types.h"
#include "common/const_value.h"
#include <deque>
namespace robot::slam
{
    struct ImuMessage
    {
        ImuMessage()
            : timestamp(0.0),
              gyr(Zero3d),
              acc(Zero3d)
        {
        }

        ImuMessage(double ts, const Vec3d& g, const Vec3d& a)
            : timestamp(ts),
              gyr(g),
              acc(a)
        {
        }

        double timestamp;  // in seconds
        Vec3d  gyr;        // gyroscope data
        Vec3d  acc;        // accelerometer data
    };

    struct Pose6D
    {
        Vec3d  acc;  // Position in 3D space
        Vec3d  gyr;  // Orientation as a quaternion
        Vec3d  vel;
        Vec3d  pos;          // Linear velocity
        double rot[9];       // Rotation matrix in row-major order
        double offset_time;  // Timestamp offset for the pose
    };

    struct MeasureGroup
    {
        MeasureGroup()
        {
            lidar_beg_time = 0.0;
            this->lidar.reset(new PointCloudType());
        };
        double                                  lidar_beg_time;
        double                                  lidar_end_time;
        CloudPtr                                lidar;
        std::deque<std::shared_ptr<ImuMessage>> imu;
    };

    struct StatesGroup
    {
        StatesGroup()
        {
            this->rot_end               = Mat3d::Identity();
            this->pos_end               = Zero3d;
            this->vel_end               = Zero3d;
            this->bias_g                = Zero3d;
            this->bias_a                = Zero3d;
            this->gravity               = Zero3d;
            this->cov                   = MD(DIM_STATE, DIM_STATE)::Identity() * INIT_COV;
            this->cov.block<9, 9>(9, 9) = MD(9, 9)::Identity() * 0.00001;
        };

        StatesGroup(const StatesGroup& b)
        {
            this->rot_end = b.rot_end;
            this->pos_end = b.pos_end;
            this->vel_end = b.vel_end;
            this->bias_g  = b.bias_g;
            this->bias_a  = b.bias_a;
            this->gravity = b.gravity;
            this->cov     = b.cov;
        };

        StatesGroup& operator=(const StatesGroup& b)
        {
            this->rot_end = b.rot_end;
            this->pos_end = b.pos_end;
            this->vel_end = b.vel_end;
            this->bias_g  = b.bias_g;
            this->bias_a  = b.bias_a;
            this->gravity = b.gravity;
            this->cov     = b.cov;
            return *this;
        };

        StatesGroup operator+(const Eigen::Matrix<double, DIM_STATE, 1>& state_add)
        {
            StatesGroup a;
            a.rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
            a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
            a.vel_end = this->vel_end + state_add.block<3, 1>(6, 0);
            a.bias_g  = this->bias_g + state_add.block<3, 1>(9, 0);
            a.bias_a  = this->bias_a + state_add.block<3, 1>(12, 0);
            a.gravity = this->gravity + state_add.block<3, 1>(15, 0);
            a.cov     = this->cov;
            return a;
        };

        StatesGroup& operator+=(const Eigen::Matrix<double, DIM_STATE, 1>& state_add)
        {
            this->rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
            this->pos_end += state_add.block<3, 1>(3, 0);
            this->vel_end += state_add.block<3, 1>(6, 0);
            this->bias_g += state_add.block<3, 1>(9, 0);
            this->bias_a += state_add.block<3, 1>(12, 0);
            this->gravity += state_add.block<3, 1>(15, 0);
            return *this;
        };

        Eigen::Matrix<double, DIM_STATE, 1> operator-(const StatesGroup& b)
        {
            Eigen::Matrix<double, DIM_STATE, 1> a;
            Mat3d                               rotd(b.rot_end.transpose() * this->rot_end);
            a.block<3, 1>(0, 0)  = Log(rotd);
            a.block<3, 1>(3, 0)  = this->pos_end - b.pos_end;
            a.block<3, 1>(6, 0)  = this->vel_end - b.vel_end;
            a.block<3, 1>(9, 0)  = this->bias_g - b.bias_g;
            a.block<3, 1>(12, 0) = this->bias_a - b.bias_a;
            a.block<3, 1>(15, 0) = this->gravity - b.gravity;
            return a;
        };

        void resetpose()
        {
            this->rot_end = Mat3d::Identity();
            this->pos_end = Zero3d;
            this->vel_end = Zero3d;
        }

        Mat3d                                       rot_end;  // the estimated attitude (rotation matrix) at the end lidar point
        Vec3d                                       pos_end;  // the estimated position at the end lidar point (world frame)
        Vec3d                                       vel_end;  // the estimated velocity at the end lidar point (world frame)
        Vec3d                                       bias_g;   // gyroscope bias
        Vec3d                                       bias_a;   // accelerator bias
        Vec3d                                       gravity;  // the estimated gravity acceleration
        Eigen::Matrix<double, DIM_STATE, DIM_STATE> cov;      // states covariance
    };

}  // namespace robot::slam

using ImuMessagePtr = std::shared_ptr<robot::slam::ImuMessage>;