
#include "localization/static_imu_init.hpp"

namespace localization
{

    void StaticIMUInit::SetParam(float time_long, int max_queue_size, float max_gyro_var, float max_acce_var)
    {
        init_time_seconds_   = time_long;
        init_queue_max_size_ = max_queue_size;
        max_static_gyro_var_ = max_gyro_var;
        max_static_acce_var_ = max_acce_var;
    }

    void StaticIMUInit::Reset()
    {
        mean_gyro_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        mean_acce_ = Eigen::Vector3d(0.0, 0.0, -1.0);
    }


    bool StaticIMUInit::AddIMUData(const sensor_msgs::msg::Imu::ConstSharedPtr imu)
    {
        if (init_success_)
        {
            return true;
        }

        if (init_imu_buffer_.empty())
        {
            init_start_time_ = rclcpp::Time(imu->header.stamp).seconds();
        }

        init_imu_buffer_.push_back(imu);

        double init_time = rclcpp::Time(imu->header.stamp).seconds() - init_start_time_;
        if (init_time > init_time_seconds_)
        {
            TryInit();
        }

        while (init_imu_buffer_.size() > init_queue_max_size_)
        {
            init_imu_buffer_.pop_front();
        }
        return false;
    }

    bool StaticIMUInit::TryInit()
    {
        if (init_imu_buffer_.size() < 10)
        {
            return false;
        }
        Eigen::Vector3d cur_acc, cur_gyr;
        ComputeMeanAndCovDiag(init_imu_buffer_, mean_gyro_, gyro_cov_,
            [](const sensor_msgs::msg::Imu::ConstSharedPtr& imu)
            {
                return Eigen::Vector3d(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
            });
        ComputeMeanAndCovDiag(init_imu_buffer_, mean_acce_, acce_cov_,
            [](const sensor_msgs::msg::Imu::ConstSharedPtr& imu)
            {
                return Eigen::Vector3d(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
            });
        //  gravity_ = - mean_acce_ /  mean_acce_.norm () * 9.81;
        // ComputeMeanAndCovDiag ( init_imu_buffer_,  mean_acce_,  acce_cov_, [this](const sensor_msgs::msg::Imu::ConstSharedPtr& imu)
        // {return Eigen::Vector3d (imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z) +  gravity_;});

        if (gyro_cov_.norm() > max_static_gyro_var_)
        {
            std::cout << "陀螺仪测量噪声太大 --> " << gyro_cov_.norm() << " > " << max_static_gyro_var_ << std::endl;
            return false;
        }

        // if ( acce_cov_.norm () >  max_static_acce_var_) {
        //         std::cout << "加速度计测量噪声太大  --> " <<  acce_cov_.norm () << " > " <<  max_static_acce_var_ << std::endl;
        //         return false;
        // }

        init_bias_gyro_ = mean_gyro_;
        init_bias_acce_ = mean_acce_;
        init_success_   = true;
        std::cout << "IMU Init Sucessful !!!!!!" << std::endl;
        return true;
    }

}  // namespace localization
