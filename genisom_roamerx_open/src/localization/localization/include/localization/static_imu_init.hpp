
#pragma once
#include <Eigen/Core>
#include <deque>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace localization
{

    /**
     * @class StaticIMUInit
     * @brief A class for initializing static IMU parameters and computing statistics.
     */
    class StaticIMUInit
    {
    public:
        /**
         * @brief Default constructor for StaticIMUInit.
         */
        StaticIMUInit() = default;

        /**
         * @brief Sets the initialization parameters for the IMU.
         * @param time_long The duration of time for initialization in seconds.
         * @param max_queue_size The maximum size of the IMU data queue.
         * @param max_gyro_var The maximum allowable variance for gyroscope data.
         * @param max_acce_var The maximum allowable variance for accelerometer data.
         */
        void SetParam(float time_long, int max_queue_size, float max_gyro_var, float max_acce_var);

        /**
         * @brief Checks if the initialization was successful.
         * @return True if initialization was successful, false otherwise.
         */
        inline bool InitSuccess() const
        {
            return init_success_;
        }

        /**
         * @brief Adds IMU data to the initialization buffer.
         * @param imu A shared pointer to the IMU data message.
         * @return True if the data was successfully added, false otherwise.
         */
        bool AddIMUData(const sensor_msgs::msg::Imu::ConstSharedPtr imu);

        /**
         * @brief Gets the computed covariance of gyroscope data.
         * @return A 3D vector representing the gyroscope covariance.
         */
        inline Eigen::Vector3d GetCovGyro() const
        {
            return gyro_cov_;
        }

        /**
         * @brief Gets the computed covariance of accelerometer data.
         * @return A 3D vector representing the accelerometer covariance.
         */
        inline Eigen::Vector3d GetCovAcce() const
        {
            return acce_cov_;
        }

        /**
         * @brief Gets the initialized gyroscope bias.
         * @return A 3D vector representing the gyroscope bias.
         */
        inline Eigen::Vector3d GetInitBg() const
        {
            return init_bias_gyro_;
        }

        /**
         * @brief Gets the initialized accelerometer bias.
         * @return A 3D vector representing the accelerometer bias.
         */
        inline Eigen::Vector3d GetInitBa() const
        {
            return init_bias_acce_;
        }

        /**
         * @brief Gets the estimated gravity vector.
         * @return A 3D vector representing the gravity.
         */
        inline Eigen::Vector3d GetGravity() const
        {
            return gravity_;
        }

    private:
        /**
         * @brief Computes the mean and diagonal covariance of a dataset.
         * @tparam C The container type of the dataset.
         * @tparam D The data type of the mean and covariance.
         * @tparam Getter A callable to extract data from the container elements.
         * @param data The dataset container.
         * @param mean The computed mean of the dataset.
         * @param cov_diag The computed diagonal covariance of the dataset.
         * @param getter The callable to extract data from the container elements.
         */
        template <typename C, typename D, typename Getter>
        void ComputeMeanAndCovDiag(const C& data, D& mean, D& cov_diag, Getter&& getter)
        {
            size_t len = data.size();
            assert(len > 1);
            mean = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                       [&getter](const D& sum, const auto& data) -> D
                       {
                           return sum + getter(data);
                       })
                   / len;
            cov_diag = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                           [&mean, &getter](const D& sum, const auto& data) -> D
                           {
                               return sum + (getter(data) - mean).cwiseAbs2().eval();
                           })
                       / (len - 1);
        }

        /**
         * @brief Attempts to initialize the IMU parameters.
         * @return True if initialization was successful, false otherwise.
         */
        bool TryInit();

        /**
         * @brief Resets the initialization state and parameters.
         */
        void Reset();

    private:
        float           init_time_seconds_;                         ///< The duration of time for initialization in seconds.
        int             init_queue_max_size_;                       ///< The maximum size of the IMU data queue.
        float           max_static_gyro_var_;                       ///< The maximum allowable variance for gyroscope data.
        float           max_static_acce_var_;                       ///< The maximum allowable variance for accelerometer data.
        bool            init_success_   = false;                    ///< Flag indicating whether initialization was successful.
        Eigen::Vector3d gyro_cov_       = Eigen::Vector3d::Zero();  ///< Covariance of gyroscope data.
        Eigen::Vector3d acce_cov_       = Eigen::Vector3d::Zero();  ///< Covariance of accelerometer data.
        Eigen::Vector3d init_bias_gyro_ = Eigen::Vector3d::Zero();  ///< Initialized gyroscope bias.
        Eigen::Vector3d init_bias_acce_ = Eigen::Vector3d::Zero();  ///< Initialized accelerometer bias.
        Eigen::Vector3d gravity_        = Eigen::Vector3d::Zero();  ///< Estimated gravity vector.

        bool            first_frame_flag_ = true;  ///< Flag indicating if the first frame has been processed.
        int             count_;                    ///< Counter for the number of processed IMU data frames.
        Eigen::Vector3d mean_gyro_;                ///< Mean of gyroscope data.
        Eigen::Vector3d mean_acce_;                ///< Mean of accelerometer data.

        std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> init_imu_buffer_;  ///< Buffer for IMU data during initialization.
        double                                            init_start_time_;  ///< Timestamp of when initialization started.
    };
}  // namespace localization
