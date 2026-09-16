#ifndef GLOBAL_LOCALIZATION_HPP
#define GLOBAL_LOCALIZATION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace localization {

    class GlobalLocalization {
        public:
            GlobalLocalization();
            ~GlobalLocalization();
        
            void init(const Eigen::Matrix4d& initial_pose);
        
            bool performGlobalLocalization(
                const pcl::PointCloud<pcl::PointXYZI>::Ptr& global_map,
                const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_cloud,
                const Eigen::Matrix4d& initial_trans,  
                Eigen::Matrix4d& final_pose
            );
        
        private:
            Eigen::Matrix4d initial_pose_; 
            int init_check_count_;        
            Eigen::Vector3d last_position_; 
            rclcpp::Logger logger_ = rclcpp::get_logger("global_localization");
        };

} // namespace localization

#endif // GLOBAL_LOCALIZATION_HPP