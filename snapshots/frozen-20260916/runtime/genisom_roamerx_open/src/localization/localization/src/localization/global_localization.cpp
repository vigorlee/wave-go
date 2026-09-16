#include "localization/global_localization.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

namespace localization {

    GlobalLocalization::GlobalLocalization()
    : init_check_count_(0), last_position_(Eigen::Vector3d::Zero()) {}

GlobalLocalization::~GlobalLocalization() {}

void GlobalLocalization::init(const Eigen::Matrix4d& initial_pose) {
    initial_pose_ = initial_pose;
}

bool GlobalLocalization::performGlobalLocalization(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& global_map,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_cloud,
    const Eigen::Matrix4d& initial_trans,  
    Eigen::Matrix4d& final_pose
) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_ds(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(current_cloud);
    voxel_filter.setLeafSize(0.2f, 0.2f, 0.2f); 
    voxel_filter.filter(*current_cloud_filtered);

    pcl::transformPointCloud(*current_cloud_filtered, *current_cloud_ds, initial_trans.cast<float>());

    // ICP 
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputTarget(global_map);
    icp.setInputSource(current_cloud_ds);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);

    pcl::PointCloud<pcl::PointXYZI> aligned_cloud;
    Eigen::Matrix4f T_corr_current;
    double fitness_score;
    icp.align(aligned_cloud);
    T_corr_current = icp.getFinalTransformation();
    fitness_score = icp.getFitnessScore();
    RCLCPP_INFO(logger_, "ICP fitness score 1: %.6f", fitness_score);

    pcl::transformPointCloud(*current_cloud_ds, *current_cloud_ds, T_corr_current);


    if (fitness_score > 0.25) {
        return false; 
    }

    // 第二次 ICP 
    icp.align(aligned_cloud);
    Eigen::Matrix4f T_corr_second = icp.getFinalTransformation();
    fitness_score = icp.getFitnessScore();
    RCLCPP_INFO(logger_, "ICP fitness score 2: %.6f", fitness_score);
    pcl::transformPointCloud(*current_cloud_ds, *current_cloud_ds, T_corr_second);

    Eigen::Matrix4d T_corr_final = T_corr_second.cast<double>() * T_corr_current.cast<double>();

    Eigen::Vector3d current_position = T_corr_final.block<3, 1>(0, 3);
    if (init_check_count_ > 0) {
        Eigen::Vector3d position_diff = current_position - last_position_;
        if (position_diff.norm() < 0.1) {
            init_check_count_++;
        } else {
            init_check_count_ = 0; 
        }
    } else {
        init_check_count_ = 1; 
    }

    last_position_ = current_position;

    if (init_check_count_ >= 2) {
        final_pose = T_corr_final; 
        return true;  
    }

    return false; 
}
} // namespace localization
