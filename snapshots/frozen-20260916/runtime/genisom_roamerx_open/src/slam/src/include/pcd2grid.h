/**
 * @brief
 */

#pragma once
#include "common.h"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>
#include <fstream>

namespace robot::slam
{
    struct Pcd2GridOptions
    {
        std::string file_name = "map";
        double thre_z_min = 0.3;
        double thre_z_max = 2.0;
        int flag_pass_through = 0;
        double map_resolution = 0.05;
        double thre_radius = 0.1;
        int thres_point_count = 10;
    };

    class Pcd2Grid
    {
    public:
        explicit Pcd2Grid(const Pcd2GridOptions &options);
        ~Pcd2Grid() {}

        void run(const CloudPtr &map_points, const std::string &file_name);

    private:
        Pcd2GridOptions options_;

        void PassThroughFilter(const CloudPtr &pcd_cloud, CloudPtr &cloud_after_pass_through);

        void RadiusOutlierFilter(const CloudPtr &pcd_cloud, CloudPtr &cloud_after_radius);

        void SetMapTopicMsg(const CloudPtr cloud, nav_msgs::msg::OccupancyGrid &msg);

        void SavePGMAndYAML(const nav_msgs::msg::OccupancyGrid &msg, const std::string &name);
    };
}

using Pcd2Grid = robot::slam::Pcd2Grid;
