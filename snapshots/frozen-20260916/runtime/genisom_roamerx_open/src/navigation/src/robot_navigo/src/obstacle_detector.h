#ifndef OBSTACLE_DETECTOR_HPP
#define OBSTACLE_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "tf2_ros/transform_listener.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2/transform_datatypes.h"

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector();

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void process_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void publish_markers(const std::vector<Eigen::Vector2d>& segments);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif  // OBSTACLE_DETECTOR_HPP
