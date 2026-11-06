#include "obstacle_detector.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <visualization_msgs/msg/marker.hpp>
#include "tf2_ros/create_timer_ros.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

ObstacleDetector::ObstacleDetector() : Node("obstacle_detector")
{
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ObstacleDetector::scan_callback, this, std::placeholders::_1));

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle_lines", 10);

  // 初始化 TF 监听器
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ObstacleDetector::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // 1. 将 /scan 转换为 PCL 点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    double angle = msg->angle_min + i * msg->angle_increment;
    double r = msg->ranges[i];

    if (r >= msg->range_min && r <= msg->range_max)
    {
      cloud->points.emplace_back(r * cos(angle), r * sin(angle), 0.0);
    }
  }

  // 2. 处理点云 -> 拟合直线
  process_point_cloud(cloud);
}

void ObstacleDetector::process_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  std::vector<Eigen::Vector2d> segments;

  if (cloud->points.empty())
    return;

  for (size_t i = 0; i < cloud->points.size() - 1; ++i)
  {
    Eigen::Vector2d p1(cloud->points[i].x, cloud->points[i].y);
    Eigen::Vector2d p2(cloud->points[i + 1].x, cloud->points[i + 1].y);

    if ((p2 - p1).norm() < 0.5)  // 设定距离阈值，避免错误连接
    {
      segments.push_back(p1);
      segments.push_back(p2);
    }
  }

  publish_markers(segments);
}

void ObstacleDetector::publish_markers(const std::vector<Eigen::Vector2d>& segments)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker line_list;

  line_list.header.frame_id = "map";
  line_list.header.stamp = this->now();
  line_list.ns = "obstacles";
  line_list.id = 0;
  line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
  line_list.scale.x = 0.05;
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;

  // 获取 base_link 到 map 的 TF 变换
  geometry_msgs::msg::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_WARN(this->get_logger(), "Could not transform base_link to map: %s", ex.what());
    return;
  }

  for (const auto& pt : segments)
  {
    geometry_msgs::msg::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = 0.0;

    tf2::doTransform(p, p, transform_stamped);

    line_list.points.push_back(p);
  }

  marker_array.markers.push_back(line_list);

  marker_pub_->publish(marker_array);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDetector>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
