#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cmath>
#include <functional>

class OdomPublisherNode : public rclcpp::Node
{
public:
    OdomPublisherNode()
        : Node("odom_communication_node"),
          x_(0.0),
          y_(0.0),
          theta_(0.0),
          vx_(0.1),
          vy_(0.0),
          vtheta_(0.05),
          update_rate_(10.0)
    {
        // 创建发布器
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom/current_pose", 10);

        // 创建定时器，周期性发布消息
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)), std::bind(&OdomPublisherNode::publish_odometry, this));
    }

private:
    void publish_odometry()
    {
        // 计算时间间隔 dt
        double dt = 1.0 / update_rate_;

        // 更新位置（假设线速度 vx_，vy_ 和角速度 vtheta_ 是常量）
        x_ += vx_ * dt;
        y_ += vy_ * dt;
        theta_ += vtheta_ * dt;

        // 计算四元数
        geometry_msgs::msg::Quaternion odom_quat;
        odom_quat.w = std::cos(theta_ / 2.0);
        odom_quat.z = std::sin(theta_ / 2.0);

        // 创建并填充 Odometry 消息
        auto odom_msg                  = nav_msgs::msg::Odometry();
        odom_msg.header.stamp          = this->get_clock()->now();
        odom_msg.header.frame_id       = "odom";  // 参考坐标系
        odom_msg.pose.pose.position.x  = x_;
        odom_msg.pose.pose.position.y  = y_;
        odom_msg.pose.pose.position.z  = 0.0;
        odom_msg.pose.pose.orientation = odom_quat;

        // 基础坐标系为 base_link
        odom_msg.child_frame_id = "base_link";

        // 填充线速度和角速度
        odom_msg.twist.twist.linear.x  = vx_;
        odom_msg.twist.twist.linear.y  = vy_;
        odom_msg.twist.twist.angular.z = vtheta_;

        // 发布消息
        odom_pub_->publish(odom_msg);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;  // Odometry 发布器
    rclcpp::TimerBase::SharedPtr                          timer_;     // 定时器

    // 用于模拟的参数
    double x_, y_, theta_;
    double vx_, vy_, vtheta_;
    double update_rate_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);                             // 初始化 ROS 2
    rclcpp::spin(std::make_shared<OdomPublisherNode>());  // 启动节点并处理回调
    rclcpp::shutdown();                                   // 关闭 ROS 2
    return 0;
}
