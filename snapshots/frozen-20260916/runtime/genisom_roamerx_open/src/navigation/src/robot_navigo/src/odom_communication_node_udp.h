#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <highlevel_connector.h>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>

class OdomCommunicationNodeUdp : public rclcpp::Node
{
public:
    OdomCommunicationNodeUdp();

    ~OdomCommunicationNodeUdp();

private:
    void publish_odometry(const navigo_sdk::highLevelState* state_msg);

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void odom_callback(const nav_msgs::msg::Odometry& msg);

    void HandlPlannerVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

private:
    double update_rate_;

    std::thread                  udp_process_thread_;
    navigo_sdk::HighLevelConnector connector_;
    navigo_sdk::highLevelCmd       cmd_;
    std::mutex                   lk_;

    std::shared_ptr<tf2_ros::TransformBroadcaster>             tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      odom_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr   odom_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr planner_vel_cmd_subscriber;

    std::chrono::time_point<std::chrono::system_clock> last_pub_time_;
};
