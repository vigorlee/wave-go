#pragma once

#include "state_estimator_lcmt.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lcm/lcm-cpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>

class OdomCommunicationNode : public rclcpp::Node
{
public:
    OdomCommunicationNode();

    ~OdomCommunicationNode();

private:
    void publish_odometry();

    void handler_interface_lcm();

    void odom_lcm_receive_handler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const state_estimator_lcmt* msg);

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
    double update_rate_;

    std::thread odom_lcm_handler_;

    state_estimator_lcmt odom_lcm_receive_msg_;

    lcm::LCM lc_;
    // std::mutex planner_vel_mutex_;

    std::shared_ptr<tf2_ros::TransformBroadcaster>           tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    odom_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
};