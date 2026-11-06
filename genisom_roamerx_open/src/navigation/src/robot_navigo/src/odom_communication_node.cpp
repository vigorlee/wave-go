#include "odom_communication_node.h"

#include <cmath>

OdomCommunicationNode::OdomCommunicationNode()
    : Node("odom_communication_node"),
      lc_("udpm://239.255.76.67:7667?ttl=255")
{
    if (!lc_.good())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize LCM instance.");
        rclcpp::shutdown();
        return;
    }

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    odom_pub_          = this->create_publisher<nav_msgs::msg::Odometry>("/odom/current_pose", 10);
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/current_pose", 10, std::bind(&OdomCommunicationNode::odom_callback, this, std::placeholders::_1));
    odom_lcm_handler_ = std::thread(&OdomCommunicationNode::handler_interface_lcm, this);

    lc_.subscribe("state_estimator", &OdomCommunicationNode::odom_lcm_receive_handler, this);
}

OdomCommunicationNode::~OdomCommunicationNode()
{
    if (odom_lcm_handler_.joinable())
    {
        odom_lcm_handler_.join();
    }
}

void OdomCommunicationNode::handler_interface_lcm()
{
    while (rclcpp::ok())
    {
        lc_.handle();
    }
}

void OdomCommunicationNode::odom_lcm_receive_handler(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan, const state_estimator_lcmt* msg)
{
    (void)rbuf;
    (void)chan;
    // std::lock_guard<std::mutex> lock(planner_vel_mutex_);
    odom_lcm_receive_msg_ = *msg;
    publish_odometry();
}

void OdomCommunicationNode::publish_odometry()
{
    geometry_msgs::msg::Quaternion odom_quat;
    odom_quat.w = odom_lcm_receive_msg_.quat[0];
    odom_quat.x = odom_lcm_receive_msg_.quat[1];
    odom_quat.y = odom_lcm_receive_msg_.quat[2];
    odom_quat.z = odom_lcm_receive_msg_.quat[3];

    auto odom_msg            = nav_msgs::msg::Odometry();
    odom_msg.header.stamp    = now();
    odom_msg.header.frame_id = "odom";

    odom_msg.pose.pose.position.x  = odom_lcm_receive_msg_.p[0];
    odom_msg.pose.pose.position.y  = odom_lcm_receive_msg_.p[1];
    odom_msg.pose.pose.position.z  = odom_lcm_receive_msg_.p[2];
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.child_frame_id        = "base_link";
    odom_msg.twist.twist.linear.x  = odom_lcm_receive_msg_.vWorld[0];
    odom_msg.twist.twist.linear.y  = odom_lcm_receive_msg_.vWorld[1];
    odom_msg.twist.twist.angular.z = odom_lcm_receive_msg_.omegaWorld[2] / 57.3;

    odom_pub_->publish(odom_msg);
}

void OdomCommunicationNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped map_odom;

    map_odom.header.stamp = msg->header.stamp;

    map_odom.header.frame_id = "map";
    map_odom.child_frame_id  = "odom";

    map_odom.transform.translation.x = 0.0;
    map_odom.transform.translation.y = 0.0;
    map_odom.transform.translation.z = 0.0;

    map_odom.transform.rotation.x = 0.0;
    map_odom.transform.rotation.y = 0.0;
    map_odom.transform.rotation.z = 0.0;
    map_odom.transform.rotation.w = 1.0;

    geometry_msgs::msg::TransformStamped odom_baselink;

    odom_baselink.header.stamp = msg->header.stamp;

    odom_baselink.header.frame_id = "odom";
    odom_baselink.child_frame_id  = "base_link";

    odom_baselink.transform.translation.x = msg->pose.pose.position.x;
    odom_baselink.transform.translation.y = msg->pose.pose.position.y;
    odom_baselink.transform.translation.z = msg->pose.pose.position.z;

    odom_baselink.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(odom_baselink);
    tf_broadcaster_->sendTransform(map_odom);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomCommunicationNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}