#include "odom_communication_node_udp.h"

#include <chrono>
#include <cmath>
#include <common.h>
#include <cstdlib>
#include <thread>

#ifdef FOR_3588
constexpr int     CLIENT_PORT = 43909;        // local port
constexpr int     SERVER_PORT = 43998;        // target port
const std::string SERVER_IP   = "127.0.0.1";  // target IP address
const std::string CLIENT_IP   = "127.0.0.1";  // local IP address
#else
constexpr int     CLIENT_PORT = 43909;            // local port
constexpr int     SERVER_PORT = 43998;            // target port
const std::string SERVER_IP   = "192.168.1.120";  // target IP address
const std::string CLIENT_IP   = "192.168.1.100";  // local IP address
#endif


OdomCommunicationNodeUdp::OdomCommunicationNodeUdp()
    : Node("odom_communication_node")
{
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    odom_pub_       = this->create_publisher<nav_msgs::msg::Odometry>("/odom/current_pose", 10);

    last_pub_time_ = std::chrono::high_resolution_clock::now();
    // 订阅自己的 odom，然后发出
    // odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "/odom/current_pose", 10, std::bind(&OdomCommunicationNodeUdp::odom_callback, this, std::placeholders::_1));

    // 接受规划速度，向 mc 发送实际速度指令
    planner_vel_cmd_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&OdomCommunicationNodeUdp::HandlPlannerVelCallback, this, std::placeholders::_1));

    connector_.creat_connector(SERVER_IP, CLIENT_IP, SERVER_PORT, CLIENT_PORT);
    if (not connector_.reg_state_recive())
    {
        RCLCPP_ERROR(this->get_logger(), "==> Init  HighLevelConnector error!");
        rclcpp::shutdown();
        return;
    }

    udp_process_thread_ = std::thread(
        [&]()
        {
            while (rclcpp::ok())
            {
                {
                    std::unique_lock lk(lk_);
                    cmd_.len      = sizeof(navigo_sdk::highLevelCmd);
                    cmd_.head     = static_cast<uint16_t>(0x5AA5);
                    cmd_.checksum = connector_.checksum(reinterpret_cast<const unsigned char*>(&cmd_));
                    connector_.SendCmd(&cmd_);
                }

                auto state = connector_.GetState();
                if (state->head == 0x5AA5)
                {
                    publish_odometry(state);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "==> state error: 收到无效的头部值 0x%X，预期值为 0x5AA5", state->head);
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

    RCLCPP_INFO(this->get_logger(), "odom started");
}

void OdomCommunicationNodeUdp::HandlPlannerVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::unique_lock lk(lk_);
    cmd_.vx       = std::fabs(msg->linear.x) < 0.085 ? 0.0 : msg->linear.x;
    cmd_.vy       = std::fabs(msg->linear.y) < 0.085 ? 0.0 : msg->linear.y;
    cmd_.yaw_rate = msg->angular.z;

    RCLCPP_INFO(this->get_logger(), "==> 发送了: %f", cmd_.vx);
}

OdomCommunicationNodeUdp::~OdomCommunicationNodeUdp()
{
    if (udp_process_thread_.joinable())
    {
        udp_process_thread_.join();
    }
}

void OdomCommunicationNodeUdp::publish_odometry(const navigo_sdk::highLevelState* state_msg)
{
    if (std::chrono::high_resolution_clock::now() - last_pub_time_ < std::chrono::milliseconds(20))
    {
        return;
    }

    geometry_msgs::msg::Quaternion odom_quat;
    odom_quat.w = state_msg->quat[0];
    odom_quat.x = state_msg->quat[1];
    odom_quat.y = state_msg->quat[2];
    odom_quat.z = state_msg->quat[3];

    auto odom_msg            = nav_msgs::msg::Odometry();
    odom_msg.header.stamp    = now();
    odom_msg.header.frame_id = "odom";

    odom_msg.pose.pose.position.x  = state_msg->position[0];
    odom_msg.pose.pose.position.y  = state_msg->position[1];
    odom_msg.pose.pose.position.z  = state_msg->position[2];
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.child_frame_id        = "base_link";
    odom_msg.twist.twist.linear.x  = state_msg->vWorld[0];
    odom_msg.twist.twist.linear.y  = state_msg->vWorld[1];
    odom_msg.twist.twist.angular.z = state_msg->omegaWorld[2];

    odom_pub_->publish(odom_msg);
    odom_callback(odom_msg);

    last_pub_time_ = std::chrono::high_resolution_clock::now();
}

void OdomCommunicationNodeUdp::odom_callback(const nav_msgs::msg::Odometry& msg)
{
    geometry_msgs::msg::TransformStamped map_odom;

    map_odom.header.stamp = msg.header.stamp;

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

    odom_baselink.header.stamp = msg.header.stamp;

    odom_baselink.header.frame_id = "odom";
    odom_baselink.child_frame_id  = "base_link";

    odom_baselink.transform.translation.x = msg.pose.pose.position.x;
    odom_baselink.transform.translation.y = msg.pose.pose.position.y;
    odom_baselink.transform.translation.z = msg.pose.pose.position.z;

    odom_baselink.transform.rotation = msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(odom_baselink);
    tf_broadcaster_->sendTransform(map_odom);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomCommunicationNodeUdp>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
