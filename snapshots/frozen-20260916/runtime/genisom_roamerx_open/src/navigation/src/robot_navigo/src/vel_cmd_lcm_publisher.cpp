#include "gamepad_lcmt.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <lcm/lcm-cpp.hpp>
#include <rclcpp/rclcpp.hpp>

class VelCmdLcmPublisher : public rclcpp::Node
{
public:
    VelCmdLcmPublisher()
        : Node("vel_cmd_lcm_publisher"),
          lc("udpm://239.255.76.67:7667?ttl=255")
    {
        planner_vel_cmd_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&VelCmdLcmPublisher::HandlPlannerVelCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "vel_cmd_lcm_publisher started");
    }

private:
    void HandlPlannerVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(planner_vel_mutex_);
        gamepad_lcmt                lcmt;
        lcmt.leftStickAnalog[1]  = std::fabs(msg->linear.x) < 0.085 ? 0.0 : msg->linear.x;
        lcmt.leftStickAnalog[0]  = std::fabs(msg->linear.y) < 0.085 ? 0.0 : msg->linear.y;
        lcmt.rightStickAnalog[0] = -msg->angular.z;

        lc.publish("vel_cmd_lcm_data", &lcmt);
    }

    lcm::LCM                                                   lc;
    std::mutex                                                 planner_vel_mutex_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr planner_vel_cmd_subscriber;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VelCmdLcmPublisher>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
