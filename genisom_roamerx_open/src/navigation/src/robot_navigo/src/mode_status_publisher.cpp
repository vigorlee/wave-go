#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class ModeStatusPublisher : public rclcpp::Node
{
public:
    ModeStatusPublisher() : Node("mode_status_publisher"), current_mode_(170), received_first_cmd_vel_(false)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/mode_switch_cmd", 10);

        timer_ = this->create_wall_timer(1s, std::bind(&ModeStatusPublisher::PublishStatus, this));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&ModeStatusPublisher::cmdVelCallback, this, std::placeholders::_1));

        last_cmd_vel_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "ModeStatusPublisher initialized");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr /*msg*/)
    {
        last_cmd_vel_time_ = this->now();

        if (!received_first_cmd_vel_)
        {
            received_first_cmd_vel_ = true;
            RCLCPP_INFO(this->get_logger(), "First /cmd_vel received, ready to switch to NAVIGATION when active");
        }

        if (current_mode_ != 171)
        {
            current_mode_ = 171;
            RCLCPP_INFO(this->get_logger(), "Switching to NAVIGATION mode (171)");
            publishCurrentMode();
        }
    }

    void PublishStatus()
    {
        auto now = this->now();
        auto elapsed = now - last_cmd_vel_time_;

        int new_mode = current_mode_;

        if (received_first_cmd_vel_)
        {
            if (elapsed < rclcpp::Duration(1s))
            {
                new_mode = 171;
            }
            else
            {
                new_mode = 170;
            }
        }
        else
        {
            new_mode = 170;
        }

        if (new_mode != current_mode_)
        {
            current_mode_ = new_mode;
            if (current_mode_ == 171)
            {
                RCLCPP_INFO(this->get_logger(), "Switching to NAVIGATION mode (171)");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Swtiching to CONTROL mode (170)");
            }
        }

        publishCurrentMode();
    }

    void publishCurrentMode()
    {
        auto msg = std_msgs::msg::Int32();
        msg.data = current_mode_;
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int current_mode_;
    bool received_first_cmd_vel_;
    rclcpp::Time last_cmd_vel_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ModeStatusPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
