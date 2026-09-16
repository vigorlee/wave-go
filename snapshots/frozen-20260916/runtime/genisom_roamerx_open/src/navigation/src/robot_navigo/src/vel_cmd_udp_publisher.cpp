#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

#include "geometry_msgs/msg/twist.hpp"
#include "highlevel_connector.h"
#include "std_msgs/msg/int32.hpp"

class VelCmdUdpPublisher : public rclcpp::Node {
 public:
  VelCmdUdpPublisher() : Node("vel_cmd_udp_publisher") {
    planner_vel_cmd_subscriber =
        this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&VelCmdUdpPublisher::HandlePlannerVelCallback, this,
                      std::placeholders::_1));
    mode_switch_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        "/mode_switch_cmd", 10,
        std::bind(&VelCmdUdpPublisher::HandleModeSwitchCallback, this,
                  std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "vel_cmd_udp_publisher started");

    // Read platform params
    this->declare_parameter("platform",
                            rclcpp::ParameterValue(std::string("")));
    this->get_parameter("platform", platform_);

    // Platform reflection table
    const std::unordered_map<std::string, std::pair<std::string, std::string>>
        platform_map = {{"NX_XG3588", {"192.168.3.120", "192.168.3.100"}},
                        {"XG3588", {"127.0.0.1", "127.0.0.1"}},
                        {"UE", {"127.0.0.1", "127.0.0.1"}}};

    auto it = platform_map.find(platform_);
    if (it != platform_map.end()) {
      SERVER_IP = it->second.first;
      CLIENT_IP = it->second.second;
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "\033[1;31mError: Unknown PLATFORM type\033[0m");
      rclcpp::shutdown();
      return;
    }

    // Port configuration
    CLIENT_PORT = 43988;
    SERVER_PORT = 43997;

    // Connector initialization
    connector_.creat_connector(SERVER_IP, CLIENT_IP, SERVER_PORT, CLIENT_PORT);
    if (not connector_.reg_state_recive()) {
      RCLCPP_ERROR(this->get_logger(), "==> Init  HighLevelConnector error!");
      rclcpp::shutdown();
      return;
    }

    udp_process_thread_ = std::thread([&]() {
      while (rclcpp::ok()) {
        {
          std::lock_guard<std::mutex> lk(planner_vel_mutex_);
          // 收到第一条 cmd_vel 才开始抢控制权，避免启动时意外打断机器人状态
          if (nav_active_) {
            cmd_.len = sizeof(navigo_sdk::highLevelCmd);
            cmd_.head = (uint16_t)0x5AA5;
            cmd_.checksum =
                connector_.checksum(reinterpret_cast<const unsigned char*>(&cmd_));
            connector_.SendCmd(&cmd_);
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    });
  }

  ~VelCmdUdpPublisher() {
    if (udp_process_thread_.joinable()) {
      udp_process_thread_.join();
    }
  }

 private:
  void HandleModeSwitchCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(planner_vel_mutex_);
    // 171=导航激活, 170=遥控待机 — 两者都映射到 mc_ctrl K_RL_MIX(18)
    const uint16_t mc_mode = 18;
    if (mc_mode == cmd_.control_mode && cmd_.enable_control_mode == 1) {
      return;
    }
    cmd_.control_mode = mc_mode;
    cmd_.enable_control_mode = 1;
    RCLCPP_INFO(this->get_logger(),
                "mode_switch_cmd=%d → mc_ctrl control_mode=18 (K_RL_MIX)",
                msg->data);
  }

  void HandlePlannerVelCallback(
      const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(planner_vel_mutex_);

    cmd_.vx = std::fabs(msg->linear.x) < 0.085 ? 0.0 : msg->linear.x;
    cmd_.vy = std::fabs(msg->linear.y) < 0.085 ? 0.0 : msg->linear.y;
    cmd_.yaw_rate = msg->angular.z;
    // 确保始终使用 mc_ctrl 识别的行走模式 K_RL_MIX(18)
    cmd_.control_mode = 18;
    cmd_.enable_control_mode = 1;
    nav_active_ = true;
  }

 private:
  int CLIENT_PORT;
  int SERVER_PORT;
  std::string SERVER_IP;
  std::string CLIENT_IP;
  std::string platform_;
  std::mutex planner_vel_mutex_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      planner_vel_cmd_subscriber;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_switch_subscriber_;

  navigo_sdk::HighLevelConnector connector_;
  navigo_sdk::highLevelCmd cmd_;
  bool nav_active_ = false;

  std::thread udp_process_thread_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<VelCmdUdpPublisher>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
