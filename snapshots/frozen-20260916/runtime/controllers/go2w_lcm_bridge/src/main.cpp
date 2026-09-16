#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <lcm/lcm-cpp.hpp>
#include <robot_sdk.pb.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "gamepad_lcmt.hpp"

namespace {

using Clock = std::chrono::steady_clock;

std::atomic<bool> running{true};

double env_double(const char* name, double fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  return end != value && *end == '\0' ? parsed : fallback;
}

void stop_handler(int) { running.store(false); }

class VelocityReceiver {
 public:
  void handle(const lcm::ReceiveBuffer*, const std::string&, const gamepad_lcmt* msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    linear_x_ = msg->leftStickAnalog[1];
    linear_y_ = msg->leftStickAnalog[0];
    angular_z_ = -msg->rightStickAnalog[0];
    updated_at_ = Clock::now();
    received_ = true;
  }

  std::array<double, 3> command(std::chrono::milliseconds timeout) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!received_ || Clock::now() - updated_at_ > timeout) {
      return {0.0, 0.0, 0.0};
    }
    return {linear_x_, linear_y_, angular_z_};
  }

 private:
  mutable std::mutex mutex_;
  double linear_x_{0.0};
  double linear_y_{0.0};
  double angular_z_{0.0};
  Clock::time_point updated_at_{};
  bool received_{false};
};

struct RobotSnapshot {
  bool valid{false};
  Clock::time_point updated_at{};
  std::array<double, 4> abad{};
  std::array<double, 4> hip{};
  std::array<double, 4> knee{};
  std::array<double, 4> foot{};
  std::array<double, 3> position{};
  std::array<double, 3> rpy{};
};

class StateReceiver {
 public:
  void handle(const robot_sdk::pb::RobotState& msg) {
    if (msg.q_abad_size() < 4 || msg.q_hip_size() < 4 || msg.q_knee_size() < 4 ||
        msg.q_foot_size() < 4) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (int leg = 0; leg < 4; ++leg) {
      snapshot_.abad[leg] = msg.q_abad(leg);
      snapshot_.hip[leg] = msg.q_hip(leg);
      snapshot_.knee[leg] = msg.q_knee(leg);
      snapshot_.foot[leg] = msg.q_foot(leg);
    }
    if (msg.position_size() >= 3) {
      for (int axis = 0; axis < 3; ++axis) {
        snapshot_.position[axis] = msg.position(axis);
      }
    }
    if (msg.rpy_size() >= 3) {
      for (int axis = 0; axis < 3; ++axis) {
        snapshot_.rpy[axis] = msg.rpy(axis);
      }
    }
    if (msg.quat_size() >= 4) {
      const double w = msg.quat(0);
      const double x = msg.quat(1);
      const double y = msg.quat(2);
      const double z = msg.quat(3);
      const double sinr_cosp = 2.0 * (w * x + y * z);
      const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
      snapshot_.rpy[0] = std::atan2(sinr_cosp, cosr_cosp);
      const double sinp = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
      snapshot_.rpy[1] = std::asin(sinp);
      const double siny_cosp = 2.0 * (w * z + x * y);
      const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
      snapshot_.rpy[2] = std::atan2(siny_cosp, cosy_cosp);
    }
    snapshot_.updated_at = Clock::now();
    snapshot_.valid = true;
  }

  RobotSnapshot snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

 private:
  mutable std::mutex mutex_;
  RobotSnapshot snapshot_;
};

void add_four(google::protobuf::RepeatedField<float>* field, float value) {
  for (int i = 0; i < 4; ++i) {
    field->Add(value);
  }
}

robot_sdk::pb::RobotCmd make_standing_command(double kp_abad, double kp_hip, double kp_knee,
                                               double kd_abad, double kd_hip, double kd_knee,
                                               double kd_foot) {
  robot_sdk::pb::RobotCmd cmd;

  add_four(cmd.mutable_q_des_abad(), 0.0F);
  add_four(cmd.mutable_q_des_hip(), 0.0F);
  add_four(cmd.mutable_q_des_knee(), -0.9F);
  add_four(cmd.mutable_q_des_foot(), 0.0F);

  add_four(cmd.mutable_qd_des_abad(), 0.0F);
  add_four(cmd.mutable_qd_des_hip(), 0.0F);
  add_four(cmd.mutable_qd_des_knee(), 0.0F);
  add_four(cmd.mutable_qd_des_foot(), 0.0F);

  add_four(cmd.mutable_kp_abad(), static_cast<float>(kp_abad));
  add_four(cmd.mutable_kp_hip(), static_cast<float>(kp_hip));
  add_four(cmd.mutable_kp_knee(), static_cast<float>(kp_knee));
  add_four(cmd.mutable_kp_foot(), 0.0F);

  add_four(cmd.mutable_kd_abad(), static_cast<float>(kd_abad));
  add_four(cmd.mutable_kd_hip(), static_cast<float>(kd_hip));
  add_four(cmd.mutable_kd_knee(), static_cast<float>(kd_knee));
  add_four(cmd.mutable_kd_foot(), static_cast<float>(kd_foot));

  add_four(cmd.mutable_tau_abad_ff(), 0.0F);
  add_four(cmd.mutable_tau_hip_ff(), 0.0F);
  add_four(cmd.mutable_tau_knee_ff(), 0.0F);
  add_four(cmd.mutable_tau_foot_ff(), 0.0F);

  return cmd;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, stop_handler);
  std::signal(SIGTERM, stop_handler);

  const std::string lcm_url = "udpm://239.255.76.67:7667?ttl=255";
  const std::string lcm_channel = "vel_cmd_lcm_data";
  const double wheel_radius = env_double("GO2W_WHEEL_RADIUS", 0.086);
  const double half_track = env_double("GO2W_HALF_TRACK", 0.193);
  const double wheel_sign = env_double("GO2W_WHEEL_SIGN", -1.0);
  const double max_linear = env_double("GO2W_MAX_LINEAR", 0.45);
  const double max_angular = env_double("GO2W_MAX_ANGULAR", 0.8);
  const double stand_abad = env_double("GO2W_STAND_ABAD", 0.0);
  const double stand_hip = env_double("GO2W_STAND_HIP", 0.8);
  const double stand_knee = env_double("GO2W_STAND_KNEE", -1.5);
  const double stand_ramp_seconds = std::max(0.5, env_double("GO2W_STAND_RAMP_SECONDS", 3.0));
  const double kp_abad = env_double("GO2W_KP_ABAD", 50.0);
  const double kp_hip = env_double("GO2W_KP_HIP", 60.0);
  const double kp_knee = env_double("GO2W_KP_KNEE", 60.0);
  const double kd_abad = env_double("GO2W_KD_ABAD", 1.5);
  const double kd_hip = env_double("GO2W_KD_HIP", 2.0);
  const double kd_knee = env_double("GO2W_KD_KNEE", 2.0);
  const double kd_foot = env_double("GO2W_KD_FOOT", 1.0);
  const auto command_timeout = std::chrono::milliseconds(300);
  const auto state_timeout = std::chrono::milliseconds(250);

  lcm::LCM lcm(lcm_url);
  if (!lcm.good()) {
    std::cerr << "Failed to initialize LCM at " << lcm_url << '\n';
    return 1;
  }

  VelocityReceiver receiver;
  lcm.subscribe(lcm_channel, &VelocityReceiver::handle, &receiver);
  std::thread lcm_thread([&lcm]() {
    while (running.load()) {
      lcm.handleTimeout(100);
    }
  });

  eCAL::Initialize(argc, argv, "go2w_lcm_bridge");
  eCAL::protobuf::CPublisher<robot_sdk::pb::RobotCmd> publisher("mujoco_cmd");
  eCAL::protobuf::CSubscriber<robot_sdk::pb::RobotState> state_subscriber("mujoco_state");
  StateReceiver state_receiver;
  state_subscriber.AddReceiveCallback(
      [&state_receiver](const char*, const robot_sdk::pb::RobotState& state, long long, long long,
                        long long) { state_receiver.handle(state); });
  robot_sdk::pb::RobotCmd command =
      make_standing_command(kp_abad, kp_hip, kp_knee, kd_abad, kd_hip, kd_knee, kd_foot);

  std::cout << "LCM " << lcm_channel << " -> eCAL mujoco_cmd"
            << " (radius=" << wheel_radius << ", half_track=" << half_track
            << ", wheel_sign=" << wheel_sign << ")\n"
            << "Stand target: abad=" << stand_abad << ", hip=" << stand_hip
            << ", knee=" << stand_knee << ", ramp=" << stand_ramp_seconds << "s\n";

  auto next_tick = Clock::now();
  auto next_report = Clock::now();
  Clock::time_point stand_started_at{};
  std::array<double, 4> initial_abad{};
  std::array<double, 4> initial_hip{};
  std::array<double, 4> initial_knee{};
  bool stand_initialized = false;
  while (running.load() && eCAL::Ok()) {
    const auto now = Clock::now();
    const RobotSnapshot state = state_receiver.snapshot();
    const bool state_fresh = state.valid && now - state.updated_at <= state_timeout;

    if (state_fresh && !stand_initialized) {
      initial_abad = state.abad;
      initial_hip = state.hip;
      initial_knee = state.knee;
      stand_started_at = now;
      stand_initialized = true;
      std::cout << "State acquired; starting smooth stand transition.\n";
    }

    double stand_alpha = 0.0;
    if (stand_initialized) {
      stand_alpha = std::clamp(
          std::chrono::duration<double>(now - stand_started_at).count() / stand_ramp_seconds, 0.0,
          1.0);
      stand_alpha = stand_alpha * stand_alpha * (3.0 - 2.0 * stand_alpha);
      for (int leg = 0; leg < 4; ++leg) {
        command.set_q_des_abad(
            leg, static_cast<float>(initial_abad[leg] + stand_alpha * (stand_abad - initial_abad[leg])));
        command.set_q_des_hip(
            leg, static_cast<float>(initial_hip[leg] + stand_alpha * (stand_hip - initial_hip[leg])));
        command.set_q_des_knee(
            leg, static_cast<float>(initial_knee[leg] + stand_alpha * (stand_knee - initial_knee[leg])));
      }
    }

    const auto velocity = receiver.command(command_timeout);
    const bool upright = state_fresh && state.position[2] > 0.2 &&
                         std::abs(state.rpy[0]) < 0.6 && std::abs(state.rpy[1]) < 0.6;
    const bool drive_enabled = stand_initialized && stand_alpha > 0.999 && upright;
    const double linear =
        drive_enabled ? std::clamp(velocity[0], -max_linear, max_linear) : 0.0;
    const double angular =
        drive_enabled ? std::clamp(velocity[2], -max_angular, max_angular) : 0.0;

    const double right = wheel_sign * (linear + half_track * angular) / wheel_radius;
    const double left = wheel_sign * (linear - half_track * angular) / wheel_radius;
    // Go2-W's physical joint order is FL, FR, RL, RR.  The XGW-compatible
    // MuJoCo path operates on slot indices, so map left/right explicitly.
    command.set_qd_des_foot(0, static_cast<float>(left));   // FL
    command.set_qd_des_foot(1, static_cast<float>(right));  // FR
    command.set_qd_des_foot(2, static_cast<float>(left));   // RL
    command.set_qd_des_foot(3, static_cast<float>(right));  // RR

    publisher.Send(command);
    if (state_fresh && now >= next_report) {
      std::cout << std::fixed << std::setprecision(3) << "state z=" << state.position[2]
                << " rpy=[" << state.rpy[0] << ',' << state.rpy[1] << ',' << state.rpy[2]
                << "] hip[FL,FR,RL,RR]=[" << state.hip[0] << ',' << state.hip[1] << ',' << state.hip[2]
                << ',' << state.hip[3] << "] knee=[" << state.knee[0] << ',' << state.knee[1]
                << ',' << state.knee[2] << ',' << state.knee[3] << "] stand=" << stand_alpha
                << " drive=" << drive_enabled << '\n';
      next_report = now + std::chrono::seconds(1);
    }
    next_tick += std::chrono::milliseconds(2);
    std::this_thread::sleep_until(next_tick);
  }

  for (int i = 0; i < 100; ++i) {
    for (int leg = 0; leg < 4; ++leg) {
      command.set_qd_des_foot(leg, 0.0F);
    }
    publisher.Send(command);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  running.store(false);
  if (lcm_thread.joinable()) {
    lcm_thread.join();
  }
  eCAL::Finalize();
  return 0;
}
