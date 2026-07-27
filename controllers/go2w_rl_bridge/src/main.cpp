#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <lcm/lcm-cpp.hpp>
#include <robot_sdk.pb.h>
#include <torch/script.h>
#include <torch/torch.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gamepad_lcmt.hpp"

namespace {

using Clock = std::chrono::steady_clock;
constexpr int kLegCount = 4;
constexpr int kActionCount = 16;
constexpr int kObservationCount = 73;
constexpr int kHistoryFrames = 5;
constexpr int kHistoryCount = kObservationCount * kHistoryFrames;

constexpr std::array<float, kActionCount> kDefaultJointPosition = {
    0.0F, 0.8F, -1.5F, 0.0F, 0.0F, 0.8F, -1.5F, 0.0F,
    0.0F, 0.8F, -1.5F, 0.0F, 0.0F, 0.8F, -1.5F, 0.0F};

std::atomic<bool> running{true};

void stop_handler(int) { running.store(false); }

double env_double(const char* name, double fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  return end != value && *end == '\0' ? parsed : fallback;
}

bool env_bool(const char* name, bool fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  const std::string text(value);
  return text == "1" || text == "true" || text == "TRUE" || text == "yes" ||
         text == "on";
}

std::string env_string(const char* name, const std::string& fallback) {
  const char* value = std::getenv(name);
  return value == nullptr || *value == '\0' ? fallback : std::string(value);
}

std::string read_mode_file(const std::string& path, const std::string& fallback) {
  if (path.empty()) {
    return fallback;
  }
  std::ifstream input(path);
  std::string mode;
  if (!input || !(input >> mode)) {
    return fallback;
  }
  std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char value) {
    return static_cast<char>(std::tolower(value));
  });
  if (mode != "avoid" && mode != "up" && mode != "down" && mode != "flat") {
    return fallback;
  }
  return mode;
}

std::string read_posture_file(const std::string& path) {
  if (path.empty()) {
    return "stand";
  }
  std::ifstream input(path);
  std::string posture;
  if (!input || !(input >> posture)) {
    return "stand";
  }
  std::transform(posture.begin(), posture.end(), posture.begin(), [](unsigned char value) {
    return static_cast<char>(std::tolower(value));
  });
  return posture == "charge" || posture == "recover" ? posture : "stand";
}

class VelocityReceiver {
 public:
  void handle(const lcm::ReceiveBuffer*, const std::string&, const gamepad_lcmt* msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    command_ = {msg->leftStickAnalog[1], msg->leftStickAnalog[0],
                -msg->rightStickAnalog[0]};
    updated_at_ = Clock::now();
    received_ = true;
  }

  std::array<float, 3> command(std::chrono::milliseconds timeout) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!received_ || Clock::now() - updated_at_ > timeout) {
      return {0.0F, 0.0F, 0.0F};
    }
    return command_;
  }

  bool updated_after(Clock::time_point threshold) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_ && updated_at_ > threshold;
  }

 private:
  mutable std::mutex mutex_;
  std::array<float, 3> command_{};
  Clock::time_point updated_at_{};
  bool received_{false};
};

struct RobotSnapshot {
  bool valid{false};
  Clock::time_point updated_at{};
  std::array<float, kActionCount> q{};
  std::array<float, kActionCount> dq{};
  std::array<float, 4> quat{1.0F, 0.0F, 0.0F, 0.0F};
  std::array<float, 3> gyro{};
  std::array<float, 3> position{};
  std::array<float, 3> rpy{};
};

class StateReceiver {
 public:
  void handle(const robot_sdk::pb::RobotState& msg) {
    if (msg.q_abad_size() < kLegCount || msg.q_hip_size() < kLegCount ||
        msg.q_knee_size() < kLegCount || msg.q_foot_size() < kLegCount ||
        msg.qd_abad_size() < kLegCount || msg.qd_hip_size() < kLegCount ||
        msg.qd_knee_size() < kLegCount || msg.qd_foot_size() < kLegCount ||
        msg.quat_size() < 4 || msg.gyro_size() < 3) {
      return;
    }

    RobotSnapshot next;
    for (int leg = 0; leg < kLegCount; ++leg) {
      const int offset = leg * 4;
      next.q[offset] = msg.q_abad(leg);
      next.q[offset + 1] = msg.q_hip(leg);
      next.q[offset + 2] = msg.q_knee(leg);
      next.q[offset + 3] = msg.q_foot(leg);
      next.dq[offset] = msg.qd_abad(leg);
      next.dq[offset + 1] = msg.qd_hip(leg);
      next.dq[offset + 2] = msg.qd_knee(leg);
      next.dq[offset + 3] = msg.qd_foot(leg);
    }
    for (int i = 0; i < 4; ++i) {
      next.quat[i] = msg.quat(i);
    }
    const float w = next.quat[0];
    const float x = next.quat[1];
    const float y = next.quat[2];
    const float z = next.quat[3];
    const float sinr_cosp = 2.0F * (w * x + y * z);
    const float cosr_cosp = 1.0F - 2.0F * (x * x + y * y);
    next.rpy[0] = std::atan2(sinr_cosp, cosr_cosp);
    const float sinp = std::clamp(2.0F * (w * y - z * x), -1.0F, 1.0F);
    next.rpy[1] = std::asin(sinp);
    const float siny_cosp = 2.0F * (w * z + x * y);
    const float cosy_cosp = 1.0F - 2.0F * (y * y + z * z);
    next.rpy[2] = std::atan2(siny_cosp, cosy_cosp);
    for (int i = 0; i < 3; ++i) {
      next.gyro[i] = msg.gyro(i);
      if (msg.position_size() >= 3) {
        next.position[i] = msg.position(i);
      }
    }
    next.updated_at = Clock::now();
    next.valid = true;

    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_ = next;
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
  for (int i = 0; i < kLegCount; ++i) {
    field->Add(value);
  }
}

robot_sdk::pb::RobotCmd make_command() {
  robot_sdk::pb::RobotCmd cmd;
  add_four(cmd.mutable_q_des_abad(), 0.0F);
  add_four(cmd.mutable_q_des_hip(), 0.0F);
  add_four(cmd.mutable_q_des_knee(), 0.0F);
  add_four(cmd.mutable_q_des_foot(), 0.0F);
  add_four(cmd.mutable_qd_des_abad(), 0.0F);
  add_four(cmd.mutable_qd_des_hip(), 0.0F);
  add_four(cmd.mutable_qd_des_knee(), 0.0F);
  add_four(cmd.mutable_qd_des_foot(), 0.0F);
  add_four(cmd.mutable_kp_abad(), 0.0F);
  add_four(cmd.mutable_kp_hip(), 0.0F);
  add_four(cmd.mutable_kp_knee(), 0.0F);
  add_four(cmd.mutable_kp_foot(), 0.0F);
  add_four(cmd.mutable_kd_abad(), 0.0F);
  add_four(cmd.mutable_kd_hip(), 0.0F);
  add_four(cmd.mutable_kd_knee(), 0.0F);
  add_four(cmd.mutable_kd_foot(), 0.0F);
  add_four(cmd.mutable_tau_abad_ff(), 0.0F);
  add_four(cmd.mutable_tau_hip_ff(), 0.0F);
  add_four(cmd.mutable_tau_knee_ff(), 0.0F);
  add_four(cmd.mutable_tau_foot_ff(), 0.0F);
  return cmd;
}

void set_leg_target(robot_sdk::pb::RobotCmd& cmd, int leg, float abad, float hip,
                    float knee, float kp, float kd, float wheel_velocity,
                    float wheel_kd) {
  cmd.set_q_des_abad(leg, abad);
  cmd.set_q_des_hip(leg, hip);
  cmd.set_q_des_knee(leg, knee);
  cmd.set_q_des_foot(leg, 0.0F);
  cmd.set_qd_des_abad(leg, 0.0F);
  cmd.set_qd_des_hip(leg, 0.0F);
  cmd.set_qd_des_knee(leg, 0.0F);
  cmd.set_qd_des_foot(leg, wheel_velocity);
  cmd.set_kp_abad(leg, kp);
  cmd.set_kp_hip(leg, kp);
  cmd.set_kp_knee(leg, kp);
  cmd.set_kp_foot(leg, 0.0F);
  cmd.set_kd_abad(leg, kd);
  cmd.set_kd_hip(leg, kd);
  cmd.set_kd_knee(leg, kd);
  cmd.set_kd_foot(leg, wheel_kd);
}

std::array<float, 3> projected_gravity(const std::array<float, 4>& quat) {
  const float w = quat[0];
  const float x = quat[1];
  const float y = quat[2];
  const float z = quat[3];
  return {2.0F * (-z * x + w * y), -2.0F * (z * y + w * x),
          1.0F - 2.0F * (w * w + z * z)};
}

float wrap_angle(float angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

class DreamWaQPolicy {
 public:
  DreamWaQPolicy(const std::string& model_dir, bool stochastic, bool training_history)
      : stochastic_(stochastic), training_history_(training_history) {
    actor_ = torch::jit::load(model_dir + "/actor_dwaq.pt", torch::kCPU);
    encoder_ = torch::jit::load(model_dir + "/encoder_dwaq.pt", torch::kCPU);
    latent_mu_ = torch::jit::load(model_dir + "/latent_mu_dwaq.pt", torch::kCPU);
    latent_var_ = torch::jit::load(model_dir + "/latent_var_dwaq.pt", torch::kCPU);
    velocity_mu_ = torch::jit::load(model_dir + "/vel_mu_dwaq.pt", torch::kCPU);
    velocity_var_ = torch::jit::load(model_dir + "/vel_var_dwaq.pt", torch::kCPU);
    actor_.eval();
    encoder_.eval();
    latent_mu_.eval();
    latent_var_.eval();
    velocity_mu_.eval();
    velocity_var_.eval();
  }

  std::array<float, kObservationCount> observation(
      const RobotSnapshot& state, const std::array<float, 3>& command) const {
    std::array<float, kObservationCount> obs{};
    const auto gravity = projected_gravity(state.quat);
    for (int i = 0; i < 3; ++i) {
      obs[i] = state.gyro[i] * 0.25F;
      obs[3 + i] = gravity[i];
    }
    obs[6] = command[0] * 2.0F;
    obs[7] = command[1] * 2.0F;
    obs[8] = command[2] * 0.25F;

    for (int i = 0; i < kActionCount; ++i) {
      const bool wheel = i % 4 == 3;
      obs[9 + i] = wheel ? 0.0F : state.q[i] - kDefaultJointPosition[i];
      obs[25 + i] = state.dq[i] * 0.05F;
      obs[41 + i] = wheel ? 0.0F : state.q[i];
      obs[57 + i] = last_action_[i];
    }
    for (float& value : obs) {
      value = std::clamp(value, -100.0F, 100.0F);
    }
    return obs;
  }

  void append_history(std::array<float, kObservationCount> obs) {
    if (training_history_) {
      obs[6] = 0.0F;
      obs[7] = 0.0F;
      obs[8] = 0.0F;
    }
    std::move(history_.begin() + kObservationCount, history_.end(), history_.begin());
    std::copy(obs.begin(), obs.end(), history_.end() - kObservationCount);
  }

  std::array<float, kActionCount> infer(
      const std::array<float, kObservationCount>& obs, double* inference_ms) {
    torch::NoGradGuard no_grad;
    const auto started = Clock::now();

    if (!training_history_) {
      append_history(obs);
    }

    auto history_tensor =
        torch::from_blob(history_.data(), {kHistoryCount}, torch::kFloat32).clone();
    auto h = encoder_.forward({history_tensor}).toTensor();
    auto latent_mu = latent_mu_.forward({h}).toTensor();
    auto velocity_mu = velocity_mu_.forward({h}).toTensor();
    torch::Tensor latent = latent_mu;
    torch::Tensor velocity = velocity_mu;

    if (stochastic_) {
      auto latent_logvar = latent_var_.forward({h}).toTensor();
      auto velocity_logvar = velocity_var_.forward({h}).toTensor();
      auto latent_sigma = torch::exp(0.5 * latent_logvar).clamp(0.0, 5.0);
      auto velocity_sigma = torch::exp(0.5 * velocity_logvar).clamp(0.0, 5.0);
      latent = latent_mu + latent_sigma * torch::randn_like(latent_sigma);
      velocity = velocity_mu + velocity_sigma * torch::randn_like(velocity_sigma);
    }

    auto obs_tensor =
        torch::from_blob(const_cast<float*>(obs.data()), {kObservationCount}, torch::kFloat32)
            .clone();
    auto actor_input = torch::cat({latent, velocity, obs_tensor}, -1);
    auto output = actor_.forward({actor_input}).toTensor().to(torch::kCPU).contiguous();
    if (output.numel() != kActionCount) {
      throw std::runtime_error("DreamWaQ actor returned an unexpected action shape");
    }

    std::array<float, kActionCount> action{};
    std::memcpy(action.data(), output.data_ptr<float>(), sizeof(float) * kActionCount);
    for (float& value : action) {
      value = std::clamp(value, -5.0F, 5.0F);
    }
    last_action_ = action;

    if (training_history_) {
      append_history(obs);
    }
    *inference_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - started).count();
    return action;
  }

  const std::array<float, kActionCount>& last_action() const { return last_action_; }
  void clear_action() { last_action_.fill(0.0F); }
  bool training_history() const { return training_history_; }
  bool stochastic() const { return stochastic_; }

 private:
  torch::jit::script::Module actor_;
  torch::jit::script::Module encoder_;
  torch::jit::script::Module latent_mu_;
  torch::jit::script::Module latent_var_;
  torch::jit::script::Module velocity_mu_;
  torch::jit::script::Module velocity_var_;
  std::array<float, kHistoryCount> history_{};
  std::array<float, kActionCount> last_action_{};
  bool stochastic_{true};
  bool training_history_{false};
};

enum class Stage {
  kWaiting,
  kStand,
  kWarmup,
  kIdle,
  kPolicy,
  kStairAssist,
  kTerrainExit,
  kCharge,
  kRecover,
  kSafety
};

const char* stage_name(Stage stage) {
  switch (stage) {
    case Stage::kWaiting:
      return "waiting";
    case Stage::kStand:
      return "stand";
    case Stage::kWarmup:
      return "warmup";
    case Stage::kIdle:
      return "idle";
    case Stage::kPolicy:
      return "policy";
    case Stage::kStairAssist:
      return "stair_assist";
    case Stage::kTerrainExit:
      return "terrain_exit";
    case Stage::kCharge:
      return "charge";
    case Stage::kRecover:
      return "recover";
    case Stage::kSafety:
      return "safety";
  }
  return "unknown";
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, stop_handler);
  std::signal(SIGTERM, stop_handler);

  torch::set_num_threads(1);
  torch::set_num_interop_threads(1);
  torch::manual_seed(static_cast<std::uint64_t>(env_double("GO2W_RL_SEED", 1.0)));

  const std::string model_dir = env_string("GO2W_RL_MODEL_DIR", DEFAULT_MODEL_DIR);
  const bool stochastic = env_bool("GO2W_RL_STOCHASTIC", true);
  const bool training_history = env_string("GO2W_RL_HISTORY_MODE", "train") == "train";
  const double stand_seconds = std::max(0.5, env_double("GO2W_RL_STAND_SECONDS", 3.0));
  const int warmup_frames =
      std::max(5, static_cast<int>(env_double("GO2W_RL_WARMUP_FRAMES", 50.0)));
  const float leg_kp = static_cast<float>(env_double("GO2W_RL_LEG_KP", 40.0));
  const float leg_kd = static_cast<float>(env_double("GO2W_RL_LEG_KD", 1.0));
  const float wheel_kd = static_cast<float>(env_double("GO2W_RL_WHEEL_KD", 0.5));
  const float action_scale = static_cast<float>(env_double("GO2W_RL_ACTION_SCALE", 0.25));
  const float wheel_scale = static_cast<float>(env_double("GO2W_RL_WHEEL_SCALE", 10.0));
  const float max_vx = static_cast<float>(env_double("GO2W_RL_MAX_VX", 1.2));
  const float max_vy = static_cast<float>(env_double("GO2W_RL_MAX_VY", 0.6));
  const float max_yaw = static_cast<float>(env_double("GO2W_RL_MAX_YAW", 1.0));
  const bool idle_stand = env_bool("GO2W_RL_IDLE_STAND", true);
  const float idle_vx_deadband =
      static_cast<float>(env_double("GO2W_RL_IDLE_VX_DEADBAND", 0.06));
  const float idle_vy_deadband =
      static_cast<float>(env_double("GO2W_RL_IDLE_VY_DEADBAND", 0.06));
  const float idle_yaw_deadband =
      static_cast<float>(env_double("GO2W_RL_IDLE_YAW_DEADBAND", 0.08));
  const float test_vx = static_cast<float>(env_double("GO2W_RL_TEST_VX", 0.0));
  const float test_vy = static_cast<float>(env_double("GO2W_RL_TEST_VY", 0.0));
  const float test_yaw = static_cast<float>(env_double("GO2W_RL_TEST_YAW", 0.0));
  const double test_delay = std::max(0.0, env_double("GO2W_RL_TEST_DELAY", 1.0));
  const double test_duration = std::max(0.0, env_double("GO2W_RL_TEST_DURATION", 30.0));
  const float test_lateral_gain =
      static_cast<float>(env_double("GO2W_RL_TEST_LATERAL_GAIN", 0.6));
  const float test_heading_gain =
      static_cast<float>(env_double("GO2W_RL_TEST_HEADING_GAIN", 1.5));
  const float test_platform_distance =
      static_cast<float>(env_double("GO2W_RL_TEST_PLATFORM_DISTANCE", 3.6));
  const float test_platform_height =
      static_cast<float>(env_double("GO2W_RL_TEST_PLATFORM_HEIGHT", 0.85));
  const bool terrain_guard = env_bool("GO2W_RL_TERRAIN_GUARD", true);
  const float downhill_pitch =
      static_cast<float>(env_double("GO2W_RL_DOWNHILL_PITCH", 0.10));
  const float downhill_max_vx =
      static_cast<float>(env_double("GO2W_RL_DOWNHILL_MAX_VX", 0.50));
  const float downhill_max_vy =
      static_cast<float>(env_double("GO2W_RL_DOWNHILL_MAX_VY", 0.15));
  const float downhill_max_yaw =
      static_cast<float>(env_double("GO2W_RL_DOWNHILL_MAX_YAW", 0.40));
  const float downhill_release_pitch =
      static_cast<float>(env_double("GO2W_RL_DOWNHILL_RELEASE_PITCH", 0.08));
  const double downhill_release_seconds =
      std::max(0.1, env_double("GO2W_RL_DOWNHILL_RELEASE_SECONDS", 0.8));
  const float terrain_exit_pitch =
      static_cast<float>(env_double("GO2W_RL_TERRAIN_EXIT_PITCH", 0.16));
  const float terrain_exit_roll =
      static_cast<float>(env_double("GO2W_RL_TERRAIN_EXIT_ROLL", 0.25));
  const float terrain_level_pitch =
      static_cast<float>(env_double("GO2W_RL_TERRAIN_LEVEL_PITCH", 0.08));
  const float terrain_level_roll =
      static_cast<float>(env_double("GO2W_RL_TERRAIN_LEVEL_ROLL", 0.12));
  const float terrain_exit_vx =
      static_cast<float>(env_double("GO2W_RL_TERRAIN_EXIT_VX", 0.18));
  const float terrain_exit_heading_gain =
      static_cast<float>(env_double("GO2W_RL_TERRAIN_EXIT_HEADING_GAIN", 0.8));
  const float terrain_exit_max_yaw =
      static_cast<float>(env_double("GO2W_RL_TERRAIN_EXIT_MAX_YAW", 0.30));
  const double terrain_level_seconds =
      std::max(0.1, env_double("GO2W_RL_TERRAIN_LEVEL_SECONDS", 0.6));
  const double terrain_exit_drive_seconds =
      std::max(0.5, env_double("GO2W_RL_TERRAIN_EXIT_DRIVE_SECONDS", 6.0));
  const bool stair_assist = env_bool("GO2W_RL_STAIR_ASSIST", true);
  const float stair_stall_command =
      static_cast<float>(env_double("GO2W_RL_STAIR_STALL_COMMAND", 0.12));
  const float stair_stall_distance =
      static_cast<float>(env_double("GO2W_RL_STAIR_STALL_DISTANCE", 0.04));
  const float stair_stall_height_rise =
      static_cast<float>(env_double("GO2W_RL_STAIR_STALL_HEIGHT_RISE", 0.025));
  const float stair_stall_max_yaw =
      static_cast<float>(env_double("GO2W_RL_STAIR_STALL_MAX_YAW", 0.25));
  const double stair_stall_seconds =
      std::max(0.5, env_double("GO2W_RL_STAIR_STALL_SECONDS", 1.5));
  const float stair_assist_vx =
      static_cast<float>(env_double("GO2W_RL_STAIR_ASSIST_VX", 1.10));
  const float stair_assist_accel =
      static_cast<float>(env_double("GO2W_RL_STAIR_ASSIST_ACCEL", 3.0));
  const float stair_assist_decel =
      static_cast<float>(env_double("GO2W_RL_STAIR_ASSIST_DECEL", 2.0));
  const float stair_entry_vx =
      static_cast<float>(env_double("GO2W_RL_STAIR_ENTRY_VX", 0.90));
  const float stair_exit_vx =
      static_cast<float>(env_double("GO2W_RL_STAIR_EXIT_VX", 0.80));
  const double stair_assist_seconds =
      std::max(1.0, env_double("GO2W_RL_STAIR_ASSIST_SECONDS", 15.0));
  const bool stair_approach_assist = env_bool("GO2W_RL_STAIR_APPROACH_ASSIST", false);
  const std::string nav_mode_file = env_string("GO2W_NAV_MODE_FILE", "");
  const std::string posture_file = env_string("GO2W_RL_POSTURE_FILE", "");
  const float charge_hip = static_cast<float>(
      std::clamp(env_double("GO2W_RL_CHARGE_HIP", 1.20), 0.90, 1.60));
  const float charge_knee = static_cast<float>(
      std::clamp(env_double("GO2W_RL_CHARGE_KNEE", -2.30), -2.60, -1.80));
  const double posture_ramp_seconds =
      std::clamp(env_double("GO2W_RL_POSTURE_RAMP_SECONDS", 2.5), 1.0, 6.0);
  const float stair_route_center_x =
      static_cast<float>(env_double("GO2W_RL_STAIR_ROUTE_CENTER_X", 0.2));
  const float stair_outbound_min_y =
      static_cast<float>(env_double("GO2W_RL_STAIR_OUTBOUND_MIN_Y", -0.2));
  const float stair_outbound_max_y =
      static_cast<float>(env_double("GO2W_RL_STAIR_OUTBOUND_MAX_Y", 3.8));
  const float stair_return_min_y =
      static_cast<float>(env_double("GO2W_RL_STAIR_RETURN_MIN_Y", 6.2));
  const float stair_return_max_y =
      static_cast<float>(env_double("GO2W_RL_STAIR_RETURN_MAX_Y", 10.2));
  const float stair_outbound_fast_y =
      static_cast<float>(env_double("GO2W_RL_STAIR_OUTBOUND_FAST_Y", 1.2));
  const float stair_outbound_slow_y =
      static_cast<float>(env_double("GO2W_RL_STAIR_OUTBOUND_SLOW_Y", 2.7));
  const float stair_return_fast_y =
      static_cast<float>(env_double("GO2W_RL_STAIR_RETURN_FAST_Y", 8.9));
  const float stair_return_slow_y =
      static_cast<float>(env_double("GO2W_RL_STAIR_RETURN_SLOW_Y", 7.1));
  const float stair_approach_max_lateral =
      static_cast<float>(env_double("GO2W_RL_STAIR_APPROACH_MAX_LATERAL", 1.25));
  const float stair_approach_max_heading =
      static_cast<float>(env_double("GO2W_RL_STAIR_APPROACH_MAX_HEADING", 0.35));
  const float stair_alignment_max_lateral =
      static_cast<float>(env_double("GO2W_RL_STAIR_ALIGNMENT_MAX_LATERAL", 0.50));
  const float stair_alignment_max_heading =
      static_cast<float>(env_double("GO2W_RL_STAIR_ALIGNMENT_MAX_HEADING", 0.20));
  const float stair_alignment_vx =
      static_cast<float>(env_double("GO2W_RL_STAIR_ALIGNMENT_VX", 0.35));
  const float stair_route_max_vy =
      static_cast<float>(env_double("GO2W_RL_STAIR_ROUTE_MAX_VY", 0.40));
  const float stair_route_max_yaw =
      static_cast<float>(env_double("GO2W_RL_STAIR_ROUTE_MAX_YAW", 0.80));
  const float stair_route_hard_max_lateral =
      static_cast<float>(env_double("GO2W_RL_STAIR_ROUTE_HARD_MAX_LATERAL", 0.65));
  const float stair_route_hard_max_heading =
      static_cast<float>(env_double("GO2W_RL_STAIR_ROUTE_HARD_MAX_HEADING", 0.55));
  const float stair_route_hard_max_roll =
      static_cast<float>(env_double("GO2W_RL_STAIR_ROUTE_HARD_MAX_ROLL", 0.60));
  const float stair_route_hard_max_pitch =
      static_cast<float>(env_double("GO2W_RL_STAIR_ROUTE_HARD_MAX_PITCH", 0.75));
  const float stair_route_emergency_max_lateral =
      static_cast<float>(env_double("GO2W_RL_STAIR_ROUTE_EMERGENCY_MAX_LATERAL", 1.00));
  const float stair_route_emergency_max_heading =
      static_cast<float>(env_double("GO2W_RL_STAIR_ROUTE_EMERGENCY_MAX_HEADING", 0.80));
  const float stair_route_emergency_max_roll =
      static_cast<float>(env_double("GO2W_RL_STAIR_ROUTE_EMERGENCY_MAX_ROLL", 0.85));
  const float stair_route_emergency_max_pitch =
      static_cast<float>(env_double("GO2W_RL_STAIR_ROUTE_EMERGENCY_MAX_PITCH", 1.00));
  const float stair_route_platform_height =
      static_cast<float>(env_double("GO2W_RL_STAIR_ROUTE_PLATFORM_HEIGHT", 1.40));
  const double stair_route_max_seconds =
      std::max(5.0, env_double("GO2W_RL_STAIR_ROUTE_MAX_SECONDS", 45.0));
  const double stair_route_no_progress_seconds =
      std::max(2.0, env_double("GO2W_RL_STAIR_ROUTE_NO_PROGRESS_SECONDS", 8.0));

  DreamWaQPolicy policy(model_dir, stochastic, training_history);

  lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=255");
  if (!lcm.good()) {
    std::cerr << "Failed to initialize LCM velocity receiver\n";
    return 1;
  }
  VelocityReceiver velocity_receiver;
  lcm.subscribe("vel_cmd_lcm_data", &VelocityReceiver::handle, &velocity_receiver);
  std::thread lcm_thread([&lcm]() {
    while (running.load()) {
      lcm.handleTimeout(100);
    }
  });

  eCAL::Initialize(argc, argv, "go2w_rl_bridge");
  eCAL::protobuf::CPublisher<robot_sdk::pb::RobotCmd> publisher("mujoco_cmd");
  eCAL::protobuf::CSubscriber<robot_sdk::pb::RobotState> subscriber("mujoco_state");
  StateReceiver state_receiver;
  subscriber.AddReceiveCallback(
      [&state_receiver](const char*, const robot_sdk::pb::RobotState& state, long long,
                        long long, long long) { state_receiver.handle(state); });

  std::cout << "DreamWaQ Go2-W policy loaded from " << model_dir << '\n'
            << "policy=50Hz command=500Hz device=CPU stochastic=" << stochastic
            << " history=" << (training_history ? "train" : "deploy") << '\n'
            << "LCM vel_cmd_lcm_data -> DreamWaQ -> eCAL mujoco_cmd\n";

  robot_sdk::pb::RobotCmd command = make_command();
  Stage stage = Stage::kWaiting;
  Clock::time_point stand_started{};
  Clock::time_point policy_started{};
  Clock::time_point success_started{};
  std::array<float, kActionCount> initial_q{};
  int history_frames = 0;
  bool initialized = false;
  bool policy_enabled = false;
  bool success_reported = false;
  double inference_ms = 0.0;
  std::array<float, 3> test_origin{};
  float test_origin_yaw = 0.0F;
  float test_progress = 0.0F;
  float test_lateral_error = 0.0F;
  float test_heading_error = 0.0F;
  std::array<float, 3> applied_command{};
  std::array<float, 3> last_drive_command{};
  Clock::time_point terrain_exit_started{};
  Clock::time_point terrain_level_started{};
  Clock::time_point downhill_level_started{};
  Clock::time_point stair_stall_started{};
  Clock::time_point stair_assist_until{};
  Clock::time_point next_nav_mode_read{};
  Clock::time_point next_posture_read{};
  Clock::time_point posture_transition_started{};
  Clock::time_point up_mode_started{};
  Clock::time_point stair_route_started{};
  Clock::time_point stair_route_progress_at{};
  Clock::time_point stair_route_level_started{};
  std::array<float, 2> stair_stall_origin{};
  std::array<float, kActionCount> posture_initial_q{};
  float terrain_exit_yaw = 0.0F;
  float stair_approach_speed = 0.0F;
  bool terrain_exit_active = false;
  bool terrain_exit_armed = false;
  bool downhill_active = false;
  bool stair_assist_active = false;
  bool stair_approach_active = false;
  int stair_drive_direction = 0;
  int stair_route_direction = 0;
  int stair_up_direction = 0;
  int downhill_direction = 0;
  float stair_route_lateral_error = 0.0F;
  float stair_route_heading_error = 0.0F;
  float stair_route_best_progress = 0.0F;
  std::string nav_mode{"avoid"};
  std::string posture{"stand"};
  bool stair_route_fault = false;
  bool stair_route_degraded = false;
  bool stair_route_degraded_reported = false;
  bool state_timeout_active = false;
  bool downhill_limited = false;
  bool recover_release_to_policy = false;

  auto next_publish = Clock::now();
  auto next_policy = Clock::now();
  auto next_report = Clock::now();
  while (running.load() && eCAL::Ok()) {
    const auto now = Clock::now();
    const RobotSnapshot state = state_receiver.snapshot();
    const bool state_fresh =
        state.valid && now - state.updated_at <= std::chrono::milliseconds(250);

    if (now >= next_nav_mode_read) {
      const std::string next_mode = read_mode_file(nav_mode_file, "avoid");
      if (next_mode != nav_mode) {
        std::cout << "Navigation mode changed: " << nav_mode << " -> " << next_mode
                  << '\n';
        if (next_mode == "up") {
          up_mode_started = now;
        }
        nav_mode = next_mode;
      }
      next_nav_mode_read = now + std::chrono::milliseconds(200);
    }

    if (now >= next_posture_read) {
      const std::string next_posture = read_posture_file(posture_file);
      if (next_posture != posture) {
        recover_release_to_policy = posture == "recover" && next_posture == "stand";
        posture = next_posture;
        posture_initial_q = state_fresh ? state.q : kDefaultJointPosition;
        posture_transition_started = now;
        std::cout << "Posture changed to " << posture << " with a "
                  << posture_ramp_seconds << " s transition.\n";
      }
      next_posture_read = now + std::chrono::milliseconds(200);
    }

    if (state_fresh && !initialized) {
      initial_q = state.q;
      stand_started = now;
      initialized = true;
      stage = Stage::kStand;
      std::cout << "State acquired; starting smooth stand transition.\n";
    }

    if (initialized && !state_fresh) {
      if (!state_timeout_active) {
        std::cerr << "Robot state timed out; holding joints and stopping wheels.\n";
      }
      state_timeout_active = true;
      if (stair_up_direction != 0) {
        stair_route_fault = true;
      }
      stage = Stage::kSafety;
      applied_command = {};
      for (int leg = 0; leg < kLegCount; ++leg) {
        const int offset = leg * 4;
        set_leg_target(command, leg, state.q[offset], state.q[offset + 1],
                       state.q[offset + 2], 10.0F, 2.0F, 0.0F, 1.0F);
      }
    } else if (state_fresh) {
      state_timeout_active = false;
    }

    if (state_fresh && now >= next_policy && initialized) {
      next_policy += std::chrono::milliseconds(20);
      const double stand_elapsed = std::chrono::duration<double>(now - stand_started).count();
      double stand_alpha = std::clamp(stand_elapsed / stand_seconds, 0.0, 1.0);
      stand_alpha = stand_alpha * stand_alpha * (3.0 - 2.0 * stand_alpha);

      if (stand_alpha < 1.0) {
        stage = Stage::kStand;
        for (int leg = 0; leg < kLegCount; ++leg) {
          const int offset = leg * 4;
          const float abad = static_cast<float>(
              initial_q[offset] + stand_alpha * (kDefaultJointPosition[offset] - initial_q[offset]));
          const float hip = static_cast<float>(initial_q[offset + 1] +
                                               stand_alpha * (kDefaultJointPosition[offset + 1] -
                                                              initial_q[offset + 1]));
          const float knee = static_cast<float>(initial_q[offset + 2] +
                                                stand_alpha * (kDefaultJointPosition[offset + 2] -
                                                               initial_q[offset + 2]));
          set_leg_target(command, leg, abad, hip, knee, leg_kp, leg_kd, 0.0F, wheel_kd);
        }
      } else if (history_frames < warmup_frames) {
        stage = Stage::kWarmup;
        policy.append_history(policy.observation(state, {0.0F, 0.0F, 0.0F}));
        ++history_frames;
        for (int leg = 0; leg < kLegCount; ++leg) {
          const int offset = leg * 4;
          set_leg_target(command, leg, kDefaultJointPosition[offset],
                         kDefaultJointPosition[offset + 1],
                         kDefaultJointPosition[offset + 2], leg_kp, leg_kd, 0.0F,
                         wheel_kd);
        }
      } else {
        if (!policy_enabled) {
          policy_started = now;
          test_origin = state.position;
          test_origin_yaw = state.rpy[2];
          policy_enabled = true;
          std::cout << "Policy control enabled.\n";
        }
        stage = Stage::kPolicy;

        if (stair_up_direction != 0) {
          constexpr float kHalfPi = 1.57079632679F;
          const float route_heading = stair_up_direction > 0 ? kHalfPi : -kHalfPi;
          const float route_progress = stair_up_direction * state.position[1];
          stair_route_lateral_error =
              -std::sin(route_heading) * (state.position[0] - stair_route_center_x);
          stair_route_heading_error = wrap_angle(state.rpy[2] - route_heading);
          if (route_progress >= stair_route_best_progress + 0.04F) {
            stair_route_best_progress = route_progress;
            stair_route_progress_at = now;
          }

          const bool beyond_stairs =
              stair_up_direction > 0 ? state.position[1] >= stair_outbound_max_y
                                     : state.position[1] <= stair_return_min_y;
          const bool stable_on_platform =
              beyond_stairs && state.position[2] >= stair_route_platform_height &&
              std::abs(state.rpy[0]) <= terrain_level_roll &&
              std::abs(state.rpy[1]) <= terrain_level_pitch;
          if (stable_on_platform) {
            if (stair_route_level_started.time_since_epoch().count() == 0) {
              stair_route_level_started = now;
            } else if (std::chrono::duration<double>(now - stair_route_level_started).count() >=
                       terrain_level_seconds) {
              std::cout << "Committed stair route completed on the upper platform.\n";
              stair_up_direction = 0;
              stair_route_started = {};
              stair_route_progress_at = {};
              stair_route_level_started = {};
              stair_approach_speed = 0.0F;
              stair_route_degraded = false;
              stair_route_degraded_reported = false;
            }
          } else {
            stair_route_level_started = {};
          }

          if (stair_up_direction != 0) {
            const bool route_geometry_degraded =
                std::abs(stair_route_lateral_error) > stair_route_hard_max_lateral ||
                std::abs(stair_route_heading_error) > stair_route_hard_max_heading ||
                std::abs(state.rpy[0]) > stair_route_hard_max_roll ||
                std::abs(state.rpy[1]) > stair_route_hard_max_pitch;
            const bool route_geometry_emergency =
                std::abs(stair_route_lateral_error) > stair_route_emergency_max_lateral ||
                std::abs(stair_route_heading_error) > stair_route_emergency_max_heading ||
                std::abs(state.rpy[0]) > stair_route_emergency_max_roll ||
                std::abs(state.rpy[1]) > stair_route_emergency_max_pitch;
            const bool route_timed_out =
                std::chrono::duration<double>(now - stair_route_started).count() >=
                stair_route_max_seconds;
            const bool route_stalled =
                std::chrono::duration<double>(now - stair_route_progress_at).count() >=
                stair_route_no_progress_seconds;
            stair_route_degraded =
                route_geometry_degraded || route_timed_out || route_stalled;
            if (stair_route_degraded && !stair_route_degraded_reported) {
              std::cerr << "Committed stair route degraded: geometry="
                        << (route_geometry_degraded ? 1 : 0)
                        << " timeout=" << (route_timed_out ? 1 : 0)
                        << " stalled=" << (route_stalled ? 1 : 0)
                        << "; retaining forward traversal with bounded corrections.\n";
              stair_route_degraded_reported = true;
            } else if (!stair_route_degraded) {
              stair_route_degraded_reported = false;
            }
            if (route_geometry_emergency && !stair_route_fault) {
              stair_route_fault = true;
              std::cerr << "Committed stair route emergency stop: lateral="
                        << stair_route_lateral_error
                        << " heading=" << stair_route_heading_error
                        << " roll=" << state.rpy[0] << " pitch=" << state.rpy[1] << '\n';
            }
          }
        }

        const bool unsafe = state.position[2] < 0.18F || std::abs(state.rpy[0]) > 1.15F ||
                            std::abs(state.rpy[1]) > 1.15F || stair_route_fault;
        if (unsafe) {
          stage = Stage::kSafety;
          applied_command = {};
          terrain_exit_active = false;
          terrain_exit_armed = false;
          terrain_level_started = {};
          downhill_active = false;
          downhill_level_started = {};
          stair_assist_active = false;
          stair_approach_active = false;
          stair_approach_speed = 0.0F;
          stair_stall_started = {};
          for (int leg = 0; leg < kLegCount; ++leg) {
            const int offset = leg * 4;
            set_leg_target(command, leg, state.q[offset], state.q[offset + 1],
                           state.q[offset + 2], 10.0F, 2.0F, 0.0F, 1.0F);
          }
        } else {
          std::array<float, 3> desired =
              velocity_receiver.command(std::chrono::milliseconds(300));
          const double policy_elapsed =
              std::chrono::duration<double>(now - policy_started).count();
          const float dx = state.position[0] - test_origin[0];
          const float dy = state.position[1] - test_origin[1];
          const float forward_x = std::cos(test_origin_yaw);
          const float forward_y = std::sin(test_origin_yaw);
          test_progress = forward_x * dx + forward_y * dy;
          test_lateral_error = -forward_y * dx + forward_x * dy;
          test_heading_error = wrap_angle(state.rpy[2] - test_origin_yaw);
          const bool test_active =
              (test_vx != 0.0F || test_vy != 0.0F || test_yaw != 0.0F) &&
              policy_elapsed >= test_delay &&
              (test_duration == 0.0 || policy_elapsed < test_delay + test_duration);
          if (test_active) {
            const bool on_platform = test_progress >= test_platform_distance &&
                                     state.position[2] >= test_platform_height;
            desired = {on_platform ? 0.0F : test_vx,
                       test_vy - test_lateral_gain * test_lateral_error,
                       test_yaw - test_heading_gain * test_heading_error};
          }

          const bool was_stair_approach_active = stair_approach_active;
          stair_approach_active = false;
          stair_route_direction = 0;
          stair_route_lateral_error = 0.0F;
          stair_route_heading_error = 0.0F;
          if (stair_approach_assist && !test_active && nav_mode == "up" &&
              stair_up_direction == 0 && desired[0] > idle_vx_deadband &&
              velocity_receiver.updated_after(up_mode_started)) {
            constexpr float kHalfPi = 1.57079632679F;
            float route_heading = 0.0F;
            if (state.position[1] >= stair_outbound_min_y &&
                state.position[1] <= stair_outbound_max_y) {
              route_heading = kHalfPi;
              stair_route_direction = 1;
            } else if (state.position[1] >= stair_return_min_y &&
                       state.position[1] <= stair_return_max_y) {
              route_heading = -kHalfPi;
              stair_route_direction = -1;
            }

            if (stair_route_direction != 0) {
              const float route_dx = state.position[0] - stair_route_center_x;
              stair_route_lateral_error = -std::sin(route_heading) * route_dx;
              stair_route_heading_error = wrap_angle(state.rpy[2] - route_heading);
              stair_approach_active =
                  std::abs(stair_route_lateral_error) <= stair_approach_max_lateral &&
                  std::abs(stair_route_heading_error) <= stair_approach_max_heading;
              const bool route_entry_aligned =
                  std::abs(stair_route_lateral_error) <= stair_alignment_max_lateral &&
                  std::abs(stair_route_heading_error) <= stair_alignment_max_heading;
              if (stair_approach_active && route_entry_aligned) {
                stair_up_direction = stair_route_direction;
                stair_route_started = now;
                stair_route_progress_at = now;
                stair_route_best_progress = stair_up_direction * state.position[1];
                stair_route_level_started = {};
                stair_route_degraded = false;
                stair_route_degraded_reported = false;
                std::cout << "Committed stair route direction=" << stair_up_direction
                          << "; suppressing stop, reverse, and spin commands until the upper "
                             "platform.\n";
              }
            }
          }
          if (stair_up_direction != 0) {
            constexpr float kHalfPi = 1.57079632679F;
            const float route_heading = stair_up_direction > 0 ? kHalfPi : -kHalfPi;
            stair_route_direction = stair_up_direction;
            stair_route_lateral_error =
                -std::sin(route_heading) * (state.position[0] - stair_route_center_x);
            stair_route_heading_error = wrap_angle(state.rpy[2] - route_heading);
            stair_approach_active = true;
            if (!was_stair_approach_active) {
              stair_approach_speed =
                  std::max(0.2F, std::min(std::abs(applied_command[0]), stair_assist_vx));
            }
            float stair_target_speed = stair_assist_vx;
            if ((stair_route_direction > 0 && state.position[1] < stair_outbound_fast_y) ||
                (stair_route_direction < 0 && state.position[1] > stair_return_fast_y)) {
              stair_target_speed = std::min(stair_target_speed, stair_entry_vx);
            } else if ((stair_route_direction > 0 &&
                        state.position[1] > stair_outbound_slow_y) ||
                       (stair_route_direction < 0 &&
                        state.position[1] < stair_return_slow_y)) {
              stair_target_speed = std::min(stair_target_speed, stair_exit_vx);
            }
            const bool route_aligned =
                std::abs(stair_route_lateral_error) <= stair_alignment_max_lateral &&
                std::abs(stair_route_heading_error) <= stair_alignment_max_heading;
            if (!route_aligned || stair_route_degraded) {
              stair_target_speed = std::min(stair_target_speed, stair_alignment_vx);
            }
            if (stair_approach_speed < stair_target_speed) {
              stair_approach_speed = std::min(
                  stair_target_speed, stair_approach_speed + 0.02F * stair_assist_accel);
            } else {
              stair_approach_speed = std::max(
                  stair_target_speed, stair_approach_speed - 0.02F * stair_assist_decel);
            }
            desired = {stair_approach_speed,
                       std::clamp(-test_lateral_gain * stair_route_lateral_error,
                                  -stair_route_max_vy, stair_route_max_vy),
                       std::clamp(-test_heading_gain * stair_route_heading_error,
                                  -stair_route_max_yaw, stair_route_max_yaw)};
            stage = Stage::kStairAssist;
          } else {
            stair_approach_speed = 0.0F;
            if (stair_approach_active && desired[0] > stair_alignment_vx) {
              desired[0] = stair_alignment_vx;
            }
          }

          desired[0] = std::clamp(desired[0], -max_vx, max_vx);
          desired[1] = std::clamp(desired[1], -max_vy, max_vy);
          desired[2] = std::clamp(desired[2], -max_yaw, max_yaw);

          const bool requested_idle = std::abs(desired[0]) < idle_vx_deadband &&
                                      std::abs(desired[1]) < idle_vy_deadband &&
                                      std::abs(desired[2]) < idle_yaw_deadband;

          if (!requested_idle && std::abs(desired[0]) >= 0.10F) {
            last_drive_command = desired;
            terrain_exit_armed = true;
          }

          // A contact with any obstacle can look like a stair-riser stall in
          // body height and velocity.  Never let the high-speed stair assist
          // override MPPI outside the two known stair centerline corridors.
          constexpr float kHalfPi = 1.57079632679F;
          const bool stair_centered =
              std::abs(state.position[0] - stair_route_center_x) <=
              stair_alignment_max_lateral;
          const bool outbound_stair_corridor =
              state.position[1] >= stair_outbound_min_y &&
              state.position[1] <= stair_outbound_max_y && stair_centered &&
              std::abs(wrap_angle(state.rpy[2] - kHalfPi)) <=
                  stair_alignment_max_heading;
          const bool return_stair_corridor =
              state.position[1] >= stair_return_min_y &&
              state.position[1] <= stair_return_max_y && stair_centered &&
              std::abs(wrap_angle(state.rpy[2] + kHalfPi)) <=
                  stair_alignment_max_heading;
          const bool stair_assist_corridor =
              outbound_stair_corridor || return_stair_corridor;

          const int drive_direction = desired[0] > 0.0F ? 1 : -1;
          const bool stair_stall_tracking = stair_assist_active ||
              stair_stall_started.time_since_epoch().count() != 0;
          const float stair_assist_command_threshold =
              stair_stall_tracking ? idle_vx_deadband : stair_stall_command;
          const bool assist_command = !requested_idle &&
                                      std::abs(desired[0]) >= stair_assist_command_threshold &&
                                      std::abs(desired[1]) < 0.10F &&
                                      std::abs(desired[2]) <= stair_stall_max_yaw;
          const bool assist_hold_command =
              !requested_idle && std::abs(desired[0]) >= idle_vx_deadband;
          if (stair_assist_active &&
              (!stair_assist_corridor || !assist_hold_command ||
               drive_direction != stair_drive_direction || now >= stair_assist_until)) {
            stair_assist_active = false;
            stair_stall_started = {};
          }

          const bool stall_candidate = stair_assist && stair_assist_corridor && assist_command &&
                                       state.position[2] >=
                                           test_origin[2] + stair_stall_height_rise &&
                                       std::abs(state.rpy[0]) < 0.25F &&
                                       std::abs(state.rpy[1]) < 0.25F;
          if (!stair_assist_active && stall_candidate) {
            if (stair_stall_started.time_since_epoch().count() == 0 ||
                drive_direction != stair_drive_direction) {
              stair_stall_started = now;
              stair_stall_origin = {state.position[0], state.position[1]};
              stair_drive_direction = drive_direction;
            } else {
              const float stall_dx = state.position[0] - stair_stall_origin[0];
              const float stall_dy = state.position[1] - stair_stall_origin[1];
              if (std::hypot(stall_dx, stall_dy) >= stair_stall_distance) {
                stair_stall_started = now;
                stair_stall_origin = {state.position[0], state.position[1]};
              } else if (std::chrono::duration<double>(now - stair_stall_started).count() >=
                         stair_stall_seconds) {
                stair_assist_active = true;
                stair_assist_until = now + std::chrono::duration_cast<Clock::duration>(
                                               std::chrono::duration<double>(
                                                   stair_assist_seconds));
                std::cout << "Stair assist active; increasing command after a detected riser "
                             "stall.\n";
              }
            }
          } else if (!stall_candidate && !stair_assist_active) {
            stair_stall_started = {};
          }

          if (stair_assist_active) {
            const float assist_speed =
                std::min(stair_assist_vx,
                         std::max(std::abs(desired[0]),
                                  std::abs(applied_command[0]) + 0.02F * stair_assist_accel));
            desired[0] = std::copysign(assist_speed, desired[0]);
            stage = Stage::kStairAssist;
          }

          const bool downhill_detected =
              terrain_guard && !stair_approach_active &&
              std::abs(state.rpy[1]) >= downhill_pitch &&
              desired[0] * state.rpy[1] > 0.0F;
          const int desired_direction =
              downhill_active && std::abs(desired[0]) < idle_vx_deadband
                  ? downhill_direction
                  : (desired[0] >= 0.0F ? 1 : -1);
          if (downhill_detected) {
            downhill_active = true;
            downhill_direction = desired_direction;
            downhill_level_started = {};
          } else if (downhill_active) {
            if (requested_idle ||
                (std::abs(desired[0]) >= idle_vx_deadband &&
                 desired_direction != downhill_direction)) {
              downhill_active = false;
              downhill_level_started = {};
            } else if (std::abs(state.rpy[1]) <= downhill_release_pitch) {
              if (downhill_level_started.time_since_epoch().count() == 0) {
                downhill_level_started = now;
              } else if (std::chrono::duration<double>(now - downhill_level_started).count() >=
                         downhill_release_seconds) {
                downhill_active = false;
                downhill_level_started = {};
              }
            } else {
              downhill_level_started = {};
            }
          }

          downhill_limited = terrain_guard && downhill_active &&
                             desired_direction == downhill_direction;
          if (downhill_limited) {
            desired[0] = std::copysign(std::min(std::abs(desired[0]), downhill_max_vx),
                                       desired[0]);
            desired[1] = std::clamp(desired[1], -downhill_max_vy, downhill_max_vy);
            desired[2] = std::clamp(desired[2], -downhill_max_yaw, downhill_max_yaw);
          }

          const bool rough_attitude = std::abs(state.rpy[1]) >= terrain_exit_pitch ||
                                      std::abs(state.rpy[0]) >= terrain_exit_roll;
          const bool descending_attitude =
              std::abs(state.rpy[1]) >= terrain_exit_pitch &&
              last_drive_command[0] * state.rpy[1] > 0.0F;
          if (terrain_guard && terrain_exit_armed && !terrain_exit_active && requested_idle &&
              descending_attitude && std::abs(last_drive_command[0]) >= 0.10F) {
            terrain_exit_active = true;
            terrain_exit_armed = false;
            terrain_exit_started = now;
            terrain_level_started = {};
            terrain_exit_yaw = state.rpy[2];
            std::cout << "Terrain exit guard active; delaying idle until level ground.\n";
          } else if (terrain_exit_active && !requested_idle) {
            terrain_exit_active = false;
            terrain_level_started = {};
          } else if (!terrain_exit_active && requested_idle && !rough_attitude) {
            terrain_exit_armed = false;
            last_drive_command = {};
          }

          if (terrain_exit_active) {
            const bool level = std::abs(state.rpy[1]) <= terrain_level_pitch &&
                               std::abs(state.rpy[0]) <= terrain_level_roll;
            if (level) {
              if (terrain_level_started.time_since_epoch().count() == 0) {
                terrain_level_started = now;
              } else if (std::chrono::duration<double>(now - terrain_level_started).count() >=
                         terrain_level_seconds) {
                terrain_exit_active = false;
                terrain_exit_armed = false;
                terrain_level_started = {};
                last_drive_command = {};
                desired = {0.0F, 0.0F, 0.0F};
                std::cout << "Terrain exit guard complete; level-ground idle enabled.\n";
              }
            } else {
              terrain_level_started = {};
            }

            if (terrain_exit_active) {
              const double exit_elapsed =
                  std::chrono::duration<double>(now - terrain_exit_started).count();
              desired[0] = exit_elapsed < terrain_exit_drive_seconds
                               ? std::copysign(terrain_exit_vx, last_drive_command[0])
                               : 0.0F;
              desired[1] = 0.0F;
              desired[2] = std::clamp(
                  -terrain_exit_heading_gain * wrap_angle(state.rpy[2] - terrain_exit_yaw),
                  -terrain_exit_max_yaw, terrain_exit_max_yaw);
              stage = Stage::kTerrainExit;
            }
          }

          applied_command = desired;
          const bool idle = idle_stand && !terrain_exit_active &&
                            std::abs(desired[0]) < idle_vx_deadband &&
                            std::abs(desired[1]) < idle_vy_deadband &&
                            std::abs(desired[2]) < idle_yaw_deadband;
          const double posture_elapsed =
              posture_transition_started.time_since_epoch().count() == 0
                  ? posture_ramp_seconds
                  : std::chrono::duration<double>(now - posture_transition_started).count();
          const bool posture_transition_active = posture_elapsed < posture_ramp_seconds;
          const bool charge_requested = posture == "charge";
          const bool recover_requested = posture == "recover";
          const bool policy_recovery =
              recover_requested || (recover_release_to_policy && posture_transition_active);
          const auto apply_policy_command = [&](const std::array<float, 3>& policy_command) {
            const auto obs = policy.observation(state, policy_command);
            const auto action = policy.infer(obs, &inference_ms);
            for (int leg = 0; leg < kLegCount; ++leg) {
              const int offset = leg * 4;
              set_leg_target(
                  command, leg,
                  kDefaultJointPosition[offset] + action_scale * action[offset],
                  kDefaultJointPosition[offset + 1] + action_scale * action[offset + 1],
                  kDefaultJointPosition[offset + 2] + action_scale * action[offset + 2],
                  leg_kp, leg_kd, wheel_scale * action[offset + 3], wheel_kd);
            }
          };
          if (policy_recovery) {
            stage = Stage::kRecover;
            applied_command = {};
            apply_policy_command({0.0F, 0.0F, 0.0F});
          } else if (charge_requested || posture_transition_active) {
            stage = charge_requested ? Stage::kCharge : Stage::kStand;
            applied_command = {};
            inference_ms = 0.0;
            policy.clear_action();
            policy.append_history(policy.observation(state, {0.0F, 0.0F, 0.0F}));
            double posture_alpha =
                std::clamp(posture_elapsed / posture_ramp_seconds, 0.0, 1.0);
            posture_alpha = posture_alpha * posture_alpha * (3.0 - 2.0 * posture_alpha);
            const float target_hip = charge_requested ? charge_hip : kDefaultJointPosition[1];
            const float target_knee =
                charge_requested ? charge_knee : kDefaultJointPosition[2];
            for (int leg = 0; leg < kLegCount; ++leg) {
              const int offset = leg * 4;
              const float abad = posture_initial_q[offset] +
                                 static_cast<float>(posture_alpha) *
                                     (kDefaultJointPosition[offset] - posture_initial_q[offset]);
              const float hip = posture_initial_q[offset + 1] +
                                static_cast<float>(posture_alpha) *
                                    (target_hip - posture_initial_q[offset + 1]);
              const float knee = posture_initial_q[offset + 2] +
                                 static_cast<float>(posture_alpha) *
                                     (target_knee - posture_initial_q[offset + 2]);
              set_leg_target(command, leg, abad, hip, knee, leg_kp, leg_kd, 0.0F,
                             wheel_kd);
            }
          } else if (idle) {
            stage = Stage::kIdle;
            inference_ms = 0.0;
            policy.clear_action();
            policy.append_history(policy.observation(state, {0.0F, 0.0F, 0.0F}));
            for (int leg = 0; leg < kLegCount; ++leg) {
              const int offset = leg * 4;
              set_leg_target(command, leg, kDefaultJointPosition[offset],
                             kDefaultJointPosition[offset + 1],
                             kDefaultJointPosition[offset + 2], leg_kp, leg_kd, 0.0F,
                             wheel_kd);
            }
          } else {
            apply_policy_command(desired);
          }
        }
      }
    }

    publisher.Send(command);

    if (state_fresh && policy_enabled &&
        (stage == Stage::kPolicy || stage == Stage::kStairAssist ||
         stage == Stage::kTerrainExit || stage == Stage::kIdle) &&
        test_progress >= test_platform_distance - 0.3F &&
        state.position[2] >= test_platform_height && std::abs(test_lateral_error) < 0.55F &&
        std::abs(state.rpy[0]) < 0.35F && std::abs(state.rpy[1]) < 0.35F &&
        std::abs(test_heading_error) < 0.35F) {
      if (success_started.time_since_epoch().count() == 0) {
        success_started = now;
      } else if (!success_reported && now - success_started >= std::chrono::seconds(2)) {
        std::cout << "STAIR_SUCCESS pos=[" << state.position[0] << ',' << state.position[1]
                  << ',' << state.position[2] << "]\n";
        success_reported = true;
      }
    } else {
      success_started = {};
    }

    if (state_fresh && now >= next_report) {
      const auto& action = policy.last_action();
      const auto [min_it, max_it] = std::minmax_element(action.begin(), action.end());
      std::cout << std::fixed << std::setprecision(3) << "stage=" << stage_name(stage)
                << " pos=[" << state.position[0] << ',' << state.position[1] << ','
                << state.position[2] << "] rpy=[" << state.rpy[0] << ',' << state.rpy[1]
                << ',' << state.rpy[2] << "] action=[" << *min_it << ',' << *max_it
                << "] test=[" << test_progress << ',' << test_lateral_error << ','
                << test_heading_error << "] cmd=[" << applied_command[0] << ','
                << applied_command[1] << ',' << applied_command[2] << "] guard=["
                << (downhill_limited ? 1 : 0) << ',' << (terrain_exit_active ? 1 : 0)
                << ',' << ((stair_assist_active || stair_approach_active) ? 1 : 0)
                << "] mode=" << nav_mode << " route=[" << stair_route_direction << ','
                << stair_route_lateral_error << ',' << stair_route_heading_error
                << ',' << (stair_up_direction != 0 ? 1 : 0) << ','
                << (stair_route_fault ? 1 : 0) << ','
                << (stair_route_degraded ? 1 : 0) << "] infer_ms="
                << inference_ms << '\n';
      next_report = now + std::chrono::milliseconds(500);
    }

    next_publish += std::chrono::milliseconds(2);
    std::this_thread::sleep_until(next_publish);
    if (Clock::now() - next_publish > std::chrono::milliseconds(100)) {
      next_publish = Clock::now();
    }
  }

  for (int i = 0; i < 100; ++i) {
    for (int leg = 0; leg < kLegCount; ++leg) {
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
