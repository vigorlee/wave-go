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
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "gamepad_lcmt.hpp"

namespace {

using Clock = std::chrono::steady_clock;
constexpr int kGroupCount = 4;
constexpr int kJointsPerGroup = 3;
constexpr int kActionCount = kGroupCount * kJointsPerGroup;
constexpr int kObservationCount = 47;
constexpr float kPi = 3.14159265358979323846F;

// Official unitree_rl_gym G1 12-DoF order:
// left hip pitch/roll/yaw, knee, ankle pitch/roll, then the right leg.
constexpr std::array<float, kActionCount> kDefaultJointPosition = {
    -0.1F, 0.0F, 0.0F, 0.3F, -0.2F, 0.0F,
    -0.1F, 0.0F, 0.0F, 0.3F, -0.2F, 0.0F};
constexpr std::array<float, kActionCount> kJointKp = {
    100.0F, 100.0F, 100.0F, 150.0F, 40.0F, 40.0F,
    100.0F, 100.0F, 100.0F, 150.0F, 40.0F, 40.0F};
constexpr std::array<float, kActionCount> kJointKd = {
    2.0F, 2.0F, 2.0F, 4.0F, 2.0F, 2.0F,
    2.0F, 2.0F, 2.0F, 4.0F, 2.0F, 2.0F};

std::atomic<bool> running{true};

void stop_handler(int) { running.store(false); }

double env_double(const char* name, double fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  return end != value && *end == '\0' && std::isfinite(parsed) ? parsed : fallback;
}

std::string env_string(const char* name, const std::string& fallback) {
  const char* value = std::getenv(name);
  return value == nullptr || *value == '\0' ? fallback : std::string(value);
}

class VelocityReceiver {
 public:
  void handle(const lcm::ReceiveBuffer*, const std::string&, const gamepad_lcmt* msg) {
    const std::array<float, 3> next_command = {
        msg->leftStickAnalog[1], msg->leftStickAnalog[0],
        -msg->rightStickAnalog[0]};
    if (!std::all_of(next_command.begin(), next_command.end(),
                     [](float value) { return std::isfinite(value); })) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    command_ = next_command;
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

 private:
  mutable std::mutex mutex_;
  std::array<float, 3> command_{};
  Clock::time_point updated_at_{};
  bool received_{false};
};

struct RobotSnapshot {
  bool valid{false};
  bool has_position{false};
  bool has_velocity{false};
  std::uint64_t sequence{0};
  Clock::time_point updated_at{};
  std::array<float, kActionCount> q{};
  std::array<float, kActionCount> dq{};
  std::array<float, 4> quat{1.0F, 0.0F, 0.0F, 0.0F};
  std::array<float, 3> gyro{};
  std::array<float, 3> position{};
  std::array<float, 3> velocity{};
  std::array<float, 3> rpy{};
};

bool finite_snapshot(const RobotSnapshot& state) {
  for (float value : state.q) {
    if (!std::isfinite(value)) return false;
  }
  for (float value : state.dq) {
    if (!std::isfinite(value)) return false;
  }
  for (float value : state.quat) {
    if (!std::isfinite(value)) return false;
  }
  for (float value : state.gyro) {
    if (!std::isfinite(value)) return false;
  }
  if (state.has_position) {
    for (float value : state.position) {
      if (!std::isfinite(value)) return false;
    }
  }
  if (state.has_velocity) {
    for (float value : state.velocity) {
      if (!std::isfinite(value)) return false;
    }
  }
  return true;
}

class StateReceiver {
 public:
  void handle(const robot_sdk::pb::RobotState& msg) {
    if (msg.q_abad_size() < kGroupCount || msg.q_hip_size() < kGroupCount ||
        msg.q_knee_size() < kGroupCount || msg.qd_abad_size() < kGroupCount ||
        msg.qd_hip_size() < kGroupCount || msg.qd_knee_size() < kGroupCount ||
        msg.quat_size() < 4 || msg.gyro_size() < 3) {
      return;
    }

    RobotSnapshot next;
    for (int group = 0; group < kGroupCount; ++group) {
      const int joint = kJointsPerGroup * group;
      next.q[joint] = msg.q_abad(group);
      next.q[joint + 1] = msg.q_hip(group);
      next.q[joint + 2] = msg.q_knee(group);
      next.dq[joint] = msg.qd_abad(group);
      next.dq[joint + 1] = msg.qd_hip(group);
      next.dq[joint + 2] = msg.qd_knee(group);
    }

    float quat_norm_squared = 0.0F;
    for (int axis = 0; axis < 4; ++axis) {
      next.quat[axis] = msg.quat(axis);
      quat_norm_squared += next.quat[axis] * next.quat[axis];
    }
    if (!std::isfinite(quat_norm_squared) || quat_norm_squared < 1.0e-8F) {
      return;
    }
    const float inverse_norm = 1.0F / std::sqrt(quat_norm_squared);
    for (float& value : next.quat) value *= inverse_norm;

    for (int axis = 0; axis < 3; ++axis) next.gyro[axis] = msg.gyro(axis);
    if (msg.position_size() >= 3) {
      next.has_position = true;
      for (int axis = 0; axis < 3; ++axis) next.position[axis] = msg.position(axis);
    }
    if (msg.v_world_size() >= 3) {
      next.has_velocity = true;
      for (int axis = 0; axis < 3; ++axis) next.velocity[axis] = msg.v_world(axis);
    }

    const float w = next.quat[0];
    const float x = next.quat[1];
    const float y = next.quat[2];
    const float z = next.quat[3];
    next.rpy[0] = std::atan2(2.0F * (w * x + y * z),
                             1.0F - 2.0F * (x * x + y * y));
    next.rpy[1] = std::asin(std::clamp(2.0F * (w * y - z * x), -1.0F, 1.0F));
    next.rpy[2] = std::atan2(2.0F * (w * z + x * y),
                             1.0F - 2.0F * (y * y + z * z));
    if (!finite_snapshot(next)) {
      return;
    }

    next.updated_at = Clock::now();
    next.valid = true;
    std::lock_guard<std::mutex> lock(mutex_);
    next.sequence = ++sequence_;
    snapshot_ = next;
  }

  RobotSnapshot snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

 private:
  mutable std::mutex mutex_;
  RobotSnapshot snapshot_;
  std::uint64_t sequence_{0};
};

void add_four(google::protobuf::RepeatedField<float>* field, float value) {
  for (int group = 0; group < kGroupCount; ++group) field->Add(value);
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

void set_joint_target(robot_sdk::pb::RobotCmd& cmd, int joint, float q, float kp,
                      float kd) {
  const int group = joint / kJointsPerGroup;
  switch (joint % kJointsPerGroup) {
    case 0:
      cmd.set_q_des_abad(group, q);
      cmd.set_qd_des_abad(group, 0.0F);
      cmd.set_kp_abad(group, kp);
      cmd.set_kd_abad(group, kd);
      cmd.set_tau_abad_ff(group, 0.0F);
      break;
    case 1:
      cmd.set_q_des_hip(group, q);
      cmd.set_qd_des_hip(group, 0.0F);
      cmd.set_kp_hip(group, kp);
      cmd.set_kd_hip(group, kd);
      cmd.set_tau_hip_ff(group, 0.0F);
      break;
    default:
      cmd.set_q_des_knee(group, q);
      cmd.set_qd_des_knee(group, 0.0F);
      cmd.set_kp_knee(group, kp);
      cmd.set_kd_knee(group, kd);
      cmd.set_tau_knee_ff(group, 0.0F);
      break;
  }
}

void set_damping_command(robot_sdk::pb::RobotCmd& cmd, const RobotSnapshot& state,
                         float damping) {
  for (int joint = 0; joint < kActionCount; ++joint) {
    set_joint_target(cmd, joint, state.valid ? state.q[joint] : 0.0F, 0.0F, damping);
  }
}

std::array<float, 3> projected_gravity(const std::array<float, 4>& quat) {
  const float w = quat[0];
  const float x = quat[1];
  const float y = quat[2];
  const float z = quat[3];
  return {2.0F * (-z * x + w * y), -2.0F * (z * y + w * x),
          1.0F - 2.0F * (w * w + z * z)};
}

float approach(float current, float target, float max_delta) {
  return current + std::clamp(target - current, -max_delta, max_delta);
}

float wrap_angle(float angle) { return std::remainder(angle, 2.0F * kPi); }

torch::Device select_device(const std::string& name) {
  if (name == "cpu") return torch::Device(torch::kCPU);
  if (name == "cuda" || name.rfind("cuda:", 0) == 0) {
    if (!torch::cuda::is_available()) {
      throw std::runtime_error("G1_RL_DEVICE requests CUDA, but CUDA is unavailable");
    }
    return torch::Device(name);
  }
  throw std::runtime_error("G1_RL_DEVICE must be cpu, cuda, or cuda:<index>");
}

class G1Policy {
 public:
  G1Policy(const std::string& path, const torch::Device& device, float action_clip)
      : device_(device), module_(torch::jit::load(path, device)),
        action_clip_(action_clip) {
    module_.eval();
    reset();
  }

  void reset() {
    last_action_.fill(0.0F);
    if (module_.find_method("reset_memory")) {
      module_.run_method("reset_memory");
    }
  }

  std::array<float, kObservationCount> observation(
      const RobotSnapshot& state, const std::array<float, 3>& command,
      float phase) const {
    std::array<float, kObservationCount> obs{};
    const auto gravity = projected_gravity(state.quat);
    for (int axis = 0; axis < 3; ++axis) {
      obs[axis] = state.gyro[axis] * 0.25F;
      obs[3 + axis] = gravity[axis];
    }
    obs[6] = command[0] * 2.0F;
    obs[7] = command[1] * 2.0F;
    obs[8] = command[2] * 0.25F;
    for (int joint = 0; joint < kActionCount; ++joint) {
      obs[9 + joint] = state.q[joint] - kDefaultJointPosition[joint];
      obs[21 + joint] = state.dq[joint] * 0.05F;
      obs[33 + joint] = last_action_[joint];
    }
    obs[45] = std::sin(2.0F * kPi * phase);
    obs[46] = std::cos(2.0F * kPi * phase);
    for (float& value : obs) value = std::clamp(value, -100.0F, 100.0F);
    return obs;
  }

  std::array<float, kActionCount> infer(
      const std::array<float, kObservationCount>& obs, double* inference_ms) {
    torch::NoGradGuard no_grad;
    const auto started = Clock::now();
    auto input = torch::from_blob(const_cast<float*>(obs.data()),
                                  {1, kObservationCount}, torch::kFloat32)
                     .clone()
                     .to(device_);
    auto output = module_.forward({input}).toTensor().to(torch::kCPU).contiguous();
    if (output.scalar_type() != torch::kFloat32 || output.numel() != kActionCount) {
      throw std::runtime_error("G1 policy returned an unexpected action tensor");
    }
    std::array<float, kActionCount> action{};
    std::memcpy(action.data(), output.data_ptr<float>(), sizeof(float) * kActionCount);
    for (float& value : action) {
      if (!std::isfinite(value)) {
        throw std::runtime_error("G1 policy returned a non-finite action");
      }
      value = std::clamp(value, -action_clip_, action_clip_);
    }
    last_action_ = action;
    *inference_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - started).count();
    return action;
  }

 private:
  torch::Device device_;
  torch::jit::script::Module module_;
  float action_clip_;
  std::array<float, kActionCount> last_action_{};
};

enum class Stage {
  kWaiting,
  kStand,
  kIdle,
  kHolding,
  kPolicy,
  kStopping,
  kComplete,
  kTestFailed,
  kStateStale,
  kSafety
};

const char* stage_name(Stage stage) {
  switch (stage) {
    case Stage::kWaiting:
      return "waiting";
    case Stage::kStand:
      return "stand";
    case Stage::kIdle:
      return "idle";
    case Stage::kHolding:
      return "holding";
    case Stage::kPolicy:
      return "policy";
    case Stage::kStopping:
      return "stopping";
    case Stage::kComplete:
      return "complete";
    case Stage::kTestFailed:
      return "test_failed";
    case Stage::kStateStale:
      return "state_stale";
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

  const std::string policy_type = env_string("G1_RL_POLICY_TYPE", "dreamwaq");
  if (policy_type != "dreamwaq" && policy_type != "official") {
    std::cerr << "G1_RL_POLICY_TYPE must be dreamwaq or official\n";
    return 1;
  }
  const char* default_model_path = policy_type == "dreamwaq"
                                       ? DEFAULT_DREAMWAQ_MODEL_PATH
                                       : DEFAULT_OFFICIAL_MODEL_PATH;
  const std::string model_path = env_string("G1_RL_MODEL", default_model_path);
  const std::string device_name = env_string("G1_RL_DEVICE", "cpu");
  const std::string lcm_url =
      env_string("G1_RL_LCM_URL", "udpm://239.255.76.67:7667?ttl=255");
  const std::string lcm_channel = env_string("G1_RL_LCM_CHANNEL", "vel_cmd_lcm_data");
  const double stand_seconds = std::max(0.0, env_double("G1_RL_STAND_SECONDS", 0.05));
  const auto command_timeout = std::chrono::milliseconds(std::max(
      1, static_cast<int>(env_double("G1_RL_COMMAND_TIMEOUT_MS", 300.0))));
  const auto state_timeout = std::chrono::milliseconds(std::max(
      1, static_cast<int>(env_double("G1_RL_STATE_TIMEOUT_MS", 250.0))));
  const float max_vx = static_cast<float>(std::abs(env_double("G1_RL_MAX_VX", 0.8)));
  const float max_vy = static_cast<float>(std::abs(env_double("G1_RL_MAX_VY", 0.5)));
  const float max_yaw = static_cast<float>(std::abs(env_double("G1_RL_MAX_YAW", 1.57)));
  const float action_scale = static_cast<float>(env_double("G1_RL_ACTION_SCALE", 0.25));
  const float action_clip = static_cast<float>(std::abs(env_double(
      "G1_RL_ACTION_CLIP", 10.0)));
  const float idle_linear_threshold = static_cast<float>(
      std::abs(env_double("G1_RL_IDLE_LINEAR_THRESHOLD", 0.04)));
  const float idle_yaw_threshold = static_cast<float>(
      std::abs(env_double("G1_RL_IDLE_YAW_THRESHOLD", 0.05)));
  const int idle_delay_frames = std::max(
      0, static_cast<int>(std::ceil(
             std::max(0.0, env_double("G1_RL_IDLE_DELAY_SECONDS", 0.25)) / 0.02)));
  const int idle_transition_frames = std::max(
      0, static_cast<int>(std::ceil(
             std::max(0.0, env_double("G1_RL_IDLE_TRANSITION_SECONDS", 1.0)) /
             0.02)));
  const float hold_position_gain = static_cast<float>(
      std::max(0.0, env_double("G1_RL_HOLD_POSITION_GAIN", 0.8)));
  const float hold_velocity_gain = static_cast<float>(
      std::max(0.0, env_double("G1_RL_HOLD_VELOCITY_GAIN", 0.4)));
  const float hold_yaw_gain = static_cast<float>(
      std::max(0.0, env_double("G1_RL_HOLD_YAW_GAIN", 0.8)));
  const float hold_yaw_rate_gain = static_cast<float>(
      std::max(0.0, env_double("G1_RL_HOLD_YAW_RATE_GAIN", 0.4)));
  const float max_hold_linear = static_cast<float>(
      std::abs(env_double("G1_RL_MAX_HOLD_LINEAR", 0.18)));
  const float max_hold_yaw = static_cast<float>(
      std::abs(env_double("G1_RL_MAX_HOLD_YAW", 0.25)));
  const float hold_position_tolerance = static_cast<float>(
      std::abs(env_double("G1_RL_HOLD_POSITION_TOLERANCE", 0.12)));
  const float hold_yaw_tolerance = static_cast<float>(
      std::abs(env_double("G1_RL_HOLD_YAW_TOLERANCE", 0.20)));
  const float safety_damping =
      static_cast<float>(env_double("G1_RL_SAFETY_DAMPING", 8.0));
  const float fall_height = static_cast<float>(env_double("G1_RL_FALL_HEIGHT", 0.38));
  const float fall_tilt = static_cast<float>(env_double("G1_RL_FALL_TILT", 0.9));
  const float test_vx = static_cast<float>(env_double("G1_RL_TEST_VX", 0.0));
  const double test_delay = std::max(0.0, env_double("G1_RL_TEST_DELAY", 0.0));
  const double test_duration = std::max(0.0, env_double("G1_RL_TEST_DURATION", 25.0));
  const float test_lateral_gain = static_cast<float>(
      std::max(0.0, env_double("G1_RL_TEST_LATERAL_GAIN", 0.4)));
  const float test_heading_gain = static_cast<float>(
      std::max(0.0, env_double("G1_RL_TEST_HEADING_GAIN", 0.8)));
  const float test_max_vy =
      static_cast<float>(std::abs(env_double("G1_RL_TEST_MAX_VY", 0.15)));
  const float test_max_yaw =
      static_cast<float>(std::abs(env_double("G1_RL_TEST_MAX_YAW", 0.35)));
  const float test_vy_step =
      static_cast<float>(std::abs(env_double("G1_RL_TEST_VY_STEP", 0.02)));
  const float test_yaw_step =
      static_cast<float>(std::abs(env_double("G1_RL_TEST_YAW_STEP", 0.04)));
  const float test_decel_step =
      static_cast<float>(std::abs(env_double("G1_RL_TEST_DECEL_STEP", 0.04)));
  const float test_reference_y =
      static_cast<float>(env_double("G1_RL_TEST_REFERENCE_Y", 0.0));
  const float test_reference_yaw =
      static_cast<float>(env_double("G1_RL_TEST_REFERENCE_YAW", 0.0));
  const float test_stop_x =
      static_cast<float>(env_double("G1_RL_TEST_STOP_X", 7.2));
  const float test_top_rise = static_cast<float>(
      std::max(0.0, env_double("G1_RL_TEST_TOP_RISE", 0.20)));
  const float requested_ground_margin = static_cast<float>(
      std::max(0.0, env_double("G1_RL_TEST_GROUND_MARGIN", 0.10)));
  const float test_ground_margin =
      std::min(requested_ground_margin, std::max(0.0F, test_top_rise - 0.01F));
  const float test_top_min_x =
      static_cast<float>(env_double("G1_RL_TEST_TOP_MIN_X", 3.05));
  const float test_top_max_x =
      static_cast<float>(env_double("G1_RL_TEST_TOP_MAX_X", 5.15));
  const float test_down_min_x =
      static_cast<float>(env_double("G1_RL_TEST_DOWN_MIN_X", 6.65));
  const float test_max_course_y = static_cast<float>(
      std::abs(env_double("G1_RL_TEST_MAX_COURSE_Y", 0.80)));
  const int test_settle_frames = std::max(
      1, static_cast<int>(std::ceil(
             std::max(0.0, env_double("G1_RL_TEST_SETTLE_SECONDS", 1.0)) / 0.02)));
  const int test_post_hold_frames = std::max(
      1, static_cast<int>(std::ceil(
             std::max(0.0, env_double("G1_RL_TEST_POST_HOLD_SECONDS", 2.0)) /
             0.02)));
  const bool test_mode = std::abs(test_vx) > 1.0e-6F;

  std::unique_ptr<G1Policy> policy;
  torch::Device device(torch::kCPU);
  try {
    device = select_device(device_name);
    policy = std::make_unique<G1Policy>(model_path, device, action_clip);
  } catch (const std::exception& error) {
    std::cerr << "Failed to load G1 policy: " << error.what() << '\n';
    return 1;
  }

  lcm::LCM lcm(lcm_url);
  if (!lcm.good()) {
    std::cerr << "Failed to initialize LCM at " << lcm_url << '\n';
    return 1;
  }
  VelocityReceiver velocity_receiver;
  lcm.subscribe(lcm_channel, &VelocityReceiver::handle, &velocity_receiver);
  std::thread lcm_thread([&lcm]() {
    while (running.load()) lcm.handleTimeout(100);
  });

  eCAL::Initialize(argc, argv, "g1_rl_bridge");
  eCAL::protobuf::CPublisher<robot_sdk::pb::RobotCmd> publisher("mujoco_cmd");
  eCAL::protobuf::CSubscriber<robot_sdk::pb::RobotState> subscriber("mujoco_state");
  StateReceiver state_receiver;
  subscriber.AddReceiveCallback(
      [&state_receiver](const char*, const robot_sdk::pb::RobotState& state, long long,
                        long long, long long) { state_receiver.handle(state); });

  std::cout << "Unitree G1 12-DoF policy loaded from " << model_path << '\n'
            << "policy_type=" << policy_type
            << " action_clip=" << action_clip
            << " idle_hold=[" << idle_linear_threshold << ',' << idle_yaw_threshold
            << "] delay_frames=" << idle_delay_frames
            << " transition_frames=" << idle_transition_frames
            << " hold_gain=" << hold_position_gain << '\n'
            << "policy=50Hz command_hold=500Hz device=" << device.str()
            << " torch_threads=1\n"
            << "LCM " << lcm_channel << " -> policy -> eCAL mujoco_cmd\n";

  robot_sdk::pb::RobotCmd command = make_command();
  Stage stage = Stage::kWaiting;
  Stage reported_stage = stage;
  RobotSnapshot last_state;
  std::array<float, kActionCount> initial_q{};
  std::array<float, 3> applied_command{};
  bool initialized = false;
  bool policy_enabled = false;
  bool locomotion_active = false;
  bool idle_hold_active = false;
  bool pose_hold_active = false;
  bool state_stale_active = false;
  bool safety_latched = false;
  bool stand_command_ready = false;
  bool stand_command_announced = false;
  bool test_started = false;
  bool test_reached_top = false;
  bool test_descended = false;
  bool test_stopping = false;
  bool test_completed = false;
  bool test_success_announced = false;
  bool test_failed = false;
  int test_top_frames = 0;
  int test_down_frames = 0;
  int test_stable_frames = 0;
  int test_post_hold_stable_frames = 0;
  float test_start_z = 0.0F;
  float test_peak_z = 0.0F;
  float test_max_abs_y = 0.0F;
  float test_smoothed_vx = 0.0F;
  float test_smoothed_vy = 0.0F;
  float test_smoothed_yaw = 0.0F;
  int idle_command_frames = 0;
  int idle_transition_step = 0;
  std::array<float, kActionCount> idle_start_q{};
  std::array<float, 2> hold_position{};
  float hold_yaw = 0.0F;
  std::uint64_t stand_step = 0;
  std::uint64_t policy_step = 0;
  double inference_ms = 0.0;
  float max_abs_action = 0.0F;

  auto fail_test = [&](const char* reason, const RobotSnapshot& state) {
    if (test_failed) return;
    test_failed = true;
    test_stopping = false;
    test_completed = false;
    test_smoothed_vx = 0.0F;
    test_smoothed_vy = 0.0F;
    test_smoothed_yaw = 0.0F;
    std::cerr << "STAIR_FAILED reason=" << reason;
    if (state.has_position) {
      std::cerr << " xy=[" << state.position[0] << ',' << state.position[1]
                << "] z=" << state.position[2] << " peak_z=" << test_peak_z;
    }
    std::cerr << '\n';
  };

  auto next_publish = Clock::now();
  auto next_policy = Clock::now();
  auto next_report = Clock::now();
  int result = 0;
  try {
    while (running.load() && eCAL::Ok()) {
      const auto now = Clock::now();
      const RobotSnapshot state = state_receiver.snapshot();
      if (state.valid) last_state = state;
      const bool state_fresh =
          state.valid && now - state.updated_at <= state_timeout;

      if (state_fresh && !initialized) {
        initial_q = state.q;
        initialized = true;
        stage = Stage::kStand;
        std::cout << "State acquired; starting smooth stand transition.\n";
      }
      if (state_fresh && state_stale_active) {
        state_stale_active = false;
        std::cout << "State stream recovered; restarting policy memory.\n";
      }

      const bool policy_due = now >= next_policy;
      if (policy_due) {
        next_policy += std::chrono::milliseconds(20);
        if (next_policy <= now) {
          next_policy = now + std::chrono::milliseconds(20);
        }

        if (!initialized) {
          stage = Stage::kWaiting;
          for (int joint = 0; joint < kActionCount; ++joint) {
            set_joint_target(command, joint, kDefaultJointPosition[joint],
                             kJointKp[joint], kJointKd[joint]);
          }
        } else if (!state_fresh) {
          if (!state_stale_active) {
            state_stale_active = true;
            if (test_mode && policy_enabled && !test_failed &&
                !test_success_announced) {
              fail_test("state_stale", last_state);
            }
            policy->reset();
            locomotion_active = false;
            idle_hold_active = false;
            pose_hold_active = false;
            idle_command_frames = 0;
            idle_transition_step = 0;
            policy_step = 0;
          }
          stage = Stage::kStateStale;
          applied_command = {};
          inference_ms = 0.0;
          max_abs_action = 0.0F;
          test_smoothed_vx = 0.0F;
          test_smoothed_vy = 0.0F;
          test_smoothed_yaw = 0.0F;
          set_damping_command(command, last_state, safety_damping);
        } else if (safety_latched) {
          stage = Stage::kSafety;
          applied_command = {};
          set_damping_command(command, state, safety_damping);
        } else {
          constexpr double kPolicyDtSeconds = 0.02;
          const std::uint64_t required_stand_steps =
              static_cast<std::uint64_t>(std::ceil(stand_seconds / kPolicyDtSeconds));
          if (required_stand_steps == 0) {
            stand_command_ready = true;
          }
          if (stand_step < required_stand_steps) {
            const double stand_elapsed =
                static_cast<double>(stand_step + 1) * kPolicyDtSeconds;
            double stand_alpha = stand_seconds > 0.0
                                     ? std::clamp(stand_elapsed / stand_seconds, 0.0, 1.0)
                                     : 1.0;
            stand_alpha = stand_alpha * stand_alpha * (3.0 - 2.0 * stand_alpha);
            stage = Stage::kStand;
            for (int joint = 0; joint < kActionCount; ++joint) {
              const float target = static_cast<float>(
                  initial_q[joint] +
                  stand_alpha * (kDefaultJointPosition[joint] - initial_q[joint]));
              set_joint_target(command, joint, target, kJointKp[joint], kJointKd[joint]);
            }
            ++stand_step;
            stand_command_ready = true;
          } else {
            if (!policy_enabled) {
              policy_enabled = true;
              std::cout << "Policy control enabled.\n";
            }
            const bool fallen =
                (state.has_position && state.position[2] < fall_height) ||
                std::abs(state.rpy[0]) > fall_tilt || std::abs(state.rpy[1]) > fall_tilt;
            if (fallen) {
              if (test_mode && test_started) {
                fail_test(test_completed ? "post_complete_fall" : "fall", state);
              }
              safety_latched = true;
              stage = Stage::kSafety;
              applied_command = {};
              set_damping_command(command, state, safety_damping);
              std::cerr << "Fall protection latched; restart the bridge after resetting G1.\n";
            } else {
              constexpr float kPolicyDt = 0.02F;
              constexpr float kGaitPeriod = 0.8F;
              constexpr int kCourseConfirmFrames = 5;
              const float cos_yaw = std::cos(state.rpy[2]);
              const float sin_yaw = std::sin(state.rpy[2]);
              const float body_vx = state.has_velocity
                                        ? cos_yaw * state.velocity[0] +
                                              sin_yaw * state.velocity[1]
                                        : 0.0F;
              const float body_vy = state.has_velocity
                                        ? -sin_yaw * state.velocity[0] +
                                              cos_yaw * state.velocity[1]
                                        : 0.0F;
              const float policy_time =
                  static_cast<float>(policy_step + 1) * kPolicyDt;
              const bool test_deadline_reached =
                  test_mode && test_duration > 0.0 &&
                  policy_time >= test_delay + test_duration;
              if (test_deadline_reached && !test_failed &&
                  !test_success_announced) {
                const char* reason = !test_started
                                         ? "not_started_timeout"
                                         : (test_completed
                                                ? "post_hold_timeout"
                                                : (test_stopping ? "settle_timeout"
                                                                 : "timeout"));
                fail_test(reason, state);
              }
              if (!test_mode) {
                applied_command = velocity_receiver.command(command_timeout);
              } else {
                applied_command = {};
                if (test_failed) {
                  stage = Stage::kTestFailed;
                } else if (test_completed) {
                  stage = Stage::kComplete;
                } else if (policy_time >= test_delay && state.has_position) {
                  if (!test_started) {
                    test_started = true;
                    test_start_z = state.position[2];
                    test_peak_z = test_start_z;
                    test_max_abs_y = std::abs(state.position[1] - test_reference_y);
                    test_smoothed_vx = test_vx;
                    std::cout << "STAIR_BENCHMARK_STARTED z=" << test_start_z
                              << " stop_x=" << test_stop_x << " vx=" << test_vx << '\n';
                  }

                  const float lateral_error = state.position[1] - test_reference_y;
                  const float abs_lateral_error = std::abs(lateral_error);
                  test_peak_z = std::max(test_peak_z, state.position[2]);
                  test_max_abs_y = std::max(test_max_abs_y, abs_lateral_error);

                  const float top_x_lo = std::min(test_top_min_x, test_top_max_x);
                  const float top_x_hi = std::max(test_top_min_x, test_top_max_x);
                  const bool top_sample =
                      state.position[0] >= top_x_lo && state.position[0] <= top_x_hi &&
                      abs_lateral_error <= test_max_course_y &&
                      state.position[2] >= test_start_z + test_top_rise;
                  test_top_frames = top_sample ? test_top_frames + 1 : 0;
                  if (test_top_frames >= kCourseConfirmFrames) {
                    test_reached_top = true;
                  }

                  const bool down_sample =
                      test_reached_top && state.position[0] >= test_down_min_x &&
                      abs_lateral_error <= test_max_course_y &&
                      state.position[2] <= test_start_z + test_ground_margin;
                  test_down_frames = down_sample ? test_down_frames + 1 : 0;
                  if (test_down_frames >= kCourseConfirmFrames) {
                    test_descended = true;
                  }

                  const bool stop_reached =
                      test_vx > 0.0F ? state.position[0] >= test_stop_x
                                     : state.position[0] <= test_stop_x;
                  if (abs_lateral_error > test_max_course_y) {
                    fail_test("off_course", state);
                    stage = Stage::kTestFailed;
                  } else if (!test_stopping && stop_reached) {
                    if (test_reached_top && test_descended) {
                      test_stopping = true;
                      test_stable_frames = 0;
                    } else {
                      fail_test("course_not_traversed", state);
                      stage = Stage::kTestFailed;
                    }
                  }

                  if (test_stopping && !test_failed) {
                    stage = Stage::kStopping;
                    test_smoothed_vx =
                        approach(test_smoothed_vx, 0.0F, test_decel_step);
                    test_smoothed_vy =
                        approach(test_smoothed_vy, 0.0F, test_vy_step);
                    test_smoothed_yaw =
                        approach(test_smoothed_yaw, 0.0F, test_yaw_step);
                    applied_command =
                        {test_smoothed_vx, test_smoothed_vy, test_smoothed_yaw};
                    const bool command_stopped =
                        std::abs(test_smoothed_vx) < 1.0e-4F &&
                        std::abs(test_smoothed_vy) < 1.0e-4F &&
                        std::abs(test_smoothed_yaw) < 1.0e-4F;
                    const bool pose_stable =
                        state.position[2] >= test_start_z - 0.05F &&
                        std::abs(state.rpy[0]) <= 0.25F &&
                        std::abs(state.rpy[1]) <= 0.25F;
                    const bool velocity_stable =
                        state.has_velocity &&
                        std::hypot(state.velocity[0], state.velocity[1]) <= 0.12F &&
                        std::abs(state.gyro[2]) <= 0.25F;
                    test_stable_frames =
                        command_stopped && pose_stable && velocity_stable
                            ? test_stable_frames + 1
                            : 0;
                    if (test_stable_frames >= test_settle_frames) {
                      test_completed = true;
                      test_stopping = false;
                      pose_hold_active = false;
                      test_post_hold_stable_frames = 0;
                      stage = Stage::kComplete;
                      applied_command = {};
                      std::cout << "STAIR_TRAVERSAL_COMPLETE xy=["
                                << state.position[0] << ',' << state.position[1]
                                << "] z=" << state.position[2] << '\n';
                    }
                  } else if (!test_failed) {
                    const float yaw_error = wrap_angle(state.rpy[2] - test_reference_yaw);
                    const float tilt_scale = std::clamp(
                        (0.55F - std::abs(state.rpy[1])) / 0.35F, 0.25F, 1.0F);
                    const float target_vy =
                        tilt_scale *
                        std::clamp(-test_lateral_gain * lateral_error,
                                   -test_max_vy, test_max_vy);
                    const float target_yaw =
                        tilt_scale *
                        std::clamp(-test_heading_gain * yaw_error, -test_max_yaw,
                                   test_max_yaw);
                    test_smoothed_vy =
                        approach(test_smoothed_vy, target_vy, test_vy_step);
                    test_smoothed_yaw =
                        approach(test_smoothed_yaw, target_yaw, test_yaw_step);
                    applied_command =
                        {test_smoothed_vx, test_smoothed_vy, test_smoothed_yaw};
                  }
                } else {
                  test_smoothed_vx = 0.0F;
                  test_smoothed_vy = 0.0F;
                  test_smoothed_yaw = 0.0F;
                }
              }
              applied_command[0] = std::clamp(applied_command[0], -max_vx, max_vx);
              applied_command[1] = std::clamp(applied_command[1], -max_vy, max_vy);
              applied_command[2] = std::clamp(applied_command[2], -max_yaw, max_yaw);

              const bool requested_command_idle =
                  std::hypot(applied_command[0], applied_command[1]) <=
                      idle_linear_threshold &&
                  std::abs(applied_command[2]) <= idle_yaw_threshold;
              if (!test_mode && requested_command_idle) {
                ++idle_command_frames;
              } else {
                idle_command_frames = 0;
              }
              const bool pose_hold_requested =
                  requested_command_idle && state.has_position &&
                  state.has_velocity &&
                  (test_mode || !locomotion_active ||
                   idle_command_frames >= idle_delay_frames);
              float hold_distance = 0.0F;
              float hold_yaw_error = 0.0F;
              if (pose_hold_requested) {
                if (!pose_hold_active) {
                  hold_position = {state.position[0], state.position[1]};
                  hold_yaw = state.rpy[2];
                  pose_hold_active = true;
                  test_post_hold_stable_frames = 0;
                  std::cout << "Pose hold anchored at [" << hold_position[0] << ','
                            << hold_position[1] << "] yaw=" << hold_yaw << '\n';
                }
                const float world_error_x = hold_position[0] - state.position[0];
                const float world_error_y = hold_position[1] - state.position[1];
                const float body_error_x =
                    cos_yaw * world_error_x + sin_yaw * world_error_y;
                const float body_error_y =
                    -sin_yaw * world_error_x + cos_yaw * world_error_y;
                hold_distance = std::hypot(world_error_x, world_error_y);
                hold_yaw_error = wrap_angle(hold_yaw - state.rpy[2]);
                applied_command[0] = std::clamp(
                    hold_position_gain * body_error_x - hold_velocity_gain * body_vx,
                    -max_hold_linear, max_hold_linear);
                applied_command[1] = std::clamp(
                    hold_position_gain * body_error_y - hold_velocity_gain * body_vy,
                    -max_hold_linear, max_hold_linear);
                applied_command[2] = std::clamp(
                    hold_yaw_gain * hold_yaw_error -
                        hold_yaw_rate_gain * state.gyro[2],
                    -max_hold_yaw, max_hold_yaw);
              } else if (!requested_command_idle || !locomotion_active) {
                pose_hold_active = false;
              }
              applied_command[0] = std::clamp(applied_command[0], -max_vx, max_vx);
              applied_command[1] = std::clamp(applied_command[1], -max_vy, max_vy);
              applied_command[2] = std::clamp(applied_command[2], -max_yaw, max_yaw);

              const bool hold_completed_test = test_failed && !pose_hold_active;
              const bool hold_manual_idle =
                  !test_mode && requested_command_idle &&
                  !locomotion_active &&
                  (!state.has_position || !state.has_velocity);

              if (hold_completed_test || hold_manual_idle) {
                if (!idle_hold_active) {
                  idle_start_q = state.q;
                  idle_transition_step = 0;
                  idle_hold_active = true;
                }
                if (locomotion_active) {
                  policy->reset();
                  locomotion_active = false;
                  policy_step = 0;
                }
                applied_command = {};
                inference_ms = 0.0;
                max_abs_action = 0.0F;
                if (test_failed) {
                  stage = Stage::kTestFailed;
                } else {
                  stage = Stage::kIdle;
                }
                float idle_alpha = 1.0F;
                if (idle_transition_frames > 0) {
                  idle_alpha = std::clamp(
                      static_cast<float>(idle_transition_step + 1) /
                          static_cast<float>(idle_transition_frames),
                      0.0F, 1.0F);
                  idle_alpha = idle_alpha * idle_alpha * (3.0F - 2.0F * idle_alpha);
                  if (idle_transition_step < idle_transition_frames) {
                    ++idle_transition_step;
                  }
                }
                for (int joint = 0; joint < kActionCount; ++joint) {
                  const float target =
                      idle_start_q[joint] +
                      idle_alpha * (kDefaultJointPosition[joint] - idle_start_q[joint]);
                  set_joint_target(command, joint, target,
                                   kJointKp[joint], kJointKd[joint]);
                }
              } else {
                idle_hold_active = false;
                idle_transition_step = 0;
                if (!locomotion_active) {
                  policy->reset();
                  policy_step = 0;
                  locomotion_active = true;
                  std::cout << "Locomotion policy memory reset.\n";
                }
                stage = test_failed
                            ? Stage::kTestFailed
                            : (test_completed
                                   ? Stage::kComplete
                                   : (test_stopping
                                          ? Stage::kStopping
                                          : (pose_hold_active ? Stage::kHolding
                                                              : Stage::kPolicy)));
                const float active_policy_time =
                    static_cast<float>(policy_step + 1) * kPolicyDt;
                const float phase =
                    std::fmod(active_policy_time, kGaitPeriod) / kGaitPeriod;
                const auto obs = policy->observation(state, applied_command, phase);
                const auto action = policy->infer(obs, &inference_ms);
                ++policy_step;
                max_abs_action = 0.0F;
                for (int joint = 0; joint < kActionCount; ++joint) {
                  max_abs_action = std::max(max_abs_action, std::abs(action[joint]));
                  const float target =
                      kDefaultJointPosition[joint] + action_scale * action[joint];
                  set_joint_target(command, joint, target, kJointKp[joint],
                                   kJointKd[joint]);
                }
                if (test_completed && !test_success_announced) {
                  const bool pose_stable =
                      pose_hold_active && hold_distance <= hold_position_tolerance &&
                      std::abs(hold_yaw_error) <= hold_yaw_tolerance &&
                      state.position[2] >= test_start_z - 0.05F &&
                      std::abs(state.rpy[0]) <= 0.25F &&
                      std::abs(state.rpy[1]) <= 0.25F;
                  test_post_hold_stable_frames = pose_stable
                                                     ? test_post_hold_stable_frames + 1
                                                     : 0;
                  if (test_post_hold_stable_frames >= test_post_hold_frames) {
                    test_success_announced = true;
                    std::cout << "STAIR_SUCCESS xy=[" << state.position[0] << ','
                              << state.position[1] << "] z=" << state.position[2]
                              << " peak_z=" << test_peak_z
                              << " max_abs_y=" << test_max_abs_y << '\n';
                  }
                }
              }
            }
          }
        }
      }

      publisher.Send(command);
      if (stand_command_ready && !stand_command_announced) {
        std::cout << "STAND_COMMAND_ACTIVE\n";
        stand_command_announced = true;
      }
      if (stage != reported_stage) {
        std::cout << "stage=" << stage_name(stage) << '\n';
        reported_stage = stage;
      }
      if (now >= next_report) {
        next_report = now + std::chrono::seconds(1);
        const double state_age_ms = state.valid
                                        ? std::chrono::duration<double, std::milli>(
                                              now - state.updated_at)
                                              .count()
                                        : -1.0;
        std::cout << std::fixed << std::setprecision(3)
                  << "stage=" << stage_name(stage) << " state_age_ms=" << state_age_ms
                  << " seq=" << state.sequence << " policy_step=" << policy_step
                  << " cmd=[" << applied_command[0] << ',' << applied_command[1] << ','
                  << applied_command[2] << "] z="
                  << (state.has_position ? state.position[2] : -1.0F)
                  << " xy=[" << (state.has_position ? state.position[0] : 0.0F) << ','
                  << (state.has_position ? state.position[1] : 0.0F) << ']'
                  << " vel=[" << (state.has_velocity ? state.velocity[0] : 0.0F) << ','
                  << (state.has_velocity ? state.velocity[1] : 0.0F) << ']'
                  << " rpy=[" << state.rpy[0] << ',' << state.rpy[1] << ',' << state.rpy[2]
                  << "] inference_ms=" << inference_ms
                  << " max_action=" << max_abs_action << " benchmark=["
                  << (test_reached_top ? 1 : 0) << ',' << (test_descended ? 1 : 0)
                  << "]\n";
      }

      next_publish += std::chrono::milliseconds(2);
      if (next_publish <= Clock::now()) next_publish = Clock::now() + std::chrono::milliseconds(2);
      std::this_thread::sleep_until(next_publish);
    }
  } catch (const std::exception& error) {
    std::cerr << "G1 bridge stopped after an error: " << error.what() << '\n';
    result = 1;
  }

  running.store(false);
  if (lcm_thread.joinable()) lcm_thread.join();
  eCAL::Finalize();
  return result;
}
