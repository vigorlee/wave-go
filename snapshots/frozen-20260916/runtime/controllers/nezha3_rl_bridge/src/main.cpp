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
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "gamepad_lcmt.hpp"

namespace {
using Clock = std::chrono::steady_clock;
constexpr int kDofs = 16;
constexpr int kObs = 57;
constexpr int kHistoryFrames = 5;
constexpr int kHistory = kObs * kHistoryFrames;
constexpr std::array<float, kDofs> kDefault = {
    0.1F, 0.925F, -1.85F, 0.0F, -0.1F, 0.925F, -1.85F, 0.0F,
    0.1F, 0.925F, -1.85F, 0.0F, -0.1F, 0.925F, -1.85F, 0.0F};
constexpr std::array<float, 4> kKp = {150.0F, 220.0F, 220.0F, 0.0F};
constexpr std::array<float, 4> kKd = {4.0F, 4.0F, 4.0F, 1.2F};

std::atomic<bool> running{true};
void stop_handler(int) { running.store(false); }

double env_double(const char* name, double fallback) {
  const char* text = std::getenv(name);
  if (!text || !*text) return fallback;
  char* end = nullptr;
  const double value = std::strtod(text, &end);
  return end != text && *end == '\0' ? value : fallback;
}

std::string env_string(const char* name, const std::string& fallback) {
  const char* text = std::getenv(name);
  return text && *text ? text : fallback;
}

bool env_bool(const char* name, bool fallback = false) {
  const char* text = std::getenv(name);
  if (!text || !*text) return fallback;
  const std::string value(text);
  return value == "1" || value == "true" || value == "TRUE" || value == "yes";
}

class VelocityReceiver {
 public:
  void handle(const lcm::ReceiveBuffer*, const std::string&, const gamepad_lcmt* msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    command_ = {msg->leftStickAnalog[1], msg->leftStickAnalog[0],
                -msg->rightStickAnalog[0]};
    updated_ = Clock::now();
    received_ = true;
  }
  std::array<float, 3> command() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!received_ || Clock::now() - updated_ > std::chrono::milliseconds(300)) return {};
    return command_;
  }
 private:
  mutable std::mutex mutex_;
  std::array<float, 3> command_{};
  Clock::time_point updated_{};
  bool received_{false};
};

struct Snapshot {
  bool valid{false};
  Clock::time_point updated{};
  std::array<float, kDofs> q{}, dq{};
  std::array<float, 4> quat{1, 0, 0, 0};
  std::array<float, 3> gyro{}, position{}, velocity{}, rpy{};
  bool has_velocity{false};
};

class StateReceiver {
 public:
  void handle(const robot_sdk::pb::RobotState& msg) {
    if (msg.q_abad_size() < 4 || msg.q_hip_size() < 4 || msg.q_knee_size() < 4 ||
        msg.q_foot_size() < 4 || msg.qd_abad_size() < 4 || msg.qd_hip_size() < 4 ||
        msg.qd_knee_size() < 4 || msg.qd_foot_size() < 4 || msg.quat_size() < 4 ||
        msg.gyro_size() < 3) return;
    Snapshot next;
    constexpr std::array<int,4> sdk_to_policy = {1, 0, 3, 2};
    for (int leg = 0; leg < 4; ++leg) {
      const int i = sdk_to_policy[leg] * 4;
      next.q[i] = msg.q_abad(leg); next.q[i+1] = msg.q_hip(leg);
      next.q[i+2] = msg.q_knee(leg); next.q[i+3] = msg.q_foot(leg);
      next.dq[i] = msg.qd_abad(leg); next.dq[i+1] = msg.qd_hip(leg);
      next.dq[i+2] = msg.qd_knee(leg); next.dq[i+3] = msg.qd_foot(leg);
    }
    for (int i = 0; i < 4; ++i) next.quat[i] = msg.quat(i);
    for (int i = 0; i < 3; ++i) {
      next.gyro[i] = msg.gyro(i);
      if (msg.position_size() >= 3) next.position[i] = msg.position(i);
      if (msg.v_world_size() >= 3) next.velocity[i] = msg.v_world(i);
    }
    next.has_velocity = msg.v_world_size() >= 3;
    const float w=next.quat[0], x=next.quat[1], y=next.quat[2], z=next.quat[3];
    next.rpy[0] = std::atan2(2*(w*x+y*z), 1-2*(x*x+y*y));
    next.rpy[1] = std::asin(std::clamp(2*(w*y-z*x), -1.0F, 1.0F));
    next.rpy[2] = std::atan2(2*(w*z+x*y), 1-2*(y*y+z*z));
    next.updated = Clock::now(); next.valid = true;
    std::lock_guard<std::mutex> lock(mutex_); snapshot_ = next;
  }
  Snapshot get() const { std::lock_guard<std::mutex> lock(mutex_); return snapshot_; }
 private:
  mutable std::mutex mutex_;
  Snapshot snapshot_;
};

void add_four(google::protobuf::RepeatedField<float>* field) {
  for (int i=0; i<4; ++i) field->Add(0.0F);
}
robot_sdk::pb::RobotCmd make_command() {
  robot_sdk::pb::RobotCmd cmd;
  add_four(cmd.mutable_q_des_abad()); add_four(cmd.mutable_q_des_hip());
  add_four(cmd.mutable_q_des_knee()); add_four(cmd.mutable_q_des_foot());
  add_four(cmd.mutable_qd_des_abad()); add_four(cmd.mutable_qd_des_hip());
  add_four(cmd.mutable_qd_des_knee()); add_four(cmd.mutable_qd_des_foot());
  add_four(cmd.mutable_kp_abad()); add_four(cmd.mutable_kp_hip());
  add_four(cmd.mutable_kp_knee()); add_four(cmd.mutable_kp_foot());
  add_four(cmd.mutable_kd_abad()); add_four(cmd.mutable_kd_hip());
  add_four(cmd.mutable_kd_knee()); add_four(cmd.mutable_kd_foot());
  add_four(cmd.mutable_tau_abad_ff()); add_four(cmd.mutable_tau_hip_ff());
  add_four(cmd.mutable_tau_knee_ff()); add_four(cmd.mutable_tau_foot_ff());
  return cmd;
}
void set_target(robot_sdk::pb::RobotCmd& cmd, int leg, const std::array<float,kDofs>& q,
                const std::array<float,kDofs>& wheel_vel) {
  constexpr std::array<int,4> sdk_to_policy = {1, 0, 3, 2};
  const int i=sdk_to_policy[leg]*4;
  cmd.set_q_des_abad(leg,q[i]); cmd.set_q_des_hip(leg,q[i+1]);
  cmd.set_q_des_knee(leg,q[i+2]); cmd.set_q_des_foot(leg,0);
  cmd.set_qd_des_abad(leg,0); cmd.set_qd_des_hip(leg,0); cmd.set_qd_des_knee(leg,0);
  cmd.set_qd_des_foot(leg,wheel_vel[i+3]);
  cmd.set_kp_abad(leg,kKp[0]); cmd.set_kp_hip(leg,kKp[1]);
  cmd.set_kp_knee(leg,kKp[2]); cmd.set_kp_foot(leg,kKp[3]);
  cmd.set_kd_abad(leg,kKd[0]); cmd.set_kd_hip(leg,kKd[1]);
  cmd.set_kd_knee(leg,kKd[2]); cmd.set_kd_foot(leg,kKd[3]);
}

std::array<float,3> gravity(const std::array<float,4>& q) {
  const float w=q[0], x=-q[1], y=-q[2], z=-q[3];
  return {-2*(x*z+w*y), -2*(y*z-w*x), -(w*w-x*x-y*y+z*z)};
}

class Policy {
 public:
  explicit Policy(const std::string& path) : model_(torch::jit::load(path, torch::kCPU)) {
    model_.eval();
  }
  std::array<float,kObs> observation(const Snapshot& s, const std::array<float,3>& cmd) {
    std::array<float,kObs> obs{};
    const auto g=gravity(s.quat);
    for (int i=0;i<3;++i) obs[i]=s.gyro[i]*0.25F;
    for (int i=0;i<3;++i) obs[3+i]=g[i]*0.5F;
    obs[6]=cmd[0]*3.5F; obs[7]=cmd[1]; obs[8]=cmd[2]*0.7F;
    for (int i=0;i<kDofs;++i) {
      obs[9+i]=(i%4==3)?0.0F:s.q[i]-kDefault[i];
      obs[25+i]=s.dq[i]*0.05F;
      obs[41+i]=last_action_[i];
      obs[9+i]=std::clamp(obs[9+i],-100.0F,100.0F);
      obs[25+i]=std::clamp(obs[25+i],-100.0F,100.0F);
    }
    return obs;
  }
  void warm(const std::array<float,kObs>& obs) { append(obs); }
  std::array<float,kDofs> infer(const std::array<float,kObs>& obs) {
    torch::NoGradGuard guard;
    auto o=torch::from_blob(const_cast<float*>(obs.data()),{1,kObs},torch::kFloat32).clone();
    auto h=torch::from_blob(history_.data(),{1,kHistory},torch::kFloat32).clone();
    auto out=model_.forward({o,h}).toTensor().to(torch::kCPU).contiguous();
    if (out.numel()!=kDofs) throw std::runtime_error("policy returned non-16 action");
    std::memcpy(last_action_.data(),out.data_ptr<float>(),sizeof(float)*kDofs);
    for(float& a:last_action_) a=std::clamp(a,-100.0F,100.0F);
    append(obs);
    return last_action_;
  }
 private:
  void append(const std::array<float,kObs>& obs) {
    std::move(history_.begin()+kObs,history_.end(),history_.begin());
    std::copy(obs.begin(),obs.end(),history_.end()-kObs);
  }
  torch::jit::script::Module model_;
  std::array<float,kHistory> history_{};
  std::array<float,kDofs> last_action_{};
};
}  // namespace

int main(int argc,char** argv) {
  std::signal(SIGINT,stop_handler); std::signal(SIGTERM,stop_handler);
  torch::set_num_threads(1); torch::set_num_interop_threads(1);
  Policy policy(env_string("NEZHA3_RL_MODEL",DEFAULT_MODEL));
  const float test_vx=static_cast<float>(env_double("NEZHA3_RL_TEST_VX",0));
  const double test_delay=env_double("NEZHA3_RL_TEST_DELAY",1.0);
  const double stand_seconds=env_double("NEZHA3_RL_STAND_SECONDS",3.0);
  const bool abi_probe=env_bool("NEZHA3_RL_ABI_PROBE");
  const std::string success_axis=env_string("NEZHA3_RL_SUCCESS_AXIS","x");
  const float success_position=static_cast<float>(
      env_double("NEZHA3_RL_SUCCESS_POSITION",4.7));
  const float success_height=static_cast<float>(
      env_double("NEZHA3_RL_SUCCESS_HEIGHT",0.75));
  lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=255");
  VelocityReceiver velocity; lcm.subscribe("vel_cmd_lcm_data",&VelocityReceiver::handle,&velocity);
  std::thread lcm_thread([&]{while(running.load()) lcm.handleTimeout(100);});
  eCAL::Initialize(argc,argv,"nezha3_rl_bridge");
  eCAL::protobuf::CPublisher<robot_sdk::pb::RobotCmd> publisher("mujoco_cmd");
  eCAL::protobuf::CSubscriber<robot_sdk::pb::RobotState> subscriber("mujoco_state");
  StateReceiver states;
  std::atomic<bool> probe_received{false};
  subscriber.AddReceiveCallback([&](const char*,const robot_sdk::pb::RobotState& s,long long,long long,long long){
    if (abi_probe && !probe_received.exchange(true)) {
      auto print_field = [](const char* name, const google::protobuf::RepeatedField<float>& values) {
        std::cout << name << "=";
        for (int i=0; i<values.size(); ++i) std::cout << (i ? "," : "") << values.Get(i);
        std::cout << "\n";
      };
      print_field("q_abad",s.q_abad()); print_field("q_hip",s.q_hip());
      print_field("q_knee",s.q_knee()); print_field("q_foot",s.q_foot());
      print_field("qd_abad",s.qd_abad()); print_field("qd_hip",s.qd_hip());
      print_field("qd_knee",s.qd_knee()); print_field("qd_foot",s.qd_foot());
      std::cout.flush();
    }
    states.handle(s);
  });
  auto command=make_command();
  auto start=Clock::now(), next_policy=start, next_publish=start, next_report=start;
  std::array<float,kDofs> initial{}, targets=kDefault, wheels{};
  if (!abi_probe) for(int leg=0;leg<4;++leg)set_target(command,leg,targets,wheels);
  bool acquired=false, active=false, success=false; int warmup=0;
  std::cout<<"Nezha3 stair policy loaded; waiting for mujoco_state\n";
  while(running.load() && eCAL::Ok()) {
    const auto now=Clock::now(); const auto state=states.get();
    const bool fresh=state.valid && now-state.updated<std::chrono::milliseconds(250);
    if (abi_probe && probe_received.load()) break;
    if(fresh && !acquired){initial=state.q; start=now; acquired=true; std::cout<<"State acquired; standing\n";}
    if(fresh && acquired && now>=next_policy){
      next_policy=now+std::chrono::milliseconds(20);
      const double elapsed=std::chrono::duration<double>(now-start).count();
      if(elapsed<stand_seconds){
        float a=static_cast<float>(std::clamp(elapsed/stand_seconds,0.0,1.0)); a=a*a*(3-2*a);
        for(int i=0;i<kDofs;++i) targets[i]=(i%4==3)?0:initial[i]+a*(kDefault[i]-initial[i]);
      }else if(warmup<kHistoryFrames){policy.warm(policy.observation(state,{})); ++warmup; targets=kDefault;}
      else if(state.position[2]<0.15F || std::abs(state.rpy[0])>1.1F || std::abs(state.rpy[1])>1.1F){
        targets=state.q; wheels.fill(0); std::cout<<"STAIR_SAFETY z="<<state.position[2]<<"\n"; running=false;
      }else{
        if(!active){active=true; start=now; std::cout<<"Policy control enabled\n";}
        auto desired=velocity.command();
        if(test_vx!=0 && std::chrono::duration<double>(now-start).count()>=test_delay) desired[0]=test_vx;
        desired[0]=std::clamp(desired[0],-1.5F,1.5F); desired[1]=std::clamp(desired[1],-0.8F,0.8F); desired[2]=std::clamp(desired[2],-1.0F,1.0F);
        const auto action=policy.infer(policy.observation(state,desired));
        for(int i=0;i<kDofs;++i){if(i%4==3) wheels[i]=action[i]*10.0F; else targets[i]=kDefault[i]+action[i]*0.25F;}
        const float success_progress=
            success_axis=="y" ? state.position[1] : state.position[0];
        if(!success && success_progress>success_position && state.position[2]>success_height){
          success=true;
          std::cout<<"STAIR_SUCCESS x="<<state.position[0]<<" y="<<state.position[1]
                   <<" z="<<state.position[2]<<"\n";
        }
      }
      for(int leg=0;leg<4;++leg)set_target(command,leg,targets,wheels);
    }
    if(now>=next_publish){next_publish=now+std::chrono::milliseconds(2); publisher.Send(command);}
    if(fresh && now>=next_report){next_report=now+std::chrono::seconds(1); std::cout<<"state x="<<state.position[0]<<" y="<<state.position[1]<<" z="<<state.position[2]<<" roll="<<state.rpy[0]<<" pitch="<<state.rpy[1]<<" q="<<state.q[0]<<","<<state.q[1]<<","<<state.q[2]<<" wheel="<<wheels[3]<<","<<wheels[7]<<","<<wheels[11]<<","<<wheels[15]<<" active="<<active<<"\n";}
    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }
  running=false; if(lcm_thread.joinable())lcm_thread.join(); eCAL::Finalize();
  return success?0:1;
}
