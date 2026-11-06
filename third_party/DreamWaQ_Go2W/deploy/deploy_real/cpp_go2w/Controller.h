/**
 * @file Controller.h
 * @brief GO2W Robot RL Deployment Controller (DreamWaQ)
 */
#include <mutex>

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

#include "torch/script.h"
#include <eigen3/Eigen/Eigen>

#include "RemoteController.h"
#include "DataBuffer.h"
#include <string>
#include <array>

class Controller
{
public:
    Controller(const std::string &net_interface);
    
    // State machine functions
    void zero_torque_state();
    void move_to_default_pos();
    void default_pos_state();
    void run();
    void damp();
    
    bool isSelectPressed() const { return remote_controller.button[KeyMap::select] == 1; }

private:
    // Callbacks
    void low_state_message_handler(const void *message);
    void low_cmd_write_handler();
    
    // Command generation
    void init_cmd_go();
    void create_zero_cmd();
    void create_damping_cmd();
    
    // Coordinate transformation (real <-> sim joint order)
    std::array<float, 16> trans_r2s(const std::array<float, 16>& qj);
    std::array<float, 16> trans_s2r(const std::array<float, 16>& qj);
    
    // Utility functions
    std::array<float, 3> get_gravity_orientation(const std::array<float, 4>& quat);
    torch::Tensor reparameterise(torch::Tensor mean, torch::Tensor logvar);
    uint32_t crc32_core(uint32_t* ptr, uint32_t len);

    // Communication
    unitree::common::ThreadPtr low_cmd_write_thread_ptr;
    DataBuffer<unitree_go::msg::dds_::LowState_> mLowStateBuf;
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    unitree_go::msg::dds_::LowCmd_ low_cmd;
    RemoteController remote_controller;

    // Config (from g2w.yaml)
    float control_dt;
    std::array<int, 16> joint2motor_idx;
    std::array<float, 16> kps;
    std::array<float, 16> kds;
    std::array<float, 16> default_real_angles;
    std::array<float, 16> default_sim_angles;
    std::array<int, 4> wheel_sim_indices;
    float ang_vel_scale, dof_err_scale, dof_vel_scale, action_scale;
    std::array<float, 3> cmd_scale;
    int num_actions, num_obs;

    // State
    std::array<float, 16> qj, dqj, action;
    Eigen::VectorXf obs, obs_hist_buf;
    std::array<float, 3> cmd;
    int counter;

    // DreamWaQ models
    torch::jit::script::Module actor, encoder, latent_mu, latent_var, vel_mu, vel_var;

    // Mutex for low_cmd protection
    std::mutex cmd_mutex;
};

#endif
