#include "Controller.h"
#include "benchmark_timer.h"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

// 全局性能监控器（在 benchmark_main.cpp 中定义）
extern benchmark::PerformanceMonitor g_monitor;

Controller::Controller(const std::string &net_interface)
{
    // Load config from g2w.yaml using absolute path
    YAML::Node yaml_node = YAML::LoadFile("/home/jackie/DreamWaQ_Go2W/deploy/deploy_real/configs/g2w.yaml");

    control_dt = yaml_node["control_dt"].as<float>();
    
    auto idx_vec = yaml_node["joint2motor_idx"].as<std::vector<int>>();
    for (int i = 0; i < 16; i++) joint2motor_idx[i] = idx_vec[i];
    
    auto kps_vec = yaml_node["kps"].as<std::vector<float>>();
    auto kds_vec = yaml_node["kds"].as<std::vector<float>>();
    for (int i = 0; i < 16; i++) { kps[i] = kps_vec[i]; kds[i] = kds_vec[i]; }
    
    auto default_real_vec = yaml_node["default_real_angles"].as<std::vector<float>>();
    auto default_sim_vec = yaml_node["default_sim_angles"].as<std::vector<float>>();
    for (int i = 0; i < 16; i++) { 
        default_real_angles[i] = default_real_vec[i]; 
        default_sim_angles[i] = default_sim_vec[i]; 
    }
    
    auto wheel_sim_vec = yaml_node["wheel_sim_indices"].as<std::vector<int>>();
    for (size_t i = 0; i < wheel_sim_vec.size() && i < 4; i++) 
        wheel_sim_indices[i] = wheel_sim_vec[i];

    ang_vel_scale = yaml_node["ang_vel_scale"].as<float>();
    dof_err_scale = yaml_node["dof_err_scale"].as<float>();
    dof_vel_scale = yaml_node["dof_vel_scale"].as<float>();
    action_scale = yaml_node["action_scale"].as<float>();
    
    auto cmd_scale_vec = yaml_node["cmd_scale"].as<std::vector<float>>();
    for (int i = 0; i < 3; i++) cmd_scale[i] = cmd_scale_vec[i];
    
    num_actions = yaml_node["num_actions"].as<int>();
    num_obs = yaml_node["num_obs"].as<int>();

    // Initialize state variables (matching Python __init__)
    qj.fill(0.0f);
    dqj.fill(0.0f);
    action.fill(0.0f);
    obs.setZero(num_obs);              // 73 dimensions
    obs_hist_buf.setZero(num_obs * 5); // 365 dimensions (5 frames)
    cmd.fill(0.0f);
    counter = 0;

    // Load DreamWaQ model modules (matching Python)
    std::string model_path = "/home/jackie/DreamWaQ_Go2W/deploy/pre_train/g2wDWAQ/";
    actor = torch::jit::load(model_path + "actor_dwaq.pt");
    encoder = torch::jit::load(model_path + "encoder_dwaq.pt");
    latent_mu = torch::jit::load(model_path + "latent_mu_dwaq.pt");
    latent_var = torch::jit::load(model_path + "latent_var_dwaq.pt");
    vel_mu = torch::jit::load(model_path + "vel_mu_dwaq.pt");
    vel_var = torch::jit::load(model_path + "vel_var_dwaq.pt");

    // Initialize DDS communication
    unitree::robot::ChannelFactory::Instance()->Init(0, net_interface);

    lowcmd_publisher.reset(new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowstate_subscriber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));

    lowcmd_publisher->InitChannel();
    lowstate_subscriber->InitChannel(std::bind(&Controller::low_state_message_handler, this, std::placeholders::_1), 1);

    // Wait for low state connection (matching Python wait_for_low_state)
    while (!mLowStateBuf.GetDataPtr())
    {
        usleep(100000);
    }
    std::cout << "Successfully connected to the robot." << std::endl;

    // Initialize low_cmd (matching Python init_cmd_go) - CALLED ONCE
    init_cmd_go();

    // Release motion control mode (matching Python MotionSwitcherClient)
    unitree::robot::b2::MotionSwitcherClient msc;
    msc.SetTimeout(10.0f);
    msc.Init();
    
    std::string robotForm, motionName;
    msc.CheckMode(robotForm, motionName);
    while (!motionName.empty())
    {
        std::cout << "Releasing motion control mode: " << motionName << std::endl;
        msc.ReleaseMode();
        sleep(5);
        msc.CheckMode(robotForm, motionName);
    }
    std::cout << "Motion control mode released." << std::endl;

    // Create background thread for command publishing (2ms interval, like G1)
    low_cmd_write_thread_ptr = unitree::common::CreateRecurrentThreadEx(
        "low_cmd_write", UT_CPU_ID_NONE, 2000, &Controller::low_cmd_write_handler, this);

    std::cout << "Controller init done!" << std::endl;
}

// Matching Python init_cmd_go EXACTLY
void Controller::init_cmd_go()
{
    std::lock_guard<std::mutex> lock(cmd_mutex); // 加锁（虽然此时线程可能未启动，但为了安全建议加上）
    
    // 建议增加：清零整个结构体，防止内存垃圾影响 CRC
    std::memset(&low_cmd, 0, sizeof(low_cmd));

    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;
    
    const float PosStopF = 2.146e9f;
    const float VelStopF = 16000.0f;
    
    for (int i = 0; i < 20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = 0x01;
        low_cmd.motor_cmd()[i].q() = PosStopF;
        low_cmd.motor_cmd()[i].dq() = VelStopF;
        low_cmd.motor_cmd()[i].kp() = 0;
        low_cmd.motor_cmd()[i].kd() = 0;
        low_cmd.motor_cmd()[i].tau() = 0;
    }
}

// Matching Python trans_r2s exactly
std::array<float, 16> Controller::trans_r2s(const std::array<float, 16>& qj)
{
    std::array<float, 16> tmp = qj;
    
    tmp[0] = qj[3];  tmp[1] = qj[4];  tmp[2] = qj[5];  tmp[3] = qj[13];
    tmp[4] = qj[0];  tmp[5] = qj[1];  tmp[6] = qj[2];  tmp[7] = qj[12];
    tmp[8] = qj[9];  tmp[9] = qj[10]; tmp[10] = qj[11]; tmp[11] = qj[15];
    tmp[12] = qj[6]; tmp[13] = qj[7]; tmp[14] = qj[8];  tmp[15] = qj[14];
    
    return tmp;
}

// Matching Python trans_s2r exactly
std::array<float, 16> Controller::trans_s2r(const std::array<float, 16>& qj)
{
    std::array<float, 16> tmp = qj;
    
    tmp[3] = qj[0];  tmp[4] = qj[1];  tmp[5] = qj[2];  tmp[13] = qj[3];
    tmp[0] = qj[4];  tmp[1] = qj[5];  tmp[2] = qj[6];  tmp[12] = qj[7];
    tmp[9] = qj[8];  tmp[10] = qj[9]; tmp[11] = qj[10]; tmp[15] = qj[11];
    tmp[6] = qj[12]; tmp[7] = qj[13]; tmp[8] = qj[14];  tmp[14] = qj[15];
    
    return tmp;
}

// Matching Python get_gravity_orientation exactly
std::array<float, 3> Controller::get_gravity_orientation(const std::array<float, 4>& quat)
{
    float qw = quat[0];
    float qx = quat[1];
    float qy = quat[2];
    float qz = quat[3];
    
    std::array<float, 3> gravity;
    gravity[0] = 2.0f * (-qz * qx + qw * qy);
    gravity[1] = -2.0f * (qz * qy + qw * qx);
    gravity[2] = 1.0f - 2.0f * (qw * qw + qz * qz);
    
    return gravity;
}

// Matching Python reparameterise exactly
torch::Tensor Controller::reparameterise(torch::Tensor mean, torch::Tensor logvar)
{
    torch::Tensor var = torch::exp(logvar * 0.5f);
    torch::Tensor code_temp = torch::randn_like(var);
    return mean + var * code_temp;
}

// Matching Python create_zero_cmd - does NOT modify mode
void Controller::create_zero_cmd()
{
    std::lock_guard<std::mutex> lock(cmd_mutex); // 加锁

    for (int i = 0; i < 20; i++)
    {
        low_cmd.motor_cmd()[i].q() = 0;
        low_cmd.motor_cmd()[i].dq() = 0;
        low_cmd.motor_cmd()[i].kp() = 0;
        low_cmd.motor_cmd()[i].kd() = 0;
        low_cmd.motor_cmd()[i].tau() = 0;
    }
}

// Matching Python create_damping_cmd - does NOT modify mode
void Controller::create_damping_cmd()
{
    std::lock_guard<std::mutex> lock(cmd_mutex); // 加锁

    for (int i = 0; i < 20; i++)
    {
        low_cmd.motor_cmd()[i].q() = 0;
        low_cmd.motor_cmd()[i].dq() = 0;
        low_cmd.motor_cmd()[i].kp() = 0;
        low_cmd.motor_cmd()[i].kd() = 8;
        low_cmd.motor_cmd()[i].tau() = 0;
    }
}

// Matching Python zero_torque_state exactly
void Controller::zero_torque_state()
{
    std::cout << "Enter zero torque state." << std::endl;
    std::cout << "Waiting for the start signal..." << std::endl;
    
    while (remote_controller.button[KeyMap::start] != 1)
    {
        create_zero_cmd();
        // Command is sent by background thread (low_cmd_write_handler)
        usleep(static_cast<useconds_t>(control_dt * 1000000));
    }
}

// Matching Python move_to_default_pos exactly
void Controller::move_to_default_pos()
{
    std::cout << "Moving to default pos." << std::endl;
    
    float total_time = 1.0f;
    int num_step = static_cast<int>(total_time / control_dt);
    
    auto low_state = mLowStateBuf.GetDataPtr();
    std::array<float, 16> init_dof_pos;
    for (int i = 0; i < 16; i++)
    {
        init_dof_pos[i] = low_state->motor_state()[joint2motor_idx[i]].q();
    }
    
    for (int i = 0; i < num_step; i++)
    {
        float alpha = static_cast<float>(i) / num_step;
        
        // Only leg joints (first 12), matching Python: for j in range(12)
        for (int j = 0; j < 12; j++)
        {
            int motor_idx = joint2motor_idx[j];
            float target_pos = default_real_angles[j];
            low_cmd.motor_cmd()[motor_idx].q() = init_dof_pos[j] * (1 - alpha) + target_pos * alpha;
            low_cmd.motor_cmd()[motor_idx].dq() = 0;
            low_cmd.motor_cmd()[motor_idx].kp() = 80;  // Matching Python
            low_cmd.motor_cmd()[motor_idx].kd() = 5;   // Matching Python
            low_cmd.motor_cmd()[motor_idx].tau() = 0;
        }
        
        // Command is sent by background thread
        usleep(static_cast<useconds_t>(control_dt * 1000000));
    }
}

// Matching Python default_pos_state exactly
void Controller::default_pos_state()
{
    std::cout << "Enter default pos state." << std::endl;
    std::cout << "Waiting for the Button A signal..." << std::endl;
    
    while (remote_controller.button[KeyMap::A] != 1)
    {
        // Leg joints only (first 12), matching Python
        for (int i = 0; i < 12; i++)
        {
            int motor_idx = joint2motor_idx[i];
            low_cmd.motor_cmd()[motor_idx].q() = default_real_angles[i];
            low_cmd.motor_cmd()[motor_idx].dq() = 0;
            low_cmd.motor_cmd()[motor_idx].kp() = 70.0f;  // Matching Python
            low_cmd.motor_cmd()[motor_idx].kd() = 5.0f;   // Matching Python
            low_cmd.motor_cmd()[motor_idx].tau() = 0;
        }
        
        // Command is sent by background thread
        usleep(static_cast<useconds_t>(control_dt * 1000000));
    }
}

// Matching Python run() exactly - WITH BENCHMARK TIMING
void Controller::run()
{
    g_monitor.startLoop();  // 开始循环计时
    counter++;
    
    auto low_state = mLowStateBuf.GetDataPtr();
    
    // Get motor data (matching Python)
    for (int i = 0; i < 16; i++)
    {
        qj[i] = low_state->motor_state()[joint2motor_idx[i]].q();
        dqj[i] = low_state->motor_state()[joint2motor_idx[i]].dq();
    }
    
    // Transform to sim order (matching Python)
    auto qj_sim = trans_r2s(qj);
    auto dqj_sim = trans_r2s(dqj);
    auto action_sim = trans_r2s(action);
    
    // Angular velocity (matching Python)
    std::array<float, 3> ang_vel;
    for (int i = 0; i < 3; i++)
    {
        ang_vel[i] = low_state->imu_state().gyroscope()[i];
    }
    std::array<float, 3> ang_vel_obs;
    for (int i = 0; i < 3; i++)
    {
        ang_vel_obs[i] = ang_vel[i] * ang_vel_scale;
    }
    
    // Gravity orientation (matching Python)
    std::array<float, 4> quat;
    for (int i = 0; i < 4; i++)
    {
        quat[i] = low_state->imu_state().quaternion()[i];
    }
    auto gravity_orientation = get_gravity_orientation(quat);
    
    // Command from joystick (matching Python)
    cmd[0] = remote_controller.ly;           // 前后速度
    cmd[1] = remote_controller.lx * -1.0f;   // 横向速度
    cmd[2] = remote_controller.rx * -1.0f;   // 旋转速度
    
    // Joint error (matching Python)
    std::array<float, 16> err_obs;
    for (int i = 0; i < 16; i++)
    {
        err_obs[i] = qj_sim[i] - default_sim_angles[i];
    }
    // Zero out wheel indices
    for (int i = 0; i < 4; i++)
    {
        err_obs[wheel_sim_indices[i]] = 0.0f;
    }
    // Scale
    for (int i = 0; i < 16; i++)
    {
        err_obs[i] *= dof_err_scale;
    }
    
    // Joint velocity
    std::array<float, 16> dqj_obs;
    for (int i = 0; i < 16; i++)
    {
        dqj_obs[i] = dqj_sim[i] * dof_vel_scale;
    }
    
    // Joint position
    std::array<float, 16> qj_obs = qj_sim;
    for (int i = 0; i < 4; i++)
    {
        qj_obs[wheel_sim_indices[i]] = 0.0f;
    }
    
    // Build observation vector (matching Python exactly)
    obs(0) = ang_vel_obs[0];
    obs(1) = ang_vel_obs[1];
    obs(2) = ang_vel_obs[2];
    
    obs(3) = gravity_orientation[0];
    obs(4) = gravity_orientation[1];
    obs(5) = gravity_orientation[2];
    
    obs(6) = cmd[0] * cmd_scale[0];
    obs(7) = cmd[1] * cmd_scale[1];
    obs(8) = cmd[2] * cmd_scale[2];
    
    for (int i = 0; i < num_actions; i++)
    {
        obs(9 + i) = err_obs[i];
    }
    
    for (int i = 0; i < num_actions; i++)
    {
        obs(9 + num_actions + i) = dqj_obs[i];
    }
    
    for (int i = 0; i < num_actions; i++)
    {
        obs(9 + num_actions * 2 + i) = qj_obs[i];
    }
    
    for (int i = 0; i < num_actions; i++)
    {
        obs(9 + num_actions * 3 + i) = action_sim[i];
    }
    
    // Update history buffer (matching Python)
    for (int i = 0; i < num_obs * 4; i++)
    {
        obs_hist_buf(i) = obs_hist_buf(i + num_obs);
    }
    for (int i = 0; i < num_obs; i++)
    {
        obs_hist_buf(num_obs * 4 + i) = obs(i);
    }
    
    // DreamWaQ forward pass (matching Python exactly) - WITH TIMING
    g_monitor.startInference();  // 开始推理计时
    
    g_monitor.startEncoder();  // 开始 encoder 计时
    torch::Tensor obs_hist_tensor = torch::from_blob(obs_hist_buf.data(), {obs_hist_buf.size()}, torch::kFloat).clone();
    
    std::vector<torch::jit::IValue> encoder_inputs;
    encoder_inputs.push_back(obs_hist_tensor);
    torch::Tensor h = encoder.forward(encoder_inputs).toTensor();
    g_monitor.endEncoder();  // 结束 encoder 计时
    
    std::vector<torch::jit::IValue> h_inputs;
    h_inputs.push_back(h);
    
    torch::Tensor vel_mu_out = vel_mu.forward(h_inputs).toTensor();
    torch::Tensor vel_var_out = vel_var.forward(h_inputs).toTensor();
    torch::Tensor latent_mu_out = latent_mu.forward(h_inputs).toTensor();
    torch::Tensor latent_var_out = latent_var.forward(h_inputs).toTensor();
    
    torch::Tensor vel = reparameterise(vel_mu_out, vel_var_out);
    torch::Tensor latent = reparameterise(latent_mu_out, latent_var_out);
    
    torch::Tensor code = torch::cat({vel, latent}, -1);
    
    torch::Tensor obs_tensor = torch::from_blob(obs.data(), {obs.size()}, torch::kFloat).clone();
    torch::Tensor obs_all = torch::cat({code, obs_tensor}, -1);
    
    g_monitor.startActor();  // 开始 actor 计时
    std::vector<torch::jit::IValue> actor_inputs;
    actor_inputs.push_back(obs_all);
    torch::Tensor action_tensor = actor.forward(actor_inputs).toTensor();
    g_monitor.endActor();  // 结束 actor 计时
    
    g_monitor.endInference();  // 结束推理计时
    
    // Copy action output
    std::array<float, 16> action_sim_out;
    std::memcpy(action_sim_out.data(), action_tensor.data_ptr<float>(), 16 * sizeof(float));
    
    // Transform back to real order (matching Python)
    qj = trans_s2r(qj_sim);
    action = trans_s2r(action_sim_out);
    dqj = trans_s2r(dqj_sim);
    
    // 用于控制质量测量
    std::array<float, 12> target_positions;
    std::array<float, 12> actual_positions;
    
    {
        std::lock_guard<std::mutex> lock(cmd_mutex); // 加锁：只保护数据写入
        
        // Send motor commands (matching Python exactly)
        for (int i = 0; i < 16; i++)
        {
            int motor_idx = joint2motor_idx[i];

            if (i >= 12)
            {
                // Wheel: velocity control (matching Python)
                low_cmd.motor_cmd()[motor_idx].q() = 0.0f;
                low_cmd.motor_cmd()[motor_idx].dq() = action[i] * 10.0f;
                low_cmd.motor_cmd()[motor_idx].kp() = 0.0f;
                low_cmd.motor_cmd()[motor_idx].kd() = kds[i];
                low_cmd.motor_cmd()[motor_idx].tau() = 0.0f;
            }
            else
            {
                // Leg: position control (matching Python)
                float target_pos = default_real_angles[i] + action[i] * action_scale;
                low_cmd.motor_cmd()[motor_idx].q() = target_pos;
                low_cmd.motor_cmd()[motor_idx].dq() = 0.0f;
                low_cmd.motor_cmd()[motor_idx].kp() = kps[i];
                low_cmd.motor_cmd()[motor_idx].kd() = kds[i];
                low_cmd.motor_cmd()[motor_idx].tau() = 0.0f;

                // 记录控制质量数据
                target_positions[i] = target_pos;
                actual_positions[i] = qj[i];
            }
        }
    }
    
    
    // 记录控制质量（匹配 Python）
    g_monitor.recordControlQuality(target_positions.data(), actual_positions.data(), 12);
    
    g_monitor.endLoop();  // 结束循环计时（在 sleep 之前！与 Python 一致）
    
    // Command is sent by background thread
    usleep(static_cast<useconds_t>(control_dt * 1000000));
}

void Controller::damp()
{
    std::cout << "Entering damp mode..." << std::endl;
    
    // Get current position
    auto low_state = mLowStateBuf.GetDataPtr();
    std::array<float, 16> current_pos;
    for (int i = 0; i < 16; i++)
    {
        current_pos[i] = low_state->motor_state()[joint2motor_idx[i]].q();
    }
    
    // Gradually reduce kp to allow smooth settling (1 second)
    int num_steps = 50;
    for (int step = 0; step < num_steps; step++)
    {
        float alpha = 1.0f - static_cast<float>(step) / num_steps;  // 1.0 -> 0.0
        
        for (int i = 0; i < 12; i++)  // Leg joints only
        {
            int motor_idx = joint2motor_idx[i];
            low_cmd.motor_cmd()[motor_idx].q() = current_pos[i];
            low_cmd.motor_cmd()[motor_idx].dq() = 0;
            low_cmd.motor_cmd()[motor_idx].kp() = kps[i] * alpha;  // Gradually reduce
            low_cmd.motor_cmd()[motor_idx].kd() = 5.0f;
            low_cmd.motor_cmd()[motor_idx].tau() = 0;
        }
        
        usleep(static_cast<useconds_t>(control_dt * 1000000));
    }
    
    // Final: enter full damp mode
    create_damping_cmd();
    usleep(200000);
    
    std::cout << "Exit" << std::endl;
}

void Controller::low_state_message_handler(const void *message)
{
    unitree_go::msg::dds_::LowState_* ptr = (unitree_go::msg::dds_::LowState_*)message;
    mLowStateBuf.SetData(*ptr);
    
    // Parse wireless_remote using RemoteController (matching Python exactly)
    std::vector<uint8_t> wireless_data(ptr->wireless_remote().begin(), ptr->wireless_remote().end());
    remote_controller.set(wireless_data);
}

// Background thread: sends command every 2ms (matching G1 pattern)
void Controller::low_cmd_write_handler()
{
    low_cmd.crc() = crc32_core((uint32_t*)(&low_cmd), (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(low_cmd);
}

// CRC32 calculation (matching SDK)
uint32_t Controller::crc32_core(uint32_t* ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}
