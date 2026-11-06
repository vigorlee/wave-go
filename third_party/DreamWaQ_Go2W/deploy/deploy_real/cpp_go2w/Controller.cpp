/**
 * @file Controller.cpp
 * @brief GO2W 机器人 RL 部署控制器实现
 */

#include "Controller.h"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

// DDS 话题名称定义
#define TOPIC_LOWCMD "rt/lowcmd"      // 电机指令话题
#define TOPIC_LOWSTATE "rt/lowstate"  // 机器人状态话题

/**
 * 完成以下初始化工作：
 * 1. 从 YAML 配置文件加载参数
 * 2. 初始化状态变量
 * 3. 加载神经网络模型
 * 4. 建立 DDS 通信
 * 5. 释放内置运动控制
 * 6. 启动指令发送线程
 */
Controller::Controller(const std::string &net_interface)
{
    // ==================== 1. 加载配置文件 ====================
    YAML::Node yaml_node = YAML::LoadFile("/home/jackie/DreamWaQ_Go2W/deploy/deploy_real/configs/g2w.yaml");

    // 控制周期（秒）
    control_dt = yaml_node["control_dt"].as<float>();
    
    // 关节到电机索引映射（GO2W 有16个关节：12腿+4轮）
    auto idx_vec = yaml_node["joint2motor_idx"].as<std::vector<int>>();
    for (int i = 0; i < 16; i++) joint2motor_idx[i] = idx_vec[i];
    
    // PD 控制增益
    auto kps_vec = yaml_node["kps"].as<std::vector<float>>();
    auto kds_vec = yaml_node["kds"].as<std::vector<float>>();
    for (int i = 0; i < 16; i++) { kps[i] = kps_vec[i]; kds[i] = kds_vec[i]; }
    
    // 默认站立角度（实机顺序和仿真顺序）
    auto default_real_vec = yaml_node["default_real_angles"].as<std::vector<float>>();
    auto default_sim_vec = yaml_node["default_sim_angles"].as<std::vector<float>>();
    for (int i = 0; i < 16; i++) { 
        default_real_angles[i] = default_real_vec[i]; 
        default_sim_angles[i] = default_sim_vec[i]; 
    }
    
    // 轮子在仿真顺序中的索引（用于置零处理）
    auto wheel_sim_vec = yaml_node["wheel_sim_indices"].as<std::vector<int>>();
    for (size_t i = 0; i < wheel_sim_vec.size() && i < 4; i++) 
        wheel_sim_indices[i] = wheel_sim_vec[i];

    // 观测缩放因子
    ang_vel_scale = yaml_node["ang_vel_scale"].as<float>();  // 角速度缩放
    dof_err_scale = yaml_node["dof_err_scale"].as<float>();  // 关节误差缩放
    dof_vel_scale = yaml_node["dof_vel_scale"].as<float>();  // 关节速度缩放
    action_scale = yaml_node["action_scale"].as<float>();    // 动作缩放
    
    // 速度指令缩放 [vx, vy, omega]
    auto cmd_scale_vec = yaml_node["cmd_scale"].as<std::vector<float>>();
    for (int i = 0; i < 3; i++) cmd_scale[i] = cmd_scale_vec[i];
    
    // 动作和观测维度
    num_actions = yaml_node["num_actions"].as<int>();  // 16 (12腿+4轮)
    num_obs = yaml_node["num_obs"].as<int>();          // 73

    // ==================== 2. 初始化状态变量 ====================
    qj.fill(0.0f);      // 关节位置
    dqj.fill(0.0f);     // 关节速度
    action.fill(0.0f);  // 上一步动作
    obs.setZero(num_obs);              // 当前观测 (73维)
    obs_hist_buf.setZero(num_obs * 5); // 历史观测缓冲区 (5帧 x 73维 = 365维)
    cmd.fill(0.0f);     // 速度指令
    counter = 0;        // 周期计数器

    // ==================== 3. 加载 DreamWaQ 神经网络模型 ====================
    std::string model_path = "/home/jackie/DreamWaQ_Go2W/deploy/pre_train/g2wDWAQ/";
    actor = torch::jit::load(model_path + "actor_dwaq.pt");          // 策略网络
    encoder = torch::jit::load(model_path + "encoder_dwaq.pt");      // 历史编码器
    latent_mu = torch::jit::load(model_path + "latent_mu_dwaq.pt");  // 隐变量均值
    latent_var = torch::jit::load(model_path + "latent_var_dwaq.pt"); // 隐变量方差
    vel_mu = torch::jit::load(model_path + "vel_mu_dwaq.pt");        // 速度估计均值
    vel_var = torch::jit::load(model_path + "vel_var_dwaq.pt");      // 速度估计方差

    // ==================== 4. 初始化 DDS 通信 ====================
    unitree::robot::ChannelFactory::Instance()->Init(0, net_interface);

    // 创建发布器和订阅器
    lowcmd_publisher.reset(new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowstate_subscriber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));

    // 初始化通道
    lowcmd_publisher->InitChannel();
    lowstate_subscriber->InitChannel(std::bind(&Controller::low_state_message_handler, this, std::placeholders::_1), 1);

    // 等待 LowState 连接（阻塞直到收到第一条消息）
    while (!mLowStateBuf.GetDataPtr())
    {
        usleep(100000);  // 100ms
    }
    std::cout << "成功连接到机器人。" << std::endl;

    // 初始化电机指令结构（只调用一次）
    init_cmd_go();

    // ==================== 5. 释放内置运动控制模式 ====================
    // GO2W 默认启动时有内置运动控制，需要释放才能进行底层控制
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

    // ==================== 6. 启动指令发送线程 ====================
    // 每2ms发送一次电机指令，确保发送频率稳定
    low_cmd_write_thread_ptr = unitree::common::CreateRecurrentThreadEx(
        "low_cmd_write", UT_CPU_ID_NONE, 2000, &Controller::low_cmd_write_handler, this);

    std::cout << "Controller init done!" << std::endl;
}

/**
 * 设置 LowCmd 消息的头部和标志位，
 * 将所有电机设为安全的初始状态。
 */
void Controller::init_cmd_go()
{
    std::lock_guard<std::mutex> lock(cmd_mutex); // 加锁（虽然此时线程可能未启动，但为了安全建议加上）
    
    // 建议增加：清零整个结构体，防止内存垃圾影响 CRC
    std::memset(&low_cmd, 0, sizeof(low_cmd));

    // 消息头（固定格式）
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;  // 底层控制模式
    low_cmd.gpio() = 0;
    
    // 位置和速度停止标志（SDK 定义的特殊值）
    const float PosStopF = 2.146e9f;
    const float VelStopF = 16000.0f;
    
    // 初始化所有20个电机（GO2W 实际使用16个）
    for (int i = 0; i < 20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = 0x01;  // 力矩控制模式
        low_cmd.motor_cmd()[i].q() = PosStopF;
        low_cmd.motor_cmd()[i].dq() = VelStopF;
        low_cmd.motor_cmd()[i].kp() = 0;
        low_cmd.motor_cmd()[i].kd() = 0;
        low_cmd.motor_cmd()[i].tau() = 0;
    }
}

/**
 * 映射关系：将实机的腿和轮子顺序转换为仿真器的顺序
 */
std::array<float, 16> Controller::trans_r2s(const std::array<float, 16>& qj)
{
    std::array<float, 16> tmp = qj;
    
    // 腿部关节重排
    tmp[0] = qj[3];  tmp[1] = qj[4];  tmp[2] = qj[5];  tmp[3] = qj[13];
    tmp[4] = qj[0];  tmp[5] = qj[1];  tmp[6] = qj[2];  tmp[7] = qj[12];
    tmp[8] = qj[9];  tmp[9] = qj[10]; tmp[10] = qj[11]; tmp[11] = qj[15];
    tmp[12] = qj[6]; tmp[13] = qj[7]; tmp[14] = qj[8];  tmp[15] = qj[14];
    
    return tmp;
}

/**
 * 将神经网络输出的动作转换回实机电机顺序
 */
std::array<float, 16> Controller::trans_s2r(const std::array<float, 16>& qj)
{
    std::array<float, 16> tmp = qj;
    
    // 逆向映射
    tmp[3] = qj[0];  tmp[4] = qj[1];  tmp[5] = qj[2];  tmp[13] = qj[3];
    tmp[0] = qj[4];  tmp[1] = qj[5];  tmp[2] = qj[6];  tmp[12] = qj[7];
    tmp[9] = qj[8];  tmp[10] = qj[9]; tmp[11] = qj[10]; tmp[15] = qj[11];
    tmp[6] = qj[12]; tmp[7] = qj[13]; tmp[8] = qj[14];  tmp[14] = qj[15];
    
    return tmp;
}

/**
 * 将世界坐标系的重力向量 [0, 0, -1] 转换到机体坐标系
 * @param quat 机体姿态四元数 [w, x, y, z]
 * @return 机体坐标系下的重力方向 [gx, gy, gz]
 */
std::array<float, 3> Controller::get_gravity_orientation(const std::array<float, 4>& quat)
{
    float qw = quat[0];
    float qx = quat[1];
    float qy = quat[2];
    float qz = quat[3];
    
    // 使用四元数旋转公式计算
    std::array<float, 3> gravity;
    gravity[0] = 2.0f * (-qz * qx + qw * qy);
    gravity[1] = -2.0f * (qz * qy + qw * qx);
    gravity[2] = 1.0f - 2.0f * (qw * qw + qz * qz);
    
    return gravity;
}

/**
 * VAE 重参数化采样
 * z = mean + exp(logvar/2) * epsilon
 * 其中 epsilon ~ N(0, 1)
 * 
 * 这种采样方式保持梯度可传播。
 */
torch::Tensor Controller::reparameterise(torch::Tensor mean, torch::Tensor logvar)
{
    torch::Tensor var = torch::exp(logvar * 0.5f);      // 标准差
    torch::Tensor code_temp = torch::randn_like(var);    // 随机噪声
    return mean + var * code_temp;
}

/**
 * 将所有电机设为零力矩状态，机器人自由下垂。
 */
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

/**
 * 将所有电机设为纯阻尼模式（kd=8），
 */
void Controller::create_damping_cmd()
{
    std::lock_guard<std::mutex> lock(cmd_mutex); // 加锁

    for (int i = 0; i < 20; i++)
    {
        low_cmd.motor_cmd()[i].q() = 0;
        low_cmd.motor_cmd()[i].dq() = 0;
        low_cmd.motor_cmd()[i].kp() = 0;
        low_cmd.motor_cmd()[i].kd() = 8;  // 仅保留阻尼
        low_cmd.motor_cmd()[i].tau() = 0;
    }
}

/**
 * 机器人进入零力矩状态，所有关节自由。
 * 等待用户按下 Start 键后退出此状态。
 */
void Controller::zero_torque_state()
{
    std::cout << "进入零力矩状态。" << std::endl;
    std::cout << "等待 Start 键信号..." << std::endl;
    
    while (remote_controller.button[KeyMap::start] != 1)
    {
        create_zero_cmd();
        // 指令由后台线程发送
        usleep(static_cast<useconds_t>(control_dt * 1000000));
    }
}

/**
 * 从当前关节角度平滑插值到默认站立角度。
 */
void Controller::move_to_default_pos()
{
    std::cout << "Moving to default pos." << std::endl;
    
    float total_time = 1.0f;  // 过渡时间1秒
    int num_step = static_cast<int>(total_time / control_dt);
    
    // 获取当前关节角度
    auto low_state = mLowStateBuf.GetDataPtr();
    std::array<float, 16> init_dof_pos;
    for (int i = 0; i < 16; i++)
    {
        init_dof_pos[i] = low_state->motor_state()[joint2motor_idx[i]].q();
    }
    
    // 线性插值过渡
    for (int i = 0; i < num_step; i++)
    {
        float alpha = static_cast<float>(i) / num_step;  // 插值系数 0->1
        
        // 仅控制腿部关节（前12个），轮子保持不动
        for (int j = 0; j < 12; j++)
        {
            int motor_idx = joint2motor_idx[j];
            float target_pos = default_real_angles[j];
            // 位置插值：init * (1-alpha) + target * alpha
            low_cmd.motor_cmd()[motor_idx].q() = init_dof_pos[j] * (1 - alpha) + target_pos * alpha;
            low_cmd.motor_cmd()[motor_idx].dq() = 0;
            low_cmd.motor_cmd()[motor_idx].kp() = 80;  // 较高刚度确保平滑过渡
            low_cmd.motor_cmd()[motor_idx].kd() = 5;
            low_cmd.motor_cmd()[motor_idx].tau() = 0;
        }
        
        usleep(static_cast<useconds_t>(control_dt * 1000000));
    }
}

/**
 * 使用位置控制保持默认站立姿态
 */
void Controller::default_pos_state()
{
    std::cout << "进入默认位置状态。" << std::endl;
    std::cout << "等待 A 键信号..." << std::endl;
    
    while (remote_controller.button[KeyMap::A] != 1)
    {
        // 腿部关节保持默认位置
        for (int i = 0; i < 12; i++)
        {
            int motor_idx = joint2motor_idx[i];
            low_cmd.motor_cmd()[motor_idx].q() = default_real_angles[i];
            low_cmd.motor_cmd()[motor_idx].dq() = 0;
            low_cmd.motor_cmd()[motor_idx].kp() = 70.0f;
            low_cmd.motor_cmd()[motor_idx].kd() = 5.0f;
            low_cmd.motor_cmd()[motor_idx].tau() = 0;
        }
        
        usleep(static_cast<useconds_t>(control_dt * 1000000));
    }
}

/**
 * 执行完整的观测-推理-控制流程：
 * 1. 读取机器人状态
 * 2. 构建观测向量
 * 3. 神经网络推理
 * 4. 生成电机指令
 */
void Controller::run()
{
    counter++;
    
    auto low_state = mLowStateBuf.GetDataPtr();
    
    // ==================== 1. 读取关节状态 ====================
    for (int i = 0; i < 16; i++)
    {
        qj[i] = low_state->motor_state()[joint2motor_idx[i]].q();   // 位置
        dqj[i] = low_state->motor_state()[joint2motor_idx[i]].dq(); // 速度
    }
    
    // 转换为仿真顺序
    auto qj_sim = trans_r2s(qj);
    auto dqj_sim = trans_r2s(dqj);
    auto action_sim = trans_r2s(action);
    
    // ==================== 2. 读取 IMU 数据 ====================
    // 角速度
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
    
    // 姿态四元数 -> 重力方向
    std::array<float, 4> quat;
    for (int i = 0; i < 4; i++)
    {
        quat[i] = low_state->imu_state().quaternion()[i];
    }
    auto gravity_orientation = get_gravity_orientation(quat);
    
    // ==================== 3. 获取摇杆指令 ====================
    cmd[0] = remote_controller.ly;           // 前后速度 (左摇杆Y)
    cmd[1] = remote_controller.lx * -1.0f;   // 横向速度 (左摇杆X，取反)
    cmd[2] = remote_controller.rx * -1.0f;   // 旋转速度 (右摇杆X，取反)
    
    // ==================== 4. 构建观测向量 ====================
    // 关节误差（与默认位置的偏差）
    std::array<float, 16> err_obs;
    for (int i = 0; i < 16; i++)
    {
        err_obs[i] = qj_sim[i] - default_sim_angles[i];
    }
    // 轮子误差置零（轮子没有位置参考）
    for (int i = 0; i < 4; i++)
    {
        err_obs[wheel_sim_indices[i]] = 0.0f;
    }
    // 应用缩放
    for (int i = 0; i < 16; i++)
    {
        err_obs[i] *= dof_err_scale;
    }
    
    // 关节速度
    std::array<float, 16> dqj_obs;
    for (int i = 0; i < 16; i++)
    {
        dqj_obs[i] = dqj_sim[i] * dof_vel_scale;
    }
    
    // 关节位置（轮子置零）
    std::array<float, 16> qj_obs = qj_sim;
    for (int i = 0; i < 4; i++)
    {
        qj_obs[wheel_sim_indices[i]] = 0.0f;
    }
    
    // ==================== 5. 填充观测向量 (共73维) ====================
    // [0-2]: 角速度
    obs(0) = ang_vel_obs[0];
    obs(1) = ang_vel_obs[1];
    obs(2) = ang_vel_obs[2];
    
    // [3-5]: 重力方向
    obs(3) = gravity_orientation[0];
    obs(4) = gravity_orientation[1];
    obs(5) = gravity_orientation[2];
    
    // [6-8]: 速度指令
    obs(6) = cmd[0] * cmd_scale[0];
    obs(7) = cmd[1] * cmd_scale[1];
    obs(8) = cmd[2] * cmd_scale[2];
    
    // [9-24]: 关节误差
    for (int i = 0; i < num_actions; i++)
    {
        obs(9 + i) = err_obs[i];
    }
    
    // [25-40]: 关节速度
    for (int i = 0; i < num_actions; i++)
    {
        obs(9 + num_actions + i) = dqj_obs[i];
    }
    
    // [41-56]: 关节位置
    for (int i = 0; i < num_actions; i++)
    {
        obs(9 + num_actions * 2 + i) = qj_obs[i];
    }
    
    // [57-72]: 上一步动作
    for (int i = 0; i < num_actions; i++)
    {
        obs(9 + num_actions * 3 + i) = action_sim[i];
    }
    
    // ==================== 6. 更新历史观测缓冲区 ====================
    // 左移一帧，将最新观测放入末尾
    for (int i = 0; i < num_obs * 4; i++)
    {
        obs_hist_buf(i) = obs_hist_buf(i + num_obs);
    }
    for (int i = 0; i < num_obs; i++)
    {
        obs_hist_buf(num_obs * 4 + i) = obs(i);
    }
    
    // ==================== 7. DreamWaQ 神经网络推理 ====================
    // 历史编码器处理5帧观测
    torch::Tensor obs_hist_tensor = torch::from_blob(obs_hist_buf.data(), {obs_hist_buf.size()}, torch::kFloat).clone();
    
    std::vector<torch::jit::IValue> encoder_inputs;
    encoder_inputs.push_back(obs_hist_tensor);
    torch::Tensor h = encoder.forward(encoder_inputs).toTensor();
    
    // 从编码结果生成速度估计和隐变量
    std::vector<torch::jit::IValue> h_inputs;
    h_inputs.push_back(h);
    
    torch::Tensor vel_mu_out = vel_mu.forward(h_inputs).toTensor();
    torch::Tensor vel_var_out = vel_var.forward(h_inputs).toTensor();
    torch::Tensor latent_mu_out = latent_mu.forward(h_inputs).toTensor();
    torch::Tensor latent_var_out = latent_var.forward(h_inputs).toTensor();
    
    // VAE 采样
    torch::Tensor vel = reparameterise(vel_mu_out, vel_var_out);
    torch::Tensor latent = reparameterise(latent_mu_out, latent_var_out);
    
    // 拼接编码结果与当前观测
    torch::Tensor code = torch::cat({vel, latent}, -1);
    torch::Tensor obs_tensor = torch::from_blob(obs.data(), {obs.size()}, torch::kFloat).clone();
    torch::Tensor obs_all = torch::cat({code, obs_tensor}, -1);
    
    // 策略网络推理
    std::vector<torch::jit::IValue> actor_inputs;
    actor_inputs.push_back(obs_all);
    torch::Tensor action_tensor = actor.forward(actor_inputs).toTensor();
    
    // 提取动作输出
    std::array<float, 16> action_sim_out;
    std::memcpy(action_sim_out.data(), action_tensor.data_ptr<float>(), 16 * sizeof(float));
    
    // 转换回实机顺序
    qj = trans_s2r(qj_sim);
    action = trans_s2r(action_sim_out);
    dqj = trans_s2r(dqj_sim);
    
    // ==================== 8. 生成电机指令 ====================
    // 【修改开始】：增加大括号来限制锁的生命周期
    {
        std::lock_guard<std::mutex> lock(cmd_mutex); // 加锁：只保护数据写入
        
        // Send motor commands (matching Python exactly)
        for (int i = 0; i < 16; i++)
        {
            int motor_idx = joint2motor_idx[i];
            
            if (i >= 12)
            {
                // Wheel: velocity control
                low_cmd.motor_cmd()[motor_idx].q() = 0.0f;
                low_cmd.motor_cmd()[motor_idx].dq() = action[i] * 10.0f;
                low_cmd.motor_cmd()[motor_idx].kp() = 0.0f;
                low_cmd.motor_cmd()[motor_idx].kd() = kds[i];
                low_cmd.motor_cmd()[motor_idx].tau() = 0.0f;
            }
            else
            {
                // Leg: position control
                low_cmd.motor_cmd()[motor_idx].q() = default_real_angles[i] + action[i] * action_scale;
                low_cmd.motor_cmd()[motor_idx].dq() = 0.0f;
                low_cmd.motor_cmd()[motor_idx].kp() = kps[i];
                low_cmd.motor_cmd()[motor_idx].kd() = kds[i];
                low_cmd.motor_cmd()[motor_idx].tau() = 0.0f;
            }
        }
    } // 【修改结束】：大括号结束，锁在这里自动释放！
    // 发送线程现在可以自由获取锁并发送数据了
    
    // 指令由后台线程发送
    usleep(static_cast<useconds_t>(control_dt * 1000000));
}

/**
 * 阻尼模式退出
 * 1. 逐渐降低腿部刚度
 * 2. 最终进入纯阻尼模式
 */
void Controller::damp()
{
    std::cout << "Entering damp mode..." << std::endl;
    
    // 获取当前位置
    auto low_state = mLowStateBuf.GetDataPtr();
    std::array<float, 16> current_pos;
    for (int i = 0; i < 16; i++)
    {
        current_pos[i] = low_state->motor_state()[joint2motor_idx[i]].q();
    }
    
    // 1秒内逐渐降低 kp
    int num_steps = 50;
    for (int step = 0; step < num_steps; step++)
    {
        float alpha = 1.0f - static_cast<float>(step) / num_steps;  // 1.0 -> 0.0
        
        for (int i = 0; i < 12; i++)
        {
            int motor_idx = joint2motor_idx[i];
            low_cmd.motor_cmd()[motor_idx].q() = current_pos[i];
            low_cmd.motor_cmd()[motor_idx].dq() = 0;
            low_cmd.motor_cmd()[motor_idx].kp() = kps[i] * alpha;
            low_cmd.motor_cmd()[motor_idx].kd() = 5.0f;
            low_cmd.motor_cmd()[motor_idx].tau() = 0;
        }
        
        usleep(static_cast<useconds_t>(control_dt * 1000000));
    }
    
    // 最终进入纯阻尼模式
    create_damping_cmd();
    usleep(200000);
    
    std::cout << "Exit" << std::endl;
}

/**
 * DDS 订阅器收到消息时调用，存储状态并解析遥控器数据。
 */
void Controller::low_state_message_handler(const void *message)
{
    unitree_go::msg::dds_::LowState_* ptr = (unitree_go::msg::dds_::LowState_*)message;
    mLowStateBuf.SetData(*ptr);
    
    // 解析遥控器数据
    std::vector<uint8_t> wireless_data(ptr->wireless_remote().begin(), ptr->wireless_remote().end());
    remote_controller.set(wireless_data);
}

/**
 * 每2ms执行一次，计算 CRC 校验并发送指令。
 */
void Controller::low_cmd_write_handler()
{
    // 加锁：在计算 CRC 和发送期间，禁止其他线程修改 low_cmd
    std::lock_guard<std::mutex> lock(cmd_mutex);

    low_cmd.crc() = crc32_core((uint32_t*)(&low_cmd), (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(low_cmd);
}

/**
 * 用于 LowCmd 消息的完整性CRC校验，
 * 确保电机指令传输正确。
 */
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
