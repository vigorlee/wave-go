/*
 * @Author: zhangjian zhangjian@jushenzhiren.com
 * @Date: 2024-12-19 16:53:30
 * @LastEditors: zhangjian zhangjian@jushenzhiren.com
 * @LastEditTime: 2024-12-26 19:47:17
 * @FilePath: /navigo_sdk/include/common.h
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
namespace navigo_sdk {

struct alignas(4) robotCmd  {
  uint16_t head = 0; //头
  uint16_t len = 0;  //长度

  float q_des_abad[4] = {0.0, 0.0, 0.0, 0.0}; // A关节角度指令
  float q_des_hip[4] = {0.0, 0.0, 0.0, 0.0};  // H关节角度指令
  float q_des_knee[4] = {0.0, 0.0, 0.0, 0.0}; // K关节角度指令


  float qd_des_abad[4] = {0.0, 0.0, 0.0, 0.0}; // A关节角速度指令
  float qd_des_hip[4] = {0.0, 0.0, 0.0, 0.0};  // H关节角速度指令
  float qd_des_knee[4] = {0.0, 0.0, 0.0, 0.0}; // K关节角速度指令


  float kp_abad[4] = {0.0, 0.0, 0.0, 0.0}; // A关节Kp值
  float kp_hip[4] = {0.0, 0.0, 0.0, 0.0};  // H关节Kp值
  float kp_knee[4] = {0.0, 0.0, 0.0, 0.0}; // K关节Kp值


  float kd_abad[4] = {0.0, 0.0, 0.0, 0.0}; // A关节Kd值
  float kd_hip[4] = {0.0, 0.0, 0.0, 0.0};  // H关节Kd值
  float kd_knee[4] = {0.0, 0.0, 0.0, 0.0}; // K关节Kd值


  float tau_abad_ff[4] = {0.0, 0.0, 0.0, 0.0}; // A关节扭矩指令
  float tau_hip_ff[4] = {0.0, 0.0, 0.0, 0.0};  // H关节扭矩指令
  float tau_knee_ff[4] = {0.0, 0.0, 0.0, 0.0}; // K关节扭矩指令


  unsigned int checksum = 0; // 校验字
};

struct alignas(4) highLevelCmd {
  uint16_t head = 0;
  uint16_t len = 0;
  float vx = 0.0;
  float vy = 0.0;
  float roll_rate = 0.0;
  float pitch_rate = 0.0;
  float yaw_rate = 0.0;
  float height_rate = 0.0;
  uint16_t motion_mode = 0;
  uint16_t control_mode = 0;
  uint16_t enable_control_mode = 0;
  unsigned int checksum = 0;
};

struct alignas(4) robotState {
  uint16_t head = 0; //头
  uint16_t len = 0;  //长度

  float q_abad[4] = {0.0, 0.0, 0.0, 0.0}; // A关节角度
  float q_hip[4] = {0.0, 0.0, 0.0, 0.0};  // H关节角度
  float q_knee[4] = {0.0, 0.0, 0.0, 0.0}; // K关节角度


  float qd_abad[4] = {0.0, 0.0, 0.0, 0.0}; // A关节角速度
  float qd_hip[4] = {0.0, 0.0, 0.0, 0.0};  // H关节角速度
  float qd_knee[4] = {0.0, 0.0, 0.0, 0.0}; // K关节角速度


  float tau_abad_fb[4] = {0.0, 0.0, 0.0, 0.0};// A关节扭矩反馈
  float tau_hip_fb[4] = {0.0, 0.0, 0.0, 0.0};  // H关节扭矩反馈
  float tau_knee_fb[4] = {0.0, 0.0, 0.0, 0.0}; // K关节扭矩反馈

  float quat[4] = {0.0, 0.0, 0.0, 0.0};
  float gyro[3] = {0.0, 0.0, 0.0};
  float acc[3] = {0.0, 0.0, 0.0};
  float rpy[3] = {0.0, 0.0, 0.0};
  float time_stamp = 0.0;
  float desire_vel_x = 0.0;
  float desire_vel_y = 0.0;
  float desire_yaw_rate = 0.0;
  uint16_t in_rl_state = 0;
  unsigned int checksum = 0; // 校验字
};

struct alignas(4) highLevelState {
  uint16_t head = 0; //头
  uint16_t len = 0;  //长度

  float quat[4] = {0.0, 0.0, 0.0, 0.0};
  float gyro[3] = {0.0, 0.0, 0.0};
  float acc[3] = {0.0, 0.0, 0.0};
  float rpy[3] = {0.0, 0.0, 0.0};
  float time_stamp = 0.0;
  float vWorld[3] = {0.0, 0.0, 0.0};
  float position[3] = {0.0, 0.0, 0.0};
  float omegaWorld[3] = {0.0, 0.0, 0.0};
  unsigned int checksum = 0; // 校验字
};

} // namespace navigo_sdk
