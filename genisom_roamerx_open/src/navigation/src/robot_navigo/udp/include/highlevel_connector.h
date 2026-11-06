/*
 * @Author: zhangjian zhangjian@jushenzhiren.com
 * @Date: 2024-12-19 17:24:55
 * @LastEditors: zhangjian zhangjian@jushenzhiren.com
 * @LastEditTime: 2024-12-26 19:46:56
 * @FilePath: /navigo_sdk/include/rt_udp.h
 */
#pragma once
#include "common.h"
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
namespace navigo_sdk {
class HighLevelConnector {
private:
  struct HighLevelConnectorImpl;

  std::shared_ptr<HighLevelConnectorImpl> impl_;

  void ReciveState(const highLevelState &robot_state);
  highLevelState robot_state_;

public:
  HighLevelConnector();
  ~HighLevelConnector(){};

  void SendCmd(highLevelCmd *robot_cmd);
  unsigned int checksum(const unsigned char *data);
  highLevelState *GetState();
  bool reg_state_recive();

  void creat_connector(const std::string &target_ip,
                       const std::string &local_ip, const int target_port,
                       const int local_port);
};
} // namespace navigo_sdk
