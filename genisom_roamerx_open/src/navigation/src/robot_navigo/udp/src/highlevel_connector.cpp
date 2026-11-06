/*
 * @Author: zhangjian zhangjian@jushenzhiren.com
 * @Date: 2024-12-19 17:25:03
 * @LastEditors: zhangjian zhangjian@jushenzhiren.com
 * @LastEditTime: 2024-12-27 12:11:27
 * @FilePath: /navigo_sdk/src/rt_udp.cpp
 */
#include "highlevel_connector.h"
#include "udp_components.h"
#include <functional>
using namespace navigo_sdk;
/**
 * @brief  check code
 * @param  data
 * @param  len
 * @return uint32_t
 */

struct HighLevelConnector::HighLevelConnectorImpl {
  Udp<highLevelState, highLevelCmd> *ptr;
};

HighLevelConnector::HighLevelConnector() {
  impl_ = std::make_shared<HighLevelConnectorImpl>();
}

void HighLevelConnector::creat_connector(const std::string &target_ip,
                                        const std::string &local_ip,
                                        const int target_port,
                                        const int local_port) {
  impl_->ptr = new Udp<highLevelState, highLevelCmd>(target_ip, local_ip,
                                             target_port, local_port);
}

bool HighLevelConnector::reg_state_recive() {

  bool data_rel = impl_->ptr->init_async();

  impl_->ptr->reg_async_receive(
      [this](highLevelState &state) { this->ReciveState(state); });
  if (!data_rel) {
    return false;
  }
  return true;
}

highLevelState *HighLevelConnector::GetState() {
  if (impl_->ptr->is_time_out()) {
    memset(&robot_state_, 0, sizeof(highLevelState));
  }
  return &robot_state_;
}
void HighLevelConnector::SendCmd(highLevelCmd *robot_cmd) {
  impl_->ptr->sendMsg(robot_cmd);
}

void HighLevelConnector::ReciveState(const highLevelState &robot_state) {
  static int checksum_error_count = 0;
  memcpy(&robot_state_, &robot_state, sizeof(highLevelState));
  unsigned int calc_checksum =
      checksum(reinterpret_cast<const unsigned char *>(&robot_state));
  if (calc_checksum != robot_state.checksum) {
    checksum_error_count++;
    printf("Recive State ERROR BAD CHECKSUM GOT 0x%hx EXPECTED 0x%hx\n",
           calc_checksum, robot_state.checksum);
    if (checksum_error_count > 2) {
      printf("COM FAIL!");
    }
  } else {
    checksum_error_count = 0;
  }
}

unsigned int HighLevelConnector::checksum(const unsigned char *data) {

  unsigned int checksum = 0;
  for (size_t i = 0; i < sizeof(data); ++i) {
    checksum += data[i];
  }
  return checksum;
}