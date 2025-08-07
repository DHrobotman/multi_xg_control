#pragma once

#include "mc_sdk_msg/msg/high_level_cmd.hpp"
#include "mc_sdk_msg/msg/high_level_robot_state.hpp"

namespace mc_sdk_ros {
#define PASSIVE 0
#define LIEDOWN 51
#define STANDUP 1
#define MOVE 18
#define BALANCESTAND 21
#define JUMP 5
#define FRONTJUMP 6
#define BACKFLIP 2

class HighLevelClient {
public:
  HighLevelClient();
  ~HighLevelClient();

  void passive(mc_sdk_msg::msg::HighLevelCmd &requst);
  void lieDown(mc_sdk_msg::msg::HighLevelCmd &requst);
  void standUp(mc_sdk_msg::msg::HighLevelCmd &requst);
  void move(mc_sdk_msg::msg::HighLevelCmd &requst, float vx, float vy,
            float vyaw);
  void attitudeCtrl(mc_sdk_msg::msg::HighLevelCmd &requst, float vroll,
                    float vpitch, float vyaw, float vheight);
  void jump(mc_sdk_msg::msg::HighLevelCmd &requst);
  void frontJump(mc_sdk_msg::msg::HighLevelCmd &requst);
  void backflip(mc_sdk_msg::msg::HighLevelCmd &requst);
};
} // namespace mc_sdk_ros