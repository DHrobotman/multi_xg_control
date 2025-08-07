#include "highlevel_client.h"
#include "geometry_msgs/msg/vector3.hpp"

using namespace mc_sdk_ros;

HighLevelClient::HighLevelClient() {}
HighLevelClient::~HighLevelClient() {}

void HighLevelClient::passive(mc_sdk_msg::msg::HighLevelCmd &requst) {
  requst.set__control_mode(PASSIVE);
  requst.set__motion_mode(0);
}
void HighLevelClient::lieDown(mc_sdk_msg::msg::HighLevelCmd &requst) {
  requst.set__control_mode(LIEDOWN);
  requst.set__motion_mode(0);
}
void HighLevelClient::standUp(mc_sdk_msg::msg::HighLevelCmd &requst) {
  requst.set__control_mode(STANDUP);
  requst.set__motion_mode(0);
}
void HighLevelClient::move(mc_sdk_msg::msg::HighLevelCmd &requst, float vx,
                           float vy, float vyaw) {
  requst.set__control_mode(MOVE);
  requst.set__motion_mode(0);
  geometry_msgs::msg::Vector3 cmd_vel;
  geometry_msgs::msg::Vector3 cmd_angular;
  cmd_vel.x = vx;
  cmd_vel.y = vy;
  cmd_vel.z = 0.0;
  cmd_angular.x = 0.0;
  cmd_angular.y = 0.0;
  cmd_angular.z = vyaw;
  requst.set__cmd_vel(cmd_vel);
  requst.set__cmd_angular(cmd_angular);
}
void HighLevelClient::attitudeCtrl(mc_sdk_msg::msg::HighLevelCmd &requst,
                                   float vroll, float vpitch, float vyaw,
                                   float vheight) {
  requst.set__control_mode(BALANCESTAND);
  requst.set__motion_mode(0);
  geometry_msgs::msg::Vector3 cmd_vel;
  geometry_msgs::msg::Vector3 cmd_angular;
  cmd_vel.x = 0.0;
  cmd_vel.y = 0.0;
  cmd_vel.z = vheight;
  cmd_angular.x = vroll;
  cmd_angular.y = vpitch;
  cmd_angular.z = vyaw;
  requst.set__cmd_vel(cmd_vel);
  requst.set__cmd_angular(cmd_angular);

}
void HighLevelClient::jump(mc_sdk_msg::msg::HighLevelCmd &requst) {
  requst.set__control_mode(BALANCESTAND);
  requst.set__motion_mode(JUMP);
}
void HighLevelClient::frontJump(mc_sdk_msg::msg::HighLevelCmd &requst) {
  requst.set__control_mode(BALANCESTAND);
  requst.set__motion_mode(FRONTJUMP);
}
void HighLevelClient::backflip(mc_sdk_msg::msg::HighLevelCmd &requst) {
  requst.set__control_mode(BALANCESTAND);
  requst.set__motion_mode(BACKFLIP);
}