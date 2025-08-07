#include "mc_sdk_node.h"

ZSIBOT_IMPL_TOPIC(SDKCmd, Ecal);
ZSIBOT_IMPL_TOPIC(SDKState, Ecal);
using namespace mc_sdk_node;

void McSdkNode::init() {
  sdk_pub_->init("sdk_cmd");
  sdk_sub_->init("sdk_robotstate");

  sdk_sub_->subscribe(
      [this](const zsibot::rmw::MessagePtr<robot_sdk::pb::SDKRobotState> &msg) {
        mc_sdk_robot_state_.acc.set__x(msg->data.acc()[0]);
        mc_sdk_robot_state_.acc.set__y(msg->data.acc()[1]);
        mc_sdk_robot_state_.acc.set__z(msg->data.acc()[2]);

        mc_sdk_robot_state_.gyro.set__x(msg->data.gyro()[0]);
        mc_sdk_robot_state_.gyro.set__y(msg->data.gyro()[1]);
        mc_sdk_robot_state_.gyro.set__z(msg->data.gyro()[2]);

        mc_sdk_robot_state_.rpy.set__x(msg->data.rpy()[0]);
        mc_sdk_robot_state_.rpy.set__y(msg->data.rpy()[1]);
        mc_sdk_robot_state_.rpy.set__z(msg->data.rpy()[2]);

        mc_sdk_robot_state_.pos.set__x(msg->data.position()[0]);
        mc_sdk_robot_state_.pos.set__y(msg->data.position()[1]);
        mc_sdk_robot_state_.pos.set__z(msg->data.position()[2]);

        mc_sdk_robot_state_.vel.set__x(msg->data.reserved_float()[0]);
        mc_sdk_robot_state_.vel.set__y(msg->data.reserved_float()[1]);
        mc_sdk_robot_state_.vel.set__z(msg->data.reserved_float()[2]);

        mc_sdk_robot_state_.vel_world.set__x(msg->data.v_world()[0]);
        mc_sdk_robot_state_.vel_world.set__y(msg->data.v_world()[1]);
        mc_sdk_robot_state_.vel_world.set__z(msg->data.v_world()[2]);

        publisher_->publish(mc_sdk_robot_state_);
      });
}

void McSdkNode::cmdCallback(const HighLevelCmd &msg) {
  RCLCPP_INFO(this->get_logger(), "Received command: %d", msg.control_mode);
  mc_sdk_cmd_.set_control_mode(msg.control_mode);
  mc_sdk_cmd_.set_motion_mode(msg.motion_mode);
  mc_sdk_cmd_.set_vx(msg.cmd_vel.x);
  mc_sdk_cmd_.set_vy(msg.cmd_vel.y);
  mc_sdk_cmd_.set_vz(msg.cmd_vel.z);
  mc_sdk_cmd_.set_yaw_rate(msg.cmd_angular.z);
  mc_sdk_cmd_.set_pitch_rate(msg.cmd_angular.y);
  mc_sdk_cmd_.set_roll_rate(msg.cmd_angular.x);
  
  // 添加时间戳
  mc_sdk_cmd_.set_time_stamp(rclcpp::Clock().now().nanoseconds());
  
  sdk_pub_->publish(mc_sdk_cmd_);
}