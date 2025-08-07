#pragma once

#include "mc_sdk_msg/msg/high_level_cmd.hpp"
#include "mc_sdk_msg/msg/high_level_robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zsibot/base.hpp"
#include "zsibot/rmw/ecal.hpp"
#include "robot_sdk.pb.h"
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

using namespace std::chrono_literals;
using mc_sdk_msg::msg::HighLevelCmd;
using mc_sdk_msg::msg::HighLevelRobotState;

ZSIBOT_TOPIC(robot_sdk::pb::SDKCmd, SDKCmd);
ZSIBOT_TOPIC(robot_sdk::pb::SDKRobotState, SDKState);

namespace mc_sdk_node {
class McSdkNode : public rclcpp::Node {
public:
  McSdkNode() : rclcpp::Node("mc_sdk_node") {
    RCLCPP_INFO(this->get_logger(), "public node is created!");
    publisher_ =
        this->create_publisher<HighLevelRobotState>("highlevel_robotstate", 10);

    subscriber_ = this->create_subscription<HighLevelCmd>(
        "highlevel_cmd", 10,
        std::bind(&McSdkNode::cmdCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "sub node is created!");

    sdk_pub_ = std::make_unique<zsibot::rmw::Publisher<SDKCmd>>();
    sdk_sub_ = std::make_unique<zsibot::rmw::Subscriber<SDKState>>();

    init();

  }
  ~McSdkNode() {}

private:
  void init();
  void cmdCallback(const HighLevelCmd &msg);

private:
  rclcpp::Publisher<HighLevelRobotState>::SharedPtr publisher_;
  rclcpp::Subscription<HighLevelCmd>::SharedPtr subscriber_;
  std::unique_ptr<zsibot::rmw::Subscriber<SDKState>> sdk_sub_;
  std::unique_ptr<zsibot::rmw::Publisher<SDKCmd>> sdk_pub_;

  robot_sdk::pb::SDKCmd mc_sdk_cmd_;
  HighLevelRobotState mc_sdk_robot_state_;
};
} // namespace mc_sdk_ros