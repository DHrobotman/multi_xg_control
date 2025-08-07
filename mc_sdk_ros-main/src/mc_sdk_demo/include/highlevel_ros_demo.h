#pragma once

#include "highlevel_client.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/char.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>

namespace mc_sdk_ros {
class SdkRosDemoNode : public rclcpp::Node {
public:
  SdkRosDemoNode();
  ~SdkRosDemoNode();
  
  void StartKeyboardThread();
  void StopKeyboardThread();

private:
  void process();
  void oneStepStop();
  void handleKeyRelease();  
  void cmdprocess(char c);
  void processKeyWithDebounce(char key); 
  void robotStateCallback(const mc_sdk_msg::msg::HighLevelRobotState &msg);
  void syncCmdCallback(const std_msgs::msg::String::SharedPtr msg);
  void cmdCallback(const std_msgs::msg::Int16::SharedPtr msg);

  void KeyboardLoop();

  rclcpp::Publisher<mc_sdk_msg::msg::HighLevelCmd>::SharedPtr publisher_;
  rclcpp::Subscription<mc_sdk_msg::msg::HighLevelRobotState>::SharedPtr
      subscriber_;
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr key_sub_;  // 添加键盘订阅
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sync_sub_; // 添加同步指令订阅
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr cmd_subscriber_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr key_timeout_timer_;  // 键盘超时检测定时器
  
  mc_sdk_ros::HighLevelClient highlevel_client_;
  mc_sdk_msg::msg::HighLevelCmd pub_cmd_;
  std::mutex mutex_;
  std::atomic<bool> key_pressed_{false};  // 标记是否有按键按下
  std::atomic<char> current_key_{'\0'};   // 当前按下的键
  char last_key_;
  long long last_cmd_time_;
  std::atomic<bool> key_timeout_{false};  // 标记是否已处理过超时
  std::chrono::milliseconds key_debounce_time_;
  std::chrono::milliseconds key_activation_time_;
  std::chrono::steady_clock::time_point last_key_change_time_;
  std::chrono::steady_clock::time_point key_activation_start_time_;
  std::chrono::steady_clock::time_point release_time_;  // 记录按键释放时间
  std::chrono::steady_clock::time_point last_key_time_; // 记录最后收到键盘指令的时间
  std::atomic<bool> release_pending_{false};           // 标记是否有待处理的释放操作
  std::atomic<bool> running_{false};
  std::thread keyboard_thread_;
};
} // namespace mc_sdk_ros