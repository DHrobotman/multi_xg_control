#include "highlevel_ros_demo.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <sstream>

using namespace mc_sdk_ros;
using mc_sdk_msg::msg::HighLevelCmd;
using mc_sdk_msg::msg::HighLevelRobotState;
using std_msgs::msg::Char;
using std_msgs::msg::String;
using namespace std::chrono_literals;

SdkRosDemoNode::SdkRosDemoNode() 
  : rclcpp::Node("mc_sdk_demo"),
    key_pressed_(false),
    current_key_('\0'),
    last_key_('\0'),
    key_timeout_(false),
    release_pending_(false),
    last_cmd_time_(0),
    running_(false),
    key_debounce_time_(std::chrono::milliseconds(30)),
    key_activation_time_(std::chrono::milliseconds(5)) 
{
  RCLCPP_INFO(this->get_logger(), "Node initialized");

  // 创建指令发布器
  publisher_ = this->create_publisher<HighLevelCmd>("highlevel_cmd", 10);
  
  // 创建定时器处理延迟停止逻辑
  timer_ = this->create_wall_timer(
      5ms, [this]() { this->process(); });
  
  // 订阅机器人状态话题
  subscriber_ = this->create_subscription<HighLevelRobotState>(
      "highlevel_robotstate", 10,
      [this](const HighLevelRobotState::SharedPtr msg) {
        this->robotStateCallback(*msg);
      });

  // 订阅同步控制指令话题（替换原来的 /cmd_key 订阅）
  sync_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/sync_cmd", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        this->syncCmdCallback(msg);
      });

  cmd_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
    "/user_cmd", 10,
     [this](const std_msgs::msg::Int16::SharedPtr msg) {
        this->cmdCallback(msg);
      });


  // 添加一个定时器检测键盘输入超时
  key_timeout_timer_ = this->create_wall_timer(
      50ms, [this]() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_key_time_);
        // 如果超过50ms没有收到键盘输入，并且当前有按键按下状态，则触发释放处理
        if (elapsed.count() >= 50 && !key_timeout_ && key_pressed_) {
          handleKeyRelease();
          key_timeout_ = true;
        }
      });

  RCLCPP_INFO(this->get_logger(), "已订阅 /sync_cmd 话题，等待同步指令...");
}

void SdkRosDemoNode::syncCmdCallback(const std_msgs::msg::String::SharedPtr msg) {
  // 解析时间戳和指令
  std::string data = msg->data;
  size_t pos = data.find(':');
  if (pos != std::string::npos) {
    long long timestamp = std::stoll(data.substr(0, pos));
    char cmd = data.substr(pos + 1)[0];
    
    // 避免重复处理相同指令
    if (timestamp <= last_cmd_time_) {
      return;
    }
    
    last_cmd_time_ = timestamp;
    
    std::lock_guard<std::mutex> lock(mutex_);
    // 更新最后收到键盘指令的时间
    last_key_time_ = std::chrono::steady_clock::now();
    key_timeout_ = false;
    
    // 处理接收到的键盘指令（带去抖动）
    processKeyWithDebounce(cmd);
    
    // 如果是退出指令，关闭节点
    if (cmd == 'x') {
      RCLCPP_INFO(this->get_logger(), "收到退出指令，关闭节点...");
      rclcpp::shutdown();
    }
  }
}

void SdkRosDemoNode::processKeyWithDebounce(char key) {
  auto now = std::chrono::steady_clock::now();
  
  // 如果这是新按键或者与上次按键不同
  if (key != last_key_) {
    last_key_ = key;
    last_key_change_time_ = now;
    key_activation_start_time_ = now; // 记录按键开始时间
    return; // 等待稳定
  }
  
  // 检查按键是否已经稳定了足够长的时间
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_key_change_time_);
  if (elapsed >= key_debounce_time_) {
    // 检查按键是否持续了足够长的时间以激活
    auto activation_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - key_activation_start_time_);
    if (activation_elapsed >= key_activation_time_) {
      // 按键已经稳定且持续了足够时间，处理它
      cmdprocess(key);
    }
  }
}

SdkRosDemoNode::~SdkRosDemoNode() {
  // 析构函数中无需再处理终端设置，因为键盘监听已独立
}

void SdkRosDemoNode::process() {
  // 检查是否有待处理的延迟停止操作
  if (release_pending_) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - release_time_);
    
    // 如果已经过了200ms，则执行停止操作
    if (elapsed.count() >= 50) {
      highlevel_client_.move(pub_cmd_, 0.0, 0.0, 0.0);
      release_pending_ = false;
      // 发布停止指令
      pub_cmd_.stamp = this->now();
      publisher_->publish(pub_cmd_);
    }
  }
  
  static HighLevelCmd last_cmd;
  static int cnt = 0;
  
  if (last_cmd.motion_mode != 0) {
    if (++cnt > 5) {
      cnt = 0;
      pub_cmd_.motion_mode = 0;
    }
  }
  
  // 只有当指令发生变化时才发布，避免持续发布相同指令浪费带宽
  bool cmd_changed = false;
  
  // 检查指令是否发生变化
  if (last_cmd.stamp.nanosec != pub_cmd_.stamp.nanosec || 
      last_cmd.stamp.sec != pub_cmd_.stamp.sec ||
      last_cmd.control_mode != pub_cmd_.control_mode ||
      last_cmd.motion_mode != pub_cmd_.motion_mode ||
      last_cmd.cmd_vel.x != pub_cmd_.cmd_vel.x ||
      last_cmd.cmd_vel.y != pub_cmd_.cmd_vel.y ||
      last_cmd.cmd_vel.z != pub_cmd_.cmd_vel.z ||
      last_cmd.cmd_angular.x != pub_cmd_.cmd_angular.x ||
      last_cmd.cmd_angular.y != pub_cmd_.cmd_angular.y ||
      last_cmd.cmd_angular.z != pub_cmd_.cmd_angular.z) {
    cmd_changed = true;
  }
  
  // 更新时间戳
  pub_cmd_.stamp = this->now();
  
  // 只有指令改变时才发布
  if (cmd_changed) {
    publisher_->publish(pub_cmd_);
    // 更新last_cmd为当前指令
    last_cmd = pub_cmd_;
  }
}

void SdkRosDemoNode::handleKeyRelease() {
  if (key_pressed_) {
    // 当检测到运动控制类按键状态变化时，设置延迟停止
    switch (current_key_) {
      case 'w':
      case 's':
      case 'a':
      case 'd':
      case 'q':
      case 'e':
        release_pending_ = true;
        release_time_ = std::chrono::steady_clock::now();
        break;
      default:
        break;
    }
    
    key_pressed_ = false;
    current_key_ = '\0';
  }
}

void SdkRosDemoNode::cmdprocess(char c) {
  RCLCPP_INFO(this->get_logger(), "接收到键盘控制指令: %c", c);
  
  // 记录按键状态
  key_pressed_ = true;
  current_key_ = c;
  
  // 如果有延迟停止操作在等待，取消它
  if (release_pending_) {
    release_pending_ = false;
  }
  
  switch (c) {
    case 'w': 
      highlevel_client_.move(pub_cmd_, 0.5, 0.0, 0.0);
      RCLCPP_INFO(this->get_logger(), "向前移动");
      break;
    case 's': 
      highlevel_client_.move(pub_cmd_, -0.5, 0.0, 0.0);
      RCLCPP_INFO(this->get_logger(), "向后移动");
      break;
    case 'a': 
      highlevel_client_.move(pub_cmd_, 0.0, 0.5, 0.0);
      RCLCPP_INFO(this->get_logger(), "向左移动");
      break;
    case 'd': 
      highlevel_client_.move(pub_cmd_, 0.0, -0.5, 0.0);
      RCLCPP_INFO(this->get_logger(), "向右移动");
      break;
    case 'q': 
      highlevel_client_.move(pub_cmd_, 0.0, 0.0, 0.5);
      RCLCPP_INFO(this->get_logger(), "逆时针旋转");
      break;
    case 'e': 
      highlevel_client_.move(pub_cmd_, 0.0, 0.0, -0.5);
      RCLCPP_INFO(this->get_logger(), "顺时针旋转");
      break;
    case 'c': 
      highlevel_client_.move(pub_cmd_, 0.0, 0.0, 0.0);
      RCLCPP_INFO(this->get_logger(), "停止移动");
      break;
    case '0': 
      // 清除运动模式相关字段，避免与其他模式冲突
      pub_cmd_.cmd_vel.x = 0.0;
      pub_cmd_.cmd_vel.y = 0.0;
      pub_cmd_.cmd_vel.z = 0.0;
      pub_cmd_.cmd_angular.x = 0.0;
      pub_cmd_.cmd_angular.y = 0.0;
      pub_cmd_.cmd_angular.z = 0.0;
      highlevel_client_.passive(pub_cmd_);
      RCLCPP_INFO(this->get_logger(), "被动模式");
      break;
    case '1': 
      // 清除运动模式相关字段，避免与其他模式冲突
      pub_cmd_.cmd_vel.x = 0.0;
      pub_cmd_.cmd_vel.y = 0.0;
      pub_cmd_.cmd_vel.z = 0.0;
      pub_cmd_.cmd_angular.x = 0.0;
      pub_cmd_.cmd_angular.y = 0.0;
      pub_cmd_.cmd_angular.z = 0.0;
      highlevel_client_.lieDown(pub_cmd_);
      RCLCPP_INFO(this->get_logger(), "卧倒");
      break;
    case '2': 
      // 清除运动模式相关字段，避免与其他模式冲突
      pub_cmd_.cmd_vel.x = 0.0;
      pub_cmd_.cmd_vel.y = 0.0;
      pub_cmd_.cmd_vel.z = 0.0;
      pub_cmd_.cmd_angular.x = 0.0;
      pub_cmd_.cmd_angular.y = 0.0;
      pub_cmd_.cmd_angular.z = 0.0;
      highlevel_client_.standUp(pub_cmd_);
      RCLCPP_INFO(this->get_logger(), "站立");
      break;
    case '3': 
      // 清除运动模式相关字段，避免与其他模式冲突
      pub_cmd_.cmd_vel.x = 0.0;
      pub_cmd_.cmd_vel.y = 0.0;
      pub_cmd_.cmd_vel.z = 0.0;
      pub_cmd_.cmd_angular.x = 0.0;
      pub_cmd_.cmd_angular.y = 0.0;
      pub_cmd_.cmd_angular.z = 0.0;
      highlevel_client_.jump(pub_cmd_);
      RCLCPP_INFO(this->get_logger(), "跳跃");
      break;
    case '4': 
      // 清除运动模式相关字段，避免与其他模式冲突
      pub_cmd_.cmd_vel.x = 0.0;
      pub_cmd_.cmd_vel.y = 0.0;
      pub_cmd_.cmd_vel.z = 0.0;
      pub_cmd_.cmd_angular.x = 0.0;
      pub_cmd_.cmd_angular.y = 0.0;
      pub_cmd_.cmd_angular.z = 0.0;
      highlevel_client_.frontJump(pub_cmd_);
      RCLCPP_INFO(this->get_logger(), "前跳");
      break;
    case '6': 
      // 清除运动模式相关字段，避免与其他模式冲突
      pub_cmd_.cmd_vel.x = 0.0;
      pub_cmd_.cmd_vel.y = 0.0;
      pub_cmd_.cmd_vel.z = 0.0;
      pub_cmd_.cmd_angular.x = 0.0;
      pub_cmd_.cmd_angular.y = 0.0;
      pub_cmd_.cmd_angular.z = 0.0;
      highlevel_client_.backflip(pub_cmd_);
      RCLCPP_INFO(this->get_logger(), "后空翻");
      break;
    case '7': 
      // 清除运动模式相关字段，避免与其他模式冲突
      pub_cmd_.cmd_vel.x = 0.0;
      pub_cmd_.cmd_vel.y = 0.0;
      pub_cmd_.cmd_vel.z = 0.0;
      pub_cmd_.cmd_angular.x = 0.0;
      pub_cmd_.cmd_angular.y = 0.0;
      pub_cmd_.cmd_angular.z = 0.0;
      highlevel_client_.attitudeCtrl(pub_cmd_, 0.0, 0.5, 0.0, 0.0);
      RCLCPP_INFO(this->get_logger(), "姿态控制");
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "未知指令: %c", c);
      break;
  }
  
  // 更新时间戳，让process函数检测到变化并发布指令
  pub_cmd_.stamp = this->now();
}
void SdkRosDemoNode::robotStateCallback(const HighLevelRobotState &msg) {
  RCLCPP_DEBUG(this->get_logger(), "机器人状态: acc.x=%.2f", msg.acc.x);
}


void SdkRosDemoNode::cmdCallback(const std_msgs::msg::Int16::SharedPtr msg) {
    int c = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received cmd: %d", c);
    switch (c) {
    case 1: highlevel_client_.move(pub_cmd_, 0.5, 0.0, 0.0); oneStepStop(); break;
    case 2: highlevel_client_.move(pub_cmd_, -0.5, 0.0, 0.0); oneStepStop(); break;     // 后退
    case 3: highlevel_client_.move(pub_cmd_, 0.0, 0.0, 1.0); oneStepStop(); break;      // 左转
    case 4: highlevel_client_.move(pub_cmd_, 0.0, 0.0, -1.0); oneStepStop(); break;     // 右转
    case 5: highlevel_client_.move(pub_cmd_, 0.0, 0.0, 0.0); oneStepStop(); break;      // 停止
    case 6: highlevel_client_.move(pub_cmd_, 0.0, 0.5, 0.0); oneStepStop(); break;      // 左移
    case 7: highlevel_client_.move(pub_cmd_, 0.0, -0.5, 0.0); oneStepStop(); break;     // 右移
    case 10: highlevel_client_.passive(pub_cmd_); break;
    case 11: highlevel_client_.lieDown(pub_cmd_); break;
    case 12: highlevel_client_.standUp(pub_cmd_); break;
    case 13: highlevel_client_.jump(pub_cmd_); break;
    case 14: highlevel_client_.frontJump(pub_cmd_); break;
    case 16: highlevel_client_.backflip(pub_cmd_); break;
    case 17: highlevel_client_.attitudeCtrl(pub_cmd_, 0.0, 0.5, 0.0, 0.0); break;
    default: break;
    }

}

void SdkRosDemoNode::oneStepStop()
{
  publisher_->publish(pub_cmd_);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  highlevel_client_.move(pub_cmd_, 0.0, 0.0, 0.0);
  publisher_->publish(pub_cmd_);  
}