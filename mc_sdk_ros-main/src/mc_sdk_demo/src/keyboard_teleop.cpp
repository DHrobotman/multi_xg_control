#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/char.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>

class KeyboardTeleop : public rclcpp::Node {
public:
  KeyboardTeleop() : Node("keyboard_teleop") {
    // 发布到同步控制话题
    sync_pub_ = this->create_publisher<std_msgs::msg::String>("/sync_cmd", 10);
    tcgetattr(STDIN_FILENO, &orig_termios_);
    startKeyboardLoop();
    RCLCPP_INFO(this->get_logger(), "Keyboard teleop node started. Press 'x' to exit.");
  }

private:
  void startKeyboardLoop() {
    std::thread([this]() {
      struct termios newt;
      tcgetattr(STDIN_FILENO, &newt);
      newt.c_lflag &= ~(ICANON | ECHO);
      newt.c_cc[VMIN] = 1;
      newt.c_cc[VTIME] = 0;
      tcsetattr(STDIN_FILENO, TCSANOW, &newt);

      std::cout << "\033[1;32m" << "控制所有机器人 (w/s/a/d/q/e/c/0~7):" << "\033[0m" << std::endl;
      std::cout << "  w: 前进    s: 后退    a: 左移    d: 右移" << std::endl;
      std::cout << "  q: 左旋    e: 右旋    c: 停止" << std::endl;
      std::cout << "  0: 被动    1: 卧倒    2: 站立    3: 跳跃" << std::endl;
      std::cout << "  4: 前跳    6: 后空翻  7: 姿态控制" << std::endl;
      std::cout << "按 'x' 退出控制..." << std::endl;

      char c;
      while (rclcpp::ok() && read(STDIN_FILENO, &c, 1) > 0) {
        if (c == 'x') {
          std::cout << "\nExiting...\n";
      rclcpp::shutdown();
          break;
        }

        // 发送带时间戳的同步指令
        auto msg = std_msgs::msg::String();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        msg.data = std::to_string(timestamp) + ":" + std::string(1, c);
        sync_pub_->publish(msg);
        std::cout << "\033[1;36m>> 广播同步指令: " << c << " (时间戳: " << timestamp << ")\033[0m" << std::endl;
      }

      // 恢复终端
      tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    }).detach();
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sync_pub_;
  struct termios orig_termios_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleop>());
  rclcpp::shutdown();
  return 0;
}