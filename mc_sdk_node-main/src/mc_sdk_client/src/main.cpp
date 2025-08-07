#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "mc_sdk_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<mc_sdk_node::McSdkNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}