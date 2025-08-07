cmake_minimum_required(VERSION 3.10)
project(mc_sdk_ros)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mc_sdk_msg REQUIRED)

#为keyboard_teleop创建单独的可执行文件
add_executable(keyboard_teleop src/keyboard_teleop.cpp src/highlevel_client.cpp)
target_include_directories(keyboard_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(keyboard_teleop rclcpp std_msgs geometry_msgs mc_sdk_msg)

#为主可执行文件排除keyboard_teleop.cpp
file(GLOB_RECURSE sources "src/*.cpp")
list(FILTER sources EXCLUDE REGEX ".*keyboard_teleop.cpp$")
add_executable(mc_sdk_ros ${sources})
target_include_directories(mc_sdk_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
message(${CMAKE_CURRENT_BINARY_DIR})
ament_target_dependencies(mc_sdk_ros rclcpp std_msgs geometry_msgs mc_sdk_msg)

install(TARGETS mc_sdk_ros keyboard_teleop
  DESTINATION lib/${PROJECT_NAME}
)
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()