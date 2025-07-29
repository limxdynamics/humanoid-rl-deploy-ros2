# Copyright information
#
# © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

# Set the minimum required CMake version
cmake_minimum_required(VERSION 3.10)

project(robot_hw)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(limxsdk_lowlevel REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(humanoid_hw_node 
  src/HardwareBase.cpp
  src/HumanoidHardware.cpp
  src/HardwareLoop.cpp
  src/HumanoidHardwareNode.cpp
)
ament_target_dependencies(humanoid_hw_node
  rclcpp
  hardware_interface
  controller_manager
  controller_manager_msgs
  urdf
  limxsdk_lowlevel
)

install(TARGETS humanoid_hw_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
