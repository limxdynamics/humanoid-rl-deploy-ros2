// Copyright information
//
// © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include "robot_controllers/DampingHumanoidController.h"

#include "controller_interface/helpers.hpp"

namespace robot_controllers
{
  DampingHumanoidController::DampingHumanoidController()
  {
  }

  bool DampingHumanoidController::onInit()
  {
    if (!loadDampingCfg())
    {
      RCLCPP_ERROR(rclcpp::get_logger("DampingHumanoidController"), "Error in loadDampingCfg");
      return false;
    }

    return true;
  }

  void DampingHumanoidController::onStart()
  {
    
  }

  void DampingHumanoidController::onUpdate()
  {
    for (std::size_t i = 0; i < jointNum_; i++)
    {
      this->setJointCommandValue(std::to_string(i), "position", 0);
      this->setJointCommandValue(std::to_string(i), "velocity", 0);
      this->setJointCommandValue(std::to_string(i), "kp", 0);
      this->setJointCommandValue(std::to_string(i), "kd", dampingKd_[i]);
      this->setJointCommandValue(std::to_string(i), "effort", 0);
      this->setJointCommandValue(std::to_string(i), "mode", 2);
    }
  }

  void DampingHumanoidController::onStop()
  {
  }

  // Loads the damping configuration.
  bool DampingHumanoidController::loadDampingCfg()
  {
    try
    {
      // Declare and check parameters
      if (!get_node()->has_parameter("DampingHumanoidController.damping_kd")) {
        get_node()->declare_parameter<std::vector<double>>("DampingHumanoidController.damping_kd", dampingKd_);
      }
      if (!get_node()->get_parameter("DampingHumanoidController.damping_kd", dampingKd_)) {
        RCLCPP_FATAL(rclcpp::get_logger("DampingHumanoidController"), "Failed to get 'DampingHumanoidController.damping_kd' parameter");
        return false;
      }
    }
    catch (const std::exception &e)
    {
      // Error handling.
      RCLCPP_ERROR(rclcpp::get_logger("DampingHumanoidController"), "Error in the DampingHumanoidController: %s", e.what());
      return false;
    }

    return true;
  }
} // namespace

#include "pluginlib/class_list_macros.hpp"

// Export the class as a plugin.
PLUGINLIB_EXPORT_CLASS(robot_controllers::DampingHumanoidController, controller_interface::ControllerInterface)
