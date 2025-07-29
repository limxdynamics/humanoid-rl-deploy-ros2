// Copyright information
//
// © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include "robot_controllers/StandHumanoidController.h"

#include "controller_interface/helpers.hpp"

namespace robot_controllers
{
  StandHumanoidController::StandHumanoidController()
  {
  }

  bool StandHumanoidController::onInit()
  {
    if (!loadStandCfg())
    {
      RCLCPP_ERROR(rclcpp::get_logger("StandHumanoidController"), "Error in loadStandCfg");
      return false;
    }
    return true;
  }

  void StandHumanoidController::onStart()
  {
    ik_standPercent_ = 0;
    loopCount_ = 0;
    initJointAngles_.resize(jointNum_, 0.0);
    for (std::size_t i = 0; i < jointNum_; i++)
    {
      initJointAngles_[i] = this->getJointStateValue(std::to_string(i), "position");
    }
  }

  void StandHumanoidController::onUpdate()
  {
    if (loopCount_ < totalLoopCount_) {
      ik_standPercent_ = loopCount_ / static_cast<double>(totalLoopCount_);
    } else {
      ik_standPercent_ = 1.0;
    }

    for (std::size_t i = 0; i < jointNum_; i++) {
      double pos_des = initJointAngles_[i] * (1 - ik_standPercent_) + stand_pos_[i] * ik_standPercent_;
      this->setJointCommandValue(std::to_string(i), "position", pos_des);
      this->setJointCommandValue(std::to_string(i), "velocity", 0);
      this->setJointCommandValue(std::to_string(i), "kp", stand_kp_[i]);
      this->setJointCommandValue(std::to_string(i), "kd", stand_kd_[i]);
      this->setJointCommandValue(std::to_string(i), "effort", 0);
      this->setJointCommandValue(std::to_string(i), "mode", 2);
    }

    loopCount_++;
  }

  void StandHumanoidController::onStop()
  {
  }

  // Loads the reinforcement learning configuration.
  bool StandHumanoidController::loadStandCfg()
  {
    try
    {
      // Declare and check parameters
      if (!get_node()->has_parameter("StandHumanoidController.stand_pos")) {
        get_node()->declare_parameter<std::vector<double>>("StandHumanoidController.stand_pos", stand_pos_);
      }
      if (!get_node()->get_parameter("StandHumanoidController.stand_pos", stand_pos_)) {
        RCLCPP_FATAL(rclcpp::get_logger("StandHumanoidController"), "Failed to get 'StandHumanoidController.stand_pos' parameter");
        return false;
      }

      if (!get_node()->has_parameter("StandHumanoidController.stand_kp")) {
        get_node()->declare_parameter<std::vector<double>>("StandHumanoidController.stand_kp", stand_kp_);
      }
      if (!get_node()->get_parameter("StandHumanoidController.stand_kp", stand_kp_)) {
        RCLCPP_FATAL(rclcpp::get_logger("StandHumanoidController"), "Failed to get 'StandHumanoidController.stand_kp' parameter");
        return false;
      }

      if (!get_node()->has_parameter("StandHumanoidController.stand_kd")) {
        get_node()->declare_parameter<std::vector<double>>("StandHumanoidController.stand_kd", stand_kd_);
      }
      if (!get_node()->get_parameter("StandHumanoidController.stand_kd", stand_kd_)) {
        RCLCPP_FATAL(rclcpp::get_logger("StandHumanoidController"), "Failed to get 'StandHumanoidController.stand_kd' parameter");
        return false;
      }
    }
    catch (const std::exception &e)
    {
      // Error handling.
      RCLCPP_ERROR(rclcpp::get_logger("StandHumanoidController"), "Error in the ControllerCfg: %s", e.what());
      return false;
    }

    return true;
  }
} // namespace

#include "pluginlib/class_list_macros.hpp"

// Export the class as a plugin.
PLUGINLIB_EXPORT_CLASS(robot_controllers::StandHumanoidController, controller_interface::ControllerInterface)
