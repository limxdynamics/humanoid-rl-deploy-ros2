// Copyright information
//
// © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_DAMPING_HUMANOID_CONTROLLER_H_
#define _LIMX_DAMPING_HUMANOID_CONTROLLER_H_

#include <onnxruntime_cxx_api.h>
#include <vector>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_controllers/ControllerBase.h"

namespace robot_controllers {
/**
 * @brief Class representing the DampingHumanoidController.
 */
class CONTROLLER_INTERFACE_PUBLIC DampingHumanoidController : public robot_controllers::ControllerBase {
public:
  DampingHumanoidController();

  bool onInit() override;
  void onUpdate() override;
  void onStart() override;
  void onStop() override;

private:
  
  // Load Damping parameters
  bool loadDampingCfg();

private:
  std::vector<double> dampingKd_;
};

} // namespace robot_controllers

#endif //_LIMX_DAMPING_HUMANOID_CONTROLLER_H_
