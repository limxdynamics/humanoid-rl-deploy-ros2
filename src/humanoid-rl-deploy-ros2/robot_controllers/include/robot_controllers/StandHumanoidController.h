// Copyright information
//
// © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_STAND_HUMANOID_CONTROLLER_H_
#define _LIMX_STAND_HUMANOID_CONTROLLER_H_

#include <onnxruntime_cxx_api.h>
#include <vector>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_controllers/ControllerBase.h"

namespace robot_controllers {
/**
 * @brief Class representing the StandHumanoidController.
 */
class CONTROLLER_INTERFACE_PUBLIC StandHumanoidController : public robot_controllers::ControllerBase {
public:
  StandHumanoidController();

  bool onInit() override;
  void onUpdate() override;
  void onStart() override;
  void onStop() override;

private:
  // Load stand configuration settings
  bool loadStandCfg();

private:
  int64_t loopCount_;
  std::vector<double> stand_kp_, stand_kd_, stand_pos_;
  vector_t ik_standDesirejoints_;
  int64_t totalLoopCount_{2000};
  double ik_standPercent_{0.0};
  std::vector<double> initJointAngles_;

};

} // namespace robot_controllers

#endif //_LIMX_STAND_HUMANOID_CONTROLLER_H_
