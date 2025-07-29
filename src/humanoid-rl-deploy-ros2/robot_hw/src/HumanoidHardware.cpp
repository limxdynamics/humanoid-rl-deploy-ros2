// Copyright information
//
// © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include "robot_hw/HumanoidHardware.h"

namespace robot_hw
{

  // Constructor for the HumanoidHardware class
  // Initializes the base hardware with the Humanoid instance from limxsdk
  HumanoidHardware::HumanoidHardware()
      : robot_hw::HardwareBase(limxsdk::Humanoid::getInstance()) {}

} // namespace robot_hw
