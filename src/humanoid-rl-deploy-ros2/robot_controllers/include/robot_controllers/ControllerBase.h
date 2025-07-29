// Copyright information
//
// © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_CONTROLLER_BASE_H_
#define _LIMX_CONTROLLER_BASE_H_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <map>
#include <functional>
#include <Eigen/Geometry>
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/duration.hpp"
#include "urdf/model.h"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace robot_controllers {
// Define vector types for Eigen
using vector3_t = Eigen::Matrix<float, 3, 1>;
using vector_t = Eigen::Matrix<float, Eigen::Dynamic, 1>;
using matrix_t = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>; // Type alias for matrices

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Base class for robot controllers.
 */
class CONTROLLER_INTERFACE_PUBLIC ControllerBase : public controller_interface::ControllerInterface {
public:
  /**
   * @brief Constructor for ControllerBase.
   */
  ControllerBase() {}

  /**
   * @brief Virtual function to initialize the controller.
   * 
   * @return True if initialization is successful, false otherwise.
   */
  virtual bool onInit() = 0;

  /**
   * @brief Virtual function to update the controller.
   */
  virtual void onUpdate() = 0;

  /**
   * @brief Virtual function to start the controller.
   */
  virtual void onStart() = 0;

  /**
   * @brief Virtual function to stop the controller.
   */
  virtual void onStop() = 0;

public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  /**
   * @brief Sets the command value for a joint.
   * 
   * @param joint_index Index of the joint.
   * @param type Type of command (e.g., position, velocity).
   * @param value Command value to set.
   */
  void setJointCommandValue(const std::string & joint_index, const std::string & type, double value);

  /**
   * @brief Gets the state value of a joint.
   * 
   * @param joint_index Index of the joint.
   * @param type Type of state (e.g., position, velocity).
   * @return The state value of the joint.
   */
  double getJointStateValue(const std::string & joint_index, const std::string & type);

  /**
   * @brief Gets the value of a sensor.
   * 
   * @param sensor_name Name of the sensor.
   * @param type Type of sensor data (e.g., orientation, acceleration).
   * @return The sensor value.
   */
  double getSensorValue(const std::string & sensor_name, const std::string & type);

  /**
   * @brief Declares and checks a parameter.
   * 
   * @param param_name Name of the parameter.
   * @param param_value Value of the parameter.
   * @return 0 if the parameter is successfully declared and retrieved, 1 otherwise.
   */
  template<typename T>
  int declareAndCheckParameter(const std::string & param_name, T & param_value) {
    if (!get_node()->has_parameter(param_name)){
      get_node()->declare_parameter<T>(param_name, param_value);
    }
    return static_cast<int>(!get_node()->get_parameter(param_name, param_value));
  }

  std::map<std::string, hardware_interface::LoanedStateInterface*> state_interfaces_map_; ///< Map of state interfaces.
  std::map<std::string, hardware_interface::LoanedCommandInterface*> command_interfaces_map_; ///< Map of command interfaces.
  int jointNum_{31}; ///< joint number.
};
} // namespace robot_controllers

#endif // _LIMX_CONTROLLER_BASE_H_
