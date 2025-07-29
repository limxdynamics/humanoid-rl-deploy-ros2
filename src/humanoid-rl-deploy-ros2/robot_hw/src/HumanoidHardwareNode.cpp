/**
 * @file HumanoidHWNode.cpp
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_hw/HumanoidHardware.h"
#include "robot_hw/HardwareLoop.h"

// Controller name
static std::string controller_name_ = "DampingHumanoidController";

// Publisher for sending velocity commands to the robot
static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_ = nullptr;

// Shared pointer to the hardware loop managing the robot's control loop
static std::shared_ptr<robot_hw::HardwareLoop> hw_loop_ = nullptr;

// Pointer to the Humanoid robot instance
static limxsdk::Humanoid *robot_ = nullptr;

// Variable to keep track of the calibration state of the robot
static int calibration_state_ = -1;

/**
 * @brief Declare and check a parameter
 *
 * Template function to declare a parameter if it does not exist, and then retrieve its value.
 *
 * @tparam T Type of the parameter
 * @param param_name Name of the parameter
 * @param param_value Reference to the variable to store the parameter value
 * @return true if the parameter was successfully retrieved, false otherwise
 */
template <typename T>
bool declareAndCheckParameter(const std::string &param_name, T &param_value)
{
  if (!hw_loop_->getNode()->has_parameter(param_name))
  {
    hw_loop_->getNode()->declare_parameter<T>(param_name, param_value);
  }
  return hw_loop_->getNode()->get_parameter(param_name, param_value);
}

/**
 * @brief Callback function for handling diagnostic messages
 *
 * This function processes diagnostic messages to monitor the calibration state and handle errors
 * related to EtherCAT and IMU.
 *
 * @param msg Shared pointer to the received diagnostic message
 */
static void subscribeDiagnosticValueCallback(const limxsdk::DiagnosticValueConstPtr &msg)
{
  // Check if the diagnostic message pertains to calibration
  if (msg->name == "calibration")
  {
    RCLCPP_WARN(rclcpp::get_logger("HumanoidHardwareNode"), "Calibration state: %d, msg: %s", msg->code, msg->message.c_str());
    calibration_state_ = msg->code;
  }

  // Check if the diagnostic message pertains to EtherCAT
  if (msg->name == "ethercat" && msg->level == limxsdk::DiagnosticValue::ERROR)
  {
    RCLCPP_FATAL(rclcpp::get_logger("HumanoidHardwareNode"), "Ethercat code: %d, msg: %s", msg->code, msg->message.c_str());
    hw_loop_->stopController(controller_name_);
    abort();
  }

  // Check if the diagnostic message pertains to IMU
  if (msg->name == "imu" && msg->level == limxsdk::DiagnosticValue::ERROR)
  {
    RCLCPP_FATAL(rclcpp::get_logger("HumanoidHardwareNode"), "IMU code: %d, msg: %s", msg->code, msg->message.c_str());
    hw_loop_->stopController(controller_name_);
    abort();
  }
}

/**
 * @brief Callback function for handling joystick input
 *
 * This function processes joystick input to start and stop the robot controller and publish
 * velocity commands based on the joystick axes.
 *
 * @param msg Shared pointer to the received joystick message
 */
static void subscribeSensorJoyCallback(const limxsdk::SensorJoyConstPtr &msg)
{
  // Logic for starting StandHumanoidController
  int BTN_L1, BTN_Y, BTN_X, BTN_A, BTN_B;
  if (declareAndCheckParameter("joystick_buttons.L1", BTN_L1) && declareAndCheckParameter("joystick_buttons.Y", BTN_Y))
  {
    if (msg->buttons[BTN_L1] == 1 && msg->buttons[BTN_Y] == 1)
    {
      hw_loop_->switchController("StandHumanoidController");
    }
  }

  // Logic for starting MimicHumanoidController
  if (declareAndCheckParameter("joystick_buttons.L1", BTN_L1) && declareAndCheckParameter("joystick_buttons.B", BTN_B))
  {
    if (msg->buttons[BTN_L1] == 1 && msg->buttons[BTN_B] == 1)
    {
      hw_loop_->switchController("MimicHumanoidController");
    }
  }

  // Logic for starting MimicHumanoidController
  if (declareAndCheckParameter("joystick_buttons.L1", BTN_L1) && declareAndCheckParameter("joystick_buttons.A", BTN_A))
  {
    if (msg->buttons[BTN_L1] == 1 && msg->buttons[BTN_A] == 1)
    {
      hw_loop_->switchController("DampingHumanoidController");
    }
  }

  // Logic for stopping controller
  if (declareAndCheckParameter("joystick_buttons.L1", BTN_L1) && declareAndCheckParameter("joystick_buttons.X", BTN_X))
  {
    if (msg->buttons[BTN_L1] == 1 && msg->buttons[BTN_X] == 1)
    {
      // hw_loop_->stopController("MimicHumanoidController");
      // hw_loop_->stopController("StandHumanoidController");
      hw_loop_->stopController("DampingHumanoidController");
    }
  }
}

/**
 * @brief Main function
 *
 * Initializes the ROS 2 node, sets up the hardware loop and subscribers, and starts the node
 * to process callbacks.
 *
 * @param argc Number of command-line arguments
 * @param argv Array of command-line arguments
 * @return int Exit status
 */
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Default robot IP address, can be overridden by command-line argument
  std::string robot_ip = "127.0.0.1";
  if (argc > 1)
  {
    robot_ip = argv[1];
  }

  // Get an instance of the Humanoid robot
  robot_ = limxsdk::Humanoid::getInstance();

  // Initialize the Humanoid instance with the robot IP
  if (!robot_->init(robot_ip))
  {
    RCLCPP_ERROR(rclcpp::get_logger("HumanoidHardwareNode"), "Failed to connect to the robot: %s", robot_ip.c_str());
    abort();
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("HumanoidHardwareNode"), "Connected to the robot: %s", robot_ip.c_str());
  }

  // Create a unique pointer to the HumanoidHardware object
  std::unique_ptr<robot_hw::HardwareBase> hw = std::make_unique<robot_hw::HumanoidHardware>();

  // Create a HardwareLoop object with the node and hardware
  hw_loop_ = std::make_shared<robot_hw::HardwareLoop>(hw);

  // Start the hardware loop
  hw_loop_->start();

  // Retrieve the value of the "use_gazebo" parameter
  bool use_gazebo = false;
  if (!declareAndCheckParameter("use_gazebo", use_gazebo))
  {
    use_gazebo = false;
  }

  // Create a publisher for velocity commands
  cmd_vel_pub_ = hw_loop_->getNode()->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Subscribe to joystick and diagnostic messages
  robot_->subscribeSensorJoy(&subscribeSensorJoyCallback);
  robot_->subscribeDiagnosticValue(&subscribeDiagnosticValueCallback);

  // Spin the node to process callbacks and keep it alive
  rclcpp::spin(hw_loop_->getNode());

  // Shutdown the ROS 2 system
  rclcpp::shutdown();
  return 0;
}
