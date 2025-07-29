// Copyright information
//
// © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include "robot_controllers/MimicHumanoidController.h"

#include "controller_interface/helpers.hpp"

namespace robot_controllers
{
  MimicHumanoidController::MimicHumanoidController()
  {
  }

  bool MimicHumanoidController::onInit()
  {
    if (!loadRLCfg())
    {
      RCLCPP_ERROR(rclcpp::get_logger("MimicHumanoidController"), "Error in loadRLCfg");
      return false;
    }
    if (!loadModel())
    {
      RCLCPP_ERROR(rclcpp::get_logger("MimicHumanoidController"), "Error in loadModel");
      return false;
    }

    return true;
  }

  void MimicHumanoidController::onStart()
  {
    // Initialize state variables
    is_first_rec_obs_ = true;       // Flag for first observation capture
    loop_count_ = 0;                // Main loop counter
    std::fill(actions_.begin(), actions_.end(), 0.0);           // Reset action buffer
    std::fill(action_filtered_.begin(), action_filtered_.end(), 0.0); // Reset filtered actions
    std::fill(observations_.begin(), observations_.end(), 0.0); // Reset observation buffer
    last_actions_.setZero();        // Reset previous actions
    command_filtered_.setZero();    // Reset filtered commands
    motion_phase_ = 0;              // Reset motion progress
    motion_iter_ = 0;               // Reset motion frame index
  }

  void MimicHumanoidController::onUpdate()
  {
    // Run inference at decimated rate (reduces computational load)
    if (loop_count_ % decimation_ == 0) 
    {
      computeObservation();   // Update sensor observations
      computeLinEncoder();   // Run linear encoder inference
      computePrivEncoder();  // Run privileged encoder inference
      computeActions();       // Generate new actions via policy network

      // Clip actions to safe range
      double action_min = -clip_actions_;
      double action_max = clip_actions_;
      std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                      [action_min, action_max](double x) { return std::max(action_min, std::min(action_max, x)); });
      
      // Update history with new actions
      for (int i = 0; i < actions_.size(); ++i) 
        last_actions_(i) = actions_[i];
      
      motion_iter_++; // Advance motion sequence
    }

    // Generate motor commands from actions
    vector_t action_cliped(actions_size_);
    std::size_t action_index = 0;
    const float soft_torque_limit = 0.95; // 95% of hard limit for safety margin

    for (std::size_t i = 0; i < actions_size_; i++) 
    {
      // Calculate safe action bounds based on torque limits
      double position = this->getJointStateValue(std::to_string(i), "position");
      double velocity = this->getJointStateValue(std::to_string(i), "velocity");
      double action_min = position + (damping_[action_index] * velocity -
                                              user_torque_limit_[action_index] * soft_torque_limit) /
                                              stiffness_[action_index];
      double action_max = position + (damping_[action_index] * velocity +
                                              user_torque_limit_[action_index] * soft_torque_limit) /
                                              stiffness_[action_index];

      // Clip action to safe bounds and scale to motor command
      action_cliped[action_index] = std::max(action_min / action_scale_[action_index],
                                            std::min(action_max / action_scale_[action_index], actions_[action_index]));
      double pos_des = action_cliped[action_index] * action_scale_[action_index]; // Desired position

      this->setJointCommandValue(std::to_string(i), "position", pos_des);
      this->setJointCommandValue(std::to_string(i), "velocity", 0);
      this->setJointCommandValue(std::to_string(i), "kp", stiffness_[action_index]);
      this->setJointCommandValue(std::to_string(i), "kd", damping_[action_index]);
      this->setJointCommandValue(std::to_string(i), "effort", 0);
      this->setJointCommandValue(std::to_string(i), "mode", 2);

      action_index++;
    }

    loop_count_++;       // Increment loop counter
  }

  void MimicHumanoidController::onStop()
  {
  }

  bool MimicHumanoidController::loadModel()
  {
    // Construct paths to ONNX model files based on robot type
    std::string policy_file;
    std::string lin_encoder_file;
    std::string priv_encoder_file;

    if (declareAndCheckParameter<std::string>("robot_controllers_mimic_policy_file", policy_file))
    {
      RCLCPP_ERROR(rclcpp::get_logger("MimicHumanoidController"), "Failed to retrieve policy path from the parameter server!");
      return false;
    }

    if (declareAndCheckParameter<std::string>("robot_controllers_mimic_priv_encoder_file", priv_encoder_file))
    {
      RCLCPP_ERROR(rclcpp::get_logger("MimicHumanoidController"), "Failed to retrieve priv encoder path from the parameter server!");
      return false;
    }

    if (declareAndCheckParameter<std::string>("robot_controllers_mimic_lin_encoder_file", lin_encoder_file))
    {
      RCLCPP_ERROR(rclcpp::get_logger("MimicHumanoidController"), "Failed to retrieve lin encoder path from the parameter server!");
      return false;
    }

    // Initialize ONNX Runtime environment (manages logging and resources)
    onnxEnvPrt_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "MimicControllerOnnx"));

    // Configure ONNX session options (single-threaded execution for determinism)
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(1);
    sessionOptions.SetInterOpNumThreads(1);

    Ort::AllocatorWithDefaultOptions allocator; // Allocator for ONNX metadata

    // Load and initialize policy network (generates motor commands from observations)
    RCLCPP_INFO(rclcpp::get_logger("MimicHumanoidController"), "load encoder from: %s", policy_file.c_str());
    policy_session_ptr_ = std::make_unique<Ort::Session>(*onnxEnvPrt_, policy_file.c_str(), sessionOptions);
    
    // Extract input metadata (names and shapes) for policy network
    policy_input_names_.clear();
    policy_input_shapes_.clear();
    for (std::size_t i = 0; i < policy_session_ptr_->GetInputCount(); i++) 
    {
      policy_input_names_.push_back(policy_session_ptr_->GetInputName(i, allocator));
      policy_input_shapes_.push_back(policy_session_ptr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    }

    // Extract output metadata (names and shapes) for policy network
    policy_output_names_.clear();
    policy_output_shapes_.clear();
    for (std::size_t i = 0; i < policy_session_ptr_->GetOutputCount(); i++)
    {
      policy_output_names_.push_back(policy_session_ptr_->GetOutputName(i, allocator));
      policy_output_shapes_.push_back(policy_session_ptr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    }

    // Load and initialize linear encoder (processes motion references)
    RCLCPP_INFO(rclcpp::get_logger("MimicHumanoidController"), "load encoder from: %s", lin_encoder_file.c_str());
    lin_encoder_session_ptr_ = std::make_unique<Ort::Session>(*onnxEnvPrt_, lin_encoder_file.c_str(), sessionOptions);
    
    // Extract input/output metadata for linear encoder
    lin_encoder_input_names_.clear();
    lin_encoder_input_shapes_.clear();
    for (std::size_t j = 0; j < lin_encoder_session_ptr_->GetInputCount(); ++j) 
    {
      lin_encoder_input_names_.push_back(lin_encoder_session_ptr_->GetInputName(j, allocator));
      lin_encoder_input_shapes_.push_back(lin_encoder_session_ptr_->GetInputTypeInfo(j).GetTensorTypeAndShapeInfo().GetShape());
    }
    lin_encoder_output_names_.clear();
    lin_encoder_output_shapes_.clear();
    for (std::size_t j = 0; j < lin_encoder_session_ptr_->GetOutputCount(); ++j) 
    {
      lin_encoder_output_names_.push_back(lin_encoder_session_ptr_->GetOutputName(j, allocator));
      lin_encoder_output_shapes_.push_back(lin_encoder_session_ptr_->GetOutputTypeInfo(j).GetTensorTypeAndShapeInfo().GetShape());
    }

    // Load and initialize privileged encoder (processes proprioceptive data)
    RCLCPP_INFO(rclcpp::get_logger("MimicHumanoidController"), "load encoder from: %s", priv_encoder_file.c_str());
    priv_encoder_session_ptr_ = std::make_unique<Ort::Session>(*onnxEnvPrt_, priv_encoder_file.c_str(), sessionOptions);
    
    // Extract input/output metadata for privileged encoder
    priv_encoder_input_names_.clear();
    priv_encoder_input_shapes_.clear();
    for (std::size_t j = 0; j < priv_encoder_session_ptr_->GetInputCount(); ++j) 
    {
      priv_encoder_input_names_.push_back(priv_encoder_session_ptr_->GetInputName(j, allocator));
      priv_encoder_input_shapes_.push_back(priv_encoder_session_ptr_->GetInputTypeInfo(j).GetTensorTypeAndShapeInfo().GetShape());
    }
    priv_encoder_output_names_.clear();
    priv_encoder_output_shapes_.clear();
    for (std::size_t j = 0; j < priv_encoder_session_ptr_->GetOutputCount(); ++j) 
    {
      priv_encoder_output_names_.push_back(priv_encoder_session_ptr_->GetOutputName(j, allocator));
      priv_encoder_output_shapes_.push_back(priv_encoder_session_ptr_->GetOutputTypeInfo(j).GetTensorTypeAndShapeInfo().GetShape());
    }

    RCLCPP_INFO(rclcpp::get_logger("MimicHumanoidController"), "Successfully loaded ONNX models!");
    return true;
  }

  // Loads the reinforcement learning configuration.
  bool MimicHumanoidController::loadRLCfg()
  {
    try
    {
      int error = 0;

      // Total frames in motion reference
      error += declareAndCheckParameter<int>("MimicHumanoidController.motion_frames", motion_frames_);


      // Scales policy outputs to motor commands
      if (!get_node()->has_parameter("MimicHumanoidController.control.action_scale")) {
        get_node()->declare_parameter<std::vector<double>>("MimicHumanoidController.control.action_scale", action_scale_);
      }
      if (!get_node()->get_parameter("MimicHumanoidController.control.action_scale", action_scale_)) {
        RCLCPP_FATAL(rclcpp::get_logger("MimicHumanoidController"), "Failed to get 'MimicHumanoidController.control.action_scale' parameter");
        return false;
      }

      // Safety torque limits
      if (!get_node()->has_parameter("MimicHumanoidController.control.user_torque_limit")) {
        get_node()->declare_parameter<std::vector<double>>("MimicHumanoidController.control.user_torque_limit", user_torque_limit_);
      }
      if (!get_node()->get_parameter("MimicHumanoidController.control.user_torque_limit", user_torque_limit_)) {
        RCLCPP_FATAL(rclcpp::get_logger("MimicHumanoidController"), "Failed to get 'MimicHumanoidController.control.user_torque_limit' parameter");
        return false;
      }

      // Position control gains (stiffness)
      if (!get_node()->has_parameter("MimicHumanoidController.control.kp")) {
        get_node()->declare_parameter<std::vector<double>>("MimicHumanoidController.control.kp", stiffness_);
      }
      if (!get_node()->get_parameter("MimicHumanoidController.control.kp", stiffness_)) {
        RCLCPP_FATAL(rclcpp::get_logger("MimicHumanoidController"), "Failed to get 'MimicHumanoidController.control.kp' parameter");
        return false;
      }

      // Velocity control gains (damping)
      if (!get_node()->has_parameter("MimicHumanoidController.control.kd")) {
        get_node()->declare_parameter<std::vector<double>>("MimicHumanoidController.control.kd", damping_);
      }
      if (!get_node()->get_parameter("MimicHumanoidController.control.kd", damping_)) {
        RCLCPP_FATAL(rclcpp::get_logger("MimicHumanoidController"), "Failed to get 'MimicHumanoidController.control.kd' parameter");
        return false;
      }

      // Downsampling factor for inference
      error += declareAndCheckParameter<int32_t>("MimicHumanoidController.control.decimation", decimation_);

      // Observation clipping threshold
      error += declareAndCheckParameter<float>("MimicHumanoidController.normalization.clip_scales.clip_observations", clip_obs_);

      // Action clipping threshold
      error += declareAndCheckParameter<float>("MimicHumanoidController.normalization.clip_scales.clip_actions", clip_actions_);

      // Linear velocity scaling factor
      error += declareAndCheckParameter<float>("MimicHumanoidController.normalization.obs_scales.lin_vel", lin_vel_);

      // Angular velocity scaling factor
      error += declareAndCheckParameter<float>("MimicHumanoidController.normalization.obs_scales.ang_vel", ang_vel_);

      // Joint position scaling factor
      error += declareAndCheckParameter<float>("MimicHumanoidController.normalization.obs_scales.dof_pos", dof_pos_);

      // Joint position scaling factor
      error += declareAndCheckParameter<float>("MimicHumanoidController.normalization.obs_scales.dof_vel", dof_vel_);

      // Number of motor actions
      error += declareAndCheckParameter<int32_t>("MimicHumanoidController.size.actions_size", actions_size_);

      // Size of observation vector
      error += declareAndCheckParameter<int32_t>("MimicHumanoidController.size.observations_size", observations_size_);

      // Number of past observations to store
      error += declareAndCheckParameter<int32_t>("MimicHumanoidController.size.obs_history_length", obs_history_length_);

      // Output sizes of encoders
      std::vector<double> encoder_output_size_temp;
      if (!get_node()->has_parameter("MimicHumanoidController.size.encoder_output_size")) {
        get_node()->declare_parameter<std::vector<double>>("MimicHumanoidController.size.encoder_output_size", encoder_output_size_temp);
      }
      if (!get_node()->get_parameter("MimicHumanoidController.size.encoder_output_size", encoder_output_size_temp)) {
        RCLCPP_FATAL(rclcpp::get_logger("MimicHumanoidController"), "Failed to get 'MimicHumanoidController.size.encoder_output_size' parameter");
        return false;
      }

      encoder_output_size_.resize(encoder_output_size_temp.size());

      for(std::size_t i = 0; i < encoder_output_size_temp.size(); i++) {
        encoder_output_size_[i] = (int32_t)encoder_output_size_temp[i];
      } 

      // Log the result of parameter fetching
      if (error != 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger("MimicHumanoidController"), "Some parameters could not be retrieved. Number of errors: %d", error);
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("MimicHumanoidController"), "All parameters retrieved successfully.");
      }

      // Initialize buffers
      actions_.resize(actions_size_, 0.); // Raw policy outputs
      action_filtered_.resize(actions_size_, 0.); // Smoothed actions
      observations_.resize(observations_size_, 0.); // Current observation vector

      // Encoder output buffers
      lin_encoder_output_.resize(encoder_output_size_[0], 0.); // Output from linear encoder
      priv_encoder_output_.resize(encoder_output_size_[1], 0.); // Output from privileged encoder

      // State initialization
      last_actions_.resize(actions_size_);
      last_actions_.setZero(); // Previous actions for history
      command_filtered_.setZero(); // Smoothed user commands
    }
    catch (const std::exception &e)
    {
      // Error handling.
      RCLCPP_ERROR(rclcpp::get_logger("MimicHumanoidController"), "Error in the MimicHumanoidController: %s", e.what());
      return false;
    }

    return true;
  }

  // Computes actions using the policy model.
  void MimicHumanoidController::computeActions()
  {
    // Create memory buffer for ONNX tensor
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    std::vector<Ort::Value> input_values;
    std::vector<float> combined_obs = observations_; // Start with base observations

    // Append filtered commands to input
    combined_obs.push_back(command_filtered_(0));
    combined_obs.push_back(command_filtered_(1));
    combined_obs.push_back(command_filtered_(2));

    // Append encoder outputs to input
    for (const auto& item : lin_encoder_output_) 
      combined_obs.push_back(item);
    for (const auto& item : priv_encoder_output_) 
      combined_obs.push_back(item);

    // Create input tensor for policy network
    input_values.push_back(Ort::Value::CreateTensor<float>(
      memory_info, 
      combined_obs.data(), 
      combined_obs.size(),
      policy_input_shapes_[0].data(), 
      policy_input_shapes_[0].size()
    ));

    // Run policy inference
    Ort::RunOptions run_options;
    std::vector<Ort::Value> output_values = policy_session_ptr_->Run(
      run_options, 
      policy_input_names_.data(), 
      input_values.data(), 
      1,  // Number of inputs
      policy_output_names_.data(), 
      1   // Number of outputs
    );
    // Extract actions from policy output
    for (std::size_t i = 0; i < actions_size_; i++) 
    {
      actions_[i] = *(output_values[0].GetTensorMutableData<float>() + i);
    }
  }

  void MimicHumanoidController::computeObservation()
  {
    // Get IMU orientation
    Eigen::Quaternionf q_wi;
    q_wi.coeffs()(0) = this->getSensorValue("imu", "orientation_x");
    q_wi.coeffs()(1) = this->getSensorValue("imu", "orientation_y");
    q_wi.coeffs()(2) = this->getSensorValue("imu", "orientation_z");
    q_wi.coeffs()(3) = this->getSensorValue("imu", "orientation_w");

    // Compute projected gravity vector (used for orientation estimation)
    vector3_t gravity_vector(0, 0, -1); // Gravity in world frame
    vector3_t projected_gravity(q_wi.toRotationMatrix().transpose() * gravity_vector);

    // Extract angular velocity from IMU
    vector3_t base_ang_vel(
      this->getSensorValue("imu", "angular_velocity_x"),
      this->getSensorValue("imu", "angular_velocity_y"),
      this->getSensorValue("imu", "angular_velocity_z")
    );

    // Extract joint positions and velocities from robot state
    vector_t dof_joint_pos, dof_joint_vel;
    dof_joint_pos.resize(actions_size_);
    dof_joint_vel.resize(actions_size_);
    int count = 0;
    for (std::size_t i = 0; i < actions_size_; ++i) 
    {
      dof_joint_pos(count) = this->getJointStateValue(std::to_string(i), "position"); // Joint position
      dof_joint_vel(count++) = this->getJointStateValue(std::to_string(i), "velocity"); // Joint velocity
    }

    // Assemble observation vector
    vector_t actions(last_actions_); // Previous actions (for temporal context)
    Eigen::Matrix<float, Eigen::Dynamic, 1> obs(observations_size_);

    // Populate observation segments with normalized data
    obs.segment(0, 3) = base_ang_vel * ang_vel_; // Scaled angular velocity
    obs.segment(3, 3) = projected_gravity; // Projected gravity vector
    obs.segment(6, actions_size_) = dof_joint_pos * dof_pos_; // Scaled joint positions
    obs.segment(6 + actions_size_, actions_size_) = dof_joint_vel * dof_vel_; // Scaled joint velocities
    obs.segment(6 + 2 * actions_size_, actions_size_) = actions; // Previous actions

    // Update motion phase (normalized progress through motion sequence)
    motion_phase_ = static_cast<float>(motion_iter_) / motion_frames_;
    if (motion_phase_ >= 1.0) motion_phase_ = 1.0; // Clamp to 1.0 at end

    // Add motion phase to observation
    obs(6 + 3 * actions_size_) = motion_phase_;

    // Copy Eigen vector to std::vector for ONNX compatibility
    for (std::size_t i = 0; i < observations_size_; i++) 
    {
      observations_[i] = static_cast<float>(obs(i));
    }

    // Initialize observation history buffer on first run
    if (is_first_rec_obs_) 
    {
      proprio_history_buffer_.resize(obs_history_length_ * observations_size_);
      for (std::size_t i = 0; i < obs_history_length_; ++i) 
      {
        proprio_history_buffer_.segment(observations_size_ * i, observations_size_) = obs.head(observations_size_).cast<float>();
      }
      is_first_rec_obs_ = false;
    }

    // Update observation history (shift old data, add new observation)
    encoder_input_ = proprio_history_buffer_;
    for (std::size_t i = 0; i < observations_size_ * (obs_history_length_ - 1); ++i) 
    {
      proprio_history_buffer_[i + observations_size_] = encoder_input_[i]; // Shift left
    }
    proprio_history_buffer_.head(observations_size_) = obs.head(observations_size_).cast<float>(); // Add new observation

    // Clip observations to prevent extreme values
    float obs_min = -clip_obs_;
    float obs_max = clip_obs_;
    std::transform(observations_.begin(), observations_.end(), observations_.begin(),
                   [obs_min, obs_max](float x) { return std::max(obs_min, std::min(obs_max, x)); });
  }

  void MimicHumanoidController::computeLinEncoder() {
    // Create memory buffer for ONNX tensor
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    std::vector<Ort::Value> input_values;
    
    // Create input tensor from observation history
    input_values.push_back(Ort::Value::CreateTensor<float>(
      memory_info, 
      proprio_history_buffer_.data(), 
      proprio_history_buffer_.size(), 
      lin_encoder_input_shapes_[0].data(), 
      lin_encoder_input_shapes_[0].size()
    ));

    // Run linear encoder inference
    Ort::RunOptions run_options;
    std::vector<Ort::Value> output_values = lin_encoder_session_ptr_->Run(
      run_options, 
      lin_encoder_input_names_.data(),
      input_values.data(), 
      1,  // Number of inputs
      lin_encoder_output_names_.data(), 
      1   // Number of outputs
    );

    // Extract and store encoder outputs (used for policy input)
    for (std::size_t i = 0; i < encoder_output_size_[0]; ++i) 
    {
      lin_encoder_output_[i] = *(output_values[0].GetTensorMutableData<float>() + i);
      linear_velocity_estimated_[i] = lin_encoder_output_[i]; // Update velocity estimate
    }
  }

  void MimicHumanoidController::computePrivEncoder() {
    // Create memory buffer for ONNX tensor
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    std::vector<Ort::Value> input_values;
    
    // Create input tensor from observation history
    input_values.push_back(Ort::Value::CreateTensor<float>(
      memory_info, 
      proprio_history_buffer_.data(), 
      proprio_history_buffer_.size(), 
      priv_encoder_input_shapes_[0].data(), 
      priv_encoder_input_shapes_[0].size()
    ));

    // Run privileged encoder inference
    Ort::RunOptions run_options;
    std::vector<Ort::Value> output_values = priv_encoder_session_ptr_->Run(
      run_options, 
      priv_encoder_input_names_.data(),
      input_values.data(), 
      1,  // Number of inputs
      priv_encoder_output_names_.data(), 
      1   // Number of outputs
    );

    // Extract and store encoder outputs (used for policy input)
    for (std::size_t i = 0; i < encoder_output_size_[1]; ++i) 
    {
      priv_encoder_output_[i] = *(output_values[0].GetTensorMutableData<float>() + i);
    }
  }
} // namespace

#include "pluginlib/class_list_macros.hpp"

// Export the class as a plugin.
PLUGINLIB_EXPORT_CLASS(robot_controllers::MimicHumanoidController, controller_interface::ControllerInterface)
