// Copyright information
//
// © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_MIMIC_HUMANOID_CONTROLLER_H_
#define _LIMX_MIMIC_HUMANOID_CONTROLLER_H_

#include <onnxruntime_cxx_api.h>
#include <vector>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_controllers/ControllerBase.h"

namespace robot_controllers {
/**
 * @brief Class representing the MimicHumanoidController.
 */
class CONTROLLER_INTERFACE_PUBLIC MimicHumanoidController : public robot_controllers::ControllerBase {
public:
  MimicHumanoidController();

  bool onInit() override;
  void onUpdate() override;
  void onStart() override;
  void onStop() override;

  // Enumeration for controller modes
  enum class Mode : uint8_t {
    STAND,  // Stand mode
    WALK,   // Walk mode
  };

private:
  // Load RL configuration settings
  bool loadRLCfg();

  // Load the model for the controller
  bool loadModel();

  // Compute actions for the controller
  void computeActions();

  // Compute observations for the controller
  void computeObservation();
 
  // Execute linear encoder inference
  void computeLinEncoder();

  // Execute privileged encoder inference
  void computePrivEncoder();

private:
    // ONNX Runtime components
    std::shared_ptr<Ort::Env> onnxEnvPrt_;  // Shared ORT environment
    
    // Policy network configuration
    std::vector<std::vector<int64_t>> policy_input_shapes_;
    std::vector<std::vector<int64_t>> policy_output_shapes_;
    std::unique_ptr<Ort::Session> policy_session_ptr_;
    std::vector<const char *> policy_input_names_; 
    std::vector<const char *> policy_output_names_;

    // Linear encoder configuration
    std::vector<float> lin_encoder_output_;
    std::unique_ptr<Ort::Session> lin_encoder_session_ptr_;
    std::vector<const char *> lin_encoder_input_names_;
    std::vector<const char *> lin_encoder_output_names_;
    std::vector<std::vector<int64_t>> lin_encoder_input_shapes_;
    std::vector<std::vector<int64_t>> lin_encoder_output_shapes_;

    // Privileged encoder configuration  
    std::vector<float> priv_encoder_output_;
    std::unique_ptr<Ort::Session> priv_encoder_session_ptr_;
    std::vector<const char *> priv_encoder_input_names_;
    std::vector<const char *> priv_encoder_output_names_;
    std::vector<std::vector<int64_t>> priv_encoder_input_shapes_;
    std::vector<std::vector<int64_t>> priv_encoder_output_shapes_;

    // Motion control state
    float motion_phase_{0.};      // Normalized progress [0,1]
    float motion_times_{5.0};     // Total duration (sec)
    int motion_frames_{1000};     // Reference frames
    int motion_iter_{0};          // Current frame
    vector_t motion_cur_ref_;     // Current target
    std::vector<vector_t> motion_refs_;  // Full motion sequence
    vector3_t linear_velocity_estimated_{vector3_t::Zero()};  // Estimated linear velocity (x, y, z)

    // Safety limits
    std::vector<double> user_torque_limit_;  // Per-joint torque caps

    // Control parameters
    std::vector<double> action_scale_;  // Policy output scaling
    std::vector<double> stiffness_;     // Position gains (Kp)
    std::vector<double> damping_;       // Velocity gains (Kd)

    // Network configuration
    std::vector<int32_t> encoder_output_size_;  // Encoder output dims
    int32_t decimation_;        // Control cycle decimation
    int32_t actions_size_;      // Motor command count
    int32_t observations_size_; // Observation vector size  
    int32_t obs_history_length_;  // History window size

    // Normalization parameters
    float clip_obs_;     // Observation clip threshold
    float clip_actions_; // Action clip threshold
    float lin_vel_;      // Linear velocity scale
    float ang_vel_;      // Angular velocity scale  
    float dof_pos_;      // Joint position scale
    float dof_vel_;      // Joint velocity scale

    // Data buffers
    Eigen::Matrix<float, Eigen::Dynamic, 1> proprio_history_buffer_;  // Observation history
    Eigen::Matrix<float, Eigen::Dynamic, 1> encoder_input_;  // Encoder inputs

    // Action buffers
    std::vector<double> actions_;       // Raw policy outputs
    std::vector<float> observations_;  // Current observations
    std::vector<float> action_filtered_;  // Filtered actions

    // System state
    vector_t last_actions_;          // Previous actions
    vector3_t command_filtered_;     // Filtered commands
    bool is_first_rec_obs_;          // First obs flag
    int64_t loop_count_;             // Cycle counter
};

} // namespace robot_controllers

#endif //_LIMX_MIMIC_HUMANOID_CONTROLLER_H_
