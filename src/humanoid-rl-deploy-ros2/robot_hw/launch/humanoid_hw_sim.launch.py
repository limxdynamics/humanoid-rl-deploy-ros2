from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import sys

def generate_launch_description():
    robot_type = os.getenv("ROBOT_TYPE")

    # Check if the ROBOT_TYPE environment variable is set, otherwise exit with an error
    if not robot_type:
        print("\033[31mError: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'.\033[0m")
        sys.exit(1)

    robot_controllers_mimic_priv_encoder_file = PathJoinSubstitution([FindPackageShare("robot_controllers"), "config/"+ robot_type +"/mimic/policy/priv_encoder.onnx"])
    robot_controllers_mimic_lin_encoder_file = PathJoinSubstitution([FindPackageShare("robot_controllers"), "config/"+ robot_type +"/mimic/policy/lin_encoder.onnx"])
    robot_controllers_mimic_policy_file = PathJoinSubstitution([FindPackageShare("robot_controllers"), "config/"+ robot_type +"/mimic/policy/policy.onnx"])
    robot_controllers_mimic_params_file = PathJoinSubstitution([FindPackageShare("robot_controllers"), "config/"+ robot_type +"/mimic/params.yaml"])
    robot_controllers_damping_params_file = PathJoinSubstitution([FindPackageShare("robot_controllers"), "config/"+ robot_type +"/damping/params.yaml"])
    robot_controllers_stand_params_file = PathJoinSubstitution([FindPackageShare("robot_controllers"), "config/"+ robot_type +"/stand/params.yaml"])
    robot_controllers_file = PathJoinSubstitution([FindPackageShare("robot_controllers"), "config/robot_controllers.yaml"])
    robot_hw_joystick_file = PathJoinSubstitution([FindPackageShare("robot_hw"), "config/joystick.yaml"])
    robot_hw_file = PathJoinSubstitution([FindPackageShare("robot_hw"), "config/robot_hw.yaml"])

    return LaunchDescription([
        Node(
            package='robot_hw',
            executable='humanoid_hw_node',
            name='robot_hw_node',
            output='screen',
            arguments=["127.0.0.1"],
            parameters=[
                {
                    "robot_controllers_mimic_priv_encoder_file": robot_controllers_mimic_priv_encoder_file,
                    "robot_controllers_mimic_lin_encoder_file": robot_controllers_mimic_lin_encoder_file,
                    "robot_controllers_mimic_policy_file": robot_controllers_mimic_policy_file,
                },
                robot_controllers_stand_params_file, 
                robot_controllers_damping_params_file, 
                robot_controllers_mimic_params_file,
                robot_controllers_stand_params_file, 
                robot_controllers_file,
                robot_hw_joystick_file,
                robot_hw_file,
            ],
        ),
    ])