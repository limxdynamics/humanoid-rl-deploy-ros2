<p align="center">
  <img src="doc/banner.png" alt="humanoid-rl-deploy-ros2: the LimX HU_D04 humanoid waving, driven by an ONNX policy" width="100%">
</p>

<p align="center">
  <b>English</b> | <a href="README_cn.md">中文</a>
</p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-Apache--2.0-2f6feb" alt="License: Apache-2.0"></a>
  <img src="https://img.shields.io/badge/ROS%202-Humble-2f6feb" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/Ubuntu-22.04-2f6feb" alt="Ubuntu 22.04">
  <img src="https://img.shields.io/badge/ONNX%20Runtime-1.10-2f6feb" alt="ONNX Runtime 1.10">
</p>

# humanoid-rl-deploy-ros2

Deploy reinforcement-learning policies on the LimX humanoid with ROS 2 and
`ros2_control`: the same build runs in the MuJoCo simulator and on the real
robot, and the remote controller switches between standing and the policy.

> **Scope:** the ROS 2 deployment side. It ships a hardware node, three
> controllers and one example policy (a greeting motion) as ONNX files. Policy
> training is not part of this repository; the simulator is
> [humanoid-mujoco-sim](https://github.com/limxdynamics/humanoid-mujoco-sim).

## Highlights

- **Sim and real from one build.** Two launch files that differ only in the
  robot address: `127.0.0.1` for MuJoCo, `10.192.1.2` for the robot.
- **ONNX policy, no Python at runtime.** C++ inference with ONNX Runtime: a
  policy plus two encoders, run at 100 Hz inside a 1 kHz control loop.
- **Controllers on the remote.** `L1 + Y` stands up, `L1 + B` plays the policy,
  `L1 + A` switches to damping.
- **Safety clamps in the controller.** Every joint target is clipped so the PD
  torque stays within 95% of a per-joint torque limit.
- **Standard `ros2_control` plugins.** Controllers load through pluginlib, so a
  new controller is one more class and one more line in the config.

## Supported robots

| | Model | Description | DoF | Format |
|:-:|---|---|:-:|---|
| <img src="doc/model_hu_d04_01.png" width="320"> | `HU_D04_01` | Full-size humanoid: legs, 3-DoF waist, 2-DoF head, 7-DoF arms | 31 | ONNX policy + YAML |
| <img src="doc/model_hu_d03_03.png" width="320"> | `HU_D03_03` | Previous generation, discontinued ([humanoid-description](https://github.com/limxdynamics/humanoid-description)) | 31 | ONNX policy + YAML |

Pick one with `ROBOT_TYPE` (see [Create a Workspace](#2-create-a-workspace)).

## What it does

<p align="center">
  <img src="doc/controls_en.png" alt="Remote controller: L1 plus Y makes the humanoid stand, L1 plus B makes it wave" width="100%">
</p>

<p align="center">
  <img src="doc/motion_en.png" alt="Six frames of HU_D04_01 in MuJoCo running the shipped policy: stand, turn and raise the right arm, wave, lower the arm, back to neutral" width="100%">
</p>

<p align="center">
  <img src="doc/sim2real_en.png" alt="The same controller drives the humanoid in MuJoCo on a laptop and the real robot over Ethernet at 10.192.1.2" width="100%">
</p>

## Quick start

In simulation, on Ubuntu 22.04 with ROS 2 Humble and ONNX Runtime installed
([section 1](#1-deployment-environment-configuration)):

```bash
mkdir -p ~/limx_ws && cd ~/limx_ws
git clone --recurse https://github.com/limxdynamics/humanoid-mujoco-sim.git
git clone --recurse https://github.com/limxdynamics/humanoid-rl-deploy-ros2.git
echo 'export ROBOT_TYPE=HU_D04_01' >> ~/.bashrc && source ~/.bashrc

# terminal 1: simulator
pip install humanoid-mujoco-sim/limxsdk-lowlevel/python3/amd64/limxsdk-*.whl
python humanoid-mujoco-sim/simulator.py

# terminal 2: controller
source /opt/ros/humble/setup.bash
cd ~/limx_ws/humanoid-rl-deploy-ros2
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch robot_hw humanoid_hw_sim.launch.py
```

Then start the virtual remote
(`~/limx_ws/humanoid-mujoco-sim/robot-joystick/robot-joystick`) and press
`L1 + Y`, then `L1 + B`.

## 1. Deployment Environment Configuration

- Install ROS 2 Humble: Set up a ROS 2 Humble-based algorithm Development Environment on the Ubuntu 22.04 operating system. For installation, please refer to the documentation: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html, and choose "ros-humble-desktop" for installation. After the installation of ROS 2 Humble is completed, enter the following Shell commands in the Bash end point to install the libraries required by the Development Environment:

    ```bash
    sudo apt update
    sudo apt install ros-humble-urdf \
                ros-humble-urdfdom \
                ros-humble-urdfdom-headers \
                ros-humble-kdl-parser \
                ros-humble-hardware-interface \
                ros-humble-controller-manager \
                ros-humble-controller-interface \
                ros-humble-controller-manager-msgs \
                ros-humble-control-msgs \
                ros-humble-controller-interface \
                ros-humble-gazebo-* \
                ros-humble-rviz* \
                ros-humble-rqt-gui \
                ros-humble-rqt-robot-steering \
                ros-humble-plotjuggler* \
                ros-humble-control-toolbox \
                ros-humble-ros2-control \
                ros-humble-ros2-controllers \
                ros-dev-tools \
                cmake build-essential libpcl-dev libeigen3-dev libopencv-dev libmatio-dev \
                python3-pip libboost-all-dev libtbb-dev liburdfdom-dev liborocos-kdl-dev -y
    ```

    

- Install the onnxruntime dependency, download link:https://github.com/microsoft/onnxruntime/releases/tag/v1.10.0. Please choose the appropriate version to download according to your operating system and platform. For example, on Ubuntu 20.04 x86_64, please follow the steps below for installation:
  
    ```Bash
    wget https://github.com/microsoft/onnxruntime/releases/download/v1.10.0/onnxruntime-linux-x64-1.10.0.tgz
    
    tar xvf onnxruntime-linux-x64-1.10.0.tgz
    
    sudo cp -a onnxruntime-linux-x64-1.10.0/include/* /usr/include
    sudo cp -a onnxruntime-linux-x64-1.10.0/lib/* /usr/lib
    ```



## 2. Create a Workspace

You can create an RL deployment development workspace by following these steps:
- Open a Bash terminal.
- Create a new directory to store the workspace. For example, you can create a directory named "limx_ws" in the user's home directory:
    ```Bash
    mkdir -p ~/limx_ws/src
    ```
    
- Download the MuJoCo simulator
    ```Bash
    cd ~/limx_ws

    # Option 1: HTTPS
    git clone --recurse https://github.com/limxdynamics/humanoid-mujoco-sim.git

    # Option 2: SSH
    git clone --recurse git@github.com:limxdynamics/humanoid-mujoco-sim.git
    ```
    
- Download the motion control algorithm:
    ```Bash
    cd ~/limx_ws

    # Option 1: HTTPS
    git clone --recurse https://github.com/limxdynamics/humanoid-rl-deploy-ros2.git

    # Option 2: SSH
    git clone --recurse git@github.com:limxdynamics/humanoid-rl-deploy-ros2.git
    ```
    
- Set the robot model: If not set yet, follow these steps.

  - List the available robot types using the Shell command `tree -L 1 humanoid-rl-deploy-ros2/robot_controllers/config` ：

    ```
    cd ~/limx_ws/humanoid-rl-deploy-ros2/src
    tree -L 1 humanoid-rl-deploy-ros2/robot_controllers/config
    humanoid-rl-deploy-ros2/robot_controllers/config
    ├── HU_D03_03
    └── HU_D04_01
    
    ```

  - Take `HU_D04_01` (please replace it with the actual robot type) as an example to set the robot model type:

    ```
    echo 'export ROBOT_TYPE=HU_D04_01' >> ~/.bashrc && source ~/.bashrc
    ```

## 3. Simulation Debugging

- Run the MuJoco simulator (Python 3.8 or above is recommended)

  - Open a Bash terminal.

  - Install the motion control development library:

    - Linux x86_64 environment

      ```bash
      cd ~/limx_ws
      pip install humanoid-mujoco-sim/limxsdk-lowlevel/python3/amd64/limxsdk-*.whl
      ```

    - Linux aarch64 environment

      ```bash
      cd ~/limx_ws
      pip install humanoid-mujoco-sim/limxsdk-lowlevel/python3/aarch64/limxsdk-*.whl
      ```

  - Run the MuJoCo simulator:

    ```bash
    cd ~/limx_ws
    python humanoid-mujoco-sim/simulator.py
    ```

- Run the algorithm

  - Open a Bash terminal.

  - Navigate to your workspace and complete the compilation:

    ```bash
    # If you have Conda installed, temporarily deactivate the Conda environment
    # Because Conda may interfere with the ROS runtime environment settings
    conda deactivate

    # Set up the ROS compilation environment
    source /opt/ros/humble/setup.bash

    # Compile the algorithm code
    cd ~/limx_ws/humanoid-rl-deploy-ros2
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

  - Run the algorithm

    ```bash
    # If you have Conda installed, temporarily deactivate the Conda environment
    # Because Conda may interfere with the ROS runtime environment settings
    conda deactivate

    # Set up the ROS compilation environment
    source /opt/ros/humble/setup.bash

    # Run the algorithm
    cd ~/limx_ws/humanoid-rl-deploy-ros2
    source install/setup.bash
    ros2 launch robot_hw humanoid_hw_sim.launch.py
    ```

    ![](doc/simulator.gif)

- Virtual remote controller: You can use a virtual remote controller to operate the robot during simulation. The following are the specific steps to use the virtual remote controller.

  - Open a Bash terminal.

  - Run the virtual remote controller

    ```
    ~/limx_ws/humanoid-mujoco-sim/robot-joystick/robot-joystick
    ```

    ![](doc/robot-joystick.png)


  - At this point, you can use the virtual remote controller to control the robot.

    | **Button** | **Mode**         | **Description**                                                    |
    | -------- | ---------------- | ----------------------------------------------------------- |
    | L1+Y     | Switch to standing mode   | If the robot cannot stand, click "Reset" in the MuJoco interface to reset it. |
    | L1+B     | Switch to greeting mode |                                                             |

## 4. Real Machine Debugging

- Set your computer's IP: Ensure that your computer is connected to the robot body through an external network port. Set your computer's IP address to: `10.192.1.200`, and you can successfully ping `10.192.1.2` using the Shell command `ping 10.192.1.2`. Set the IP of your development computer as shown in the following figure:

  ![img](doc/ip.png)

- Algorithm compilation:

  ```bash
  # If you have Conda installed, temporarily deactivate the Conda environment
  # Because Conda may interfere with the ROS runtime environment settings
  conda deactivate

  # Set up the ROS compilation environment
  source /opt/ros/humble/setup.bash

  # Compile the algorithm code
  cd ~/limx_ws/humanoid-rl-deploy-ros2
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

- Robot preparation:

  - Hang the robot with the hooks on the left and right shoulders.
  - After turning on the power, press the right joystick button on the remote controller to start the robot's motors.
  - Press the remote controller buttons `R1 + DOWN` to switch to developer mode. In this mode, users can develop their own motion control algorithms. (This mode will remain effective after the next startup; to exit developer mode, press `R1 + LEFT`).

- Real machine deployment and operation. In the Bash terminal, simply use the following Shell command to start the control algorithm:

  ```bash
  # If you have Conda installed, temporarily deactivate the Conda environment
  # Because Conda may interfere with the ROS runtime environment settings
  conda deactivate

  # Set up the ROS compilation environment
  source /opt/ros/humble/setup.bash

  # Run the algorithm
  cd ~/limx_ws/humanoid-rl-deploy-ros2
  source install/setup.bash
  ros2 launch robot_hw humanoid_hw.launch.py
  ```

- At this point, you can use the remote controller button `L1 + △` to make the robot enter the standing mode.

- Press `L1 + 〇` on the remote controller to control the robot to greet.

## 5. Controllers and the remote

`robot_hw_node` runs a 1 kHz loop with a `controller_manager` and switches
controllers from the remote (`robot_hw/src/HumanoidHardwareNode.cpp`):

| Buttons | Controller | What it does |
|---|---|---|
| `L1 + Y` | `StandHumanoidController` | Moves from the current pose to `stand_pos` with `stand_kp` / `stand_kd` |
| `L1 + B` | `MimicHumanoidController` | Runs the ONNX policy (the greeting motion) |
| `L1 + A` | `DampingHumanoidController` | Holds every joint with damping only (`damping_kd`) |
| `L1 + X` | stops `DampingHumanoidController` | |

On the robot's own remote, Y and B are the `△` and `〇` buttons.
`DampingHumanoidController` is also stopped automatically if the robot reports an
EtherCAT or IMU error.

## 6. Use your own policy

`MimicHumanoidController` loads three ONNX files from
`robot_controllers/config/$ROBOT_TYPE/mimic/policy/` and its parameters from
`mimic/params.yaml` next to them. Every 10 control ticks (100 Hz):

1. It builds a 100-value observation: base angular velocity (x 0.25), projected
   gravity, 31 joint positions, 31 joint velocities (x 0.05), the last action,
   and the motion phase (`step / motion_frames`).
2. It keeps the last 10 observations (1000 values) and feeds them to
   `lin_encoder.onnx` (3 outputs) and `priv_encoder.onnx` (32 outputs).
3. `policy.onnx` takes the observation, a 3-value command (zero in this
   controller) and both encoder outputs, 138 values in all, and returns 31
   actions.
4. Each joint target is `action x action_scale`, sent with `kp` / `kd`.

To deploy another policy with the same interface, replace the three files and
update `params.yaml` (`motion_frames`, `action_scale`, `kp`, `kd`, torque limits),
then run `colcon build` again so the installed copy under `install/` is updated.
Joint order is legs (left, right), waist, head, left arm, right arm.

## License

Apache License 2.0; see [LICENSE](LICENSE) and [NOTICE](NOTICE).
