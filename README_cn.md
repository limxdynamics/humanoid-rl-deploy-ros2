# 中文 | [English](README.md)
# 训练结果部署



## 1. 部署环境配置

- 安装ROS 2 Foxy：在 Ubuntu 20.04 操作系统上建立基于 ROS 2 Foxy 的算法开发环境。 安装请参考文档： https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html ，选择“ros-foxy-desktop”进行安装。ROS 2 Foxy 安装完成后，Bash 终端输入以下 Shell 命令，安装开发环境所依赖的库：

    ```
    sudo apt update
    sudo apt install ros-foxy-urdf \
                     ros-foxy-urdfdom \
                     ros-foxy-urdfdom-headers \
                     ros-foxy-kdl-parser \
                     ros-foxy-hardware-interface \
                     ros-foxy-controller-manager \
                     ros-foxy-controller-interface \
                     ros-foxy-controller-manager-msgs \
                     ros-foxy-control-msgs \
                     ros-foxy-controller-interface \
                     ros-foxy-gazebo-* \
                     ros-foxy-rviz* \
                     ros-foxy-rqt-gui \
                     ros-foxy-rqt-robot-steering \
                     ros-foxy-plotjuggler* \
                     ros-foxy-control-toolbox \
                     ros-foxy-ros2-control \
                     ros-foxy-ros2-controllers \
                     ros-foxy-robot-controllers \
                     ros-foxy-robot-controllers-interface \
                     ros-foxy-robot-controllers-msgs \
                     ros-dev-tools \
                     cmake build-essential libpcl-dev libeigen3-dev libopencv-dev libmatio-dev \
                     python3-pip libboost-all-dev libtbb-dev liburdfdom-dev liborocos-kdl-dev -y
    ```

    

- 安装onnxruntime依赖，下载连接：https://github.com/microsoft/onnxruntime/releases/tag/v1.10.0  。请您根据自己的操作系统和平台选择合适版本下载。如在Ubuntu 20.04 x86_64，请按下面步骤进行安装：
  
    ```Bash
    wget https://github.com/microsoft/onnxruntime/releases/download/v1.10.0/onnxruntime-linux-x64-1.10.0.tgz
    
    tar xvf onnxruntime-linux-x64-1.10.0.tgz
    
    sudo cp -a onnxruntime-linux-x64-1.10.0/include/* /usr/include
    sudo cp -a onnxruntime-linux-x64-1.10.0/lib/* /usr/lib
    ```



## 2. 创建工作空间

可以按照以下步骤，创建一个RL部署开发工作空间：

- 打开一个Bash终端。

- 创建一个新目录来存放工作空间。例如，可以在用户的主目录下创建一个名为“limx_ws”的目录：

  ```Bash
  mkdir -p ~/limx_ws
  ```

- 下载 MuJoCo 仿真器

  ```Bash
  cd ~/limx_ws
  
  # Option 1: HTTPS
  git clone --recurse https://github.com/limxdynamics/humanoid-mujoco-sim.git
  
  # Option 2: SSH
  git clone --recurse git@github.com:limxdynamics/humanoid-mujoco-sim.git
  ```

- 下载运动控制算法：

  ```Bash
  cd ~/limx_ws
  
  # Option 1: HTTPS
  git clone -b feature/foxy --recurse https://github.com/limxdynamics/humanoid-rl-deploy-ros2.git
  
  # Option 2: SSH
  git clone -b feature/foxy --recurse git@github.com:limxdynamics/humanoid-rl-deploy-ros2.git
  ```


- 设置机器人型号：如果尚未设置，请按照以下步骤进行设置。

  - 通过 Shell 命令 `tree -L 1 humanoid-rl-deploy-ros2/robot_controllers/config` 列出可用的机器人类型：

    ```
    cd ~/limx_ws/humanoid-rl-deploy-2/src
    tree -L 1 humanoid-rl-deploy-ros2/robot_controllers/config
    humanoid-rl-deploy-ros2/robot_controllers/config
    └── HU_D03_03
    
    ```

  - 以`HU_D03_03`（请根据实际机器人类型进行替换）为例，设置机器人型号类型：

    ```
    echo 'export ROBOT_TYPE=HU_D03_03' >> ~/.bashrc && source ~/.bashrc
    ```

### 4、仿真调试

- 运行MuJoco 仿真器（推荐Pyhon 3.8 及以上版本）

  - 打开一个 Bash 终端。

  - 安装运动控制开发库：

    - Linux x86_64 环境

      ```bash
      cd ~/limx_ws
      pip install humanoid-mujoco-sim/limxsdk-lowlevel/python3/amd64/limxsdk-*-py3-none-any.whl
      ```

    - Linux aarch64 环境

      ```bash
      cd ~/limx_ws
      pip install humanoid-mujoco-sim/limxsdk-lowlevel/python3/aarch64/limxsdk-*-py3-none-any.whl
      ```

  - 运行 MuJoCo 仿真器：

    ```bash
    cd ~/limx_ws
    python humanoid-mujoco-sim/simulator.py
    ```

- 运行算法

  - 打开一个 Bash 终端。

  - 请进到您的工作空间，完成编译：

    ```bash
    # 如您安装了Conda，请临时禁用 Conda 环境
    # 因为 Conda 会干扰 ROS 的运行环境设置
    conda deactivate
    
    # 设置ROS编译环境
    source /opt/ros/foxy/setup.bash
    
    # 编译算法代码
    cd ~/limx_ws/humanoid-rl-deploy-ros2
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

  - 运行算法

    ```bash
    # 如您安装了Conda，请临时禁用 Conda 环境
    # 因为 Conda 会干扰 ROS 的运行环境设置
    conda deactivate
    
    # 设置ROS编译环境
    source /opt/ros/foxy/setup.bash
    
    # 运行算法
    cd ~/limx_ws/humanoid-rl-deploy-ros2
    source install/setup.bash
    ros2 launch robot_hw humanoid_hw_sim.launch.py
    ```

    ![](doc/simulator.gif)

- 虚拟遥控器：仿真的时候可以使用虚拟遥控器来操作机器人。以下是使用虚拟遥控器的具体步骤。

  - 打开一个 Bash 终端。

  - 运行虚拟遥控器

    ```
    ~/limx_ws/humanoid-mujoco-sim/robot-joystick/robot-joystick
    ```

    ![](doc/robot-joystick.png)


  - 这时，您可以使用虚拟遥控器来控制机器人。

    | **按键** | **模式**         | **说明**                                                    |
    | -------- | ---------------- | ----------------------------------------------------------- |
    | L1+Y     | 切换到站立模式   | 如机器人没法站立，请点击MuJoco界面中的“Reset”进行复位一下。 |
    | L1+B     | 切换到打招呼模式 |                                                             |

### 4、真机调试

- 设置您电脑IP：确保您的电脑与机器人本体通过外置网口连接。设置您的电脑IP地址为：`10.192.1.200`，并通过Shell命令`ping 10.192.1.2` 能够正常ping通。如下图所示对您的开发电脑进行IP设置：

  ![img](doc/ip.png)

- 算法编译：

  ```bash
  # 如您安装了Conda，请临时禁用 Conda 环境
  # 因为 Conda 会干扰 ROS 的运行环境设置
  conda deactivate
  
  # 设置ROS编译环境
  source /opt/ros/foxy/setup.bash
  
  # 编译算法代码
  cd ~/limx_ws/humanoid-rl-deploy-ros2
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

- 机器人准备工作：

  - 请通过机器人左右肩膀的挂钩把机器人吊起来。
  - 按电源键开机后，按下遥控器`右摇杆`按键，启动机器人电机。
  - 按下遥控器按键 `L1 + START`切换到开发者模式。在此模式下，用户可以开发自己的运动控制算法。（此模式设置后，下次开机继续生效，如需退出开发者模式请按`L1 + L2 + START`退出）。

- 实机部署运行。在Bash终端只需下面Shell命令启动控制算法：

  ```bash
  # 如您安装了Conda，请临时禁用 Conda 环境
  # 因为 Conda 会干扰 ROS 的运行环境设置
  conda deactivate
  
  # 设置ROS编译环境
  source /opt/ros/foxy/setup.bash
  
  # 运行算法
  cd ~/limx_ws/humanoid-rl-deploy-ros2
  source install/setup.bash
  ros2 launch robot_hw humanoid_hw.launch.py
  ```

- 这时您可以通过遥控器按键`L1 + Y`让机器人进入站立模式。

- 遥控器按`L1 + B`控制机器人打招呼。
