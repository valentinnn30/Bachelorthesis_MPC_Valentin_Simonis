This repository contains all files necessary to reproduce the experiments with the robot using ROS2 conducted during my bachelor thesis. It is derived from the ReefRanger repository but has been simplified to include only the components relevant to the tests.

First, this file provides an overview of all dependencies and environment specifications. Below, you will find the instructions on how to reproduce the commands.

The most important information about the mpc-specific files can be found at the bottom of the page.

# Dependencies

- Ubuntu 22.04 jammy jellyfisch
- Ros2 humble hawskbill installation guide https://docs.ros.org/en/humble/Installation.html
- Configuring environment https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html
- Python version 3.10.12
- Pip version 22.0.2
- Setuptools-75.6.0 
  - setuptool versin downgrade: python3 -m pip install setuptools==58.0.4
- Wheel 0.45.1
- Numpy 2.2.0
  has to be lower than 2: pip install numpy<2
- Pandas 2.2.3
- pynput
  pip install pynput
- pyserial 3.5
  pip install pyserial
- ultralytics
  pip install ultralytics
- Foxglove
  https://foxglove.dev/download
  sudo apt install ros-$ROS_DISTRO-foxglove-bridge
- Pangolin: 
  sudo apt update
  sudo apt install cmake g++ libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols
  sudo apt install libepoxy-dev
  cd ~/Pangolin/build
  rm -rf *
  cmake ..
  make -j$(nproc)
  sudo make install
-  xarco
  sudo apt update
  sudo apt install ros-humble-xacro
- permission to user to acess port
  sudo usermod -a -G dialout $USER
  check via: groups
  check permission: ls -l /dev/ttyACM0

- Nav2 msgs:
  - sudo apt-get install ros-humble-nav2-msgs
- Rosdep
  - colcon build --symlink-install --install-base ./install


# Usage

### First time installing
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
rosdep install --from-paths src --ignore-src -r -y

### Auto source
nano ~/.bashrc
source Bachelorthesis_MPC_Valentin_Simonis/robot/aki/install/setup.bash

### Build and source
cd Bachelorthesis_MPC_Valentin_Simonis/robot/aki
colcon build
source install/setup.bash

## Launching the Autonomous Mission (using PID)

### Launch Robot(Yolo, Slam, etc)
cd Bachelorthesis_MPC_Valentin_Simonis/robot/aki/
ros2 launch robot_launch.py

### Launch state machine
cd Bachelorthesis_MPC_Valentin_Simonis/robot/aki/
ros2 run nav_tools statemachine

## Ground truth collection and evaluation

### Launching the ground truth collection process
cd Bachelorthesis_MPC_Valentin_Simonis/robot/aki/
ros2 launch modeling_launch.py

### Processing the ground truth data
cd Bachelorthesis_MPC_Valentin_Simonis/robot/aki/
- ros2 run modeling_tools multiple_inverse
- ros2 run modeling_tools robotpos
- ros2 run modeling_tools savetocsv

## Running the MPC

### Launching the MPC helper files
cd Bachelorthesis_MPC_Valentin_Simonis/robot/aki/
ros2 launch modeling_launch.py

### Running the MPC
cd Bachelorthesis_MPC_Valentin_Simonis/robot/aki/
ros2 run nav_slam mpc_tvp
### The MPC is implemented in mpc_tvp.py and mpc_config_tvp.py at Bachelorthesis_MPC_Valentin_Simonis/robot/aki/src/nav_slam/nav_slam

## Switch between vertical and general MPC
file location: Bachelorthesis_MPC_Valentin_Simonis/robot/aki/src/nav_slam/nav_slam/mpc_config_tvp.py
- get_mpc() function sets up the general MPC
- get_mpc_bcuonly() function sets up the vertical MPC

file location: Bachelorthesis_MPC_Valentin_Simonis/robot/aki/src/nav_slam/nav_slam/mpc_tvp.py
- Comment line 107 and uncomment line 109 for vertical MPC
- reverse for general MPC
- change line 181: use t=0.2 for general MPC and t=0.5 for vertical MPC 




