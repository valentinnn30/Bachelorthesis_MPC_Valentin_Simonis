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


- Large File Storage:
  - sudo apt install git-lfs
  - Go into folder with src-> git lfs install -> git lfs pull
- Gazebo Harmonic:
  - wget https://gazebosim.org/repos/gz-archive-keyring.gpg -O - | sudo apt-key add –
  - sudo sh -c 'echo "deb [arch=amd64] http://packages.osrfoundation.org/gz/gz-ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
  - sudo apt update
  - sudo apt install gz-harmonic
  - sudo apt-get install ros-humble-ros-gzharmonic

- Nav2 msgs:
  - sudo apt-get install ros-humble-nav2-msgs
- Rosdep
  - colcon build --symlink-install --install-base ./install
- Robot localization:
  -


