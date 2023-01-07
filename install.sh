#!/bin/bash

# wget -O - https://gist.githubusercontent.com/danidask/4ac330977444b6c0edcbd99a3991cb83/raw | bash

set -e

echo 'Choose version:'
echo '1 - Desktop (Recommended): ROS, RViz, demos, tutorials.'
echo '2 - Base (Bare Bones): No GUI tools.'
read VERSION
if [ "$VERSION" != "1" ] && [ "$VERSION" != "2" ]; then
  echo 'Only 1 or 2 are allowed, not '$VERSION
  exit 1
fi

# https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/
cd ~
sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt-get install -y

locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common -y
sudo add-apt-repository universe

sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update; sudo apt-get upgrade -y

if [ "$VERSION" = "1" ]; then
  sudo apt-get install -y ros-humble-desktop
else
  sudo apt-get install -y ros-humble-ros-base
fi

# Install development tools: Compilers and other tools to build ROS packages
sudo apt install ros-dev-tools -y
sudo apt install python3-colcon-common-extensions -y

# Add source to shell startup script and update rosdep
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
sudo rosdep init -y
rosdep update

sudo apt-get update -y
sudo apt dist-upgrade -y

# Install build tools and moveit2
sudo apt-get install -y ros-humble-moveit
sudo apt-get install -y ros-humble-ros2-control ros-humble-ros2-controllers

# Just using keyboard teleop and joystick teleop
# Also vcs commands are not suported in .sh files for some reason
#vcs import < Desktop-Arm/desktop_arm.repos

# Install python pip installer and any additional python packages separated by space
sudo apt install python3-pip python-is-python3 -y
pip install pyserial inputs

# Finally build workspace and any dependencies
cd ~/ros2_ws
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source ~/ros2_ws/install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# Install ssh tools, networking tools, and raspi configuration gui
sudo apt-get install -y openssh-server net-tools raspi-config
sudo apt-get dist-upgrade -y
sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt autoremove -y
