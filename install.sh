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
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
# locale  # verify settings
sudo apt-get install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/nullsudo apt-get update
sudo apt-get upgrade -y'

if [ "$VERSION" = "1" ]; then
  sudo apt-get install -y ros-foxy-desktop
else
  sudo apt-get install -y ros-foxy-ros-base
fi

# Add source and update rosdep
source /opt/ros/foxy/setup.bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

<<<<<<< HEAD
sudo apt-get install -y cmake build-essential python3-pip python3-colcon-common-extensions python3-argcomplete python3-rosdep python3-setuptools
=======
sudo apt-get install -y \
    cmake \
    build-essential \ 
    python3-pip \ 
    python3-colcon-common-extensions \
    python3-argcomplete \
    python3-rosdep
     \
    python3-setuptools \

>>>>>>> ros2
sudo rosdep init
rosdep update

# Add colcon_cd to your shell startup script
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/ros2_install
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc

sudo apt-get update
sudo apt dist-upgrade -y

# Install build tools and moveit2


sudo apt-get install -y ros-foxy-moveit ros-foxy-ros2-control ros-foxy-ros2-controllers python3-vcstool
vcs import < Desktop-Arm/desktop_arm.repos

# Finally build workspace and any dependencies
cd ~/ros2_ws
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
olcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source ~/ros2_ws/install/setup.bash
echo "~/ros2_ws/install/setup.bash" >> ~/.bashrc
sudo apt-get update
sdo apt-get upgrade -y
