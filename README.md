# Raspberry Pi ROS2 Desktop Robot Arm System
Installation and Quick Start instructions for the Desktop Robot Arm project running ROS2 Foxy off a Raspberry Pi 4. Contains folders for scripts, libraries, examples, and readme files


---
[Installation](#installation)  
[Demo](#demo)    
---

## Installation:
<a name="installation"/>
These instructions are dependend on running Ubuntu server on a Raspberry Pi 4. If starting with a brand new Raspi 4 and a blank microSD card, start with step 0 if you need to set up hardware.

---

0. Setting up Raspberry Pi 4
   + Install Ubuntu server image on Raspi 4 following these [general instructions](https://itsfoss.com/install-ubuntu-server-raspberry-pi/)
 
1. Install [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html)
   + Set locale
     ~~~
     locale  # check for UTF-4
     sudo apt update && sudo apt install locales
     sudo locale-gen en_US en_US.UTF-8
     sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
     export LANG=en_US.UTF-8
     ~~~
   + Set up ROS repositories
     ~~~
     sudo apt update && sudo apt install curl gnupg2 lsb-release
     sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
     ~~~
   + Install ros2 foxy packages (ROS2 nodes are currently being developed and tested with the desktop version of Foxy... later versions will just need base version)
     ~~~
     sudo apt-get update; sudo apt install ros-foxy-desktop
     source /opt/ros/foxy/setup.bash
     echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
     ~~~
2. Install [Moveit2](https://moveit.ros.org/install-moveit2/binary/)
   + Install core moveit2 resources for ros2 foxy
     ~~~
     sudo apt install ros-foxy-moveit
     ~~~
   + Update all ros packages and install colcon building tools
     ~~~
     sudo apt install python3-rosdep2
     sudo rosdep init; rosdep update; sudo apt update; sudo apt dist-upgrade
     sudo apt install python3-colcon-common-extensions python3-vcstool
     ~~~
   + Create colcon workspace and get moveit2 tutorial resources
     ~~~
     mkdir -p ~/ws_moveit2/src
     cd ~/ws_moveit2/src
     git clone https://github.com/ros-planning/moveit2_tutorials.git
     vcs import < moveit2_tutorials/moveit2_tutorials.repos
     ~~~
   + Build and source workspace (set MAKEFLAGS to use one processor to avoid out-of-memory issues when building, pick release version, build packages one at a time)
     ~~~
     rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
     cd ~/ws_moveit2
     MAKEFLAGS="-j1 -l1" colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --executor sequential
     source ~/ws_moveit2/install/setup.bash
     echo 'source ~/ws_moveit2/install/setup.bash' >> ~/.bashrc
     ~~~
3. Install Buddy Arm packages
 ```
cd ~/ws_moveit2/src
git clone https://github.com/Jshulgach/Desktop-Arm.git
cd ~/ws_moveit2
colcon build --symlink-install
source ./install/setup.bash
```
---
## Demo:
<a name="demo"/>

Run the teleop demo file by opening a terminal:
```
ros2 launch buddy_arm demo_teleop.launch.py
```
Since only the keyboard control is operational right now, open a second terminal and type the following:
```
ros2 run moveit2_tutorials servo_keyboard_input
```


