[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
# Desktop-Robotic-Arm
[![ ](tbd)

The Desktop Robot Arm (better name TBD) is the complete package of a ROS2-based robot controller and the [Moveo](https://www.bcn3d.com/bcn3d-moveo-the-future-of-learning-robotic-arm/), a small-medium-sized fully 3D printable Open source 6 axis robotic arm. 

It is functionally and aesthetically similar to other amazing robot arms like the [KUKA KG Agilus](https://www.kuka.com/en-us/products/robotics-systems/industrial-robots/kr-agilus) or the [Fanuc LR Mate](https://www.fanucamerica.com/products/robots/series/lr-mate/lr-mate-200id) robotic arms in the robotics industry but is aimed for personal use, research, education and anyone interested in making their own robotic arm, at an affordable price! 

The main "selling points" of this arm that separate it from other existing DIY arms are:
+ Raspberry Pi 4 Model B - main processing unit for motion planning and listener routines
+ Arduino Mega microcontroller to interface with the robot systems
+ ROS2 Foxy (with Moveit2 for motion planning)
+ All 3D printable components (aside from nuts & bolts)
+ Less that $1,000 with the B.O.M
+ Capacity to integrate more features

# Where to start ? 
This Github repository contains Installation and Quick Start instructions for the Desktop Robot Arm project running ROS2 Foxy off a Raspberry Pi 4, and contains folders for scripts, libraries, examples, and readme files. If you want to make the arm yourself, start with Hardware.

Table of Contents
---
+ [Hardware](#hardware)    
+ [Raspberry PI 4 Installation](#rpi4-installation)    
+ [Arduino Installation](#arduino-installation)    
+ [Dance Demo](#demo)    
+ [XBox Teleop](#xbox-teleop)    
+ [Simulated Model](#simulated-model)    
+ [Voice Control - Coming Soon!](#voice-control)
+ [Home Assistant Integration - Coming Soon!](#home-assistant-integration)
+ [HTC Vive Integration - coming Soon!](#htc-vive)
---

## Hardware:
<a name="hardware"/>

1. Building the arm
   + The full list of parts needed to assemble the arm can be found in the [B.O.M](tbd).

   + Instructables has a greate [step-by-step guide](https://www.instructables.com/Build-a-Giant-3D-Printed-Robot-Arm/) on 3D printing and assembling the arm pieces. 
   
2. Electrical wiring
   + A wiring diagram for the RAMPS 1.4 shield and Arduino Mega can be found in the [Electrical](tbd) folder.
   + A reference for the RAMPS 1.4 shield can be found on their [website](https://reprap.com/wiki/RAMPS_1.4) which shows the pin assignments. Please be sure to refer only to the 1.4 version of the shield.

---

## Raspberry Pi 4 Software Installation:
<a name="rpi4-installation"/>
These instructions were tested on Ubuntu server running on a Raspberry Pi 4. If starting with a brand new Raspi and a blank microSD card, start with step 0 if you need to set up hardware.


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
3. Install Desktop Arm packages
 ```
cd ~/ws_moveit2/src
git clone https://github.com/Jshulgach/Desktop-Arm.git
cd ~/ws_moveit2
colcon build --symlink-install
source ./install/setup.bash
```
---

## Arduino Installation:
<a name="arduino-installation"/>
An Arduino Mega is used to fit with the RAMPS 1.4 Shield. Since the shield was designed to control a 3D printer or CNC machine with various stepper motor controllers and servos (along with limit switches and sensor readings), it was a great existing product to utilize. The [primary microcontroller firmware](#) needs to be flashed onto the Arduino Mega, but it is RECOMMENDED to flash some of the [test scripts](#) first to make sure the arm works, like the dancing demo!

---

## Arduino Dance Demo
<a name="demo"/>

Flash the 'desktop-arm-dancing-demo.ino' script onto the Arduino and enable power to the arm. It should slowly run through random positions with the joints and verify that the system works.

[![ ](tbd)

--- 

## XBox Teleop Demo
<a name="xbox-teleop">

[![ ](tbd)
Run the teleop demo file by opening a terminal:
```
ros2 launch buddy_arm demo_teleop.launch.py
```
Since only the keyboard control is operational right now, open a second terminal and type the following:
```
ros2 run moveit2_tutorials servo_keyboard_input
```
---

Simulated Model 
<a name="simulated-model">
---
Coming Soon!


Voice Control
<a name="voice-control">
---
Coming Soon!


Home Assistant Integration
<a name="home-assistant-integration">
---
Coming Soon!


HTC Vive Integration
<a name="htc-vive">
---
Coming Soon!

---

# Support the project

This project is completely Open source and free to all, but any help in terms of donations or advice is really appreciated. Thank you!



