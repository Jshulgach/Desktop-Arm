[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
# Desktop-Robotic-Arm
![](https://github.com/Jshulgach/Desktop-Arm/blob/master/media/solidworks-full-arm.PNG)

The Desktop Robot Arm (better name TBD) is the complete package of a ROS2-based robot controller and the [Moveo](https://www.bcn3d.com/bcn3d-moveo-the-future-of-learning-robotic-arm/), a small-medium-sized fully 3D printable Open source 6 axis robotic arm. 

It is functionally and aesthetically similar to other amazing robot arms like the [KUKA KG Agilus](https://www.kuka.com/en-us/products/robotics-systems/industrial-robots/kr-agilus) or the [Fanuc LR Mate](https://www.fanucamerica.com/products/robots/series/lr-mate/lr-mate-200id) robotic arms in the robotics industry but is aimed for personal use, research, education and anyone interested in making their own robotic arm, at an affordable price! 

The main "selling points" of this arm that separate it from other existing DIY arms are:
+ Raspberry Pi 4 Model B - main processing unit for motion planning and listener routines
+ Arduino Mega microcontroller to interface with the robot systems
+ Runs on ROS2 (with Moveit2 for motion planning)
+ All 3D printable components (aside from nuts & bolts)
+ Less that $1,000 with the B.O.M
+ Capacity to integrate more features

# Where to start ? 
This Github repository contains Installation and Quick Start instructions for the Desktop Robot Arm project running ROS2 Humble off a Raspberry Pi 4, and contains folders for scripts, libraries, examples, and readme files. If you want to make the arm yourself, start with Hardware.

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
These instructions were tested on Ubuntu 22.04.1 LTS running on a Raspberry Pi 4 64-bit 4Gb. If starting with a brand new Raspi and a blank microSD card, start with step 1 if you need to set up hardware.

1. Download the following software packages for flashing and writing to a new microSD card
   + [SD Memory Card Formatter](https://www.sdcard.org/downloads/formatter/)
   + [Raspberry Pi Imager](https://www.raspberrypi.com/software/)

2. Setting up Raspberry Pi 4
   + Follow the Imager instructions on writing the Ubuntu server image on Raspi 4. Please select Ubuntu 22.04 LTS. DO NOT install Ubuntu 22.10 LTS as the ROS2 humble debian packages aren't supported yet.
 
3. Install packages
   + Clone the Desktop Arm Github repository onto the specific `ros2_ws` folder which we create 
   ~~~
     mkdir -p ~/ros2_ws/src
     cd ~/ros2_ws/src
     git clone https://github.com/Jshulgach/Desktop-Arm.git
     ~~~

   + Run the `install.sh` bash script, which also includes installation steps for [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Development-Setup.html), [Moveit2](https://moveit.ros.org/install-moveit2/binary/), and other dependent packages.
     ~~~
     cd Desktop-Arm
     chmod +x install.sh
     ./install.sh
     ~~~
   
## Arduino Installation:
<a name="arduino-installation"/>
An Arduino Mega is used to fit with the RAMPS 1.4 Shield. Since the shield was designed to control a 3D printer or CNC machine with various stepper motor controllers and servos (along with limit switches and sensor readings), it was a great existing product to utilize. The primary microcontroller firmware needs to be flashed onto the Arduino Mega, but it is RECOMMENDED to flash some of the test scripts first to make sure the arm works, like the dancing demo!

---

## Arduino Dance Demo
<a name="demo"/>

Flash the 'desktop-arm-dancing-demo.ino' script onto the Arduino and enable power to the arm. It should slowly run through random positions with the joints and verify that the system works.

![ ](tbd)

--- 

## XBox Teleop Demo
<a name="xbox-teleop">

![ ](tbd)
Run the teleop demo file by opening a terminal:
```
ros2 launch desktop_arm teleop.launch.py
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

This project is completely open source and free to all, but any help in terms of donations or advice is really appreciated. Thank you!



