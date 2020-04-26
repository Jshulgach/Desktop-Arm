#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16, String
from sensor_msgs.msg import Joy
from math import atan2
import numpy as np
import time

class RobotArmControl(object):
    def __init__(self):
	self.msg = Joy()
        self.torso_state=90
        self.shoulder_state=45
        self.elbow_state=15
        self.wristrot_state=90
        self.wristbend_state=10
        self.claw_state=180
	self.demo_button = 0
	self.all_buttons = []
        self.chat_pub = rospy.Publisher("chatter", String, queue_size=1000)
        self.torso_pub = rospy.Publisher("torso_servo", UInt16, queue_size=10)
        self.shoulder_pub = rospy.Publisher("shoulder_servo", UInt16, queue_size=10)
        self.elbow_pub = rospy.Publisher("elbow_servo", UInt16, queue_size=10)
        self.wristrot_pub = rospy.Publisher("wristrot_servo", UInt16, queue_size=10)
        self.wristbend_pub = rospy.Publisher("wristbend_servo", UInt16, queue_size=10)
        self.claw_pub = rospy.Publisher("claw_servo", UInt16, queue_size=10)

    def move_servos(self):
	self.torso_pub.publish(self.torso_state)
	self.shoulder_pub.publish(self.shoulder_state)
	self.elbow_pub.publish(self.elbow_state)
	self.wristrot_pub.publish(self.wristrot_state)
	self.wristbend_pub.publish(self.wristbend_state)
	self.claw_pub.publish(self.claw_state)

    def joy_callback(self,msg):
	self.msg = msg
	self.update_buttons(self.msg)
        self.update_servos(self.msg)

    def run_robot(self):
        mesg = str("===================== Running Arm ======================")
        self.chat_pub.publish(mesg)
	self.chat_pub.publish(str("Demo Button="+str(self.demo_button)))
        self.move_torso()
        self.move_shoulder()
        self.move_elbow()
        self.move_wristrot()
        self.move_wristbend()
        self.move_claw()

	if self.demo_button==1:
	    self.run_demo()

    def move_torso(self):
        self.torso_pub.publish(self.torso_state)
        self.chat_pub.publish(str("Torso Angle="+str(self.torso_state)))

    def move_shoulder(self):
        self.shoulder_pub.publish(self.shoulder_state)
        self.chat_pub.publish(str("Shoulder Angle="+str(self.shoulder_state)))

    def move_elbow(self):
        self.elbow_pub.publish(self.elbow_state)
        self.chat_pub.publish(str("Elbow Angle="+str(self.elbow_state)))

    def move_wristrot(self):
        self.wristrot_pub.publish(self.wristrot_state)
        self.chat_pub.publish(str("WristRot Angle="+str(self.wristrot_state)))

    def move_wristbend(self):
	self.wristbend_pub.publish(self.wristbend_state)
        self.chat_pub.publish(str("WristBend Angle="+str(self.wristbend_state)))

    def move_claw(self):
	self.claw_pub.publish(self.claw_state)
        self.chat_pub.publish(str("Claw Angle="+str(self.claw_state)))

    def update_buttons(self, msg):
	self.demo_button = msg.buttons[0]
	self.all_buttons = msg.buttons
        #self.chat_pub.publish(str("Demo Button="+str(self.demo_button)))

    def update_servos(self, msg):
	new_pos = UInt16()
	gain = 2

	# Update Torso
        torso_joy = msg.axes[0];
        old_pos = self.torso_state;
        new_pos = round(old_pos - gain*torso_joy)
	if new_pos >=0 and new_pos <=180:
            self.torso_state = new_pos

	# Update shoulder
        shoulder_joy = msg.axes[7]
	old_pos = self.shoulder_state
        new_pos = round(old_pos + gain*shoulder_joy)
	if new_pos <= 180 and new_pos >= 0:
	    self.shoulder_state = new_pos

	# Update elbow
        elbow_joy = msg.axes[1]
        old_pos = self.elbow_state
        new_pos = round(old_pos + gain*elbow_joy)
	if new_pos <= 180 and new_pos >= 0:
	    self.elbow_state = new_pos

	# Update wristrot
        wristrot_joy = -msg.axes[3]
        old_pos = self.wristrot_state
        new_pos = round(old_pos + gain*wristrot_joy)
        if new_pos <= 180 and new_pos >= 0:
            self.wristrot_state = new_pos

	# Update wristbend
        wristbend_joy = msg.axes[4]
        old_pos = self.wristbend_state
        new_pos = round(old_pos + gain*wristbend_joy)
        if new_pos <= 180 and new_pos >= 0:
	    self.wristbend_state = new_pos

	# Update claw
        trig = self.msg.axes[5]
        if trig > 0.1:
            self.claw_state=180
        else:
            self.claw_state=30

    def run_demo(self):
        mesg = str("=================== Running Arm Demo ====================")
        self.chat_pub.publish(mesg)

	# Set up arm
	self.torso_state=90
	self.shoulder_state=100
	self.elbow_state=65
        self.wristrot_state=90
	self.move_servos()
	time.sleep(1)

        # Adjust grabber above printing bed
        self.torso_state=180
	self.wristbend_state=90
        self.claw_state=180
	self.move_servos()
	time.sleep(0.5)

	# Lower grabber to printing bed
        self.shoulder_state=140
        self.elbow_state=70
	self.move_servos()
	time.sleep(0.5)

	# Grab object
	self.claw_state=30
	self.move_servos()
	time.sleep(0.5)

	# Move grabber above trash can
        self.torso_state=100
	self.shoulder_state=100
        self.elbow_state=35
        self.wristbend_state=125
        self.move_servos()
        time.sleep(1)

	# Let go
	self.claw_state=180
	self.move_servos

def main():
    rospy.init_node('desktop_teleop_arm')
    r = rospy.Rate(10)
    robotarm = RobotArmControl()
    robotarm.move_servos()
    joy_sub = rospy.Subscriber("/joy",Joy, robotarm.joy_callback)
    while not rospy.is_shutdown():
        robotarm.run_robot()
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
