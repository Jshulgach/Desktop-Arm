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
        self.torso_pos = 90
        self.shoulder_pos = 45
        self.elbow_pos = 15
        self.wristrot_pos = 90
        self.wristbend_pos = 10
        self.claw_pos = 180
	self.torso_state = 0
	self.shoulder_state = 0
	self.elbow_state = 0
	self.wristrot_state = 0
	self.wristbend_state = 0
	self.claw_state = 0
	self.demo_button = 0
	self.all_buttons = []
	self.threshold = 0.5
	self.arduino_connected = 0
        self.joy_sub = rospy.Subscriber("/joy",Joy, self.joy_callback)
        self.chat_pub = rospy.Publisher("chatter", String, queue_size=1000)
        self.torso_pub = rospy.Publisher("torso_servo", UInt16, queue_size=10)
        self.shoulder_pub = rospy.Publisher("shoulder_servo", UInt16, queue_size=10)
        self.elbow_pub = rospy.Publisher("elbow_servo", UInt16, queue_size=10)
        self.wristrot_pub = rospy.Publisher("wristrot_servo", UInt16, queue_size=10)
        self.wristbend_pub = rospy.Publisher("wristbend_servo", UInt16, queue_size=10)
        self.claw_pub = rospy.Publisher("claw_servo", UInt16, queue_size=10)
	self.raspi_connect_pub = rospy.Publisher("raspi_connect", UInt16, queue_size=10)
	#self.arduino_connect_sub = rospy.Subscriber("arduino_connect", UInt16, self.arduino_callback)

    #def arduino_callback(self,msg):
#	if (msg.data)

    def move_servos(self):
	self.torso_pub.publish(self.torso_pos)
	self.shoulder_pub.publish(self.shoulder_pos)
	self.elbow_pub.publish(self.elbow_pos)
	self.wristrot_pub.publish(self.wristrot_pos)
	self.wristbend_pub.publish(self.wristbend_pos)
	self.claw_pub.publish(self.claw_pos)

    def joy_callback(self,msg):
	self.msg = msg
	self.update_buttons(msg)
        self.update_servos(msg)


    def update_buttons(self, msg):
	self.demo_button = msg.buttons[0]
	self.all_buttons = msg.buttons
        #self.chat_pub.publish(str("Demo Button="+str(self.demo_button)))


    def update_servos(self, msg):
	
	# Update Torso
	if (msg.axes[0] > self.threshold and self.torso_pos <=180):
	    self.torso_state = 1
	elif (msg.axes[0] < -(self.threshold) and self.torso_pos >=0):
	    self.torso_state = -1
	else:
	    self.torso_state = 0

	# Update shoulder
	if (msg.axes[7] > self.threshold and self.shoulder_pos <= 180):
	    self.shoulder_state = 1
	elif (msg.axes[7] < -(self.threshold) and self.shoulder_pos >= 0):
	    self.shoulder_state = -1
	else:
	    self.shoulder_state = 0

	# Update elbow
        if (msg.axes[1] > self.threshold and self.elbow_pos<=180):
            self.elbow_state = 1
	elif (msg.axes[1] < -(self.threshold) and self.elbow_pos>=0):
            self.elbow_state = -1
	else:
            self.elbow_state = 0
    
	# Update wristrot
	if (msg.axes[3] > self.threshold and self.wristrot_pos<=180):
	    self.wristrot_state = 1*(-1) # invert rotation direction for this servo
	elif (msg.axes[3] < -(self.threshold) and self.wristrot_pos>=0):
	    self.wristrot_state = -1*(-1)
	else:
	   self.wristrot_state = 0

	# Update wristbend
	if (msg.axes[4] > self.threshold and self.wristbend_pos<=180):
	    self.wristbend_state = 1
	elif (msg.axes[4] < -(self.threshold) and self.wristbend_pos>=0):
	    self.wristbend_state = -1
	else:
	    self.wristbend_state = 0

	# Update claw
        trig = self.msg.axes[5]
        if trig > self.threshold:
            self.claw_pos=180
        else:
            self.claw_pos=30

    def move_torso(self):
	new_pos = UInt16()
	new_pos = self.torso_pos + self.torso_state
	self.torso_pos = new_pos
        self.torso_pub.publish(self.torso_pos)
        self.chat_pub.publish(str("Torso Angle="+str(self.torso_pos)))

    def move_shoulder(self):
	new_pos = UInt16()
	new_pos = self.shoulder_pos + self.shoulder_state
	self.shoulder_pos = new_pos
        self.shoulder_pub.publish(self.shoulder_pos)
        self.chat_pub.publish(str("Shoulder Angle="+str(self.shoulder_pos)))

    def move_elbow(self):
	new_pos = UInt16()
	new_pos = self.elbow_pos + self.elbow_state
	self.elbow_pos = new_pos
        self.elbow_pub.publish(self.elbow_pos)
        self.chat_pub.publish(str("Elbow Angle="+str(self.elbow_pos)))

    def move_wristrot(self):
	new_pos = UInt16()
	new_pos = self.wristrot_pos + self.wristrot_state
	self.wristrot_pos = new_pos
        self.wristrot_pub.publish(self.wristrot_pos)
        self.chat_pub.publish(str("WristRot Angle="+str(self.wristrot_pos)))

    def move_wristbend(self):
	new_pos = UInt16()
	new_pos = self.wristbend_pos + self.wristbend_state
	self.wristbend_pos = new_pos
	self.wristbend_pub.publish(self.wristbend_pos)
        self.chat_pub.publish(str("WristBend Angle="+str(self.wristbend_pos)))

    def move_claw(self):
	self.claw_pub.publish(self.claw_pos)
        self.chat_pub.publish(str("Claw Angle="+str(self.claw_pos)))


    def run_robot(self):
	# Should constantly be updating joint positions based on teleop control states 
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


    def run_demo(self):
        mesg = str("=================== Running Arm Demo ====================")
        self.chat_pub.publish(mesg)

	# Set up arm
	self.torso_pos=90
	self.shoulder_pos=100
	self.elbow_pos=65
        self.wristrot_pos=90
	self.move_servos()
	time.sleep(1)

        # Adjust grabber above printing bed
        self.torso_pos=180
	self.wristbend_pos=90
        self.claw_pos=180
	self.move_servos()
	time.sleep(0.5)

	# Lower grabber to printing bed
        self.shoulder_pos=140
        self.elbow_pos=70
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

    # Wait for connection status from arduino
    #while robotarm.arduino_connected==0:
    #	pass

    robotarm.move_servos()

    while not rospy.is_shutdown():
        # Send connect status to arduino
        raspi_connect_msg = UInt16()
        raspi_connect_msg.data = 1;
        robotarm.raspi_connect_pub.publish(raspi_connect_msg)
        robotarm.run_robot()
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
