#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16, String
from sensor_msgs.msg import Joy
from math import atan2
import numpy as np


class RobotArmControl(object):
    def __init__(self):
        self.torso_state=90;
        self.shoulder_state=0;
        self.elbow_state=180;
        self.wristrot_state=0;
        self.wristbend_state=0;
        self.claw_state=90;
        self.chat_pub = rospy.Publisher("chatter", String, queue_size=1000)
        self.torso_pub = rospy.Publisher("torso_servo", UInt16, queue_size=10)
        self.shoulder_pub = rospy.Publisher("shoulder_servo", UInt16, queue_size=10)
        self.elbow_pub = rospy.Publisher("elbow_servo", UInt16, queue_size=10)
        self.wristrot_pub = rospy.Publisher("wristrot_servo", UInt16, queue_size=10)
        self.wristbend_pub = rospy.Publisher("wristbend_servo", UInt16, queue_size=10)
        self.claw_pub = rospy.Publisher("claw_servo", UInt16, queue_size=10)


    def joy_callback(self, msg):
        #self.move_torso(msg)
        self.move_claw(msg)
        #angle = UInt16();
        #self.wrist_x = msg.axes[0];
        #self.wrist_y = msg.axes[1];


    def move_claw(self, msg):
        new_pos = UInt16();
        trig = msg.axes[5]
        if trig > 0:
            self.claw_pub.publish(180)
        else:
            self.claw_pub.publish(30)

    def move_torso(self, msg):
        # take current value and add joystick change if any to updatenew servo position
        new_pos = UInt16()
        torso_joy = msg.axes[3];
        old_pos = self.torso_state;
        new_pos = round(old_pos + torso_joy)
        self.torso_state = new_pos
        self.torso_pub.publish(new_pos)
        #new_pos = round(atan2(-self.wrist_y,self.wrist_x)*180/(3.14159));
        mesg = str("Torso Angle="+str(new_pos))
        #self._servo_pub.publish(angle);
        self.chat_pub.publish(mesg);



    def joy_to_angle(y1,y2,x1,x2):
        #y=y1+(x-x1)*(y2-y1)/(x2-x1)
        return 0


def main():
    rospy.init_node('desktop_teleop_arm')
    r = rospy.Rate(10)
    robotarm = RobotArmControl()
    joy_sub = rospy.Subscriber("/joy",Joy, robotarm.joy_callback)
    while not rospy.is_shutdown():
        rospy.spin()
        r.sleep()

if __name__ == '__main__':
    main()
