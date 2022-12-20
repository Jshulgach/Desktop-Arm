#!/usr/bin/env python3
"""
Copyright 2022 Jonathan Shulgach

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

Contact: Jonathan Shulgach (jonathan@shulgach.com)

Messy collection of controller types and enums ... 

"""
# ROS libraries
import rclpy
from sensor_msgs.msg import Joy, JointState
from control_msgs.msg import JointJog
from geometry_msgs.msg import Twist, TwistStamped

controllerTypes = {
    1: "keyboard",
    2: "xbox",
    }
    
controllerTopics = {
    "keyboard":"cmd_vel",
    "xbox":"/joy",
    "spacemouse":"/spacenav/joy"
    }
   
robotStateTypes = {
    "joint_state":1,
    "joint_jog":2,
    "twist":3,
    }

STICK_TRANS_X = 0 # 0
STICK_TRANS_Y = 1 # 1
STICK_TRANS_Z = 2 # originally positive
STICK_ANGULAR_X = 3
STICK_ANGULAR_Y = 4
STICK_ANGULAR_Z = 5

XBOX_BASE_LEFT = 2
XBOX_BASE_RIGHT = 5


def convertSpacenavToCmd(self, msg, twist_msg, joint_msg):
        # The bread and butter: map buttons to twist commands
        twist_msg.twist.linear.x = msg.axes[STICK_TRANS_X]
        twist_msg.twist.linear.y = msg.axes[STICK_TRANS_Y]
        twist_msg.twist.linear.z = msg.axes[STICK_TRANS_Z]

        twist_msg.twist.angular.x = msg.axes[STICK_ANGULAR_X]
        twist_msg.twist.angular.y = msg.axes[STICK_ANGULAR_Y]
        twist_msg.twist.angular.z = msg.axes[STICK_ANGULAR_Z]

        return True, twist_msg, joint_msg


class KeyboardController:
    def __init__(self, parent=None, operation_type=None, controller_info=None):
        #The keyboard controller class that has all the methods and properties 
        #a keyboard controller should have!
        
        #if controller_info is not None:
            #self.info = {"pub_type": controller_info.pub_type, 
            #             "pub_topic": controller_info.pub_topic, 
            #             "sub_type": controller_info.sub_type, 
            #             "sub_topic": controller_info.sub_topic
            #}
            
        self.info = {"pub_type": JointState, "pub_topic": "/joint_states", "sub_type": Twist, "sub_topic": "/cmd_vel"}
        self.parent = parent # In case parameters need to be shared between the main driver and the user controller
        self.use_twist = False
        self.twist_msg = None
        self.joint_msg = None

        if operation_type==robotStateTypes["twist"]:
            # Twist or cartesian commands not supported yet
            if (self.parent is not None):
                self.parent.logger("Twist commands not suppoted yet for this controller")
        
        elif operation_type==robotStateTypes["joint_jog"]:
            # Joint Jogging not supported yet
            if (self.parent is not None):
                self.parent.logger("Joint Jogging not supported yet for this controller")
        
        elif operation_type==robotStateTypes["joint_state"]:
            # Publish directly to "/joint_state" topic
            self.parent.logger("Creating JointState type for controller")
            
            if (self.parent is not None):
                self.sub = self.parent.create_subscription( self.info["sub_type"], self.info["sub_topic"], self.sub_CB, 10 )
                self.pub = self.parent.create_publisher( self.info["pub_type"], self.info["pub_topic"], 10 )
                
        else:
            if self.parent:
                self.parent.logger("Invalid operation type passed. User controller failed set up.")
        self.parent.logger("Keyboard controller")


    def sub_CB(self, msg):
        # Convert the joystick message to Twist or JointJog (or JointState!) and publish
        twist_msg, joint_msg = self.create_ros_msg()
        self.use_twist, self.twist_msg, self.joint_msg = self.convertKeyboardToCmd(msg, twist_msg, joint_msg, self.parent.robot_state_type)
        
        
    def create_ros_msg(self):
        twist_msg = TwistStamped()
        if self.parent.robot_state_type == robotStateTypes["joint_state"]:
            joint_msg = JointState()
        else:
            joint_msg = JointJog()
        return twist_msg, joint_msg
                

    def convertKeyboardToCmd(self, msg, twist_msg, joint_msg, operation_type=robotStateTypes["joint_state"]):
        self.parent.logger("convert command")
        # The bread and butter: map buttons to twist commands
        use_twist = False
        
        if operation_type==robotStateTypes["twist"]:
            # not supported yet
            # map buttons to twist commands (keyboard only uses twist right now)
            self.twist_msg.linear.z = msg.linear.z
            self.twist_msg.linear.y = msg.linear.y
            self.twist_msg.linear.x = msg.linear.x

            self.twist_msg.angular.y = msg.angular.y
            self.twist_msg.angular.x = msg.angular.x
            self.twist_msg.angular.z = msg.angular.z
            self.parent.logger("twist not supported yet")
            
        elif operation_type==robotStateTypes["joint_jog"]:
            # Making this a JointJog message type
            # not supported yet
            self.parent.logger("joint jog not supported yet")
            
        elif operation_type==robotStateTypes["joint_state"]:
            # Making this a JointState message type. It takes the controller inputs and converts them to new joint positions based on the previous positions 
          try:
            joint_msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
            
            # move up to 1 degree every step             
            #joint_1 = self.parent.joint_step*msg.linear.x # I think this is left joystick
            #if joint_1 > self.parent.joint_1_step_limit:
            #    joint_1 = self.parent.joint_1_step_limit

            joint_msg.position = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
 
          except:
            self.parent.logger("something went wrong")


        return use_twist, twist_msg, joint_msg

    def update(self):
    
        #if self.current_joint_state == self.previous_joint_state:
        #    # Don't publish old data, just skip
        #    return
        
        if self.use_twist:
            # publish the TwistStamped
            if self.twist_msg is not None:
                self.twist_msg.header.frame_id = self.parent.frame_to_publish
                self.twist_msg.header.stamp = self.parent.get_clock().now().to_msg()
                self.parent.twist_pub.publish(self.twist_msg)
        else:
            # publish the JointState message
            if self.joint_msg is not None:
                self.parent.logger("publishing")
                self.joint_msg.header.frame_id = self.parent.frame_to_publish
                self.joint_msg.header.stamp = self.parent.get_clock().now().to_msg()
                self.parent.joint_pub.publish(self.joint_msg)

        return self.joint_msg.position, self.joint_msg.position
        

class XboxController:
    def __init__(self, parent=None, operation_type=None, controller_info=None):
        #The xbox controller class that has all the methods and properties 
        #a new controller should have!
        
        #if controller_info is not None:
            #self.info = {"pub_type": controller_info.pub_type, 
            #             "pub_topic": controller_info.pub_topic, 
            #             "sub_type": controller_info.sub_type, 
            #             "sub_topic": controller_info.sub_topic
            #}
        
        self.info = {"pub_type": JointState, "pub_topic": "/joint_states", "sub_type": Joy, "sub_topic": "/joy"}
        self.parent = parent # In case parameters need to be shared between the main driver and the user controller
        self.use_twist = False
        self.twist_msg = None
        self.joint_msg = None
        
        if operation_type==robotStateTypes["twist"]:
            # Twist or cartesian commands not supported yet
            self.twist_msg = TwistStamped()
            if (self.parent is not None):
                self.parent.logger("Twist commands not suppoted yet for this controller")
        
        elif operation_type==robotStateTypes["joint_jog"]:
            # TJoint Jogging not supported yet
            self.joint_msg = JointJog()
            if (self.parent is not None):
                self.parent.logger("Joint Jogging not supported yet for this controller")
        
        elif operation_type==robotStateTypes["joint_state"]:
            # Publish directly to "/joint_state" topic
            self.joint_msg = JointState()
            self.parent.logger("Creating JointState type for controller")
            
            if (self.parent is not None):
                self.sub = self.parent.create_subscription( self.info["sub_type"], self.info["sub_topic"], self.sub_CB, 10)
                self.pub = self.parent.create_publisher( self.info["pub_type"], self.info["pub_topic"], 10)
                
        else:
            if self.parent:
                self.parent.logger("Invalid operation type passed. User controller failed set up.")
                
        
    def sub_CB(self, msg):
        # Convert the joystick message to Twist or JointJog (or JointState!) and publish
        self.use_twist, self.twist_msg, self.joint_msg = self.convertXboxToCmd(msg, self.twist_msg, self.joint_msg, self.parent.robot_state_type)
        
        
    def convertXboxToCmd(self, msg, twist_msg, joint_msg, operation_type=robotStateTypes["joint_state"]):
        # The bread and butter: map buttons to twist commands
        use_twist = False
        if operation_type==robotStateTypes["twist"]:
            use_twist = True
            twist_msg.twist.linear.x = msg.axes[STICK_TRANS_X]
            twist_msg.twist.linear.y = msg.axes[STICK_TRANS_Y]
            twist_msg.twist.linear.z = msg.axes[STICK_TRANS_Z]
            twist_msg.twist.angular.x = msg.axes[STICK_ANGULAR_X]
            twist_msg.twist.angular.y = msg.axes[STICK_ANGULAR_Y]
            twist_msg.twist.angular.z = msg.axes[STICK_ANGULAR_Z]
            
        elif operation_type==robotStateTypes["joint_jog"]:
            # Making this a JointJog message type
            joint_msg.joint_names[0] = "joint_1"
            joint_msg.velocities[0] = msg.axes[0] # I think this is left joystick
            #elf.desired_joint_state = [0,0,0,0,0,0]
            
        elif operation_type==robotStateTypes["joint_state"]:
            # Making this a JointState message type. It takes the controller inputs and converts them to new joint positions based on the previous positions 
            names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
            joint_msg.name.append(names)           
            
            # move up to 1 degree every step 
            
            joint_1 = self.parent.joint_step*msg.axes[0] # I think this is left joystick
            #if joint_1 > self.parent.joint_1_step_limit:
            #    joint_1 = self.parent.joint_1_step_limit
            vals = [0,0,0,0,0]
            joint_msg.position.append(0.0)
            joint_msg.position.append(0.0)
            joint_msg.position.append(0.0)
            joint_msg.position.append(0.0)
            joint_msg.position.append(0.0)

            
        return use_twist, twist_msg, joint_msg
    
    def update(self):
    
        #if self.current_joint_state == self.previous_joint_state:
        #    # Don't publish old data, just skip
        #    return
        
        if self.use_twist:
            # publish the TwistStamped
            self.twist_msg.header.frame_id = self.parent.frame_to_publish
            self.twist_msg.header.stamp = self.parent.get_clock().now().to_msg()
            self.parent.twist_pub.publish(self.twist_msg)
        else:
            # publish the JointState message
            self.joint_msg.header.frame_id = self.parent.frame_to_publish
            self.joint_msg.header.stamp = self.parent.get_clock().now().to_msg()
            #self.parent.joint_pub.publish(self.joint_msg)

        return [0,0,0,0,0,0], [0,0,0,0,0,0]
            
            
            
