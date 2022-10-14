#!/usr/bin/env python3
"""
Copyright 2022 Jonathan Shulgach

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

Contact: Jonathan Shulgach (jonathan@shulgach.com)

Messy collection of controller types, probably will remove most of this 

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
    "twist":2,
    }

STICK_TRANS_X = 0 # 0
STICK_TRANS_Y = 1 # 1
STICK_TRANS_Z = 2 # originally positive
STICK_ANGULAR_X = 3
STICK_ANGULAR_Y = 4
STICK_ANGULAR_Z = 5

XBOX_BASE_LEFT = 2
XBOX_BASE_RIGHT = 5


"""  
# In production, definitely not working yet...  
class Keyboard:
    def __init__(self, parent_node = None):
        #Keyboard object with controller settings
        #   param: parent_node - (Node) ros2 node inherited from robot_controller.py
        
        self.parent_node = parent_node
        self.controller = {"type": "keyboard", "twist_msg": TwistStamped, "joint_msg": JointJog, "sub_topic": "/cmd_vel" } 
        if self.parent_node is not None:
            self.controller['subscriber'] = self.parent_node.create_subscription(Twist, self.controller['sub_topic'], self.keyboardCB, 1)
           
    def keyboardCB(self, msg):

      # This call updates the frame for twist commands
      self.updateCmdFrame(self.parent_node.frame_to_publish_, msg)

      # Convert the keyboard message to Twist or JointJog and publish
      if self.convertKeyboardToCmd(msg):
      
          # publish the TwistStamped
          self.controller['twist_msg'].header.frame_id = self.parent_node.frame_to_publish
          self.controller['twist_msg'].header.stamp = self.parent_node.now()
          self.parent_node.twist_pub.publish(self.controller['twist_msg'])


    def convertKeyboardToCmd(msg):
        #This converts a keyboard input to a TwistStamped message
        #param: msg - (Twist) ROS2 message data type        
        #return  true if you want to publish a Twist, false if you want to publish a JointJog
        
        # map buttons to twist commands (keyboard only uses twist right now)
        self.controller['twist_msg'].linear.z = msg.linear.z
        self.controller['twist_msg'].linear.y = msg.linear.y
        self.controller['twist_msg'].linear.x = msg.linear.x

        self.controller['twist_msg'].angular.y = msg.angular.y
        self.controller['twist_msg'].angular.x = msg.angular.x
        self.controller['twist_msg'].angular.z = msg.angular.z

        return True

    def convertSpacenavToCmd(self, msg, twist_msg, joint_msg):
        # The bread and butter: map buttons to twist commands
        twist_msg.twist.linear.x = msg.axes[STICK_TRANS_X]
        twist_msg.twist.linear.y = msg.axes[STICK_TRANS_Y]
        twist_msg.twist.linear.z = msg.axes[STICK_TRANS_Z]

        twist_msg.twist.angular.x = msg.axes[STICK_ANGULAR_X]
        twist_msg.twist.angular.y = msg.axes[STICK_ANGULAR_Y]
        twist_msg.twist.angular.z = msg.axes[STICK_ANGULAR_Z]

        return True, twist_msg, joint_msg


    def updateCmdFrame(frame_to_publish, msg):
        #checks whether any buttons are being pressed that should move specific 
        #joints instead of the end effector.
        
        #param: frame_to_publish - (string) moveit frame to publish to
        #param: msg - (Twist) specific ros message type for keyboard
        
        
        # TO-DO: 
        pass
"""


def convertSpacenavToCmd(self, msg, twist_msg, joint_msg):
        # The bread and butter: map buttons to twist commands
        twist_msg.twist.linear.x = msg.axes[STICK_TRANS_X]
        twist_msg.twist.linear.y = msg.axes[STICK_TRANS_Y]
        twist_msg.twist.linear.z = msg.axes[STICK_TRANS_Z]

        twist_msg.twist.angular.x = msg.axes[STICK_ANGULAR_X]
        twist_msg.twist.angular.y = msg.axes[STICK_ANGULAR_Y]
        twist_msg.twist.angular.z = msg.axes[STICK_ANGULAR_Z]

        return True, twist_msg, joint_msg





class XboxController:
    def __init__(self, parent=None, operation_type=None):
        #The xbox controller class that has all the methods and properties 
        #a new controller should have!
        
        self.joint_state_controller_info = {"pub_type": JointState, "pub_topic": "/joint_states", "sub_type": Joy, "sub_topic": "/joy"}
        self.parent = parent # In case parameters need to be shared between the main driver and the user controller
        self.use_twist = False
        self.twist_msg = TwistStamped()
        self.joint_msg = JointJog()
        
        if operation_type==robotStateTypes["twist"]:
            # Twist or cartesian commands not supported yet
            if (self.parent is not None):
                self.parent.logger("Twist commands not suppoted yet for this controller")
        
        elif operation_type==robotStateTypes["joint_jog"]:
            # TJoint Jogging not supported yet
            if (self.parent is not None):
                self.parent.logger("Joint Jogging not supported yet for this controller")
        
        elif operation_type==robotStateTypes["joint_state"]:
            # Publish directly to "/joint_state" topic
            self.parent.logger("Creating JointState type for controller")
            
            if (self.parent is not None):
                self.sub = self.parent.create_subscription(self.joint_state_controller_info["sub_type"], self.joint_state_controller_info["sub_topic"], self.sub_CB, 10)
                self.pub = self.parent.create_publisher(self.joint_state_controller_info["pub_type"], self.joint_state_controller_info["pub_topic"], 10)
        else:
            if self.parent:
                self.parent.logger("Invalid operation type passed. User controller failed set up.")
                
        
    def sub_CB(self, msg):
        # Convert the joystick message to Twist or JointJog (or JointState!) and publish
        self.use_twist, self.twist_msg, self.joint_msg = self.convertXboxToCmd(msg, self.twist_msg, self.joint_msg, 2)
        
        
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
            
            joint_msg.joint_names[0] = "joint_1"
            joint_msg.joint_names[1] = "joint_2"
            joint_msg.joint_names[2] = "joint_3"
            joint_msg.joint_names[3] = "joint_4"
            joint_msg.joint_names[4] = "joint_5"
            joint_msg.joint_names[5] = "joint_6"
            
            for joint_pos in self.current_joint_state:
            
            joint_1 = self.joint_1_step*msg.axes[0] # I think this is left joystick
            if joint_1 > self.parent.joint_1_step_limit:
                joint_1 = self.parent.joint_1_step_limit
                
            joint_msg.position[0] = joint_1
            joint_msg.position[0] = 0
            joint_msg.position[0] = 0
            joint_msg.position[0] = 0
            joint_msg.position[0] = 0
            joint_msg.position[0] = 0

            
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
            
            
            
