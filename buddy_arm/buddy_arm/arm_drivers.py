#!/usr/bin/env python3

# ROS libraries
import rclpy
from sensor_msgs.msg import Joy
from control_msgs.msg import JointLog
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped


controllerTypes = {
    1: "keyboard",
    2: "xbox",
    }
    
controllerTopics = {
    "keyboard":"cmd_vel",
    "xbox":"/joy",
    "spacemouse":"/spacenav/joy"
    }
    
class Keyboard:
    def __init__(self, parent_node = None):
        """Keyboard object with controller settings
           param: parent_node - (Node) ros2 node inherited from robot_controller.py
        """
        self.parent_node = parent_node
        self.controller = {"type": "keyboard", "twist_msg": TwistStamped(), "joint_msg": JointLog(), "sub_topic": "/cmd_vel" } 
        if self.parent_node is not None:
            self.controller['subscriber'] = self.create_subscription(Twist, self.controller['sub_topic'], self.keyboardCB, 1)
           
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
        """ This converts a keyboard input to a TwistStamped message
        param: msg - (Twist) ROS2 message data type        
        return  true if you want to publish a Twist, false if you want to publish a JointJog
        """
        # map buttons to twist commands (keyboard only uses twist right now)
        self.controller['twist_msg'].linear.z = msg.linear.z
        self.controller['twist_msg'].linear.y = msg.linear.y
        self.controller['twist_msg'].linear.x = msg.linear.x

        self.controller['twist_msg'].angular.y = msg.angular.y
        self.controller['twist_msg'].angular.x = msg.angular.x
        self.controller['twist_msg'].angular.z = msg.angular.z

        return True


    def updateCmdFrame(frame_to_publish, msg):
        """checks whether any buttons are being pressed that should move specific 
        joints instead of the end effector.
        
        param: frame_to_publish - (string) moveit frame to publish to
        param: msg - (Twist) specific ros message type for keyboard
        
        """
        # TO-DO: 
        pass

"""
class Controller:
    def __init__(self, parent=None):
        #The controller class that has all the methods and properties 
        #a new controller should have!
        
        
        
        self.controller = {"type": None, 
                           "pub_topic": None, 
                           "publisher": None, 
                           "sub_topic": None, 
                           "subscriber": None }
        self.parent = parent
        
    def add_controller(controller_choice="none"):
        #Add a new controller to controller group. 
       # 
        #   param: controller_choice - (string) default "none" will set default to keyboard 
       # 
        
        # See if ROS2 controller type from launch file has been set or override set
        if controller_choice is "none":
            # default controller is using keyboard
            self.controller['type'] = "keyboard"
            self.controller['msg'] = Twist
            self.controller['pub_topic'] = "/cmd_vel"
        else:
            self.controller.type = controller_choice
            self.controller.pub_topic = controllerTopics[controller_choice]
        
        self.controller['subscriber'] = self.parent.create_subscription(self.controller[')
                  joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>("/joy", ROS_QUEUE_SIZE, std::bind(&JoyToServo::joyCB, this, std::placeholders::_1));
            
            
            
"""       
            
            
            
