#!/usr/bin/env python3
"""
Copyright 2022 Jonathan Shulgach

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

Contact: Jonathan Shulgach (jonathan@shulgach.com)

'robot_controller.py' handles the main task of starting the moveit_servo server and handling movement commands coming in and converting them into robot movements. 

"""

# ROS libraries
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from moveit_msgs.msg import PlanningScene
from sensor_msgs.msg import Joy, JointState

# robot drivers
#from desktop_arm.arm_drivers import *


STICK_TRANS_X = 0 # 0
STICK_TRANS_Y = 1 # 1
STICK_TRANS_Z = 2 # originally positive
STICK_ANGULAR_X = 3
STICK_ANGULAR_Y = 4
STICK_ANGULAR_Z = 5



class ControllerToRobot(Node):
    def __init__(self):
        """This is the object that handles controller inputs and directs them to topics understood
        by the Servo Server for twist and joint changes. 
        
        param: Node - argument passed through 
        
        """    
        super().__init__('robot_controller_node')
        
        # Potential update, want to get current joint state
        self.current_joint_state_ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.desired_joint_state_ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                
        # declare ROS2 params
        self.init_ros_params()

        # create publishers and subscribers
        self.init_pub_sub()
        
        # Create controller object
        #self.controller = Keyboard(self) # default for now
        #self.controller = Controller()
        #self.controller.add_controller()
        
        # Start servo service client
        self.create_servo_client()
        self.logger("Servo server node created!")
        
        
    def init_ros_params(self):
        # Declare all ros2 params
        #self.declare_parameter("controller_type", "none")
        self.declare_parameter("eef_frame_ID","tool0")
        self.declare_parameter("base_frame_ID","base_link")      
        self.declare_parameter("controller_type","xbox")      
         
        self.eef_frame_ID = self.get_parameter("eef_frame_ID").value
        self.base_frame_ID = self.get_parameter("base_frame_ID").value
        self.controller_type = self.get_parameter("controller_type").value
        self.frame_to_publish = self.base_frame_ID
      
      
    def init_pub_sub(self):
        # Set up all publishers and subscribers with configured topics.
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_server/delta_twist_cmds", 10)
        self.joint_pub = self.create_publisher(JointJog, "/servo_server/delta_joint_cmds", 10)
        self.collision_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.chatter_pub = self.create_publisher(String, "chatter", 10)
        
        
        self.joy_sub = self.create_subscription(Joy, "/spacenav/joy", self.joy_CB, 10)
        
        
    def create_servo_client(self):
        # Create a service client to start the ServoServer
        self.servo_start_client = self.create_client(Trigger,"/servo_server/start_servo")
        while not self.servo_start_client.wait_for_service(timeout_sec=1.0):
           self.get_logger().info('service not available, waiting again...')
        self.servo_start_client.call_async(Trigger.Request())

    def logger(self, msg):
        # Send message to terminal
        self.get_logger().info(str(msg))
        
        
    def joy_CB(self, msg):
        # Create the messages we might publish
        twist_msg = TwistStamped()
        joint_msg = JointJog()

    
        # Convert the joystick message to Twist or JointJog and publish
        use_twist, twist_msg, joint_msg = self.convertSpacenavToCmd(msg, twist_msg, joint_msg)
        if use_twist:
            # publish the TwistStamped
            twist_msg.header.frame_id = self.frame_to_publish
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.twist_pub.publish(twist_msg)


    def convertSpacenavToCmd(self, msg, twist_msg, joint_msg):
        # The bread and butter: map buttons to twist commands
        twist_msg.twist.linear.x = msg.axes[STICK_TRANS_X]
        twist_msg.twist.linear.y = msg.axes[STICK_TRANS_Y]
        twist_msg.twist.linear.z = msg.axes[STICK_TRANS_Z]

        twist_msg.twist.angular.x = msg.axes[STICK_ANGULAR_X]
        twist_msg.twist.angular.y = msg.axes[STICK_ANGULAR_Y]
        twist_msg.twist.angular.z = msg.axes[STICK_ANGULAR_Z]

        return True, twist_msg, joint_msg


    
def main(args=None):
    rclpy.init(args=args)
    node = ControllerToRobot()
    
    while rclpy.ok():
        rclpy.spin(node)
        
    # Explicitly destroy node
    node.destroy_node()
    rclpy.shutdown()
    

if __name__=="__main__":
    main()
        
    
