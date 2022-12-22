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
from rclpy.parameter import Parameter
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from moveit_msgs.msg import PlanningScene

# robot drivers
from desktop_arm.arm_drivers import *


class ControllerToRobot(Node):
    def __init__(self, node_name='desktop_arm_controller_node', rate=10, verbose=False):
        """This is the object that handles controller inputs and directs them to topics understood
        by the Servo Server for twist and joint changes. 
        
        Parameters:
        -----------
        node_name : str
            Specific name for ROS2 Node
        rate : int
            Publishing rate for running update commands
        verbose : bool
            Enable/disable verbose output for debugging on the terminal 
        
        """    
        super().__init__(node_name,
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=False
                         )
        self.rate = rate
        self.verbose = verbose
        
        # Potential update, want to get current joint state
        self.current_joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.previous_joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.desired_joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #self.robot_state_type = robotStateTypes["joint_state"]
                
        # declare ROS2 params, create publishers, subscribers, and timer
        self.init_ros_params()
        self.init_pub_sub()
        self.create_timer(1/self.rate, self.timer_callback) # frequency rate of running commands
        self.robot_state_type = robotStateTypes[self.get_parameter("robot_cmd_type").value]
        self.logger("Robot message operation type: {}".format(self.get_parameter("robot_cmd_type").value))
        
        # Create controller object, assume XBOX controller        
        self.user_controller = XboxController(self, self.robot_state_type)
        #self.user_controller = KeyboardController(self, self.robot_state_type)


        
        # Start servo service client
        #self.create_servo_client()
        self.logger("Robot Controller node created!")
        
        
    def init_ros_params(self):
        """ Declare all ros2 params
        """
        self.declare_parameter("base_frame_ID","base_link")      
        self.declare_parameter("eef_frame_ID","tool0")
        self.declare_parameter("controller_type","xbox")
        self.declare_parameter("robot_cmd_type","joint_state")
        self.declare_parameter("joint_step", 1.0) # in degrees
        self.declare_parameter("publishing_rate", 1.0) # safe, once every second
        self.declare_parameter("controller_joint_names", None)
        #self.robot_state_type = robotStateTypes["joint_state"] # Will change to parameter passed

        #self.joint_names = self.get_parameter("controller_joint_names").value
        #if self.joint_names is not None:
        #    for joint in self.joint_names:
        #        self.declare_patameter(joint+".min_lim")
        #        self.declare_patameter(joint+".max_lim")
        #else:
        #    self.logger('Warning: Joint names not found or sisue with loading. Joint state publishing disabled')
        
         
        self.eef_frame_ID = self.get_parameter("eef_frame_ID").value
        self.base_frame_ID = self.get_parameter("base_frame_ID").value
        self.controller_type = self.get_parameter("controller_type").value
        self.joint_step = self.get_parameter("joint_step").value
        self.frame_to_publish = self.base_frame_ID
      
      
    def init_pub_sub(self):
        """ Set up all publishers and subscribers with configured topics.
        """
        #self.twist_pub = self.create_publisher(TwistStamped, "/servo_server/delta_twist_cmds", 10) # Once the kinematics urdf file is fixed these will be enables
        #self.joint_pub = self.create_publisher(JointJog, "/servo_server/delta_joint_cmds", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.collision_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.chatter_pub = self.create_publisher(String, "chatter", 10)
        
        
    def create_servo_client(self):
        """ Create a service client to start the ServoServer
        """
        self.servo_start_client = self.create_client(Trigger,"/servo_server/start_servo")
        while not self.servo_start_client.wait_for_service(timeout_sec=1.0):
           self.logger('service not available, waiting again...')
        self.servo_start_client.call_async(Trigger.Request())


    def logger(self, msg):
        """ Send message to terminal
        """
        self.get_logger().info(str(msg))
        
        
    def timer_callback(self):
        """ The callback function for the timer to update the system
        """
        if self.verbose:
            self.get_logger().info("Updating system")
        
        # The controller object should handle publishing to the right topic for robot movement
        self.current_joint_state, self.desired_joint_state = self.user_controller.update()
        
        
        

    
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
        
    
