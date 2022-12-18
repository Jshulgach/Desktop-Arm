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
    def __init__(self, node_name='desktop_arm_controller_node', rate=1, verbose=False):
        """ This is the object that handles controller inputs and directs them to topics understood
        by the Servo Server for twist and joint changes. 
        
        Parameters
        ----------
        node_name: str
            Node name argument passed through 
        rate : int
            update cycle rate for the 'update' command, used to handle data being sent to 
            the microcontroller
        verbose : bool
            Enable/disable debugging text output on the terminal
        
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
        self.robot_state_type = robotStateTypes["joint_state"] # Will change to parameter passed
        self.parameters = {}
                
        # declare ROS2 params, create publishers, subscribers, and timer
        self.init_ros_params()
        self.init_pub_sub()
        self.create_timer(1/self.parameters["publishing_rate"], self.timer_callback) # frequency rate of running commands
          
        # Create controller object
        # self.controller_type # assume XBOX controller
        self.user_controller = XboxController(self, self.robot_state_type)
        #self.user_controller = KeyboardController(self, self.robot_state_type)


        
        # Start servo service client
        #self.create_servo_client() # TO-DO once the cartesian planning is all figured out
        self.logger("Servo server node created!")
        
        
    def init_ros_params(self):
        """ Declare all ros2 params
        
        The 'add_parameter()' method will declare parameters if they haven't been included yet
        
        """
        self.add_parameter("base_frame_ID","base_link")
        self.add_parameter("eef_frame_ID","tool0")
        self.add_parameter("controller_type","xbox")  
        self.add_parameter("joint_step", 1.0) # in degrees
        self.add_parameter("publishing_rate", 1.0) # safe, once every second
        self.add_parameter("controller_joint_names", None)

        #self.joint_names = self.get_parameter("controller_joint_names").value
        #if self.joint_names is not None:
        #    for joint in self.joint_names:
        #        self.declare_patameter(joint+".min_lim")
        #        self.declare_patameter(joint+".max_lim")
        #else:
        #    self.logger('Warning: Joint names not found or sisue with loading. Joint state publishing disabled')
        
         
        self.eef_frame_ID = self.parameters["eef_frame_ID"]
        self.base_frame_ID = self.parameters["base_frame_ID"]
        self.controller_type = self.parameters["controller_type"]
        self.joint_step = self.parameters["joint_step"]
        self.eef_frame_ID = self.parameters["eef_frame_ID"]
        #self.base_frame_ID = self.get_parameter("base_frame_ID").value
        #self.controller_type = self.get_parameter("controller_type").value
        #self.joint_step = self.get_parameter("joint_step").value
        self.frame_to_publish = self.base_frame_ID
      
      
    def add_parameter(self, param, val):
        """ Helper function to store parameters within a dict
        """
        self.declare_parameter(param, val)
        self.parameters[param] = self.get_parameter(param).value
        self.logger("Added parameter - {}: {}".format(param, self.parameters[param]))
    
    def init_pub_sub(self):
        """ A function that creates all the publishers and subscribers for the robot state, 
            general event chatter, and twist commands
        
        
        """ 
        # Set up all publishers and subscribers with configured topics.
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_server/delta_twist_cmds", 10)
        #self.joint_pub = self.create_publisher(JointJog, "/servo_server/delta_joint_cmds", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.collision_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.chatter_pub = self.create_publisher(String, "chatter", 10)
        
        
    def create_servo_client(self):
        """ Create a service client to start the ServoServer
        
            Depends on the controller manager and plugin from moveit
        """
        self.servo_start_client = self.create_client(Trigger,"/servo_server/start_servo")
        while not self.servo_start_client.wait_for_service(timeout_sec=1.0):
           self.get_logger().info('service not available, waiting again...')
        self.servo_start_client.call_async(Trigger.Request())


    def logger(self, msg):
        """ Send message to terminal, enforce as string"""
        self.get_logger().info(str(msg))
        
        
    def timer_callback(self):
        """ The timer callback function that gets called depending on the cycling rate set by 
            'publishing_rate' 
            
        """
        if self.verbose:
            self.get_logger().info("Updating system")
        
        # The controller object should handle publishing to the right topic for robot movement
        self.current_joint_state, self.desired_joint_state = self.user_controller.update()
        #self.logger("current joint values".format(self.current_joint_state))
        
        
        

    
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
        
    
