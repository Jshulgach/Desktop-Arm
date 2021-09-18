#!/usr/bin/env python3


# ROS libraries
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import PlanningScene
from sensor_msgs.msg import Joy

# robot drivers
from buddy_arm.arm_drivers import *


class ControllerToRobot(Node):
    def __init__(self):
        """This is the object that handles controller inputs and directs them to topics understood
        by the Servo Server for twist and joint changes. 
        
        param: Node - argument passed through 
        
        """    
        super().__init__('robot_controller_node')
     
        # declare ROS2 params
        self.init_ros_params()

        # Create controller object
        self.controller = Keyboard(self)
        
        # create publishers and subscribers
        self.init_pub_sub()
        
        # Create controller object
        #self.controller = Controller()
        #self.controller.add_controller()
        
        # Start servo service client
        self.create_servo_client()
        
        
    def init_ros_params(self):
        # Declare all ros2 params
        #self.declare_parameter("controller_type", "none");
        #self.controller_choice = self.get_parameter("controller_type").value      
        self.eef_frame_ID = 'panda_link8'
        self.base_frame_ID = 'panda_link0'
        self.frame_to_publish = self.base_frame_ID
      
    def init_pub_sub(self):
        # Set up all publishers and subscribers with configured topics.
        self.twist_pub = self.create_publisher(TwistStamped, "/delta_twist_cmds", 10)
        self.joint_pub = self.create_publisher(TwistStamped, "/delta_joint_cmds", 10)
        self.collision_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

            
    def create_servo_client(self):
        # Create a service client to start the ServoServer
        self.servo_start_client = self.create_client(Trigger,"/servo_server/start_servo")
        while not self.servo_start_client.wait_for_service(timeout_sec=5.0):
           self.get_logger().info('service not available, waiting again...')
        self.servo_start_client.call_async(Trigger.Request())

    
        
        
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
        
    
