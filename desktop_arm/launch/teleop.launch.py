import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
import xacro

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("fake_hardware", default_value="false", description="Launch RViz?"))
    declared_arguments.append(DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?"))

    #Initialize arguments
    enable_abb_hardware = LaunchConfiguration("fake_hardware")    
    launch_rviz = LaunchConfiguration("launch_rviz")
     
    # Get parameters for the Servo node
    servo_yaml = load_yaml("desktop_arm", "config/desktop-arm_simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Get URDF and SRDF
    robot_description_semantic_config = load_file("desktop_arm", "config/desktop-arm.srdf")
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }
    robot_description_config = xacro.process_file( os.path.join( 
        get_package_share_directory("desktop_arm"), "config", "desktop-arm.urdf.xacro") 
    )
    robot_description = {"robot_description": robot_description_config.toxml()}
    
    # RViz
    rviz_config_file = get_package_share_directory('desktop_arm') + "/rviz/rviz_sim.rviz"
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description, robot_description_semantic],
    )

    # ros2_control
    ros2_controllers_path = os.path.join(
        get_package_share_directory("desktop_arm"),
        "config",
        "desktop-arm_ros_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # Load controllers
    load_controllers = []
    for controller in ["desktop_arm_controller", "joint_state_broadcaster"]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]
        
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )
    
    # Publishes tf's for the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
                        
    # Launch as much as possible in components
    container = ComposableNodeContainer(
            name='moveit_servo_container',
            namespace='/',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='moveit_servo',
                    plugin='moveit_servo::ServoServer',
                    name='servo_server',
                    parameters=[servo_params, robot_description, robot_description_semantic],
                    extra_arguments=[{'use_intra_process_comms' : True}]),
            ],
            output='screen',
    )
    
    spacenav_node = Node(package='spacenav',
        name="spacenav_node", 
        executable='spacenav_node' , 
        output='screen',
    )
    
    joy_node = Node(package='joy',
        name='joy_node',
        executable='joy_node',
        output='screen',
    )
    
    joints_yaml = os.path.join(
        get_package_share_directory("desktop_arm_description"),
        "config", 
        "joint_limits.yaml"
    )
    desktop_arm_node = Node(
        package='desktop_arm',
        executable='robot_controller.py',
        name='desktop_arm_controller_node',
        parameters=[
                    {'base_frame_ID':'base_link',
                    'eef_frame_ID':'end_effector',
                    'controller_type':'xbox',
                    'publishing_rate':4.0,
                     #'controller_type':'keyboard',
                     #joints_yaml
        }],
    )
    
    arduino_node = Node(
        package='desktop_arm',
        executable='arduino_driver.py',
        name='arduino_node',
    )
    
    launch_nodes = [rviz_node,
                    #ros2_control_node,
                    joy_node,
                    #spacenav_node,
                    #static_tf,
                    robot_state_publisher,
                    #container,
                    desktop_arm_node,
                    #arduino_node,
                    ] + load_controllers

                    
    #if fake_hardware is not True:
    #    launch_nodes.append([ros2_control_node, container])
    
    drift_rot_override = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2 service call '/servo_server/change_drift_dimensions' 'moveit_msgs/srv/ChangeDriftDimensions' '{drift_x_translation: False, drift_y_translation: False, drift_z_translation: False, drift_x_rotation: False, drift_y_rotation: True, drift_z_rotation: False}'"],
                shell=True,
                output="screen",
            )
        ]
    )
    #launch_nodes.append(drift_rot_override)
    return LaunchDescription(declared_arguments + launch_nodes)
