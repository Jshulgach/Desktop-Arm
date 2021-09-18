import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
import xacro


def select_controller(control_type):
  try:
    if control_type=="keyboard":
        return Node(package="moveit2_tutorials", executable="servo_keyboard_input")
    if control_type=="joy":
        return Node(package="joy", executable="joy_node" )
    if control_type=="spacemouse":
        return Node(package="spacenav", executable="spacenav_node" )
  except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Get parameters for the Servo node
    servo_yaml = load_yaml("moveit_servo", "config/panda_simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Get URDF and SRDF with xacro
    robot_description_config = xacro.process_file(
        os.path.join(get_package_share_directory("moveit_resources_panda_moveit_config"),"config","panda.urdf.xacro" )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file( "moveit_resources_panda_moveit_config", "config/panda.srdf" )
    robot_description_semantic = { "robot_description_semantic": robot_description_semantic_config }
    
    robot_controllers = os.path.join( get_package_share_directory("moveit_resources_panda_moveit_config"), "config", "panda_ros_controllers.yaml" )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # RViz
    rviz_config_file = ( get_package_share_directory("moveit2_tutorials") + "/config/demo_rviz_config.rviz" )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description, robot_description_semantic],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    # A node to publish world -> panda_link0 transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
        parameters=[ {'/child_frame_id' : 'panda_link0', '/frame_id' : 'world'} ],
    )

    # ==================== Set up possible controller type =============================
    user_controller_node = select_controller("keyboard")
    # ===================================================================================
    
    # Create special buddy arm servo controller node
    arm_driver = Node(
        package="buddy_arm",
        executable="robot_controller.py",
        name="arm_controller_driver",
        #parameters[ {'controller_type':'keyboard'} ],
    )


    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoServer",
                name="servo_server",
                parameters=[
                    servo_params,
                    robot_description,
                    robot_description_semantic,
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
            #ComposableNode(
            #    package="moveit_servo",
            #    plugin="moveit_servo::JoyToServoPub",
            #    name="controller_to_servo_node",
            #    extra_arguments=[{"use_intra_process_comms": True}],
            #)
        ],
        output="screen",
    )

    node_list = [
        ros2_control_node,
        robot_state_pub_node,
        #rviz_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        static_tf,
        container,
        #user_controller_node, #doesn't seem to collect inputs when running from launch file
        arm_driver
    ]
    return LaunchDescription(node_list)

