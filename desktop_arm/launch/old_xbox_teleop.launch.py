import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import xacro


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

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("fake_hardware", default_value="true", description="Disable communicationwith microcontroller"))
    declared_arguments.append(DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"))
    
    # General arguments
    launch_rviz = LaunchConfiguration("launch_rviz")
    fake_hardware = LaunchConfiguration("fake_hardware") 
    
    
    # Get parameters for the Servo node
    servo_yaml = load_yaml("desktop_arm", "config/desktop-arm_simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Get URDF and SRDF
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("desktop_arm"),
            "config",
            "test.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    
    # Get URDF and SRDF
    robot_description_semantic_config = load_file("desktop_arm", "config/desktop-arm.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}
  
    
    # RViz
    #rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), "rviz", "view_robot.rviz"])
    rviz_config_file = get_package_share_directory('moveit_servo') + "/config/demo_rviz_config.rviz"
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
    ros2_controllers_path = os.path.join(get_package_share_directory("desktop_arm"), "config", "desktop-arm_ros_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
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
    
    # Load controllers
    load_controllers = []
    for controller in ["robot_controller", "joint_state_broadcaster"]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    spacenav_node = Node(package='spacenav',
        executable='spacenav_node', 
        name="spacenav_node", 
        output='screen',
    )

    joy_node = Node(package='joy',
        name="joy_node",
        executable="joy_node",
        output="screen",
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
            ),
            ComposableNode(
                package='desktop-arm',
                executable='arm_control',
                name='controller_to_servo_node',
                parameters=[{'base_frame_ID':'base_link',
                             'eef_frame_ID': 'eef_link'},
                ],
                extra_arguments=[{'use_intra_process_comms' : True}]
            ),
        ],
        output="screen",
    )


    launch_nodes = [rviz_node,
                    ros2_control_node,
                    static_tf,
                    robot_state_publisher,
                    container,
                    ] + load_controllers
                    
    #if not fake_hardware:
    #    launch_nodes.append(arduino_node)                
                    
    return LaunchDescription(declared_arguments, launch_nodes, load_controllers)
