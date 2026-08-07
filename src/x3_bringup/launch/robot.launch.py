from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node 
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
import os

# launch description:
def generate_launch_description():
    # define paths:
    description_path = get_package_share_directory("x3_description")
    bringup_path     = get_package_share_directory("x3_bringup")
    bt_path          = get_package_share_directory("x3_bt_handler")
    model_path       = os.path.join(description_path, "urdf", "x3.urdf.xacro")
    odom_path        = os.path.join(bringup_path, "launch", "odom.launch.py")
    bt_launch_path   = os.path.join(bt_path, "launch", "bt_launch.py") 

    # define arguments:
    agent_name = LaunchConfiguration("agent_name")
    agent_name_arg = DeclareLaunchArgument(
        "agent_name",
        default_value = "agent1",
        description   = "Namespace for the robot"
    )

    agent_type = LaunchConfiguration("agent_type")
    agent_type_arg = DeclareLaunchArgument(
        "agent_type",
        default_value = "typeA",
        description   = "Type of agent to be launched"
    )

    model = LaunchConfiguration("model")
    model_arg = DeclareLaunchArgument(
        "model",
        default_value = model_path,
        description   = "Absolute path to robot URDF/xacro file"
    )

    agent_initial_x = LaunchConfiguration("agent_initial_x")
    agent_initial_x_arg = DeclareLaunchArgument(
        "agent_initial_x",
        default_value = "0.0",
        description   = "Initial global x position of the agent to be launched"
    )

    agent_initial_y = LaunchConfiguration("agent_initial_y")
    agent_initial_y_arg = DeclareLaunchArgument(
        "agent_initial_y",
        default_value = "0.0",
        description   = "Initial global y position of the agent to be launched"
    )

    agent_initial_yaw = LaunchConfiguration("agent_initial_yaw")
    agent_initial_yaw_arg = DeclareLaunchArgument(
        "agent_initial_yaw",
        default_value = "0.0",
        description   = "Initial global yaw of the agent to be launched"
    )

    num_agents = LaunchConfiguration("num_agents")
    num_agents_arg = DeclareLaunchArgument(
        "num_agents",
        default_value = "2",
        description   = "Number of agents in the MRS"
    )

    drl_model = LaunchConfiguration("drl_model")
    drl_model_arg = DeclareLaunchArgument(
        "drl_model",
        default_value = "SAC_099",
        description   = "DRL model to be used for the navigational policy"
    )

    # form the robot description:
    robot_description = ParameterValue(
        Command(["xacro ", model, " agent_name:=", agent_name, " agent_type:=", agent_type]),
        value_type = str
    )

    # set frames:
    lidar_frame = PythonExpression(["'", agent_name, "' + '_lidar_link'"])
    prefix      = PythonExpression(["'", agent_name, "' + '_'"])
    imu_frame   = PythonExpression(["'", agent_name, "' + '_imu_link'"])

    # robot state publisher node:
    robot_state_publisher_node = Node(
        package     = "robot_state_publisher",
        executable  = "robot_state_publisher",
        namespace   = agent_name,
        parameters  = [{"robot_description" : robot_description}]
    )

    # lidar node:
    lidar_node = Node(
        package     = "rplidar_ros",
        executable  = "rplidar_node",
        name        = "rplidar_node",
        namespace   = agent_name,
        output      = "screen",
        parameters  = [{
            "channel_type"     : "serial",
            "serial_port"      : "/dev/rplidar",
            "serial_baudrate"  : 1000000,
            "frame_id"         : lidar_frame,
            "inverted"         : False,
            "flip_x_axis"      : True,
            "angle_compensate" : True,
            "scan_mode"        : "Standard"
        }]
    )

    # odometry nodes:
    odom_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([odom_path]), 
        launch_arguments = {"agent_name" : agent_name}.items()
    )

    # low level driver node for IMU, wheel encoders + motors:
    driver_node = Node(
        package     = "x3_bringup",
        executable  = "mcnamu_driver",
        namespace   = agent_name,
        output      = "screen",
        parameters  = [{
            "Prefix"   : prefix,
            "imu_link" : imu_frame
        }]
    )

    # BT node:
    bt_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bt_launch_path]), 
        launch_arguments = {
            "agent_name"         : agent_name,
            "agent_type"         : agent_type,
            "agent_initial_x"    : agent_initial_x,
            "agent_initial_y"    : agent_initial_y,
            "agent_initial_yaw"  : agent_initial_yaw,
            "num_agents"         : num_agents,
            "drl_model"          : drl_model,
            "model_path"         : model_path
        }.items()
    )

    return LaunchDescription([
        # args:
        agent_name_arg,
        agent_type_arg, 
        model_arg,
        agent_initial_x_arg,
        agent_initial_y_arg,
        agent_initial_yaw_arg,
        num_agents_arg,
        drl_model_arg,

        # nodes:
        robot_state_publisher_node,
        lidar_node, 
        odom_nodes, 
        driver_node,
        bt_node
    ])
