import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node 
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # declare launch arguments:
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

    use_gui = LaunchConfiguration("use_gui")
    use_gui_arg = DeclareLaunchArgument(
        "use_gui", 
        default_value = "true",
        description   = "Flag to use the GUI for joint state publisher"
    )

    rviz_only = LaunchConfiguration("rviz_only")
    rviz_only_arg = DeclareLaunchArgument(
        "rviz_only",
        default_value = "true",
        description   = "Set to true when running on real hardware"
    )

    model = LaunchConfiguration("model")
    model_arg = DeclareLaunchArgument(
        "model",
        default_value = os.path.join(
            get_package_share_directory("x3_description"), "urdf", "x3.urdf.xacro"),
        description = "Absolute path to robot URDF file"
    )

    # form the robot description:
    robot_description = ParameterValue(
        Command(["xacro ", model, " agent_name:=", agent_name, " agent_type:=", agent_type]),
        value_type = str
    )

    # robot state publisher node:
    robot_state_publisher = Node(
        package     = "robot_state_publisher",
        executable  = "robot_state_publisher",
        namespace   = agent_name,
        parameters  = [{"robot_description" : robot_description}],
        condition   = UnlessCondition(rviz_only)
    )

    # joint state publisher gui node:
    joint_state_publisher_gui = Node(
        package     = "joint_state_publisher_gui",
        executable  = "joint_state_publisher_gui", 
        namespace   = agent_name,
        condition   = IfCondition(PythonExpression(["'", rviz_only, "' == 'false' and '", use_gui, "' == 'true'"]))
    )

    # joint state publisher node:
    joint_state_publisher = Node(
        package     = "joint_state_publisher",
        executable  = "joint_state_publisher",
        namespace   = agent_name,
        condition   = IfCondition(PythonExpression(["'", rviz_only, "' == 'false' and '", use_gui, "' == 'false'"]))
    )

    # rviz node:
    rviz_node = Node(
        package     = "rviz2",
        executable  = "rviz2",
        name        = "rviz2",
        arguments   = ["-d", os.path.join(get_package_share_directory("x3_description"), "rviz","display.rviz")]
    )

    return LaunchDescription([
        agent_name_arg,
        agent_type_arg,
        use_gui_arg,
        rviz_only_arg,
        model_arg,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])
    