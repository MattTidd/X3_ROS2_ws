import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node 
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # path to config:
    # description_pkg_dir = get_package_share_directory("x3_description")
    
    # rviz node:
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # arguments=["-d", os.path.join(description_pkg_dir, "rviz", "real.rviz")]
    )

    return LaunchDescription([
        # nodes:
        rviz
    ])