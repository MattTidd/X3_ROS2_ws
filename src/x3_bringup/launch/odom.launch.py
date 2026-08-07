from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch_ros.actions import Node, PushRosNamespace
import os
import yaml

# function to generate ekf parameter files based on the template + agent name:
def generate_ekf_config(agent_name : str, output_dir : str):
    # define the desired parameters:
    ekf_params = {
        f"{agent_name}/ekf_filter_node": {
            "ros__parameters" : {
                # preamble settings:
                "frequency"                  : 30.0,
                "sensor_timeout"             : 0.1,
                "two_d_mode"                 : True,
                "transform_time_offset"      : 0.0,
                "transform_timeout"          : 0.0,
                "print_diagnostics"          : False,
                "debug"                      : False,
                "debug_out_file"             : "robot_localization_debug.txt",
                "permit_correct_publication" : False,
                "publish_acceleration"       : False,
                "publish_tf"                 : True,

                # frame settings:
                "map_frame"       : "map",
                "odom_frame"      : "odom",
                "base_link_frame" : f"{agent_name}_base_link",
                "world_frame"     : "odom",

                # sensor settings:
                "twist0"              : "vel_covariance",
                "twist0_queue_size"   : 10,
                "twist0_nodelay"      : False,
                "twist0_differential" : False,
                "twist0_relative"     : False,
                "twist0_config"       : [False, False, False,
                                         False, False, False,
                                         True,  True, False,
                                         False, False, True, 
                                         False, False, False],

                "odom0"                      : "odom_rf2o_covariance",
                "odom0_queue_size"           : 10,
                "odom0_nodelay"              : False,
                "odom0_differential"         : False,
                "odom0_relative"             : False,
                "odom0_pose_use_child_frame" : False,
                "odom0_config"               : [True, True, False,  
                                                False, False, True,
                                                False, False, False,
                                                False, False, False,
                                                False, False, False],

                "imu0"                                   : "imu/data_covariance",
                "imu0_queue_size"                        : 10,
                "imu0_differential"                      : False,
                "imu0_relative"                          : False,
                "imu0_remove_gravitational_acceleration" : True,
                "imu0_config"                            : [False, False, False,
                                                            False, False, False, 
                                                            False, False, False,
                                                            True, True, True,  
                                                            True,  True, False],

                # advanced settings:
                "use_control"           : False,
                "stamped_control"       : False,
                "control_timeout"       : 0.2,
                "control_config"        : [True, False, False, False, False, True],
                "acceleration_limits"   : [1.3, 0.0, 0.0, 0.0, 0.0, 3.4],
                "deceleration_limits"   : [1.3, 0.0, 0.0, 0.0, 0.0, 4.5],
                "acceleration_gains"    : [0.8, 0.0, 0.0, 0.0, 0.0, 0.9],
                "deceleration_gains"    : [1.0, 0.0, 0.0, 0.0, 0.0, 1.0],

                # covariances:
                "process_noise_covariance" : [5e-3,  0.0,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                              0.0,   5e-2,  0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                              0.0,   0.0,   6e-2,   0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                              0.0,   0.0,   0.0,    3e-2,   0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                              0.0,   0.0,   0.0,    0.0,    3e-2,   0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                              0.0,   0.0,   0.0,    0.0,    0.0,    6e-2,   0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                              0.0,   0.0,   0.0,    0.0,    0.0,    0.0,    2e-2,    0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                              0.0,   0.0,   0.0,    0.0,    0.0,    0.0,    0.0,     4e-2,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                              0.0,   0.0,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     1e-2,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                                              0.0,   0.0,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    1e-2,   0.0,    0.0,    0.0,    0.0,    0.0,
                                              0.0,   0.0,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.01,   0.0,    0.0,    0.0,    0.0,
                                              0.0,   0.0,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    2e-2,   0.0,    0.0,    0.0,
                                              0.0,   0.0,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    1e-2,   0.0,    0.0,
                                              0.0,   0.0,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    1e-2,   0.0,
                                              0.0,   0.0,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    1e-2],

                "initial_estimate_covariance" : [1e-9, 0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0,    0.0,    0.0,
                                                 0.0,  1e-9,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0,    0.0,    0.0,
                                                 0.0,  0.0,    1e-9,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0,    0.0,    0.0,
                                                 0.0,  0.0,    0.0,    1e-9,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0,    0.0,    0.0,
                                                 0.0,  0.0,    0.0,    0.0,    1e-9,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0,    0.0,    0.0,
                                                 0.0,  0.0,    0.0,    0.0,    0.0,    1e-9,   0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0,    0.0,    0.0,
                                                 0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    1e-9,   0.0,    0.0,    0.0,     0.0,     0.0,     0.0,    0.0,    0.0,
                                                 0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    1e-9,   0.0,    0.0,     0.0,     0.0,     0.0,    0.0,    0.0,
                                                 0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    1e-9,   0.0,     0.0,     0.0,     0.0,    0.0,    0.0,
                                                 0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    1e-9,    0.0,     0.0,     0.0,    0.0,    0.0,
                                                 0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     1e-9,    0.0,     0.0,    0.0,    0.0,
                                                 0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     1e-9,    0.0,    0.0,    0.0,
                                                 0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     1e-9,   0.0,    0.0,
                                                 0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0,    1e-9,   0.0,
                                                 0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0,    0.0,    1e-9]
            }
        }
    }

    # write these parameters to an output file:
    path = os.path.join(output_dir, f"{agent_name}_ekf_params.yaml")
    with open(path, "w") as f:
        yaml.dump(ekf_params, f)

# define an opaque function to use the agent_name:
def setup(context, *args, **kwargs):
    # get agent_name:
    agent_name = LaunchConfiguration("agent_name").perform(context)

    # get the paths:
    pkg_path = get_package_share_directory("x3_bringup")
    ekf_config_path = os.path.join(pkg_path, "config")

    # generate the file:
    generate_ekf_config(agent_name = agent_name, output_dir = ekf_config_path)

    return []

# launch description:
def generate_launch_description():
    # define paths:
    pkg_path = get_package_share_directory("x3_bringup")

    # define launch arguments:
    agent_name = LaunchConfiguration("agent_name")
    agent_name_arg = DeclareLaunchArgument(
        "agent_name",
        default_value = "agent1",
        description   = "Namespace for the robot"
    )

    # set parameters:
    ekf_path        = PathJoinSubstitution([pkg_path, "config", [agent_name, TextSubstitution(text = "_ekf_params.yaml")]])
    base_frame      = PythonExpression(["'", agent_name, "' + '_base_link'"])
    imu_frame       = PythonExpression(["'", agent_name, "' + '_imu_link'"])
    
    # laser scan matching node:
    laser_scan_matcher_node = Node(
        package = "rf2o_laser_odometry",
        executable = "rf2o_laser_odometry_node",
        name = "rf2o_laser_odometry",
        namespace = agent_name,
        output = "screen",
        parameters = [{
            "laser_scan_topic"      : "scan",
            "odom_topic"            : "odom_rf2o",
            "publish_tf"            : False,
            "base_frame_id"         : base_frame,
            "odom_frame_id"         : "odom",
            "init_pose_from_topic"  : "",
            "freq"                  : 10.0
        }]
    )

    # covariance filter node:
    covariance_filter_node = Node(
        package    = "x3_covariance_filter",
        executable = "covariance_filter_node",
        name       = "covariance_filter_node",
        parameters = [{"agent_name" : agent_name}],
        output     = "screen"
    )

    covariance_filter_node = GroupAction(
        actions = [
            PushRosNamespace(agent_name),
            covariance_filter_node
        ]
    )

    # EKF node:
    ekf_filter_node = Node(
        package = "robot_localization",
        executable = "ekf_node",
        name = "ekf_filter_node",
        namespace = agent_name,
        output = "screen",
        parameters = [ekf_path],
        remappings = [("odometry/filtered", "odom")]
    )

    return LaunchDescription([
        # args:
        agent_name_arg,

        # opaque function:
        OpaqueFunction(function = setup),

        # nodes:
        laser_scan_matcher_node,
        covariance_filter_node,
        ekf_filter_node
    ])