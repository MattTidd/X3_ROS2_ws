# X3_ROS2_ws:
Practical implementation of a developed methodology for performing task allocation and execution on heterogeneously composed multi-robot systems (MRS) using Deep Reinforcement Learning (DRL) and Behaviour Trees (BTs).

## Usage:
To launch a given agent within the MRS, the following can be run in the command line:

``ros2 launch x3_bringup robot.launch.py``

Where the following arguments are accepted:

- ``agent_name``: Namespace for the agent. Used to prefix all joints, links, topics, and nodes.
- ``agent_type``: Type of agent to be launched. Defines the capabilities of the agent, and is used in auctioning.
- ``model``: Absolute path to the URDF of the agent.
- ``agent_initial_x``: Initial global x position of the agent to be launched.
- ``agent_initial_y``: Initial global y position of the agent to be launched.
- ``agent_initial_yaw``: Initial global yaw of the agent to be launched.
- ``num_agents``: Number of agents in the MRS.
- ``drl_model``: Name of the DRL model to be used for the navigational policy.

**For launching a single agent, an example usage would then be:**

```bash
ros2 launch x3_bringup robot.launch.py agent_name:=agent1 agent_type:=typeA agent_initial_x:=0.0 agent_initial_y:=0.0 agent_initial_yaw:=0.0 num_agents:=1 drl_model:=SAC_099
```

To interact with the MRS, the following can be run in the command line:

``ros2 run x3_gui_interface x3_gui_interface``

Where the following argument is accepted:

- ``num_agents``: Number of agents in the MRS.

**For use with a single agent, an example usage would then be:**

```bash
ros2 run x3_gui_interface x3_gui_interface --ros-args -p num_agents:=1
```

To log agent odometry, the following can be run in the command line:

``ros2 run x3_path_logger path_logger_node``

Which accepts the following arguments:

- ``num_agents``: Number of agents in the MRS.
- ``goal_tolerance``: Minimum difference between current agent position and goal position for completion.
- ``save_dir``: Directory for saving the odometry of the agents.

**For use with a single agent, an example usage would then be:**

```bash
ros2 run x3_path_logger path_logger_node --ros-args -p num_agents:=1 -p goal_tolerance:=0.2
```

## Project Structure:
Currently, the project structure is as follows:
```txt
├── 📂 scripts/: directory for complementary scripts used to analyze the developed methodology.
│   └── 📄 visualize_path.py: script used for visualizing the path each agent has taken during 
│   │       a mission.
├── 📂 src/: directory containing the ROS2 packages for the developed methodology.
│   ├── 📂 rf2o_laser_odometry/: Implementation of an odometric planar laser scan matcher 
│   │      available here: https://github.com/MAPIRlab/rf2o_laser_odometry.
│   │
│   ├── 📂 rplidar_ros/: SLAMTEC LiDAR ROS2 packages, available here:
│   │      https://github.com/Slamtec/rplidar_ros/.
│   │
│   ├── 📂 x3_bringup/: Main package for launching the developed methodology on real hardware.
│   │
│   ├── 📂 x3_bt_handler/: Package that hosts the main BT functionality for an agent in the MRS.
│   │
│   ├── 📂 x3_covariance_filter/: Packge for running a tunable covariance filter.
│   │
│   ├── 📂 x3_description/: Package containing the URDF of an agent within the MRS.
│   │
│   ├── 📂 x3_drl_policy/: Package containing the developed DRL formulation, implemented as a 
│   │      ROS2 node leveraging an action server-client model.
│   │
│   ├── 📂 x3_gui_interface/: Package containing the GUI used to interact with the system via the 
│   │      formation of missions, which are several sequentially ordered tasks.
│   │
│   ├── 📂 x3_nav_bringup/: Package containing the goal client used by the developed 
│   │      DRL formulation.
│   │
│   ├── 📂 x3_nav_interfaces/: Package containing the actions and messages used by the 
│   │      developed methodology.
│   │
│   ├── 📂 x3_path_logger/: Package used for logging the odometry of each agent, so it may be 
│   │      visualized using the visualization script.
│   │
│   └── 📂 x3_teleop/: Package for teleoperating an agent, for debugging purposes.
```