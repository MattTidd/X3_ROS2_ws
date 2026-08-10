# imports:
import os
import json
import math
import rclpy
import datetime
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from x3_nav_interfaces.msg import Goal

# class for the node:
class PathLoggerNode(Node):
    # constructor for the node:
    def __init__(self):
        # inherit from parent:
        super().__init__("path_logger_node")

        # declare parameters:
        self.declare_parameter("num_agents", 2)
        self.declare_parameter("goal_tolerance", 0.2)
        self.declare_parameter("save_dir", os.path.join(os.path.expanduser("~"), "X3_ROS2_ws", "scripts", "recorded_paths"))

        # add parameters to the class:
        self.num_agents     = self.get_parameter("num_agents").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.save_dir       = self.get_parameter("save_dir").value
        self.mission_name   = f"mission_{len(os.listdir(self.save_dir))}"

        # initialize the per-agent stats:
        self.logging_active = {f"agent{i}" : False for i in range(1, self.num_agents + 1)}
        self.poses          = {f"agent{i}" : [] for i in range(1, self.num_agents + 1)}
        self.start_time     = {f"agent{i}" : None for i in range(1, self.num_agents + 1)}
        self.last_pos       = {f"agent{i}" : None for i in range(1, self.num_agents + 1)}
        self.total_dist     = {f"agent{i}" : 0.0 for i in range(1, self.num_agents + 1)}
        self.goals          = {}
        self.mission_saved  = False

        # goal subscriber:
        self.create_subscription(
            Goal, 
            "/goal",
            self._goal_callback, 
            10
        )

        # odometry subscriber:
        for i in range(1, self.num_agents + 1):
            self.create_subscription(
                Odometry,
                f"/agent{i}/odom",
                lambda msg, agent_id = i: self._odom_callback(msg, agent_id),
                10
            )

        # logging subscribers:
        for i in range(1, self.num_agents + 1):
            self.create_subscription(
                Bool,
                f"/agent{i}/start_logging",
                lambda msg, agent_id = i: self._start_logging_callback(msg, agent_id),
                10
            )

        for i in range(1, self.num_agents + 1):
            self.create_subscription(
                Bool,
                f"/agent{i}/stop_logging",
                lambda msg, agent_id = i: self._stop_logging_callback(msg, agent_id),
                10
            )

        # mission complete subscriber:
        self.create_subscription(String, "/mission_complete", self._mission_complete_callback, 10)

        # let user know logger is working:
        self.get_logger().info(f"PathLoggerNode ready | agents: {self.num_agents} | mission name: {self.mission_name}")

    # method for getting yaw from quaternion:
    def _yaw_from_quaternion(self, q):
        # calcuate and return yaw:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # start logging callback:
    def _start_logging_callback(self, msg: Bool, agent_id: int):
        # form agent name key for dict indexing:
        key = f"agent{agent_id}"

        # reset agent and start logging:
        if msg.data and not self.logging_active[key]:
            self.poses[key]          = []
            self.start_time[key]     = None
            self.last_pos[key]       = None
            self.total_dist[key]     = 0.0
            self.logging_active[key] = True
            self.get_logger().info(f"{key} won the auction - logging started")

    # stop logging callback:
    def _stop_logging_callback(self, msg: Bool, agent_id: int):
        # form agent name key for dict indexing:
        key = f"agent{agent_id}"

        # stop logging:
        self.logging_active[key] = False

        # handle resetting if needed:
        if msg.data:
            self.get_logger().info(f'{key} successfully reached goal')
        else:
            self.poses[key]          = []
            self.start_time[key]     = None
            self.last_pos[key]       = None
            self.total_dist[key]     = 0.0
            self.get_logger().info(f'{key} failed/timed out — poses discarded')

    # odometry callback:
    def _odom_callback(self, msg: Odometry, agent_id: int):
        # form agent name key for dict indexing:
        key = f"agent{agent_id}"

        # check to see if the agent should be actively logging:
        if not self.logging_active[key]:
            return

        # get current time:
        now = self.get_clock().now().nanoseconds * 1e-9

        # add start time to dict:
        if self.start_time[key] is None:
            self.start_time[key] = now

        # extract information from message:
        t = now - self.start_time[key]
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)

        # accumulate distance:
        if self.last_pos[key] is not None:
            dx = x - self.last_pos[key][0]
            dy = y - self.last_pos[key][1]
            self.total_dist[key] += math.sqrt(dx*dx + dy*dy)
        
        # update last pose of agent:
        self.last_pos[key] = (x, y)

        # update pose of agent for time t:
        self.poses[key].append({
            "t"   : round(t, 4),
            "x"   : round(x, 4),
            "y"   : round(y, 4),
            "yaw" : round(yaw, 4) 
        })

    # goal callback:
    def _goal_callback(self, msg: Goal):
        # grab information from goal topic:
        goal_name = msg.id
        if goal_name != "":
            goal_x    = msg.pose.pose.position.x
            goal_y    = msg.pose.pose.position.y
            goal_type = msg.required_capability

            # populate the dict pertaining to the goals:
            self.goals[goal_name] = {"type": goal_type, "x" : goal_x, "y" : goal_y}

    # mission complete callback:
    def _mission_complete_callback(self, msg: String):
        # check value of msg:
        if not msg.data or self.mission_saved:
            return

        # update flag for mission saving:
        self.mission_saved = True

        # create directory for the mission:
        mission_dir = os.path.join(self.save_dir, f"{self.mission_name}")
        os.makedirs(mission_dir, exist_ok = True)

        # save agent data:
        for i in range(1, self.num_agents + 1):
            # extract poses + elapsed time:
            poses = self.poses[f"agent{i}"]
            elapsed = (poses[-1]["t"] - poses[0]["t"]) if len(poses) >= 2 else 0.0

            # collate data:
            data = {
                "mission_name"   : self.mission_name,
                "goal_tolerance" : self.goal_tolerance,
                "elapsed_time"   : round(elapsed, 4),
                "total_distance" : round(self.total_dist[f"agent{i}"], 4),
                "poses"          : poses
            }

            # make and dump to file:
            filename = os.path.join(mission_dir, f"agent{i}.json")
            with open(filename, "w") as f:
                json.dump(data, f, indent = 2)

            # log to user that the agent was saved:
            self.get_logger().info(f"Saved agent{i} -> {filename}")

        # save goal data:
        filename = os.path.join(mission_dir, "goals.json")
        with open(filename, "w") as f:
            json.dump(self.goals, f, indent = 2)

# define main function:
def main():
    # initialize rclpy:
    rclpy.init()

    # instantiate node:
    node = PathLoggerNode()

    # spin node:
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
