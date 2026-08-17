# imports:
import os
import json
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# define function for loading an agent json:
def load_agent_json(mission_dir: str) -> dict:
    """
    Function for loading the JSON file of an agent, named agentN.json, from a given mission directory.

    :param mission_dir: Directory for mission data
    :type mission_dir: str
    """
    # instantiate an empty dict for the data from the JSON:
    data = {}

    # get each of the files and load them to the dict, with the key being the agent name:
    for fname in sorted(os.listdir(mission_dir)):
        # if the file starts with "agent" and ends with ".json"
        if fname.startswith("agent") and fname.endswith(".json"):
            # strip the ".json" extension off the fname:
            agent_id = fname.replace(".json", "")

            # open the file and write to the dict:
            with open(os.path.join(mission_dir, fname), "r") as f:
                data[agent_id] = json.load(f)

    # return the data:
    return data

# define function for loading the goal json:
def load_goal_json(mission_dir: str) -> dict:
    """
    Function for loading goal JSON file, named goals.json, from a given mission directory.

    :param mission_dir: Directory for goal data
    :type mission_dir: str
    """
    # instantiate an empty dict for the data from the JSON:
    data = {}

    # get each of the files in the mission dir:
    for fname in sorted(os.listdir(mission_dir)):
        # if the file starts with "goals" and ends with ".json":
        if fname.startswith("goals") and fname.endswith(".json"):
            # open the file and write to the dict:
            with open(os.path.join(mission_dir, fname), "r") as f:
                data = json.load(f)

    # return the data:
    return data

# set directory to the paths:
mission_name = "mission_11"
mission_dir  = os.path.join(os.path.expanduser("~"), "X3_ROS2_ws", "scripts", "recorded_paths", mission_name)

# load the agent and goal data:
agent_data = load_agent_json(mission_dir = mission_dir)
goal_data  = load_goal_json(mission_dir = mission_dir)

# define initial odometry of agents used:
agent_params = {
    "agent1" : {"type" : "typeA", "x" : -1.64, "y" : 0.22},
    "agent2" : {"type" : "typeB", "x" : -1.64, "y" : -0.509}
}

settings = {
    "figure.autolayout": True,
    "font.size": 18,
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif", "serif"],
}
plt.rcParams.update(settings)

# define a figure:
fig, ax = plt.subplots(figsize = (10, 10))
i = 1

# for every agent:
for agent_id, data in agent_data.items():
    # grab agent specific initialized values:
    agent_init = agent_params.get(agent_id, {"type" : "typeA", "x" : 0.0, "y" : 0.0})
    init_pose  = {"x" : agent_init["x"], "y" : agent_init["y"]}
    agent_type = agent_init["type"]
    colour     = "tab:blue" if agent_type == "typeA" else "tab:red"

    # grab pose information from agent data:
    total_x   = []
    total_y   = []
    total_yaw = []

    poses = data["poses"]
    for pose in poses:
        x   = [p["x"] + init_pose["x"] for p in pose]
        y   = [p["y"] + init_pose["y"] for p in pose]
        yaw = [p["yaw"] for p in pose]

        total_x.extend(x)
        total_y.extend(y)
        total_yaw.extend(yaw)

    # grab elapsed time + distance + goal_tolerance:
    elapsed        = data["elapsed_time"]
    distance       = data["total_distance"]
    goal_tolerance = data["goal_tolerance"]

    # form label for plot:
    label = f"Agent {i}"

    # plot line:
    line = ax.plot(total_x, total_y, "--", alpha = 0.5, lw = 2, color = colour, label = label)

    # mark the start and end of the path:
    ax.scatter(total_x[0],  total_y[0],  marker = "o", s = 80, color = colour, zorder = 5)
    ax.scatter(total_x[-1], total_y[-1], marker = "X", s = 80, color = colour, zorder = 5)

    # advance counter: 
    i += 1

# TODO manually add obstacles here:
ax.add_patch(patches.Rectangle((-0.63, -0.06),      width = 0.26,   height = 0.34,  color = "black", alpha = 0.5))
ax.add_patch(patches.Rectangle((-0.925, 0),         width = 0.29,   height = 0.195, color = "black", alpha = 0.5))
ax.add_patch(patches.Rectangle((-0.6375, -1.215),   width = 0.135,  height = 0.34,  color = "black", alpha = 0.5))
ax.add_patch(patches.Rectangle((0.51, 0.14),        width = 0.165,  height = 0.237, color = "black", alpha = 0.5))
ax.add_patch(patches.Rectangle((-0.125, 1.07),      width = 0.235,  height = 0.29,  color = "black", alpha = 0.5))
ax.add_patch(patches.Rectangle((-1.405, 1.02),      width = 0.42,   height = 0.22,  color = "black", alpha = 0.5))
ax.add_patch(patches.Rectangle((1.19, -0.955),      width = 0.32,   height = 0.32,  color = "black", alpha = 0.5))
ax.add_patch(patches.Rectangle((1.98, -0.55),       width = 0.315,  height = 0.47,  color = "black", alpha = 0.5))
ax.add_patch(patches.Rectangle((1.53, 0.65),        width = 0.12,   height = 0.23,  color = "black", alpha = 0.5))
ax.add_patch(patches.Rectangle((0.925, 1.53),       width = 0.39,   height = 0.25,  color = "black", alpha = 0.5))

# TODO add goal plotting:
# set tolerance, plot circle for goal and dilate by tolerance
for goal_id, data in goal_data.items():
    # get the type and subsequent colour of the goal:
    goal_colour = "blue" if data["type"] == "typeA" else "red"

    # get position of the goal:
    goal_x, goal_y = data["x"], data["y"]

    # plot the goal:
    ax.add_patch(patches.Circle((goal_x, goal_y), radius = 0.015, color = goal_colour, alpha = 0.9))
    ax.add_patch(patches.Circle((goal_x, goal_y), radius = goal_tolerance, color = goal_colour, alpha = 0.15))
    # ax.text(x = goal_x - 0.2, y = goal_y - (goal_tolerance + 0.1), s = goal_id)

# plot settings:
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_xlim((-2.5, 2.5))
ax.set_ylim((-1.5, 2))
ax.tick_params(axis = "both", which = "both", direction = "in")
ax.minorticks_off()
ax.grid(False)
ax.legend(loc = "upper left", fontsize = 14)
ax.set_aspect("equal")
plt.title("Mission 6")
plt.tight_layout()
plt.savefig(f"{mission_name}.svg", format = "svg", bbox_inches = "tight")
plt.show()

