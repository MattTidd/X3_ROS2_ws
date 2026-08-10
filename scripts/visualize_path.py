# imports:
import os
import json
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# define function for loading the json:
def load_agent_json(mission_dir: str) -> dict:
    """
    Function for loading the JSON file of an agent, named agentN.json, from a given mission directory.

    :param mission_dir: Directory for mission paths
    :type mission_dir: str
    """
    # # instantiate an empty dict for the data from the JSON:
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

# set directory to the paths:
mission_dir = os.path.join(os.path.expanduser("~"), "X3_ROS2_ws", "scripts", "recorded_paths", "path1_070826_1723")

# define initial odometry of agents used:
agent_params = {
    "agent1" : {"type" : "typeA", "x" : 0.0, "y" : 0.0},
    # "agent2" : {"type" : "typeB", "x" : 1.0, "y" : 0.5}
}

# define the positions of goals in a dict:
goals = {
    "goal1" : {"type" : "typeA", "x" :  2.0,  "y": 0.75},
    "goal2" : {"type" : "typeB", "x" : -2.0,  "y": 0.75},
    # "goal3" : {"type" : "typeA", "x" :  2.0,  "y": 0.75},
    # "goal4" : {"type" : "typeA", "x" :  2.0,  "y": 0.75},
}

# define a figure:
fig, ax = plt.subplots(figsize = (10, 10))

# load the data for an agent:
agent_data = load_agent_json(mission_dir = mission_dir)

# for every agent:
for agent_id, data in agent_data.items():
    # grab agent specific initialized values:
    agent_init = agent_params.get(agent_id, {"type" : "typeA", "x" : 0.0, "y" : 0.0})
    init_pose  = {"x" : agent_init["x"], "y" : agent_init["y"]}
    agent_type = agent_init["type"]
    colour     = "tab:blue" if agent_type == "typeA" else "tab:red"

    # grab pose information from agent data:
    poses = data["poses"]
    x     = [p["x"] + init_pose["x"] for p in poses]
    y     = [p["y"] + init_pose["y"] for p in poses]
    yaw   = [p["yaw"] for p in poses]

    # grab elapsed time + distance + goal_tolerance:
    elapsed        = data["elapsed_time"]
    distance       = data["total_distance"]
    goal_tolerance = data["goal_tolerance"]

    # form label for plot:
    label = f"{agent_id} | {elapsed:.2f}s | {distance:.2f}m"

    # plot line:
    line = ax.plot(x, y, "--", alpha = 0.5, lw = 2, color = colour, label = label)

    # draw arrows for heading:
    indices = range(0, len(x), 10)
    ax.quiver(
        [x[i] for i in indices],
        [y[i] for i in indices],
        [np.cos(yaw[i]) for i in indices],
        [np.sin(yaw[i]) for i in indices],
        color      = line[0].get_color(),
        scale      = 75, 
        width      = 0.004,
        headwidth  = 5,
        headlength = 5,
        alpha      = 1.0
    )

    # mark the start and end of the path:
    ax.scatter(x[0],  y[0],  marker = "o", s = 80, color = colour, zorder = 5)
    ax.scatter(x[-1], y[-1], marker = "X", s = 80, color = colour, zorder = 5)

# TODO manually add obstacles here:
# ax.add_patch(patches.Rectangle((1.25, 1), 0.1, 0.1, color = "red", alpha = 0.5))
# ax.add_patch(patches.Circle((1.25, 0.25), radius = 0.1, color = "red", alpha = 0.5))

# TODO add goal plotting:
# set tolerance, plot circle for goal and dilate by tolerance
for goal_id, goal_data in goals.items():
    # get the type and subsequent colour of the goal:
    goal_colour = "blue" if goal_data["type"] == "typeA" else "red"

    # get position of the goal:
    goal_x, goal_y = goal_data["x"], goal_data["y"]

    # plot the goal:
    ax.add_patch(patches.Circle((goal_x, goal_y), radius = 0.015, color = goal_colour, alpha = 0.9))
    ax.add_patch(patches.Circle((goal_x, goal_y), radius = goal_tolerance, color = goal_colour, alpha = 0.15))
    ax.text(x = goal_x - 0.16, y = goal_y - (goal_tolerance + 0.1), s = goal_id)

# plot settings:
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_xlim((-4.0, 4.0))
ax.set_ylim((-4.0, 4.0))
ax.tick_params(axis = "both", which = "both", direction = "in", labelsize = "14")
ax.minorticks_on()
ax.grid(False)
ax.legend(loc = "upper left", fontsize = 14)
ax.set_aspect("equal")
ax.set_title(os.path.basename(mission_dir), fontsize = 14)
plt.tight_layout()
plt.show()

