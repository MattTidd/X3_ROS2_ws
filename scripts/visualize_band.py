# imports:
import os 
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.ndimage import gaussian_filter1d, maximum_filter1d, minimum_filter1d, grey_closing

# function for loading mission data:
def load_missions(base_dir : str, mission_names : list[str]) -> tuple[dict, dict]:
    # instantiate dicts for agent and goal data:
    agent_runs = {}
    goal_data  = {}

    # for every mission passed:
    for mission_name in mission_names:
        # get that directory:
        mission_dir = os.path.join(base_dir, mission_name)

        # if it does not exist, warn user:
        if not os.path.isdir(mission_dir):
            print(f"Warning: mission directory not found: {mission_dir}")
            continue

        # get each of the files and load them to the dict, with the key being the agent name:
        for fname in sorted(os.listdir(mission_dir)):
            # if the file starts with "agent" and ends with ".json"
            if fname.startswith("agent") and fname.endswith(".json"):
                # strip the ".json" extension off the fname:
                agent_id = fname.replace(".json", "")

                # get the file path:
                fpath = os.path.join(mission_dir, fname)

                # open the filepath:
                with open(fpath, "r") as f:
                    data = json.load(f)

                # add that id to the runs:
                if agent_id not in agent_runs:
                    agent_runs[agent_id] = {
                        "runs" : [],
                        "goal_tolerance" : data.get("goal_tolerance", 0.2)
                    }

                # each JSON contains a single agent trajectory:
                agent_runs[agent_id]["runs"].append(data["poses"])

            # otherwise load the goal file:
            elif fname == "goals.json":
                # open file:
                with open(os.path.join(mission_dir, fname), "r") as f:
                    # save JSON data:
                    goal_data = json.load(f)
            
    return agent_runs, goal_data

# function for computing a cumulative arc length:
def compute_arc_length(x : np.ndarray, y : np.ndarray) -> np.ndarray:
    # get the difference along the axes:
    dx = np.diff(x)
    dy = np.diff(y)

    # find the segment lengths:
    segment_lengths = np.hypot(dx, dy)

    return np.concatenate([
        [0.0],
        np.cumsum(segment_lengths)
    ])

# function for building a mean trajectory and variability band:
def build_envelope(runs, 
               init_pose, 
               n_samples = 1_000):
    # initialize a list for trajectories:
    trajectories = []

    # resample each run onto a common progress axis:
    progress = np.linspace(0.0, 1.0, n_samples)

    # for every run:
    for run in runs:
        # instantiate list of all poses:
        all_poses = []

        # for each segment in a given run:
        for segment in run:
            # if segement exists, add to list of all poses:
            if segment:
                all_poses.extend(segment)

        # ensure two agents have been added:
        if len(all_poses) < 2:
            continue

        # extract global x/y position:
        x = np.array([p["x"] + init_pose["x"] for p in all_poses])
        y = np.array([p["y"] + init_pose["y"] for p in all_poses])

        # remove duplicate consecutive points:
        dx   = np.diff(x)        # get difference along x axis
        dy   = np.diff(y)        # get difference along y axis
        keep = np.concatenate([  # mask for keeping points
            [True],
            np.hypot(dx, dy) > 1e-6
        ])

        x = x[keep] # apply mask to x
        y = y[keep] # apply mask to y

        # ensure two agents:
        if len(x) < 2:
            continue

        # calculate arc length:
        s = compute_arc_length(x, y)
        if s[-1] <= 1e-9:
            continue

        # normalize trajectory progress:
        s /= s[-1]

        # ensure unique interpolation coordinates exist:
        s, unique_idx = np.unique(
            s,
            return_index = True
        )

        # apply mask:
        x = x[unique_idx]
        y = y[unique_idx]

        # interpolate a complete run onto a common progress grid:
        x_interp = np.interp(progress, s, x)
        y_interp = np.interp(progress, s, y)
        trajectories.append(np.column_stack([x_interp, y_interp]))

    # ensure trajectories found are valid:
    if not trajectories:
        raise ValueError("No valid trajectories found")

    # convert to array:
    trajectories = np.asarray(trajectories) # shape: (n_runs, n_samples, 2)

    # find mean trajectory:
    mean_x = np.mean(trajectories[:, :, 0], axis = 0)
    mean_y = np.mean(trajectories[:, :, 1], axis = 0)

    # tangent to mean trajectory:
    tangent_x    = np.gradient(mean_x)
    tangent_y    = np.gradient(mean_y)
    tangent_norm = np.hypot(tangent_x, tangent_y)

    # prevent division by zero:
    tangent_norm[tangent_norm < 1e-12] = 1.0
    tangent_x /= tangent_norm
    tangent_y /= tangent_norm

    # normal to mean trajectory:
    normal_x = -tangent_y
    normal_y = tangent_x

    # mean path points
    mean_points = np.column_stack((mean_x, mean_y))

    # start at zero so the mean is always inside the band:
    upper_raw = np.full(n_samples, -np.inf)
    lower_raw = np.full(n_samples, np.inf)

    # for every run:
    for run in trajectories:
        # distance from every run point to every mean point:
        diff = (run[:, None, :] - mean_points[None, :, :])
        d_sq = np.sum(diff ** 2, axis = 2)

        # closest mean sample for every run point:
        closest_idx = np.argmin(d_sq, axis = 1)

        # deviation from its closest mean point:
        delta = run - mean_points[closest_idx]

        # signed normal distance:
        distances = (delta[:, 0] * normal_x[closest_idx] + delta[:, 1] * normal_y[closest_idx])

        # only update upper with positive distances:
        positive = distances > 0
        np.maximum.at(
            upper_raw,
            closest_idx[positive],
            distances[positive]
        )

        # only update lower with negative distances:
        negative = distances < 0
        np.minimum.at(
            lower_raw,
            closest_idx[negative],
            distances[negative]
        )

    # interpolate:
    upper_raw[~np.isfinite(upper_raw)] = np.nan
    lower_raw[~np.isfinite(lower_raw)] = np.nan
    idx = np.arange(n_samples)

    valid = ~np.isnan(upper_raw)
    upper_distance = np.interp(
        idx,
        idx[valid],
        upper_raw[valid]
    )

    valid = ~np.isnan(lower_raw)
    lower_distance = np.interp(
        idx,
        idx[valid],
        lower_raw[valid]
    )

    # filter:
    upper_distance = grey_closing(upper_distance, size = 11)
    lower_distance = -grey_closing(-lower_distance, size = 11)

    # reconstruct boundaries
    x_upper = mean_x + upper_distance * normal_x
    y_upper = mean_y + upper_distance * normal_y

    x_lower = mean_x + lower_distance * normal_x
    y_lower = mean_y + lower_distance * normal_y

    return {
        "progress": progress,

        "x_mean": mean_x,
        "y_mean": mean_y,

        "x_upper": x_upper,
        "y_upper": y_upper,

        "x_lower": x_lower,
        "y_lower": y_lower,

        "all_x": trajectories[:, :, 0],
        "all_y": trajectories[:, :, 1],

        "n_runs": len(trajectories)
    }

# define base directory:
base_dir = os.path.join(os.path.expanduser("~"), "X3_ROS2_ws", "scripts", "recorded_paths")

# define desired mission names:
mission_names = [f"mission_{i}" for i in range(6, 12)]

# define title of plot:
plot_title = "Mean Agent Trajectory with Min-Max Envelope"

# define the save name:
save_name = "mean_trajectory_band"

# define initial odometry of agents used:
agent_params = {
    "agent1" : {"type" : "typeA", "x" : -1.64, "y" : 0.22},
    "agent2" : {"type" : "typeB", "x" : -1.64, "y" : -0.509}
}

# load data:
agent_runs, goal_data = load_missions(base_dir = base_dir, mission_names = mission_names)

# define plot settings:
settings = {
    "figure.autolayout": True,
    "font.size": 18,
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif", "serif"],
}

# apply plot settings:
plt.rcParams.update(settings)

# create a figure:
fig, ax = plt.subplots(figsize = (10, 10))

# plotting agent paths:
for agent_idx, (agent_id, adata) in enumerate(agent_runs.items(), start = 1):
    # initialize agent:
    agent_init = agent_params.get(agent_id, {"type": "typeA", "x": 0.0, "y": 0.0})

    # grab the initial pose:
    init_pose = {"x" : agent_init["x"], "y" : agent_init["y"]}

    # set colour:
    if agent_init["type"] == "typeA":
        colour = "tab:blue"
    else:
        colour = "tab:red"

    # label for agent:
    label = f"Agent {agent_idx}"

    # get trajectory information:
    band = build_envelope(adata["runs"],
                          init_pose, 
                          n_samples = 1000)

    # # plot an individual trajectory:
    # for k in range(band["all_x"].shape[0]):
    #     ax.plot(band["all_x"][k], band["all_y"][k], "-", alpha = 0.2, lw = 1, color = colour)

    # plot mean trajectory:
    ax.plot(
        band["x_mean"],
        band["y_mean"],
        "--",
        alpha = 0.8,
        lw = 2.0,
        color = colour,
        label = label
    )

    # plot upper and lower band:
    ax.plot(
        band["x_upper"][:-2],
        band["y_upper"][:-2],
        "-",
        color=colour,
        lw=2,
        alpha=0.4
    )

    ax.fill(
        np.concatenate([
            band["x_upper"],
            band["x_lower"][::-1]
        ]),
        np.concatenate([
            band["y_upper"],
            band["y_lower"][::-1]
        ]),
        color=colour,
        alpha=0.2,
        linewidth=0
    )

    ax.plot(
        band["x_lower"][:-2],
        band["y_lower"][:-2],
        "-",
        color=colour,
        lw=2,
        alpha=0.4
    )

    # mark the start position:
    ax.scatter(
        band["x_mean"][0],
        band["y_mean"][0],
        marker = "o",
        s      = 80,
        color  = colour, 
        zorder = 5
    )

    # mark the end position:
    ax.scatter(
        band["x_mean"][-1],
        band["y_mean"][-1],
        marker = "X",
        s      = 80,
        color  = colour,
        zorder = 5
    )

# manually plot obstacles:
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

# plot goals:
for goal_id, data in goal_data.items():
    # get the type and subsequent colour of the goal:
    goal_colour = "blue" if data["type"] == "typeA" else "red"

    # get position of the goal:
    goal_x, goal_y = data["x"], data["y"]

    # grab tolerance:
    goal_tol = data.get(
        "goal_tolerance",
        0.2)

    # plot:
    ax.add_patch(patches.Circle((goal_x, goal_y), radius = 0.015,    color = goal_colour, alpha = 0.90))
    ax.add_patch(patches.Circle((goal_x, goal_y), radius = goal_tol, color = goal_colour, alpha = 0.15))

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
plt.title(plot_title)
plt.tight_layout()
plt.savefig(f"{save_name}.svg", format = "svg", bbox_inches = "tight")
plt.show()








    #         # find closest point on mean trajectory
    #         diff = mean_points - point
    #         dist_sq = np.sum(diff**2, axis=1)
    #         j = np.argmin(dist_sq)

    #         # vector from mean to trajectory point
    #         dx = point[0] - mean_x[j]
    #         dy = point[1] - mean_y[j]

    #         # signed perpendicular distance
    #         d = (
    #             dx * normal_x[j]
    #             + dy * normal_y[j]
    #         )

    #         distance_bins[j].append(d)