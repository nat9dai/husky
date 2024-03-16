import rosbag
import matplotlib.pyplot as plt
import numpy as np

# Define the names of the Huskies and the floor plane for exclusion.
huskies = ["husky_0", "husky_1", "husky_2"]
exclude_models = huskies + ["asphalt_plane", "ground_plane"]

# Initialize dictionaries to store positions.
husky_positions = {name: [] for name in huskies}
obstacle_positions = {}

# Path to your bag file
bag_path = '/home/nat/.ros/test_1_mpc.bag'

# Reading the bag file
with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/gazebo/model_states']):
        for i, name in enumerate(msg.name):
            position = msg.pose[i].position
            if name in husky_positions:
                # Swap x and y, then negate the new x for 90 degrees CW rotation
                husky_positions[name].append((-position.y, position.x))
            elif name not in exclude_models:
                # Apply the same transformation to obstacles
                obstacle_positions[name] = (-position.y, position.x)

# Plotting the trajectories and obstacles.
plt.figure()
legend_handles = []  # List to keep track of legend entries

for name, positions in husky_positions.items():
    x_vals, y_vals = zip(*positions) if positions else ([], [])
    line, = plt.plot(x_vals, y_vals, label=name)
    legend_handles.append(line)

    # Place direction-indicating triangle heads at specific points along the trajectory
    num_markers = 5
    indices = np.round(np.linspace(0, len(positions) - 1, num_markers)).astype(int)
    for idx, index in enumerate(indices):
        # Compute the direction angle
        if name == "husky_0" and index == indices[-1]:  # Special case for the last point of husky_0
            direction = np.arctan2(y_vals[index] - y_vals[index - 1], x_vals[index] - x_vals[index - 1])
            direction += np.pi  # Rotate 180 degrees
        elif index < len(positions) - 1:
            direction = np.arctan2(y_vals[index + 1] - y_vals[index], x_vals[index + 1] - x_vals[index])
        else:
            direction = np.arctan2(y_vals[index] - y_vals[index - 1], x_vals[index] - x_vals[index - 1])

        plt.quiver(x_vals[index], y_vals[index], np.cos(direction), np.sin(direction), 
                   scale=50, color=line.get_color(), headwidth=6, headlength=10, headaxislength=10, 
                   width=0.005, pivot='mid', minlength=0.01)

# Plot all obstacles in the same color without adding them to the legend
obstacle_color = 'gray'
for name, position in obstacle_positions.items():
    plt.plot(position[0], position[1], 'o', color=obstacle_color)

# Creating the legend only for the robots
plt.legend(handles=legend_handles, loc='upper left')

plt.xlabel('X position')
plt.ylabel('Y position')
plt.title('Trajectories of Husky Robots with Directional Indicators')
plt.show()
