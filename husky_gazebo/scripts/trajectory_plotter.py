#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
import matplotlib.pyplot as plt

class TrajectoryPlotter:
    def __init__(self):
        rospy.init_node('trajectory_plotter', anonymous=True)

        self.husky_positions = {"husky_0": [], "husky_1": [], "husky_2": []}
        self.obstacle_positions = []  # List to store positions of static obstacles

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)

    def callback(self, data):
        for i, name in enumerate(data.name):
            if name in self.husky_positions:
                position = data.pose[i].position
                self.husky_positions[name].append((position.x, position.y))
            elif name not in ["husky_0", "husky_1", "husky_2", "asphalt_plane"]:
                # Record only the first position because they are static
                if len(self.obstacle_positions) < len(data.name) - 4:
                    position = data.pose[i].position
                    self.obstacle_positions.append((position.x, position.y))

    def plot_trajectories(self):
        plt.figure()
        for name, positions in self.husky_positions.items():
            x_vals, y_vals = zip(*positions) if positions else ([], [])
            plt.plot(x_vals, y_vals, label=name)

        # Plot the obstacles
        if self.obstacle_positions:
            obs_x, obs_y = zip(*self.obstacle_positions)
            plt.scatter(obs_x, obs_y, c='red', label='Obstacles')

        plt.xlabel('X position')
        plt.ylabel('Y position')
        plt.title('Trajectories of Husky Robots and Static Obstacles')
        plt.legend()
        plt.show()

if __name__ == '__main__':
    plotter = TrajectoryPlotter()
    rospy.spin()  # Keep the node running until it's interrupted
    plotter.plot_trajectories()  # Once rospy.spin() exits, plot the trajectories
