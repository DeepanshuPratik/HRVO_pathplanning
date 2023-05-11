
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import numpy as np


def plot_robot(robot, robot1,robot2, obstacles, robot_radius, num_steps, sim_time):
    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(0, 10), ylim=(0, 10))
    ax.set_aspect('equal')
    ax.grid()
    line, = ax.plot([], [], '--r')
    line1, = ax.plot([], [], '--r')
    line2, = ax.plot([], [], '--r')

    robot1_patch = Circle((robot[0, 0], robot[1, 0]),
                         robot_radius, facecolor='green', edgecolor='black')
    robot2_patch = Circle((robot1[0, 0], robot1[1, 0]),
                         robot_radius, facecolor='green', edgecolor='black')
    robot3_patch = Circle((robot2[0, 0], robot2[1, 0]),
                         robot_radius, facecolor='green', edgecolor='black')
    obstacle_list = []
    for obstacle in range(np.shape(obstacles)[2]):
        obstacle = Circle((0, 0), robot_radius,
                          facecolor='aqua', edgecolor='black')
        obstacle_list.append(obstacle)

    def init():
        ax.add_patch(robot1_patch)
        ax.add_patch(robot2_patch)
        ax.add_patch(robot3_patch)
        for obstacle in obstacle_list:
            ax.add_patch(obstacle)
        line.set_data([], [])
        line1.set_data([], [])
        line2.set_data([], [])
        return [robot3_patch] + [line2] + [robot2_patch] + [line1] + [robot1_patch] + [line] + obstacle_list

    def animate(i):
        robot1_patch.center = (robot[0, i], robot[1, i])
        robot2_patch.center = (robot1[0, i], robot1[1, i])
        robot3_patch.center = (robot2[0, i], robot2[1, i])
        for j in range(len(obstacle_list)):
            obstacle_list[j].center = (obstacles[0, i, j], obstacles[1, i, j])
        line.set_data(robot[0, :i], robot[1, :i])
        line1.set_data(robot1[0, :i], robot1[1, :i])
        line2.set_data(robot2[0, :i], robot2[1, :i])
        return [robot3_patch] + [line2] + [robot2_patch] + [line1] + [robot1_patch] + [line] + obstacle_list

    init()
    step = (sim_time / num_steps)
    for i in range(num_steps):
        animate(i)
        plt.pause(step)


