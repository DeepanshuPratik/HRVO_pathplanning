from multiple_robot import plot_robot
from creating_obstacle import create_obstacle
from robot_velocity import compute_max_possible_velocity
import numpy as np
import random

SIM_TIME = 6.
TIMESTEP = 0.1
NUMBER_OF_TIMESTEPS = int(SIM_TIME/TIMESTEP)
VMAX = 2
VMIN = 0.1


def simulate(Goal1_x, Goal1_y, Goal2_x, Goal2_y,obstacle_count,Robot_radius):
    obstacles = create_obstacle(SIM_TIME, NUMBER_OF_TIMESTEPS,int(obstacle_count))

    start1 = np.array([random.randrange(0,10),random.randrange(0,10), 0, 0])
    start2 = np.array([random.randrange(0,10),random.randrange(0,10), 0, 0])
    start3 = np.array([random.randrange(0,10),random.randrange(0,10), 0, 0])
    goal1 = np.array([int(Goal1_x), int(Goal1_y), 0, 0]) 
    goal2 = np.array([int(Goal2_x), int(Goal2_y), 0, 0])
    goal3 = np.array([1,0, 0, 0])

    robot1_state = start1 
    robot2_state = start2
    robot3_state = start3 
    robot1_state_history = np.empty((NUMBER_OF_TIMESTEPS, NUMBER_OF_TIMESTEPS))
    robot2_state_history = np.empty((NUMBER_OF_TIMESTEPS, NUMBER_OF_TIMESTEPS))
    robot3_state_history = np.empty((NUMBER_OF_TIMESTEPS, NUMBER_OF_TIMESTEPS))
    
    for i in range(NUMBER_OF_TIMESTEPS):
        v1_desired = compute_max_possible_velocity(robot1_state, goal1, float(Robot_radius), VMAX)
        control_vel1 = compute_velocity(
            float(Robot_radius), robot1_state, obstacles[:, i, :], v1_desired)
        robot1_state = update_state(robot1_state, control_vel1)
        robot1_state_history[:4, i] = robot1_state

        v2_desired = compute_max_possible_velocity(robot2_state, goal2, float(Robot_radius), VMAX)
        control_vel2 = compute_velocity(
            float(Robot_radius), robot2_state, obstacles[:, i, :], v2_desired)
        robot2_state = update_state(robot2_state, control_vel2)
        robot2_state_history[:4, i] = robot2_state

        v3_desired = compute_max_possible_velocity(robot3_state, goal3, float(Robot_radius), VMAX)
        control_vel3 = compute_velocity(
            float(Robot_radius), robot3_state, obstacles[:, i, :], v3_desired)
        robot3_state = update_state(robot3_state, control_vel3)
        robot3_state_history[:4, i] = robot3_state

    plot_robot(
        robot1_state_history, robot2_state_history, robot3_state_history, obstacles, float(Robot_radius), NUMBER_OF_TIMESTEPS, SIM_TIME)
    # plot_robot(
    #     robot2_state_history, obstacles, float(Robot_radius), NUMBER_OF_TIMESTEPS, SIM_TIME)

def compute_velocity(Robot_radius,robot, obstacles, v_desired):
    pA = robot[:2]
    vA = robot[-2:]
    number_of_obstacles = np.shape(obstacles)[1]
    Amat = np.empty((number_of_obstacles * 2, 2))
    bvec = np.empty((number_of_obstacles * 2))
    for i in range(number_of_obstacles):
        obstacle = obstacles[:, i]
        pB = obstacle[:2]
        vB = obstacle[2:]
        dispBA = pA - pB
        distBA = np.linalg.norm(dispBA)
        thetaBA = np.arctan2(dispBA[1], dispBA[0])
        if 2.2 * (Robot_radius) > distBA:
            distBA = 2.2*(Robot_radius)
        phi_obst = np.arcsin(2.2*(Robot_radius)/distBA)
        phi_left = thetaBA + phi_obst
        phi_right = thetaBA - phi_obst

        # VO
        translation = vB
        Atemp, btemp = create_constraints(translation, phi_left, "left")
        Amat[i*2, :] = Atemp
        bvec[i*2] = btemp
        Atemp, btemp = create_constraints(translation, phi_right, "right")
        Amat[i*2 + 1, :] = Atemp
        bvec[i*2 + 1] = btemp

    # Create search-space
    th = np.linspace(0, 2*np.pi, 20)
    vel = np.linspace(0, VMAX, 5)

    vv, thth = np.meshgrid(vel, th)

    vx_sample = (vv * np.cos(thth)).flatten()
    vy_sample = (vv * np.sin(thth)).flatten()

    v_sample = np.stack((vx_sample, vy_sample))

    v_satisfying_constraints = check_constraints(v_sample, Amat, bvec)

    # Objective function
    size = np.shape(v_satisfying_constraints)[1]
    diffs = v_satisfying_constraints - \
        ((v_desired).reshape(2, 1) @ np.ones(size).reshape(1, size))
    norm = np.linalg.norm(diffs, axis=0)
    min_index = np.where(norm == np.amin(norm))[0][0]
    cmd_vel = (v_satisfying_constraints[:, min_index])

    return cmd_vel


def check_constraints(v_sample, Amat, bvec):
    length = np.shape(bvec)[0]

    for i in range(int(length/2)):
        v_sample = check_inside(v_sample, Amat[2*i:2*i+2, :], bvec[2*i:2*i+2])

    return v_sample


def check_inside(v, Amat, bvec):
    v_out = []
    for i in range(np.shape(v)[1]):
        if not ((Amat @ v[:, i] < bvec).all()):
            v_out.append(v[:, i])
    return np.array(v_out).T


def create_constraints(translation, angle, side):
    # create line
    origin = np.array([0, 0, 1])
    point = np.array([np.cos(angle), np.sin(angle)])
    line = np.cross(origin, point)
    line = translate_line(line, translation)

    if side == "left":
        line *= -1

    A = line[:2]
    b = -line[2]

    return A, b


def translate_line(line, translation):
    matrix = np.eye(3)
    matrix[2, :2] = -translation[:2]
    return matrix @ line


def update_state(x, v):
    new_state = np.empty((4))
    new_state[:2] = x[:2] + v * TIMESTEP
    new_state[-2:] = v
    return new_state
