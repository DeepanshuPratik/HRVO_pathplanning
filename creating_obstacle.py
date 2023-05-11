import numpy as np
import random

def create_obstacle(simulation_time, number_timesteps,number_of_obstacles):
    if number_of_obstacles!=0: 
        v = random.randrange(-3,3)
        p0 = np.array([random.randrange(0,10),random.randrange(0,10)])
        obstacle = create_robot(p0,v,(np.pi)/(random.randrange(1,6)),simulation_time,number_timesteps).reshape(4, number_timesteps, 1)
        obstacles = obstacle

    for i in range(1,number_of_obstacles):
        v = random.randrange(-3,3)
        p0 = np.array([random.randrange(0,10),random.randrange(0,10)])
        obstacle = create_robot(p0,v,(np.pi)/(random.randrange(1,6)),simulation_time,number_timesteps).reshape(4, number_timesteps, 1)
        obstacles = np.dstack((obstacles, obstacle))
    if number_of_obstacles!=0:
        return obstacles
    else : 
        v = 0
        p0 = np.array([-3,10])
        obstacle = create_robot(p0,v,(np.pi)/2,simulation_time,number_timesteps).reshape(4, number_timesteps, 1)
        obstacles = obstacle
        return obstacles
    

def create_robot(p0, v, theta, sim_time, num_timesteps):
    # Creates obstacles starting at p0 and moving at v in theta direction
    t = np.linspace(0, sim_time, num_timesteps)
    theta = theta * np.ones(np.shape(t))
    vx = v * np.cos(theta)
    vy = v * np.sin(theta)
    v = np.stack([vx, vy])
    p0 = p0.reshape((2, 1))
    p = p0 + np.cumsum(v, axis=1) * (sim_time / num_timesteps)
    p = np.concatenate((p, v))
    return p