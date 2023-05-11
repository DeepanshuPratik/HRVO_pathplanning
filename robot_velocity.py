import numpy as np

def compute_max_possible_velocity(current_coordinates, goal_coordinates, robot_radius, max_speed):
    displacement_vec = (goal_coordinates - current_coordinates)[:2]
    norm = np.linalg.norm(displacement_vec)
    if norm < robot_radius / 5:
        return np.zeros(2)
    displacement_vec = displacement_vec / norm
    np.shape(displacement_vec)
    robot_velocity = max_speed * displacement_vec
    return robot_velocity
