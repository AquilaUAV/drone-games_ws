import numpy as np
from numpy.linalg import linalg
from math import *

instances_num = 6
border_size = 10.0


def calc_final_poses(last_central, land_height=0.0):
    final_poses = []
    last_central = last_central[len(last_central) - 2: len(last_central)]
    move_finish = np.array(last_central[1])
    move_vector = move_finish - np.array(last_central[0])
    move_finish = move_finish - move_vector * border_size / linalg.norm(move_vector)
    print(move_finish)
    move_finish[2] = 0.0
    land_x = np.cross(move_vector, np.array([0.0, 0.0, 1.0]))
    lanx_y = np.cross(np.array([0.0, 0.0, 1.0]), land_x)
    land_x /= linalg.norm(land_x)
    lanx_y /= linalg.norm(lanx_y)
    size_x = ceil(sqrt(instances_num))
    size_y = ceil(int(instances_num) / size_x)
    step_x = 2 * border_size / (size_x + 1)
    step_y = 2 * border_size / (size_y + 1)
    for i in range(int(instances_num)):
        x = i % size_x
        y = i // size_x
        local_land = ((1 + x) * step_x - border_size) * land_x + ((1 + y) * step_y - border_size) * lanx_y + move_finish
        final_poses.append(local_land.tolist())
    return final_poses


last_central = [[90.0, 0.0, 10.0], [120.0, 0.0, 10.0], [120.0, 30.0, 10.0]]
print(calc_final_poses([[120.0, 70.0, 10.0], [120.0, 130.0, 10.0]]))
