import bfs_path_planning
import map_data
from inchworm_data import *
from config import *
import numpy as np

start_coords = [4, 0, 1]
end_coords = [1, 6, 5]
bfs_path_planning.find_path(map_data.initialize_grid_with_structures(), start_coords, end_coords, False)

# FINAL_MAP = np.array([
#     [  # Z = 0
#         [0, 0, 0, 0, 0, 0], # X row
#         [0, 0, 0, 0, 0, 0],
#         [0, 0, 0, 0, 0, 0],
#         [0, 0, 0, 0, 0, 0],
#         [0, 0, 0, 0, 0, 0]
#     ],
#     [  # Z = 1
#         [0, 0, 0, 0, 0, 0],
#         [0, 0, 0, 0, 0, 0],
#         [0, 0, 0, 0, 0, 0],
#         [0, 0, 0, 0, 0, 0],
#         [0, 0, 0, 0, 0, 0]
#     ],
#     [  # Z = 2
#         [0, 0, 0, 0, 0, 0],
#         [0, 0, 0, 0, 0, 0],
#         [0, 0, 0, 0, 0, 0],
#         [0, 0, 0, 0, 0, 0],
#         [0, 0, 0, 0, 0, 0]
#     ]])
# print(type(FINAL_MAP))

# my_inchy = Inchworm(InchwormOrientation.NORTH, FINAL_MAP, start_coords)
# my_inchy.plan_path()