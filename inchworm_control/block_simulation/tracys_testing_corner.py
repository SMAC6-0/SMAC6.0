import bfs_path_planning
import map_data
from inchworm_data import *
from config import *
import numpy as np

start_coords = [4, 0, 1]
end_coords = [6, 5, 1]
grid = map_data.initialize_grid()
grid = map_data.update_grid_with_structure(grid, end_coords)
# bfs_path_planning.find_path(grid, start_coords, BD_LOC1, False)
# bfs_path_planning.find_path(grid, BD_LOC1, end_coords, True)

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

my_inchy = Inchworm(InchwormOrientation.NORTH, grid, start_coords)
my_inchy.plan_path()