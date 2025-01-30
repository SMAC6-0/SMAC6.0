import map_data
from inchworm_data import *
from config import *
import numpy as np

start_coords = [10, 0, 10]
end_coords = [20, 0, 20]
FINAL_MAP = np.array([
    [  # Z = 0
        [0, 0, 0, 0, 0, 0], # X row
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]
    ],
    [  # Z = 1
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]
    ],
    [  # Z = 2
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]
    ]])
print(type(FINAL_MAP))
my_inchy = Inchworm(InchwormOrientation.NORTH, FINAL_MAP, start_coords)
my_inchy.plan_path()