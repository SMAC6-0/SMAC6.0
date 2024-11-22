from enum import Enum
import copy
from config import *

class GridStatus(Enum):
    WALKABLE = 0
    NOT_WALKABLE = 1
    INCHWORM_PATH = -1
    INCOMING_BLOCK = 2
    SUPPLY_DEPOT = 3

inchworm_paths = {}

class Node:
    def __init__(self, x, z, y, is_obs, g = 0, h = 0):
        self.x = x
        self.z = z
        self.y = y
        self.is_obs = is_obs # indication if obstacle
        self.g = g # cost
        self.h = h # heuristic
        self.f = g + h # total cost
        self.parent = None

    def __lt__(self, other):
        return self.f < other.f # node comparing for priority queue
    
def initialize_grid_with_structures():
    """
    Initalize the empty 3D workspace such that all cells on the bottom layer are walkable, and the rest are not walkable.

    Returns:
        grid [list]: A 3D list representing the initialized workspace where only the floor is walkable. (All z coordinates = 0).
    """
    # Initialize an empty 3D grid with all cells represented as NOT_WALKABLE
    grid = [[[GridStatus.NOT_WALKABLE.value for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)] 

    # Make the bottom layer (z = 0) WALKABLE
    for x in range(GRID_SIZE):
        for y in range(GRID_SIZE):
            grid[x][0][y] = GridStatus.WALKABLE.value
    
    return grid

def mark_block_depot(grid):
    """
    Initalize all block depots in grid. This is configured in config.py
    
    Args:
        grid [list]: A 3D list of the workspace

    Returns:
        grid [list]: A 3D list of the workspace with the supply depot.
    """
    for i in BD_LOCS[i]:
        x, z, y = BD_LOCS[i]
        if is_in_bounds(grid, BD_LOCS[i]):
            grid[x][z][y] = GridStatus.SUPPLY_DEPOT.value
        else:
            raise ValueError(f"Error: depot location {BD_LOCS[i]} is out of bounds") 
    return grid
    
def update_grid_with_structure(grid, structure):
    """
    Update the 3D workspace being passed in such that the passed in structure becomes walkable and the space beneath it is not.

    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0) or not (1). 
        structure (tuple): A tuple containing the (x, z, y) coordinates of the structure's 
                           position in the grid. This is a single block. 
    Returns:
        grid: (list): An updated 3D list (grid) where the floor & structure is walkable and the cell beneath the structure is not. 
    """ 
    # for structure in structures:        
    x, z, y = structure  # Ensure the order matches your design

    if is_in_bounds(grid, structure):
        grid[x][z][y] = GridStatus.WALKABLE.value #curr cell
        if z - 1 >= 0:
            grid[x][z-1][y] = GridStatus.NOT_WALKABLE.value #cell below
    return grid 

def set_inchworm_path(grid, x, z, y, inchworm_id):
    grid[x][z][y] = GridStatus.INCHWORM_PATH.value
    inchworm_paths[(x, z, y)] = inchworm_id

def set_neighbors(prioritize_vertical):
    if prioritize_vertical:
        primary_neighbors = [(0, 0, 1), (0, 0, -1)]
        secondary_neighbors = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)]
    else:
        primary_neighbors = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)]
        secondary_neighbors = [(0, 0, 1), (0, 0, -1)]

    neighbor_directions = primary_neighbors + secondary_neighbors
    return neighbor_directions

def rework_path_3d(curr_node, holding_block):
    """
    Reverse calculated path to go from start to goal.
    """
    path = []
    while curr_node:
        path.append(([curr_node.x, curr_node.y, curr_node.z], holding_block))
        print("reversed path")
        curr_node = curr_node.parent
    return path[::-1], len(path) - 1

def is_valid_position_3d(grid, x, z, y):
    if (0 <= x < len(grid) and 0 <= y < len(grid[0]) and 0 <= z < len(grid[0][0])):
        is_obs = grid[x][z][y] != GridStatus.WALKABLE.value
        return Node(x, z, y, is_obs=is_obs)
    return None

def is_in_bounds(grid, coords):
    x, y, z = coords
    return 0 <= x < len(grid) and 0 <= z < len(grid[0]) and 0 <= y < len(grid[0][0])

def is_goal_reached_3d(curr_node, goal_node):
    return (curr_node.x == goal_node.x and 
            curr_node.y == goal_node.y and 
            curr_node.z == goal_node.z)

def is_valid_start_goal_3d(grid, start, goal):
    return (is_valid_position_3d(grid, start) and 
            is_valid_position_3d(grid, goal) and 
            not grid[start[0]][start[1]][start[2]] and 
            not grid[goal[0]][goal[1]][goal[2]])