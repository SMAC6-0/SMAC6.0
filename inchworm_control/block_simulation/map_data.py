from enum import Enum
import copy
from config import BD_LOC, GRID_SIZE

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
    
def initialize_grid_with_structures(grid_size = GRID_SIZE, bd_loc = None):
    """
    Initalize the empty 3D workspace such that all cells on the bottom layer are walkable, and the rest are not walkable.

    Args:
        GRID_SIZE (int): The size of the workspace, as a grid.
    Returns:
        list: A 3D list representing the initialized workspace where only the floor is walkable. (All z coordinates = 0).
    """
    grid = [[[GridStatus.NOT_WALKABLE.value for _ in range(grid_size)] for _ in range(grid_size)] for _ in range(grid_size)]

    # Make the bottom layer (z=0) walkable
    for x in range(grid_size):
        for y in range(grid_size):
            grid[x][0][y] = GridStatus.WALKABLE.value
            
    if bd_loc:
        mark_block_depot(grid, bd_loc)
    return grid

def mark_block_depot(grid, bd_loc = BD_LOC):
    x, y, z = bd_loc
    if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and 0 <= z < len(grid[0][0]):
        grid[x][z][y] = GridStatus.WALKABLE.value
        if z - 1 >= 0:
            grid[x][z - 1][y] = GridStatus.NOT_WALKABLE.value
    else:
        print(f"Error: BD_LOC {bd_loc} is out of bounds") 
    
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

def update_grid_with_structure(grid, structure):
    # for structure in structures:        
    x, z, y = structure  # Ensure the order matches your design

    if ((0 <= x < GRID_SIZE) and (0 <= y < GRID_SIZE) and (0 <= z < GRID_SIZE)):
        grid[x][z][y] = GridStatus.WALKABLE.value #curr cell
        if z - 1 >= 0:
            grid[x][z-1][y] = GridStatus.NOT_WALKABLE.value #cell below
    return grid

def update_grid_with_structure_not_walkable(grid, structure):
    # for structure in structures:        
    x, z, y = structure  # Ensure the order matches your design

    if ((0 <= x < GRID_SIZE) and (0 <= y < GRID_SIZE) and (0 <= z < GRID_SIZE)):
        grid[x][z-1][y] = GridStatus.WALKABLE  # cell below
    return grid 

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

def is_goal_reached_3d(curr_node, goal_node):
    return (curr_node.x == goal_node.x and 
            curr_node.y == goal_node.y and 
            curr_node.z == goal_node.z)

def is_valid_start_goal_3d(grid, start, goal):
    return (is_valid_position_3d(grid, start) and 
            is_valid_position_3d(grid, goal) and 
            not grid[start[0]][start[1]][start[2]] and 
            not grid[goal[0]][goal[1]][goal[2]])