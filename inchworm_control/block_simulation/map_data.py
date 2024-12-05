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

class Cell:
    def __init__(self, x: int, z: int, y: int, is_obs: bool, g = 0, h = 0): 
        """
        Initialize the Cell class. It represents a single cell (location) within the map or grid, and is used for path planning purposes. 
        Args:
            x (int): x location of the cell.
            z (int): z location of the cell.
            y (int): y location of the cell.
            is_obs (bool): True if this cell is occupied, not walkable. False if walkable. 
            g (int): The cost to reach this cell. 
            h (int): Evaluated additional heuristic cost to reach this cell. 
        """
        self.x = x
        self.z = z
        self.y = y
        self.is_obs = is_obs
        self.g = g
        self.h = h 
        self.f = g + h # total cost
        self.parent = None # The parent may later be set as another Cell object. 

    def __lt__(self, other): 
        """
        Less than. Returns true is this Cell object's total cost is less than the total cost on the inputted Cell (other). 
        This is used for node comparison for the priority queue. 
        Args:
            other (Cell): Another Cell object. 
        """
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
        grid (list): An updated 3D list (grid) where the floor & structure is walkable and the cell beneath the structure is not. 
    """ 
    # for structure in structures:        
    x, z, y = structure  # Ensure the order matches your design

    if is_in_bounds(grid, structure):
        grid[x][z][y] = GridStatus.WALKABLE.value #curr cell
        if z - 1 >= 0:
            grid[x][z-1][y] = GridStatus.NOT_WALKABLE.value #cell below
    return grid 

def set_inchworm_path(grid, x, z, y, inchworm_id):
    """
    Sets the possible ewalkable and the space beneath it is not.

    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0) or not (1). 
        structure (tuple): A tuple containing the (x, z, y) coordinates of the structure's 
                           position in the grid. This is a single block. 
    Returns:
        grid (list): An updated 3D list (grid) where the floor & structure is walkable and the cell beneath the structure is not. 
    """ 
    grid[x][z][y] = GridStatus.INCHWORM_PATH.value
    inchworm_paths[(x, z, y)] = inchworm_id

def set_neighbors(prioritize_vertical, allow_diagonal, allow_large_build):    
    base_neighbors = [(1, 0, 0), (-1, 0, 0), (0, 0, 1), (0, 0, -1)]
    vertical_neighbors = [(0, 1, 0), (0, -1, 0)]
    diagonal_neighbors = [(1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0),
                          (1, 0, 1), (0, 1, 1), (-1, 0, 1), (0, -1, 1),
                          (1, 0, -1), (0, 1, -1), (-1, 0, -1), (0, -1, -1),
                          (1, 1, 1), (1, -1, 1), (-1, 1, 1), (-1, -1, 1),
                          (1, 1, -1), (1, -1, -1), (-1, 1, -1), (-1, -1, -1)]
    large_build_neighbors = [(1, 2, 0), (1, -2, 0), (-1, 2, 0), (-1, -2, 0),
                             (0, 2, -1), (0, -2, -1), (0, -2, 1), (0, 2, 1)]
    
    # combined neighbor_directions based on conditions
    neighbor_directions = base_neighbors
    
    if prioritize_vertical:
        neighbor_directions += vertical_neighbors
    if allow_diagonal:
        neighbor_directions += diagonal_neighbors
    if allow_large_build:
        neighbor_directions += large_build_neighbors
        
    return neighbor_directions

# Define test cases based on the original arrays
test_cases = {
    "no_vertical_large_build": {
        "prioritize_vertical": False,
        "allow_diagonal": True,
        "allow_large_build": True,
        "expected": [
            (1, 0, 0), (-1, 0, 0), (0, 0, 1), (0, 0, -1), #first layer aside from up down
            (0, 1, 0), (0, -1, 0), #up down
            (1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0), #xz plane y = 0 cross
            (1, 0, 1), (0, 1, 1), (-1, 0, 1), (0, -1, 1), # xz plane y = 1 cross
            (1, 0, -1), (0, 1, -1), (-1, 0, -1), (0, -1, -1), # xz plane y = -1 cross
            (1, 1, 1), (1, -1, 1), (-1, 1, 1), (-1, -1, 1), # xz plane y = 1 corners
            (1, 1, -1), (1, -1, -1), (-1, 1, -1), (-1, -1, -1) #xz plane y = -1 corners
        ]
    },
    "no_vertical_no_diagonal_small_build": {
        "prioritize_vertical": False,
        "allow_diagonal": False,
        "allow_large_build": True,
        "expected": [
            (1, 0, 0), (-1, 0, 0), (0, 0, 1), (0, 0, -1), #first layer aside from up down
            (1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0), #xz plane y = 0 cross
            (1, 2, 0), (1, -2, 0), (-1, 2, 0), (-1, -2, 0), 
            (0, 2, -1), (0, -2, -1), (0, -2, 1), (0, 2, 1),
            (0, 1, -1), (0, -1, -1), (0, -1, 1), (0, 1, 1),
        ]
    },
    "vertical_no_diagonal_small_build": {
        "prioritize_vertical": True,
        "allow_diagonal": False,
        "allow_large_build": True,
        "expected": [
            (1, 0, 0), (-1, 0, 0), (0, 0, 1), (0, 0, -1), 
            (0, 1, 0), (0, -1, 0),
            (1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0), 
            (1, 2, 0), (1, -2, 0), (-1, 2, 0), (-1, -2, 0), 
            (0, 1, -1), (0, -1, -1), (0, -1, 1), (0, 1, 1),
            (0, 2, -1), (0, -2, -1), (0, -2, 1), (0, 2, 1)
        ]
    }
}

# Function to validate outputs
def validate_neighbors(test_cases):
    for name, case in test_cases.items():
        result = set_neighbors(
            prioritize_vertical=case["prioritize_vertical"],
            allow_diagonal=case["allow_diagonal"],
            allow_large_build=case["allow_large_build"]
        )
        if sorted(result) == sorted(case["expected"]):
            print(f"{name}: PASSED")
        else:
            print(f"{name}: FAILED")
            print("Expected:", sorted(case["expected"]))
            print("Got:", sorted(result))

# Run validation
validate_neighbors(test_cases)

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
        return Cell(x, z, y, is_obs=is_obs)
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
    
def start_search_3d(grid, start, goal):
    start_cell = Cell(start[0], start[1], start[2])
    goal_cell = Cell(goal[0], goal[1], goal[2])
    visited = [[[False for _ in range(len(grid))] for _ in range(len(grid[0]))] for _ in range(grid[0][0])]
    queue = [start_cell]
    visited[start_cell[0]][start_cell[0][0]][start_cell[0][0][0]]
    steps = 0
    return goal_cell, visited, queue, steps