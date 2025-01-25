from enum import Enum
import copy
import numpy as np
from config import *
from path_conversion import *

class GridStatus(Enum):
    WALKABLE = 0
    NOT_WALKABLE = 1
    INCHWORM_PATH = -1
    INCOMING_BLOCK = 2
    SUPPLY_DEPOT = 3

class Cell:
    def __init__(self, x: int, z: int, y: int, is_obs: bool = False, g = 0, h = 0): 
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
        This is used for cell comparison for the priority queue. 
        
        Args:
            other (Cell): Another Cell object. 
        """
        return self.f < other.f # cell comparing for priority queue
    
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
        if is_valid_position_3d(grid, BD_LOCS[i]):
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
    x, z, y = structure

    if is_valid_position_3d(grid, structure):
        grid[x][z][y] = GridStatus.WALKABLE.value #curr cell
        if z - 1 >= 0:
            grid[x][z-1][y] = GridStatus.NOT_WALKABLE.value #cell below
    return grid 

def set_inchworm_path_to_grid(grid, inchworm_path):
    """
    Sets the possible walkable and the space beneath it is not.

    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0) or not (1). 
    Returns:
        grid (list): An updated 3D list (grid) where the floor & structure is walkable and the cell beneath the structure is not. 
    """ 
    for x, z, y in inchworm_path:
        grid[x][z][y] = GridStatus.INCHWORM_PATH.value
    
    return grid

def set_neighbors(prioritize_vertical, allow_diagonal=True, allow_large_build=False):    
    """
    Sets the neighbors in an algorithm.

    Args:
        prioritize_vertical (boolean): . 
        allow_diagonal (boolean): . 
        allow_large_build (boolean)
    Returns:
        neighbor_directions (list(tuple)): An updated 3D list (grid) where the floor & structure is walkable and the cell beneath the structure is not. 
    """ 
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

def rework_path_3d(curr_cell, is_holding_block):
    """
    Reverse calculated path to go from start to goal.
    
    Args:
        curr_cell (Cell): The current position of an inchworm.
        is_holding_block (boolean): A boolean indicating if the inchworm is holding a block or not.
    Returns:
        path (list(tuple)): A reworked path found in a path planning algorithm.
        steps (int): The number of steps in a path.
    """
    path = []
    while curr_cell:
        path.append(([curr_cell.x, curr_cell.z, curr_cell.y], is_holding_block))
        print("reversed path")
        curr_cell = curr_cell.parent
    return path[::-1], len(path) - 1

def create_cell(grid, coords):
    """
    Create Cell data type from a coordinate.
    
    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0) or not (1).
        coords (tuple): A coordinate within a grid.
    Returns:
        cell (Cell): The corresponding Cell of the given coordinate.
    """
    x, z, y = coords[0], coords[1], coords[2]
    if is_valid_position_3d(grid, [x, z, y]):
        new_cell = Cell(x, z, y)
        
        if grid[x][z][y] == GridStatus.WALKABLE.value:  
            new_cell.is_obs = False
        else:
            new_cell.is_obs = True
            
        return new_cell
    raise ValueError(f"Error: Invalid position at {coords}.")
    
def is_valid_position_3d(grid, coords):
    """
    Validates a coordinate to see if it is in bounds.

    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0) or not (1). 
        coords (tuple): A tuple containing the (x, z, y) coordinates of a position. 
    Returns:
        (boolean): A boolean confirming or denying a coordinate. 
    """ 
    x, z, y = coords[0], coords[1], coords[2]
    if 0 <= x < len(grid) and 0 <= z < len(grid[0]) and 0 <= y < len(grid[0][0]):
        return True
    raise ValueError(f"Error: Invalid position at {coords}.") 

def is_goal_reached_3d(curr_cell, goal_cell):
    """
    Validates if the current cell is the goal cell.

    Args:
        curr_cell (Cell): A Cell of the current position of an inchworm.  
        goal_cell (Cell): A Cell of the goal position of an inchworm's path.  
    Returns:
        (boolean): A boolean confirming or denying if the current cell is the goal cell. 
    """
    return (curr_cell.x == goal_cell.x and 
            curr_cell.y == goal_cell.y and 
            curr_cell.z == goal_cell.z)

def is_valid_start_goal_3d(grid, start, goal):
    """
    Validates a coordinate to see if it is in bounds.

    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0) or not (1). 
        start (tuple): A tuple of the starting position in an inchworm's path.
        goal (tuple): A tuple of the goal position in an inchworm's path. 
    Returns:
        (boolean): A boolean confirming or denying a coordinate. 
    """
    start_cell = create_cell(grid, start)
    goal_cell = create_cell(grid, goal)
    return start_cell.is_obs and goal_cell.is_obs
    
def start_search_3d(grid, start, goal):
    """
    Takes given grid, start position, and goal position of pathfinding and initializes a search.

    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0) or not (1). 
        start (tuple): A tuple of the starting position in an inchworm's path.
        goal (tuple): A tuple of the goal position in an inchworm's path. 
    Returns:
        goal_cell (Cell): .
        visited (list(boolean)): .
        queue (list(Cell)): .
        steps (int): The number of steps in a path.
    """
    start_cell = create_cell(grid, start)
    goal_cell = create_cell(grid, goal)
    visited = [[[False for _ in range(len(grid))] for _ in range(len(grid[0]))] for _ in range(len(grid[0][0]))]
    queue = [start_cell]
    visited[start_cell.x][start_cell.z][start_cell.y] = True
    steps = 0
    return goal_cell, visited, queue, steps

def handle_block_depot():
    #TODO
    pass

def convert_coordinate_to_steps(current_coord, next_coord, orientation, is_holding_block, end_flag):
    """
    Determines the steps needed to get from current_coord to next_coord by taking into account the
    direction of movement and new orientation of the inchworm's position in the 3D grid.
    
    Note: To make it more intuitive, think of it on the XY plane.
          Because the leading foot never changes, there's no way for the inchworm to ever step 
          diagonally backwards. Additionally, regular stepping forward and backward is just the 
          inchworm turning and doing a right or left step.

    Args:
        current_coord (tuple): The current position (x, z, y).
        next_coord (tuple): The next position (x, z, y).
        orientation (InchwormOrientation): The current orientation.
        is_holding_block (boolean): Whether the inchworm is holding a block.
        end_flag (boolean): Indicates the end of path.

    Returns:
        tuple: A formatted step name and the new orientation.
    """
    movement_vector = np.subtract(next_coord, current_coord)
    magnitude = int(np.linalg.norm(movement_vector))
    
    if magnitude == 0:
        print("Warning: No movement required.")
        return [], "null"
    
    normalized_vector = tuple(int(coord // magnitude) if magnitude != 0 else 0 for coord in movement_vector)

    # Orientation here is based on NORTH.
    base_mappings = {
        ( 1,  0,  0): ("RIGHT", InchwormOrientation.EAST),
        (-1,  0,  0): ("LEFT", InchwormOrientation.WEST),
        ( 0,  0,  1): ("RIGHT", InchwormOrientation.NORTH),
        ( 0,  0, -1): ("LEFT", InchwormOrientation.SOUTH),
        ( 0,  1,  0): ("UP", orientation),
        ( 0, -1,  0): ("DOWN", orientation),
        # Diagonal movements
        ( 1,  1,  0): ("UP_RIGHT", InchwormOrientation.EAST),
        (-1,  1,  0): ("UP_LEFT", InchwormOrientation.WEST),
        ( 0,  1,  1): ("UP_FORWARD", InchwormOrientation.NORTH),
        ( 0,  1, -1): ("UP_BACK", InchwormOrientation.SOUTH),
        ( 1, -1,  0): ("DOWN_RIGHT", InchwormOrientation.EAST),
        (-1, -1,  0): ("DOWN_LEFT", InchwormOrientation.WEST),
        ( 0, -1,  1): ("DOWN_FORWARD", InchwormOrientation.NORTH),
        ( 0, -1, -1): ("DOWN_BACK", InchwormOrientation.SOUTH),
    }
    
    orientation_transforms = {
        "NORTH": lambda x, z, y: (x, z, y),  
        "SOUTH": lambda x, z, y: (-x, z, -y),
        "EAST": lambda x, z, y: (y, z, -x),  
        "WEST": lambda x, z, y: (-y, z, x),  
    }
    
    transform = orientation_transforms[orientation]
    transformed_vector = transform(*normalized_vector)

    if transformed_vector in base_mappings:
        step_name, new_orientation = base_mappings[transformed_vector]

        if magnitude > 1:
            if "UP" in step_name or "DOWN" in step_name:
                verticality = step_name.split("_")[0]
                horizontality = step_name.split("_")[-1]
                step_name = f"{verticality}_{magnitude}_{horizontality}"
            else:
                horizontality = step_name
                step_name = f"{magnitude}_{horizontality}"
        
        if next_coord == BD_LOC1:
            return "GRAB_{step_name}", new_orientation
        elif is_holding_block & end_flag:
            return "PLACE_{step_name}", new_orientation
        else:
            return "STEP_{step_name}", new_orientation

    # Handle undefined or unexpected movements
    print(f"Warning: Undefined movement vector {movement_vector} between {current_coord} and {next_coord}")
    return ["UNKNOWN_STEP"], "null"