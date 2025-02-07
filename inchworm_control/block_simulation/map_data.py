from enum import Enum
import numpy as np
from config import *
import bfs_path_planning
import copy

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
    
def initialize_grid():
    """
    Initalize the empty 3D workspace such that all cells on the bottom layer are walkable, and the rest are not walkable.
    It additionally marks the block depots if there.

    Returns:
        grid [list]: A 3D list representing the initialized workspace where only the floor is walkable. (All z coordinates = 0).
    """
    # Initialize an empty 3D grid with all cells represented as NOT_WALKABLE
    grid = [[[GridStatus.NOT_WALKABLE.value for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)] 

    # Make the bottom layer (z = 0) WALKABLE
    for x in range(GRID_SIZE):
        for y in range(GRID_SIZE):
            grid[x][0][y] = GridStatus.WALKABLE.value
    
    grid = mark_block_depot(grid)
    return grid

def mark_block_depot(grid):
    """
    Initalize all block depots in grid. This is configured in config.py
    
    Args:
        grid [list]: A 3D list of the workspace
    Returns:
        grid [list]: A 3D list of the workspace with the supply depot.
    """
    for i in range(len(BD_LOCS)):
        x, z, y = BD_LOCS[i]
        if is_valid_position_3d(grid, BD_LOCS[i]):
            grid[x][z - 1][y] = GridStatus.SUPPLY_DEPOT.value
            grid[x][z][y] = GridStatus.WALKABLE.value
        else:
            raise ValueError(f"Error: depot location {BD_LOCS[i]} is out of bounds") 
    return grid
    
def update_grid_with_coord(grid, coord, status: GridStatus):
    """
    Update the 3D workspace being passed in such that the passed in structure becomes walkable and the space beneath it is not.

    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0), not (1), inchworm_path (-inchworm_id), 
                     incoming_block (2), & supply_depot (3). 
        coord (tuple): A tuple containing the (x, z, y) coordinates of the structure's 
                           position in the grid. This is a single block. 
        status (GridStatus): The GridStatus needed for coord.
    Returns:
        grid (list): An updated 3D list (grid) of the current map snapshot. 
    """ 
    # for structure in structures:        
    x, z, y = coord

    if is_valid_position_3d(grid, coord):
        grid[x][z][y] = GridStatus.WALKABLE.value #curr cell
        if z - 1 >= 0:
            grid[x][z-1][y] = status.value #cell below
    return grid

def set_inchworm_path_to_grid(grid, inchworm_path):
    """
    Sets the inchworm path on the grid.

    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0), not (1), inchworm_path (-inchworm_id), 
                     incoming_block (2), & supply_depot (3). 
    Returns:
        grid (list): An updated 3D list (grid) of the current map snapshot. 
    """ 
    print("set the path to grid")
    for step in range(len(inchworm_path)-1): 
        x, z, y = inchworm_path[step][0] # 1st index gets step, 2nd index gets coord and not holding_block
        grid[x][z][y] = GridStatus.INCHWORM_PATH.value
    return grid

def set_neighbors(allow_vertical=True, allow_vert_diagonal=True, allow_horz_diagonal=False, allow_alls_diagonal=False, allow_large_build=False):    
    """
    Sets the neighbors in an algorithm.

    Args:
        allow_vertical (boolean): . 
        allow_vert_diagonal (boolean): . 
        allow_horz_diagonal (boolean): . 
        allow_alls_diagonal (boolean): . 
        allow_large_build (boolean)
    Returns:
        neighbor_directions (list(tuple)): An updated 3D list (grid) where the floor & structure is walkable and the cell beneath the structure is not. 
    """ 
    base_neighbors = [(1, 0, 0), (-1, 0, 0), (0, 0, 1), (0, 0, -1)]
    vertical_neighbors = [(0, 1, 0), (0, -1, 0)]
    diagonal_vert_neighbors = [(1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0),
                               (0, 1, 1), (0, 1, -1), (0, -1, 1), (0, -1, -1)]
    diagonal_horz_neighbors = [(1, 0, 1), (1, 0, -1), (-1, 0, 1), (-1, 0, -1)]
    diagonal_alls_neighbors = [(1, 1, 1), (1, -1, 1), (-1, 1, 1), (-1, -1, 1),
                               (1, 1, -1), (1, -1, -1), (-1, 1, -1), (-1, -1, -1)]
    large_build_neighbors = [(1, 2, 0), (1, -2, 0), (-1, 2, 0), (-1, -2, 0),
                             (0, 2, -1), (0, -2, -1), (0, -2, 1), (0, 2, 1)]
    
    # combined neighbor_directions based on conditions
    neighbor_directions = base_neighbors
    
    if allow_vertical:
        neighbor_directions += vertical_neighbors
    if allow_vert_diagonal:
        neighbor_directions += diagonal_vert_neighbors
    if allow_horz_diagonal:
        neighbor_directions += diagonal_horz_neighbors
    if allow_alls_diagonal:
        neighbor_directions += diagonal_alls_neighbors
    if allow_large_build:
        neighbor_directions += large_build_neighbors
        
    return neighbor_directions

def reverse_path_3d(curr_cell, holding_block):
    """
    Reverse calculated path to go from start to goal.
    
    Args:
        curr_cell (Cell): The current position of an inchworm.
        holding_block (boolean): A boolean indicating if the inchworm is holding a block or not.
    Returns:
        path (list(tuple)): A reworked path found in a path planning algorithm.
    """
    path = []
    print("reversing the past")
    prev_holding_block = holding_block
    while curr_cell:
        if [curr_cell.x, curr_cell.z, curr_cell.y] == BD_LOCS[0]:
            curr_cell.z -= 1
            holding_block = True
        else:
            holding_block = prev_holding_block
        path.append(([curr_cell.x, curr_cell.z, curr_cell.y], holding_block))
        curr_cell = curr_cell.parent
    return path[::-1], len(path) - 1

def create_cell(grid, coords):
    """
    Create Cell data type from a coordinate.
    
    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0), not (1), inchworm_path (-inchworm_id), 
                     incoming_block (2), & supply_depot (3). 
        coords (tuple): A coordinate within a grid (x, z, y).
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
                     the corresponding cell is walkable (0), not (1), inchworm_path (-inchworm_id), 
                     incoming_block (2), & supply_depot (3). 
        coords (tuple): A tuple containing the (x, z, y) coordinates of a position. 
    Returns:
        (boolean): A boolean confirming or denying a coordinate. 
    """ 
    x, z, y = coords[0], coords[1], coords[2]
    if 0 <= x < len(grid) and 0 <= z < len(grid[0]) and 0 <= y < len(grid[0][0]):
        return True
    return False

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
                     the corresponding cell is walkable (0), not (1), inchworm_path (-inchworm_id), 
                     incoming_block (2), & supply_depot (3). 
        start (tuple): A tuple of the starting position in an inchworm's path.
        goal (tuple): A tuple of the goal position in an inchworm's path. 
    Returns:
        (boolean): A boolean confirming or denying a coordinate. 
    """
    start_cell = create_cell(grid, start)
    goal_cell = create_cell(grid, goal)
    return not (start_cell.is_obs and goal_cell.is_obs)
    
def start_search_3d(grid, start, goal):
    """
    Takes given grid, start position, and goal position of pathfinding and initializes a search.

    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0), not (1), inchworm_path (-inchworm_id), 
                     incoming_block (2), & supply_depot (3). 
        start (tuple): A tuple of the starting position in an inchworm's path.
        goal (tuple): A tuple of the goal position in an inchworm's path. 
    Returns:
        goal_cell (Cell): .
        visited (list(boolean)): .
        queue (list(Cell)): .
    """
    start_cell = create_cell(grid, start)
    goal_cell = create_cell(grid, goal)
    visited = [[[False for _ in range(len(grid))] for _ in range(len(grid[0]))] for _ in range(len(grid[0][0]))]
    queue = [start_cell]
    visited[start_cell.x][start_cell.z][start_cell.y] = True
    return goal_cell, visited, queue

def handle_multiple_block_depots():
    #TODO: how path planning is affected by the existence of multiple block depots 
    pass

def determine_helper_blocks(grid, path_start, path_end):
    #TODO
    # right now, this function only recalculates bfs by searching for vertical paths, for the case when the structure is something like a column
    # in the future, this function should be able to determine if a helper block is needed, and if so, where to place it
    path_coords = bfs_path_planning.find_path(grid, path_start, path_end, False)
    if path_coords == []:
        RuntimeError(f"Cannot find helper blocks for path.")
    else:
        return path_coords

def initiate_find_path(grid, path_start, path_end, curr_orientation, holding_block):
    """
    Converts the list of coordinates from a path planning algorithm into inchworm movesets

    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0), not (1), inchworm_path (-inchworm_id), 
                     incoming_block (2), & supply_depot (3). 
        path_start (tuple): The starting position of the path.
        path_end (tuple): The ending position of the path.
        curr_orientation (enum): N, E, S, or W 
        holding_block(bool): True if the inchworm is holding a block.
    Returns:
        grid: (list): An updated 3D list (grid) of the current map shapshot. 
    """ 
    path_coords = bfs_path_planning.find_path(grid, path_start, path_end, holding_block) # get the path

    # if no path was found, check to see if you'll need a helper block
    if path_coords == []:
        print(f"Checking for helper block now for start: {path_start}, goal: {path_end}")
        path_coords = determine_helper_blocks(grid, path_start, path_end)

    path_list = copy.deepcopy(path_coords[0])
    steps = []
    # goes through each coordinate in path and retrieves the step to go from the current location to the next location
    for i in range(len(path_list) - 1):
        current_coord = path_list[i][0]
        next_coord = path_list[i + 1][0]
            
        end_flag = bool(next_coord == path_end) # if it is done basically
        step_instructions, orientation = convert_coordinate_to_steps(grid, np.array(current_coord), np.array(next_coord), curr_orientation, holding_block, end_flag)
        steps.append(step_instructions)
        curr_orientation = orientation

    return path_coords, steps

def convert_coordinate_to_steps(grid, current_coord: tuple[int], next_coord: tuple[int], orientation, holding_block, end_flag):
    """
    Determines the steps needed to get from current_coord to next_coord by taking into account the
    direction of movement and new orientation of the inchworm's position in the 3D grid.
    
    Note: To make it more intuitive, think of it on the XY plane.
          Because the leading foot never changes, there's no way for the inchworm to ever step 
          diagonally backwards. Additionally, regular stepping forward and backward is just the 
          inchworm turning and doing a right or left step.

    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0), not (1), inchworm_path (-inchworm_id), 
                     incoming_block (2), & supply_depot (3). 
        current_coord (tuple): The current position (x, z, y).
        next_coord (tuple): The next position (x, z, y).
        orientation (InchwormOrientation): The current orientation.
        holding_block (boolean): Whether the inchworm is holding a block.
        end_flag (boolean): Indicates the end of path.

    Returns:
        step_instructions, new_orientation (tuple): A formatted step instruction and the new orientation.
    """
    movement_vector = np.subtract(next_coord, current_coord)
    magnitude = int(np.linalg.norm(movement_vector))
    
    if magnitude == 0:
        print("Warning: No movement required.")
        return [], "null"
    
    # If horizontally diagonal, needs to split into 2 sequential steps.
    if abs(movement_vector[0]) > 0 and abs(movement_vector[2]) > 0:  # Diagonal in x-y plane
        intermediate_coord = ((int(current_coord[0] + np.sign(movement_vector[0])), current_coord[1], current_coord[2]))
        
        # Process the two components
        step1, orientation1 = convert_coordinate_to_steps(grid, current_coord, intermediate_coord, orientation, holding_block, end_flag)
        step2, orientation2 = convert_coordinate_to_steps(grid, intermediate_coord, next_coord, orientation1, holding_block, end_flag)
        
        combined_steps = f"{step1}\n{step2}"
        return combined_steps, orientation2
    
    normalized_vector = tuple(int(coord // magnitude) if magnitude != 0 else 0 for coord in movement_vector)
    
    orientation_transforms = {
        InchwormOrientation.NORTH: lambda x, z, y: (x, z, y),  
        InchwormOrientation.SOUTH: lambda x, z, y: (-x, z, -y),
        InchwormOrientation.EAST: lambda x, z, y: (-y, z, x),  
        InchwormOrientation.WEST: lambda x, z, y: (y, z, -x),  
    }
    
    transform = orientation_transforms[orientation]
    # print(f"orientation: {orientation}")
    transformed_vector = transform(*normalized_vector)
    
    # Orientation here is based on NORTH.
    base_mappings = {
        # Horizontal movements
        ( 1,  0,  0): ("RIGHT"),
        (-1,  0,  0): ("LEFT"),
        ( 0,  0,  1): ("FORWARD"),
        ( 0,  0, -1): ("BACK"),
        ( 0,  1,  0): ("UP"),
        ( 0, -1,  0): ("DOWN"),
        # Vertically diagonal movements
        ( 1,  1,  0): ("UP_RIGHT"),
        (-1,  1,  0): ("UP_LEFT"),
        ( 0,  1,  1): ("UP_FORWARD"),
        ( 0,  1, -1): ("UP_BACK"),
        ( 1, -1,  0): ("DOWN_RIGHT"),
        (-1, -1,  0): ("DOWN_LEFT"),
        ( 0, -1,  1): ("DOWN_FORWARD"),
        ( 0, -1, -1): ("DOWN_BACK")
    }

    if transformed_vector in base_mappings:
        step_instructions = base_mappings[transformed_vector]

        if magnitude > 1:
            if "UP" in step_instructions or "DOWN" in step_instructions:
                verticality = step_instructions.split("_")[0]
                horizontality = step_instructions.split("_")[-1]
                step_instructions = f"{verticality}_{magnitude}_{horizontality}"
            else:
                horizontality = step_instructions
                step_instructions = f"{magnitude}_{horizontality}"
                
        new_orientation = get_orientation(step_instructions, orientation)
        
        #TODO: handle any block depot'
        if holding_block:
            step_instructions = f"{step_instructions}_BLOCK"
            
        if (next_coord == [BD_LOC1[0], BD_LOC1[1]-1, BD_LOC1[2]]).all():
            return f"GRAB_{step_instructions}", new_orientation
        elif holding_block & end_flag:
            return f"PLACE_{step_instructions}", new_orientation
        else:
            return f"STEP_{step_instructions}", new_orientation

    # Handle undefined or unexpected movements
    print(f"Warning: Undefined movement vector {movement_vector} between {current_coord} and {next_coord}")
    return ["UNKNOWN_STEP"], "null"

def get_orientation(movement: str, orientation: InchwormOrientation):
    if "RIGHT" in movement: 
        return orientation.rotate(1)
    elif "LEFT" in movement: 
        return orientation.rotate(-1)
    elif "BACK" in movement: 
        return orientation.rotate(2)
    else:
        return orientation    
    