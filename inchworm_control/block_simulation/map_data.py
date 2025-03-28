from enum import IntEnum
import numpy as np
from config import *
import bfs_path_planning
from colorama import Fore, init
init(autoreset=True)
from inchworm_data import Inchworm

class GridStatus(IntEnum):
    WALKABLE = 0
    NOT_WALKABLE = 1
    INCHWORM_PATH = -1
    INCOMING_BLOCK = 2
    SUPPLY_DEPOT = 3
    
    @classmethod
    def inchworm_path(cls, iw_id):
        """Generate an inchworm path status dynamically using a negative ID."""
        if iw_id in Inchworm.inchworm_list:
            return -iw_id
        raise ValueError(f"Inchworm ID {iw_id} is not valid.")

    @classmethod
    def is_inchworm_path(cls, value):
        """Check if the given value represents an inchworm path."""
        return isinstance(value, int) and value < 0 and abs(value) in Inchworm.inchworm_list

    @classmethod
    def which_inchworm(cls, value):
        """Return the inchworm ID if the value is an inchworm path, otherwise None."""
        if cls.is_inchworm_path(value):
            return abs(value)
        return None

    @classmethod
    def from_value(cls, value):
        """Determine the GridStatus type, automatically recognizing inchworm paths."""
        if value in cls._value2member_map_:
            return cls(value)
        elif cls.is_inchworm_path(value):
            return value  # Return as valid inchworm path
        raise ValueError(f"Invalid GridStatus value: {value}")

class Cell:
    def __init__(self, x: int, y: int, z: int, is_obs: bool = False, g = 0, h = 0): 
        """
        Initialize the Cell class. It represents a single cell (location) within the map or grid, and is used for path planning purposes. 
        
        Args:
            x (int): x location of the cell.
            y (int): y location of the cell.
            z (int): z location of the cell.
            is_obs (bool): True if this cell is occupied, not walkable. False if walkable. 
            g (int): The cost to reach this cell. 
            h (int): Evaluated additional heuristic cost to reach this cell. 
        """
        self.x = x
        self.y = y
        self.z = z
        self.is_obs = is_obs
        self.g = g
        self.h = h 
        self.f = g + h # total cost
        self.parent = None # The parent may later be set as another Cell object. 

    def __lt__(self, other): 
        """
        Less than. Returns true is this Cell object's total cost is less than the total cost on the inputted Cell (other). 
        This is used for cell comparison for the priority queue (the frontier). 
        
        Args:
            other (Cell): Another Cell object. 
        """
        return self.f < other.f # cell comparing for priority queue (the frontier)
    
def initialize_grid():
    """
    Initalize the empty 3D workspace such that all cells on the bottom layer are walkable, and the rest are not walkable.
    It additionally marks the block depots if there.

    Returns:
        grid [list]: A 3D list representing the initialized workspace where only the floor is walkable. (All z coordinates = 0).
    """
    # Initialize an empty 3D grid with all cells represented as NOT_WALKABLE
    grid = [[[GridStatus.NOT_WALKABLE.value for z in range(GRID_HEIGHT)] for y in range(GRID_SIZE)] for x in range(GRID_SIZE)] 

    # Make the bottom layer (z = 0) WALKABLE
    for x in range(len(grid)):
        for y in range(len(grid[0])):
            grid[x][y][0] = GridStatus.WALKABLE.value
    
    grid = mark_depot_and_seed(grid)
    return grid

def mark_depot_and_seed(grid):
    """
    Initalize all block depots and the seed block in grid. This is configured in config.py
    
    Args:
        grid [list]: A 3D list of the workspace
    Returns:
        grid [list]: A 3D list of the workspace with the supply depot.
    """
    for i in range(len(BD_LOCS)):
        x, y, z = BD_LOCS[i]
        if is_valid_position_3d(grid, BD_LOCS[i]):
            grid[x][y][z - 1] = GridStatus.SUPPLY_DEPOT.value
            grid[x][y][z] = GridStatus.WALKABLE.value
        else:
            raise ValueError(f"Error: depot location {BD_LOCS[i]} is out of bounds") 
        
    x, y, z = SEED_BK
    grid[x][y][z - 1] = GridStatus.NOT_WALKABLE.value
    grid[x][y][z] = GridStatus.WALKABLE.value
    return grid
    
def update_grid_status(grid, coord, status: GridStatus=GridStatus.NOT_WALKABLE):
    """
    Update the 3D workspace being passed in such that the passed in structure becomes walkable and the space beneath it is not.

    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0), not (1), inchworm_path (-inchworm_id), 
                     incoming_block (2), & supply_depot (3). 
        coord (tuple): A tuple containing the (x, y, z) coordinates of the structure's 
                           position in the grid. This is a single block. 
        status (GridStatus): The GridStatus needed for coord.
    Returns:
        grid (list): An updated 3D list (grid) of the current map snapshot. 
    """ 
    # for structure in structures:        
    x, y, z = coord

    if is_valid_position_3d(grid, coord):
        if status == GridStatus.INCOMING_BLOCK.value:
            grid[x][y][z] = GridStatus.INCOMING_BLOCK.value
            if z - 1 >= 0:
                grid[x][y][z - 1] = GridStatus.NOT_WALKABLE.value #cell below
        elif status == GridStatus.SUPPLY_DEPOT.value:
            grid[x][y][z] = GridStatus.WALKABLE.value
            if z - 1 >= 0:
                grid[x][y][z - 1] = GridStatus.SUPPLY_DEPOT.value #cell below
        elif status < 0: 
            grid[x][y][z] = status
        else:
            grid[x][y][z] = GridStatus.WALKABLE.value #curr cell
            if z - 1 >= 0:
                grid[x][y][z-1] = status #cell below

    return grid

def set_inchworm_path_to_grid(grid, inchworm_path, iw_id):
    """
    Sets the inchworm path on the grid.

    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0), not (1), inchworm_path (-inchworm_id), 
                     incoming_block (2), & supply_depot (3). 
    Returns:
        grid (list): An updated 3D list (grid) of the current map snapshot. 
    """ 
    for step in range(len(inchworm_path)-1): 
        x, y, z = inchworm_path[step]
        grid[x][y][z] = iw_id * GridStatus.INCHWORM_PATH.value
    return grid

def rm_inchworm_path_from_grid(grid, inchworm_path, iw_id):
    """
    Remove the inchworm path from the grid.

    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0), not (1), inchworm_path (-inchworm_id), 
                     incoming_block (2), & supply_depot (3). 
    Returns:
        grid (list): An updated 3D list (grid) of the current map snapshot. 
    """ 
    for step in range(len(inchworm_path)-1): 
        x, y, z = inchworm_path[step] 
        if grid[x][y][z] == iw_id * GridStatus.INCHWORM_PATH.value:
            if [x, y, z + 1] == BD_1_LOC: 
                grid[x][y][z] = GridStatus.SUPPLY_DEPOT.value
            else:
                if grid[x][y][z + 1] == GridStatus.WALKABLE.value:
                    grid[x][y][z] = GridStatus.NOT_WALKABLE.value
                else:
                    grid[x][y][z] = GridStatus.WALKABLE.value
    return grid


def set_neighbors(allow_adjacent=True, allow_vertical=True, allow_vert_diagonal=True, allow_horz_diagonal=False, allow_alls_diagonal=False, allow_large_build=False):    
    """
    Sets the neighbors for use in (search) algorithms.

    Args:
        allow_vertical (boolean): True to allow cells directly above and below the current cell. 
        allow_vert_diagonal (boolean): True to add neighbors adjacent in the xy plane, but within 1 block up/down. 
        allow_horz_diagonal (boolean): True to add neighbors diagonal in the xy plane. 
        allow_alls_diagonal (boolean): True to add neighbors diagonal in the xy plane, but within 1 block up/down. 
        allow_large_build (boolean): True to allow neighbors within 1 block horizontally, but +-3 vertically. This being true allows the inchworm to path
            plan to place blocks up to 3 blocks tall. 
    Returns:
        neighbor_directions (list(tuple)): A list of directions to nearby cells. 
    """ 
    base_neighbors = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)] # Adjacent cells in xy plane, excluding diagonals. 
    vertical_neighbors = [(0, 0, 1), (0, 0, -1)]
    diagonal_vert_neighbors = [(1, 0, 1), (1, 0, -1), (-1, 0, 1), (-1, 0, -1),
                               (0, 1, 1), (0, 1, -1), (0, -1, 1), (0, -1, -1)]
    diagonal_horz_neighbors = [(1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0)]
    diagonal_alls_neighbors = [(1, 1, 1), (1, 1, -1), (-1, 1, 1), (-1, 1, -1),
                               (1, -1, 1), (1, -1, -1), (-1, -1, 1), (-1, -1, -1)]
    large_build_neighbors = [(1, 0, 2), (1, 0, -2), (-1, 0, 2), (-1, 0, -2),
                             (0, -1, 2), (0, -1, -2), (0, 1, -2), (0, 1, 2),
                             (1, 0, 3), (1, 0, -3), (-1, 0, 3), (-1, 0, -3),
                             (0, -1, 3), (0, -1, -3), (0, 1, -3), (0, 1, 3)]
    
    # Combine neighbor_directions based on conditions
    neighbor_directions = []
    if allow_adjacent:
        neighbor_directions += base_neighbors
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

def reverse_path_3d(curr_cell: Cell, holding_block: bool) -> list[int]:
    """
    Reverse calculated path to go from start to goal.
    
    Args:
        curr_cell (Cell): The current position of an inchworm.
        holding_block (bool): A boolean indicating if the inchworm is holding a block or not.
    Returns:
        path (list[int]): A reworked path found in a path planning algorithm of coord and holding_block.
    """
    path = []
    prev_holding_block = holding_block
    while curr_cell:
        if [curr_cell.x, curr_cell.y, curr_cell.z] == BD_LOCS[0]:
            curr_cell.z -= 1
            holding_block = True
        else:
            holding_block = prev_holding_block
        path.append([curr_cell.x, curr_cell.y, curr_cell.z])
        curr_cell = curr_cell.parent
    return path[::-1]

def create_cell(grid, coords):
    """
    Create Cell data type from a coordinate.
    
    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0), not (1), inchworm_path (-inchworm_id), 
                     incoming_block (2), & supply_depot (3). 
        coords (tuple): A coordinate within a grid (x, y, z).
    Returns:
        cell (Cell): The corresponding Cell of the given coordinate.
    """
    x, y, z = coords[0], coords[1], coords[2]
    if is_valid_position_3d(grid, [x, y, z]):
        new_cell = Cell(x, y, z)
        
        if grid[x][y][z] == GridStatus.WALKABLE.value:  
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
        coords (tuple): A tuple containing the (x, y, z) coordinates of a position. 
    Returns:
        (boolean): A boolean confirming or denying a coordinate. 
    """ 
    x, y, z = coords[0], coords[1], coords[2]
    if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and 0 <= z < len(grid[0][0]):
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
        frontier (list(Cell)): .
    """
    start_cell = create_cell(grid, start)
    goal_cell = create_cell(grid, goal)
    visited = [[[False for z in range(len(grid[0][0]))] for y in range(len(grid[0]))] for x in range(len(grid))]
    frontier = [start_cell]
    visited[start_cell.x][start_cell.y][start_cell.z] = True
    return goal_cell, visited, frontier

def handle_multiple_block_depots():
    #TODO: how path planning is affected by the existence of multiple block depots 
    pass

def determine_helper_blocks(grid, path_start, path_end, iw_id):
    #TODO
    # right now, this function only recalculates bfs by searching for vertical paths, for the case when the structure is something like a column
    # in the future, this function should be able to determine if a helper block is needed, and if so, where to place it
    path_coords = []
    return path_coords 

    # path_coords = bfs_path_planning.find_path(grid, path_start, path_end, iw_id, False)
    # if path_coords == []:
    #     RuntimeError(f"Cannot find helper blocks for path.")
    # else:
    #     return path_coords

def initiate_find_path(grid, path_start, path_end, curr_orientation: InchwormOrientation, holding_block: bool, iw_id: int):
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
    c_space_grid = buffer_iw_paths(grid, iw_id)
    path_coords = bfs_path_planning.find_path(c_space_grid, path_start, path_end, iw_id, holding_block) # get the path

    # if no path was found, check to see if you'll need a helper block
    if path_coords == []:
        print(Fore.MAGENTA + f"Checking for helper block now for start: {path_start}, goal: {path_end}")
        path_coords = determine_helper_blocks(c_space_grid, path_start, path_end, iw_id)

    steps = []
    # goes through each coordinate in path and retrieves the step to go from the current location to the next location
    if path_coords:
        for i in range(len(path_coords) - 1):
            current_coord = path_coords[i]
            next_coord = path_coords[i + 1]
                
            end_flag = bool(next_coord == path_end) # if it is done basically
            step_instructions, orientation = get_direction_instruction(current_coord, next_coord, curr_orientation, holding_block, end_flag)
            steps.extend(step_instructions)
            curr_orientation = orientation

    return path_coords, steps, curr_orientation

def convert_coordinate_to_steps(grid, current_coord, next_coord, orientation: InchwormOrientation, holding_block: bool, end_flag):
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
        current_coord (tuple): The current position (x, y, z).
        next_coord (tuple): The next position (x, y, z).
        orientation (InchwormOrientation): The current orientation.
        holding_block (boolean): Whether the inchworm is holding a block.
        end_flag (boolean): Indicates the end of path.

    Returns:
        step_instructions, new_orientation (tuple): A formatted step instruction and the new orientation.
    """    
    
    movement_vector = np.array(next_coord) - np.array(current_coord)
    magnitude = (np.linalg.norm(movement_vector))
    
    if magnitude == 0:
        print(Fore.MAGENTA + "Warning: No movement required.")
        return [], "null"
    
    normalized_vector = ((coord // magnitude) if magnitude != 0 else 0 for coord in movement_vector)
    is_multi_axis_combinable = np.count_nonzero(normalized_vector) == 2 or (np.count_nonzero(normalized_vector) == 3 and end_flag) # combine directions when at end or when only 2
    
    # If there is only one axis changing
    if magnitude == 1 and np.count_nonzero(movement_vector) == 1:
        return get_direction_instruction(current_coord, next_coord, orientation, holding_block, end_flag)
    
    all_instructions = []
    updated_orientation = orientation
    if  is_multi_axis_combinable: # separate into single-axis steps
        intermediate_coord = current_coord
        for axis in range(3):
            # for _ in range(abs(movement_vector[axis])): # TODO: double check if necessary
            separated_vector = [0, 0, 0]
            separated_vector[axis] = (np.sign(movement_vector[axis]))
            next_step_coord = (np.add(intermediate_coord, separated_vector))
            
            sub_instructions, sub_orientation = convert_coordinate_to_steps(
                grid, intermediate_coord, next_step_coord, updated_orientation, holding_block, end_flag
            )
                
            intermediate_coord = next_step_coord
            updated_orientation = sub_orientation
            all_instructions.extend(sub_instructions) 
        return all_instructions, sub_orientation
    else:
        return get_direction_instruction(current_coord, next_coord, orientation, holding_block, end_flag)

    # Handle undefined or unexpected movements
    print(Fore.MAGENTA + f"Warning: Undefined movement vector {movement_vector} between {current_coord} and {next_coord}")
    return ["UNKNOWN_STEP"], "null"

def get_direction_instruction(current_coord, next_coord, orientation, holding_block, end_flag):
    movement_vector = np.subtract(next_coord, current_coord)
    
    direction_mappings = {
        0: {1: "RIGHT",     -1: "LEFT"},    #X
        1: {1: "FORWARD",   -1: "BACK"},    #Y
        2: {1: "UP",        -1: "DOWN" }    #Z
    }
    
    orientation_transforms = {
        InchwormOrientation.NORTH: lambda x, y, z: [ x,  y, z],  
        InchwormOrientation.SOUTH: lambda x, y, z: [-x, -y, z],
        InchwormOrientation.EAST:  lambda x, y, z: [-y,  x, z],  
        InchwormOrientation.WEST:  lambda x, y, z: [ y, -x, z],  
    }
    
    # create instruction
    instructions = [] 
    transform = orientation_transforms[orientation]
    transformed_vector = transform(*movement_vector)

    for axis in [2, 1, 0]: # prioritize Z, then Y, then X according to direction_mappings
        # If there is change on this axis: 
        if transformed_vector[axis] != 0:
            coord_change = transformed_vector[axis]
            direction_step = direction_mappings[axis][int(np.sign(coord_change))]
            instructions.append(direction_step)
            if coord_change > 1 or coord_change < -1:
                instructions.append(str(coord_change))
            # print(f"coord_change: {coord_change} for {axis} axis for Transition between {current_coord} & {next_coord} while {orientation.name}. resulting step: {direction_step}")
    # Determine the orientation based on the recent axis change
    new_orientation = get_orientation(direction_step, orientation)
    all_instructions = "_".join(instructions)
    # print(f"transformed vector: {transformed_vector}")
    # print(f"orientation: {updated_orientation}")
    
    #TODO: handle any block depot'
    if holding_block:
        all_instructions = f"{all_instructions}_BLOCK"
    
    # for bd_loc in BD_LOCS:
    if (next_coord == [BD_1_LOC[0], BD_1_LOC[1], BD_1_LOC[2]-1]):
        all_instructions = f"GRAB_{all_instructions}"
    elif holding_block and end_flag:
        all_instructions = f"PLACE_{all_instructions}"
    else:
        all_instructions = f"STEP_{all_instructions}"
    # print(f"Transition between {current_coord} & {next_coord} while {orientation.name} --> {all_instructions} going {new_orientation.name}")
    return [all_instructions], new_orientation

def get_orientation(movement: str, orientation: InchwormOrientation):
    if "RIGHT" in movement: 
        return orientation.rotate(1)
    elif "LEFT" in movement: 
        return orientation.rotate(-1)
    elif "BACK" in movement: 
        return orientation.rotate(2)
    else:
        return orientation
    
def buffer_iw_paths(grid, iw_id: int):
    """
    To avoid collisions, buffers the inchworm paths of *other* IWs. 
    Args: 
        iw_id (int): the ID of the IW that is trying to path plan around the other IWs
    Returns: 
        grid: the 3D list map, now with a bunch of extra cells marked as IW paths
    """
    # if on the map there is another iw path, have its neighbors also turn into iw_path
    
    # check all of grid for inchworm paths
    buffer_list = [] # list of coords that need to be updated for buffering
    neighbor_directions = set_neighbors()
    path_count = []

    # Make the bottom layer (z = 0) WALKABLE
    for x in range(len(grid)):
        for y in range(len(grid[0])):
            for z in range(len(grid[0][0])):
                cell_status = grid[x][y][z]   
                
                if ((cell_status != iw_id * GridStatus.INCHWORM_PATH.value) and cell_status < 0 and [x, y, z] != SEED_BK and [x, y, z] != BD_1_LOC): # is some inchworm path, but not its own
                    for dx, dy, dz in neighbor_directions:
                        nx, ny, nz = x + dx, y + dy, z + dz
                        if (is_valid_position_3d(grid, [nx, ny, nz]) 
                            and not (is_neighbor_of_cell(grid, [nx, ny, nz], SEED_BK, neighbor_directions) or is_neighbor_of_cell(grid, [nx, ny, nz], BD_1_LOC, neighbor_directions))):
                            n_status = grid[nx][ny][nz]
                            if (n_status == GridStatus.WALKABLE.value or n_status == GridStatus.INCOMING_BLOCK.value):
                                path_count.append((nx, ny, nz, cell_status))
                    if (len(path_count) > 2):
                        # print(Fore.MAGENTA + f"Path count: ", path_count)
                        for new_info in path_count:
                            buffer_list.append(new_info)
                        path_count = []
    
    # print(Fore.MAGENTA + f"buffer list vals: ", buffer_list)
    for (nx, ny, nz, new_status) in buffer_list:
        grid = update_grid_status(grid, [nx, ny, nz], new_status)
    # print(Fore.MAGENTA + f"grid: ", grid)
    
    grid = update_grid_status(grid, SEED_BK)
    grid = update_grid_status(grid, BD_1_LOC, GridStatus.SUPPLY_DEPOT.value)
    
    return grid

def is_neighbor_of_cell(grid: list, coord_compare: list, og_coord: list, neighbor_directions: list):
    """
    Returns true if a potential buffer cell is within the "off limits zone" of another block. 
    Args:
        grid (list)
        coord_compare (list(list)): coordinate of a potential path buffer cell. 
        og_coord: coordinate of a cell that can't be trapped by the buffer 
        neighbor_directions: 
    """
    neighbors = []
    if not is_valid_position_3d(grid, coord_compare) and not is_valid_position_3d(grid, og_coord):
        return False
    
    if (coord_compare == og_coord):
        return True
    
    gx, gy, gz = og_coord
    for dx, dy, dz in neighbor_directions:
        nx, ny, nz = gx + dx, gy + dy, gz + dz
        # print(f"is_neighbor_of_cell: Trying to see if {coord_compare} is neighbor of {og_coord} at {nx, ny, nz} ")
        neighbors.append([nx, ny, nz])
        if is_valid_position_3d(grid, [nx, ny, nz]) and coord_compare == [nx, ny, nz]:
            return True
    return False

def create_placement_pivot(): 
    """
    Modify the path to add a little step to left or right before placing a block, since the IW needs to be right next to the structure to reach higher locations. 
    """
    pass