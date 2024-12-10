import copy
from enum import Enum
from path_planning import *
from config import BD_LOC1, CURRENT_LOC, CURRENT_ORIENTATION, InchwormOrientation, DEMO

# converts the list of coords from bfs to a list of inchworm movements
# returns the list of path coordinates and the list of steps from start to end
def convert_path_coords_to_steps(grid, path_start, path_end):
    """
    Converts the list of coordinates from a pathplanning algorithm into inchworm movesets

    Args:
        grid (list): A 3D list representing the workspace, where each element indicates whether
                     the corresponding cell is walkable (0) or not (1). 
        path_start (tuple): .
        path_end (tuple): .
    Returns:
        grid: (list): An updated 3D list (grid) where the floor & structure is walkable and the cell beneath the structure is not. 
    """ 
    global CURRENT_LOC, CURRENT_ORIENTATION

    # get the path
    path_coords, num_steps = bfs_3d(grid, path_start, path_end)

    # if no path was found, check to see if you'll need a helper block
    if num_steps == -1:
        grid, path_coords, num_steps = determine_helper_blocks(grid, path_start, path_end)

    # if the start is the Block Depot, it is holding a block
    holding_block = False
    if(path_start == BD_LOC1):
        holding_block = True

    steps = []
    # goes through each coordinate in path and retrieves the step to go from the current location to the next location
    for i in range(len(path_coords) - 1):
        current_coord = path_coords[i][0]
        next_coord = path_coords[i + 1][0] 

        # offset to handle the inchworm's position when it's on the block depot
        if current_coord == BD_LOC1:
            x, z, y = current_coord
            current_coord = [x, z-1, y]

        # get the movement direction and the new orientation
        movement_direction, new_orientation = get_direction(current_coord, next_coord)

        # if the next coord is the block depot, the next step should be a grabbing step
        if next_coord == BD_LOC1:
            steps.append(("GRAB_{movement_direction}", holding_block))
            
        # if the next coordinate is the goal(and not BD), then we need to place the block
        elif next_coord == path_end:
            steps.append(("PLACE_{movement_direction}", holding_block))
            # once it places the block, the currnt location will be on top of where the block is
            x, z, y = next_coord
            next_coord = [x, z+1, y]
        else:
            # general case
            steps.append((update_steps(movement_direction), holding_block))

        # update the current location and orientation
        CURRENT_LOC = next_coord
        if(new_orientation != "null"):
            CURRENT_ORIENTATION = new_orientation

    return path_coords, steps

# given the movement direction, returns the step to take
def update_steps(movement_direction):
    step_mappings = {
        "NORTH": {
            "FORWARD": "STEP_FORWARD", "BACK": "STEP_BACK", "LEFT": "STEP_LEFT", "RIGHT": "STEP_RIGHT",
            "UP": "CLIMB_UP", "DOWN": "CLIMB_DOWN", "DIAGONAL_UP_RIGHT": "STEP_UP_RIGHT", 
            "DIAGONAL_UP_LEFT": "STEP_UP_LEFT", "DIAGONAL_UP_FORWARD": "STEP_UP",
            "DIAGONAL_DOWN_RIGHT": "STEP_DOWN_RIGHT", "DIAGONAL_DOWN_LEFT": "STEP_DOWN_LEFT",
            "DIAGONAL_DOWN_FORWARD": "STEP_DOWN", "DIAGONAL_UP_2_RIGHT": "STEP_UP_2_RIGHT", 
            "DIAGONAL_UP_2_LEFT": "STEP_UP_2_LEFT", "DIAGONAL_UP_2_FORWARD": "STEP_UP_2",
            "DIAGONAL_DOWN_2_RIGHT": "STEP_DOWN_2_RIGHT", "DIAGONAL_DOWN_2_LEFT": "STEP_DOWN_2_LEFT",
            "DIAGONAL_DOWN_2_FORWARD": "STEP_DOWN_2"
            # movements that are iffy
            # "DIAGONAL_UP_BACK": STEP_BACK, "DIAGONAL_DOWN_BACK": STEP_BACK
            # "DIAGONAL_UP_2_BACK": STEP_BACK_2, "DIAGONAL_DOWN_2_BACK": STEP_BACK_2"
        },
        "SOUTH": {
            "FORWARD": "STEP_BACK", "BACK": "STEP_FORWARD", "LEFT": "STEP_RIGHT", "RIGHT": "STEP_LEFT",
            "UP": "CLIMB_UP", "DOWN": "CLIMB_DOWN", "DIAGONAL_UP_RIGHT": "STEP_UP_LEFT", "DIAGONAL_UP_LEFT": "STEP_UP_RIGHT",
            "DIAGONAL_UP_BACK": "STEP_UP", "DIAGONAL_DOWN_RIGHT": "STEP_DOWN_LEFT",
            "DIAGONAL_DOWN_LEFT": "STEP_DOWN_RIGHT", "DIAGONAL_DOWN_BACK": "STEP_DOWN",
            "DIAGONAL_UP_2_RIGHT": "STEP_UP_2_LEFT", "DIAGONAL_UP_2_LEFT": "STEP_UP_2_RIGHT",
            "DIAGONAL_UP_2_BACK": "STEP_UP_2", "DIAGONAL_DOWN_2_RIGHT": "STEP_DOWN_2_LEFT",
            "DIAGONAL_DOWN_LEFT": "STEP_DOWN_2_RIGHT"
            # movements that are iffy
            # DIAGONAL_UP_FORWARD": "STEP_UP", "DIAGONAL_DOWN_FORWARD": "STEP_DOWN",
            # "DIAGONAL_UP_2_FORWARD": "STEP_UP_2", "DIAGONAL_DOWN_2_FORWARD": STEP_DOWN_2
        },
        "EAST": {
            "FORWARD": "STEP_LEFT", "BACK": "STEP_RIGHT", "LEFT": "STEP_BACK", "RIGHT": "STEP_FORWARD",
            "UP": "CLIMB_UP", "DOWN": "CLIMB_DOWN", "DIAGONAL_UP_RIGHT": "STEP_UP",
            "DIAGONAL_UP_FORWARD": "STEP_UP_LEFT", "DIAGONAL_UP_BACK": "STEP_UP_RIGHT",
            "DIAGONAL_DOWN_RIGHT": "STEP_DOWN", "DIAGONAL_DOWN_FORWARD": "STEP_DOWN_LEFT",
            "DIAGONAL_DOWN_BACK": "STEP_DOWN_RIGHT", "DIAGONAL_UP_2_RIGHT": "STEP_UP_2",
            "DIAGONAL_UP_2_FORWARD": "STEP_UP_2_LEFT", "DIAGONAL_UP_2_BACK": "STEP_UP_2_RIGHT",
            "DIAGONAL_DOWN_2_RIGHT": "STEP_DOWN_2", "DIAGONAL_DOWN_2_FORWARD": "STEP_DOWN_2_LEFT",
            "DIAGONAL_DOWN_2_BACK": "STEP_DOWN_2_RIGHT"
            # movements that are iffy
            # "DIAGONAL_UP_LEFT": "STEP_UP_RIGHT", "DIAGONAL_DOWN_LEFT": "STEP_DOWN_RIGHT",
            # "DIAGONAL_UP_2_LEFT": "STEP_UP_2_RIGHT", "DIAGONAL_DOWN_2_LEFT": "STEP_DOWN_2_RIGHT"
        },
        "WEST": {
            "FORWARD": "STEP_RIGHT", "BACK": "STEP_LEFT", "LEFT": "STEP_FORWARD", "RIGHT": "STEP_BACK",
            "UP": "CLIMB_UP", "DOWN": "CLIMB_DOWN", "DIAGONAL_UP_LEFT": "STEP_UP", 
            "DIAGONAL_UP_FORWARD": "STEP_UP_RIGHT", "DIAGONAL_UP_BACK": "STEP_UP_LEFT",
            "DIAGONAL_DOWN_LEFT": "STEP_DOWN", "DIAGONAL_DOWN_FORWARD": "STEP_DOWN_RIGHT",
            "DIAGONAL_DOWN_BACK": "STEP_DOWN_LEFT", "DIAGONAL_UP_2_LEFT": "STEP_UP_2",
            "DIAGONAL_UP_2_FORWARD": "STEP_UP_2_RIGHT", "DIAGONAL_UP_2_BACK": "STEP_UP_2_LEFT",
            "DIAGONAL_DOWN_2_LEFT": "STEP_DOWN_2", "DIAGONAL_DOWN_2_FORWARD": "STEP_DOWN_2_RIGHT",
            "DIAGONAL_DOWN_2_BACK": "STEP_DOWN_2_LEFT"
            # movements that are iffy
            # "DIAGONAL_UP_RIGHT": "STEP_UP", "DIAGONAL_DOWN_RIGHT": "STEP_DOWN",
            # "DIAGONAL_UP_2_RIGHT": "STEP_UP_2", "DIAGONAL_DOWN_2_RIGHT": "STEP_DOWN_2"
        }
    }
    return step_mappings[CURRENT_ORIENTATION.name].get(movement_direction, "ERROR: invalid orientation")

# returns the direction of the movement and the new orientation
def get_direction(current_coord, next_coord):
    delta_x = next_coord[0] - current_coord[0]
    delta_y = next_coord[2] - current_coord[2]
    delta_z = next_coord[1] - current_coord[1] # this is the vertical difference

    # these movements are relative to when you are looking normally at a x, y, z plane
    movement_directions = {
        # horizontal & vertical movements
        (1, 0, 0): ('RIGHT', InchwormOrientation.EAST), (-1, 0, 0): ('LEFT', InchwormOrientation.WEST),
        (0, 0, 1): ('RIGHT', InchwormOrientation.NORTH), (0, 0, -1): ('LEFT', InchwormOrientation.SOUTH),
        (0, 1, 0): ('UP', "null"), (0, -1, 0): ('DOWN', "null"), 
        # diagonal up movements
        (1, 1, 0): ('DIAGONAL_UP_RIGHT', InchwormOrientation.EAST), 
        (-1, 1, 0): ('DIAGONAL_UP_LEFT', InchwormOrientation.WEST),
        (0, 1, 1): ('DIAGONAL_UP_FORWARD', InchwormOrientation.NORTH), 
        (0, 1, -1): ('DIAGONAL_UP_BACK', InchwormOrientation.NORTH),
        # diagonal down movements
        (1, -1, 0): ('DIAGONAL_UP_RIGHT', InchwormOrientation.EAST), 
        (-1, -1, 0): ('DIAGONAL_UP_LEFT', InchwormOrientation.WEST),
        (0, -1, 1): ('DIAGONAL_UP_FORWARD', InchwormOrientation.NORTH), 
        (0, -1, -1): ('DIAGONAL_UP_BACK', InchwormOrientation.SOUTH),
        # diagonal up 2 movements
        (1, 2, 0): ('DIAGONAL_UP_2_RIGHT', InchwormOrientation.EAST), 
        (-1, 2, 0): ('DIAGONAL_UP_2_LEFT', InchwormOrientation.WEST),
        (0, 2, 1): ('DIAGONAL_UP_2_FORWARD', InchwormOrientation.NORTH), 
        (0, 2, -1): ('DIAGONAL_UP_2_BACK', InchwormOrientation.NORTH),
        # diagonal down 2 movements
        (1, -2, 0): ('DIAGONAL_DOWN_2_RIGHT', InchwormOrientation.EAST), 
        (-1, -2, 0): ('DIAGONAL_DOWN_2_LEFT', InchwormOrientation.WEST),
        (0, -2, 1): ('DIAGONAL_DOWN_2_FORWARD', InchwormOrientation.NORTH), 
        (0, -2, -1): ('DIAGONAL_DOWN_2_BACK', InchwormOrientation.SOUTH),
        # simplified down 1 movements
        (2, -1, 1): ('SIMPLIFIED_POS_1_DOWN_1', InchwormOrientation.EAST),
        (-2, -1, 1): ('SIMPLIFIED_POS_2_DOWN_1', InchwormOrientation.WEST),
        (2, -1, -1): ('SIMPLIFIED_POS_3_DOWN_1', InchwormOrientation.EAST),
        (-2, -1, -1): ('SIMPLIFIED_POS_4_DOWN_1', InchwormOrientation.WEST),
        # simplified down 2 movements
        (2, -2, 1): ('SIMPLIFIED_POS_1_DOWN_2', InchwormOrientation.EAST),
        (-2, -2, 1): ('SIMPLIFIED_POS_2_DOWN_2', InchwormOrientation.WEST),
        (2, -2, -1): ('SIMPLIFIED_POS_3_DOWN_2', InchwormOrientation.EAST),
        (-2, -2, -1): ('SIMPLIFIED_POS_4_DOWN_2', InchwormOrientation.WEST)
        # TODO Simplified down 3 movements
    }
    for key, value in movement_directions.items():
        if (key[0] == delta_x) & (key[1] == delta_y) & (key[2] == delta_z):
            return value
        return 'error', InchwormOrientation.SOUTH

# this function will determine if a helper block is needed to reach a certain location
def determine_helper_blocks(grid, path_start, path_end):
    # right now, this function only recalculates bfs by searching for vertical paths, for the case when the structure is something like a column
    # in the future, this function should be able to determine if a helper block is needed, and if so, where to place it
    grid, path_coords, num_steps = bfs_vertical_path(grid, path_start, path_end)
    return grid, path_coords, num_steps

# once a block is placed, this function manually adds an extra coordinate to the path in order to simplfify the movement after the inchworm places a block
# it then adds the extra coordinate and step to get there to complete_path and complete_steps
def simplify_steps(PAST_LOC, complete_path, complete_steps):
    global CURRENT_LOC, CURRENT_ORIENTATION
    # Case 1 and 3 
    if CURRENT_LOC[0] < BD_LOC1[0]:
        new_start = [PAST_LOC[0]+2, PAST_LOC[1], PAST_LOC[2]]
    elif CURRENT_LOC[0] > BD_LOC1[0]:
        new_start = [PAST_LOC[0]-2, PAST_LOC[1], PAST_LOC[2]]
    else:
        print("you are already on the block depot") 

    # in this case, get_direction returns the correct step, so no need to call update_steps
    movement_direction, new_orientation = get_direction(CURRENT_LOC, new_start)

    # add these values to the complete path and steps
    complete_steps.append((movement_direction, False))
    complete_path.append((new_start, False))

    # update the current location and orientation
    if(new_orientation != "null"):
        CURRENT_ORIENTATION = new_orientation
    CURRENT_LOC = new_start
    
    return complete_path, complete_steps  

# returns: 
# -a list of all the path coords for all the structures like [[(x1, y1, z1), (x2, y2, z2), ...], [(x1, y1, z1), (x2, y2, z2), ...], ...]
# -a list of all the steps to build all the structures like [(STEP_FORWARD, False), (STEP_LEFT, False), ...] Note: the boolean indicates in the inchworm is holding a block or not
def dev_total_path_steps(structures, misc_blocks):
    grid = initialize_grid_with_structures()
    update_grid_with_structure(grid)
    complete_path = []
    complete_steps = []
    list_of_goals = []
    print('misc_blocks',misc_blocks)

    if not structures:
        print("No known structures.")
    else:
        print("looking for known structures.")

        for structure in structures:
            # get path coords for each coord in the structure
            # should go from current location to block depot, then from block depot to block until last block in structure
            list_of_structure_coords = structure[1]
            for coord in list_of_structure_coords:
                PAST_LOC = copy.deepcopy(CURRENT_LOC)
                print("corod: ", coord)
                # get path and steps from current location to block depot
                bd_path, bd_steps = convert_path_coords_to_steps(grid, CURRENT_LOC, BD_LOC1)
                print("bd_path: ", bd_path)
                # pop the first value in list of path coords to remove repeat coords
                bd_path.pop(0)

                # edit the coordinate for simulation purposes
                x, z, y = coord
                new_coord = [x, z-1, y]

                # add the new coordinate to the list of goals
                list_of_goals.append(new_coord)
                
                # get path and steps from current location to coordinate in structure
                block_path, block_steps = convert_path_coords_to_steps(grid, CURRENT_LOC, new_coord)
                # update grid to indicate that the placed block can now be walked on
                grid = update_grid_with_structure(grid, coord)

                # pop the first value in list of path coords to remove repeat coords
                block_path.pop(0)
                
                # get the second to last location in the block_path
                PAST_LOC = block_path[-2][0]

                complete_path.extend(bd_path)
                complete_path.extend(block_path)

                complete_steps.extend(bd_steps)
                complete_steps.extend(block_steps)

                # simplify the step after placing if it is the demo and it's not the last block in the structure
                if DEMO == True and coord != list_of_structure_coords[-1]:
                    complete_path, complete_steps = simplify_steps(PAST_LOC, complete_path, complete_steps)
    if not misc_blocks:
        print("No misc_block.")
    else:
        print("looking for misc_block.")
        # at this point, we have paths and stepa for each block in each structure, but not the miscellanous blocks
        # search for path and steps for each miscellanous block, and add it to the complete path and steps
        for coord in misc_blocks:
            PAST_LOC = copy.deepcopy(CURRENT_LOC)

            # get path and steps from current location to block depot  
            bd_path, bd_steps = convert_path_coords_to_steps(grid, CURRENT_LOC, BD_LOC1)

            # pop the first value in list of path coords to remove repeat coords
            try:
                bd_path.pop(0)
            except IndexError:
                print('bd_path:', bd_path)

            # edit the coordinate for simulation purposes
            x, z, y = coord
            new_coord = [x, z-1, y]

            list_of_goals.append(new_coord)
            
            # get path and steps from current location to coordinate in structure
            block_path, block_steps = convert_path_coords_to_steps(grid, CURRENT_LOC, new_coord)
            # update grid to indicate that the placed block can now be walked on
            grid = update_grid_with_structure(grid, coord)

            # pop the first value in list of path coords to remove repeat coords
            block_path.pop(0)

            complete_path.extend(bd_path)
            complete_path.extend(block_path)

            complete_steps.extend(bd_steps)
            complete_steps.extend(block_steps)  

            # simplify the step after placing if it is the demo
            if DEMO == True:
                complete_path, complete_steps = simplify_steps(PAST_LOC, complete_path, complete_steps)

    return complete_path, complete_steps, list_of_goals