import copy
from enum import Enum
from path_planning import *
from config import BD_LOC, CURRENT_LOC, CURRENT_ORIENTATION, InchwormOrientation, DEMO

# converts the list of coords from bfs to a list of inchworm movements
# returns the list of path coordinates and the list of steps from start to end
def convert_path_coords_to_steps(grid, path_start, path_end):
    global CURRENT_LOC, CURRENT_ORIENTATION

    # get the path
    path_coords, num_steps = bfs_3d(grid, path_start, path_end)

    # if no path was found, check to see if you'll need a helper block
    if num_steps == -1:
        grid, path_coords, num_steps = determine_helper_blocks(grid, path_start, path_end)

    # if the start is the Block Depot, it is holding a block
    holding_block = False
    if(path_start == BD_LOC):
        holding_block = True

    steps = []
    # goes through each coordinate in path and retrieves the step to go from the current location to the next location
    for i in range(len(path_coords) - 1):
        current_coord = path_coords[i][0]
        next_coord = path_coords[i + 1][0] 

        # offset to handle the inchworm's position when it's on the block depot
        if current_coord == BD_LOC:
            x, z, y = current_coord
            current_coord = [x, z-1, y]

        # get the movement direction and the new orientation
        movement_direction, new_orientation = get_direction(current_coord, next_coord)

        # if the next coord is the block depot, the next step should be a grabbing step
        if next_coord == BD_LOC:
            steps.append((update_manipulation_step(movement_direction, "GRAB"), holding_block))
            
        # if the next coordinate is the goal(and not BD), then we need to place the block
        elif next_coord == path_end:
            steps.append((update_manipulation_step(movement_direction, "PLACE"), holding_block))
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

    if(CURRENT_ORIENTATION == InchwormOrientation.NORTH):
        if(movement_direction == 'FORWARD'):
            return "STEP_FORWARD"
        elif(movement_direction == 'BACK'):
            return "STEP_BACK"
        elif(movement_direction == 'LEFT'): 
            return "STEP_LEFT"
        elif(movement_direction == 'RIGHT'):
            return "STEP_RIGHT"
        elif(movement_direction == 'UP'):
            return "CLIMB_UP"
        elif(movement_direction == 'DOWN'):
            return "CLIMB_DOWN"
        elif(movement_direction == 'DIAGONAL_UP_RIGHT'):
            return "STEP_UP_RIGHT"
        elif(movement_direction == 'DIAGONAL_UP_LEFT'):
            return "STEP_UP_LEFT"
        elif(movement_direction == 'DIAGONAL_UP_FORWARD'):
            return "STEP_UP"
        # elif(movement_direction == 'DIAGONAL_UP_BACK'):
        #     return STEP_BACK
        elif(movement_direction == 'DIAGONAL_DOWN_RIGHT'):
            return "STEP_DOWN_RIGHT"
        elif(movement_direction == 'DIAGONAL_DOWN_LEFT'):
            return "STEP_DOWN_LEFT"
        elif(movement_direction == 'DIAGONAL_DOWN_FORWARD'):
            return "STEP_DOWN"
        # elif(movement_direction == 'DIAGONAL_DOWN_BACK'):
        #     return STEP_BACK
        elif(movement_direction == 'DIAGONAL_UP_2_RIGHT'):
            return "STEP_UP_2_RIGHT"
        elif(movement_direction == 'DIAGONAL_UP_2_LEFT'):
            return "STEP_UP_2_LEFT"
        elif(movement_direction == 'DIAGONAL_UP_2_FORWARD'):
            return "STEP_UP_2"
        # elif(movement_direction == 'DIAGONAL_UP_2_BACK'):
        #     return STEP_BACK_2
        elif(movement_direction == 'DIAGONAL_DOWN_2_RIGHT'):
            return "STEP_DOWN_2_RIGHT"
        elif(movement_direction == 'DIAGONAL_DOWN_2_LEFT'):
            return "STEP_DOWN_2_LEFT"
        elif(movement_direction == 'DIAGONAL_DOWN_2_FORWARD'):
            return "STEP_DOWN_2"
        # elif(movement_direction == 'DIAGONAL_DOWN_2_BACK'):
        #     return STEP_BACK_2

    elif(CURRENT_ORIENTATION == InchwormOrientation.SOUTH):
        if(movement_direction == 'FORWARD'):
            return "STEP_BACK"
        elif(movement_direction == 'BACK'):
            return "STEP_FORWARD"
        elif(movement_direction == 'LEFT'): 
            return "STEP_RIGHT"
        elif(movement_direction == 'RIGHT'):
            return "STEP_LEFT"
        elif(movement_direction == 'UP'):
            return "CLIMB_UP"
        elif(movement_direction == 'DOWN'):
            return "CLIMB_DOWN"
        elif(movement_direction == 'DIAGONAL_UP_RIGHT'):
            return "STEP_UP_LEFT"
        elif(movement_direction == 'DIAGONAL_UP_LEFT'):
            return "STEP_UP_RIGHT"
        # elif(movement_direction == 'DIAGONAL_UP_FORWARD'):
        #     return STEP_UP
        elif(movement_direction == 'DIAGONAL_UP_BACK'):
            return "STEP_UP"
        elif(movement_direction == 'DIAGONAL_DOWN_RIGHT'):
            return "STEP_DOWN_LEFT"
        elif(movement_direction == 'DIAGONAL_DOWN_LEFT'):
            return "STEP_DOWN_RIGHT"
        # elif(movement_direction == 'DIAGONAL_DOWN_FORWARD'):
        #     return STEP_DOWN
        elif(movement_direction == 'DIAGONAL_DOWN_BACK'):
            return "STEP_DOWN"
        elif(movement_direction == 'DIAGONAL_UP_2_RIGHT'):
            return "STEP_UP_2_LEFT"
        elif(movement_direction == 'DIAGONAL_UP_2_LEFT'):
            return "STEP_UP_2_RIGHT"
        # elif(movement_direction == 'DIAGONAL_UP_2_FORWARD'):
        #     return STEP_UP_2
        elif(movement_direction == 'DIAGONAL_UP_2_BACK'):
            return "STEP_UP_2"
        elif(movement_direction == 'DIAGONAL_DOWN_2_RIGHT'):
            return "STEP_DOWN_2_LEFT"
        elif(movement_direction == 'DIAGONAL_DOWN_LEFT'):
            return "STEP_DOWN_2_RIGHT"
        # elif(movement_direction == 'DIAGONAL_DOWN_2_FORWARD'):
        #     return STEP_DOWN_2
        
    elif(CURRENT_ORIENTATION == InchwormOrientation.EAST):
        if(movement_direction == 'FORWARD'):
            return "STEP_LEFT"
        elif(movement_direction == 'BACK'):
            return "STEP_RIGHT"
        elif(movement_direction == 'LEFT'): 
            return "STEP_BACK"
        elif(movement_direction == 'RIGHT'):
            return "STEP_FORWARD"
        elif(movement_direction == 'UP'):
            return "CLIMB_UP"
        elif(movement_direction == 'DOWN'):
            return "CLIMB_DOWN"
        elif(movement_direction == 'DIAGONAL_UP_RIGHT'):
            return "STEP_UP"
        # elif(movement_direction == 'DIAGONAL_UP_LEFT'):
        #     return STEP_UP_RIGHT
        elif(movement_direction == 'DIAGONAL_UP_FORWARD'):
            return "STEP_UP_LEFT"
        elif(movement_direction == 'DIAGONAL_UP_BACK'):
            return "STEP_UP_RIGHT"
        elif(movement_direction == 'DIAGONAL_DOWN_RIGHT'):
            return "STEP_DOWN"
        # elif(movement_direction == 'DIAGONAL_DOWN_LEFT'):
        #     return STEP_DOWN_RIGHT
        elif(movement_direction == 'DIAGONAL_DOWN_FORWARD'):
            return "STEP_DOWN_LEFT"
        elif(movement_direction == 'DIAGONAL_DOWN_BACK'):
            return "STEP_DOWN_RIGHT"
        elif(movement_direction == 'DIAGONAL_UP_2_RIGHT'):
            return "STEP_UP_2"
        # elif(movement_direction == 'DIAGONAL_UP_2_LEFT'):
        #     return STEP_UP_2_RIGHT
        elif(movement_direction == 'DIAGONAL_UP_2_FORWARD'):
            return "STEP_UP_2_LEFT"
        elif(movement_direction == 'DIAGONAL_UP_2_BACK'):
            return "STEP_UP_2_RIGHT"
        elif(movement_direction == 'DIAGONAL_DOWN_2_RIGHT'):
            return "STEP_DOWN_2"
        # elif(movement_direction == 'DIAGONAL_DOWN_2_LEFT'):
        #     return STEP_DOWN_2_RIGHT
        elif(movement_direction == 'DIAGONAL_DOWN_2_FORWARD'):
            return "STEP_DOWN_2_LEFT"
        elif(movement_direction == 'DIAGONAL_DOWN_2_BACK'):
            return "STEP_DOWN_2_RIGHT"
        
    elif(CURRENT_ORIENTATION == InchwormOrientation.WEST):
        if(movement_direction == 'FORWARD'):
            return "STEP_RIGHT"
        elif(movement_direction == 'BACK'):
            return "STEP_LEFT"
        elif(movement_direction == 'LEFT'): 
            return "STEP_FORWARD"
        elif(movement_direction == 'RIGHT'):
            return "STEP_BACK"
        elif(movement_direction == 'UP'):
            return "CLIMB_UP"
        elif(movement_direction == 'DOWN'):
            return "CLIMB_DOWN"
        # elif(movement_direction == 'DIAGONAL_UP_RIGHT'):
        #     return STEP_UP
        elif(movement_direction == 'DIAGONAL_UP_LEFT'):
            return "STEP_UP"
        elif(movement_direction == 'DIAGONAL_UP_FORWARD'):
            return "STEP_UP_RIGHT"
        elif(movement_direction == 'DIAGONAL_UP_BACK'):
            return "STEP_UP_LEFT"
        # elif(movement_direction == 'DIAGONAL_DOWN_RIGHT'):
        #     return  STEP_DOWN
        elif(movement_direction == 'DIAGONAL_DOWN_LEFT'):
            return "STEP_DOWN"
        elif(movement_direction == 'DIAGONAL_DOWN_FORWARD'):
            return "STEP_DOWN_RIGHT"
        elif(movement_direction == 'DIAGONAL_DOWN_BACK'):
            return "STEP_DOWN_LEFT"
        # elif(movement_direction == 'DIAGONAL_UP_2_RIGHT'):
        #     return STEP_UP_2
        elif(movement_direction == 'DIAGONAL_UP_2_LEFT'):
            return "STEP_UP_2"
        elif(movement_direction == 'DIAGONAL_UP_2_FORWARD'):
            return "STEP_UP_2_RIGHT"
        elif(movement_direction == 'DIAGONAL_UP_2_BACK'):
            return "STEP_UP_2_LEFT"
        # elif(movement_direction == 'DIAGONAL_DOWN_2_RIGHT'):
        #     return  STEP_DOWN_2
        elif(movement_direction == 'DIAGONAL_DOWN_2_LEFT'):
            return "STEP_DOWN_2"
        elif(movement_direction == 'DIAGONAL_DOWN_2_FORWARD'):
            return "STEP_DOWN_2_RIGHT"
        elif(movement_direction == 'DIAGONAL_DOWN_2_BACK'):
            return "STEP_DOWN_2_LEFT"
    else:
        print("ERROR: invalid orientation")

# given the movement direction, returns the step to take, but with a manipulation action attached to it (grab/place)
def update_manipulation_step(movement_direction, manipulation):
    # manipulation is either "GRAB" or "PLACE"

    if(CURRENT_ORIENTATION == InchwormOrientation.NORTH):
        if(movement_direction == 'FORWARD'):
            return manipulation + "_FORWARD"
        elif(movement_direction == 'LEFT'): 
            return manipulation + "_LEFT"
        elif(movement_direction == 'RIGHT'):
            return manipulation + "_RIGHT"
        elif(movement_direction == 'DIAGONAL_UP_RIGHT'):
            return manipulation + "_UP_RIGHT"
        elif(movement_direction == 'DIAGONAL_UP_LEFT'):
            return manipulation + "_UP_LEFT"
        elif(movement_direction == 'DIAGONAL_UP_FORWARD'):
            return manipulation + "_UP_FORWARD"
        # elif(movement_direction == 'DIAGONAL_UP_BACK'):
        #     return STEP_BACK
        elif(movement_direction == 'DIAGONAL_DOWN_RIGHT'):
            return manipulation + "_DOWN_RIGHT"
        elif(movement_direction == 'DIAGONAL_DOWN_LEFT'):
            return manipulation + "_DOWN_LEFT"
        elif(movement_direction == 'DIAGONAL_DOWN_FORWARD'):
            return manipulation + "_DOWN_FORWARD"
        elif(movement_direction == 'DIAGONAL_UP_2_RIGHT'):
            return manipulation + "_UP_2_RIGHT"
        elif(movement_direction == 'DIAGONAL_UP_2_LEFT'):
            return manipulation + "_UP_2_LEFT"
        elif(movement_direction == 'DIAGONAL_UP_2_FORWARD'):
            return manipulation + "_UP_2_FORWARD"
        # elif(movement_direction == 'DIAGONAL_UP_2_BACK'):
        #     return STEP_BACK_2
        elif(movement_direction == 'DIAGONAL_DOWN_2_RIGHT'):
            return manipulation + "_DOWN_2_RIGHT"
        elif(movement_direction == 'DIAGONAL_DOWN_2_LEFT'):
            return manipulation + "_DOWN_2_LEFT"
        elif(movement_direction == 'DIAGONAL_DOWN_2_FORWARD'):
            return manipulation + "_DOWN_2_FORWARD"

    elif(CURRENT_ORIENTATION == InchwormOrientation.SOUTH):
        if(movement_direction == 'BACK'):
            return manipulation + "_FORWARD"
        elif(movement_direction == 'LEFT'): 
            return manipulation + "_RIGHT"
        elif(movement_direction == 'RIGHT'):
            return manipulation + "_LEFT"
        elif(movement_direction == 'DIAGONAL_UP_RIGHT'):
            return manipulation + "_UP_LEFT"
        elif(movement_direction == 'DIAGONAL_UP_LEFT'):
            return manipulation + "_UP_RIGHT"
        # elif(movement_direction == 'DIAGONAL_UP_FORWARD'):
        #     return STEP_UP
        elif(movement_direction == 'DIAGONAL_UP_BACK'):
            return manipulation + "_UP_FORWARD"
        elif(movement_direction == 'DIAGONAL_DOWN_RIGHT'):
            return manipulation + "_DOWN_LEFT"
        elif(movement_direction == 'DIAGONAL_DOWN_LEFT'):
            return manipulation + "_DOWN_RIGHT"
        # elif(movement_direction == 'DIAGONAL_DOWN_FORWARD'):
        #     return STEP_DOWN
        elif(movement_direction == 'DIAGONAL_DOWN_BACK'):
            return manipulation + "_DOWN_FORWARD"
        elif(movement_direction == 'DIAGONAL_UP_2_RIGHT'):
            return manipulation + "_UP_2_LEFT"
        elif(movement_direction == 'DIAGONAL_UP_2_LEFT'):
            return manipulation + "_UP_2_RIGHT"
        # elif(movement_direction == 'DIAGONAL_UP_2_FORWARD'):
        #     return STEP_UP_2
        elif(movement_direction == 'DIAGONAL_UP_2_BACK'):
            return manipulation + "_UP_2_FORWARD"
        elif(movement_direction == 'DIAGONAL_DOWN_2_RIGHT'):
            return manipulation + "_DOWN_2_LEFT"
        elif(movement_direction == 'DIAGONAL_DOWN_2_LEFT'):
            return manipulation + "_DOWN_2_RIGHT"
        # elif(movement_direction == 'DIAGONAL_DOWN_2_FORWARD'):
        #     return STEP_DOWN_2
        elif(movement_direction == 'DIAGONAL_DOWN_2_BACK'):
            return manipulation + "_DOWN_2_FORWARD"
        
    elif(CURRENT_ORIENTATION == InchwormOrientation.EAST):
        if(movement_direction == 'FORWARD'):
            return manipulation + "_LEFT"
        elif(movement_direction == 'BACK'):
            return manipulation + "_RIGHT"
        elif(movement_direction == 'RIGHT'):
            return manipulation + "_FORWARD"
        elif(movement_direction == 'DIAGONAL_UP_RIGHT'):
            return manipulation + "_UP_FORWARD"
        # elif(movement_direction == 'DIAGONAL_UP_LEFT'):
        #     return STEP_UP_RIGHT
        elif(movement_direction == 'DIAGONAL_UP_FORWARD'):
            return manipulation + "_UP_LEFT"
        elif(movement_direction == 'DIAGONAL_UP_BACK'):
            return manipulation + "_UP_RIGHT"
        elif(movement_direction == 'DIAGONAL_DOWN_RIGHT'):
            return manipulation + "_DOWN_FORWARD"
        # elif(movement_direction == 'DIAGONAL_DOWN_LEFT'):
        #     return STEP_DOWN_RIGHT
        elif(movement_direction == 'DIAGONAL_DOWN_FORWARD'):
            return manipulation + "_DOWN_LEFT"
        elif(movement_direction == 'DIAGONAL_DOWN_BACK'):
            return manipulation + "_DOWN_RIGHT"
        elif(movement_direction == 'DIAGONAL_UP_2_RIGHT'):
            return manipulation + "_UP_2_FORWARD"
        # elif(movement_direction == 'DIAGONAL_UP_2_LEFT'):
        #     return STEP_UP_2_RIGHT
        elif(movement_direction == 'DIAGONAL_UP_2_FORWARD'):
            return manipulation + "_UP_2_LEFT"
        elif(movement_direction == 'DIAGONAL_UP_2_BACK'):
            return manipulation + "_UP_2_RIGHT"
        elif(movement_direction == 'DIAGONAL_DOWN_2_RIGHT'):
            return manipulation + "_DOWN_2_FORWARD"
        # elif(movement_direction == 'DIAGONAL_DOWN_2_LEFT'):
        #     return STEP_DOWN_2_RIGHT
        elif(movement_direction == 'DIAGONAL_DOWN_2_FORWARD'):
            return manipulation + "_DOWN_2_LEFT"
        elif(movement_direction == 'DIAGONAL_DOWN_2_BACK'):
            return manipulation + "_DOWN_2_RIGHT"
        
    elif(CURRENT_ORIENTATION == InchwormOrientation.WEST):
        if(movement_direction == 'FORWARD'):
            return manipulation + "_RIGHT"
        elif(movement_direction == 'BACK'):
            return manipulation + "_LEFT"
        elif(movement_direction == 'LEFT'): 
            return manipulation + "_FORWARD"
        # elif(movement_direction == 'DIAGONAL_UP_RIGHT'):
        #     return STEP_UP
        elif(movement_direction == 'DIAGONAL_UP_LEFT'):
            return manipulation + "_UP_FORWARD"
        elif(movement_direction == 'DIAGONAL_UP_FORWARD'):
            return manipulation + "_UP_RIGHT"
        elif(movement_direction == 'DIAGONAL_UP_BACK'):
            return manipulation + "_UP_LEFT"
        # elif(movement_direction == 'DIAGONAL_DOWN_RIGHT'):
        #     return  STEP_DOWN
        elif(movement_direction == 'DIAGONAL_DOWN_LEFT'):
            return manipulation + "_DOWN_FORWARD"
        elif(movement_direction == 'DIAGONAL_DOWN_FORWARD'):
            return manipulation + "_DOWN_RIGHT"
        elif(movement_direction == 'DIAGONAL_DOWN_BACK'):
            return manipulation + "_DOWN_LEFT"
        # elif(movement_direction == 'DIAGONAL_UP_2_RIGHT'):
        #     return STEP_UP_2
        elif(movement_direction == 'DIAGONAL_UP_2_LEFT'):
            return manipulation + "_UP_2_FORWARD"
        elif(movement_direction == 'DIAGONAL_UP_2_FORWARD'):
            return manipulation + "_UP_2_RIGHT"
        elif(movement_direction == 'DIAGONAL_UP_2_BACK'):
            return manipulation + "_UP_2_LEFT"
        # elif(movement_direction == 'DIAGONAL_DOWN_2_RIGHT'):
        #     return  STEP_DOWN_2
        elif(movement_direction == 'DIAGONAL_DOWN_2_LEFT'):
            return manipulation + "_DOWN_2_FORWARD"
        elif(movement_direction == 'DIAGONAL_DOWN_2_FORWARD'):
            return manipulation + "_DOWN_2_RIGHT"
        elif(movement_direction == 'DIAGONAL_DOWN_2_BACK'):
            return manipulation + "_DOWN_2_LEFT"
    else:
        print("ERROR: invalid orientation")

# returns the direction of the movement and the new orientation
def get_direction(current_coord, next_coord):
    delta_x = next_coord[0] - current_coord[0]
    delta_y = next_coord[2] - current_coord[2]
    delta_z = next_coord[1] - current_coord[1] # this is the vertical difference

    # these movements are relative to when you are looking normally at a x, y, z plane
    # horizontal movements
    if delta_x == 1 and delta_z == 0 and delta_y == 0:
        return 'RIGHT', InchwormOrientation.EAST
    elif delta_x == -1 and delta_z == 0 and delta_y == 0:
        return 'LEFT', InchwormOrientation.WEST
    elif delta_x == 0 and delta_z == 0 and delta_y == 1:
        return 'FORWARD', InchwormOrientation.NORTH
    elif delta_x == 0 and delta_z == 0 and delta_y == -1:
        return 'BACK', InchwormOrientation.SOUTH
    # vertical movements
    elif delta_x == 0 and delta_z == 1 and delta_y == 0:
        return 'UP', "null"
    elif delta_x == 0 and delta_z == -1 and delta_y == 0:
        return 'DOWN', "null"
    # Diagonal up movements
    elif delta_x == 1 and delta_z == 1 and delta_y == 0:
        return 'DIAGONAL_UP_RIGHT', InchwormOrientation.EAST
    elif delta_x == -1 and delta_z == 1 and delta_y == 0:
        return 'DIAGONAL_UP_LEFT', InchwormOrientation.WEST
    elif delta_x == 0 and delta_z == 1 and delta_y == 1:
        return 'DIAGONAL_UP_FORWARD', InchwormOrientation.NORTH
    elif delta_x == 0 and delta_z == 1 and delta_y == -1:
        return 'DIAGONAL_UP_BACK', InchwormOrientation.NORTH
    # Diagonal down movements
    elif delta_x == 1 and delta_z == -1 and delta_y == 0:
        return 'DIAGONAL_DOWN_RIGHT', InchwormOrientation.EAST
    elif delta_x == -1 and delta_z == -1 and delta_y == 0:
        return 'DIAGONAL_DOWN_LEFT', InchwormOrientation.WEST
    elif delta_x == 0 and delta_z == -1 and delta_y == 1:
        return 'DIAGONAL_DOWN_FORWARD', InchwormOrientation.NORTH
    elif delta_x == 0 and delta_z == -1 and delta_y == -1:
        return 'DIAGONAL_DOWN_BACK', InchwormOrientation.SOUTH
    # Diagonal up 2 movements
    elif delta_x == 1 and delta_z == 2 and delta_y == 0:
        return 'DIAGONAL_UP_2_RIGHT', InchwormOrientation.EAST
    elif delta_x == -1 and delta_z == 2 and delta_y == 0:
        return 'DIAGONAL_UP_2_LEFT', InchwormOrientation.WEST
    elif delta_x == 0 and delta_z == 2 and delta_y == 1:
        return 'DIAGONAL_UP_2_FORWARD', InchwormOrientation.NORTH
    elif delta_x == 0 and delta_z == 2 and delta_y == -1:
        return 'DIAGONAL_UP_2_BACK', InchwormOrientation.NORTH
    # Diagonal down 2 movements
    elif delta_x == 1 and delta_z == -2 and delta_y == 0:
        return 'DIAGONAL_DOWN_2_RIGHT', InchwormOrientation.EAST
    elif delta_x == -1 and delta_z == -2 and delta_y == 0:
        return 'DIAGONAL_DOWN_2_LEFT', InchwormOrientation.WEST
    elif delta_x == 0 and delta_z == -2 and delta_y == 1:
        return 'DIAGONAL_DOWN_2_FORWARD', InchwormOrientation.NORTH
    elif delta_x == 0 and delta_z == -2 and delta_y == -1:
        return 'DIAGONAL_DOWN_2_BACK', InchwormOrientation.SOUTH
    # Simplified down 1 movements
    elif delta_x == 2 and delta_z == -1 and delta_y == 1:
        return 'SIMPLIFIED_POS_1_DOWN_1', InchwormOrientation.EAST
    elif delta_x == -2 and delta_z == -1 and delta_y == 1:
        return 'SIMPLIFIED_POS_2_DOWN_1', InchwormOrientation.WEST
    elif delta_x == 2 and delta_z == -1 and delta_y == -1:
        return 'SIMPLIFIED_POS_3_DOWN_1', InchwormOrientation.EAST
    elif delta_x == -2 and delta_z == -1 and delta_y == -1:
        return 'SIMPLIFIED_POS_4_DOWN_1', InchwormOrientation.WEST
    # Simplified down 2 movements
    elif delta_x == 2 and delta_z == -2 and delta_y == 1:
        return 'SIMPLIFIED_POS_1_DOWN_2', InchwormOrientation.EAST
    elif delta_x == -2 and delta_z == -2 and delta_y == 1:
        return 'SIMPLIFIED_POS_2_DOWN_2', InchwormOrientation.WEST
    elif delta_x == 2 and delta_z == -2 and delta_y == -1:
        return 'SIMPLIFIED_POS_3_DOWN_2', InchwormOrientation.EAST
    elif delta_x == -2 and delta_z == -2 and delta_y == -1:
        return 'SIMPLIFIED_POS_4_DOWN_2', InchwormOrientation.WEST
    # TODO Simplified down 3 movements
    else:
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
    if CURRENT_LOC[0] < BD_LOC[0]:
        new_start = [PAST_LOC[0]+2, PAST_LOC[1], PAST_LOC[2]]
    elif CURRENT_LOC[0] > BD_LOC[0]:
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
    grid = initialize_grid_with_structures(50)
    update_grid_with_structure(grid, BD_LOC)
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
                bd_path, bd_steps = convert_path_coords_to_steps(grid, CURRENT_LOC, BD_LOC)
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
            bd_path, bd_steps = convert_path_coords_to_steps(grid, CURRENT_LOC, BD_LOC)

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