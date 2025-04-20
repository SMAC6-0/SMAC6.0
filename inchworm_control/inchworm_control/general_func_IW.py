from enum import Enum
from time import sleep
from colorama import Fore
import copy

# from inchworm_control.ik_test import IkTest
class STEP_TYPE(Enum):
    STEP = 1
    PLACE = 2
    GRAB = 3
    STEP_W_BLOCK = 4

# Control which direction the end effector is pointing relative to the world frame. 
class EE_direction(Enum):
    DOWN = 90
    UP = 0  # This points horizontally away from the inchworm 

class IW_MOVEMENTS(Enum):
    ATTACH_LEADING_FOOT = 1 # activate the servos of the leading foot (NEW EE)
    DETACH_LEADING_FOOT = 2 # release the servos of the leading foot (NEW EE)
    MOVE_LEADING_FOOT = 3

    ATTACH_FOLLOWING_FOOT = 4
    DETACH_FOLLOWING_FOOT = 5
    MOVE_FOLLOWING_FOOT = 6

# Common positions the inchworm must travel to, with the format of [x, y, z, EE_direction] relative to the frame of the inchworm pivot foot. (units: num blocks, deg) 
# The home position is when the inchworm is NOT holding any blocks
HOME_POSITION = [1, 0, 0, EE_direction.DOWN.value] # When both feet are next to each other. Applies for either pivot foot
ABOVE_HOME = [1, 0, 0.5, EE_direction.DOWN.value] # "ABOVE" positions are for trajectory planning, moving the inchworm straight up 

# The following home positions are when the inchworm is holding a block. 
PIVOT_OFF_BLOCK_HOME_POSITION = [1, 0, 1, EE_direction.DOWN.value] # From perspective of the pivot foot, which is NOT holding the block. The other foot is on the block. 
PIVOT_OFF_BLOCK_ABOVE_HOME = [1, 0, 1.5, EE_direction.DOWN.value]

PIVOT_ON_BLOCK_HOME_POSITION = [1, 0, -1, EE_direction.DOWN.value]  # From perspective of the pivot foot, which is on the block
PIVOT_ON_BLOCK_ABOVE_HOME = [1, 0, -0.5, EE_direction.DOWN.value]


# this is based of off the leading foot
def move_iw_general(step_type, deltaX, deltaY, deltaZ):
    print(f"Current step: {step_type}")

    if deltaX > 1 or deltaY > 1 or deltaY < - 1 or deltaZ > 3:
        raise Exception(f"Out of bounds. IW cannnot go to the position: {deltaX, deltaY, deltaZ}")
    
    # decide the movement
    if deltaX == 1 and deltaY == 0 and deltaZ == 0: # move forward
        leading_foot_goal = [deltaX + 1, deltaY, deltaZ]
    elif deltaX == 0 and deltaY != 0 and deltaZ == 0: # turns
        if deltaY < 0: # turn right
            leading_foot_goal = [deltaX+1, deltaY, deltaZ]
        else: # turn left
            leading_foot_goal = [deltaX+1, deltaY, deltaZ]
    elif deltaX == -1 and deltaY == 0 and deltaZ == 0: # turn around
        leading_foot_goal = [deltaX - 1, deltaY, deltaZ]
    else:
        leading_foot_goal = [deltaX+1, deltaY, deltaZ]

    leading_foot_goal_above = copy.deepcopy(leading_foot_goal)
    leading_foot_goal_above[2] += 0.5

    print(Fore.BLUE + "leading foot location", leading_foot_goal)
    print(Fore.BLUE + "leading foot above location", leading_foot_goal_above)
    movements = [] # store the movements the IW is going to perform 
    holding_block = False # is IW holding a block
    prev_position = [] # stores the previous position of the IW
    next_position = [] # stores the next position of the IW
    positions = [] # stores the list of positions the IW needs to go to
    pivot_foot = 1 # stores the pivot foot

    print(Fore.RED + "Attach following foot")

    match step_type:
        case STEP_TYPE.STEP: # handles the forward, backward, lefts and rights
            # Detach leading foot
            print(Fore.RED + "Detach leading foot")

            positions.append(HOME_POSITION)
            positions.append(ABOVE_HOME)
            movements.append(IW_MOVEMENTS.MOVE_LEADING_FOOT)

            positions.append(leading_foot_goal_above)
            movements.append(IW_MOVEMENTS.MOVE_LEADING_FOOT)

            positions.append(leading_foot_goal)
            movements.append(IW_MOVEMENTS.MOVE_LEADING_FOOT)

            print(Fore.RED + "Attach leading foot")
            movements.append(IW_MOVEMENTS.ATTACH_LEADING_FOOT)

            print(Fore.RED + "Leading foot is in place")

        case STEP_TYPE.PLACE: # IW is holding a block!!
            holding_block = True
            # lift the front foot up a lil more 
            print("lift leading foot")

            # lift the front foot up to above home more 
            print(f"lift leading foot from {HOME_POSITION} to {ABOVE_HOME}")
            movements.append(IW_MOVEMENTS.LIFT_LEADING_FOOT)

        case STEP_TYPE.GRAB:
            # Detach leading foot
            print("Detach leading foot")
            movements.append(IW_MOVEMENTS.DETACH_LEADING_FOOT)

            # lift the front foot up to above home more 
            print(f"lift leading foot from {HOME_POSITION} to {ABOVE_HOME}")
            movements.append(IW_MOVEMENTS.LIFT_LEADING_FOOT)
            

        case STEP_TYPE.STEP_W_BLOCK: # IW is holding a block!!
            holding_block = True
            # lift the front foot up a lil more 
            print("lift leading foot")
    
    for i in range(len(movements) -1):
        prev_position = positions[i]
        next_position = positions[i+1]

        print(f"MOVE {movements[i]} from {prev_position} to {next_position}")

    
    leading_foot_goal_above = []
if __name__ == "__main__":
    # ik_test = IkTest()
    move_iw_general(STEP_TYPE.STEP, 0, 0, 1)