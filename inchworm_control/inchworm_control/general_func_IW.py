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
    ATTACH_DETACH_SERVO = 1 # activate/release servos 
    MOVE_LEADING_FOOT = 2

    MOVE_FOLLOWING_FOOT = 3

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
    print(Fore.CYAN+f"Step type: {STEP_TYPE.STEP} w/ deltaX, deltaY and deltaZ = {deltaX, deltaY, deltaZ} ")

    if abs(deltaX) > 1 or abs(deltaY) > 1 or deltaZ > 3:
        raise Exception(f"Out of bounds. IW cannnot go to the position: {deltaX, deltaY, deltaZ}")
    
    # decide the movement
    if deltaX == 1 and deltaY == 0 and deltaZ == 0: # move forward
        leading_foot_goal = [deltaX + 1, deltaY, deltaZ, EE_direction.DOWN.value]
        following_foot_home = [1, 0, 0, EE_direction.DOWN.value]
        following_foot_goal = [deltaX + 1, deltaY, deltaZ, EE_direction.DOWN.value]
    elif deltaX == 0 and deltaY != 0 and deltaZ == 0: # turns
        if deltaY < 0: # turn right
            leading_foot_goal = [deltaX+1, deltaY, deltaZ, EE_direction.DOWN.value]
            following_foot_home = [1, 0, 0, EE_direction.DOWN.value]
            following_foot_goal = [deltaX + 1, - deltaY, deltaZ, EE_direction.DOWN.value]
        else: # turn left
            leading_foot_goal = [deltaX+1, deltaY, deltaZ, EE_direction.DOWN.value]
            following_foot_home = [1, 0, 0, EE_direction.DOWN.value]
            following_foot_goal = [deltaX + 1, - deltaY, deltaZ, EE_direction.DOWN.value]
    elif deltaX == -1 and deltaY == 0 and deltaZ == 0: # turn around
        leading_foot_goal = [deltaX - 1, deltaY, deltaZ, EE_direction.DOWN.value]
        following_foot_home = []
        following_foot_goal = []
    else:
        leading_foot_goal = [deltaX+1, deltaY, deltaZ, EE_direction.DOWN.value]
        following_foot_goal = [deltaX + 1, deltaY, deltaZ, EE_direction.DOWN.value]

    leading_foot_home = [1, 0, deltaZ, EE_direction.DOWN.value]
    leading_foot_home_above = copy.deepcopy(leading_foot_home)
    leading_foot_home_above[2] += 0.5

    leading_foot_goal_above = copy.deepcopy(leading_foot_goal)
    leading_foot_goal_above[2] += 0.5

    if following_foot_home and following_foot_goal: # only copy if this is not empty
        following_foot_home_above = copy.deepcopy(following_foot_home)
        following_foot_home_above[2] += 0.5

        following_foot_goal_above = copy.deepcopy(following_foot_goal)
        following_foot_goal_above[2] += 0.5
    else:
        following_foot_goal_above = []
        following_foot_home_above = []


    # print(Fore.BLUE + "leading foot location", leading_foot_goal)
    # print(Fore.BLUE + "leading foot above location", leading_foot_goal_above)
    movements = [] # store the movements the IW is going to perform 
    holding_block = False # is IW holding a block
    prev_position = [] # stores the previous position of the IW
    next_position = [] # stores the next position of the IW
    positions = [] # stores the list of positions the IW needs to go to
    pivot_foot = 1 # stores the pivot foot

    match step_type:
        case STEP_TYPE.STEP: # handles the forward, backward, lefts and rights steps
            positions.append(leading_foot_home)
            positions.append(leading_foot_home_above)
            movements.append(IW_MOVEMENTS.MOVE_LEADING_FOOT)

            positions.append(leading_foot_goal_above)
            movements.append(IW_MOVEMENTS.MOVE_LEADING_FOOT)

            positions.append(leading_foot_goal)
            movements.append(IW_MOVEMENTS.MOVE_LEADING_FOOT)

            # leading food is in place, activate necessary servos
            movements.append(IW_MOVEMENTS.ATTACH_DETACH_SERVO)

            positions.append(following_foot_goal)
            positions.append(following_foot_goal_above)
            movements.append(IW_MOVEMENTS.MOVE_FOLLOWING_FOOT)

            positions.append(following_foot_home_above)
            movements.append(IW_MOVEMENTS.MOVE_FOLLOWING_FOOT)

            positions.append(following_foot_home)
            movements.append(IW_MOVEMENTS.MOVE_FOLLOWING_FOOT)


        case STEP_TYPE.PLACE: # IW is holding a block!!
            holding_block = True
            leading_foot_home[2] = 1
            leading_foot_home_above[2] += 1

            leading_foot_goal[2] = 1
            leading_foot_goal_above[2] += 1

            positions.append(leading_foot_home)
            positions.append(leading_foot_home_above)
            movements.append(IW_MOVEMENTS.MOVE_LEADING_FOOT)

            positions.append(leading_foot_goal_above)
            movements.append(IW_MOVEMENTS.MOVE_LEADING_FOOT)

            positions.append(leading_foot_goal)
            movements.append(IW_MOVEMENTS.MOVE_LEADING_FOOT)

            # leading food is in place, activate necessary servos
            movements.append(IW_MOVEMENTS.ATTACH_DETACH_SERVO)

            positions.append(following_foot_goal)
            positions.append(following_foot_goal_above)
            movements.append(IW_MOVEMENTS.MOVE_FOLLOWING_FOOT)

            positions.append(following_foot_home_above)
            movements.append(IW_MOVEMENTS.MOVE_FOLLOWING_FOOT)

            positions.append(following_foot_home)
            movements.append(IW_MOVEMENTS.MOVE_FOLLOWING_FOOT)

        case STEP_TYPE.GRAB:
            pass
            

        case STEP_TYPE.STEP_W_BLOCK: # IW is holding a block!!
            holding_block = True

    # takes care of actual movements
    # attach detach the feeties based on holding block and pivot foot 
    print(Fore.RED + f"Attach and detach, pivot foot = {pivot_foot} and IW holding block is {holding_block} and step is {step_type}")

    for i in range(len(movements)):
        if movements[i] == IW_MOVEMENTS.ATTACH_DETACH_SERVO: #now it's time to change the pivot foot
            print(Fore.YELLOW + "Leading Foot is in place!")
            pivot_foot = 5
            print(Fore.RED + f"Attach and detach, pivot foot = {pivot_foot} and IW holding block is {holding_block} and step is {step_type}")

        else:
            prev_position = positions[i]
            next_position = positions[i+1]

            if next_position == []: # empty 
                break
            else:
                print(f"do {movements[i]} from {prev_position} to {next_position}")

if __name__ == "__main__":
    # ik_test = IkTest()
    # print(Fore.CYAN+"--------------------- STEP ---------------------")
    # move_iw_general(STEP_TYPE.STEP, 1, 0, 0) # forward
    # move_iw_general(STEP_TYPE.STEP, -1, 0, 0) # backwards
    # move_iw_general(STEP_TYPE.STEP, 0, 1, 0) # left
    # move_iw_general(STEP_TYPE.STEP, 0, -1, 0) # right 
    
    print(Fore.CYAN+"--------------------- PLACE ---------------------")
    move_iw_general(STEP_TYPE.PLACE, 1, 0, 0) # forward
    # move_iw_general(STEP_TYPE.PLACE, -1, 0, 0) # backwards
    # move_iw_general(STEP_TYPE.PLACE, 0, 1, 0) # left
    # move_iw_general(STEP_TYPE.PLACE, 0, -1, 0) # right 
    # move_iw_general(STEP_TYPE.PLACE, 1, 0, 1) # place 1 block high
    # move_iw_general(STEP_TYPE.PLACE, 1, 0, 2) # place 1 block high

    # print(Fore.CYAN+"--------------------- GRAB ---------------------")
    # move_iw_general(STEP_TYPE.STEP, 1, 0, 0) # forward
    # move_iw_general(STEP_TYPE.STEP, -1, 0, 0) # backwards
    # move_iw_general(STEP_TYPE.STEP, 0, 1, 0) # left
    # move_iw_general(STEP_TYPE.STEP, 0, -1, 0) # right 

    # print(Fore.CYAN+"--------------------- STEP W/ BLOCK ---------------------")
    # move_iw_general(STEP_TYPE.STEP, 1, 0, 0) # forward
    # move_iw_general(STEP_TYPE.STEP, -1, 0, 0) # backwards
    # move_iw_general(STEP_TYPE.STEP, 0, 1, 0) # left
    # move_iw_general(STEP_TYPE.STEP, 0, -1, 0) # right 