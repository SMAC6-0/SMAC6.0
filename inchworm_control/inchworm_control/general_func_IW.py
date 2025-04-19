from enum import Enum
from time import sleep


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
    LIFT_LEADING_FOOT = 3 # lift leading foot from the home position
    MOVE_LEADING_FOOT = 4

    ATTACH_FOLLOWING_FOOT = 1
    DETACH_FOLLOWING_FOOT = 2

    
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

    if deltaX > 1 or deltaY > 1 or deltaZ > 3:
        raise Exception(f"Out of bounds. IW cannnot go to the position: {deltaX, deltaY, deltaZ}")
    
    # decide the movement
    if deltaX == 1 and deltaY == 0 and deltaZ == 0: # move forward
        leading_foot_goal = [deltaX + 1, deltaY, deltaZ]
    elif deltaX == 0 and deltaY != 0 and deltaZ == 0: # turns
        if deltaY < 0: # turn right
            leading_foot_goal = [deltaX, deltaY-1, deltaZ]
        else: # turn left
            leading_foot_goal = [deltaX, deltaY+1, deltaZ]
    elif deltaX == -1 and deltaY == 0 and deltaZ == 0: # turn around
        leading_foot_goal = [deltaX - 1, deltaY, deltaZ]
    else:
        leading_foot_goal = [deltaX, deltaY, deltaZ]

    print("leading foot location", leading_foot_goal)
    movements = [] # store the movements the IW is going to perform 
    
    # following foot
    print("Attach following foot")
    movements.append(IW_MOVEMENTS.ATTACH_FOLLOWING_FOOT)
    match step_type:
        case STEP_TYPE.STEP:
            # Detach leading foot
            print("Detach leading foot")
            movements.append(IW_MOVEMENTS.DETACH_LEADING_FOOT)
            
            # lift the front foot above home to  
            # print(f"lift leading foot from {ABOVE_HOME} to {leading_foot_goal_above}")

            # # lift the front foot above home to  
            # print(f"lift leading foot from {leading_foot_goal_above} to {leading_foot_goal}")
            
        case STEP_TYPE.PLACE:
            # lift the front foot up a lil more 
            print("lift leading foot")

        case STEP_TYPE.GRAB:
            # Detach leading foot
            print("Detach leading foot")
            # lift the front foot up a lil more 
            print("lift leading foot")

        case STEP_TYPE.STEP_W_BLOCK:
            # lift the front foot up a lil more 
            print("lift leading foot")
    
    # lift the front foot up to above home more 
    print(f"lift leading foot from {HOME_POSITION} to {ABOVE_HOME}")
    movements.append(IW_MOVEMENTS.LIFT_LEADING_FOOT)

    
    leading_foot_goal_above = []
if __name__ == "__main__":
    # ik_test = IkTest()
    move_iw_general(STEP_TYPE.STEP, 0, 0, 3)