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

BLOCK_INTERFACING_TIME = 1
# Common positions the inchworm must travel to, with the format of [x, y, z, EE_direction] relative to the frame of the inchworm pivot foot. (units: num blocks, deg) 
# The home position is when the inchworm is NOT holding any blocks
HOME_POSITION = [1, 0, 0, EE_direction.DOWN.value] # When both feet are next to each other. Applies for either pivot foot
ABOVE_HOME = [1, 0, 0.5, EE_direction.DOWN.value] # "ABOVE" positions are for trajectory planning, moving the inchworm straight up 

# The following home positions are when the inchworm is holding a block. 
PIVOT_OFF_BLOCK_HOME_POSITION = [1, 0, 1, EE_direction.DOWN.value] # From perspective of the pivot foot, which is NOT holding the block. The other foot is on the block. 
PIVOT_OFF_BLOCK_ABOVE_HOME = [1, 0, 1.5, EE_direction.DOWN.value]

PIVOT_ON_BLOCK_HOME_POSITION = [1, 0, -1, EE_direction.DOWN.value]  # From perspective of the pivot foot, which is on the block
PIVOT_ON_BLOCK_ABOVE_HOME = [1, 0, -0.5, EE_direction.DOWN.value]

class IkTest:
    # this is based of off the leading foot
    def move_iw_general(self, step_type, deltaX, deltaY, deltaZ):
        """
        Plan and execute a movement for the inchworm (IW) robot based on a specified step type and target offset.
        The robot will move its leading and following feet according to the given deltas in the IW frame

        Args:
            step_type (STEP_TYPE): Type of step being executed (e.g. PLACE, GRAB, STEP, STEP_W_BLOCK).
            deltaX (float): Forward/backward movement in IW frame.
            deltaY (float): Sideways movement (used for turning).
            deltaZ (float): Vertical movement (block level).
        """
        print(Fore.CYAN+f"Step type: {step_type} w/ deltaX, deltaY and deltaZ = {deltaX, deltaY, deltaZ} ")

        # Raise an error if the requested move is outside safe bounds for the IW to perform
        if abs(deltaX) > 1 or abs(deltaY) > 1 or abs(deltaZ) > 2:
            raise Exception(f"Out of bounds. IW cannnot go to the position: {deltaX, deltaY, deltaZ}")
        
        pivot_foot = 1 # stores the pivot foot
        holding_block = step_type in [STEP_TYPE.PLACE, STEP_TYPE.STEP_W_BLOCK] # True if these are the step_types, False otherwise
        movements = [] # store the movements the IW is going to perform 
        positions = [] # stores the list of positions the IW needs to go to

        # Compute all goal positions needed for the step based on deltas
        goals = self.get_foot_goals(deltaX, deltaY, deltaZ)

        # Extract leading and following foot positions (home, goal, and above each)
        leading_foot_home = goals["leading_foot_home"]
        leading_foot_home_above = goals["leading_foot_home_above"]
        leading_foot_goal = goals["leading_foot_goal"]
        leading_foot_goal_above = goals["leading_foot_goal_above"]

        following_foot_home = goals["following_foot_home"]
        following_foot_home_above = goals["following_foot_home_above"]
        following_foot_goal = goals["following_foot_goal"]
        following_foot_goal_above = goals["following_foot_goal_above"]

        # attach detach the feeties based on holding block and pivot foot 
        print(Fore.RED + f"[Attach/Detach] pivot foot = {pivot_foot},holding block = {holding_block}, step = {step_type}")
        sleep(BLOCK_INTERFACING_TIME)

        # Account for the fact that the IW is holding a block
        if step_type in [STEP_TYPE.PLACE, STEP_TYPE.STEP_W_BLOCK]: # IW is holding a block! 
            leading_foot_home[2] = 1
            leading_foot_home_above[2] = 1.5

        # Account for the fact that the leading foot is holding a block and follwoing foot must account for that when stepping
        if step_type in [STEP_TYPE.GRAB, STEP_TYPE.STEP_W_BLOCK]: # IW is stepping with a block! 
            holding_block = True
            if following_foot_home or following_foot_goal:
                following_foot_home[2] = -1
                following_foot_home_above[2] = 0.5

                following_foot_goal[2] -= 1
                following_foot_goal_above[2] = 0.5
        
        # If the step isn't a flat STEP (e.g. it's a place/grab), the leading foot ends up higher
        if step_type not in [STEP_TYPE.STEP]: # IW leading foot ends on top of a block!
            leading_foot_goal[2] += 1
            leading_foot_goal_above[2] += 1

        # Append leading foot trajectory and attach/detach step
        positions += [leading_foot_home, leading_foot_home_above, leading_foot_goal_above, leading_foot_goal]
        movements += [IW_MOVEMENTS.MOVE_LEADING_FOOT] * 3
        # leading food is in place, activate necessary servos
        movements.append(IW_MOVEMENTS.ATTACH_DETACH_SERVO)

        # If not placing a block, move the following foot too
        if step_type not in [STEP_TYPE.PLACE]:
            movements += [IW_MOVEMENTS.MOVE_FOLLOWING_FOOT] * 3
            positions += [following_foot_goal, following_foot_goal_above, following_foot_home_above, following_foot_home]        
        
        # Send movement commands to motors for execution
        self.move_motors(movements, positions, pivot_foot, holding_block, step_type)   
    
    # -------------------------- ********************** --------------------------
    # -------------------------- ***HELPER FUNCTIONS*** --------------------------
    # -------------------------- ********************** --------------------------

    def get_foot_goals(self, deltaX, deltaY, deltaZ):
        """
        Calculate the 3D positions for both the leading and following foot, given a movement direction.

        Determines the home and goal positions of each foot and their 'above' counterparts, considering the deltas.

        Args:
            deltaX (float): Movement in the X direction (forward/backward).
            deltaY (float): Movement in the Y direction (lefts/rights).
            deltaZ (float): Height change (e.g. block level change).

        Returns:
            dict: Contains all positions needed for foot trajectory:
                - leading_foot_home / goal / home_above / goal_above
                - following_foot_home / goal / home_above / goal_above
        """
        

        down = EE_direction.DOWN.value

        # Default positions
        leading_foot_home = [1, 0, deltaZ, down]
        leading_foot_goal = [deltaX + 1, deltaY, deltaZ, down]
        following_foot_home = [deltaX, deltaY, deltaZ, down]
        following_foot_goal = [deltaX + 1, deltaY, deltaZ, down]

        # decide the psotions for some movements
        if deltaX == 1 and deltaY == 0 and deltaZ == 0: # move forward
            print("Move Forward")
            following_foot_home = [deltaX, 0, deltaZ, down]
        elif deltaX == 0 and deltaY != 0 and deltaZ == 0: # turns
            following_foot_home = [deltaX + 1, 0, deltaZ, down]
            if deltaY < 0: # turn right
                print("Turn Right")
                following_foot_goal = [deltaX + 1, - deltaY, deltaZ, down]
            else: # turn left
                print("Turn Left")
                following_foot_goal = [deltaX + 1, - deltaY, deltaZ, EE_direction.DOWN.value]
        elif deltaX == -1 and deltaY == 0: # turn around
            print("Turn Around")
            leading_foot_goal = [deltaX - 1, deltaY, deltaZ, down]
            following_foot_home = []
            following_foot_goal = []
        # else:
        #     print("Moveee")
        #     following_foot_home = [deltaX + 1, deltaY, deltaZ, EE_direction.DOWN.value]
        #     following_foot_goal = [deltaX + 1, deltaY, deltaZ, EE_direction.DOWN.value]

        # get the above positions
        leading_foot_home_above = self.get_above_position(leading_foot_home)
        leading_foot_goal_above = self.get_above_position(leading_foot_goal)

        if following_foot_home and following_foot_goal: # only copy if this is not empty
            following_foot_home_above = self.get_above_position(following_foot_home)
            following_foot_goal_above = self.get_above_position(following_foot_goal)
        else:
            following_foot_goal_above = []
            following_foot_home_above = []

        # print(f"leading_foot_goal {leading_foot_goal} leading_foot_home: {leading_foot_home}, leading_foot_goal_above: {leading_foot_goal_above}, leading_foot_home_above: {leading_foot_home_above}," 
        #     f"following_foot_goal: {following_foot_goal}, following_foot_home: {following_foot_home}, following_foot_goal_above: {following_foot_goal_above}, following_foot_home_above: {following_foot_home_above}")
        return {
            "leading_foot_goal": leading_foot_goal,
            "leading_foot_home": leading_foot_home,
            "leading_foot_goal_above": leading_foot_goal_above,
            "leading_foot_home_above": leading_foot_home_above,
            "following_foot_goal": following_foot_goal,
            "following_foot_home": following_foot_home,
            "following_foot_goal_above": following_foot_goal_above,
            "following_foot_home_above": following_foot_home_above
        }

    def get_above_position(self, positon):
        """
        Return a new position with the Z-height lifted by 0.5 units. Used to lift a foot above a block or the board before moving to a goal location.

        Args:
            position (list): The original foot position as a 1x4 vector.

        Returns:
            list: A new position with Z lifted.
        """
        above_positon = copy.deepcopy(positon)
        above_positon[2] += 0.5
        return above_positon

    def move_motors(self, movements, positions, pivot_foot, holding_block, step_type):
        """
        Execute the planned sequence of movements by sending them to the motors.

        Handles switching the pivot foot and applying delays during servo attachment/detachment.

        Args:
            movements (list): List of movement types (enum).
            positions (list): List of 1x4 vectors specifying foot locations.
            pivot_foot (int): The initial pivot foot (1 or 5).
            holding_block (bool): Whether the IW is carrying a block.
            step_type (STEP_TYPE): The type of step being executed.
        """
        # takes care of actual movements
        for i in range(len(movements)):
            if movements[i] == IW_MOVEMENTS.ATTACH_DETACH_SERVO: #now it's time to change the pivot foot
                print(Fore.YELLOW + "Leading Foot is in place!")
                pivot_foot = 5
                print(Fore.RED + f"[Attach/Detach] pivot foot = {pivot_foot},holding block = {holding_block}, step = {step_type}")
                sleep(BLOCK_INTERFACING_TIME)

            else:
                prev_position = positions[i]
                next_position = positions[i+1]

                if not next_position: # empty 
                    break

                print(f"do {movements[i]} from {prev_position} to {next_position}")

if __name__ == "__main__":
    ik_test = IkTest()
    print(Fore.CYAN+"--------------------- STEP ---------------------")
    ik_test.move_iw_general(STEP_TYPE.STEP, 1, 0, 0) # forward
    ik_test.move_iw_general(STEP_TYPE.STEP, -1, 0, 0) # backwards
    ik_test.move_iw_general(STEP_TYPE.STEP, 0, 1, 0) # left
    ik_test.move_iw_general(STEP_TYPE.STEP, 0, -1, 0) # right 
    
    print(Fore.CYAN+"--------------------- PLACE ---------------------")
    ik_test.move_iw_general(STEP_TYPE.PLACE, 1, 0, 0) # forward 1, 2 and 0 blocks high
    ik_test.move_iw_general(STEP_TYPE.PLACE, -1, 0, 0) # backwards 1, 2 and 0 blocks high
    ik_test.move_iw_general(STEP_TYPE.PLACE, 0, 1, 0) # left 0, 1, 2 blocks high
    ik_test.move_iw_general(STEP_TYPE.PLACE, 0, -1, 0) # right 0, 1, 2 blocks high

    print(Fore.CYAN+"--------------------- GRAB ---------------------")
    ik_test.move_iw_general(STEP_TYPE.GRAB, 1, 0, 0) # forward
    ik_test.move_iw_general(STEP_TYPE.GRAB, -1, 0, 0) # backwards 
    ik_test.move_iw_general(STEP_TYPE.GRAB, 0, 1, 0) # left
    ik_test.move_iw_general(STEP_TYPE.GRAB, 0, -1, 0) # right 


    print(Fore.CYAN+"--------------------- STEP W/ BLOCK ---------------------")
    ik_test.move_iw_general(STEP_TYPE.STEP_W_BLOCK, 1, 0, 0) # forward
    ik_test.move_iw_general(STEP_TYPE.STEP_W_BLOCK, -1, 0, 0) # backwards
    ik_test.move_iw_general(STEP_TYPE.STEP_W_BLOCK, 0, 1, 0) # left
    ik_test.move_iw_general(STEP_TYPE.STEP_W_BLOCK, 0, -1, 0) # right 