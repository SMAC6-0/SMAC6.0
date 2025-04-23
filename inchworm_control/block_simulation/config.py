from enum import Enum

# this file holds variables that the user can change just in this file before running

GRID_SIZE = 8  # Define the size of your grid in one direction 
                # Then, use this dimension to create a 3D list representing the cubical workspace 
                # where every cell is 0, representing that all those cells are walkable 
GRID_HEIGHT = 4

### BLOCK DEPOTS & SEED BLOCK ###
BD_1_LOC = [4, 4, 1] # The location where new blocks are sourced/placed to then be picked up by the inchworm robot, where the coordinates are represented as (x, y, z).
BD_LOCS = [BD_1_LOC] # The locations of all block depots (if there are multiple)
SEED_BK = [6, 6, 1]

### INCHWORMS ###
# - LOCATIONS - #
NUM_INCHWORMS = 2
IW_1_LOC = [6, 7, 0] # Starting location of the inchworm robot, where the coordinates are represented as (x, y, z).
# IW_2_LOC = [1, 1, 0]
IW_LOCS = [IW_1_LOC] # , IW_2_LOC]

# - ORIENTATION - #
# Define the possible orientations of the inchworm relative to the world
class InchwormOrientation(Enum):
    NORTH = 0       # +y direction
    EAST = 1        # +x
    SOUTH = 2       # -y 
    WEST = 3        # -x

    def rotate(self, steps):
        """
        Rotate the orientation by a number of steps.
        Positive steps rotate clockwise, negative steps rotate counterclockwise.
        Ex: +1 -> RIGHT. Ex: 2 -> Turns around
        """
        new_value = (self.value + steps) % len(InchwormOrientation)
        return InchwormOrientation(new_value)
        
IW_1_ORIENTATION = InchwormOrientation.EAST
IW_2_ORIENTATION = InchwormOrientation.WEST
IW_ORIENTATIONS = [IW_1_ORIENTATION] #, IW_2_ORIENTATION]

# if the simulation is for the demo, set this to True
SIMULATION = False
LARGE_BUILD = False
MANUAL_TESTING = False

# have inchworm starting inline with BD 
# column should not be in line with BD