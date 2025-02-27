from enum import Enum

# this file holds variables that the user can change just in this file before running

GRID_SIZE = 50  # Define the size of your grid in one direction 
                # Then, use this dimension to create a 3D list representing the cubical workspace 
                # where every cell is 0, representing that all those cells are walkable 

### BLOCK DEPOTS & SEED BLOCK###
BD_1_LOC = [4, 4, 1] # The location where new blocks are sourced/placed to then be picked up by the inchworm robot, where the coordinates are represented as (x, y, z).
BD_LOCS = [BD_1_LOC] # The locations of all block depots (if there are multiple)
SEED_BK = [6, 6, 1]

### INCHWORMS ###
IW_1_LOC = [4, 1, 0] # Starting location of the inchworm robot, where the coordinates are represented as (x, y, z).
IW_2_LOC = [12, 12, 0]
IW_LOCS = [IW_1_LOC, IW_2_LOC]

# Define the possible orientations of the inchworm
class InchwormOrientation(Enum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

    def rotate(self, steps):
        """
        Rotate the orientation by a number of steps.
        Positive steps rotate clockwise, negative steps rotate counterclockwise.
        """
        new_value = (self.value + steps) % len(InchwormOrientation)
        return InchwormOrientation(new_value)
        
CURRENT_ORIENTATION = InchwormOrientation.NORTH

# if the simulation is for the demo, set this to True
SIMULATION = True
LARGE_BUILD = False

# have inchworm starting inline with BD 
# column should not be in line with BD