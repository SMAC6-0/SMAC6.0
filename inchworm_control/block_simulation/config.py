from enum import Enum

# this file holds variables that the user can change just in this file before running

GRID_SIZE = 10  # Define the size of your grid in one direction 
                # Then, use this dimension to create a 3D list representing the cubical workspace 
                # where every cell is 0, representing that all those cells are walkable 
BD_LOC1 = [4, 1, 4] # The location where new blocks are sourced/placed to then be picked up by the inchworm robot, where the coordinates are represented as (x, z, y).
CURRENT_LOC = [4, 0, 1] # Starting location of the inchworm robot, where the coordinates are represented as (x, z, y).
BD_LOCS = [BD_LOC1] # The locations of all block depots (if there are multiple)
# if the simulation is for the demo, set this to True
SIMULATION = True
LARGE_BUILD = False

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

# have inchworm starting inline with BD 
# column should not be in line with BD