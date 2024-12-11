'''
The purpose of this file is to store all data necessary to run a simulation of 
*multiple* inchworms building a structure. It is NOT the running of the engine (textures, 
colors, button presses). This simulation simulates not the processes of any one robot, but of 
all inchworm agents. It assumes that the blocks are able to communicate with each other as 
desired. It must store all inchworms, the desired structure, and the current map as interpreted
by "the structure" a.k.a. the blocks themselves. This simulation assumes that inchworms 
"get updated" by interfacing with the existing structure. It also assumes that the seed block 
is located at its final position from the beginning. 
'''

import copy
from config import *
# from inchworm_data import Inchworm
# import map_data
from path_conversion import * 
from search import search


blocks_placed = []
found_structures = []
misc_blocks = []

# Leg locations for the inchworm. Point is the position of the leading leg and prev_point is the position of the second leg
point = CURRENT_LOC
prev_point = point

class SimData: 
    def __init__(): 
        blocks_placed = []
        found_structures = []
        misc_blocks = []

        # Leg locations for the inchworm. Point is the position of the leading leg and prev_point is the position of the second leg
        point = CURRENT_LOC
        prev_point = point
        pass

def append(listName: list, appendedThing): 
    listName.append(appendedThing)

def run_sim(): 

    # initialize the map as the blocks/structure knows it
    # empty_map = map_data.initialize_grid_with_structures()
    existing_inchworms = []
    initialized_inchworms = []


    # Initialize inchworms 
    # inchworm_1 = Inchworm(1, CURRENT_ORIENTATION, None, empty_map, CURRENT_LOC)
    # existing_inchworms.append(inchworm_1)


    #TODO: could set up for loop to initialize desired num of inchworms 