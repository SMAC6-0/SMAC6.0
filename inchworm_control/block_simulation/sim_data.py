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
# import sim


# blocks_placed = []
# found_structures = []
# misc_blocks = []

# # Leg locations for the inchworm. Point is the position of the leading leg and prev_point is the position of the second leg
# point = CURRENT_LOC
# prev_point = point

class SimData: 
    def __init__(self): 
        self.blocks_placed = []
        self.found_structures = []
        self.misc_blocks = []
        self.coords_to_spawn = [] # the complete path
        # self.path_steps = None 
        self.goal = []

        # TODO: make these custom to the inchworm 
        # Leg locations for the inchworm. Point is the position of the leading leg and prev_point is the position of the second leg
        self.point = CURRENT_LOC
        self.prev_point = self.point
        # self.holding_block

        # initialize the map as the blocks/structure knows it
        # empty_map = map_data.initialize_grid_with_structures()
        existing_inchworms = []
        initialized_inchworms = []
        pass

    def plan_path(self): 
        # TODO: transfer this function to the inchworm class 

        sorted_list = sorted(self.misc_blocks, key=lambda coordinate: coordinate[1])
        self.coords_to_spawn, path_steps , self.goal= dev_total_path_steps(self.found_structures, sorted_list)
        step_getter(path_steps)
        for point in self.goal:
            point[1] += 1  # Increment the second value

    def get_next_point(self): 
        (self.point, holding_block) = self.coords_to_spawn.pop(0)  # Get the next point
        x, z, y = self.point
        if holding_block:
            z = z+1
        return x, z, y
    
    def generate_pyramid(self, base_size):
        """
        Generates a quarter section of a 10-by-10 pyramid of blocks (if base_size = 5).
        Args:
            base_size (int): Base size of the quarter of the pyramid. 
        Returns:
            pyramid: list of tuple (x, y, z). List block locations. 
        """
        pyramid = []
        # Each layer
        for y in range(base_size):
            # Each row
            for x in range(base_size - y):
                # Each column
                for z in range(base_size - y):
                    pyramid.append((x+10, y+1, z+10))
        return pyramid



def run_sim(): 

    # initialize the map as the blocks/structure knows it
    # empty_map = map_data.initialize_grid_with_structures()
    existing_inchworms = []
    initialized_inchworms = []


    # Initialize inchworms 
    # inchworm_1 = Inchworm(1, CURRENT_ORIENTATION, None, empty_map, CURRENT_LOC)
    # existing_inchworms.append(inchworm_1)


    #TODO: could set up for loop to initialize desired num of inchworms 

def step_getter(steps):
    """
    Write the steps to steps.txt
    """
    complete_steps = copy.deepcopy(steps)
    file_path = "steps.txt"
    
    with open(file_path, 'w') as file:
        for step in complete_steps:
            file.write(f"{step}\n")
# sim.app.run()