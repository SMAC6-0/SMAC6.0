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



class SimData: 
    def __init__(self): 
        self.blocks_placed = []
        self.found_structures = []
        self.misc_blocks = []
        self.coords_to_spawn = [] # the complete path
        # self.path_steps = None 
        self.goal = []
        self.goal_progress_index = 0

        # TODO: make these custom to the inchworm 
        # Leg locations for the inchworm. Point is the position of the leading leg and prev_point is the position of the second leg
        self.point = CURRENT_LOC
        self.prev_point = self.point
        # self.holding_block

        # initialize the map as the blocks/structure knows it
        # empty_map = map_data.initialize_grid_with_structures()
        existing_inchworms = []
        initialized_inchworms = []
        

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
    
    def generate_building(self): 
        simplify_and_ensure_connectivity("inchworm_control/block_simulation/Assets/Structures/empire.xyz", "inchworm_control/block_simulation/Assets/Structures/empire2.xyz", grid_size=10)
        coordinates = read_and_place_voxels_from_file("inchworm_control/block_simulation/Assets/Structures/empire2.xyz")
        return coordinates



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

def simplify_and_ensure_connectivity(input_file_path, output_file_path, grid_size):
    """
    Simplifies an XYZ file and ensures each voxel is at least connected to one other voxel.

    Args:
        input_file_path: Path to the input XYZ file.
        output_file_path: Path to the output simplified XYZ file.
        grid_size: Size of the grid cell for downsampling and connectivity checks.
    """
    voxel_grid = {}  # Use a dictionary to represent a sparse grid
    with open(input_file_path, 'r') as file:
        for line in file:
            x, y, z = map(float, line.strip().split())
            # Convert coordinates to a grid position
            grid_pos = (round(x / grid_size), round(y / grid_size), round(z / grid_size))
            
            # Check for connectivity: Ensure at least one neighbor exists
            neighbors = [
                (grid_pos[0] + dx, grid_pos[1] + dy, grid_pos[2] + dz)
                for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
                if not (dx == dy == dz == 0)  # Exclude the voxel itself
            ]
            if any(neighbor in voxel_grid for neighbor in neighbors):
                voxel_grid[grid_pos] = True
            else:
                # If no neighbors, check if it's the first voxel; if so, add it anyway to start the connectivity chain
                if not voxel_grid:
                    voxel_grid[grid_pos] = True

    # Write the simplified and connected voxels to the output file
    with open(output_file_path, 'w') as file:
        for grid_pos in voxel_grid.keys():
            # Convert grid positions back to coordinates
            x, y, z = [coord * grid_size for coord in grid_pos]
            file.write(f"{x} {y} {z}\n")

# Example usage
# simplify_and_ensure_connectivity('path/to/your/original_file.xyz', 'path/to/your/simplified_file.xyz', grid_size=10)

        

def read_and_place_voxels_from_file(file_path):
    coordinates_from_file = []

    with open(file_path, 'r') as file:
        for line in file:
            # Split the line into coordinates and convert them to integers
            x, y, z = [int(float(coord)) for coord in line.strip().split()]
            
            # Your voxel placement logic here
            # Replace `spawn_cube` and `Voxel` with your actual function and class names
            # Assuming `spawn_cube` is a function to call for placing the cube, which you might or might not need
            # spawn_cube(x, y, z, '')  # Uncomment and use if needed
            # cube = Voxel(position=Vec3(x, y, z), texture=smart_block_texture)
            coordinates_from_file.append(((x/10)-60, z/10,(y/10)+20))
            # blocks_placed.append(coordinates_from_file)

    return coordinates_from_file