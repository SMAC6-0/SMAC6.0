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
import map_data
from search import search
from inchworm_data import Inchworm
from colorama import Fore, Style, init

from inchworm_control.blueprint import blueprint 

class SimData: 
    def __init__(self): 
        self.seed_block = [] # TODO: algo to deduce seed block based on what is in the sim (based on goal struct)
        self.blocks_placed = [] # list of blocks user places in sim
        self.incoming_blocks = [] 
        self.all_paths = []
        self.final_structure = map_data.initialize_grid() # Struct IWs are trying to build 
        self.current_map = map_data.initialize_grid() # overall progress towards final struct
        
        self.existing_inchworms = []
        self.initialized_inchworms = []
        self.cleared_path_flag = False
        
    def generate_final_structure_map(self, blocks_placed: list[list[int]]): 
        """Convert blocks placed in sim to 3D list parsable everywhere else. Evaluates the seed block as the first 
        block to be placed according to blueprint algorithm. """
        # Store final struct in 3D list. Update Final Map with all other blocks. (Supply depot & seed bk already marked)
        blocks_placed.sort(key=lambda lowest: lowest[2]) # sort the blocks placed so that the ones with the lowest z coords are update in the map first 
        for block in blocks_placed: 
            self.final_structure = map_data.update_grid_status(self.final_structure, (block[0], block[1], block[2]))
    
    def send_map_to_IW(self, inchworm): 
        """
        If the IW is at its goal, structure removes the IW path from its map and sends the IW a map snapshot
        """
        x, y, z = inchworm.leading_foot_loc
        # TODO: far future: check if IW is adjacent to blocks (use map_data.set_neighbors)
        # If yes, get newly placed block's coords from iw 

        # Structure verifies that block is in correct location 
        if inchworm.leading_foot_loc == inchworm.goal and inchworm.paths: 
            if (self.current_map[x][y][z] == map_data.GridStatus.INCOMING_BLOCK.value) or (self.current_map[x][y][z] == map_data.GridStatus.WALKABLE.value):
                # Update current_map by clearing the iw path 
                self.current_map = map_data.rm_inchworm_path_from_grid(self.current_map, inchworm.paths)

                # Update current_map w new block 
                self.current_map == map_data.update_grid_status(self.current_map, [x, y, z])

                # Send current_map to IW 
                inchworm.current_map = copy.deepcopy(self.current_map)
                print(Fore.GREEN + "Struct should have sent its map to the IW")
                self.cleared_path_flag = True
                return True
        # return self.current_map # TODO: maybe unnecessary

    def new_IW_paths_received(self, inchworm): 
        # Make sure the previous path is cleared at least once before this
        if self.cleared_path_flag and inchworm.paths:  #inchworm.paths and inchworm.leading_foot_loc == inchworm.goal: 
            self.current_map = map_data.set_inchworm_path_to_grid(self.current_map, inchworm.paths, inchworm.id)
            x, y, z = inchworm.goal
            self.current_map[x][y][z] == map_data.update_grid_status(self.current_map, [x, y, z], map_data.GridStatus.INCOMING_BLOCK)
            print(Fore.GREEN + "struct's map updated w new IW path")
            self.cleared_path_flag = False
            return True
        else: 
            # print("struct did not receive new IW path")
            return False
        # get path & new incoming block from iw - DIFFERENT FUNC 
        # update current map with incoming block and paths 

    def spawn_inchworms(self, num_inchworms: int): 
        """
        Args: 
            num_inchworms (int): number of inchworms building the structure
        """
        for i in range(num_inchworms): 
            self.existing_inchworms.append(Inchworm(CURRENT_ORIENTATION, self.final_structure, CURRENT_LOC))
            self.existing_inchworms[i].current_map = map_data.update_grid_status(self.existing_inchworms[i].current_map, SEED_BK)
        print(Fore.GREEN + "inchworms spawned")

    def get_next_steps(self): 
        """
        Returns all of the next steps that all inchworms will be taking
        """
        for inchworm in self.existing_inchworms: 
            return inchworm.get_next_point() # x, z, y
        # TODO: return a list of all the next points of travel
    
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
        for z in range(base_size):
            # Each row
            for x in range(base_size - z):
                # Each column
                for y in range(base_size - z):
                    pyramid.append((x+10, y+10, z+1))
        return pyramid
    
    def generate_building(self): 
        simplify_and_ensure_connectivity("inchworm_control/block_simulation/Assets/Structures/empire.xyz", "inchworm_control/block_simulation/Assets/Structures/empire2.xyz", grid_size=10)
        coordinates = read_and_place_voxels_from_file("inchworm_control/block_simulation/Assets/Structures/empire2.xyz")
        return coordinates

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
            coordinates_from_file.append(((x/10)-60,(y/10)+20, z/10))
            # blocks_placed.append(coordinates_from_file)

    return coordinates_from_file

