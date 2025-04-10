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
from colorama import Fore, init
import numpy as np
import itertools
init(autoreset=True)

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
        self.cleared_path_flags = {}
        
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
        if (inchworm.leading_foot_loc == inchworm.goal and inchworm.paths) or inchworm.state.value == 3: 
            if (self.current_map[x][y][z] == map_data.GridStatus.INCOMING_BLOCK.value) or (self.current_map[x][y][z] == map_data.GridStatus.WALKABLE.value):
                # Update current_map by clearing the iw path 
                self.current_map = map_data.rm_inchworm_path_from_grid(self.current_map, inchworm.paths, inchworm.id)
                # Update current_map w new block 
                self.current_map == map_data.update_grid_status(self.current_map, [x, y, z])

            # Send current_map to IW 
            inchworm.current_map = copy.deepcopy(self.current_map)

            print(Fore.GREEN + f"Struct should have sent its map to IW {inchworm.id}")
            self.cleared_path_flags[inchworm.id] = True # Path is cleared flag, meaning struct is set to receive updates with a new path 
            return True

    def new_IW_paths_received(self, inchworm): 
        # Make sure the previous path is cleared at least once before this
        if self.cleared_path_flags[inchworm.id] and inchworm.paths:  #inchworm.paths and inchworm.leading_foot_loc == inchworm.goal: 
            self.current_map = map_data.set_inchworm_path_to_grid(self.current_map, inchworm.paths, inchworm.id)
            x, y, z = inchworm.goal
            self.current_map[x][y][z] == map_data.update_grid_status(self.current_map, [x, y, z], map_data.GridStatus.INCOMING_BLOCK.value)
            print(Fore.GREEN + f"struct's map updated w new IW {inchworm.id} path")
            self.cleared_path_flags[inchworm.id] = False # This IW's paths now exist on the struct's map again
            return True
        else: 
            # print("struct did not receive new IW path")
            return False
        # get path & new incoming block from iw - DIFFERENT FUNC 
        # update current map with incoming block and paths 

    def detect_IW_collision(self): 
        """Raises an error if any of the inchworm feet are in the location of the other inchworms."""
        # Compare 2 inchworms at a time from the list of all existing inchworms. 
        for a, b in itertools.combinations(self.existing_inchworms, 2):
            # Bitwise comparison of foot locations. If any foot loc is the same as any other foot loc, it is true.
            if {tuple(a.leading_foot_loc), tuple(a.lagging_foot_loc)} & {tuple(b.leading_foot_loc), tuple(b.lagging_foot_loc)}:
                raise RuntimeError(Fore.GREEN + f"COLLISION between IW{a.id} & IW{b.id} at {a.leading_foot_loc}, {a.lagging_foot_loc} and {b.leading_foot_loc}, {b.lagging_foot_loc}")

    def spawn_inchworms(self, num_inchworms: int): 
        """
        Args: 
            num_inchworms (int): number of inchworms building the structure
        """
        for i in range(num_inchworms): 
            self.existing_inchworms.append(Inchworm(IW_ORIENTATIONS[i], self.final_structure, IW_LOCS[i]))
            self.existing_inchworms[i].current_map = map_data.update_grid_status(self.existing_inchworms[i].current_map, SEED_BK)

            # For however many IWs exist, store flag in dictionary 
            self.cleared_path_flags[i+1] = False # The key is i+1 to correspond to the IW ID
        # print(Fore.GREEN + "inchworms spawned")
        print(Fore.GREEN + f"existing inchworms: {self.existing_inchworms}")

    def get_next_steps(self): 
        """
        Returns all of the next steps that all inchworms will be taking
        """
        for inchworm in self.existing_inchworms: 
            return inchworm.get_next_point() # x, z, y
        # TODO: return a list of all the next points of travel

    def generate_demo(self):
        coordinates = []
        coordinates.append([SEED_BK[0]+1, SEED_BK[1], SEED_BK[2]])
        for z in (2, 3): 
            coordinates.append([SEED_BK[0], SEED_BK[1], z])
            coordinates.append([SEED_BK[0]+1, SEED_BK[1], z])
        return coordinates            
    
    def generate_pyramid(self, base_size=5):
        """
        Generates a quarter section of a 10-by-10 pyramid of blocks (if base_size = 5).
        Args:
            base_size (int): Base size of the quarter of the pyramid. 
        Returns:
            pyramid: list of list [x, y, z]. List block locations. 
        """
        pyramid = []
        # Each layer
        for z in range(base_size):
            # Each row
            for x in range(base_size - z):
                # Each column
                for y in range(base_size - z):
                    pyramid.append([x+SEED_BK[0], y+SEED_BK[1], z+1])
        return pyramid
    
    def generate_diag_pyramid(self, base_size=5):
        """
        Generates a diagonal pyramid of blocks depending on the base_size.
        Args:
            base_size (int): Base size of the quarter of the pyramid. 
        Returns:
            diag_pyramid: list of list [x, y, z]. List block locations. 
        """
        diag_pyramid = []
        seed_x, seed_y, seed_z = SEED_BK

        if base_size % 2 == 0:
            raise ValueError("base_size must be an odd number for a symmetric pyramid.")

        radius = base_size // 2

        for z in range(radius + 1):  # Number of layers = radius + 1
            layer_radius = radius - z
            for dx in range(-layer_radius, layer_radius + 1):
                for dy in range(-layer_radius, layer_radius + 1):
                    if abs(dx) + abs(dy) <= layer_radius:
                        x = seed_x + dx
                        y = seed_y + dy
                        diag_pyramid.append([x, y, seed_z + z + 1])  # z+1 to build above seed
        return diag_pyramid
    
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
            coordinates_from_file.append([(x/10)-60,(y/10)+20, z/10])
            # blocks_placed.append(coordinates_from_file)

    return coordinates_from_file

