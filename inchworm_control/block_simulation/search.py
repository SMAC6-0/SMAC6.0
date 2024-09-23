import numpy as np
from known_structures import known_structures

# shifts all block positions so that corner of strucutre is at (0, 0, 0)
def shift_to_origin(blocks_placed):
    smallest_x = float('inf')
    smallest_y = float('inf')
    
    for position in blocks_placed:
        if(position[0] < smallest_x):
            smallest_x = position[0]
        if(position[2] < smallest_y):
            smallest_y = position[2]
    
    for position in blocks_placed:
        position[0] -= smallest_x
        position[2] -= smallest_y

# converts blocks_placed into a 3D array of blocks
def prepare_world_data(blocks_placed, world_size=(20,80,20)):
    world_data = np.zeros(world_size)
    for block in blocks_placed:
        # Convert the Vec3 positions to integer indices
        x, y, z = int(block.x), int(block.y), int(block.z)
        world_data[x, y, z] = 1  # Mark the presence of a block
    return world_data

# The new function to extract coordinates
def extract_coordinates(world_data):
    coordinates = []
    for x in range(world_data.shape[0]):
        for y in range(world_data.shape[1]):
            for z in range(world_data.shape[2]):
                if world_data[x, y, z] == 1:
                    coordinates.append((x, y, z))
    return coordinates

# converts structures into a 3D array of blocks
def structure_to_3d_array(structure, size=(3,3,3)):
    array = np.zeros(size)
    for pos in structure:
        array[pos] = 1  # or any other identifier
    return array

# implementation of 3D convolution
def convolve_3d(world_data, kernel):
    kernel_size = kernel.shape
    result = np.zeros_like(world_data)

    for x in range(world_data.shape[0] - kernel_size[0]):
        for y in range(world_data.shape[1] - kernel_size[1]):
            for z in range(world_data.shape[2] - kernel_size[2]):
                result[x, y, z] = np.sum(world_data[x:x+kernel_size[0], y:y+kernel_size[1], z:z+kernel_size[2]] * kernel)
    
    return result

# uses convolve_3d to find instances of the known structures within the world data
def find_structures(world_data, structures):
    found_structures = []
    for known_structure in structures:
        structure_kernel = structure_to_3d_array(known_structure)
        convolution_result = convolve_3d(world_data, structure_kernel)
        threshold = np.sum(structure_kernel)  # Adjust the threshold as needed
        matches = np.where(convolution_result == threshold)

        for match in zip(*matches):
            found_structures.append(match)  # These are the starting points of matched structures
    
    return found_structures

# just takes in one known structure
def find_structures_simple(world_data, structure):
    found_structures = []
    structure_kernel = structure_to_3d_array(structure)
    convolution_result = convolve_3d(world_data, structure_kernel)
    threshold = np.sum(structure_kernel)  # Adjust the threshold as needed
    matches = np.where(convolution_result == threshold)

    for match in zip(*matches):
        found_structures.append(match) 
        
    return found_structures

def get_full_structure_coordinates(start_coord, structure):
    full_coords = []
    for rel_pos in structure:
        full_coords.append((start_coord[0] + rel_pos[0], start_coord[1] + rel_pos[1], start_coord[2] + rel_pos[2]))
    return full_coords


# main function
# blocks_placed is a list of Vec3 positions of blocks placed
# returns a list of lists of Vec3 positions of the found structures
# for example, if a 3-block staircase is found, the list will contain 3 Vec3 positions
def search(blocks_placed):
    all_found_structures = []
    world_data = prepare_world_data(blocks_placed)
    
    # Iterate through each structure type and its coordinates in the dictionary
    for structure_name, structure_coords_list in known_structures.items():
        # structure_coords_list is the list of coordinates for each structure
        while True:
            # Continuously search for the structure until none are found
            found_structures = find_structures_simple(world_data, structure_coords_list)
            if not found_structures:
                break  # No more structures of this type found

            for start_coord in found_structures:
                full_structure_coords = get_full_structure_coordinates(start_coord, structure_coords_list)
                # Check if all parts of the structure still exist in world_data
                if all(world_data[x, y, z] == 1 for x, y, z in full_structure_coords):
                    # tuple should be (structure name, [coords])
                    new_tuple = (structure_name, full_structure_coords)
                    all_found_structures.append(new_tuple)
                    for x, y, z in full_structure_coords:
                        world_data[x, y, z] = 0  # Remove block

    # convert world_data back to blocks_placed
    blocks_placed = extract_coordinates(world_data)
    return all_found_structures, blocks_placed