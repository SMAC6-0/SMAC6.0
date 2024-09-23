# Install Ursina before using this "pip install ursina"
# Tutorial https://www.youtube.com/watch?v=DHSRaVeQxIk
# What are you doing here?!

# Imports
from ursina import *
from ursina.prefabs.first_person_controller import FirstPersonController
import random 
from search import search
from path_conversion import * 
from config import CURRENT_LOC, BD_LOC, DEMO
import copy 

app = Ursina()

# stltovoxel /Users/canguven/Downloads/tower.stl /Users/canguven/Downloads/yarrak.xyz  --resolution 50

# Variables
sky_texture = load_texture("Assets/Textures/skybox.png")
white_block_texture = load_texture("Assets/Textures/white_block.png")
smart_block_texture = load_texture("Assets/Textures/smart_block_new.png")
smart_block_texture_step = load_texture("Assets/Textures/smart_block_step_new.png")
smart_block_texture_red = load_texture("Assets/Textures/smart_block_red_outline.png")
smart_block_texture_blue = load_texture("Assets/Textures/smart_block_blue_outline.png")
smart_block_texture_yellow = load_texture("Assets/Textures/smart_block_yellow_outline.png")
smart_block_texture_green = load_texture("Assets/Textures/smart_block_neon_green_outline.png")
smart_block_outline = load_texture("Assets/Textures/smart_block_outline.png")

# Color Steps
smart_block_texture_step_red = load_texture("Assets/Textures/smart_block_red_step.png")
smart_block_texture_step_blue = load_texture("Assets/Textures/smart_block_blue_step.png")
smart_block_texture_step_yellow = load_texture("Assets/Textures/smart_block_yellow_step.png")
smart_block_texture_step_green = load_texture("Assets/Textures/smart_block_green_step.png") 

# More Variables
last_colored_block = None
last_block_original_texture = None
last_colored_block_2 = None
last_block_original_texture_2 = None
window.exit_button.visible = False
key_g_pressed = False  
key_t_pressed = False  
key_n_pressed = False 
key_p_pressed = False
placed_block = None 
spawned = False
spawn_x, spawn_y, spawn_z = 0, 0, 0
blocks_placed = []
found_structures = []
coords_to_spawn = []
misc_blocks = []
complete_steps = []
path_steps = None 
number = 0 
# Leg locations for the inchworm. Point is the position of the leading leg and prev_point is the position of the second leg
point = CURRENT_LOC
prev_point = point

# Updates every frame
def update():
    global blocks_placed, key_g_pressed, key_t_pressed,key_p_pressed, key_n_pressed, coords_to_spawn, last_colored_block, last_block_original_texture, last_colored_block_2, last_block_original_texture_2, found_structures, misc_blocks, prev_point, point, placed_block, spawned, spawn_x, spawn_y, spawn_z, goal, number, path_steps

    # Generate the pyramid coordinates
    if held_keys["g"] and not key_g_pressed:
        pyramid_coordinates = generate_pyramid(5)
        # Spawn cubes for each coordinate in the pyramid
        for x, y, z in pyramid_coordinates:
            spawn_cube(x, y, z,'')  # Replace
            cube = Voxel(position=Vec3(x, y, z),  texture=smart_block_texture)
            blocks_placed.append(cube.position) 
        key_g_pressed = True  # Set the flag to True after printing
    
    if not held_keys["g"]:
        key_g_pressed = False

    if held_keys["t"] and not key_t_pressed:
        # To use this function, just pass the path to your 'empire.xyz' file
        # file_path = '/Users/canguven/Downloads/yarrak.xyz'
        # UPDATE THE FILES HERE
        simplify_and_ensure_connectivity('/Users/canguven/Downloads/empire.xyz', '/Users/canguven/Downloads/empire2.xyz', grid_size=10)
        coordinates = read_and_place_voxels_from_file('/Users/canguven/Downloads/empire2.xyz')
        # pyramid_coordinates = generate_pyramid(5)
        # # Spawn cubes for each coordinate in the pyramid
        # for x, y, z in pyramid_coordinates:
        #     spawn_cube(x, y, z,'')  # Replace
        #     cube = Voxel(position=Vec3(x, y, z),  texture=smart_block_texture) 
        #     blocks_placed.append(cube.position) 
        for x, y, z in coordinates:
            spawn_cube(x, y, z,'')  # Replace
            cube = Voxel(position=Vec3(x, y, z),  texture=smart_block_texture)
            blocks_placed.append(cube.position) 
        key_t_pressed = True  # Set the flag to True after printing
    
    if not held_keys["t"]:
        key_t_pressed = False

    # Search(Look) for structures
    if held_keys["l"]:
        found_structures, misc_blocks = show_structures()

    # Generate paths and inchworm steps
    if held_keys["p"] and not key_p_pressed:
        spawn_cube(BD_LOC[0], BD_LOC[1], BD_LOC[2], 'n')
        sorted_list = sorted(misc_blocks, key=lambda coordinate: coordinate[1])
        coords_to_spawn, path_steps , goal= dev_total_path_steps(found_structures, sorted_list)
        step_getter(path_steps)
        for point in goal:
            point[1] += 1  # Increment the second value

        key_p_pressed = True

    if not held_keys["p"] and key_p_pressed:
        key_p_pressed = False

    if held_keys["n"] and not key_n_pressed and coords_to_spawn:
        (point, holding_block) = coords_to_spawn.pop(0)  # Get the next point
        x, z, y = point
        if holding_block:
            z = z+1

        already_placed_block = None
        for e in scene.entities:
            if hasattr(e, 'position') and e.position == Vec3(x, z, y):
                already_placed_block = e
                break

        # first check if the last_colored_block was spawned bc we need to delete that block from blocks_placed and despawn it
        if spawned:
            delete_cube(spawn_x, spawn_z, spawn_y)
            spawned = False

        # If there is a previously colored block, restore to original texture
        elif last_colored_block is not None:
                last_colored_block.texture = last_block_original_texture
        if already_placed_block:

            # Store the original texture before changing it
            last_block_original_texture = already_placed_block.texture
            if (already_placed_block.position.x, already_placed_block.position.y, already_placed_block.position.z) == tuple(map(float, goal[number])):
                last_block_original_texture = smart_block_texture
                new_texture = smart_block_texture 
                number += 1
            else:
                new_texture = check_block_color(already_placed_block.position.x, already_placed_block.position.y, already_placed_block.position.z)
            already_placed_block.texture = new_texture
            last_colored_block = already_placed_block

        else:
            spawned = True
            spawned_block = spawn_cube(x, z, y,'step')
            spawn_x, spawn_y, spawn_z = x, y, z
            last_colored_block = spawned_block
            last_block_original_texture = smart_block_texture

        key_n_pressed = True

    if not held_keys["n"] and key_n_pressed:
        x2, z2, y2 = prev_point
        already_placed_block_2 = None
        for e in scene.entities:
            if hasattr(e, 'position') and e.position == Vec3(x2, z2, y2):
                already_placed_block_2 = e
                break

        # If there is a previously colored block, restore to original texture
        if last_colored_block_2 is not None:
            last_colored_block_2.texture = last_block_original_texture_2
        
        if already_placed_block_2:
            # Store the original texture before changing it
            last_block_original_texture_2 = already_placed_block_2.texture
            new_texture2 = check_block_color(already_placed_block_2.position.x, already_placed_block_2.position.y, already_placed_block_2.position.z)
            already_placed_block_2.texture = new_texture2
            last_colored_block_2 = already_placed_block_2
        else:
            spawned_block_2 = spawn_cube(x2, z2, y2,'step')
            last_colored_block_2 = spawned_block_2
            last_block_original_texture_2 = smart_block_texture

        prev_point = point
        key_n_pressed = False



# writes steps to a txt file              
def step_getter(steps):
    complete_steps = copy.deepcopy(steps)
    file_path = "steps.txt"
    
    with open(file_path, 'w') as file:
        for step in complete_steps:
            file.write(f"{step}\n")

# Searches for known structures and changes the color of strucutres found 
def show_structures():
    found_structures, misc_blocks = search(blocks_placed)
    for structure in found_structures:
        structure_pos = structure[1]  
        structure_name = structure[0] #string
        for block in structure_pos:
            delete_cube(block[0], block[1], block[2])
            spawn_cube(block[0], block[1], block[2], structure_name[-1])
            #WHEN WE ARE IMPLEMENTING THE COLORS  spawn_cube(block[0], block[1], block[2], color_index)
    for block in misc_blocks:
            delete_cube(block[0], block[1], block[2])
            spawn_cube(block[0], block[1], block[2], 'misc')
    return found_structures, misc_blocks

# Voxel (block) properties
class Voxel(Button):
    def __init__(self, position = (0, 0, 0), texture = white_block_texture):
        super().__init__(
            parent = scene,
            position = position,
            model = "Assets/Models/Block",
            origin_y = 0.5,
            texture = texture,
            color = color.color(0, 0, random.uniform(0.9, 1)),
            highlight_color = color.light_gray,
            scale = 0.5
        )

    # What happens to blocks on inputs
    def input(self,key):
        if self.hovered:
            if key == "left mouse down":
                voxel = Voxel(position = self.position + mouse.normal, texture = smart_block_texture) 
                # only add blocks above field
                if(voxel.position[1] > 0):
                    blocks_placed.append(voxel.position) 
            if key == "right mouse down":
                try: 
                    blocks_placed.remove(self.position)
                except Exception as e: 
                    print("Block not found")
                destroy(self)

# Skybox
class Sky(Entity):
    def __init__(self):
        super().__init__(
            parent = scene,
            model = "Sphere",
            texture = sky_texture,
            scale = 150,
            double_sided = True
        )


# HELPER FUNCTIONS
        
def simplify_and_ensure_connectivity(input_file_path, output_file_path, grid_size):
    """
    Simplifies an XYZ file and ensures each voxel is at least connected to one other voxel.

    Parameters:
    - input_file_path: Path to the input XYZ file.
    - output_file_path: Path to the output simplified XYZ file.
    - grid_size: Size of the grid cell for downsampling and connectivity checks.
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

        
# Generates a quarter section of a 10-by-10 pyramid of blocks
def generate_pyramid(base_size):
    pyramid = []
    # Each layer
    for y in range(base_size):
        # Each row
        for x in range(base_size - y):
            # Each column
            for z in range(base_size - y):
                pyramid.append((x+10, y+1, z+10))
    return pyramid

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




# Checks the color of the block at the specified position
# This is used to simulate the steping on a already placed block and def check_block_color(x, y, z):
def check_block_color(x, y, z):
    block_color = None
    target_position = Vec3(x, y, z)
    existing_cube_texture = None
    for e in scene.entities:
        if hasattr(e, 'position') and e.position == target_position:
            if hasattr(e, 'texture'):  # Assuming entities have a 'texture' attribute
                existing_cube_texture = e.texture
            break  # Stop searching once a block at the target position is found

    # If the position is occupied for stepping 
    if existing_cube_texture is not None:
        if existing_cube_texture == smart_block_texture: 
                block_color = smart_block_texture_step
        elif existing_cube_texture == white_block_texture: 
                block_color = smart_block_texture_step_red
        elif existing_cube_texture == smart_block_texture_red: 
                block_color = smart_block_texture_step_red
        elif existing_cube_texture == smart_block_texture_green: 
                block_color = smart_block_texture_step_green
        elif existing_cube_texture == smart_block_texture_blue: 
                block_color = smart_block_texture_step_blue 
        elif existing_cube_texture == smart_block_texture_yellow: 
                block_color = smart_block_texture_step_yellow
        elif existing_cube_texture == smart_block_texture_step: 
                block_color = smart_block_texture
        elif existing_cube_texture == smart_block_texture_step_red:
                block_color = smart_block_texture_step_red
        elif existing_cube_texture == smart_block_texture_step_green:
                block_color = smart_block_texture_step_green
        elif existing_cube_texture == smart_block_texture_step_blue:
                block_color = smart_block_texture_step_blue
        elif existing_cube_texture == smart_block_texture_step_yellow:      
                block_color = smart_block_texture_step_yellow
        elif existing_cube_texture == smart_block_outline:      
                block_color = smart_block_texture

        else:
                print(f"Unexpected texture: {existing_cube_texture}")  # Debugging line

    return block_color

# spawns a cude in the simulation at the specified position and with the specified color
def spawn_cube(x, y, z, color_index):
    # check if the position is already occupied
    target_position = Vec3(x, y, z)

    # Assign color_index based on the input
    if color_index == 'n':
        color_index = smart_block_texture_red
    elif color_index == 's':
        color_index = smart_block_texture_blue
    elif color_index == 'e':
        color_index = smart_block_texture_yellow
    elif color_index == 'w':
        color_index = smart_block_texture_green
    elif color_index == 'step':
        color_index = smart_block_texture_step
    elif color_index == 'misc':
        color_index = smart_block_outline    
    else:
        color_index = smart_block_texture

    # Spawn the cube
    new_cube = Voxel(position=target_position, texture=color_index)
    blocks_placed.append(target_position)  # Optionally update the blocks_placed list

# delete a block from the simulation at the specified position
def delete_cube(x, y, z):
    target_position = Vec3(x, y, z)
    for e in scene.entities:
        if hasattr(e, 'position') and e.position == target_position:
            destroy(e)
            if target_position in blocks_placed:
                blocks_placed.remove(target_position)
            break

# Increase the numbers for a bigger field. 
if DEMO:
    for z in range(5): 
        for x in range(6): 
            voxel = Voxel(position = (x, 0, z))
else:
     for z in range(21): # 5
        for x in range(21): # 6 
            voxel = Voxel(position = (x, 0, z))

def look_at(target_pos, player_pos):
    if isinstance(target_pos, tuple):
        target_pos = Vec3(*target_pos)
    if isinstance(player_pos, tuple):
        player_pos = Vec3(*player_pos)

    direction = target_pos - player_pos
    yaw = math.atan2(direction.x, direction.z)
    yaw_degrees = math.degrees(yaw)
    
    distance_horizontal = math.sqrt(direction.x**2 + direction.z**2)
    pitch = math.atan2(direction.y, distance_horizontal)
    pitch_degrees = -math.degrees(pitch) 
    
    return pitch_degrees, yaw_degrees


class FlyingFirstPersonController(FirstPersonController):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.flying_enabled = False  

    def update(self):
        super().update()  
        self.handle_flying_input()

    def handle_flying_input(self):
        # Toggle flying mode with a specific key (e.g., 'f')
        if held_keys['f']:
            print("Flying enabled" if self.flying_enabled else "Flying disabled")
            self.flying_enabled = not self.flying_enabled
            self.gravity = 0 if self.flying_enabled else 1  

        # Handle vertical movement when flying is enabled
        if self.flying_enabled:
            # Change the psotion of the player here: 
            if held_keys['q']:  
                self.position += Vec3(0, 0.1, 0)  
            if held_keys['e']:  # Move down
                self.position += Vec3(0, -0.1, 0)
            if held_keys['1']:  
                self.position = Vec3(20, 10, 0)  
                pitch_degrees, yaw_degrees = look_at((10,1,10), (20, 10, 0))
                player.rotation_y = yaw_degrees
                player.camera_pivot.rotation_x = pitch_degrees
            if held_keys['2']:  
                self.position = Vec3(20, 10, 20) 
                pitch_degrees, yaw_degrees = look_at((10,1,10), (20, 10, 20))
                player.rotation_y = yaw_degrees
                player.camera_pivot.rotation_x = pitch_degrees
            if held_keys['3']:  
                self.position = Vec3(0,10, 20)  
                pitch_degrees, yaw_degrees = look_at((10,1,10), (0,10, 20))
                player.rotation_y = yaw_degrees
                player.camera_pivot.rotation_x = pitch_degrees
            if held_keys['4']:  
                self.position = Vec3(0, 10, 0)  
                pitch_degrees, yaw_degrees = look_at((10,1,10), (0, 10, 0))
                player.rotation_y = yaw_degrees
                player.camera_pivot.rotation_x = pitch_degrees


player = FlyingFirstPersonController()
sky = Sky()

app.run()