# Install Ursina before using this "pip install ursina"
# Tutorial https://www.youtube.com/watch?v=DHSRaVeQxIk
# What are you doing here?!
# This file facilitates the operation of the simulation itself: frame updates, button presses, etc.

# Imports
from ursina import *
from ursina.prefabs.first_person_controller import FirstPersonController
import random 
from search import search
from config import CURRENT_LOC, BD_LOC1, BD_LOCS, SIMULATION
import copy 
from sim_data import SimData

app = Ursina()
sim_data = SimData()

# stltovoxel /Users/canguven/Downloads/tower.stl /Users/canguven/Downloads/yarrak.xyz  --resolution 50

# Variables
sky_texture = load_texture("Assets/Textures/Skybox.png")
white_block_texture = load_texture("Assets/Textures/white_block.png")
smart_block_texture = load_texture("Assets/Textures/smart_block_new.png")
smart_block_texture_step = load_texture("Assets/Textures/smart_block_step_new.png")
smart_block_texture_red = load_texture("Assets/Textures/smart_block_red_outline.png")
smart_block_texture_blue = load_texture("Assets/Textures/smart_block_blue_outline.png")
smart_block_texture_yellow = load_texture("Assets/Textures/smart_block_yellow_outline.png")
smart_block_texture_green = load_texture("Assets/Textures/smart_block_neon_green_outline.png")
smart_block_outline = load_texture("Assets/Textures/smart_block_outline.png")
seed_block_texture = load_texture("Assets/Textures/seed_block.png")

# Color Steps
smart_block_texture_step_red = load_texture("Assets/Textures/smart_block_red_step.png")
smart_block_texture_step_blue = load_texture("Assets/Textures/smart_block_blue_step.png")
smart_block_texture_step_yellow = load_texture("Assets/Textures/smart_block_yellow_step.png")
smart_block_texture_step_green = load_texture("Assets/Textures/smart_block_green_step.png") 

# Incoming Blocks / Steps 
incoming_step_texture = load_texture("Assets/Textures/incoming_path_red.png")
incoming_block_texture = load_texture("Assets/Textures/incoming_block.png")

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

spawned = False
spawn_x, spawn_y, spawn_z = 0, 0, 0



# Updates every frame
def update():
    global key_g_pressed, key_t_pressed,key_p_pressed, key_n_pressed, last_colored_block, last_block_original_texture, last_colored_block_2, last_block_original_texture_2, spawned, spawn_x, spawn_y, spawn_z


    # Generate the pyramid coordinates
    if held_keys["g"] and not key_g_pressed:
        pyramid_coordinates = sim_data.generate_pyramid(5)
        for x, y, z in pyramid_coordinates:
            spawn_cube(x, y, z,'')  
        key_g_pressed = True  # Set the flag to True after printing
    
    if not held_keys["g"]:
        key_g_pressed = False

    if held_keys["t"] and not key_t_pressed:
        coordinates = sim_data.generate_building()
        for x, y, z in coordinates:
            spawn_cube(x, y, z,'')  
        key_t_pressed = True  
    
    if not held_keys["t"]:
        key_t_pressed = False

    # Search(Look) for structures
    if held_keys["l"]:
        sim_data.generate_final_structure_map()
        sim_data.spawn_inchworms(1)
        # show_structures()

    # Generate paths and inchworm steps. Spawns the supply depot block. 
    if held_keys["p"] and not key_p_pressed:
        spawn_cube(BD_LOCS[0][0], BD_LOCS[0][1], BD_LOCS[0][2], 'n') # consider changing accessing the supply depot to be through sim_data.py
        for inchworm in sim_data.existing_inchworms:
            inchworm.plan_path_to_structure()
            show_IW_paths(inchworm)
        # seed_block = sim_data.existing_inchworms[0].goal[0] # for now, assume that the first block in the blueprint is the seed block
        # spawn_cube(seed_block[0], seed_block[1], seed_block[2], 'seed')
        key_p_pressed = True

    if not held_keys["p"] and key_p_pressed:
        key_p_pressed = False

    if held_keys["n"] and not key_n_pressed and sim_data.existing_inchworms[0].paths[sim_data.existing_inchworms[0].goal_progress_index]: # simulates the stepping of the leading leg
        # coords_to_spawn verifies that a path exists before trying to do anything

        # TODO: consider moving block tracking to sim_data
        for inchworm in sim_data.existing_inchworms: 
            x, z, y = inchworm.get_next_point() 
        
            # first check if the last_colored_block was spawned bc we need to delete that block from blocks_placed and despawn it
            if spawned:
                delete_cube(spawn_x, spawn_z, spawn_y)
                spawned = False

            # If there is a previously colored block, restore to original texture
            elif last_colored_block is not None:
                    last_colored_block.texture = last_block_original_texture

            # This checks if there are existing block entities at the next point 
            already_placed_block = None
            for e in scene.entities:
                if hasattr(e, 'position') and e.position == Vec3(x, z, y):
                    already_placed_block = e
                    break

            if already_placed_block: # When you aren't simulating walking with cube 

                last_block_original_texture = already_placed_block.texture # Store the original texture before changing it
                # TODO: modify path planning so that not every inchworm goes to every block (just do every other or split)

                # Checks for visuals at goal location
                if (already_placed_block.position.x, already_placed_block.position.y, already_placed_block.position.z) == inchworm.get_loc_in_path():
                    # IW reaches goal coords & places block 
                    last_block_original_texture = smart_block_texture
                    new_texture = smart_block_texture 
                else:
                    # The inchworm is not yet at the goal
                    new_texture = check_block_color(already_placed_block.position.x, already_placed_block.position.y, already_placed_block.position.z)
                already_placed_block.texture = new_texture
                last_colored_block = already_placed_block
    
            else: # Walking with block in empty space
                spawned = True
                spawned_block = spawn_cube(x, z, y,'step')
                spawn_x, spawn_y, spawn_z = x, y, z
                last_colored_block = spawned_block
                last_block_original_texture = smart_block_texture
          
        key_n_pressed = True

    if not held_keys["n"] and key_n_pressed:
        x2, z2, y2 = sim_data.existing_inchworms[0].lagging_foot_loc
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
        # print("leading foot loc ", sim_data.existing_inchworms[0].leading_foot_loc)
        # print("lagging foot loc ", sim_data.existing_inchworms[0].lagging_foot_loc)

        sim_data.existing_inchworms[0].lagging_foot_loc = sim_data.existing_inchworms[0].leading_foot_loc
        key_n_pressed = False

    if held_keys["m"]:
        for inchworm in sim_data.existing_inchworms: 
            inchworm.update_state()
            show_IW_paths(inchworm)

def show_IW_paths(inchworm):
    # First extract the next block the IW is going to place
    cell = inchworm.goal
    delete_cube(cell[0], cell[1], cell[2])
    spawn_cube(cell[0], cell[1], cell[2], 'incoming')

    # Then show the path the inchworm is going to take
    for step in range(len(inchworm.paths)-1): 
        cell = inchworm.paths[step][0]
        delete_cube(cell[0], cell[1], cell[2])
        spawn_cube(cell[0], cell[1], cell[2], 'path')

def show_structures():
    """
    Searches for known structures and changes the color of structures found. 
    """
    for inchworm in sim_data.existing_inchworms:
        inchworm.found_structures, inchworm.misc_blocks = search(sim_data.blocks_placed)
        for structure in inchworm.found_structures:
            structure_pos = structure[1]  
            structure_name = structure[0] #string
            for block in structure_pos:
                delete_cube(block[0], block[1], block[2])
                spawn_cube(block[0], block[1], block[2], structure_name[-1])
                #WHEN WE ARE IMPLEMENTING THE COLORS  spawn_cube(block[0], block[1], block[2], color_index)
        for block in inchworm.misc_blocks:
                delete_cube(block[0], block[1], block[2])
                spawn_cube(block[0], block[1], block[2], 'misc')

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

    # What happens to blocks on mouse inputs
    def input(self,key):

        if self.hovered:
            if key == "left mouse down":
                voxel = Voxel(position = self.position + mouse.normal, texture = smart_block_texture) 
                # only add blocks above field
                if(voxel.position[1] > 0):
                    xoxel = int(voxel.position.x)
                    yoxel = int(voxel.position.y)
                    zoxel = int(voxel.position.z)
                    sim_data.blocks_placed.append((xoxel, yoxel, zoxel))
                    print("pos: ", (xoxel, yoxel, zoxel))
            if key == "right mouse down":
                try: 
                    sim_data.blocks_placed.remove(self.position)
                except Exception as e: 
                    print("Block not found")
                destroy(self)
                
        if key == "escape":
            stop_simulation()

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
        elif existing_cube_texture == incoming_block_texture:
            block_color = smart_block_texture_step_red
        elif existing_cube_texture == incoming_step_texture:
            block_color = smart_block_texture_step_red
        elif existing_cube_texture == seed_block_texture:
            block_color = seed_block_texture
        else:
                print(f"Unexpected texture: {existing_cube_texture}")  # Debugging line

    return block_color


def stop_simulation():
    print("User pressed 'ESC'. Stopping simulation...")
    application.quit()

# spawns a cude in the simulation at the specified position and with the specified color
def spawn_cube(x, y, z, color_index):
    """
    Spawns a cube in the simulation at the specified xyz position and with the specified color. 
    Not always a smart block, but rather any sim update happening in a cube. 
    Args:
        color_index (str): 'n' for red, 'step' for green floor
    """
    # check if the position is already occupied
    target_position = Vec3(x, y, z)

    # Assign color_index based on the input. The first few are based on different substructures. 
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
    elif color_index == 'incoming':
        color_index = incoming_block_texture
    elif color_index == 'path':
        color_index = incoming_step_texture
    elif color_index == 'seed':
        color_index = seed_block_texture
    else:
        color_index = smart_block_texture
        sim_data.blocks_placed.append(int(target_position))  # Update the block information

    # Spawn the cube
    new_cube = Voxel(position=target_position, texture=color_index)

def delete_cube(x, y, z):
    """
    Delete a block from the simulation at the specified xyz position
    """
    target_position = Vec3(x, y, z)
    for e in scene.entities:
        if hasattr(e, 'position') and e.position == target_position:
            destroy(e)
            if target_position in sim_data.blocks_placed:
                sim_data.blocks_placed.remove(target_position)
            break

# Increase the numbers for a bigger field. 
if not SIMULATION:
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