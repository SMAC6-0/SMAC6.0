# Install Ursina before using this "pip install ursina"
# Tutorial https://www.youtube.com/watch?v=DHSRaVeQxIk
# What are you doing here?!
"""
This file facilitates the operation of the simulation itself: frame updates, button presses, etc.
Important note on coordinates. Voxels are x, z, y. They consider y to be upwards, rather than z. So, 
when spawning a block, a Vec3 is used with xzy rather than xyz. If it's not a Voxel, use xyz !
"""

# Imports
from ursina import *
from ursina.prefabs.first_person_controller import FirstPersonController
import random 
from search import search
from config import *
import copy 
from sim_data import SimData

app = Ursina()
sim_data = SimData()

# stltovoxel /Users/canguven/Downloads/tower.stl /Users/canguven/Downloads/yarrak.xyz  --resolution 50

# Variables
sky_texture = load_texture("Assets/Textures/Skybox.png")
skynt_texture = load_texture("Assets/Textures/sky_texture.png")
white_block_texture = load_texture("Assets/Textures/white_block.png")
smart_block_texture = load_texture("Assets/Textures/smart_block_new.png")
smart_block_texture_step = load_texture("Assets/Textures/smart_block_step_new.png")
supply_depot_texture = load_texture("Assets/Textures/smart_block_red_outline.png")
outline_texture_blue = load_texture("Assets/Textures/smart_block_blue_outline.png")
outline_texture_yellow = load_texture("Assets/Textures/smart_block_yellow_outline.png")
outline_texture_green = load_texture("Assets/Textures/smart_block_neon_green_outline.png")
final_struct_outline = load_texture("Assets/Textures/smart_block_outline.png")
seed_block_texture = load_texture("Assets/Textures/seed_block.png")

# Color Steps
outline_step_texture_red = load_texture("Assets/Textures/smart_block_red_step.png")
outline_step_texture_blue = load_texture("Assets/Textures/smart_block_blue_step.png")
outline_step_texture_yellow = load_texture("Assets/Textures/smart_block_yellow_step.png")
outline_step_texture_green = load_texture("Assets/Textures/smart_block_green_step.png") 
seed_block_texture_step = load_texture("Assets/Textures/seed_block_step.png") 

# Incoming Blocks / Steps 
incoming_path_texture = load_texture("Assets/Textures/incoming_path_blue.png")
incoming_path_block_texture = load_texture("Assets/Textures/incoming_path_smart.png")
incoming_path_seed_texture = load_texture("Assets/Textures/incoming_path_seed.png")
incoming_block_texture = load_texture("Assets/Textures/incoming_block.png")

# More Variables
window.exit_button.visible = False
key_g_pressed = False  
key_t_pressed = False  
key_n_pressed = False 
key_p_pressed, key_l_pressed = False, False
key_k_pressed = False


# Updates every frame
def update():
    global key_g_pressed, key_l_pressed, key_t_pressed,key_p_pressed, key_n_pressed, key_k_pressed


    # Generate the pyramid coordinates
    if held_keys["g"] and not key_g_pressed:
        pyramid_coordinates = sim_data.generate_pyramid(5)
        for coord in pyramid_coordinates:
            spawn_cube(coord)  
        key_g_pressed = True  # Set the flag to True after printing
    
    if not held_keys["g"]:
        key_g_pressed = False

    if held_keys["t"] and not key_t_pressed:
        coordinates = sim_data.generate_building()
        for coord in coordinates:
            spawn_cube(coord)  
        key_t_pressed = True  
    
    if not held_keys["t"]:
        key_t_pressed = False

    if held_keys["k"] and not key_k_pressed:
        coordinates = sim_data.generate_demo()
        for coord in coordinates:
            spawn_cube(coord)  
        key_k_pressed = True  
    
    if not held_keys["k"]:
        key_k_pressed = False

    # Search(Look) for structures
    if held_keys["l"] and not key_l_pressed:
        generate_final_structure()
        sim_data.spawn_inchworms(2)
        key_l_pressed = True

    if not held_keys["l"]:
        key_l_pressed = False

    # Generate paths and inchworm steps
    if held_keys["p"] and not key_p_pressed:
        for inchworm in sim_data.existing_inchworms:
            inchworm.plan_path()
            vis_IW_paths(inchworm)
        key_p_pressed = True

    if not held_keys["p"] and key_p_pressed:
        key_p_pressed = False

    if held_keys["z"]: 
        sky.model = "Assets/Models/Block"
        sky.texture = skynt_texture

    # Simulates stepping of the leading leg, communication, state machine, etc 
    if held_keys["n"] and not key_n_pressed: 
        for inchworm in sim_data.existing_inchworms:  

            if inchworm.paths: 
                x, y, z = inchworm.get_next_point() 
                # print("IW", inchworm.id, " leading foot loc: ", inchworm.leading_foot_loc, "lagging_foot_loc: ", inchworm.lagging_foot_loc)
            
                # If the IW is holding a block (the bool spawned) despawn that block from old location before it can be moved/respawned to next step
                if inchworm.spawned:
                    delete_cube(inchworm.prev_held_block_loc)
                    inchworm.spawned = False

                # Restore texture from "where the leading foot was" to the actual texture that cell is supposed to have
                elif inchworm.last_cell is not None:
                    inchworm.last_cell.texture = inchworm.last_bk_og_texture

                # This checks if there are existing block entities at the next leading foot location 
                already_placed_block = None
                for e in scene.entities:
                    if hasattr(e, 'position') and e.position == Vec3(x, z, y):
                        already_placed_block = e
                        break

                if already_placed_block: # When you aren't simulating walking with cube 

                    inchworm.last_bk_og_texture = already_placed_block.texture # Store the original texture before changing it

                    # Checks for visuals at goal location
                    if [already_placed_block.position.x, already_placed_block.position.y, already_placed_block.position.z] == inchworm.goal:
                        # IW reaches goal coords & places block 
                        pass
                        # last_block_original_texture = smart_block_texture
                        # new_texture = smart_block_texture 
                    else:
                        # The inchworm is not yet at the goal
                        new_texture = check_block_color(already_placed_block.position.x, already_placed_block.position.y, already_placed_block.position.z)
                    already_placed_block.texture = new_texture # whatever texture the block had before, now the IW is stepping on top of it 
                    inchworm.last_cell = already_placed_block # The new leading foot loc will now me moving from this last cell
        
                else: # Walking with block in empty space
                    inchworm.spawned = True
                    spawn_cube([x, y, z], smart_block_texture_step) # Spawn the block the IW is holding, with green to show the leading foot is holding it
                    inchworm.prev_held_block_loc = [x, y, z]
            
            IW_is_at_goal = sim_data.send_map_to_IW(inchworm)
            if IW_is_at_goal: # clear the path visually before 
                vis_IW_paths(inchworm, clear_path=True)
                if inchworm.leading_foot_loc != SEED_BK:
                    delete_cube(inchworm.leading_foot_loc)
                    spawn_cube(inchworm.leading_foot_loc)
            inchworm.update_state()
            if sim_data.new_IW_paths_received(inchworm):
                vis_IW_paths(inchworm)

        key_n_pressed = True

    # Simulate stepping of the lagging foot
    if not held_keys["n"] and key_n_pressed:
        for inchworm in sim_data.existing_inchworms:  

            if inchworm.paths: 
                x2, y2, z2 = inchworm.lagging_foot_loc
                already_placed_block_2 = None
                for e in scene.entities:
                    if hasattr(e, 'position') and e.position == Vec3(x2, z2, y2):
                        already_placed_block_2 = e
                        break

                # If there is a previously colored block, restore to original texture
                if inchworm.last_cell_2 is not None:
                    inchworm.last_cell_2.texture = inchworm.last_bk_og_texture_2
                
                if already_placed_block_2:
                    # Store the original texture before changing it
                    inchworm.last_bk_og_texture_2 = already_placed_block_2.texture
                    new_texture2 = check_block_color(already_placed_block_2.position.x, already_placed_block_2.position.y, already_placed_block_2.position.z)
                    already_placed_block_2.texture = new_texture2
                    inchworm.last_cell_2 = already_placed_block_2
                else:
                    inchworm.last_cell_2 = spawn_cube([x2, y2, z2], smart_block_texture_step)
                    inchworm.last_bk_og_texture_2 = smart_block_texture

                key_n_pressed = False


    if held_keys["m"]:
        # Begin the state machine !
        for inchworm in sim_data.existing_inchworms: 
            inchworm.update_state()
            vis_IW_paths(inchworm)

def show_structure_map(): 
    """Show how the structure views the map. """
    pass

def vis_IW_paths(inchworm, clear_path=False):
    """Visualize the IW's path, or clear it"""
    # First extract the next block the IW is going to place
    if inchworm.goal != SEED_BK: 
        delete_cube(inchworm.goal)
        spawn_cube(inchworm.goal, incoming_block_texture)

    # Then show the path the inchworm is going to take
    for step in range(len(inchworm.paths)-1): 
        cell = inchworm.paths[step]

        already_placed_block = None
        for e in scene.entities:
            if hasattr(e, 'position') and e.position == Vec3(cell[0], cell[2], cell[1]):
                already_placed_block = e
                break
        if clear_path: # clear the old inchworm path 
            transition = {
                incoming_path_texture : white_block_texture, 
                incoming_path_block_texture : smart_block_texture,
                incoming_path_seed_texture : seed_block_texture,
                incoming_block_texture : incoming_block_texture, 
                seed_block_texture_step : seed_block_texture,
                smart_block_texture_step : smart_block_texture_step, 
                outline_step_texture_red : white_block_texture, 
                # Maintain textures if repeating the cleared visualization: 
                white_block_texture : white_block_texture, 
                smart_block_texture : smart_block_texture,
                seed_block_texture : seed_block_texture,              
            }
            block_color = None
        else: # show the inchworm path
            transition = {
                white_block_texture : incoming_path_texture, 
                smart_block_texture : incoming_path_block_texture,
                seed_block_texture : incoming_path_seed_texture, 
                seed_block_texture_step : incoming_path_seed_texture,
                outline_step_texture_red : incoming_path_texture,
                smart_block_texture_step : incoming_path_block_texture,
                # Maintain textures if repeating the visualization: 
                incoming_block_texture : incoming_block_texture,
                incoming_path_texture : incoming_path_texture, 
                incoming_path_block_texture : incoming_path_block_texture,
                incoming_path_seed_texture : incoming_path_seed_texture
            }
            block_color = color.hsv(inchworm.id*90, 1, 1)
        block_texture = transition[already_placed_block.texture]
        delete_cube(cell)
        spawn_cube(cell, block_texture, block_color)

def generate_final_structure():
    """
    Clear the map, visualizing where final structure must be. 
    """
    # the shallow copy lets loop go through every placed block without changing what blocks are in the final struct 
    blocks_placed = list(sim_data.blocks_placed) # makes a shallow copy of the list 
    for block in blocks_placed: 
        delete_cube(block) # func deletes blocks from sim_data
        spawn_cube(block, final_struct_outline)
    sim_data.generate_final_structure_map(blocks_placed)

# def show_structures():
#     """
#     Searches for known structures and changes the color of structures found. 
#     """
#     # TODO: If planning on using substructures for the blueprint algo, make use of this system: 

#     for inchworm in sim_data.existing_inchworms:
#         inchworm.found_structures, inchworm.misc_blocks = search(sim_data.blocks_placed)
#         for structure in inchworm.found_structures:
#             structure_pos = structure[1]  
#             structure_name = structure[0] #string
#             for block in structure_pos:
#                 delete_cube(block[0], block[1], block[2])
#                 spawn_cube(block[0], block[1], block[2], structure_name[-1])
#                 #WHEN WE ARE IMPLEMENTING THE COLORS  spawn_cube(block[0], block[1], block[2], color_index)
#         for block in inchworm.misc_blocks:
#                 delete_cube(block[0], block[1], block[2])
#                 spawn_cube(block[0], block[1], block[2], 'misc')

# Voxel (block) properties
class Voxel(Button):
    def __init__(self, position = (0, 0, 0), texture = white_block_texture, bk_color=None):
        if bk_color is None: 
            bk_color = color.color(0, 0, random.uniform(0.9, 1))
        super().__init__(
            parent = scene,
            position = position,
            model = "Assets/Models/Block",
            origin_y = 0.5,
            texture = texture,
            color = bk_color,
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
                    yoxel = int(voxel.position.z)
                    zoxel = int(voxel.position.y)
                    sim_data.blocks_placed.append([xoxel, yoxel, zoxel])
                    print("pos: ", (xoxel, yoxel, zoxel))
            if key == "right mouse down":
                try: 
                    block = [self.position.x, self.position.z, self.position.y]
                    sim_data.blocks_placed.remove(block)
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

 
def check_block_color(x, y, z):
    """ Checks the color of the block at the specified position. This is used to simulate the stepping on an already placed block. """
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
        transition = {
            # Step on top of block, retaining the same color: 
            smart_block_texture: smart_block_texture_step,
            white_block_texture: outline_step_texture_red,
            supply_depot_texture: outline_step_texture_red, 
            outline_texture_green: outline_step_texture_green,
            outline_texture_blue: outline_step_texture_blue, 
            outline_texture_yellow: outline_step_texture_yellow,
            smart_block_texture_step: smart_block_texture, #*******
            # Transition from color step toff color step --> for the second foot:
            outline_step_texture_red: outline_step_texture_red, 
            outline_step_texture_green: outline_step_texture_green,
            outline_step_texture_blue: outline_step_texture_blue, 
            outline_step_texture_yellow: outline_step_texture_yellow, 
            # Textures indicating block/steps --> it's there: 
            seed_block_texture_step : seed_block_texture,
            final_struct_outline: smart_block_texture, 
            incoming_block_texture: smart_block_texture, 
            incoming_path_texture: outline_step_texture_red, # step over the incoming path 
            seed_block_texture: seed_block_texture_step,
            incoming_path_block_texture: smart_block_texture_step
        }
        try: 
            block_color = transition[existing_cube_texture]
        except:
            print(f"Unexpected texture: {existing_cube_texture}")  # Debugging line

    return block_color


def stop_simulation():
    print("User pressed 'ESC'. Stopping simulation...")
    application.quit()

# spawns a cude in the simulation at the specified position and with the specified color
def spawn_cube(coord: list[int], texture=smart_block_texture, bk_color=None):
    """
    Spawns a cube in the simulation at the specified xyz position and with the specified color. 
    Not always a smart block, but rather any sim update happening in a cube. 
    """
    # check if the position is already occupied
    target_position = Vec3(coord[0], coord[2], coord[1])

    if texture == smart_block_texture:
        sim_data.blocks_placed.append(coord)  # Update the block information

    # Spawn the cube
    new_cube = Voxel(position=target_position, texture=texture, bk_color=bk_color)

def delete_cube(coord: list[int]):
    """
    Delete a block from the simulation at the specified xyz position
    """
    target_position = Vec3(coord[0], coord[2], coord[1])
    for e in scene.entities:
        if hasattr(e, 'position') and e.position == target_position:
            destroy(e)
            if coord in sim_data.blocks_placed: #and (target_position not in sim_data.seed_block):
                sim_data.blocks_placed.remove(coord)
            break

# Generate the simulation floor. Increase the numbers for a bigger field. 
if not SIMULATION:
    for z in range(5): 
        for x in range(6): 
            voxel = Voxel(position = (x, 0, z))
else:
    for z in range(21): # 5
        for x in range(21): # 6 
            voxel = Voxel(position = (x, 0, z))
    # spawn seed block & supply depot
    spawn_cube(SEED_BK, seed_block_texture)
    spawn_cube(BD_LOCS[0], supply_depot_texture)

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
                pitch_degrees, yaw_degrees = look_at((10, 1, 10), (20, 10, 0))
                player.rotation_y = yaw_degrees
                player.camera_pivot.rotation_x = pitch_degrees
            if held_keys['2']:  
                self.position = Vec3(20, 10, 20) 
                pitch_degrees, yaw_degrees = look_at((10, 1, 10), (20, 10, 20))
                player.rotation_y = yaw_degrees
                player.camera_pivot.rotation_x = pitch_degrees
            if held_keys['3']:  
                self.position = Vec3(0,10, 20)  
                pitch_degrees, yaw_degrees = look_at((10, 1, 10), (0,10, 20))
                player.rotation_y = yaw_degrees
                player.camera_pivot.rotation_x = pitch_degrees
            if held_keys['4']:  
                self.position = Vec3(0, 10, 0)  
                pitch_degrees, yaw_degrees = look_at((10, 1, 10), (0, 10, 0))
                player.camera_pivot.rotation_x = pitch_degrees


player = FlyingFirstPersonController()
sky = Sky()

app.run()