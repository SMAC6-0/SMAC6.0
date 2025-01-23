from enum import Enum
import copy
from config import *
import path_planning 
import path_conversion
# from map_data import Cell

class Inchworm:
    def __init__(self, id: int, orientation, paths, final_structure, location: list[int], holding_block=False):
        """
        Initialize one inchworm (abbreviated as IW) in the system.
        Args:
            id (int): This inchworm's ID number. Used to set paths in the map. 
            orientation (Enum): the direction that the IW's leading leg is facing, relative to the world grid's frame. 
            paths(list[Cell]): The Cells through which this inchworm will travel. (May be multiple, ie to the supply depot then to the structure.)
            final_structure (list[int]): xzy (3D) list storing the final structure the inchworms are trying to build.  
            location (list[int]): the xzy location of the inchworm's leading foot. 
            holding_block (bool): True if the inchworm's leading foot is holding a block. 
        """
        # Essential information for IW to keep track of
        self.id = id
        self.orientation = orientation
        self.paths = paths
        self.current_map = path_planning.initialize_grid_with_structures()
        self.final_structure = final_structure
        self.lead_foot_loc = location
        self.holding_block = holding_block

        # Path planning relevant vars
        self.coords_to_spawn = [] # the complete path
        self.goal = []
        self.goal_progress_index = 0

        # Leg locations for the inchworm. Point is the position of the leading leg and prev_point is the position of the second leg
        self.point = CURRENT_LOC
        self.prev_point = self.point

    def update_current_map(self, map): 
        """
        Updates the inchworm's map based on received updates from the structure. 
        Args: 
            map: xzy (3D) list storing the current status of the map, as the structure knows it.  
        """
        self.current_map = map

    def clear_my_path(self): 
        print("i cleared my path")
        pass

    
    def plan_path(self, misc_blocks, found_structures): 
        # TODO: transfer this function to the inchworm class 

        sorted_list = sorted(misc_blocks, key=lambda coordinate: coordinate[1])
        self.coords_to_spawn, path_steps , self.goal= path_conversion.dev_total_path_steps(found_structures, sorted_list)
        step_getter(path_steps)
        for point in self.goal:
            point[1] += 1  # Increment the second value

    def get_next_point(self): 
        """ 
        Returns the set of the next points of inchworm travel
        """
        (self.point, holding_block) = self.coords_to_spawn.pop(0)  # Get the next point
        x, z, y = self.point
        if holding_block:
            z = z+1
        return x, z, y

def step_getter(steps):
    """
    Write the steps to steps.txt
    """
    complete_steps = copy.deepcopy(steps)
    file_path = "steps.txt"
    
    with open(file_path, 'w') as file:
        for step in complete_steps:
            file.write(f"{step}\n")