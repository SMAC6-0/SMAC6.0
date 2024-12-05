from enum import Enum
import copy
from config import *
import map_data 
from map_data import Cell

class Inchworm:
    def __init__(self, id: int, orientation, paths: list[Cell], final_structure, location: list[int], holding_block=False):
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
        self.id = id
        self.orientation = orientation
        self.paths = paths
        self.current_map = map_data.initialize_grid_with_structures()
        self.final_structure = final_structure
        self.lead_foot_loc = location
        self.holding_block = holding_block

    def update_current_map(self, map): 
        """
        Updates the inchworm's map based on received updates from the structure. 
        Args: 
            map: xzy (3D) list storing the current status of the map, as the structure knows it.  
        """
        self.current_map = map

    def clear_my_path(self): 
        pass

    def plan_path(self, end): 
        pass

    # TODO: insert state machine here 
