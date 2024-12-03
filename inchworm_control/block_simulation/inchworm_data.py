from enum import Enum
import copy
from config import *
from map_data import *

class Inchworm:
    def __init__(self, id: int, orientation, paths: list[Cell], map: list[int], location: list[int], holding_block=False):
        """
        Initialize one inchworm (abbreviated as IW) in the system.
        Args: 
            id (int): This inchworm's ID number. Used to set paths in the map. 
            orientation (Enum): the direction that the IW's leading leg is facing, relative to the world grid's frame. 
            paths(list[Cell]): The Cells through which this inchworm will travel. (May be multiple, ie to the supply depot then to the structure.)
            map (list): xzy (3D) list storing the current status of the map, as this inchworm knows it. 
        """
        self.id = id
        self.orientation = orientation
        self.paths = paths
        self.map = map
        self.location = location
        self.holding_block = holding_block

    def get_location(self): 
        """
        Returns the x,z,y location of the inchworm as well as its orientation
        """
        pass

    def clear_my_path(self): 
        pass

    def plan_path(self, end): 
        pass

    # TODO: insert state machine here 

# def __main__():
#     # TODO: KASIA IS TESTING, DELETE THIS LATER 

#     inchwormA = Inchworm(1, InchwormOrientation.NORTH, )
#     print(" got to the end of main mon ami! ")
#     pass