from enum import Enum
import copy
from config import *
from map_data import *

class Inchworm:
    def __init__(self, id, orientation, paths, map, holding_block=False):
        self.id = id
        self.orientation = orientation
        self.paths = paths
        self.holding_block = holding_block
        self.map = map

    def get_location(self): 
        """
        Returns the x,z,y location of the inchworm as well as its orientation
        """
        pass

