from enum import Enum
import copy
from config import *
from map_data import *

class Inchworm:
    def __init__(self, id, orientation, paths, holding_block):
        self.id = id
        self.orientation = orientation
        self.paths = paths
        self.holding_block = False

