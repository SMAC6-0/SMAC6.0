from enum import Enum
import copy
from config import *
import path_planning 
import path_conversion
import map_data
from time import sleep

# Inchworm states
class IW_STATE(Enum):
    IDLE = 1 # added this incase we need to use it
    INITIALIZATION = 2
    PATH_PLANNING = 3
    TRAVELLING_TO_SUPPLY = 4
    TRANSPORTING_BLOCK = 5
    PLACING_BLOCK = 6
    ERROR = 7
    STRUCTURE_COMPLETE = 8

PATH_PLANNING_TIMER = 3

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
        self.current_map = map_data.initialize_grid_with_structures()
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

        # pertaining to the state machine 
        self.state = IW_STATE.INITIALIZATION
        self.initilization_flag = True
        self.print_flag = True

    def update_current_map(self, map): 
        """
        Updates the inchworm's map based on received updates from the structure. 
        Args: 
            map: xzy (3D) list storing the current status of the map, as the structure knows it.  
        """
        # TODO: does this belong in checker, handler, or outside? @Mo 
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
    
    ### STATE MACHINE 

    def run(self):
        while self.state != IW_STATE.STRUCTURE_COMPLETE:
            self.update_state()

    def update_state(self):
        match self.state:
            case IW_STATE.IDLE:
                self.handle_idle()
            case IW_STATE.INITIALIZATION:
                if self.initilization_flag:
                    self.handle_initilization()
                if self.IW_gets_Map_Snapshot(): # IW got the mapsnap shot 
                    self.handle_IW_gets_Map()
            case IW_STATE.PATH_PLANNING:
                if self.is_Path_Available(): # Path exists!
                    self.path_exists()
                else: # Path doesn't exist!
                    print("Retrying path planning after waiting")
                    self.retry_path() 
            case IW_STATE.TRAVELLING_TO_SUPPLY:
                if self.is_IW_in_supply():
                    self.handle_travelling_to_supply()
                else:
                    self.handle_error()
            case IW_STATE.TRANSPORTING_BLOCK:
                if self.is_IW_in_block():
                    self.handle_transporting_block()
                else:
                    self.handle_error()
            case IW_STATE.PLACING_BLOCK:
                if self.incorrect_block_location(): # block is placed in the wrong location
                    self.handle_error()
                elif self.IW_gets_Map_Snapshot(): # assume that the block is placed in the correct location
                    if self.is_structure_complete(): # structure is complete
                        self.handle_structure_complete()
                    else: # structure is incomplete
                        self.handle_structure_incomplete()
            case IW_STATE.ERROR:
                self.handle_error()
            case IW_STATE.STRUCTURE_COMPLETE:
                self.handle_structure_complete()

    
    ##### Checkers and Handlers

    # Handlers 

    # added this func incase we need it in the future
    def handle_idle(self):
        if self.print_flag:
            print("IDLINGGG....")
            self.print_flag = False

    
    # during the initiliaztion phase the inchworm should lift up it's gripper and touch the seed block
    # and transfer the block location to the seed block
    def handle_initilization(self):
        print("MOVINGGG...")
        # path plan to the seed block location from the supply depot
        # if path exists 
        # self.get_next_point() 
        # alternatively do try except

        print("Initializing the block")
        # touch the block infornt of it

        self.initilization_flag = False
        
    def handle_IW_gets_Map(self):
        print("Map snapshot successful.")

        print("Transferring the block data")
        # transfer the block location data 
        # TODO: MOOO help 
        # send a 1D array ended with the Initialization enum OxFA 
        # flash block that it's in unplaced location

        self.state = IW_STATE.PATH_PLANNING
        print(f"Current inchworm state: {self.state}")

    def path_exists(self):
        # MOOOO HELPPP 
        print("Sending the IW path to the structure")
        # IW sends it's path to the structure 

        print("Travelling to the supply")
        # IW begins travelling to supply location
        # self.get_next_point() # TODO

        self.state = IW_STATE.TRAVELLING_TO_SUPPLY
        print(f"Current inchworm state: {self.state}")
    
    def retry_path(self):
        # Question: Is it ok for the IW to sleep?!! cuz then it doesn't get active data yk 
        sleep(PATH_PLANNING_TIMER) # TODO: Decide if we need a  sleep here because we want to have a non blocking code
        # or stay here until the IW gets a new map!!
        # MOOO HELPPP

    def handle_travelling_to_supply(self):
        print("Touching the new block")
        # touch the new block

        print("IW flashes block with it's location")
        # MO HHELLPP MEEEE 
        
        self.state = IW_STATE.TRANSPORTING_BLOCK
        print(f"Current inchworm state: {self.state}")

    def handle_transporting_block(self):
        print("IW sends a messgae indicating block is being placed")
        # IW sends a messgae indicating block is being placed
        # MOOOOO HELPPPP

        print("Travelling to the block location")
        # IW begins travelling to block location

        self.state = IW_STATE.PLACING_BLOCK
        print(f"Current inchworm state: {self.state}")

    def handle_error(self):
        print("OHHH NOOO, ERROR ERROR")

        print("Stop Inchworm")
        # stop the inchworm
        
        print("flash red LED")
        # flash red Led 

        self.state = IW_STATE.IDLE
        print(f"Current inchworm state: {self.state}")
    
    def handle_structure_complete(self):
        print("Structure is complete YIppeee")

        # stop the iW
        print("IW Stopped")
        
        # flash green light
        print("flash Green LED")

        self.state = IW_STATE.STRUCTURE_COMPLETE
        print(f"Current inchworm state: {self.state}")

    def handle_structure_incomplete(self):
        print("Structure is incomplete")

        self.state = IW_STATE.PATH_PLANNING
        print(f"Current inchworm state: {self.state}")

    # Checkers
    def IW_gets_Map_Snapshot(self):
        # blah blah low level language 
        # TODO: ask Mo for help when the IW gets the map SnapShot back 
        # return true if the IW got the map snapshot

        got_map_snapshot = input("Did the inchworm get the map? (yes/no): \n")
        if got_map_snapshot.lower() == 'yes':
            return True
        elif got_map_snapshot.lower() == 'no':
            return False
        else:
            print("Invalid input. Please answer with 'yes' or 'no'.")
    
    def is_Path_Available(self):
        # # question how do we know if this path is the most upto date path
        # return not IW_Path == [] # return if IW_path is empty or not (True: if not empty)
        # TODO: add the path planning stuff 
        print("Planning path from supply to the next block")
        # IW path plans to the supply and to the next block
        # store that path in IW_path 
        # self.plan_path(self.misc_blocks, self.found_structures)
        # TODO: make the path planning compatible with the sim

        print("Checking path availability...")
        is_Path_Available = input("Is Path Available? (yes/no) \n")
        if is_Path_Available.lower() == 'yes':
            return True
        elif is_Path_Available.lower() == 'no':
            return False
        else:
            print("Invalid input. Please answer with 'yes' or 'no'.")

    def is_IW_in_supply(self):
        # return true if the IW is in the supply location (check the flag and compare the current IW  location through dead reckoning and the supply location)
        print("Checking if at supply location...")

        IW_in_supply = input("Is iW in supply? (yes/no) \n")
        if IW_in_supply.lower() == 'yes':
            return True
        elif IW_in_supply.lower() == 'no':
            return False
        else:
            print("Invalid input. Please answer with 'yes' or 'no'.")

    def is_IW_in_block(self):
        #  return true if the IW is in the block location (check the flag and compare the current IW  location through dead reckoning and the block location)
        print("Checking if at block location...")

        IW_in_block = input("Is iW in block location? (yes/no) \n")
        if IW_in_block.lower() == 'yes':
            return True
        elif IW_in_block.lower() == 'no':
            return False
        else:
            print("Invalid input. Please answer with 'yes' or 'no'.")

    def incorrect_block_location(self):
        # return true if the IW gets "incorrectly placed block" from the structure 
        # MOOOO HELLPOPPPP

        incorrect_block = input("IW got error 'Incorrectly Placed Block'? (yes/no) \n")
        if incorrect_block.lower() == 'yes':
            return True
        elif incorrect_block.lower() == 'no':
            return False
        else:
            print("Invalid input. Please answer with 'yes' or 'no'.")
        pass
    
    def is_structure_complete(self):
        print("Checking if structure is complete")

        # compare the current map and the blueprint
        # return true if structure is complete and false otherwise

        structure_complete = input("Is structure complete? (yes/no) \n")
        if structure_complete.lower() == 'yes':
            return True
        elif structure_complete.lower() == 'no':
            return False
        else:
            print("Invalid input. Please answer with 'yes' or 'no'.")
        pass

def step_getter(steps):
    """
    Write the steps to steps.txt
    """
    complete_steps = copy.deepcopy(steps)
    file_path = "steps.txt"
    
    with open(file_path, 'w') as file:
        for step in complete_steps:
            file.write(f"{step}\n")

if __name__ == "__main__":
    inchworm = Inchworm(1, CURRENT_ORIENTATION, None, None, CURRENT_LOC)
    try:
        inchworm.run()
    except KeyboardInterrupt:
        print("Stopping the inchworm system.") 