from enum import Enum
import copy
from config import *
import map_data
from inchworm_control.blueprint import blueprint as blueprint
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

lagging_transform = {
    InchwormOrientation.NORTH: lambda x, z, y: (x, z, y-1),  
    InchwormOrientation.SOUTH: lambda x, z, y: (x, z, y+1),
    InchwormOrientation.EAST: lambda x, z, y: (x-1, z, y), 
    InchwormOrientation.WEST: lambda x, z, y: (x+1, z, y) 
}

class Inchworm:
    next_id = 1
    inchworm_list = []
    
    def __init__(self, orientation, final_structure, location: tuple[int]=CURRENT_LOC, holding_block=False):
        """
        Initialize one inchworm (abbreviated as IW) in the system.
        Args:
            id (int): This inchworm's ID number. Used to set paths in the map. 
            orientation (Enum): the direction that the IW's leading leg is facing, relative to the world grid's frame. 
            location (tuple[int]): the xzy location of the inchworm's leading foot. 
            holding_block (bool): True if the inchworm's leading foot is holding a block. 
        """
        # Essential information for IW to keep track of
        self.id = Inchworm.next_id
        self.orientation = orientation
        
        self.current_map = map_data.initialize_grid()
        self.final_structure = final_structure
        self.holding_block = holding_block

        # Path planning relevant vars
        self.paths = [] # the list of coords
        self.goal = [] # goal coord
        self.goal_progress_index = 0
        # self.found_structures = []
        # self.misc_blocks = []

        # Leg locations for the inchworm. Point is the position of the leading leg and prev_point is the position of the second leg
        self.leading_foot_loc = location
        self.lagging_foot_loc = list(lagging_transform[orientation](*self.leading_foot_loc))

        # pertaining to the state machine 
        self.state = IW_STATE.INITIALIZATION
        self.initilization_flag = True
        self.print_flag = True
        
        Inchworm.next_id += 1
        Inchworm.inchworm_list.append(self)
    
    def __del__(self):
        """
        Deletion of inchworm in the list of inchworms.
        """
        Inchworm.inchworm_list = [iw for iw in Inchworm.inchworm_list if iw.id != self.id]

    def update_my_current_map(self, map): 
        """
        Updates the inchworm's map based on received updates from the structure. 
        Args: 
            map: xzy (3D) list storing the current status of the map, as the structure knows it.  
        """
        # TODO: does this belong in checker, handler, or outside? @Mo 
        self.current_map = map

    def send_my_next_steps(self): 
        """ Send IW path and the corresponding incoming block to the structure. """ 
        if not SIMULATION: 
            # TODO @ SAKSHI & MO: UART COMMUNICATION
            pass

    def clear_my_path(self): 
        print("i cleared my path")
        pass

    def get_loc_in_path(self): 
        return tuple(map(float, self.goal))
    
    def plan_path_to_structure(self): 
        """ Plan path from current location to block depot, then from there to the next block. """
        self.goal = self.get_next_block() # returns the next_goal (block to be placed) based on blueprint algo
        self.current_map = map_data.update_grid_with_incoming(self.current_map, self.goal) # updates map for next_goal to be incoming_block
        
        try: 
            step_instructions = []
            # Path plan first tto block depot, then to the next goal
            bd_path, bd_steps = map_data.initiate_find_path(self.current_map, self.leading_foot_loc, BD_LOC1, self.orientation, self.holding_block)
            goal_path, goal_steps = map_data.initiate_find_path(self.current_map, BD_LOC1, self.goal, self.orientation, holding_block=True)
            goal_path[0].pop(0) # Remove repeat coord
   
            # Update inchworm path & corresponding steps to travel that path
            step_instructions += bd_steps
            step_instructions += goal_steps
            self.paths += bd_path[0]
            self.paths += goal_path[0]

            # Update the inchworm's internal map with the step it will take 
            self.current_map = map_data.set_inchworm_path_to_grid(self.current_map, self.paths) # Update IW's map with the path
            
            step_getter(step_instructions)
            # for point in self.goal:
            #     point[1] += 1  # Increment the second value
        except: 
            RuntimeError("No path found, try again later.")
    
    def plan_path_to_(self, next_goal: tuple[int]): 
        """ Plan path from current location to specified goal. """
        self.goal = next_goal
        self.current_map = map_data.update_grid_with_incoming(self.current_map, next_goal) # updates map for next_goal to be incoming_block
        
        try: 
            step_instructions = []
            # Path plan first tto block depot, then to the next goal
            bd_path, bd_steps = map_data.initiate_find_path(self.current_map, self.leading_foot_loc, next_goal, self.orientation, self.holding_block)
            
            # Update inchworm path & corresponding steps to travel that path
            step_instructions += bd_steps
            self.paths += bd_path[0]

            # Update the inchworm's internal map with the step it will take 
            self.current_map = map_data.set_inchworm_path_to_grid(self.current_map, self.paths) # Update IW's map with the path
            
            step_getter(step_instructions)
        except: 
            RuntimeError("No path found, try again later.")

    def get_next_point(self): 
        """ 
        Returns the set of the next points of inchworm travel. Used for stepping through path for sim.
        """
        (self.leading_foot_loc, holding_block) = self.paths[self.goal_progress_index]  # Get the next point
        x, z, y = self.leading_foot_loc
        self.goal_progress_index += 1
        if holding_block and [x, z, y] != self.goal:
            z = z + 1
        return x, z, y
    
    def get_next_block(self) -> tuple[int]:
        """ Uses the blueprint algorithm to determine which block should be placed next. """
        # TODO: handle misc
        # sorted_list = sorted(self.misc_blocks, key=lambda coordinate: coordinate[1])
        # self.misc_blocks = sorted_list
        new_next_block = blueprint(self.current_map, self.final_structure)
        return new_next_block
    
    def get_total_inchworms(cls):
        """
        Returns the total number of inchworms in the system
        """
        return len(cls.inchworm_list)
        
    def reset_inchworms(cls):
        cls.next_id = 1
        cls.inchworm_list.clear()
    
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
                    self.handle_at_supply()
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
        print("MOVINGGG TO SEED BLOCK")
        # path plan to the seed block location from the supply depot
        # if path exists 
        # alternatively do try except
        if self.paths: 
            # move IW in sim
            if self.goal_progress_index >= len(self.paths): 
                self.initilization_flag = False 
                self.paths = [] # Reset current path 
                self.goal_progress_index = 0 # TODO: May be good to move to handle_IW_gets_Map or clear_my_path
                print("Touching the seed block")
        else: 
            x, z, y = self.get_next_block() # TODO: for now assuming that first block is seed
            self.plan_path_to_([x, z, y])
        # print("Initializing the block")
        # touch the block infornt of it

        
        
    def handle_IW_gets_Map(self):
        print("Map snapshot successful.")

        print("Transferring the block data")
        # transfer the block location data 
        # TODO: MOOO help 
        # send a 1D array ended with the Initialization enum OxFA 
        # flash block that it's in unplaced location

        self.plan_path_to_structure()
        self.state = IW_STATE.PATH_PLANNING
        print(f"Current inchworm state: {self.state}")

    def path_exists(self):
        # MOOOO HELPPP 
        print("Path found. Sending the IW path to the structure")
        print("if in sim, press m for communication ")
        # IW sends it's path to the structure 
        self.send_my_next_steps()
        # TODO !!!!! 
        print("Travelling to the supply")
        self.state = IW_STATE.TRAVELLING_TO_SUPPLY
        print(f"Current inchworm state: {self.state}")
    
    def retry_path(self):
        # Question: Is it ok for the IW to sleep?!! cuz then it doesn't get active data yk 
        sleep(PATH_PLANNING_TIMER) # TODO: Decide if we need a  sleep here because we want to have a non blocking code
        # or stay here until the IW gets a new map!!
        # MOOO HELPPP
        self.plan_path_to_structure()

    def handle_at_supply(self):
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
        if SIMULATION: 
            if self.leading_foot_loc == self.goal:
                # TODO: you forgot about sim_data's current_map 
                return True
            return False 
        else: 
            # TODO @ Mo & Sakshi
            return True
        # got_map_snapshot = input("Did the inchworm get the map? (yes/no): \n")
        # if got_map_snapshot.lower() == 'yes':
        #     return True
        # elif got_map_snapshot.lower() == 'no':
        #     return False
        # else:
        #     print("Invalid input. Please answer with 'yes' or 'no'.")
    
    def is_Path_Available(self):
        # # question how do we know if this path is the most upto date path
        # return not IW_Path == [] # return if IW_path is empty or not (True: if not empty)
        # print("Planning path from supply to the next block")
        # IW path plans to the supply and to the next block
        # store that path in IW_path 
        # self.plan_path_to_structure()

        print("Checking path availability... ")
        if self.paths and self.goal:
            return True
        else: 
            return False

        # is_Path_Available = input("Is Path Available? (yes/no) \n")
        # if is_Path_Available.lower() == 'yes':
        #     return True
        # elif is_Path_Available.lower() == 'no':
        #     return False
        # else:
        #     print("Invalid input. Please answer with 'yes' or 'no'.")

    def is_IW_in_supply(self):
        """ return true if the IW is in the supply location (check the flag and compare the current IW  location through dead reckoning and the supply location)"""
        print("Checking if at supply location...")
        print("If in sim, press n to step")
        if any(bd_loc == self.leading_foot_loc for bd_loc in BD_LOCS): 
            return True 
        else: 
            return False

        # IW_in_supply = input("Is iW in supply? (yes/no) \n")
        # if IW_in_supply.lower() == 'yes':
        #     return True
        # elif IW_in_supply.lower() == 'no':
        #     return False
        # else:
        #     print("Invalid input. Please answer with 'yes' or 'no'.")

    def is_IW_in_block(self):
        """return true if the IW is in the block location (check the flag and compare the current IW  location through dead reckoning and the block location)"""
        print("Checking if at block location...")

        if self.leading_foot_loc == self.goal: 
            return True 
        else: 
            return False
        # IW_in_block = input("Is iW in block location? (yes/no) \n")
        # if IW_in_block.lower() == 'yes':
        #     return True
        # elif IW_in_block.lower() == 'no':
        #     return False
        # else:
        #     print("Invalid input. Please answer with 'yes' or 'no'.")

    def incorrect_block_location(self):
        """ return true if the IW gets "incorrectly placed block" from the structure """
        # MOOOO HELLPOPPPP
        if SIMULATION: 
            return True # TODO: actually check if in right spot
        else: 
            return False
        # incorrect_block = input("IW got error 'Incorrectly Placed Block'? (yes/no) \n")
        # if incorrect_block.lower() == 'yes':
        #     return True
        # elif incorrect_block.lower() == 'no':
        #     return False
        # else:
        #     print("Invalid input. Please answer with 'yes' or 'no'.")
        # pass
    
    def is_structure_complete(self):
        print("Checking if structure is complete")

        # compare the current map and the blueprint
        # return true if structure is complete and false otherwise
        if self.current_map == self.final_structure: 
            return True 
        else: 
            return False
        # structure_complete = input("Is structure complete? (yes/no) \n")
        # if structure_complete.lower() == 'yes':
        #     return True
        # elif structure_complete.lower() == 'no':
        #     return False
        # else:
        #     print("Invalid input. Please answer with 'yes' or 'no'.")
        # pass

def step_getter(step_instructions):
    """
    Write the steps to steps.txt
    """
    complete_steps = copy.deepcopy(step_instructions)
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