from enum import Enum
from transitions import Machine
import time
from time import sleep

# Inchworm states
class IW_STATE(Enum):
    IDLE = 0 # added this incase we need to use it
    INITIALIZATION = 1
    PATH_PLANNING = 2
    TRAVELLING_TO_SUPPLY = 3
    TRANSPORTING_BLOCK = 4
    PLACING_BLOCK = 5
    ERROR = 6
    STRUCTURE_COMPLETE = 7

PATH_PLANNING_TIMER = 3
class Inchworm:
    def __init__(self):
        self.state = IW_STATE.INITIALIZATION

        while True: 
            self.update_state()

        pass

    def update_state(self):
        match self.state:
            case IW_STATE.IDLE:
                self.handle_idle()
            case IW_STATE.INITIALIZATION:
                self.handle_initilization()
                if (self.IW_gets_Map_Snapshot()): # IW got the mapsnap shot 
                    self.handle_IW_gets_Map()
            case IW_STATE.PATH_PLANNING:
                if (self.is_Path_Available()): # Path exists!
                    self.path_exists()
                else: # Path doesn't exist!
                    print("Retrying path planning after waiting")
                    # Question: Is it ok for the IW to sleep?!! cuz then it doesn't get active data yk 
                    sleep(PATH_PLANNING_TIMER) # TODO: Decide if we need a  sleep here because we want to have a non blocking code 
            case IW_STATE.TRAVELLING_TO_SUPPLY:
                if (self.check_supply_location()):
                    self.handle_travelling_to_supply()
            # case IW_STATE.TRANSPORTING_BLOCK:
            #     self.handle_transporting_block()
            # case IW_STATE.PLACING_BLOCK:
            #     self.handle_placing_block()
            # case IW_STATE.ERROR:
            #     self.handle_error()
            # case IW_STATE.STRUCTURE_COMPLETE:
            #     self.handle_structure_complete()

    
    ##### Checkers and Handlers

    # Handlers 

    # added this func incase we need it in the future
    def handle_idle():
        print("IDLINGGG....")
    
    # during the initiliaztion phase the inchworm should lift up it's gripper and touch the seed block
    # and transfer the block location to the seed block
    def handle_initilization(self):
        print("INITIALIZATION...")
        # path plan to the seed block location from the supply depot

        print("Initializing the block")
        # pick up the block infornt of it
        
        print("Transferring the block data")
        # transfer the block location data 
        # TODO: MOOO help 
        # send a 1D array ended with the Initialization enum OxFA 
        # flash block that it's in unplaced location
    
    def handle_IW_gets_Map(self):
        print("Map snapshot successful. Transitioning to PATH_PLANNING...")
        self.state = IW_STATE.PATH_PLANNING
        print(f"Current inchworm state: {self.state}")

    def path_exists(self):
        # MOOOO HELPPP 
        print("Sending the IW path to the structure")
        # IW sends it's path to the structure 

        print("Travelling to the supply")
        # IW begins travelling to supply location

        self.state = IW_STATE.TRAVELLING_TO_SUPPLY
        print(f"Current inchworm state: {self.state}")
    
    def swdvsdvf(self):
        print("Path exists. Transitioning to PATH_PLANNING...")
        self.state = IW_STATE.TRAVELLING_TO_SUPPLY
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

        print("Checking path availability...")
        is_Path_Available = input("Is Path Available? (yes/no) \n")
        if is_Path_Available.lower() == 'yes':
            return True
        elif is_Path_Available.lower() == 'no':
            return False
        else:
            print("Invalid input. Please answer with 'yes' or 'no'.")

    def check_supply_location(self):
        # return true if the IW is in the supply location (check the flag and compare the current IW  location through dead reckoning and the supply location)
        print("Checking if at supply location...")
        
        IW_in_supply = input("Is iW in supply? (yes/no) \n")
        if IW_in_supply.lower() == 'yes':
            return True
        elif IW_in_supply.lower() == 'no':
            return False
        else:
            print("Invalid input. Please answer with 'yes' or 'no'.")

# an instance of Inchworm Statemachine
inchworm_sm = Inchworm()

# Simulate the state machine
def run_Inchworm ():
    print("Current inchworm state:")

if __name__ == "__main__":
    run_Inchworm()

'''
# Define states
Inchworm_States = ["INITIALIZATION", "PATH_PLANNING", "TRAVELLING_TO_SUPPLY", "TRANSPORTING_BLOCK", "PLACING_BLOCK", "ERROR", "STRUCTURE_COMPLETE"]

class Inchworm:
    def __init__(self):
        # Initialize the state machine
        self.machine = Machine(model=self, states=Inchworm_States, initial="INITIALIZATION")
        print(f"Initial State: {self.state}")  

        # Set up state hooks
        self.machine.on_enter_INITIALIZATION(self.check_map_snapshot)
        self.machine.on_enter_PATH_PLANNING(self.check_path_availability)
        self.machine.on_enter_TRAVELLING_TO_SUPPLY(self.check_supply_location)
        self.machine.on_enter_TRANSPORTING_BLOCK(self.check_block_location)
        self.machine.on_enter_PLACING_BLOCK(self.check_structure_completion)

    # Callback for state hooks
    

    

    

    def check_block_location(self):
        print("Checking if at block location...")
        if self.is_IW_in_block_location():
            print("At block location. Transitioning to PLACING_BLOCK...")
            self.to_PLACING_BLOCK()  # Move to the next state automatically

    def check_structure_completion(self):
        print("Checking if structure is complete...")
        if self.is_structure_complete():
            print("Structure complete. Transitioning to STRUCTURE_COMPLETE...")
            self.to_STRUCTURE_COMPLETE()  # Move to the next state automatically

    # State action callbacks
    def on_enter_STRUCTURE_COMPLETE(self):
        print("Structure completed!")

    def on_enter_ERROR(self):
        print("An error occurred.")

    # Condition methods


    

    

    def is_IW_in_block_location(self):
        IW_in_block_location = input("Is IW in block location? \n")
        return IW_in_block_location

    def is_structure_complete(self):
        structure_complete = input("Is Structure Complete? \n")
        return structure_complete

# Instantiate and run
inchworm_sm = Inchworm()

# Explicitly call to initialize the first state transition
inchworm_sm.to_INITIALIZATION()  # Explicitly enter the INITIALIZATION state

# Simulate the state machine running
while inchworm_sm.state != "STRUCTURE_COMPLETE":
    print(f"Current State: {inchworm_sm.state}")
    time.sleep(1)  # Simulate time passing

'''