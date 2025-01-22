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
                if (self.IW_gets_Map_Snapshot()): # IW got the mapsnap shot 
                    self.handle_IW_gets_Map()
            # case IW_STATE.PATH_PLANNING:
            #     self.handle_path_planning()
            # case IW_STATE.TRAVELLING_TO_SUPPLY:
            #     self.handle_travelling_to_supply()
            # case IW_STATE.TRANSPORTING_BLOCK:
            #     self.handle_transporting_block()
            # case IW_STATE.PLACING_BLOCK:
            #     self.handle_placing_block()
            # case IW_STATE.ERROR:
            #     self.handle_error()
            # case IW_STATE.STRUCTURE_COMPLETE:
            #     self.handle_structure_complete()

    
    ##### Checkers and Handlers
    def handle_idle(self):
        print("IDILING...")

    # Handlers 
    def handle_IW_gets_Map(self):
        print("Map snapshot successful. Transitioning to PATH_PLANNING...")
        self.state = IW_STATE.PATH_PLANNING
        print(f"Current inchworm state: {self.state}")

    # Checkers
    def IW_gets_Map_Snapshot(self):
        got_map_snapshot = input("Did the inchworm get the map? (yes/no): \n")
        if got_map_snapshot.lower() == 'yes':
            return True
        elif got_map_snapshot.lower() == 'no':
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
    

    def check_path_availability(self):
        print("Checking path availability...")
        if self.is_Path_Available():
            print("Path is available. Transitioning to TRAVELLING_TO_SUPPLY...")
            self.to_TRAVELLING_TO_SUPPLY()  # Move to the next state automatically
        else:
            print("Path is not available. Retrying...")

    def check_supply_location(self):
        print("Checking if at supply location...")
        if self.is_IW_in_supply():
            print("At supply location. Transitioning to TRANSPORTING_BLOCK...")
            self.to_TRANSPORTING_BLOCK()  # Move to the next state automatically

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
    def IW_gets_Map_Snapshot(self):
        got_map_snapshot = input("Did the inchworm get the map? (yes/no): \n")
        if got_map_snapshot.lower() == 'yes':
            return True
        elif got_map_snapshot.lower() == 'no':
            return False
        else:
            print("Invalid input. Please answer with 'yes' or 'no'.")
            return self.IW_gets_Map_Snapshot()  # Recursively ask again


    def is_Path_Available(self):
        is_Path_Available = input("Is Path Available? \n")
        return is_Path_Available

    def is_IW_in_supply(self):
        IW_in_supply = input("Is iW in supply? \n")
        return IW_in_supply

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