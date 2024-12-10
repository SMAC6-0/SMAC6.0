import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from enum import Enum
from transitions import Machine



# Defining the Inchworm states 
Inchworm_States = ["INITIALIZATION", "PATH_PLANNING", "TRAVELLING_TO_SUPPLY", "TRANSPORTING_BLOCK", "PLACING_BLOCK", "ERROR", "STRUCTURE_COMPLETE"]
current_map = [
    [  # Layer 0
        [1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]
    ],
    [  # Layer 1
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]
    ],
    [  # Layer 2
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0]
    ]
]

# class for Finite State Machine
class Inchworm_StateMachine:
    def __init__(self):
        # initial state
        self.machine = Machine(model=self, states=Inchworm_States, initial= "INITIALIZATION")

        # add transitions
        self.machine.add_transition(source="INITIALIZATION", dest="PATH_PLANNING", condition="IW_gets_Map_Snapshot")
        # self.machine.add_transition(source="PATH_PLANNING", dest= "TRAVELLING_TO_SUPPLY")

        # Callbacks
        
        self.machine.on_enter_Initialization(self.on_Initialization)

        self.machine.on_enter_Path_Planning(self.on_Path_Planning)

    # during the initiliaztion phase the inchworm should lift up it's gripper and touch the seed block
    # and transfer the block location to the seed block
    def on_Initialization(self):

        
        pass



    # TODO: ask Mo for help when the IW gets the map SnapShot back 
    def IW_gets_Map_Snapshot(self):
        # blah blah low level language 
        # return true if the IW got the map snapshot
        return True

# an instance of Inchworm Statemachine
inchworm = Inchworm_StateMachine()

# Simulate the state machine
def run_inchworm_stateMachine ():
    print("Current inchworm state: {inchworm.state}")


if __name__ == "__main__":
    run_inchworm_stateMachine()


# ------------ implement state machine chatgpt code

from transitions import Machine

# Define the states
states = ["Locked", "Unlocked", "Open"]

class Door:
    def __init__(self):
        self.machine = Machine(model=self, states=states, initial="Locked")

        # Define transitions
        self.machine.add_transition(trigger="unlock", source="Locked", dest="Unlocked", conditions="is_code_correct")
        self.machine.add_transition(trigger="open", source="Unlocked", dest="Open")
        self.machine.add_transition(trigger="close", source="Open", dest="Unlocked")
        self.machine.add_transition(trigger="lock", source="Unlocked", dest="Locked")

        # Callbacks
        self.machine.on_enter_Locked(self.on_locked)
        self.machine.on_enter_Unlocked(self.on_unlocked)
        self.machine.on_enter_Open(self.on_open)

    def is_code_correct(self):
        # Example condition (could be replaced with user input)
        code = input("Enter the code: ")
        return code == "1234"

    def on_locked(self):
        print("The door is now locked.")

    def on_unlocked(self):
        print("The door is now unlocked.")

    def on_open(self):
        print("The door is open. Welcome!")

# Create the Door instance
door = Door()

# Simulate the state machine
while True:
    print(f"Current state: {door.state}")
    action = input("Choose an action (unlock, open, close, lock, quit): ").strip().lower()
    if action == "quit":
        break
    try:
        getattr(door, action)()
    except AttributeError:
        print("Invalid action!")



