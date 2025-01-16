import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from enum import Enum
from transitions import Machine
from inchworm_control.ik_test import IkTest
import time
from time import sleep


## UART stuff
UART_BAUD = 9600

# setup uart 
# uart1 = UART(0,)

SUPPLY_LOCATION = [1, 1, 1]
PATH_PLANNING_TIMER = 3 # timer for when IW can started path planning again (in seconds) 

# Defining the Inchworm states 
Inchworm_States = ["INITIALIZATION", "PATH_PLANNING", "TRAVELLING_TO_SUPPLY", "TRANSPORTING_BLOCK", "PLACING_BLOCK", "ERROR", "STRUCTURE_COMPLETE"]
BLUEPRINT = [
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

# update this map to the map the inchworm sends 
current_Map = [
    [  # Layer 0
        [0, 0, 0, 0, 0, 0],
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

IW_Path = []

# create an instance of IkTest to use the functions
# TODO: remane the IkTest to something else maybe Inchworm_Movement 
inchworm_movement = IkTest()

# class for Finite State Machine
class Inchworm_StateMachine:
    def __init__(self):
        # initial state
        self.machine = Machine(model=self, states=Inchworm_States, initial= "INITIALIZATION")

        # add transitions
        self.machine.add_transition(source="INITIALIZATION", dest="PATH_PLANNING", condition="IW_gets_Map_Snapshot", after="on_Path_Planning")
        self.machine.add_transition(source="PATH_PLANNING", dest="PATH_PLANNING", unless="is_Path_Available", after="retry_path")
        self.machine.add_transition(source="PATH_PLANNING", dest= "TRAVELLING_TO_SUPPLY", condition="is_Path_Available")
        self.machine.add_transition(source="TRAVELLING_TO_SUPPLY", dest="TRANSPORTING_BLOCK", condition="is_IW_in_supply")
        self.machine.add_transition(source="TRAVELLING_TO_SUPPLY", dest="ERROR", unless="is_IW_in_supply", after="error_action")
        self.machine.add_transition(source="TRANSPORTING_BLOCK", dest="ERROR", unless="is_IW_in_block_location", after="error_action")
        self.machine.add_transition(source="TRANSPORTING_BLOCK", dest="PLACING_BLOCK", condition="is_IW_in_block_location")
        self.machine.add_transition(source="PLACING_BLOCK", dest="ERROR", condition= "incorrect_block_location", after="error_action")

        # assume that if the IW get's map snapshot, the block is placed in the correct location
        self.machine.add_transition(source="PLACING_BLOCK", dest="STRUCTURE_COMPLETE", conditions=[self.IW_gets_Map_Snapshot, self.is_structure_complete], after="do_structure_complete")
        self.machine.add_transition(source="PLACING_BLOCK", dest="PATH_PLANNING", condition= "IW_gets_Map_Snapshot", unless= "is_structure_complete")

        # Callbacks
        self.machine.on_enter_Initialization(self.on_picking_new_block)
        self.machine.on_enter_Travelling_to_Supply(self.on_Travelling_to_Supply)
        self.machine.on_enter_Transporting_Blocks(self.on_picking_new_block)
    
    # Actions

    # during the initiliaztion phase the inchworm should lift up it's gripper and touch the seed block
    # and transfer the block location to the seed block
    def on_picking_new_block(self):
        # pick up the block infornt of it
        print("Initializing the block")

        # transfer the block location data 
        # TODO: MOOO help 
        # send a 1D array ended with the Initialization enum OxFA 


        # flash block that it's in unplaced location

        # IW starts to travel to the next block 


    def on_Path_Planning(self):
        # TODO: add the path planning stuff 
        print("Planning path from supply to the next block")

        # IW path plans to the supply and to the next block
        # store that path in IW_path 
        pass

    def retry_path(self):
        print("Retrying path planning after waiting")
        
        # Question: Is it ok for the IW to sleep?!! cuz then it doesn't get active data yk 
        sleep(PATH_PLANNING_TIMER)
        self.on_Path_Planning()
    
    def send_path_to_structure(self):
        # MOOOO HELPPP 
        print("Sending the IW path to the structure")
        pass
    
    def on_Travelling_to_Supply(self):
        self.send_path_to_structure()

        print("IW startes to travel to supply")
        pass

    def error_action(self):
        print("OH NOOO ERROR OCCURED")

        # stop the iW
        # flash red light

    def do_structure_complete(self):
        print("Structure is complete YIppeee")
        # stop the iW
        # flash green light



    # Conditionals 
    def IW_gets_Map_Snapshot(self):
        # blah blah low level language 
        # TODO: ask Mo for help when the IW gets the map SnapShot back 

        # return true if the IW got the map snapshot
        return True
    
    def is_Path_Available(self):
        # question how do we know if this path is the most upto date path
        return not IW_Path == [] # return if IW_path is empty or not (True: if not empty)
    
    def is_IW_in_supply(self):
        # return true if the IW is in the supply location (check the flag and compare the current IW  location through dead reckoning and the supply location)
        pass 

    def is_IW_in_block_location(self):
        # return true if the IW is in the block location (check the flag and compare the current IW  location through dead reckoning and the block location)
        pass 

    def incorrect_block_location(self):
        print("Block is placed in incorrect location?")
        # return true if the IW gets "incorrectly placed block" from the structure 
        pass

    def is_structure_complete(self):
        print("Structure is complete?")

        # compare the current map and the blueprint
        # return true if structure is complete and false otherwise
        pass
            

# an instance of Inchworm Statemachine
inchworm_sm = Inchworm_StateMachine()

# Simulate the state machine
def run_inchworm_stateMachine ():
    print("Current inchworm state: {inchworm_sm.state}")


if __name__ == "__main__":
    run_inchworm_stateMachine()
