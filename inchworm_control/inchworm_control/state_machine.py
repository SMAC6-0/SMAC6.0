import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from enum import Enum
from transitions import Machine

# Defining the Inchworm states 
Inchworm_States = ["INITIALIZATION", "PATH_PLANNING", "TRAVELLING_TO_SUPPLY", "TRANSPORTING_BLOCK", "PLACING_BLOCK", "ERROR", "STRUCTURE_COMPLETE"]

# class for Finite State Machine
class Inchworm_StateMachine:
    def __init__(self):
        # initial state
        self.machine = Machine(model=self, states=Inchworm_States, initial= "INITIALIZATION")

        # add transitions
        self.machine.add_transition(trigger="map_snapshot", source="INITIALIZATION")



    




