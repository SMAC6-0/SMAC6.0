import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from enum import Enum

class Inchworm_State(Enum):
    INITIALIZATION = 1
