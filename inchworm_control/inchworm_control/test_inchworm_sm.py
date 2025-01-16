import unittest
from transitions import Machine
from state_machine import Inchworm_StateMachine

class TestInchwormStateMachine (unittest.TestCase):
    def setUp(self):
        self.inchworm = Inchworm_StateMachine()

    