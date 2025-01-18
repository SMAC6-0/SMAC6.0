import unittest
from transitions import Machine
from inchworm_control.state_machine import *

class TestInchwormStateMachine (unittest.TestCase):
    def setUp(self):
        self.inchworm = Inchworm_StateMachine()

    def test_intial_state(self):
        self.assertEqual(self.inchworm.state, 'INITIALIZATION')

if __name__ == "__main__":
    unittest.main()