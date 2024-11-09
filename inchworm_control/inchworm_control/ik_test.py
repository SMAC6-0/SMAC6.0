#!/usr/bin/env python3
from inchworm_control.ik import inverseKinematics
from inchworm_control.trajectory_planning import quintic_trajectory 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
# for servo
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
import time
from inchworm_control.lewansoul_servo_bus import ServoBus
from time import sleep 
import numpy as np
from enum import Enum
import copy

class EE_direction(Enum):
    DOWN = 90
    UP = 0

HOME_POSITION = [1, 0, 0, EE_direction.DOWN.value]
ABOVE_HOME = [1, 0, 0.5, EE_direction.DOWN.value]
BLOCK_INTERFACING_TIME = 1
TRAVEL_TIME = 2
    
class IkTest(Node):
    def __init__(self):
        """
        Initialization method for the motor controller node.

        Initializes ROS2 publisher, subscriber, GPIO pins, motor angles, and step actions for the inchworm robot.
        """
        # Initialize the ROS2 node with the name 'ik_test'
        super().__init__('ik_test')

        # Create a publisher for the 'step_status' topic, which sends Float32 messages
        self.publisher_ = self.create_publisher(Float32, 'step_status', 10)

        # Create a subscriber for the 'ik_command' topic, which listens for String messages
        # The messages are handled by the listener_callback method
        self.subscription = self.create_subscription(
            String,
            'ik_command',
            self.listener_callback,
            10)
        self.subscription # Prevents unnecessary warnings

        # Initialize the connection to the servo motor bus over a USB-TTL connection
        # Note: The RPi should be connected to the bottom-left USB port and no other USB devices should be connected
        # If the connection fails, try disconnecting and reconnecting the USB port
    
        self.servo_bus = ServoBus('/dev/ttyUSB0')  
        self.get_logger().info('Node starting')

        # init motors
        self.init_motors()

        # init servos
        GPIO.setmode(GPIO.BOARD)

        # Initialize GPIO pins 11 and 13 for controlling the gripper servos
        GPIO.setup(11, GPIO.OUT)  # Pin 11 as output for servo1
        GPIO.setup(13, GPIO.OUT)  # Pin 13 as output for servo2

        # Set up PWM (Pulse Width Modulation) for the two gripper servos, with a frequency of 50Hz
        self.servo1 = GPIO.PWM(11,50) # pin 11 for servo1, pulse 50Hz
        self.servo2 = GPIO.PWM(13,50) # pin 13 for servo2, pulse 50Hz

        # Start PWM with an initial duty cycle of 0 (no movement)
        self.servo1.start(0)
        self.servo2.start(0)

        # Note: Motors are not allowed to have negative positions
        
        print("----------------Initial Motor Angles-----------------------")
        print(self.motor_1.pos_read(), 
            self.motor_2.pos_read(), 
            self.motor_3.pos_read(), 
            self.motor_4.pos_read(), 
            self.motor_5.pos_read())
        
        # Initialize a dictionary mapping possible step actions to corresponding methods
        self.step_actions = {
            # Inchworm movements
            'STEP_FORWARD': self.step_forward,
            'STEP_LEFT': self.step_left,
            'STEP_RIGHT': self.step_right,

            # Inchworm movements with one block
            'STEP_FORWARD_BLOCK': self.step_forward_block
            # 'STEP_LEFT_BLOCK': self.step_left_block,
            # 'STEP_RIGHT_BLOCK': self.step_right_block,
            # 'GRAB_UP_FORWARD': self.grab_up_forward, 
            # 'GRAB_UP_LEFT': self.grab_up_left, 
            # 'PLACE_FORWARD_BLOCK': self.place_forward,
            # 'PLACE_UP_FORWARD_BLOCK': self.place_up_forward
            # 'PLACE_UP_2_FORWARD_BLOCK': self.place_up_2_forward,
            # 'SIMPLIFIED_POS_1_DOWN_1': self.step_down_1,
            # 'SIMPLIFIED_POS_1_DOWN_2': self.step_down_2
            # Add more mappings as needed
        }      



    def listener_callback(self, msg):
        """
        Callback function for the ik_command subscriber.

        Processes incoming commands, executes the corresponding step action, and publishes the step status.
        Throws errors for failure of command execution.
        """
        self.get_logger().info('Received command to "%s' % msg.data)
        try:
            # Get the step action from the step_actions dictionary based on the received message
            action = self.step_actions.get(msg.data)

            if action:
                # If a valid action (step) is found, execute the action with pivot_foot (1 for this case)
                action()
                # TODO: Determine when pivot_foot == 5 is passed into the step functions 
            else:
                # Log a warning if the action is not recognized
                self.get_logger().warn('Unknown command: %s' % msg.data)
            sleep(1)
            
        except Exception as e:
            self.get_logger().error('Failed to move servo: "%s"' % str(e))


    def init_motors(self):
        """
        Retrieves and initializes motors from the servo bus.
        """
        self.motor_1 = self.servo_bus.get_servo(1)
        self.motor_2 = self.servo_bus.get_servo(2)
        self.motor_3 = self.servo_bus.get_servo(3)
        self.motor_4 = self.servo_bus.get_servo(4)
        self.motor_5 = self.servo_bus.get_servo(5)

        self.time_to_move = 1.5 # Set the time over which the motors will move.

        
    def move_to(self, current_pos, final_pos, travelTime, pivot_foot): 
        """
        Move the robot end effector between one location and another using quintic trajectory. 

        Args: 
            current_pos (list): the current position of the EE as a 1x4 vector
            final_pos (list): the final location of the EE as a 1x4 vector
            travelTime (float): the time taken for the movement
            pivot_foot (int): Motor identifier (1 or 5) corresponding to the foot.
        """
        # This conditional makes it so that the EE does NOT rotate when the EE is moving straight up/down.  
        # This check is essential to make sure that the wires do not get tangled as the inchworm turns. 
        # It also makes sure that it doesn't turn when it is touching the board or a block, causing it to get stuck. 
        if (current_pos[0]==final_pos[0] & current_pos[1]==final_pos[1]): # if the start&end x&y positions are the same, then the movement must be vertical 
            fix_EE_orientation = False # do not rotate the EE (motors 1 or 5)
        else: 
            fix_EE_orientation = True # rotate the EE (motors 1 or 5)

        current_pos = np.transpose(np.asarray(current_pos))
        final_pos = np.transpose(np.asarray(final_pos))

        # trajectory planning to move from above object to on object. each is a 6x1 matrix 
        # the last 4 inputs are 0 to make movement more precise (vel & accel = 0) as the EE approaches the goal
        q0 = quintic_trajectory(0,travelTime, current_pos[0], final_pos[0], 0, 0, 0, 0) # matrix for x 
        q1 = quintic_trajectory(0,travelTime, current_pos[1], final_pos[1], 0, 0, 0, 0) # matrix for y
        q2 = quintic_trajectory(0,travelTime, current_pos[2], final_pos[2], 0, 0, 0, 0) # matrix for z 
        q3 = quintic_trajectory(0,travelTime, current_pos[3], final_pos[3], 0, 0, 0, 0) # matrix for alpha 

        q_t = np.concatenate((q0, q1, q2, q3), axis=1) # 6x4 mat

        # run trajectory for task space
        self.run_trajectory(q_t, travelTime, pivot_foot, fix_EE_orientation)
    
    def move_joints(self, joint_angles, time):
        """
        Move motors to specified angles over a given time duration.

        Args:
            joint_angles(list): theta1, theta2, theta3, theta4, theta5 in degrees
            time (float): Duration to reach the target angles (in seconds).
        """
        # TODO: Look into whether it's worth calling self.time_to_move here rather than passing in time as a parameter.
        [theta1, theta2, theta3, theta4, theta5] = joint_angles
        self.motor_2.move_time_write(theta2, time)
        self.motor_3.move_time_write(theta3, time)
        self.motor_4.move_time_write(theta4, time)
        self.motor_1.move_time_write(theta1, time)
        self.motor_5.move_time_write(theta5, time)

        # Pause the program to allow the motors to finish moving. 
        sleep(time)

        print("----------------After Motor Angles-----------------------")
        print(self.motor_1.pos_read(), 
            self.motor_2.pos_read(), 
            self.motor_3.pos_read(), 
            self.motor_4.pos_read(), 
            self.motor_5.pos_read())

    def run_trajectory(self, trajCoeffs, totTime, pivot_foot, fix_EE_orientation):
        """
        Calculates current joint positions based on trajectory coefficients and current time.
        
        Args:
            trajCoeffs (list): [6x4 float] trajectory coefficients generated from quintic_trajectory()
            totTime (double): total amount of time it takes for trajectory to reach target position
            pivot_foot (int): Motor identifier (1 or 5) corresponding to the foot.
            fix_EE_orientation (bool): True if the EE rotation is being reset to 0. 
        """
        time_s = 0
        
        tic = time.perf_counter()

        while(time_s < totTime):
            # Calculate coeffs accepts 6x4
            x = trajCoeffs[0][0] + trajCoeffs[1][0]*time_s + trajCoeffs[2][0]*pow(time_s,2) + trajCoeffs[3][0]*pow(time_s,3) + trajCoeffs[4][0]*pow(time_s,4) + trajCoeffs[5][0]*pow(time_s,5)
            y = trajCoeffs[0][1] + trajCoeffs[1][1]*time_s + trajCoeffs[2][1]*pow(time_s,2) + trajCoeffs[3][1]*pow(time_s,3) + trajCoeffs[4][1]*pow(time_s,4) + trajCoeffs[5][1]*pow(time_s,5)
            z = trajCoeffs[0][2] + trajCoeffs[1][2]*time_s + trajCoeffs[2][2]*pow(time_s,2) + trajCoeffs[3][2]*pow(time_s,3) + trajCoeffs[4][2]*pow(time_s,4) + trajCoeffs[5][2]*pow(time_s,5)
            alpha = trajCoeffs[0][3] + trajCoeffs[1][3]*time_s + trajCoeffs[2][3]*pow(time_s,2) + trajCoeffs[3][3]*pow(time_s,3) + trajCoeffs[4][3]*pow(time_s,4) + trajCoeffs[5][3]*pow(time_s,5)
                     
            # running the inverseKinematics to get the joint angles
            joint_ang = inverseKinematics(x, y, z, alpha, pivot_foot, fix_EE_orientation) # the joint angles
            
            self.move_joints(joint_ang, 0.5) # running the motors to get to the point

            sleep(1/10)
            toc = time.perf_counter()
            time_s = toc - tic
        
    """
    The territory of movesets begins now...
    """

    def step_forward(self): 
        # Start moving leading foot 
        pivot_foot = 1 # the pivot foot 
        self.latch_detach(pivot_foot) 
        print("Latched detached")
        
        # EE moves straight up from board to "safe" location above the 
        self.move_to(HOME_POSITION, ABOVE_HOME, BLOCK_INTERFACING_TIME, pivot_foot) 
        print("move from ", HOME_POSITION, " ", ABOVE_HOME)
        # Move from location above home forward 
        goal = [2, 0, 0, EE_direction.DOWN.value]
        print ("goal is: ", goal)
        above_goal = copy.deepcopy(goal)
        above_goal[2] = 0.5
        print("did goal survive? goal is: ", goal)

        # Move forward and hover over the goal overhead position 
        self.move_to(ABOVE_HOME, above_goal, TRAVEL_TIME, pivot_foot)
        print("move from ", ABOVE_HOME, " ", above_goal)
        
        # Move from above goal to the goal position
        self.move_to(above_goal, goal, BLOCK_INTERFACING_TIME, pivot_foot)
        print("move from ", above_goal, " ", goal)

        print("-------------- Front leg is in place")
        sleep(3)
        
        # At this point, leading foot (@ motor 5) is back on the ground, with 1 grid cell between it and the other foot 
        # Next, the following foot moves 
        pivot_foot = 5 # now the pivot foot is 5
        self.latch_detach(pivot_foot)
        print("latch and detach for ", pivot_foot)
        
        # EE moves straight up from board to "safe" location above the 
        self.move_to(goal, above_goal, BLOCK_INTERFACING_TIME, pivot_foot) 
        print("move from ", goal, " ", above_goal)
        
        # Move forward and hover over the goal overhead position 
        self.move_to(above_goal, ABOVE_HOME, TRAVEL_TIME, pivot_foot)
        print("move from ", above_goal, " ", ABOVE_HOME)
        
        # Move from above goal to the goal position
        self.move_to(ABOVE_HOME, HOME_POSITION, BLOCK_INTERFACING_TIME, pivot_foot)
        print("move from ", ABOVE_HOME, " ", HOME_POSITION)

        print("Movement complete: STEP_FORWARD")

    # TODO: clarify turn vs step in function name 
    # sakshi it's okay. It's a step towards the left direction
    def step_left(self): 
        # positions 
        leading_foot_goal = [0, 2, 0, EE_direction.DOWN.value]
        above_leading_foot_goal = copy.deepcopy(leading_foot_goal)
        above_leading_foot_goal[2] = 0.5

        

        # Start moving leading foot 
        # gripper activated RAHHHH
        pivot_foot = 1 # the pivot foot 
        self.latch_detach(pivot_foot) 
        print("Latched detached")
        
        # EE moves straight up from board to "safe" location above the home position
        # lift the front feet from the board
        self.move_to(HOME_POSITION, ABOVE_HOME, BLOCK_INTERFACING_TIME, pivot_foot) 
        # print("move from ", HOME_POSITION, " ", ABOVE_HOME)
        
        # Move from location above home to left (hover)
        

        self.move_to(ABOVE_HOME, above_goal, TRAVEL_TIME, pivot_foot)
        print("move from ", ABOVE_HOME, " ", above_goal)
        
        # Move from above goal to the goal position
        self.move_to(above_goal, goal, BLOCK_INTERFACING_TIME, pivot_foot)
        print("move from ", above_goal, " ", goal)

        print("-------------- Front leg is in place")
        sleep(3)
        
        # At this point, leading foot (@ motor 5) is back on the ground, with 1 grid cell between it and the other foot 
        # Next, the following foot moves 

        # gripper activated RAHHHH
        pivot_foot = 5 # now the pivot foot is 5
        self.latch_detach(pivot_foot)
        print("latch and detach for ", pivot_foot)

        # Now, since the origin and axes for the inverse kinematics have flipped to be w.r.t. the other foot, 
        # goal must be adjusted. 
        following_foot_goal = [2, 0, 0, EE_direction.DOWN.value]
        print ("goal is: ", following_foot_goal)
        above_following_foot_goal = copy.deepcopy(following_foot_goal)
        above_following_foot_goal[2] = 0.5
        print("did goal survive? goal is: ", following_foot_goal)

        # lift the back feet from the board       
        # EE moves straight up from board to "safe" location above the 
        self.move_to(following_foot_goal, above_following_foot_goal, BLOCK_INTERFACING_TIME, pivot_foot) 
        print("move from ", following_foot_goal, " ", above_following_foot_goal)
        
        # rotate the back feet
        self.move_to(above_following_foot_goal, ABOVE_HOME, TRAVEL_TIME, pivot_foot)
        print("move from ", above_following_foot_goal, " ", ABOVE_HOME)
        
        # put the back feet on the board
        # Move from above goal to the goal position
        self.move_to(ABOVE_HOME, HOME_POSITION, BLOCK_INTERFACING_TIME, pivot_foot)
        print("move from ", ABOVE_HOME, " ", HOME_POSITION)

        print("movement complete: STEP_LEFT")

    def step_right(self): 
        # Start moving leading foot 
        # gripper activated RAHHHH
        pivot_foot = 1 # the pivot foot 
        self.latch_detach(pivot_foot) 
        print("Latched detached")
        
        # EE moves straight up from board to "safe" location above the home position
        # lift the front feet from the board
        self.move_to(HOME_POSITION, ABOVE_HOME, BLOCK_INTERFACING_TIME, pivot_foot) 
        print("move from ", HOME_POSITION, " ", ABOVE_HOME)
        
        # Move from location above home to left (hover)
        goal = [0, -2, 0, EE_direction.DOWN.value]
        print ("goal is: ", goal)
        above_goal = copy.deepcopy(goal)
        above_goal[2] = 0.5
        print("did goal survive? goal is: ", goal)

        self.move_to(ABOVE_HOME, above_goal, TRAVEL_TIME, pivot_foot)
        print("move from ", ABOVE_HOME, " ", above_goal)
        
        # Move from above goal to the goal position
        self.move_to(above_goal, goal, BLOCK_INTERFACING_TIME, pivot_foot)
        print("move from ", above_goal, " ", goal)

        print("-------------- Front leg is in place")
        sleep(3)
        
        # At this point, leading foot (@ motor 5) is back on the ground, with 1 grid cell between it and the other foot 
        # Next, the following foot moves 

        # gripper activated RAHHHH
        pivot_foot = 5 # now the pivot foot is 5
        self.latch_detach(pivot_foot)
        print("latch and detach for ", pivot_foot)

        # lift the back feet from the board       
        # EE moves straight up from board to "safe" location above the 
        self.move_to(goal, above_goal, BLOCK_INTERFACING_TIME, pivot_foot) 
        print("move from ", goal, " ", above_goal)
        
        # rotate the back feet
        self.move_to(above_goal, ABOVE_HOME, TRAVEL_TIME, pivot_foot)
        print("move from ", above_goal, " ", ABOVE_HOME)
        
        # put the back feet on the board
        # Move from above goal to the goal position
        self.move_to(ABOVE_HOME, HOME_POSITION, BLOCK_INTERFACING_TIME, pivot_foot)
        print("move from ", ABOVE_HOME, " ", HOME_POSITION)

        print("movement complete: STEP_RIGHT")
    
    # def step_forward_block(self): 
    #     block = True
    #     HOME_POSITION_BLOCK = [1, 0, 1, EE_direction.DOWN.value]
    #     ABOVE_HOME_BLOCK = [1, 0, 1.5, EE_direction.DOWN.value]
    #     goal_pivot_foot = [2, 0, 1, EE_direction.DOWN.value]
    #     above_goal_pivot_foot = copy.deepcopy(goal_pivot_foot)
    #     above_goal_pivot_foot[2] += 0.5

    #     goal_following_foot = [2, 0, 0, EE_direction.DOWN.value]
    #     above_goal_following_foot = copy.deepcopy(goal_pivot_foot)
    #     above_goal_following_foot[2] += 0.5

    #     # Start moving leading foot 
    #     pivot_foot = 1 # the pivot foot 
    #     self.latch_detach(pivot_foot, block) 
    #     print("Latched detached")
        
    #     # EE moves straight up from board to "safe" location along with the block 
    #     self.move_to(HOME_POSITION_BLOCK, ABOVE_HOME_BLOCK, BLOCK_INTERFACING_TIME, pivot_foot) 
    #     print("move from ", HOME_POSITION_BLOCK, " ", ABOVE_HOME_BLOCK)

    #     # Move forward and hover over the goal overhead position 
    #     self.move_to(ABOVE_HOME_BLOCK, above_goal_pivot_foot, TRAVEL_TIME, pivot_foot)
    #     print("move from ", ABOVE_HOME_BLOCK, " ", above_goal_pivot_foot)
        
    #     # Move from above goal to the goal position
    #     self.move_to(above_goal_pivot_foot, goal_pivot_foot, BLOCK_INTERFACING_TIME, pivot_foot)
    #     print("move from ", above_goal_pivot_foot, " ", goal_pivot_foot)

    #     print("-------------- Front leg is in place")
    #     sleep(3)
        
    #     # At this point, leading foot (@ motor 5) is back on the ground, with 1 grid cell between it and the other foot 
    #     # Next, the following foot moves 
    #     pivot_foot = 5 # now the pivot foot is 5
    #     self.latch_detach(pivot_foot)
    #     print("latch and detach for ", pivot_foot)
        
    #     # EE moves straight up from board to "safe" location with no block attached to it 
    #     self.move_to(goal, above_goal, BLOCK_INTERFACING_TIME, pivot_foot) 
    #     print("move from ", goal, " ", above_goal)
        
    #     # Move forward and hover over the goal overhead position 
    #     self.move_to(above_goal, ABOVE_HOME, TRAVEL_TIME, pivot_foot)
    #     print("move from ", above_goal, " ", ABOVE_HOME)
        
    #     # Move from above goal to the goal position
    #     self.move_to(ABOVE_HOME, HOME_POSITION, BLOCK_INTERFACING_TIME, pivot_foot)
    #     print("move from ", ABOVE_HOME, " ", HOME_POSITION)

    #     print("Movement complete: STEP_FORWARD")

    def latch_detach(self, pivot_foot, block = False):
        if (pivot_foot == 5): # 5 is the pivot foot
            # activate the servo of the following leg
            activate_servo(self.servo1)
            print("servo1 attached")
            
            if (block):
                # activate the servo of the leading leg because it's holding a block
                activate_servo(self.servo2)
                print("Servo2 attached") 
            else:
                # detach the leading leg
                release_servo(self.servo2)
                print("Servo2 detached")          
        elif (pivot_foot == 1): # 1 is the pivot foot
             # activate the servo of the following leg
            activate_servo(self.servo2)
            print("servo2 attached")

            if (block):
                # activate the servo of the leading leg because it's holding a block
                activate_servo(self.servo1)
                print("Servo1 attached")
            else:
                # detach the leading leg
                release_servo(self.servo1)
                print("Servo1 detached")     

## Due to indentation things, these two functions (activate/release servo) are not part of the MotorController class
# servo angle of 0 is activated, 180 released
def activate_servo(servo_id):
    """
    Activate the servo motor so that the gripper latches onto the surface. 
    Args:
        servo_id: The servo motor object to be activated. 
    """
    # Set duty cycle to move servo to 0° position (2 corresponds to 0° for most servos)
    servo_id.ChangeDutyCycle(2+(0/18))
    # Pause to allow servo to reach position
    time.sleep(1)
    # Stop sending signal to servo
    servo_id.ChangeDutyCycle(0)

# Releases the servo by moving it to 180 degrees (or a fully released position)
def release_servo(servo_id):
    """
    Release the servo motor so that the gripper detached from the surface. 
    Args:
        servo_id: The servo motor object to be released. 
    """
    # Set duty cycle to move servo to 180° position (12 corresponds to 180° for most servos)
    servo_id.ChangeDutyCycle(2+(180/18))
    # Pause to allow servo to reach position
    time.sleep(1)
    # Stop sending signal to servo
    servo_id.ChangeDutyCycle(0) 

def main(args=None):
    rclpy.init(args=args)
    ik_test = IkTest()
    rclpy.spin(ik_test)
    ik_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()