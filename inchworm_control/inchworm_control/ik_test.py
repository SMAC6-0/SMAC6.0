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
from enum import ENUM

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
            'STEP_FORWARD': self.step_forward
            # 'STEP_FORWARD_BLOCK': self.step_forward_block,
            # 'STEP_LEFT': self.step_left,
            # 'STEP_RIGHT': self.step_right,
            # 'STEP_LEFT_BLOCK': self.step_left_block,
            # 'STEP_RIGHT_BLOCK': self.step_right_block,
            # 'GRAB_UP_FORWARD': self.grab_up_forward, 
            # 'GRAB_UP_LEFT': self.grab_up_left, 
            # 'PLACE_FORWARD_BLOCK': self.place_forward,
            # 'PLACE_UP_FORWARD_BLOCK': self.place_up_forward,
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
                # If a valid action (step) is found, execute the action with which_foot_motor (1 for this case)
                action()
                # TODO: Determine when which_foot_motor == 5 is passed into the step functions 
            else:
                # Log a warning if the action is not recognized
                self.get_logger().warn('Unknown command: %s' % msg.data)
            sleep(1)
            
            # Create a new Float32 message to publish the step status
            # 0.0 indicates the step was successful, 1.0 indicates an error occurred
            msg = Float32()
            msg.data = 0.0
            
            # Publish the step status to the 'step_status' topic
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            
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

        
    def move_to(self, current_pos, final_pos, travelTime, which_foot_motor): 
        """
        Move the robot end effector between one location and another using quintic trajectory. 

        Args: 
            current_pos (list): the current position of the EE as a 1x4 vector
            final_pos (list): the final location of the EE as a 1x4 vector
            travelTime (float): the time taken for the movement
            which_foot_motor (int): Motor identifier (1 or 5) corresponding to the foot.
        """
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
        self.run_trajectory(q_t, travelTime, which_foot_motor)
    
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

    def run_trajectory(self, trajCoeffs, totTime, which_foot_motor):
        """
        Calculates current joint positions based on trajectory coefficients and current time.
        
        Args:
            trajCoeffs (list): [6x4 float] trajectory coefficients generated from quintic_trajectory()
            totTime (double): total amount of time it takes for trajectory to reach target position
            which_foot_motor (int): Motor identifier (1 or 5) corresponding to the foot.
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
            joint_ang = inverseKinematics(x, y, z, alpha, which_foot_motor) # the joint angles
            
            self.move_joints(joint_ang, 0.5) # running the motors to get to the point

            sleep(1/10)
            toc = time.perf_counter()
            time_s = toc - tic
        
    """
    The territory of movesets begins now...
    """

    class EE_direction(Enum):
        DOWN = 90
        UP = 0

    home_position = [1, 0, 0, EE_direction.DOWN]
    above_home = [1, 0, 0.5, EE_direction.DOWN]
    block_interface_time = 1
    travel_time = 2

    def step_forward(self): 
        # Start moving leading foot 
        which_foot_motor = 1 # the pivot foot 
        self.latch_detach(which_foot_motor) 

        # EE moves straight up from board to "safe" location above the 
        self.move_to(self.home_position, self.above_home, self.block_interface_time, which_foot_motor) 
        
        # Move from location above home forward 
        goal = [2, 0, 0, self.EE_direction.DOWN]
        above_goal = goal
        above_goal[2] = 0.5

        # Move forward and hover over the goal overhead position 
        self.move_to(self.above_home, above_goal, self.travel_time, which_foot_motor)
        
        # Move from above goal to the goal position
        self.move_to(above_goal, goal, self.block_interface_time, which_foot_motor)
        
        # At this point, leading foot (@ motor 5) is back on the ground, with 1 grid cell between it and the other foot 
        # Next, the following foot moves 
        which_foot_motor = 5 # now the pivot foot is 5
        self.latch_detach(which_foot_motor)
        
        # EE moves straight up from board to "safe" location above the 
        self.move_to(goal, above_goal, self.block_interface_time, which_foot_motor) 
        
        # Move forward and hover over the goal overhead position 
        self.move_to(above_goal, self.above_home, self.travel_time, which_foot_motor)
        
        # Move from above goal to the goal position
        self.move_to(self.above_home, self.home_position, self.block_interface_time, which_foot_motor)
        
      
        
    def latch_detach(self, which_foot_motor):
        if (which_foot_motor == 1): # 1 is the pivot foot
            # detach the leading leg
            release_servo(self.servo2)

            # activate the servo of the following leg
            activate_servo(self.servo1)
        elif (which_foot_motor == 5): # 5 is the pivot foot
            # detach the leading leg
            release_servo(self.servo1)

            # activate the servo of the following leg
            activate_servo(self.servo2)

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