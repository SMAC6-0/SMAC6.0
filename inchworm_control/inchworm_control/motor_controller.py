#!/usr/bin/env python3
from inchworm_control.ik import inverseKinematics
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
# for servo
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
import time
from inchworm_control.lewansoul_servo_bus import ServoBus
from time import sleep 

class MotorController(Node):
    def __init__(self):
        """
        Initialization method for the motor controller node.

        Initializes ROS2 publisher, subscriber, GPIO pins, motor angles, and step actions for the inchworm robot.
        """
        # Initialize the ROS2 node with the name 'motor_controller'
        super().__init__('motor_controller')

        # Create a publisher for the 'step_status' topic, which sends Float32 messages
        self.publisher_ = self.create_publisher(Float32, 'step_status', 10)

        # Create a subscriber for the 'motor_command' topic, which listens for String messages
        # The messages are handled by the listener_callback method
        self.subscription = self.create_subscription(
            String,
            'motor_command',
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
            'STEP_FORWARD': self.step_forward,
            'STEP_FORWARD_BLOCK': self.step_forward_block,
            'STEP_LEFT': self.step_left,
            'STEP_RIGHT': self.step_right,
            'STEP_LEFT_BLOCK': self.step_left_block,
            'STEP_RIGHT_BLOCK': self.step_right_block,
            'GRAB_UP_FORWARD': self.grab_up_forward, 
            'GRAB_UP_LEFT': self.grab_up_left, 
            'PLACE_FORWARD_BLOCK': self.place_forward,
            'PLACE_UP_FORWARD_BLOCK': self.place_up_forward,
            'PLACE_UP_2_FORWARD_BLOCK': self.place_up_2_forward,
            'SIMPLIFIED_POS_1_DOWN_1': self.step_down_1,
            'SIMPLIFIED_POS_1_DOWN_2': self.step_down_2
            # Add more mappings as needed
        }

    def listener_callback(self, msg):
        """
        Callback function for the motor_command subscriber.

        Processes incoming commands, executes the corresponding step action, and publishes the step status.
        Throws errors for failure of command execution.
        """
        self.get_logger().info('Received command to "%s' % msg.data)
        try:
            # Get the step action from the step_actions dictionary based on the received message
            action = self.step_actions.get(msg.data)

            if action:
                # If a valid action (step) is found, execute the action with which_foot_motor (1 for this case)
                action(1)
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

    def move_to(self, theta2, theta3, theta4, time):
        """
        Move motors to specified angles over a given time duration.

        Args:
            theta2 (float): Target angle for motor 2.
            theta3 (float): Target angle for motor 3.
            theta4 (float): Target angle for motor 4.
            time (float): Duration to reach the target angles (in seconds).
        """
        # TODO: Look into whether it's worth calling self.time_to_move here rather than passing in time as a parameter.
        self.motor_2.move_time_write(theta2, time)
        self.motor_3.move_time_write(theta3, time)
        self.motor_4.move_time_write(theta4, time)

        # Pause the program to allow the motors to finish moving. 
        sleep(time)
    
    ## STEP DEFINITIONS
    """
    The basic procedure of the step functions is:  
    1. Call it, specifying the foot motor
    TODO: Look into if which_foot_motor is EVER equal to 5, and hence if the if statements checking which_foot_motor 
    are irrelevant. 
    My thoughts are that it mayyy be relevant if trying to make inchworm movement bidirectional?
    2. Move gripper servos to attach/deattach 
    3. Use inverse kinematics to move to a series of specific positions that make the movement possible 
    4. Move gripper servos to attach
    """ 

    def step_forward(self, which_foot_motor):
        """
        Step forward leading with the specified foot motor. Handles the stepping motion by activating servos 
        and moving the robotic leg through various angles using inverse kinematics. 

        Args:
            which_foot_motor (int): The foot motor to activate (1 for the leading foot, 5 for the following).
        """
        print('stepping forward')
        if which_foot_motor == 1: 
            # Deatach leading foot, attach following foot 
            activate_servo(self.servo1)
            release_servo(self.servo2)
            # Move the leading foot up
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3,0,2, which_foot_motor)
            theta4 += 15 # adjust EE to point straight down 
            # TODO: implement trajectory planning 
            # TODO: instead of hard coded values, establish constant heights or "levels" (as in bring_back_leg_to_block)
            self.move_to(theta2, theta3, theta4, self.time_to_move)
            
            # Move the leading foot forward
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(6.2,0,2, which_foot_motor)
            theta4 += 20
            self.move_to(theta2, theta3, theta4, self.time_to_move)

            # Move the leading foot down 
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(6.2,0,0, which_foot_motor)
            self.move_to(theta2, theta3, theta4+5, 1.5)
            activate_servo(self.servo2)
            # sleep(1)
            # activate_servo(self.servo2)
            ## Move the following foot forward 
            #angle back leg to remove from magnetic connection
            self.bring_back_leg_to_block(which_foot_motor, 0)

        elif which_foot_motor == 5:
            pass

    def step_forward_block(self, which_foot_motor):
        """
        Step forward leading with the specified foot motor, which is currently holding a block. Handles the stepping motion by activating servos 
        and moving the robotic leg through various angles using inverse kinematics. 

        Args:
            which_foot_motor (int): The foot motor to activate (1 for the leading foot, 5 for the following).
        """
        print('stepping forward with block')

        # If which_foot_motor is 1, the base is motor 1 
        if which_foot_motor == 1: 
            self.bring_block_forward(which_foot_motor)
            self.bring_back_leg_to_block(which_foot_motor, 1)

        # If which_foot_motor 5, base is
        elif which_foot_motor == 5:
            ##move first 
            # move up
            # this first part currently does not act well because the servo does not fully actuate and the leg gets caught on the other leg
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(6,0,4,which_foot_motor)
            theta4 += 20
            activate_servo(self.servo1)
            self.move_to(theta2, theta3, theta4,self.time_to_move)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(6,0,6,which_foot_motor)
            activate_servo(self.servo1)
            self.move_to(theta2, theta3, theta4,self.time_to_move)
            
            # move forward
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(9,0,6,which_foot_motor)
            #theta4 += 40
            self.move_to(theta2, theta3, theta4,self.time_to_move)

            # move down
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(9,0,4,which_foot_motor)
            #theta4 += 20
            self.move_to(theta2, theta3, theta4,self.time_to_move)
            activate_servo(self.servo2)
        
        else:
            ValueError('are you stupid there is only 1 and 5????')

    def step_left(self, which_foot_motor):
        print('stepping left')
        if which_foot_motor == 1: 
            release_servo(self.servo2)
            activate_servo(self.servo1)

            self.lift_up(2.5)
            self.turn("left") 

            # mid-step allign
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.2,3.2,2.5,which_foot_motor)
            self.move_to(theta2, theta3, theta4+5, 1)

            # Place the EE to final pose 
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.2,3.2,0,which_foot_motor)
            self.move_to(theta2, theta3, theta4+5, 1)
            sleep(1)
            activate_servo(self.servo2)
            release_servo(self.servo1)

            # Second leg from here 
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.3,-3.3,1,5)
            self.move_to(theta2-10, theta3, theta4, 1)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.1,-3,4,5)
            self.move_to(theta2-10, theta3, theta4, 1)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.3,0,4,5)
            self.move_to(theta2, theta3, theta4, self.time_to_move)
            self.motor_1.move_time_write(theta1, self.time_to_move)
            self.motor_5.move_time_write(theta5-2, self.time_to_move)
            sleep(self.time_to_move)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.5,0,1.5,5)
            self.move_to(theta2, theta3, theta4, self.time_to_move)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.5,0,0,5)
            self.move_to(theta2+5, theta3, theta4, self.time_to_move)
            activate_servo(self.servo1)
        
        elif which_foot_motor == 5:
            pass

    def step_right(self, which_foot_motor):
        print('stepping right')
        pass

    def step_left_block(self, which_foot_motor):
        print('stepping left with a block')
        if which_foot_motor == 1:
            self.lift_up_block(1,5) # 3.3, 0, 
            self.turn_block("left") # 5, 5, 6.5
         

            # Place block down 
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.3, 3.3, 2.9, which_foot_motor)
            self.move_to(theta2+5, theta3, theta4+5, 1)

            activate_servo(self.servo2)

            self.pick_up_back_leg()

            #Second leg from here 
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.3, -3.3, -1, 5)
            self.move_to(theta2-10, theta3, theta4, 1)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.1, -3, -1, 5)
            self.move_to(theta2-10, theta3, theta4, 1)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.3, 0, -1, 5)
            self.move_to(theta2, theta3, theta4, self.time_to_move)

            self.motor_1.move_time_write(theta1-5, self.time_to_move)
            self.motor_5.move_time_write(theta5-2, self.time_to_move)
            sleep(self.time_to_move)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.5, 0, -1.5, 5)
            self.move_to(theta2-10, theta3, theta4, self.time_to_move)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.5, 0, -3, 5)
            self.move_to(theta2, theta3, theta4, self.time_to_move)
            activate_servo(self.servo1)
        
        elif which_foot_motor == 5:
            pass

    def step_right_block(self, which_foot_motor):
        print('stepping right with a block')

        if which_foot_motor == 1:
            pass

        elif which_foot_motor == 5:
            pass

    def grab_up_forward(self, which_foot_motor):
        print('grabbing up forward')

        if which_foot_motor == 1: 
            activate_servo(self.servo1)
            release_servo(self.servo2)
            sleep(1)

            print("before lift up")
            self.lift_up(5)
            print("after lift up")

            # move above block
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(6.25,0,5,which_foot_motor)
            self.move_to(theta2, theta3, theta4, self.time_to_move)

            # move on top block
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(6.25,0,3,which_foot_motor)
            self.move_to(theta2+10, theta3, theta4+10, self.time_to_move)
            
            activate_servo(self.servo2)
            self.bring_back_leg_to_block(1,1)

        # If foot 5, base is motor 5
        elif which_foot_motor == 5:
           pass

        else:
            ValueError('are you stupid there is only 1 and 5????')

    def grab_up_left(self, which_foot_motor):
        print("grabbing up left")
        if which_foot_motor == 1:
            #lift up
            #turn
            release_servo(self.servo2)
            activate_servo(self.servo1)

            self.lift_up(5)
            self.turn_block("left")      
            sleep(2)
            self.motor_5.move_time_write(self.motor_5.pos_read()+5, self.time_to_move)

            self.motor_4.move_time_write(10, self.time_to_move)
            

            # Place the EE to final pose 
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3, 4.3, 2.85,which_foot_motor)
            self.move_to(theta2, theta3, theta4, 1)
            sleep(1)
            activate_servo(self.servo2)
            release_servo(self.servo1)

            # Second leg from here 
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.3, -3.3, 1, 5)
            self.move_to(theta2-10, theta3, theta4, 1)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.1, -3, 3, 5)
            self.move_to(theta2-10, theta3, theta4, 1)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.3, 0, 4, 5)
            self.move_to(theta2, theta3, theta4, self.time_to_move)
            self.motor_1.move_time_write(theta1, self.time_to_move)
            self.motor_5.move_time_write(theta5-2, self.time_to_move)
            sleep(self.time_to_move)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.5, 0, 1.5, 5)
            self.move_to(theta2-5, theta3, theta4, self.time_to_move)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.5, 0, -3.2, 5)
            self.move_to(theta2, theta3, theta4, self.time_to_move)
            activate_servo(self.servo1)

        elif which_foot_motor == 5:
            pass
        
    def place_forward(self, which_foot_motor):
        print("placing forward")
        if which_foot_motor == 1:
            activate_servo(self.servo1)
            self.step_forward_block(1)
            release_servo(self.servo2)

        elif which_foot_motor == 5:
            pass

    def place_up_forward(self, which_foot_motor):
        print("placing up forward")
        if which_foot_motor == 1:
            activate_servo(self.servo1)
            self.lift_up_block(1,8)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(6.2,0,9,which_foot_motor)
            self.move_to(theta2, theta3, theta4, 1.5)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(6.5,0,6,which_foot_motor)
            self.move_to(theta2+10, theta3, theta4+15, 1.5)
            sleep(1)

            # #bring back foot in
            self.bring_back_leg_to_block2(which_foot_motor, 2)
            release_servo(self.servo2)
        
        elif which_foot_motor == 5:
            pass
    
    def place_up_2_forward(self, which_foot_motor):
        print("placing up 2 forward")
        if which_foot_motor == 1:
            
            activate_servo(self.servo1)
            activate_servo(self.servo2)
                
            self.motor_2.move_time_write(65, 2)
            sleep(2)
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(2.2,0,11,which_foot_motor)
            self.move_to(theta2, theta3, theta4, 2)
            
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(6,0,11,which_foot_motor)
            self.move_to(theta2, theta3, theta4, 1)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(6,0,9,which_foot_motor)
            self.move_to(theta2, theta3, theta4, 1)

        elif which_foot_motor == 5:
            pass

    def step_down_1(self, which_foot_motor):
        print("stepping down 1")
        if which_foot_motor == 1:
            # First part of the movement
            activate_servo(self.servo1)
            release_servo(self.servo2)

            self.motor_2.move_time_write(self.motor_2.pos_read()-20,0.2)
            sleep(1)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3, 0, 4.7, which_foot_motor)
            self.move_to(theta2, theta3, theta4+15, self.time_to_move)

            self.turn("left", 90)
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(0, 6, 2.5, which_foot_motor)
            self.move_to(theta2, theta3, theta4, self.time_to_move)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(0, 6, 1, which_foot_motor)
            self.move_to(theta2, theta3, theta4, self.time_to_move)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(0, 6.8, 0, which_foot_motor)
            self.move_to(theta2, theta3, theta4+5, self.time_to_move)

            activate_servo(self.servo2)

            self.pick_up_back_leg()

            # Second leg starting here 
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(0, 6, 3, 5)
            self.move_to(theta2-20, theta3, theta4, self.time_to_move)

            # bring the back foot in
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(0, 3.2, 3, 5)
            self.move_to(theta2-5, theta3, theta4, self.time_to_move)

            # turn the back foot
            theta1 = self.motor_1.pos_read()
            self.motor_1.move_time_write(theta1-92, self.time_to_move)
            sleep(self.time_to_move)

            # place down
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(0, 3.25, 0, 5)
            self.move_to(theta2, theta3, theta4, self.time_to_move)

        elif which_foot_motor == 5:
            pass

    def step_down_2(self, which_foot_motor):
        print("stepping down 2")
        if which_foot_motor == 1:
            # apply angle to lift front foot off
            # First part of the movement
            activate_servo(self.servo1)
            release_servo(self.servo2)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3, 0, 8.2, which_foot_motor)
            self.move_to(theta2, theta3, theta4+15, self.time_to_move)

            self.turn("left", 90)
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(0, 6.25, 5, which_foot_motor)
            self.move_to(theta2, theta3, theta4, self.time_to_move)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(0, 6.25, 0, which_foot_motor)
            self.move_to(theta2, theta3, theta4+5, self.time_to_move)

            activate_servo(self.servo2)

            # # place down
            self.pick_up_back_leg()

            # Second leg starting here 
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(0, 6, 3, 5)
            self.move_to(theta2-20, theta3, theta4, self.time_to_move)

            # bring the back foot in
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(0, 3.2, 3, 5)
            self.move_to(theta2-5, theta3, theta4, self.time_to_move)

            # turn the back foot
            theta1 = self.motor_1.pos_read()
            self.motor_1.move_time_write(theta1-92, self.time_to_move)
            sleep(self.time_to_move)

            # place down
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(0, 3.25, 0, 5)
            self.move_to(theta2, theta3, theta4, self.time_to_move)

        elif which_foot_motor == 5:
            pass

    def lift_up_block(self, which_foot_motor, target):
        print("lifting up")
        if which_foot_motor == 1:
            # print(self.motor_2.vin_read())
            activate_servo(self.servo1)
            activate_servo(self.servo2)
            
            self.motor_2.move_time_write(70, 1)
            
            sleep(1)
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.3,0,target,which_foot_motor)
            self.move_to(theta2, theta3, theta4+15, 2)

        elif which_foot_motor == 5:
            pass

    def lift_up(self, target):
            # move up
            # print("1")
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3,0,1,1)
            self.move_to(theta2, theta3, theta4+10, 2)
            # print("2")

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3,0,2,1)
            # print("here")
            self.move_to(theta2, theta3, theta4+20, 3)
            # print("3")

            #move up more
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3,0,target,1)
            self.move_to(theta2, theta3, theta4+20, 1) 
            # print("4")

    def bring_block_forward(self, which_foot_motor):
        if which_foot_motor == 1:
            self.lift_up_block(1,7)
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(6,0,7,which_foot_motor)
            self.move_to(theta2, theta3, theta4+15, 2)

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(6.25,0,3,which_foot_motor)
            self.move_to(theta2, theta3, theta4+7.5, 2)

        elif which_foot_motor == 5:
            pass

    def bring_back_leg_to_block(self, which_foot_motor, level):
        """
        Move the following leg so it is next to the leading leg, which is currently on top of a block.

        Args:
            which_foot_motor (int): The foot motor currently on the block (1 for the leading foot, 5 for the following).
            level (int): The elevation of the foot - the block level 
        """
        offset2 = 0
        offset = -25
        if level == 1:
            target = -3
        elif level == 0:
            target = 0
            offset2 = .9
        else: 
            target = -6
            offset = 15

        if which_foot_motor == 1:
            self.pick_up_back_leg()
            # move the back leg up
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(6,0,(target+1),5)
            self.move_to(theta2-offset, theta3-10, theta4, 3)
            sleep(1)
            # move the back leg in
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.2,0,(target+1),5)
            self.move_to(theta2, theta3, theta4, 3)
            # move the back leg down
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.2+offset2,0,target,5)
            self.move_to(theta2, theta3, theta4, 3)

            activate_servo(self.servo1)
        elif which_foot_motor == 5:
            pass       

    def bring_back_leg_to_block2(self, which_foot_motor, level):
        offset = 25
        if level == 1:
            target = -3
        else: 
            target = -6
            offset = 15

        if which_foot_motor == 1:
            # self.pick_up_back_leg()
            release_servo(self.servo1)
            self.motor_2.move_time_write(self.motor_2.pos_read()+5, 1)
            sleep(2)
            # move the back leg up
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(6,0,-4.5,5)
            self.move_to(theta2-offset, theta3-20, theta4, 1)
            # sleep(1)
            # move the back leg in
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.2,0,(target+2),5)
            self.move_to(theta2, theta3, theta4, 3)
            # move the back leg down
            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.2,0,target,5)
            self.move_to(theta2, theta3, theta4, 3)
            activate_servo(self.servo1)
        elif which_foot_motor == 5:
            pass  

    def turn(self, direction, degree = 50):
    # It doesn't handle other than 50 or 90. Either turn diagonal or turn 90 degree. 
        if direction == "left":
            if degree == 90:
                [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(0.2,6,5,1)
                theta1-=2
                turn = 3
            elif degree == 50:
                [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(4,3.3,2.5,1)
                turn = 50
            else: 
                ValueError("The turn values should be given in 90 or 50 degree. Default is 50")
            # theta3 -= 10
            self.motor_1.move_time_write(theta1, self.time_to_move)
            self.motor_5.move_time_write(theta5-turn, self.time_to_move)
            sleep(self.time_to_move)
            self.move_to(theta2, theta3, theta4, self.time_to_move)
            # Rotate just the end-effector
    
        if direction == "right":
            pass

    def turn_block(self, direction):
        if direction == "left":

            [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.3, 3.3, 4, 1)
            self.motor_5.move_time_write(theta5-50, self.time_to_move)
            sleep(self.time_to_move)
            self.move_to(theta2, theta3, theta4, 1)
            # theta3 -= 10
            self.motor_1.move_time_write(theta1, self.time_to_move)
            sleep(self.time_to_move)


            # [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(3.5, 3.5, 4.5, 1)
            # self.move_to(theta2, theta3, theta4+5, 1)
            # self.move_to(theta2, theta3, theta4, self.time_to_move)

            # Rotate just the end-effector

            
        if direction == "right":
            pass

    def step_forward_wide(self):
        print('stepping forward wide')
        release_servo(self.servo2)

        ##move first 
        # move up
        [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(6,0,4,1)
        theta4 += 20
        activate_servo(self.servo1)
        self.move_to(theta2, theta3, theta4,self.time_to_move)
        # print('motor4 pose', theta4)
        
        # move forward
        [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(9,0,3,1)
        theta4 += 20
        self.move_to(theta2, theta3, theta4,self.time_to_move)
        # print('motor4 pose', theta4)


        # move down
        [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(9,0,0,1)
        theta4 += 0
        self.move_to(theta2, theta3, theta4,self.time_to_move)
        activate_servo(self.servo2)
      


    def pick_up_back_leg(self):
        release_servo(self.servo1)
        self.motor_2.move_time_write(self.motor_2.pos_read()+15, 1)
        sleep(1)

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
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()