#!/usr/bin/env python3
from inchworm_control.ik import inverseKinematicsMQP
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
# for servo
import RPi.GPIO as GPIO
import time
from inchworm_control.lewansoul_servo_bus import ServoBus
from time import sleep 

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.publisher_ = self.create_publisher(Float32, 'step_status', 10)
        self.subscription = self.create_subscription(
            String,
            'motor_command',
            self.listener_callback,
            10)
        self.subscription

        self.servo_bus = ServoBus('/dev/ttyUSB0')  # /dev/ttyUSB0 is port for RP /dev/ttylUSB0 or USB1 for cambria(switches randomly)
        # self.servo_bus = ServoBus('/dev/ttyAMA0')  
        self.get_logger().info('Node starting')

        # init motors
        self.init_motors()

        # init servos
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(11,GPIO.OUT)
        GPIO.setup(13,GPIO.OUT)
        self.servo1 = GPIO.PWM(11,50) # pin 11 for servo1, pulse 50Hz
        self.servo2 = GPIO.PWM(13,50) # pin 13 for servo1, pulse 50Hz
        self.servo1.start(0)
        self.servo2.start(0)
        # MOTOR CANNOT GO NEGATIVE 
        # 
        # init motor angles
        # print(self.motor_1.pos_read(), self.motor_2.pos_read(), self.motor_3.pos_read(), self.motor_4.pos_read(), self.motor_5.pos_read())
        print(self.motor_1.pos_read())

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
        self.get_logger().info('Received command to "%s' % msg.data)

        try:
            # release_servo(self.servo1)
            # print(self.motor_5.vin_read())
            # print(self.motor_4.vin_read())
            # print(self.motor_3.vin_read())
            # print(self.motor_2.vin_read())
            # print(self.motor_1.vin_read())
            # self.step_forward_block(1)
            # activate_servo(self.servo2)
            # sleep(1)
            # release_servo(self.servo2)
            self.step_down_1(1)
            # action = self.step_actions.get(msg.data)

            # if action:
            #     action(1)
            # else:
            #     self.get_logger().warn('Unknown command: %s' % msg.data)
            # sleep(1)
            
            # publish step status. 0.0 means step sucessful, 1.0 means step error
            msg = Float32()
            msg.data = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            
        except Exception as e:
            self.get_logger().error('Failed to move servo: "%s"' % str(e))

    def init_motors(self):
        self.motor_1 = self.servo_bus.get_servo(1)
        self.motor_2 = self.servo_bus.get_servo(2)
        self.motor_3 = self.servo_bus.get_servo(3)
        self.motor_4 = self.servo_bus.get_servo(4)
        self.motor_5 = self.servo_bus.get_servo(5)

        self.time_to_move = 2

    def move_to(self, theta2, theta3, theta4, time):
        self.motor_2.move_time_write(theta2, time)
        self.motor_3.move_time_write(theta3, time)
        self.motor_4.move_time_write(theta4, time)
        sleep(time)
    
    # STEP DEFINITIONS 
    def step_forward(self, foot):
        print('stepping forward')
        if foot == 1: 
            activate_servo(self.servo1)
            release_servo(self.servo2)
            # move up
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3,0,2,1)
            theta4 += 20
            self.move_to(theta2, theta3, theta4, self.time_to_move)
            
            # move forward
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(6.2,0,2,1)
            theta4 += 20
            self.move_to(theta2, theta3, theta4, self.time_to_move)

            # move down
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(6.2,0,0,1)
            self.move_to(theta2, theta3, theta4+5, self.time_to_move)
            activate_servo(self.servo2)
        
            ## following leg
            #angle back leg to remove from magnetic connection
            self.pick_up_back_leg()

            # Take the step up 
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(6,0,2,5)
            self.move_to(theta2, theta3, theta4, self.time_to_move)

            # Take the step forward
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3,0,3,5)
            theta2 -= 20
            self.move_to(theta2, theta3, theta4, self.time_to_move)

            # Get ready to put the step down 
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.1,0,0,5)
            self.move_to(theta2-3, theta3, theta4-5, self.time_to_move)
            activate_servo(self.servo1)

        elif foot == 5:
            pass

    def step_forward_block(self, foot):
        print('stepping forward with block')

        #If foot 1, base is motor 1 
        if foot == 1: 
            self.bring_block_forward(1)
            self.bring_back_leg_to_block(1, 1)

        # If foot 5, base is
        elif foot == 5:
            ##move first 
            # move up
            # this first part currently does not act well because the servo does not fully actuate and the leg gets caught on the other leg
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(6,0,4,5)
            theta4 += 20
            activate_servo(self.servo1)
            self.move_to(theta2, theta3, theta4,self.time_to_move)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(6,0,6,5)
            # theta4 += 20
            activate_servo(self.servo1)
            self.move_to(theta2, theta3, theta4,self.time_to_move)
            
            # move forward
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(9,0,6,5)
            #theta4 += 40
            self.move_to(theta2, theta3, theta4,self.time_to_move)

            # move down
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(9,0,4,5)
            #theta4 += 20
            self.move_to(theta2, theta3, theta4,self.time_to_move)
            activate_servo(self.servo2)
        
        else:
            ValueError('are you stupid there is only 1 and 5????')

    def step_left(self, foot):
        print('stepping left')
        if foot == 1: 
            release_servo(self.servo2)
            activate_servo(self.servo1)

            self.lift_up(2.5)
            self.turn("left") 

            # mid-step allign
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.2,3.2,2.5,1)
            self.move_to(theta2, theta3, theta4+5, 1)

            # Place the EE to final pose 
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.2,3.2,0,1)
            self.move_to(theta2, theta3, theta4+5, 1)
            sleep(1)
            activate_servo(self.servo2)
            release_servo(self.servo1)

            # Second leg from here 
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.3,-3.3,1,5)
            self.move_to(theta2-10, theta3, theta4, 1)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.1,-3,4,5)
            self.move_to(theta2-10, theta3, theta4, 1)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.3,0,4,5)
            self.move_to(theta2, theta3, theta4, self.time_to_move)
            self.motor_1.move_time_write(theta1, self.time_to_move)
            self.motor_5.move_time_write(theta5-2, self.time_to_move)
            sleep(self.time_to_move)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.5,0,1.5,5)
            self.move_to(theta2, theta3, theta4, self.time_to_move)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.5,0,0,5)
            self.move_to(theta2+5, theta3, theta4, self.time_to_move)
            activate_servo(self.servo1)
        
        elif foot == 5:
            pass

    def step_right(self, foot):
        print('stepping right')
        pass

    def step_left_block(self, foot):
        print('stepping left with a block')
        if foot == 1:
            self.lift_up_block(1,5) # 3.3, 0, 
            # activate_servo(self.servo1)
            # activate_servo(self.servo2)
            
            # self.motor_2.move_time_write(65, 2)
            
            # sleep(2)
            # theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.3,0,6,1)
            # self.move_to(theta2, theta3, theta4+15, 2)
            self.turn_block("left") # 5, 5, 6.5

            # theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(4, 4, 5, 1)
            # self.move_to(theta2, theta3, theta4, 1)
            # # theta3 -= 10
            # self.motor_1.move_time_write(theta1, self.time_to_move)
            # self.motor_5.move_time_write(theta5-50, self.time_to_move)
            # sleep(self.time_to_move)

            #align block
            # theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(4, 4, 5, 1)
            # self.move_to(theta2, theta3, theta4+5, 1)           

            # Place block down 
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.3, 3.3, 2.9, 1)
            self.move_to(theta2+5, theta3, theta4+5, 1)

            activate_servo(self.servo2)

            self.pick_up_back_leg()

            #Second leg from here 
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.3, -3.3, -1, 5)
            self.move_to(theta2-10, theta3, theta4, 1)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.1, -3, -1, 5)
            self.move_to(theta2-10, theta3, theta4, 1)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.3, 0, -1, 5)
            self.move_to(theta2, theta3, theta4, self.time_to_move)

            self.motor_1.move_time_write(theta1-5, self.time_to_move)
            self.motor_5.move_time_write(theta5-2, self.time_to_move)
            sleep(self.time_to_move)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.5, 0, -1.5, 5)
            self.move_to(theta2-10, theta3, theta4, self.time_to_move)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.5, 0, -3, 5)
            self.move_to(theta2, theta3, theta4, self.time_to_move)
            activate_servo(self.servo1)
        
        elif foot == 5:
            pass

    def step_right_block(self, foot):
        print('stepping right with a block')

        if foot == 1:
            pass

        elif foot == 5:
            pass

    def grab_up_forward(self, foot):
        print('grabbing up forward')

        if foot == 1: 
            activate_servo(self.servo1)
            release_servo(self.servo2)

            self.lift_up(5)

            # move above block
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(6.25,0,5,1)
            self.move_to(theta2, theta3, theta4, self.time_to_move)

            # move on top block
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(6.25,0,3,1)
            self.move_to(theta2+10, theta3, theta4+10, self.time_to_move)
            
            activate_servo(self.servo2)
            self.bring_back_leg_to_block(1,1)

        # If foot 5, base is motor 5
        elif foot == 5:
           pass

        else:
            ValueError('are you stupid there is only 1 and 5????')

    def grab_up_left(self, foot):
        print("grabbing up left")
        if foot == 1:
            #lift up
            #turn
            release_servo(self.servo2)
            activate_servo(self.servo1)

            self.lift_up(5)
            self.turn_block("left")
            self.motor_4.move_time_write(10, self.time_to_move)
            

            # mid-step allign
            # theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.5,3.5,5,1)
            # self.move_to(theta2, theta3, theta4+5, 1)

            # Place the EE to final pose 
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3, 4.3, 2.85,1)
            self.move_to(theta2, theta3, theta4, 1)
            # self.motor_1.move_time_write(theta1-10, self.time_to_move) 
            sleep(1)
            activate_servo(self.servo2)
            release_servo(self.servo1)

            # Second leg from here 
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.3, -3.3, 1, 5)
            self.move_to(theta2-10, theta3, theta4, 1)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.1, -3, 3, 5)
            self.move_to(theta2-10, theta3, theta4, 1)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.3, 0, 4, 5)
            self.move_to(theta2, theta3, theta4, self.time_to_move)
            self.motor_1.move_time_write(theta1, self.time_to_move)
            self.motor_5.move_time_write(theta5-2, self.time_to_move)
            sleep(self.time_to_move)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.5, 0, 1.5, 5)
            self.move_to(theta2-5, theta3, theta4, self.time_to_move)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.5, 0, -3.2, 5)
            self.move_to(theta2, theta3, theta4, self.time_to_move)
            activate_servo(self.servo1)

        elif foot == 5:
            pass
        
    def place_forward(self, foot):
        print("placing forward")
        if foot == 1:
            self.step_forward_block(1)
            release_servo(self.servo2)

        elif foot == 5:
            pass

    def place_up_forward(self, foot):
        print("placing up forward")
        if foot == 1:
            self.lift_up_block(1,8)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(6.2,0,9,1)
            self.move_to(theta2, theta3, theta4, 1)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(6.5,0,6,1)
            self.move_to(theta2, theta3, theta4+15, 1)

            #bring back foot in
            self.bring_back_leg_to_block2(1, 2)
            release_servo(self.servo2)
        
        elif foot == 5:
            pass
    
    def place_up_2_forward(self, foot):
        print("placing up 2 forward")
        if foot == 1:
            activate_servo(self.servo1)
            activate_servo(self.servo2)
                
            self.motor_2.move_time_write(65, 2)
            sleep(2)
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(2.2,0,11,1)
            # self.motor_4.move_time_write(theta4,1)
            self.move_to(theta2, theta3, theta4, 2)
            
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(6,0,11,1)
            self.move_to(theta2, theta3, theta4, 1)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(6,0,9,1)
            self.move_to(theta2, theta3, theta4, 1)

        elif foot == 5:
            pass

    def step_down_1(self, foot):
        print("stepping down 1")
        if foot == 1:
            # First part of the movement
            activate_servo(self.servo1)
            release_servo(self.servo2)

            # apply angle to lift front foot off
            # self.motor_1.move_time_write(self.motor_1.pos_read()-15, 1)
            # # self.motor_4.move_time_write(self.motor_1.pos_read()-15, 1)
            self.motor_4.move_time_write(self.motor_4.pos_read()-10,2)
            sleep(2)
            self.motor_5.move_time_write(self.motor_5.pos_read()+25,3)
            sleep(3)

            # theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3, 0, 3.4, 1)
            # self.move_to(theta2, theta3, theta4+15, self.time_to_move)

            # self.motor_5.move_time_write(self.motor_5.pos_read()-15,1)
            # sleep(1)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3, 0, 4.5, 1)
            self.move_to(theta2, theta3, theta4+15, self.time_to_move)

            self.turn("left", 90)
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(0, 4, 2.5, 1)
            self.move_to(theta2, theta3, theta4, self.time_to_move)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(0, 3.6, 1, 1)
            self.move_to(theta2, theta3, theta4, self.time_to_move)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(0, 3.35, 0, 1)
            self.move_to(theta2, theta3, theta4+5, self.time_to_move)

            # theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(0, 4, 2.5, 1)
            # self.move_to(theta2, theta3, theta4, self.time_to_move)

            # theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(0, 3.6, 1, 1)
            # self.move_to(theta2, theta3, theta4, self.time_to_move)

            # theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(0, 3.35, 0, 1)
            # self.move_to(theta2, theta3, theta4+5, self.time_to_move)
            activate_servo(self.servo2)

            self.pick_up_back_leg()

            # Second leg starting here 
            # theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(0, 6, 3, 5)
            # self.move_to(theta2-20, theta3, theta4, self.time_to_move)

            # # bring the back foot in
            # theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(0, 3.2, 3, 5)
            # self.move_to(theta2-5, theta3, theta4, self.time_to_move)

            # # turn the back foot
            # theta1 = self.motor_1.pos_read()
            # self.motor_1.move_time_write(theta1-92, self.time_to_move)
            # sleep(self.time_to_move)

            # # place down
            # theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(0, 3.25, 0, 5)
            # self.move_to(theta2, theta3, theta4, self.time_to_move)
            self.step_forward(1)

        elif foot == 5:
            pass

    def step_down_2(self, foot):
        print("stepping down 2")
        if foot == 1:
            # apply angle to lift front foot off
            # First part of the movement
            activate_servo(self.servo1)
            release_servo(self.servo2)

            self.motor_4.move_time_write(self.motor_4.pos_read()-10,2)
            sleep(2)
            self.motor_5.move_time_write(self.motor_5.pos_read()+25,3)
            sleep(3)
            
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3, 0, 8, 1)
            self.move_to(theta2, theta3, theta4+15, self.time_to_move)

            self.turn("left", 90)
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(0, 6.25, 5, 1)
            self.move_to(theta2, theta3, theta4, self.time_to_move)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(0, 6.25, 0, 1)
            self.move_to(theta2, theta3, theta4+5, self.time_to_move)

            activate_servo(self.servo2)
            release_servo(self.servo1)

            # Second leg starting here 
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(0, 6, 3, 5)
            self.move_to(theta2-20, theta3, theta4, self.time_to_move)

            # bring the back foot in
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(0, 3.2, 3, 5)
            self.move_to(theta2-5, theta3, theta4, self.time_to_move)

            # turn the back foot
            theta1 = self.motor_1.pos_read()
            self.motor_1.move_time_write(theta1-92, self.time_to_move)
            sleep(self.time_to_move)

            # place down
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(0, 3.2, 0, 5)
            self.move_to(theta2+5, theta3, theta4, self.time_to_move)

        elif foot == 5:
            pass

    def lift_up_block(self, foot, target):
        print("lifting up")
        if foot == 1:
            # print(self.motor_2.vin_read())
            activate_servo(self.servo1)
            activate_servo(self.servo2)
            
            self.motor_2.move_time_write(70, 1)
            
            sleep(1)
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.3,0,target,1)
            self.move_to(theta2, theta3, theta4+15, 2)

        elif foot == 5:
            pass

    def lift_up(self, target):
            # move up
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3,0,1,1)
            self.move_to(theta2, theta3, theta4+10, 1)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3,0,2,1)
            self.move_to(theta2, theta3, theta4+20, 1)

            #move up more
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3,0,target,1)
            self.move_to(theta2, theta3, theta4+20, 1)        

    def bring_block_forward(self, foot):
        if foot == 1:
            self.lift_up_block(1,7)
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(6,0,7,1)
            self.move_to(theta2, theta3, theta4+15, 2)

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(6.25,0,3,1)
            self.move_to(theta2, theta3, theta4+7.5, 2)

        elif foot == 5:
            pass

    def bring_back_leg_to_block(self, foot, level):
        # offset = 25
        # if level == 1:
        #     target = -3
        # else: 
        #     target = -6
        #     offset = 15
        offset = -25
        if level == 1:
            target = -3
        else: 
            target = -6
            offset = 15

        if foot == 1:
            self.pick_up_back_leg()
            # move the back leg up
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(6,0,(target+1),5)
            self.move_to(theta2-offset, theta3-10, theta4, 3)
            sleep(1)
            # move the back leg in
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.2,0,(target+1),5)
            self.move_to(theta2, theta3, theta4, 3)
            # move the back leg down
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.2,0,target,5)
            self.move_to(theta2, theta3, theta4, 3)

            activate_servo(self.servo1)
        elif foot == 5:
            pass       

    def bring_back_leg_to_block2(self, foot, level):
        offset = 25
        if level == 1:
            target = -3
        else: 
            target = -6
            offset = 15

        if foot == 1:
            self.pick_up_back_leg()
            sleep(1)
            # move the back leg up
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(6,0,-4.5,5)
            self.move_to(theta2-offset, theta3-20, theta4, 1)
            # sleep(1)
            # move the back leg in
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.2,0,(target+2),5)
            self.move_to(theta2, theta3, theta4, 3)
            # move the back leg down
            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.2,0,target,5)
            self.move_to(theta2, theta3, theta4, 3)

            activate_servo(self.servo1)
        elif foot == 5:
            pass  

    def turn(self, direction, degree = 50):
    # It doesn't handle other than 50 or 90. Either turn diagonal or turn 90 degree. 
        if direction == "left":
            if degree == 90:
                theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(0.2,6,5,1)
                turn = 0
            elif degree == 50:
                theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(4,3.3,2.5,1)
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

            theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.3, 3.3, 4, 1)
            self.motor_5.move_time_write(theta5-50, self.time_to_move)
            sleep(self.time_to_move)
            self.move_to(theta2, theta3, theta4, 1)
            # theta3 -= 10
            self.motor_1.move_time_write(theta1, self.time_to_move)
            sleep(self.time_to_move)


            # theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(3.5, 3.5, 4.5, 1)
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
        theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(6,0,4,1)
        theta4 += 20
        activate_servo(self.servo1)
        self.move_to(theta2, theta3, theta4,self.time_to_move)
        # print('motor4 pose', theta4)
        
        # move forward
        theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(9,0,3,1)
        theta4 += 20
        self.move_to(theta2, theta3, theta4,self.time_to_move)
        # print('motor4 pose', theta4)


        # move down
        theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(9,0,0,1)
        theta4 += 0
        self.move_to(theta2, theta3, theta4,self.time_to_move)
        activate_servo(self.servo2)
      
        ## following leg
        # Take the step up 
        # release_servo(self.servo1)
        # theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(9.2,0,4,5)
        # theta2 -= 30
        # self.move_to(theta2, theta3, theta4,self.time_to_move)

        # # Take the step forward
        # theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(6.6,0,2,5)
        # theta2 -= 20
        # self.move_to(theta2, theta3, theta4,self.time_to_move)

        # # Get ready to put the step down 
        # theta1, theta2, theta3, theta4, theta5 = inverseKinematicsMQP(6.6,0,0,5)
        # self.move_to(theta2, theta3, theta4,self.time_to_move)
        # activate_servo(self.servo1)         

    def pick_up_back_leg(self):
        release_servo(self.servo1)
        self.motor_2.move_time_write(self.motor_2.pos_read()+15, 1)
        sleep(1)

# servo angle of 0 is activated, 180 released
def activate_servo(servo_id):
    servo_id.ChangeDutyCycle(2+(0/18))
    time.sleep(0.7)
    servo_id.ChangeDutyCycle(0)

def release_servo(servo_id):
    servo_id.ChangeDutyCycle(2+(180/18))
    time.sleep(0.5)
    servo_id.ChangeDutyCycle(0)

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()