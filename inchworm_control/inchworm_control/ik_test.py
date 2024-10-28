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
        
        # Constant, the width of the blocks 
        self.CUBE_WIDTH = 3 

        

    def listener_callback(self, msg):
        """
        Callback function for the ik_command subscriber.

        Processes incoming commands, executes the corresponding step action, and publishes the step status.
        Throws errors for failure of command execution.
        """
        self.get_logger().info('Received command to "%s' % msg.data)
        try:
            pos = msg.data
            positions = pos.split(', ')
            

            # [inputX, inputY, inputZ] = self.adjust_positions(float(positions[0]), float(positions[1]), float(positions[2]))
            # [theta1, theta2, theta3, theta4, theta5] = inverseKinematics(inputX, inputY, inputZ, float(positions[3]), float(positions[4]))
            
            # self.move_to(theta1, theta2, theta3, theta4, theta5, 2)

            # Trajectory planning
            self.move_to([1,0,0, 90], [1,0,1, 90], 2, 1) # move 1 block up from board to safe location.
            self.move_to([1,0,1, 90], [float(positions[0]), float(positions[1]), float(positions[2]), float(positions[3])], 2, 1)


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

    def adjust_positions(self, goal_X: float, goal_Y: float, goal_Z: float): 
        """
        Helper function to adjust the goal EE position to a format digestable for the inverse kinematics.

        Args:
            goal_X (float): the desired X position in number of blocks. 
            goal_Y (float): the desired Y position in number of blocks. 
            goal_Z (float): the desired Z position in number of blocks. 
        """
        inputX = self.CUBE_WIDTH * 1.144 * goal_X + 0.1242
        inputY = self.CUBE_WIDTH * 1.1595 * goal_Y + 0.0249
        inputZ = self.CUBE_WIDTH * 1.0786 * goal_Z - 0.0432

        return [inputX, inputY, inputZ]
        
    def move_to(self, current_pos, final_pos, travelTime, which_foot_motor): 
        """
        Move the robot end effector between one location and another using quintic trajectory. 

        Args: 
            current_pos (list): the current position of the EE as a 1x4 vector
            final_pos (list): the final location of the EE
            travelTime (float): the time taken for the movement
            which_foot_motor (int): Motor identifier (1 or 5) corresponding to the foot.
        """
        current_pos = np.transpose(current_pos)
        final_pos = np.transpose(final_pos)

        print("Current_pos", current_pos)
        print("final_pos", final_pos)
        # trajectory planning to move from above object to on object
        x = quintic_trajectory(0,travelTime, current_pos[0], final_pos[0], 0, 0, 0, 0) #  X
        y = quintic_trajectory(0,travelTime, current_pos[1], final_pos[1], 0, 0, 0, 0) # Y
        z = quintic_trajectory(0,travelTime, current_pos[2], final_pos[2], 0, 0, 0, 0) # Z
        alpha = quintic_trajectory(0,travelTime, current_pos[3], final_pos[3], 0, 0, 0, 0) # Alpha

        q_t = [x, y, z, alpha]

        print("Q_T", q_t)
        print("q_t shape: ", np.shape(q_t))

        # run trajectory for task space
        self.run_trajectory(q_t, travelTime, which_foot_motor)
        print("run_trajectory YYAYY")
        
    
    
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
        # # Pause the program to allow the motors to finish moving. 
        # sleep(time)
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
            trajCoeffs (tuple): trajectory coefficients generated from [4x6 double] quintic_trajectory(), for 5 joints
            totTime (double): total amount of time it takes for trajectory to reach target position
            which_foot_motor (int): Motor identifier (1 or 5) corresponding to the foot.
        """

        print("in Run Trajectory")
        timeMat = np.zeros((1,1))
        trajMat = np.zeros((1,5))
        zeroVec = np.zeros((1,5))
        newTrajCoeffs = trajCoeffs    
        time = 0
        print("trajCoeffs size: ", np.shape(trajCoeffs))
        print("timeMat size: ", np.shape(timeMat))
        print("zeroVec size: ", np.shape(zeroVec))

        print("before if trajCoeff == 5 ")
        # modify trajCoeffs and make it 5x6 matrix. If it's a 5x4
        # matrix, add 2 zeroVec to make them 5x6

        if(len(trajCoeffs[0]) == 5):
            newTrajCoeffs = np.concatenate((newTrajCoeffs , zeroVec, zeroVec), axis=0) # Concatenate vertically 
        
        print("newTrajCoeffs size: ", np.shape(newTrajCoeffs))
        
        tic = time.perf_counter()
        print("tic", tic)

        print("before while loop")
        while(time < totTime):
            # toc = time.perf_counter()

            print("in while loop")
            # Calculate coeffs accepts 6x5
            x = newTrajCoeffs[0][0] + newTrajCoeffs[1][0]*time + newTrajCoeffs[2][0]*pow(time,2) + newTrajCoeffs[3][0]*pow(time,3) + newTrajCoeffs[4][0]*pow(time,4) + newTrajCoeffs[5][0]*pow(time,5)
            y = newTrajCoeffs[0][1] + newTrajCoeffs[1][1]*time + newTrajCoeffs[2][1]*pow(time,2) + newTrajCoeffs[3][1]*pow(time,3) + newTrajCoeffs[4][1]*pow(time,4) + newTrajCoeffs[5][1]*pow(time,5)
            z = newTrajCoeffs[0][2] + newTrajCoeffs[1][2]*time + newTrajCoeffs[2][2]*pow(time,2) + newTrajCoeffs[3][2]*pow(time,3) + newTrajCoeffs[4][2]*pow(time,4) + newTrajCoeffs[5][2]*pow(time,5)
            alpha = newTrajCoeffs[0][3] + newTrajCoeffs[1][3]*time + newTrajCoeffs[2][3]*pow(time,2) + newTrajCoeffs[3][3]*pow(time,3) + newTrajCoeffs[4][3]*pow(time,4) + newTrajCoeffs[5][3]*pow(time,5)
            # alpha = 90
            
            pos = [x, y, z] #  The modified position
            print("pos size: ", np.shape(pos))
            print("pos   ", pos)

            # running the inverseKinematics to get the joint angles
            [x, y, z] = self.adjust_positions(x, y, z)
            joint_ang = inverseKinematics(x, y, z, alpha, which_foot_motor) # the joint angles
            trajMat = np.concatenate((trajMat, [pos, alpha]), axis=0) # Storing the x, y, z position and alpha


            print("joint_ang   ", joint_ang)
            print("trajMat   ", trajMat)
            
            self.move_joints(joint_ang, 0.5) # running the motors to get to the point

            print("moved joints")
            timeMat = np.concatenate((timeMat, time), axis=0) # stores time data
            # tic resets the timing of timeMat, so travel time and the number
            # of loop iterations is considered to keep timing conssitent
            sleep(1/10)
            toc = time.perf_counter()
            time = toc - tic
        
        print("trajectory ran YIPEEE")
        return np.concatenate((timeMat, trajMat), axis=1)

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