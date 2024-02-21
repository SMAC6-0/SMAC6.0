#!/usr/bin/env python3
import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import Float32, String  # or whatever message type you need
from inchworm_control.msg import Tuple 

class StepPublisher(Node):

    def __init__(self):
        super().__init__('step_publisher')
        self.publisher_ = self.create_publisher(Tuple, 'motor_command', 10)
        # subscription to receive when a step is completed
        self.subscription = self.create_subscription(
            Float32,
            'step_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.get_logger().info('Node starting')

        # FOR ROBOT_WS:
        self.file_path = '~/robot_ws/src/MQP/inchworm_control/block_simulation/steps.txt'
        # FOR DEV_WS:
        # self.file_path = '~/MQP/dev_ws/src/inchworm_control/block_simulation/steps.txt'
        self.file_path = os.path.expanduser(self.file_path)
        # Read and get steps from file
        self.steps = read_file_callback(self)    
        print('steps FROM STEP_PUBLISHER', self.steps)
        
        msg = Tuple()
        step = self.steps.pop(0)
        msg.step = step[0]
        msg.holding_block = step[1]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing in INIT: {step}')

    def listener_callback(self, msg):
        step_status = msg.data
        self.get_logger().info('Received step status %s' % step_status)
        try:
            # successful step
            if step_status == 0.0:
                self.get_logger().info("movement complete")
                # steps is a list of tuples like (<step type>, holding_block)
                # if holding block is true, it is holding a block for this step
                msg = Tuple()
                step = self.steps.pop(0)
                msg.step = step[0]
                msg.holding_block = step[1]
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: {step}')
            # step error
            elif step_status == 1.0:
                self.get_logger().info("movement error")

        except Exception as e:
            self.get_logger().error('Failed to move servo: "%s"' % str(e))
    
# reads from a file and returns result
def read_file_callback(self):
    try:
        with open(self.file_path, 'r') as file:
            file_content = file.readlines()  # Read lines from file
            steps = []
            for line in file_content:
                # Parse the string representation of the tuple
                step_tuple = eval(line.strip())  # Convert string to tuple
                steps.append(step_tuple)
            return steps
    except Exception as e:
        self.get_logger().error('Failed to read file: "%s"' % str(e))
        return []
    
def main(args=None):
    rclpy.init(args=args)
    step_publisher = StepPublisher()
    rclpy.spin(step_publisher)
    step_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()