import math
import random
import yaml

def load_inchworm_motor_offsets(config_file):
    """
    Loads motor offsets for an inchworm robot from a YAML configuration file.
    
    Args:
        config_file (str): Path to the YAML configuration file containing motor offsets.
        
    Returns:
        dict: A dictionary containing motor offset values for theta1, theta2, theta3, theta4, and theta5.
    """
    # Open the YAML configuration file in read mode
    with open(config_file, 'r') as file:
        # Parse the YAML content and load it into a Python dictionary
        config = yaml.safe_load(file)
    
    # Return the motor offsets from the configuration dictionary
    return config['motor_offsets']

def apply_offsets(theta_values, motor_offsets):
    """
    Applies motor offsets to the calculated joint angles (theta values).
    
    Args:
        theta_values (list): A list of calculated joint angles [theta1, theta2, theta3, theta4, theta5].
        motor_offsets (dict): A dictionary containing motor offset values for each joint.
        
    Returns:
        list: A list of adjusted joint angles with the corresponding offsets applied.
    """
    return [
        theta_values[0] + motor_offsets['theta1'],
        theta_values[1] + motor_offsets['theta2'],
        theta_values[2] + motor_offsets['theta3'],
        theta_values[3] + motor_offsets['theta4'],
        theta_values[4] + motor_offsets['theta5']
    ]

def inverseKinematics(Px, Py, Pz, alpha, which_foot_motor):
    """
    Calculates inverse kinematics for inchworm robot and adjusts motor angles using offsets.
    
    Args:
        Px (float): X coordinate of the end-effector corresponding to which_foot_motor.
        Py (float): Y coordinate of the end-effector corresponding to which_foot_motor.
        Pz (float): Z coordinate of the end-effector corresponding to which_foot_motor.
        alpha (float): The angle of the wrist joint (motor 2 or 4) w.r.t. the horizontal plane. 
            0 deg for horizontal, 90 deg for pointing down.
        which_foot_motor (int): Motor identifier (1 or 5) corresponding to the foot.
    
    Returns:
        list: A list of joint angles [theta1, theta2, theta3, theta4, theta5] adjusted for motor offsets.
    """

    # These calculations are in inches 
    L_BASE = 3.125  # base to joint 1
    L1 = 1          # Joint 1 to Joint 2
    L2 = 6.625      # Joint 2 to Joint 3
    L3 = 6.625      # Joint 3 to Joint 4
    L4 = 1          # Joint 4 to Joint 5
    L_ENDEFFECTOR = 3.125  # Joint 5 to EE

    # extract the motor offsets from the yaml file
    inchworm_motor_offsets = load_inchworm_motor_offsets('/home/smac/robot_ws/src/SMAC6.0/inchworm_control/inchworm_control/inchworm_motor_config.yaml')

    # print("motor offsets: ", inchworm_motor_offsets)

    # Convert alpha angle from radians to degrees 
    alpha = math.radians(alpha)
    
    # Calculate the pose in the XZ-plane 
    Wx = math.sqrt(Px**2 + Py**2) # distance from robot origin to EE positiion
    Wz = -Pz # This is because the z axis of the EE points down. 
    r_4 = Wx - (L4+L_ENDEFFECTOR)*math.cos(alpha) # distance from base foot frame to motor 4 (or 2, if which_foot_motor = 5)
    s = Wz + (L4 + L_ENDEFFECTOR) * math.sin(alpha) - (L_BASE + L1)  # Distance from joint 2 to the EE. 

    # Joint 2 to the center of the wrist 
    D = math.sqrt(Wx**2 + Wz**2)

    # Check if the position is reachable
    if D > (L2 + L3):
        raise ValueError('ERROR: Position is not reachable')

    # Intermediate angles and sides
    c = math.sqrt(r_4**2 + s**2)
    D = (L2**2 + c**2 - L3**2) / (2 * L2 *  c) #  = cos(beta)
    H = (L2**2 + L3**2 - c**2) / (2 * L2 * L3) # = cos(phi)

    if(D**2 > 1) | (H**2 > 1): # prevents imaginary numbers from happening later, aka prevent out of workspace
        raise ValueError("out of bounds bleh :P")
    

    beta = math.atan2(math.sqrt(1 - D**2), D)
    phi =  math.atan2(math.sqrt(1 - H**2), H) 
    gamma = math.atan2(s, r_4)

    theta3 = round(math.pi/2 - phi, 2) # Theta3 is not dependent on which_foot
    # Depending on which motor (leg) is used, calculate the angles differently
    if which_foot_motor == 1:
        #theta 1
        y = math.sqrt(1 - (Px/Wx)**2) 
        if (Py < 0):
            theta1 = math.atan2(-y, Px/Wx)
        else:
            theta1 = math.atan2(y, Px/Wx) 
    
        theta1 = math.atan2(Py, Px)

        
        # Final joint angles
        theta2 = round(math.pi/2 - (beta + gamma), 2)
        theta4 = round(alpha - theta2 - theta3, 2)

        # Joint 5 doesn't affect the pose 
        theta5 = 0

    elif which_foot_motor == 5:
        #theta 5
        y = math.sqrt(1 - (Px/Wx)**2) 
        if (Py < 0):
            theta5 = math.atan2(-y, Px/Wx)
        else:
            theta5 = math.atan2(y, Px/Wx) 
    
        theta5 = math.atan2(Py, Px)

        
        # Final joint angles
        theta4 = round(math.pi/2 - (beta + gamma), 2)
        theta2 = round(alpha - theta4 - theta3, 2)

        # Joint 5 doesn't affect the pose 
        theta1 = 0
    else:
        raise ValueError('ERROR: please choose either leg 5 or leg 1')

    # Convert radians to degrees
    theta1 = math.degrees(theta1)
    theta2 = math.degrees(-theta2)
    theta3 = math.degrees(theta3)
    theta4 = math.degrees(-theta4)
    theta5 = math.degrees(theta5)

    # theta3 *= -1
    # storing all the theta values in a list 
    theta_values = [theta1, theta2, theta3, theta4, theta5] 
    print("theta_values: ", theta_values)

    # adjusting the theta values with inchworm motor offsets
    theta_values_offsets = apply_offsets(theta_values, inchworm_motor_offsets)
    print("theta_values after motor offsets: ", theta_values_offsets)
    
    # Multiply by -1 to make the math work out 
    # theta_values[1] *= -1

    # return the theta values
    return theta_values
