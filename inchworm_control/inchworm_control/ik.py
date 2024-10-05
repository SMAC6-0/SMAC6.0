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
    J2_J4_XDIST = 2.95 # Distance between Joints 2 and 4 when the inchworm is in the home configuration (when the legs are next to each other)

    # extract the motor offsets from the yaml file
    inchworm_motor_offsets = load_inchworm_motor_offsets('/home/smac/robot_ws/src/SMAC6.0/inchworm_control/inchworm_control/inchworm_motor_config.yaml')

    # print("motor offsets: ", inchworm_motor_offsets)

    # Convert alpha angle from radians to degrees 
    alpha = math.radians(alpha)

    # error catching: Robot cannot reach the point
    maxDistArmZ = L_BASE + L1 + L2 + L3 + L4 + L_ENDEFFECTOR # max Distance the arm can reach in z axis
    maxDistArmXY = L2 + L3 + L4 + L_ENDEFFECTOR # max Distance the arm can reach in x or y axis

    print("right before max dist check")
    # throws error if coordinate is too far for the robot to reach
    if(Px > maxDistArmXY or Py > maxDistArmXY or Pz > maxDistArmZ): 
        ValueError("NOOOO MY LEGS ARE TOO SHORT :(")

    
    # See inverse kinematics derivation in the SMAC 6.0 Google Drive folder 
    # Calculate the pose in the XZ-plane 
    r = math.sqrt(Px**2 + Py**2) # distance along the ground (z=0) from robot origin to EE positiion 
    # Wz = -Pz # This is because the z axis of the EE points down. NOt applicable to the math done by SMAC6 
    r_4 = r - (L4+L_ENDEFFECTOR)*math.cos(alpha) # distance from base foot frame (robot origin) to motor 4 (or 2, if which_foot_motor = 5) along r 
    s = Pz + (L4 + L_ENDEFFECTOR) * math.sin(alpha) - (L_BASE + L1)  # Vertical (z) distance from joint 2 to joint 4. 

    # # Joint 2 to the center of the wrist 
    # D = math.sqrt(r**2 + Pz**2)

    # # Check if the position is reachable
    # if D > (L2 + L3):
    #     raise ValueError('ERROR: Position is not reachable')

    # Intermediate angles and sides
    print("right before cos(angle) calculations")
    c = math.sqrt(r_4**2 + s**2) # 3D distance between Joints 2 & 4 
    cos_beta = (L2**2 + c**2 - L3**2) / (2 * L2 *  c) #  = cos(beta)
    cos_phi = (L2**2 + L3**2 - c**2) / (2 * L2 * L3) # = cos(phi) where phi is the angle at Joint 3, between L2 & L3 
    print(" ruight before out of bo")

    if(cos_beta**2 > 1) or (cos_phi**2 > 1): # prevents imaginary numbers from happening later, aka prevent out of workspace
        raise ValueError("out of bounds bleh :P")
    

    beta = math.atan2(math.sqrt(1 - cos_beta**2), cos_beta) # the angle at joint 2 between L2 & c 
    phi =  math.atan2(math.sqrt(1 - cos_phi**2), cos_phi) # the angle at joint 3, between L2 & L3
    gamma = math.atan2(s, r_4) # at joint 2, the angle between c & the "r axis"


    # angle offsets for theta2, theta3 and theta4
    sigma = math.acos((L2**2 + J2_J4_XDIST**2 - L3**2) / (2 * L2 *  J2_J4_XDIST))
    theta2_4_offset = math.pi/2 - sigma
    

    theta3 = round(math.pi - phi, 2) # Theta3 is not dependent on which_foot
    # Depending on which motor (leg) is used, calculate the angles differently
    if which_foot_motor == 1:
        #theta 1
        y = math.sqrt(1 - (Px/r)**2) 
        if (Py < 0):
            theta1 = math.atan2(-y, Px/r)
        else:
            theta1 = math.atan2(y, Px/r) 
        
        # Final joint angles
        theta2 = round(math.pi/2 - (beta + gamma), 2)
        theta4 = round(alpha - theta2 - theta3, 2)

        # Joint 5 doesn't affect the pose 
        theta5 = 0

    elif which_foot_motor == 5:
        #theta 5
        y = math.sqrt(1 - (Px/r)**2) 
        if (Py < 0):
            theta5 = math.atan2(-y, Px/r)
        else:
            theta5 = math.atan2(y, Px/r) 
    

        
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

    print("Calculating Offsets")

    # theta3 *= -1
    # storing all the theta values in a list 
    theta_values = [theta1, theta2, theta3, theta4, theta5] 
    print("theta_values: ", theta_values)

    # adjusting the theta values with inchworm motor offsets
    theta_values = apply_offsets(theta_values, inchworm_motor_offsets)
    print("theta_values after motor offsets: ", theta_values)
    
    # Multiply by -1 to make the math work out 
    # theta_values[1] *= -1

    # return the theta values
    return theta_values
