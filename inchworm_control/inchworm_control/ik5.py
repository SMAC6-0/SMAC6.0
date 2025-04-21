import math
import random

def inverseKinematicsMQP(Px, Py, Pz, pivot_foot):
    # These calculations are in inches 
    L_base = 3.125  # base to joint 1
    L1 = 1          # Joint 1 to Joint 2
    L2 = 6.625      # Joint 2 to Joint 3
    L3 = 6.625      # Joint 3 to Joint 4
    L4 = 1          # Joint 4 to Joint 5
    L_endEffector = 3.125  # Joint 5 to EE
    
    if pivot_foot == 1: 


        # adjust Z axis of the end-effector 
        Pz_adjusted = -Pz

        # Joint 1 only for rotation 
        theta1 = math.atan2(Py, Px)

        # Calculate the pose in the XZ-plane
        Wx = math.sqrt(Px**2 + Py**2)
        Wz = Pz_adjusted 

        # Joint 2 to the center of the wrist 
        D = math.sqrt(Wx**2 + Wz**2)

        # Check if the position is reachable
        if D > (L2 + L3):
            raise ValueError('Position is not reachable')

        # Solve joints 2 and 3
        cosTheta2 = (L2**2 + D**2 - L3**2) / (2 * L2 * D)
        theta2 = math.atan2(Wz, Wx) - math.acos(cosTheta2)

        cosTheta3 = (L2**2 + L3**2 - D**2) / (2 * L2 * L3)
        theta3 = math.acos(cosTheta3) - math.pi

        # joint 4
        theta4 = -theta2 + theta3

        # Joint 5 doesn't affect the pose 
        theta5 = 0

        # Convert radians to degrees
        theta1 = math.degrees(theta1)
        theta2 = math.degrees(theta2)
        theta3 = math.degrees(theta3)
        theta4 = math.degrees(theta4)
        theta5 = math.degrees(theta5)

        # adjusted for offset
        # read motor positions and fill our accordingly

        #offset
        theta1 += 62.16
        theta2 += 105.6        
        theta3 += 37.0
        theta4 += 80.0
        theta5 += 204.0

        return theta1, theta2, theta3, theta4, theta5
    
    if pivot_foot == 5: 


        # adjust Z axis of the end-effector 
        Pz_adjusted = -Pz

        # Joint 1 only for rotation 
        theta5 = math.atan2(Py, Px)

        # Calculate the pose in the XZ-plane
        Wx = math.sqrt(Px**2 + Py**2)
        Wz = Pz_adjusted 

        # Joint 2 to the center of the wrist 
        D = math.sqrt(Wx**2 + Wz**2)

        # Check if the position is reachable
        if D > (L2 + L3):
            raise ValueError('Position is not reachable')

        # Solve joints 2 and 3
        cosTheta4 = (L2**2 + D**2 - L3**2) / (2 * L2 * D)
        theta4 = math.atan2(Wz, Wx) - math.acos(cosTheta4)

        cosTheta3 = (L2**2 + L3**2 - D**2) / (2 * L2 * L3)
        theta3 = math.acos(cosTheta3) - math.pi

        # joint 4
        theta2 = -theta4 + theta3

        # Joint 5 doesn't affect the pose 
        theta1 = 0

        # Convert radians to degrees
        theta1 = math.degrees(theta1)
        theta2 = math.degrees(theta2)
        theta3 = math.degrees(theta3)
        theta4 = math.degrees(theta4)
        theta5 = math.degrees(theta5)

        # adjusted for offset
        theta1 += 62.16
        theta2 += 105.6        
        theta3 += 37.0
        theta4 += 80.0
        theta5 += 204.0

        return theta1, theta2, theta3, theta4, theta5
   
    else:
        ValueError('error: please choose either leg 5 or leg 1')