import math
import random
import yaml
import numpy as np
import time
from inchworm_control.ik import inverseKinematics

def quintic_trajectory(self, t0, tf, q0, qf, v0, vf, a0, af): 
    """
    Calculates the position of a single joint using quintic trajectory planning. 
    
    Args: 
        t0 (float64): Start time (usually 0). 
        tf (float64): End time. 
        q0 (tuple): Start position of the end effector (x, z, y). 
        qf (tuple): End position of the end effector (x, z, y).
        v0 (float64): Starting velocity (usually 0). 
        vf (float64): Ending velocity (usually 0). 
        a0 (float64): Starting acceleration (usually 0). 
        af (float64): Ending acceleration (usually 0). 
    Returns: 
        list: Returns a coefficient matrix as a list of float64. 
    """
    bMat = [[q0], [v0], [a0], [qf], [vf], [af]]
    
    coefMat = [[1,      t0,     pow(t0,2),      pow(t0,3),          pow(t0,4),          pow(t0,5)],           # Row vector = coefficients of q0
                [0,     1,      2*t0,           3*pow(t0,2),        4*pow(t0,3),        5*pow(t0,4)],         # Row vector = coefficients of v0
                [0,     0,      2,              6*t0,               2*pow(t0,2),        20*pow(t0,3)],        # Row vector = coefficients of a0
                [1,     tf,     pow(tf,2),      pow(tf,3),          pow(tf,4),          pow(tf,5)],           # Row vector = coefficients of qf
                [0,     1,      2*tf,           3*pow(tf,2),        4*pow(tf,3),        5*pow(tf,4)],         # Row vector = coefficients of af
                [0,     0,      2,              6*tf,               12*pow(tf,2),       20*pow(tf,3)]]        # Row vector = coefficients of vf   
    coefInvMat = np.linalg.inv(coefMat)
    newCoefMat = coefInvMat @ bMat; 
    return newCoefMat


def run_trajectory(self, trajCoeffs, totTime):
    """
    Calculates current joint positions based on trajectory coefficients and current time
    
    Args:
        trajCoeffs (tuple) - trajectory coefficients generated from [4x6 double] quintic_trajectory(), for 5 joints
        totTime (double) - total amount of time it takes for trajectory to reach target position
    """
    timeMat = np.zeros(1,1)
    trajMat = np.zeros(1,5)
    zeroVec = np.zeros(1,5)
    newTrajCoeffs = trajCoeffs    
    time = 0

    # modify trajCoeffs and make it 5x6 matrix. If it's a 5x4
    # matrix, add 2 zeroVec to make them 5x6

    if(len(trajCoeffs[0]) == 5):
        newTrajCoeffs = np.concatenate((newTrajCoeffs , zeroVec, zeroVec), axis=0) # Concatenate vertically 
    
    tic = time.perf_counter()
    while(time < totTime):
        # toc = time.perf_counter()

        # Calculate coeffs accepts 6x5
        theta1 = newTrajCoeffs[0][0] + newTrajCoeffs[1][0]*time + newTrajCoeffs[2][0]*pow(time,2) + newTrajCoeffs[3][0]*pow(time,3) + newTrajCoeffs[4][0]*pow(time,4) + newTrajCoeffs[5][0]*pow(time,5)
        theta2 = newTrajCoeffs[0][1] + newTrajCoeffs[1][1]*time + newTrajCoeffs[2][1]*pow(time,2) + newTrajCoeffs[3][1]*pow(time,3) + newTrajCoeffs[4][1]*pow(time,4) + newTrajCoeffs[5][1]*pow(time,5)
        theta3 = newTrajCoeffs[0][2] + newTrajCoeffs[1][2]*time + newTrajCoeffs[2][2]*pow(time,2) + newTrajCoeffs[3][2]*pow(time,3) + newTrajCoeffs[4][2]*pow(time,4) + newTrajCoeffs[5][2]*pow(time,5)
        theta4 = newTrajCoeffs[0][3] + newTrajCoeffs[1][3]*time + newTrajCoeffs[2][3]*pow(time,2) + newTrajCoeffs[3][3]*pow(time,3) + newTrajCoeffs[4][3]*pow(time,4) + newTrajCoeffs[5][3]*pow(time,5)
        theta5 = newTrajCoeffs[0][4] + newTrajCoeffs[1][4]*time + newTrajCoeffs[2][4]*pow(time,2) + newTrajCoeffs[3][4]*pow(time,3) + newTrajCoeffs[4][4]*pow(time,4) + newTrajCoeffs[5][4]*pow(time,5)
        
        pos = [theta1, theta2, theta3] #  The modified position

        # running the ik3001() function to get the joint angles
        jointAng = self.ik3001(pos, theta3) # the joint angles
        trajMat = np.concatenate((trajMat, [pos, theta3]), axis=0) # Storing the x, y, z position and alpha
        
        self.interpolate_jp(jointAng, 500) # running the interpolate jp to get to the point
        timeMat = np.concatenate((timeMat, time), axis=0) # stores time data
        # tic resets the timing of timeMat, so travel time and the number
        # of loop iterations is considered to keep timing conssitent
        pause(1/10)
        toc = time.perf_counter()
        time = toc - tic
    
    return np.concatenate((timeMat, trajMat), axis=1)

