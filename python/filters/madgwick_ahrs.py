"""
Madgwick AHRS Filter Implementation
Ported from MATLAB to Python
"""
import numpy as np
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from quaternion_library import quatern_prod, quatern_conj


class MadgwickAHRS:
    """
    MADGWICKAHRS Implementation of Madgwick's IMU and AHRS algorithms
    
    For more information see:
    http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
    """
    
    def __init__(self, sample_period=1/256, quaternion=None, beta=0.1):
        self.sample_period = sample_period
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0]) if quaternion is None else np.array(quaternion)
        self.beta = beta
        
    def update(self, gyroscope, accelerometer, magnetometer):
        """
        Update the filter with MARG sensor data
        
        Args:
            gyroscope: Gyroscope measurement [gx, gy, gz] in rad/s
            accelerometer: Accelerometer measurement [ax, ay, az]
            magnetometer: Magnetometer measurement [mx, my, mz]
        """
        q = self.quaternion
        
        # Normalise accelerometer measurement
        if np.linalg.norm(accelerometer) == 0:
            return
        accelerometer = accelerometer / np.linalg.norm(accelerometer)
        
        # Normalise magnetometer measurement
        if np.linalg.norm(magnetometer) == 0:
            return
        magnetometer = magnetometer / np.linalg.norm(magnetometer)
        
        # Reference direction of Earth's magnetic field
        h = quatern_prod(q, quatern_prod(np.array([0, *magnetometer]), quatern_conj(q)))
        b = np.array([0, np.linalg.norm([h[1], h[2]]), 0, h[3]])
        
        # Gradient descent algorithm corrective step
        F = np.array([
            2*(q[1]*q[3] - q[0]*q[2]) - accelerometer[0],
            2*(q[0]*q[1] + q[2]*q[3]) - accelerometer[1],
            2*(0.5 - q[1]**2 - q[2]**2) - accelerometer[2],
            2*b[1]*(0.5 - q[2]**2 - q[3]**2) + 2*b[3]*(q[1]*q[3] - q[0]*q[2]) - magnetometer[0],
            2*b[1]*(q[1]*q[2] - q[0]*q[3]) + 2*b[3]*(q[0]*q[1] + q[2]*q[3]) - magnetometer[1],
            2*b[1]*(q[0]*q[2] + q[1]*q[3]) + 2*b[3]*(0.5 - q[1]**2 - q[2]**2) - magnetometer[2]
        ])
        
        J = np.array([
            [-2*q[2], 2*q[3], -2*q[0], 2*q[1]],
            [2*q[1], 2*q[0], 2*q[3], 2*q[2]],
            [0, -4*q[1], -4*q[2], 0],
            [-2*b[3]*q[2], 2*b[3]*q[3], -4*b[1]*q[2]-2*b[3]*q[0], -4*b[1]*q[3]+2*b[3]*q[1]],
            [-2*b[1]*q[3]+2*b[3]*q[1], 2*b[1]*q[2]+2*b[3]*q[0], 2*b[1]*q[1]+2*b[3]*q[3], -2*b[1]*q[0]+2*b[3]*q[2]],
            [2*b[1]*q[2], 2*b[1]*q[3]-4*b[3]*q[1], 2*b[1]*q[0]-4*b[3]*q[2], 2*b[1]*q[1]]
        ])
        
        step = J.T @ F
        step = step / np.linalg.norm(step)
        
        # Compute rate of change of quaternion
        q_dot = 0.5 * quatern_prod(q, np.array([0, *gyroscope])) - self.beta * step
        
        # Integrate to yield quaternion
        q = q + q_dot * self.sample_period
        self.quaternion = q / np.linalg.norm(q)
        
    def update_imu(self, gyroscope, accelerometer):
        """
        Update the filter with IMU sensor data (no magnetometer)
        
        Args:
            gyroscope: Gyroscope measurement [gx, gy, gz] in rad/s
            accelerometer: Accelerometer measurement [ax, ay, az]
        """
        q = self.quaternion
        
        # Normalise accelerometer measurement
        if np.linalg.norm(accelerometer) == 0:
            return
        accelerometer = accelerometer / np.linalg.norm(accelerometer)
        
        # Gradient descent algorithm corrective step
        F = np.array([
            2*(q[1]*q[3] - q[0]*q[2]) - accelerometer[0],
            2*(q[0]*q[1] + q[2]*q[3]) - accelerometer[1],
            2*(0.5 - q[1]**2 - q[2]**2) - accelerometer[2]
        ])
        
        J = np.array([
            [-2*q[2], 2*q[3], -2*q[0], 2*q[1]],
            [2*q[1], 2*q[0], 2*q[3], 2*q[2]],
            [0, -4*q[1], -4*q[2], 0]
        ])
        
        step = J.T @ F
        step = step / np.linalg.norm(step)
        
        # Compute rate of change of quaternion
        q_dot = 0.5 * quatern_prod(q, np.array([0, *gyroscope])) - self.beta * step
        
        # Integrate to yield quaternion
        q = q + q_dot * self.sample_period
        self.quaternion = q / np.linalg.norm(q)
