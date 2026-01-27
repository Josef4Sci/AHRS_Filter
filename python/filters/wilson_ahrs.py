"""
Wilson-Madgwick AHRS Filter Implementation
Ported from MATLAB to Python

Reference:
Formulation of a new gradient descent MARG orientation algorithm:
Case study on robot teleoperation
"""
import numpy as np
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from quaternion_library import quatern_prod, quatern_conj


class WilsonMadgwickAHRS:
    """
    Wilson-Madgwick AHRS implementation
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
        
        # Fused measurement
        measure_fused = np.cross(accelerometer, magnetometer)
        measure_fused = measure_fused / np.linalg.norm(measure_fused)
        
        error = np.array([
            -2*(q[0]*q[3] + q[1]*q[2]) - measure_fused[0],
            -q[0]**2 + q[1]**2 - q[2]**2 + q[3]**2 - measure_fused[1],
            -2*(q[2]*q[3] - q[0]*q[1]) - measure_fused[2]
        ])
        
        J = -2 * np.array([
            [-q[3], -q[0], q[1]],
            [-q[2], q[1], q[0]],
            [-q[1], -q[2], -q[3]],
            [-q[0], q[3], -q[2]]
        ])
        
        F = J @ error
        
        # Compute rate of change of quaternion
        q_dot = 0.5 * quatern_prod(q, np.array([0, *gyroscope])) - self.beta * F
        
        # Integrate to yield quaternion
        q = q + q_dot * self.sample_period
        self.quaternion = q / np.linalg.norm(q)


class AdmirallWilsonAHRS:
    """
    Admirall-Wilson AHRS implementation
    
    Reference:
    Improved Formulation of the IMU and MARG Orientation Gradient 
    Descent Algorithm for Motion Tracking in Human-Machine Interfaces
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
        vrm = np.array([np.linalg.norm([h[1], h[2]]), 0, h[3]])
        
        # Accelerometer error
        error_a = np.array([
            2*(q[1]*q[3] - q[0]*q[2]) - accelerometer[0],
            2*(q[0]*q[1] + q[2]*q[3]) - accelerometer[1],
            q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2 - accelerometer[2]
        ])
        
        Ja = -2 * np.array([
            [q[2], -q[1], -q[0]],
            [-q[3], -q[0], q[1]],
            [q[0], -q[3], q[2]],
            [-q[1], -q[2], -q[3]]
        ])
        
        Fa = Ja @ error_a
        
        # Magnetometer error
        error_m = np.array([
            vrm[0]*(q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2) + vrm[2]*(-2*q[0]*q[2] + 2*q[1]*q[3]) - magnetometer[0],
            vrm[0]*(-2*q[0]*q[3] + 2*q[1]*q[2]) + vrm[2]*(2*q[0]*q[1] + 2*q[2]*q[3]) - magnetometer[1],
            vrm[0]*(2*q[0]*q[2] + 2*q[1]*q[3]) + vrm[2]*(q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2) - magnetometer[2]
        ])
        
        Jm = 2 * np.array([
            [vrm[0]*q[0] - vrm[2]*q[2], -vrm[0]*q[3] + vrm[2]*q[1], vrm[0]*q[2] + vrm[2]*q[0]],
            [vrm[0]*q[1] + vrm[2]*q[3], vrm[0]*q[2] + vrm[2]*q[0], vrm[0]*q[3] - vrm[2]*q[1]],
            [-vrm[0]*q[2] - vrm[2]*q[0], vrm[0]*q[1] + vrm[2]*q[3], vrm[0]*q[0] - vrm[2]*q[2]],
            [-vrm[0]*q[3] + vrm[2]*q[1], -vrm[0]*q[0] + vrm[2]*q[2], vrm[0]*q[1] + vrm[2]*q[3]]
        ])
        
        Fm = Jm @ error_m
        
        # Compute rate of change of quaternion
        q_dot = (0.5 * quatern_prod(q, np.array([0, *gyroscope])) - 
                 self.beta * Fa - self.beta * Fm)
        
        # Integrate to yield quaternion
        q = q + q_dot * self.sample_period
        self.quaternion = q / np.linalg.norm(q)
