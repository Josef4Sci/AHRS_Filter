"""
Valenti AHRS Filter Implementation
Ported from MATLAB to Python

Reference:
Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs
August 2015, Sensors 15(8):19302-19330
DOI: 10.3390/s150819302
"""
import numpy as np
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from quaternion_library import quatern_prod, quatern_conj


class ValentiAHRS:
    """
    Valenti AHRS implementation
    """
    
    def __init__(self, quaternion=None, w_acc=0.01, w_mag=0.01):
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0]) if quaternion is None else np.array(quaternion)
        self.w_acc = w_acc
        self.w_mag = w_mag
        self.gain = 0
        
    def update(self, gyroscope, accelerometer, magnetometer, dt):
        """
        Update the filter with MARG sensor data
        
        Args:
            gyroscope: Gyroscope measurement [gx, gy, gz] in rad/s
            accelerometer: Accelerometer measurement [ax, ay, az]
            magnetometer: Magnetometer measurement [mx, my, mz]
            dt: Sample period in seconds
        """
        q = self.quaternion
        
        # Normalise accelerometer measurement
        if np.linalg.norm(accelerometer) == 0:
            return
        acc = accelerometer / np.linalg.norm(accelerometer)
        
        # Normalise magnetometer measurement
        if np.linalg.norm(magnetometer) == 0:
            return
        mag = magnetometer / np.linalg.norm(magnetometer)
        
        # Gyroscope prediction
        q_dot = 0.5 * quatern_prod(q, np.array([0, *gyroscope]))
        quat_gyr_pred = q + q_dot * dt
        q_pred = quat_gyr_pred / np.linalg.norm(quat_gyr_pred)
        
        # Accelerometer correction
        x, y, z = acc
        qr = q_pred
        vx = ((qr[0]**2 + qr[1]**2 - qr[2]**2 - qr[3]**2)*x + 
              2*(qr[1]*qr[2] - qr[0]*qr[3])*y + 
              2*(qr[1]*qr[3] + qr[0]*qr[2])*z)
        vy = (2*(qr[1]*qr[2] + qr[0]*qr[3])*x + 
              (qr[0]**2 - qr[1]**2 + qr[2]**2 - qr[3]**2)*y + 
              2*(qr[2]*qr[3] - qr[0]*qr[1])*z)
        vz = (2*(qr[1]*qr[3] - qr[0]*qr[2])*x + 
              2*(qr[2]*qr[3] + qr[0]*qr[1])*y + 
              (qr[0]**2 - qr[1]**2 - qr[2]**2 + qr[3]**2)*z)
        
        dq0 = np.sqrt((vz + 1) * 0.5)
        dqa = np.array([dq0, vy/(2.0 * dq0), -vx/(2.0 * dq0), 0])
        dqa = self.scale_quaternion(self.w_acc, dqa)
        
        q = quatern_prod(q_pred, dqa)
        
        # Magnetometer correction
        x, y, z = mag
        qr = q
        vx = ((qr[0]**2 + qr[1]**2 - qr[2]**2 - qr[3]**2)*x + 
              2*(qr[1]*qr[2] - qr[0]*qr[3])*y + 
              2*(qr[1]*qr[3] + qr[0]*qr[2])*z)
        vy = (2*(qr[1]*qr[2] + qr[0]*qr[3])*x + 
              (qr[0]**2 - qr[1]**2 + qr[2]**2 - qr[3]**2)*y + 
              2*(qr[2]*qr[3] - qr[0]*qr[1])*z)
        vz = (2*(qr[1]*qr[3] - qr[0]*qr[2])*x + 
              2*(qr[2]*qr[3] + qr[0]*qr[1])*y + 
              (qr[0]**2 - qr[1]**2 - qr[2]**2 + qr[3]**2)*z)
        
        gamma = vx**2 + vy**2
        beta = np.sqrt(gamma + vx*np.sqrt(gamma))
        dqm = np.array([beta / (np.sqrt(2.0 * gamma)), 0, 0, -vy / (np.sqrt(2.0) * beta)])
        
        dqm = self.scale_quaternion(self.w_mag, dqm)
        quat = quatern_prod(q, dqm)
        
        self.quaternion = quat / np.linalg.norm(quat)
        
    def scale_quaternion(self, gain, quat):
        """
        Scale quaternion by gain
        
        Args:
            gain: Scaling factor
            quat: Quaternion to scale
            
        Returns:
            Scaled quaternion
        """
        if quat[0] < 0.0:
            angle = np.arccos(quat[0])
            A = np.sin(angle * (1.0 - gain)) / np.sin(angle)
            B = np.sin(angle * gain) / np.sin(angle)
            quat[0] = A + B * quat[0]
            quat[1:4] = quat[1:4] * B
        else:
            quat[0] = (1.0 - gain) + gain * quat[0]
            quat[1:4] = quat[1:4] * gain
            
        return quat / np.linalg.norm(quat)
