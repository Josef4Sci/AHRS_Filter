"""
Jin Wu Kalman Filter AHRS Implementation
Ported from MATLAB to Python

Reference:
Guo, S.; Wu, J.; Wang, Z.; Qian, J. Novel MARG-Sensor Orientation Estimation Algorithm Using
Fast Kalman Filter. Journal of Sensors 2017, Article ID 8542153

Implementation based on: https://github.com/zarathustr/FKF
"""
import numpy as np
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from quaternion_library import quatern_prod, quatern_conj, measurement_quaternion_acc_mag, kalman_update


class JinWuKFAHRS:
    """
    Jin Wu Kalman Filter AHRS implementation
    """
    
    def __init__(self, sample_period=1/256, quaternion=None, sigma_g=1, sigma_a=3, sigma_m=500):
        self.sample_period = sample_period
        self.quaternion = np.array([0.5, -0.5, -0.5, -0.5]) if quaternion is None else np.array(quaternion)
        
        self.sigma_g = sigma_g * np.eye(3)
        self.sigma_a = sigma_a * np.eye(3)
        self.sigma_m = sigma_m * np.eye(3)
        
        self.Pk = 0.001 * np.eye(4)
        
    def update(self, gyroscope, accelerometer, magnetometer):
        """
        Update the filter with MARG sensor data using Kalman filter
        
        Args:
            gyroscope: Gyroscope measurement [gx, gy, gz] in rad/s
            accelerometer: Accelerometer measurement [ax, ay, az]
            magnetometer: Magnetometer measurement [mx, my, mz]
        """
        dt = self.sample_period
        qq = self.quaternion
        q0, q1, q2, q3 = qq
        
        wx, wy, wz = gyroscope
        
        # Normalise measurements
        accelerometer = accelerometer / np.linalg.norm(accelerometer)
        magnetometer = magnetometer / np.linalg.norm(magnetometer)
        
        # Magnetic field reference
        mD = np.dot(accelerometer, magnetometer)
        mN = np.sqrt(1 - mD**2)
        
        # Omega matrix
        omega4 = np.array([
            [0, -wx, -wy, -wz],
            [wx, 0, wz, -wy],
            [wy, -wz, 0, wx],
            [wz, wy, -wx, 0]
        ])
        
        # State transition matrix
        Phi = np.eye(4) + dt / 2 * omega4
        
        # Process noise
        Dk = np.array([
            [q1, q2, q3],
            [-q0, -q3, q2],
            [q3, -q0, -q1],
            [-q2, q1, -q0]
        ])
        Xi = dt**2 / 4 * Dk @ self.sigma_g @ Dk.T
        
        # Measurement quaternion
        qy, Jacob = measurement_quaternion_acc_mag(accelerometer, magnetometer, 
                                                    np.array([mN, 0, mD]), qq)
        qy = qy / np.linalg.norm(qy)
        
        # Measurement noise
        sigma_combined = np.block([
            [self.sigma_a, np.zeros((3, 3))],
            [np.zeros((3, 3)), self.sigma_m]
        ])
        Eps = Jacob @ sigma_combined @ Jacob.T
        
        # Kalman update
        q_ = qq
        Pk_ = self.Pk
        qq, self.Pk = kalman_update(q_, qy, Pk_, Phi, Xi, Eps)
        
        qq = qq / np.linalg.norm(qq)
        self.quaternion = qq / np.linalg.norm(qq)
