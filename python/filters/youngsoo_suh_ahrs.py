"""
YoungSoo Suh AHRS Filter Implementation
Ported from MATLAB to Python

Reference:
Suh, Y.S. Simple-Structured Quaternion Estimator Separating Inertial and Magnetic Sensor Effects. 
IEEE, Transactions on Aerospace and Electronic Systems 2019, 55, 2698–2706.
"""
import numpy as np
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from quaternion_library import quatern_prod, quatern_conj


class YoungSooSuhAHRS:
    """
    YoungSoo Suh AHRS implementation
    """
    
    def __init__(self, sample_period=1/256, quaternion=None, rg=0.317, ra=0.0004156, rm=0.00057):
        self.sample_period = sample_period
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0]) if quaternion is None else np.array(quaternion)
        
        self.rg = rg
        self.ra = ra
        self.rm = rm
        
        self.q_err = np.array([0.0, 0.0, 0.0])
        
        self.alf = 0
        self.bet = 0
        self.gam = 0
        
        self.k_alf = 0
        self.k_bet = 0
        
    def update(self, gyroscope, accelerometer, magnetometer):
        """
        Update the filter with MARG sensor data
        
        Args:
            gyroscope: Gyroscope measurement [gx, gy, gz] in rad/s
            accelerometer: Accelerometer measurement [ax, ay, az]
            magnetometer: Magnetometer measurement [mx, my, mz]
        """
        g = 1
        dt = self.sample_period
        q = self.quaternion
        
        # Gyroscope prediction
        q_dif_est = 0.5 * quatern_prod(q, np.array([0, *gyroscope]))
        q_est = q + q_dif_est * dt
        
        # Normalise measurements
        acc_norm = accelerometer / np.linalg.norm(accelerometer)
        magnetometer = magnetometer / np.linalg.norm(magnetometer)
        
        # Dip angle (assuming -50 degrees)
        cos_acc_mag = np.sin(np.radians(-50))
        sin_acc_mag = np.sqrt(1 - cos_acc_mag**2)
        sin_dip_ang = cos_acc_mag
        cos_dip_ang = sin_acc_mag
        
        # Kalman gain computation
        ak_minus = self.alf + self.rg * dt**2 / 4
        bk_minus = self.bet + self.rm * dt**2 / 4
        
        self.k_alf = 2 * ak_minus * g / (4 * ak_minus * g**2 + self.ra)
        self.k_bet = -2 * (bk_minus * cos_dip_ang - 4 * self.gam * sin_dip_ang) / (
            4 * ak_minus * sin_dip_ang**2 + 4 * bk_minus * cos_dip_ang**2 - 
            8 * self.gam * cos_dip_ang * sin_dip_ang + self.rm
        )
        
        self.alf = ((4 * ak_minus * g**2 + self.ra) * self.k_alf**2 - 
                    4 * ak_minus * g * self.k_alf + ak_minus)
        self.bet = ((4 * ak_minus * sin_dip_ang**2 + 4 * bk_minus * cos_dip_ang**2 + 
                     self.rm - 8 * self.gam * cos_dip_ang * sin_dip_ang) * self.k_bet**2 + 
                    4 * (bk_minus * cos_dip_ang - 4 * self.gam * sin_dip_ang) * self.k_bet + 
                    bk_minus)
        self.gam = (-(2 * g * self.k_alf - 1) * 
                    (self.gam + 2 * (self.gam * cos_dip_ang - ak_minus * sin_dip_ang) * self.k_bet))
        
        # Accelerometer measurement
        a_rot = quatern_prod(q_est, quatern_prod(np.array([0, *acc_norm]), quatern_conj(q_est)))
        z1 = (a_rot[1:3] - np.array([0, g]))
        
        # Magnetometer measurement
        m_rot = quatern_prod(q_est, quatern_prod(np.array([0, *magnetometer]), quatern_conj(q_est)))
        z2 = m_rot[3]
        
        # Kalman update
        K = np.array([
            [0, self.k_alf, 0],
            [-self.k_alf, 0, 0],
            [0, 0, self.k_bet]
        ])
        
        H = np.array([
            [0, -1, 0],
            [1, 0, 0],
            [sin_dip_ang, 0, -cos_dip_ang]
        ]) * 2
        
        z = np.array([z1[0], z1[1], z2])
        self.q_err = self.q_err + (K @ (z - H @ self.q_err))
        
        # Normalize and update quaternion
        norm_q = np.array([1, *self.q_err])
        norm_q = norm_q / np.linalg.norm(norm_q)
        q = quatern_prod(norm_q, q_est)
        
        self.quaternion = q / np.linalg.norm(q)
