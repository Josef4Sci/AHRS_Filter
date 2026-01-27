"""
Justa AHRS Filter Implementation
Ported from MATLAB to Python

Reference:
JUSTA, Josef; ŠMÍDL, Václav; HAMÁČEK, Aleš. Fast AHRS Filter for Accelerometer, 
Magnetometer, and Gyroscope Combination with Separated Sensor Corrections. 
Sensors, 2020, 20.14: 3824.
"""
import numpy as np
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from quaternion_library import quatern_prod, quatern_conj


class JustaAHRSPure:
    """
    Justa AHRS Pure implementation
    """
    
    def __init__(self, sample_period=1/256, quaternion=None, beta=1, w_acc=0.00248, w_mag=1.35e-04):
        self.sample_period = sample_period
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0]) if quaternion is None else np.array(quaternion)
        self.beta = beta
        self.w_acc = w_acc
        self.w_mag = w_mag
        self.gain = 0.0528152
        
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
        acc = accelerometer / np.linalg.norm(accelerometer)
        
        # Normalise magnetometer measurement
        if np.linalg.norm(magnetometer) == 0:
            return
        mag = magnetometer / np.linalg.norm(magnetometer)
        
        # Gyroscope prediction
        wt = gyroscope * self.sample_period / 2
        q_dot_x = np.sin(wt[0])
        q_dot_y = np.sin(wt[1])
        q_dot_z = np.sin(wt[2])
        q_dot_w = np.sqrt(1 - (q_dot_x**2 + q_dot_y**2 + q_dot_z**2))
        
        q_dot = np.array([q_dot_w, q_dot_x, q_dot_y, q_dot_z])
        quat_gyr_pred = quatern_prod(q, q_dot)
        
        # Rotation matrix
        qp = quat_gyr_pred
        R = np.array([
            [2*(0.5 - qp[2]**2 - qp[3]**2), 0, 2*(qp[1]*qp[3] - qp[0]*qp[2])],
            [2*(qp[1]*qp[2] - qp[0]*qp[3]), 0, 2*(qp[0]*qp[1] + qp[2]*qp[3])],
            [2*(qp[0]*qp[2] + qp[1]*qp[3]), 0, 2*(0.5 - qp[1]**2 - qp[2]**2)]
        ])
        
        # Predicted accelerometer
        ar = np.array([0, 0, 1])
        acc_mes_pred = R @ ar
        
        # Magnetic reference
        mr_z = np.dot(acc_mes_pred, mag)
        mr_x = np.sqrt(1 - mr_z**2)
        mr = np.array([mr_x, 0, mr_z])
        
        mag_mes_pred = R @ mr
        
        # Accelerometer correction
        ca = np.cross(acc, acc_mes_pred)
        vec_a = ca / np.linalg.norm(ca)
        
        # Magnetometer correction
        cm = np.cross(mag, mag_mes_pred)
        vec_b = cm / np.linalg.norm(cm)
        
        # Combined correction
        im = vec_a * self.w_acc / 2 + vec_b * self.w_mag / 2
        im2 = im * np.sinc(np.linalg.norm(im) / np.pi)
        q_cor = np.array([np.sqrt(1 - np.sum(im2**2)), *im2])
        
        # Final quaternion
        quat = quatern_prod(quat_gyr_pred, q_cor)
        
        if quat[0] < 0:
            quat = -quat
            
        self.quaternion = quat / np.linalg.norm(quat)


class JustaAHRSPureFast:
    """
    Justa AHRS Pure Fast implementation
    """
    
    def __init__(self, sample_period=1/256, quaternion=None, beta=1, gain=0.0528152, 
                 w_acc=0.00248, w_mag=1.35e-04):
        self.sample_period = sample_period
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0]) if quaternion is None else np.array(quaternion)
        self.beta = beta
        self.gain = gain
        self.w_acc = w_acc
        self.w_mag = w_mag
        self.mr_z = 0.895
        
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
        acc = accelerometer / np.linalg.norm(accelerometer)
        
        # Normalise magnetometer measurement
        if np.linalg.norm(magnetometer) == 0:
            return
        mag = magnetometer / np.linalg.norm(magnetometer)
        
        # Gyroscope integration
        q_dot = 0.5 * self.sample_period * quatern_prod(q, np.array([0, *gyroscope]))
        qp = q + q_dot
        qp = qp / np.linalg.norm(qp)
        
        # Rotation matrix
        R = np.array([
            [2*(0.5 - qp[2]**2 - qp[3]**2), 0, 2*(qp[1]*qp[3] - qp[0]*qp[2])],
            [2*(qp[1]*qp[2] - qp[0]*qp[3]), 0, 2*(qp[0]*qp[1] + qp[2]*qp[3])],
            [2*(qp[0]*qp[2] + qp[1]*qp[3]), 0, 2*(0.5 - qp[1]**2 - qp[2]**2)]
        ])
        
        # Predicted accelerometer
        ar = np.array([0, 0, 1])
        acc_mes_pred = R @ ar
        
        # Magnetic reference
        h = quatern_prod(q, quatern_prod(np.array([0, *mag]), quatern_conj(q)))
        mr = np.array([np.linalg.norm([h[1], h[2]]), 0, h[3]])
        mr = mr / np.linalg.norm(mr)
        mag_mes_pred = R @ mr
        
        # Accelerometer correction
        ca = np.cross(acc, acc_mes_pred)
        na = np.linalg.norm(ca)
        veca = ca / na
        
        phia = np.arcsin(na) * self.gain
        if phia > self.w_acc:
            phia = self.w_acc
            
        # Magnetometer correction
        cm = np.cross(mag, mag_mes_pred)
        n = np.linalg.norm(cm)
        vecm = cm / n
        
        phim = np.arcsin(n) * self.gain
        if phim > self.w_mag:
            phim = self.w_mag
            
        # Correction quaternion
        q_cor = np.array([1, *(veca * phia / 2 + vecm * phim / 2)])
        
        quat = quatern_prod(qp, q_cor)
        
        if quat[0] < 0:
            quat = -quat
            
        self.quaternion = quat / np.linalg.norm(quat)
