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
from numba import jit

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils import wahba_constrained
from quaternion_library_jit import fast_cross, quatern_prod, quatern_conj, quaternion_rotate_vector, quatern_prod_single, fast_normalize_3d, fast_normalize_4d, quatern_conj_single, integrate_euler

from scipy.spatial.transform import Rotation, Slerp

class JustaAHRSPure:
    """
    Justa AHRS Pure implementation
    """
    
    def __init__(self, quaternion=None, w_acc=0.00248, w_mag=1.35e-04):
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64) if quaternion is None else np.array(quaternion, dtype=np.float64)
        self.w_acc = np.float64(w_acc)
        self.w_mag = np.float64(w_mag)
        self.gain = np.float64(0.0)
        self.bias_gyro = np.array([0.0, 0.0, 0.0]) * np.pi / 180  # Example bias
        self.bias_gyro = self.bias_gyro.astype(np.float64)
        self.bias_history = []
        self.bias_history.append(self.bias_gyro.copy())

    def initFromAccMag(self, accelerometer, magnetometer):
        self.quaternion = wahba_constrained(np.array([0, 0, 1]), accelerometer, np.array([0, 1, 0]), magnetometer)[0]

    @jit(nopython=True, cache=True, fastmath=True)
    def update_step_optimized(quaternion, gyroscope, accelerometer, magnetometer, 
                            dt, w_acc, w_mag, bias_gyro):
        """
        Heavily optimized update step.
        
        Key optimizations:
        1. Remove redundant astype() calls - inputs should already be float64
        2. Replace np.linalg.norm() with manual calculation (faster for small vectors)
        3. Combine operations to reduce intermediate allocations
        4. Use fastmath=True for aggressive optimizations
        5. Inline small helper functions
        6. Reuse calculations
        """
        
        # ============================================
        # OPTIMIZATION 1: Remove astype if inputs are already float64
        # Pass float64 arrays from outside instead
        # ============================================
        q = quaternion  # Remove .astype() - ensure caller passes float64
        
        # ============================================
        # OPTIMIZATION 2: Fast normalize with early exit
        # ============================================
        acc, valid_acc = fast_normalize_3d(accelerometer)
        if not valid_acc:
            return q, np.zeros(3, dtype=np.float64)
        
        mag, valid_mag = fast_normalize_3d(magnetometer)
        if not valid_mag:
            return q, np.zeros(3, dtype=np.float64)
        
        # ============================================
        # OPTIMIZATION 3: Gyroscope prediction - combine operations
        qp = integrate_euler(q, gyroscope, dt)
        
        # ============================================
        # OPTIMIZATION 4: Predicted accelerometer - use static array
        # ============================================
        # Predefine ar to avoid allocation
        acc_mes_pred = quaternion_rotate_vector(qp, np.array([0.0, 0.0, 1.0], dtype=np.float64))
        
        # ============================================
        # OPTIMIZATION 5: Magnetic reference - combine operations
        # ============================================
        mr_z = acc_mes_pred[0]*mag[0] + acc_mes_pred[1]*mag[1] + acc_mes_pred[2]*mag[2]  # Manual dot product
        mr_x = np.sqrt(1.0 - mr_z*mr_z)
        
        mag_mes_pred = quaternion_rotate_vector(qp, np.array([0.0, mr_x, mr_z], dtype=np.float64))
        
        # ============================================
        # OPTIMIZATION 6: Accelerometer correction - avoid double normalization
        # ============================================
        # Cross product: ca = acc × acc_mes_pred
        ca_x = acc[1]*acc_mes_pred[2] - acc[2]*acc_mes_pred[1]
        ca_y = acc[2]*acc_mes_pred[0] - acc[0]*acc_mes_pred[2]
        ca_z = acc[0]*acc_mes_pred[1] - acc[1]*acc_mes_pred[0]
        
        ca_norm = np.sqrt(ca_x*ca_x + ca_y*ca_y + ca_z*ca_z)
        
        # Avoid division if norm is too small
        if ca_norm > 1e-10:
            inv_ca_norm = 1.0 / ca_norm
            vec_a_x = ca_x * inv_ca_norm
            vec_a_y = ca_y * inv_ca_norm
            vec_a_z = ca_z * inv_ca_norm
        else:
            vec_a_x = vec_a_y = vec_a_z = 0.0
        
        # ============================================
        # OPTIMIZATION 7: Magnetometer correction - same optimization
        # ============================================
        cm_x = mag[1]*mag_mes_pred[2] - mag[2]*mag_mes_pred[1]
        cm_y = mag[2]*mag_mes_pred[0] - mag[0]*mag_mes_pred[2]
        cm_z = mag[0]*mag_mes_pred[1] - mag[1]*mag_mes_pred[0]
        
        cm_norm = np.sqrt(cm_x*cm_x + cm_y*cm_y + cm_z*cm_z)
        
        if cm_norm > 1e-10:
            inv_cm_norm = 1.0 / cm_norm
            vec_b_x = cm_x * inv_cm_norm
            vec_b_y = cm_y * inv_cm_norm
            vec_b_z = cm_z * inv_cm_norm
        else:
            vec_b_x = vec_b_y = vec_b_z = 0.0
        
        # ============================================
        # OPTIMIZATION 8: Combined correction - fuse operations
        # ============================================
        w_acc_half = w_acc * 0.5
        w_mag_half = w_mag * 0.5
        
        im_x = vec_a_x * w_acc_half + vec_b_x * w_mag_half
        im_y = vec_a_y * w_acc_half + vec_b_y * w_mag_half
        im_z = vec_a_z * w_acc_half + vec_b_z * w_mag_half
        
        
        im_norm = np.sqrt(im_x*im_x + im_y*im_y + im_z*im_z)
        
        # sinc optimization: sinc(x) = sin(πx)/(πx)
        # For small x, sinc(x) ≈ 1
        if im_norm < 1e-10:
            sinc_val = 1.0
        else:
            sinc_val = np.sinc(im_norm / np.pi)
        
        im2_x = im_x * sinc_val
        im2_y = im_y * sinc_val
        im2_z = im_z * sinc_val
        
        im2_sum_sq = im2_x*im2_x + im2_y*im2_y + im2_z*im2_z
        q_cor = np.array([np.sqrt(1.0 - im2_sum_sq), im2_x, im2_y, im2_z], dtype=np.float64)
        
        # bias components
        corr_bias =np.array([ca_x + cm_x, ca_y + cm_y, ca_z + cm_z], dtype=np.float64)
        
        # ============================================
        # OPTIMIZATION 9: Final quaternion
        # ============================================
        quat = quatern_prod_single(qp, q_cor)
        
        # Sign check and normalize in one step
        if quat[0] < 0.0:
            quat = -quat
        
        return fast_normalize_4d(quat), corr_bias
    

    def update(self, gyroscope, accelerometer, magnetometer, dt):
        """
        Update the filter with MARG sensor data
        
        Args:
            gyroscope: Gyroscope measurement [gx, gy, gz] in rad/s
            accelerometer: Accelerometer measurement [ax, ay, az]
            magnetometer: Magnetometer measurement [mx, my, mz]
            dt: Sample period in seconds
        """

        self.quaternion, self.corr_bias = JustaAHRSPure.update_step_optimized(
            self.quaternion.astype(np.float64), gyroscope.astype(np.float64), accelerometer.astype(np.float64), magnetometer.astype(np.float64), np.float64(dt),
            self.w_acc, self.w_mag, self.bias_gyro
        )
        
        # interpolate bias correction
        # self.bias_gyro -= self.corr_bias * (self.gain * dt)
        # self.bias_history.append(self.bias_gyro.copy())
    
    
class Particle:
    def __init__(self, w_acc, w_mag):
        self.quaternion = []
        self.w_acc = w_acc
        self.w_mag = w_mag
        self.filter = JustaAHRSInvFast(w_acc=w_acc, w_mag=w_mag)
        
        self.hist_depth = 5
        self.history = []
        self.selected_diff_q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        
    def predict(self, new_gyroscope, new_accelerometer, new_magnetometer, new_dt):
        
        if len(self.history) > 0:
            self.filter.quaternion = quatern_prod_single(self.history[0]['quaternion'],
                                                         quatern_conj_single(self.selected_diff_q))
        
        for i, state in enumerate(self.history):
            self.filter.update(state['gyroscope'], state['accelerometer'], state['magnetometer'], state['dt'])            
            state['quaternion'] = self.filter.quaternion
        
        self.filter.update(new_gyroscope, new_accelerometer, new_magnetometer, new_dt)
        
        state = {'quaternion': self.filter.quaternion,
                    'accelerometer': new_accelerometer,
                    'magnetometer': new_magnetometer,
                    'gyroscope': new_gyroscope,
                    'dt': new_dt}
        
        if len(self.history) > self.hist_depth:
            self.history.pop(0)
            
        self.history.append(state)
        
        self.quaternion = self.filter.quaternion

    def set_quaternion(self, quaternion):
        self.quaternion = quaternion
        self.filter.quaternion = quaternion
        
    def get_last_quaternion(self):
        return self.quaternion
    
    def set_diff_quaternion(self, selected_quaternion):
        last = self.get_last_quaternion()
        inv_last = quatern_conj_single(last)
        self.selected_diff_q = quatern_prod_single(inv_last, selected_quaternion)
        
class JustaAHRSv2:
    
    def interpolate_with_scipy(self, quaternions, weights):
        """
        Use scipy's Slerp for robust interpolation.
        
        Args:
            quaternions: array of shape (N, 4) as [w, x, y, z] (scipy convention)
            weights: array of shape (N,) with weights (should sum to 1)
        """
        # Normalize weights
        weights = np.array(weights)
        weights = weights / np.sum(weights)
        
        # Create time points for each quaternion
        times = np.arange(len(quaternions))
        
        # Create Slerp interpolator
        rotations = Rotation.from_quat(quaternions, scalar_first=True)  # expects [w, x, y, z]
        slerp = Slerp(times, rotations)
        
        # Compute weighted interpolation point
        weighted_time = np.sum(times * weights)
        
        # Interpolate
        result_rotation = slerp(weighted_time)
        
        return result_rotation.as_quat(scalar_first=True)  # Return in [w, x, y, z] format
    

    """
    Justa AHRS v2
    1. get weighted quaternion from last particle history
    2. calculate diff quaternion for each last particle history 
    3. apply diff to first quaternion in history and use as start for prediction
    4. predict each particle and store history
    5. repeat    
    """
    
    def __init__(self, quaternion=None,
                 w_acc=0.00248, w_mag=1.35e-04):
        
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0]) if quaternion is None else np.array(quaternion)
        self.weights_static = np.array([0, 1.0])
        self.particle_filters = [Particle(w_acc=w_acc, w_mag=w_mag), Particle(w_acc=0, w_mag=0)]
        
        self.w_acc = w_acc
        self.w_mag = w_mag
        self.defacc = 0.9791458
        self.coefs = []
        self.bias_gyro = np.zeros(3, dtype=np.float64)
        
        self.interp_coeff = 0.5
        self.a_gyr = 5.0
        self.b_gyr = 2.0
        self.a_acc = 22.0
        self.b_acc = 2.0
        
        self.hist_depth = 5
        
    def initFromAccMag(self, accelerometer, magnetometer):
        self.quaternion = wahba_constrained(np.array([0, 0, 1]), accelerometer, np.array([0, 1, 0]), magnetometer)[0]
        for p in self.particle_filters:
            p.set_quaternion(self.quaternion)

    def update(self, gyroscope, accelerometer, magnetometer, dt):
        """
        Update the filter with MARG sensor data
        
        Args:
            gyroscope: Gyroscope measurement [gx, gy, gz] in rad/s
            accelerometer: Accelerometer measurement [ax, ay, az]
            magnetometer: Magnetometer measurement [mx, my, mz]
            dt: Sample period in seconds
        """
        
        # quats = [p.get_last_quaternion() for p in self.particle_filters]        
        # q_r = self.interpolate_with_scipy(quats, self.weights_static)
        
        for p in self.particle_filters:
            p.set_diff_quaternion(self.quaternion)
            p.predict(gyroscope, accelerometer, magnetometer, dt)
        
        quats = [p.get_last_quaternion() for p in self.particle_filters]
        q_r = self.interpolate_with_scipy(quats, self.weights_static)
        self.quaternion = q_r
        

class JustaAHRSInv:
    """
    Justa AHRS Pure Fast implementation
    """
    
    def __init__(self, quaternion=None, gain=0.0528152, w_acc=0.00248, w_mag=1.35e-04):
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0]) if quaternion is None else np.array(quaternion)
        self.gain = gain
        self.w_acc = w_acc
        self.w_mag = w_mag
        self.test = []
    
    def initFromAccMag(self, accelerometer, magnetometer):
        self.quaternion = wahba_constrained(np.array([0, 0, 1]), accelerometer, np.array([0, 1, 0]), magnetometer)[0]
        

    def update(self, gyroscope, accelerometer, magnetometer, dt):
        """
        Update the filter with MARG sensor data
        
        Args:
            gyroscope: Gyroscope measurement [gx, gy, gz] in rad/s
            accelerometer: Accelerometer measurement [ax, ay, az]
            magnetometer: Magnetometer measurement [mx, my, mz]
            dt: Sample period in seconds
        """        
        # Normalise accelerometer measurement
        if np.linalg.norm(accelerometer) == 0:
            return
        acc = accelerometer / np.linalg.norm(accelerometer)
        
        # Normalise magnetometer measurement
        if np.linalg.norm(magnetometer) == 0:
            return
        mag = magnetometer / np.linalg.norm(magnetometer)

        qp = integrate_euler(self.quaternion, gyroscope, dt)
        
        # Predicted accelerometer
        ar = np.array([0, 0, 1])
        inv_pred = quatern_conj_single(qp)
        acc_mes_pred = quaternion_rotate_vector(inv_pred, acc)


        mag_mes_pred = quaternion_rotate_vector(inv_pred, mag)
        
        mr_ref = np.array([0, np.linalg.norm([mag_mes_pred [0], mag_mes_pred [1]]), mag_mes_pred[2]])
        
        # Accelerometer correction
        ca = np.cross(acc_mes_pred, ar)
        na = np.linalg.norm(ca)
        veca = ca / na
        
        phia = self.w_acc
            
        # Magnetometer correction
        cm = np.cross(mag_mes_pred, mr_ref)
        n = np.linalg.norm(cm)
        vecm = cm / n

        vecm[0]=0
        vecm[1]=0
        
        phim = self.w_mag

        # Correction quaternion
        q_cor = np.array([1, *(veca * phia + vecm * phim)])
        
        quat = quatern_prod_single(q_cor, qp)
        
        if quat[0] < 0:
            quat = -quat
            
        self.quaternion = quat / np.linalg.norm(quat)
        self.test.append(n)

class JustaAHRSInvFast:
    """
    Justa AHRS Pure Fast implementation
    """
    
    def __init__(self, quaternion=None, gain=0.0528152, w_acc=0.00248, w_mag=1.35e-04):
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0]) if quaternion is None else np.array(quaternion)
        self.w_acc = w_acc
        self.w_mag = w_mag
        self.test = []
        self.acc_ref = np.array([0, 0, 1])
    
    def initFromAccMag(self, accelerometer, magnetometer):
        """
        Initialize the filter from accelerometer and magnetometer measurements.
        
        Args:
            accelerometer: Accelerometer measurement [ax, ay, az]
            magnetometer: Magnetometer measurement [mx, my, mz]
        """
        self.quaternion = wahba_constrained(np.array([0, 0, 1]), accelerometer, np.array([0, 1, 0]), magnetometer)[0]      


    def update(self, gyroscope, accelerometer, magnetometer, dt):
        """
        Update the filter with MARG sensor data
        
        Args:
            gyroscope: Gyroscope measurement [gx, gy, gz] in rad/s
            accelerometer: Accelerometer measurement [ax, ay, az]
            magnetometer: Magnetometer measurement [mx, my, mz]
            dt: Sample period in seconds
        """        
        acc, valid_a = fast_normalize_3d(accelerometer)    
        if not valid_a:
            return    
        mag, valid_m = fast_normalize_3d(magnetometer)
        if not valid_m:
            return

        qp = integrate_euler(self.quaternion, gyroscope, dt)

        inv_pr = quatern_conj_single(qp)
        acc_mes_pred = quaternion_rotate_vector(inv_pr, acc)
        
        #x part rotation from quat_inv
        rot_x = np.array([  2*(0.5 - qp[2]**2 - qp[3]**2),
                            2*(qp[1]*qp[2] - qp[0]*qp[3]),
                            2*(qp[0]*qp[2] + qp[1]*qp[3]) ])
        
        mag_pr_x = np.dot(rot_x, mag)

        # Accelerometer correction
        ca = fast_cross(acc_mes_pred, self.acc_ref)
        na = np.linalg.norm(ca)
        veca = ca / na
        veca *= self.w_acc
        
        #magnetic correction [0 1 0] reference -> mag_mes_pred[0] < 0
        veca[2] += -self.w_mag if mag_pr_x < 0 else self.w_mag
        
        # Correction quaternion
        q_cor = np.array([1, *(veca)])
        
        quat = quatern_prod_single(q_cor, qp)
        
        if quat[0] < 0:
            quat = -quat
            
        self.quaternion = fast_normalize_4d(quat)

