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
        # ============================================
        wt = (gyroscope - bias_gyro) * (dt * 0.5)  # Multiply once
        
        q_dot_x = np.sin(wt[0])
        q_dot_y = np.sin(wt[1])
        q_dot_z = np.sin(wt[2])
        
        # Optimization: reuse sum calculation
        sin_sum_sq = q_dot_x*q_dot_x + q_dot_y*q_dot_y + q_dot_z*q_dot_z
        q_dot_w = np.sqrt(1.0 - sin_sum_sq)
        
        # Direct array creation without intermediate variable
        qp = quatern_prod_single(q, np.array([q_dot_w, q_dot_x, q_dot_y, q_dot_z], dtype=np.float64))
        
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

def quaternion_derivative(q, gyro):
    """
    Compute quaternion derivative from angular velocity.
    
    q_dot = 0.5 * q ⊗ [0, wx, wy, wz]
    
    Parameters:
    -----------
    q : np.ndarray, shape (4,)
        Quaternion [w, x, y, z]
    gyro : np.ndarray, shape (3,)
        Angular velocity [wx, wy, wz] in rad/s
    
    Returns:
    --------
    q_dot : np.ndarray, shape (4,)
        Quaternion derivative
    """
    w, x, y, z = q
    wx, wy, wz = gyro
    
    # Quaternion derivative matrix form
    q_dot = 0.5 * np.array([
        -x*wx - y*wy - z*wz,
         w*wx + y*wz - z*wy,
         w*wy - x*wz + z*wx,
         w*wz + x*wy - y*wx
    ])
    
    return q_dot

    

class JustaAHRSv2:
    """
    Justa AHRS Pure Fast implementation
    """
    
    def __init__(self, quaternion=None, gain=12.00, 
                 w_acc=0.00248, w_mag=1.35e-04, s1=1.0, s2=1.0, s3=1.0, bias_gyro=np.array([0.336, -0.08, 0.0])):
        
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0]) if quaternion is None else np.array(quaternion)
        self.gain = gain
        self.w_acc = w_acc
        self.w_mag = w_mag
        self.defacc = 0.9791458
        self.coefs = []
        self.bias_gyro = bias_gyro * np.pi / 180  # Example bias
        self.scale_factors = np.array([s1, s2, s3])
        
        self.interp_coeff = 0.5
        self.a_gyr = 5.0
        self.b_gyr = 2.0
        self.a_acc = 22.0
        self.b_acc = 2.0
        
        self.particle_history = []
        self.hist_depth = 10
        
    def predict_particle(self):
                                      
        for i, state in enumerate(self.particle_history):
            
            if state['particle_result'] or i == 0:
                continue
            
            state['quaternion'] = self.update_step(self.particle_history[i-1]['quaternion'],
                                                                        state['gyroscope'], 
                                                state['accelerometer'], state['magnetometer'], 
                                                state['dt'], self.bias_gyro, self.defacc, 
                                                self.a_acc, self.b_acc, self.interp_coeff, 
                                                self.a_gyr, self.b_gyr, 0, self.w_mag)
            state['particle_result'] = True
                
 
        
    @staticmethod
    def update_step(quaternion, gyroscope, accelerometer, magnetometer, dt, 
                    bias_gyro, defacc, a_acc, b_acc, interp_coeff, a_gyr, b_gyr, w_acc, w_mag):
        # Normalise accelerometer measurement
        if np.linalg.norm(accelerometer) == 0:
            return quaternion
        
        acc_norm = np.linalg.norm(accelerometer)
        acc = accelerometer / acc_norm
                
        acc_norm_coeff = 1 / (1 + np.exp(np.abs(acc_norm - defacc) * a_acc - b_acc) )
        
        gyro_denoised = (gyroscope - bias_gyro)
        gyr_norm_coeff = 1 / (1 + np.exp(np.linalg.norm(gyro_denoised)*a_gyr-b_gyr))
        multiplier = (interp_coeff * acc_norm_coeff + (1 - interp_coeff) * gyr_norm_coeff)
        
        # Normalise magnetometer measurement
        if np.linalg.norm(magnetometer) == 0:
            return quaternion
        mag = magnetometer / np.linalg.norm(magnetometer)
        
        # Gyroscope integration
        #q_dot = 0.5 * dt * quatern_prod(q, np.array([0, *(self.scale_factors * (gyroscope - self.bias_gyro))]))
        qp = integrate_euler(quaternion, gyro_denoised, dt)
        
        
        # Rotation matrix
        Rt = np.array([
            [2*(0.5 - qp[2]**2 - qp[3]**2), 0, 2*(qp[1]*qp[3] - qp[0]*qp[2])],
            [2*(qp[1]*qp[2] - qp[0]*qp[3]), 0, 2*(qp[0]*qp[1] + qp[2]*qp[3])],
            [2*(qp[0]*qp[2] + qp[1]*qp[3]), 0, 2*(0.5 - qp[1]**2 - qp[2]**2)]
        ])

        # Predicted accelerometer
        ar = np.array([0, 0, 1])
        acc_mes_pred = Rt @ ar
        
        # Magnetic reference
        h = quatern_prod(quaternion, quatern_prod(np.array([0, *mag]), quatern_conj(quaternion)))
        mr = np.array([np.linalg.norm([h[1], h[2]]), 0, h[3]])
        mr = mr / np.linalg.norm(mr)
        mag_mes_pred = Rt @ mr
        
        # Accelerometer correction
        ca = np.cross(acc, acc_mes_pred)
        na = np.linalg.norm(ca)
        veca = ca / na
        
        phia = w_acc * multiplier
            
            
        # Magnetometer correction
        cm = np.cross(mag, mag_mes_pred)
        n = np.linalg.norm(cm)
        vecm = cm / n
        
        phim = w_mag
            
        # Correction quaternion
        q_cor = np.array([1, *(veca * phia / 2 + vecm * phim / 2)])
        
        quat = quatern_prod(qp, q_cor)
        
        if quat[0] < 0:
            quat = -quat
            
        return quat / np.linalg.norm(quat)
        
        
    def update(self, gyroscope, accelerometer, magnetometer, dt):
        """
        Update the filter with MARG sensor data
        
        Args:
            gyroscope: Gyroscope measurement [gx, gy, gz] in rad/s
            accelerometer: Accelerometer measurement [ax, ay, az]
            magnetometer: Magnetometer measurement [mx, my, mz]
            dt: Sample period in seconds
        """
        
        acc_norm = np.linalg.norm(accelerometer)
        acc_norm_coeff = 1 / (1 + np.exp(np.abs(acc_norm - self.defacc) * self.a_acc - self.b_acc) )
        
        particle_result = acc_norm_coeff < 0.7
        self.coefs.append(particle_result.astype(int))
        
        if particle_result:
            self.predict_particle()
            if len(self.particle_history) > 0:
                self.quaternion = self.particle_history[-1]['quaternion']
            
        accw = 0 if particle_result else self.w_acc
        
        self.quaternion = self.update_step(
            self.quaternion, gyroscope, accelerometer, magnetometer, dt,
            self.bias_gyro, self.defacc, self.a_acc, self.b_acc,
            self.interp_coeff, self.a_gyr, self.b_gyr, accw, self.w_mag
        )
        
        state = {'quaternion': self.quaternion,
                  'accelerometer': accelerometer,
                  'magnetometer': magnetometer,
                  'gyroscope': gyroscope,
                  'dt': dt,
                  'particle_result': particle_result}
        
        #fifo append
        if len(self.particle_history) > self.hist_depth:
            self.particle_history.pop(0)
            
        self.particle_history.append(state)    
        

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
        
        #magnetic correction [0 1 0] reference -> mag_mes_pred[0] < 0 -> +w_mag correction, else -w_mag correction
        veca[2] += -self.w_mag if mag_pr_x < 0 else self.w_mag
        
        # Correction quaternion
        q_cor = np.array([1, *(veca)])
        
        quat = quatern_prod_single(q_cor, qp)
        
        if quat[0] < 0:
            quat = -quat
            
        self.quaternion = fast_normalize_4d(quat)

