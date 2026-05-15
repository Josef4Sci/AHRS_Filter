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
from quaternion_library_jit import quatern_prod, quatern_conj, quaternion_rotate_vector, quatern_prod_single

class JustaAHRSPure:
    """
    Justa AHRS Pure implementation
    """
    
    def __init__(self, quaternion=None, w_acc=0.00248, w_mag=1.35e-04):
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64) if quaternion is None else np.array(quaternion, dtype=np.float64)
        self.quaternion_out = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.w_acc = w_acc
        self.w_mag = w_mag
        self.gain = 0.0528152
        self.bias_gyro = np.array([0.0, 0.0, 0.0]) * np.pi / 180  # Example bias        self.bias_history = []        self.bias_history.append(self.bias_gyro.copy())


        

    @jit(nopython=True, cache=True)
    def update(self, gyroscope, accelerometer, magnetometer, dt):
        """
        Update the filter with MARG sensor data
        
        Args:
            gyroscope: Gyroscope measurement [gx, gy, gz] in rad/s
            accelerometer: Accelerometer measurement [ax, ay, az]
            magnetometer: Magnetometer measurement [mx, my, mz]
            dt: Sample period in seconds
        """
        q = self.quaternion.astype(np.float64)
        
        # Normalise accelerometer measurement
        if np.linalg.norm(accelerometer) == 0:
            return
        acc = accelerometer / np.linalg.norm(accelerometer)
        acc = acc.astype(np.float64)
        
        # Normalise magnetometer measurement
        if np.linalg.norm(magnetometer) == 0:
            return
        mag = magnetometer / np.linalg.norm(magnetometer)
        mag = mag.astype(np.float64)
        
        # Gyroscope prediction
        wt = (gyroscope - self.bias_gyro) * dt / 2
        q_dot_x = np.sin(wt[0])
        q_dot_y = np.sin(wt[1])
        q_dot_z = np.sin(wt[2])
        q_dot_w = np.sqrt(1 - (q_dot_x**2 + q_dot_y**2 + q_dot_z**2))
        
        q_dot = np.array([q_dot_w, q_dot_x, q_dot_y, q_dot_z], dtype=np.float64)
        qp = quatern_prod_single(q, q_dot)
        
        # Rotation matrix
        # R = np.array([
        #     [2*(0.5 - qp[2]**2 - qp[3]**2), 0, 2*(qp[1]*qp[3] - qp[0]*qp[2])],
        #     [2*(qp[1]*qp[2] - qp[0]*qp[3]), 0, 2*(qp[0]*qp[1] + qp[2]*qp[3])],
        #     [2*(qp[0]*qp[2] + qp[1]*qp[3]), 0, 2*(0.5 - qp[1]**2 - qp[2]**2)]
        # ])
        
        # Predicted accelerometer
        ar = np.array([0, 0, 1], dtype=np.float64)
        acc_mes_pred = quaternion_rotate_vector(qp, ar)
        
        # Magnetic reference
        mr_z = np.dot(acc_mes_pred, mag)
        mr_x = np.sqrt(1 - mr_z**2)
        mr = np.array([0, mr_x, mr_z], dtype=np.float64)
        
        mag_mes_pred = quaternion_rotate_vector(qp, mr)
        
        # Accelerometer correction
        ca = np.cross(acc, acc_mes_pred)
        vec_a = ca / np.linalg.norm(ca)
        
        # Magnetometer correction
        cm = np.cross(mag, mag_mes_pred)
        vec_b = cm / np.linalg.norm(cm)
        self.quaternion = fast_normalize_4d(self.quaternion)
        # Combined correction
        im = vec_a * self.w_acc / 2 + vec_b * self.w_mag / 2
        im2 = im * np.sinc(np.linalg.norm(im) / np.pi)
        q_cor = np.array([np.sqrt(1 - np.sum(im2**2)), *im2], dtype=np.float64)
        
        # Final quaternion
        quat = quatern_prod_single(qp, q_cor)
        
        if quat[0] < 0:
            quat        
 = -quat
        
        res_norm = quat / np.linalg.norm(quat)
        
        self.quaternion = res_norm.astype(np.float64)

    
    
class JustaAHRSv2:
    
    class Particle:
        def __init__(self, w_acc, w_mag):
        self.quaternion = []
on
            self.w_acc = w_acc
            self.w_mag = w_mag
            self.result = False
            
            self.hist_depth = 5
            self.history = []
            
        def update_history(self, new_last_quat):
            
            if len(self.history) == 0:
                return
            
            diff = quatern_prod_single(quatern_conj_single(self.history[0]['quaternion']), new_last_quat)
            
            for state in self.history:
                state['quaternion'] = quatern_prod_single(state['quaternion'], diff)        
        self.quaternion = self.filter.quaternion

            
        def get_last_quaternion(self):
            return self.quaternion
nion']
        
        def set_diff_quaternion(self, selected_quaternion):
            last = self.get_last_quaternion()
            inv_last = quatern_conj_single(last)
            self.selected_diff_q = quatern_prod_single(inv_last, selected_quaternion)

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
        particle_filters = [JustaAHRSInvFast(w_acc=w_acc, w_mag=w_mag), JustaAHRSInvFast(w_acc=0, w_mag=0)]
        
        self.w_acc = w_acc
        self.w_mag = w_mag
        self.defacc = 0.9791458
        self.coefs = []
        self.bias_gyro = np.zeros(3, dtype=np.float64)
        self.acc_ref = np.array([0.0, 0.0, 1.0])
        
        
        self.interp_coeff = 0.5
        self.a_gyr = 5.0
        self.b_gyr = 2.0
        self.a_acc = 22.0
        self.b_acc = 2.0
        
        self.hist_depth = 5
 = quaternion_derivative(q2, gyro)
    
    q3 = q + 0.5 * k2 * dt
    q3 = q3 / np.linalg.norm(q3)
    k3 = quaternion_derivative(q3, gyro)
    
    q4 = q + k3 * dt
    q4 = q4 / np.linalg.norm(q4)
    k4 = quaternion_derivative(q4, gyro)
    
    q_new = q + (k1 + 2*k2 + 2*k3 + k4) * dt / 6
    return q_new

    

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
        self.bias_gyro = np.zeros(3, dtype=np.float64)
        #self.bias_gyro[:] = bias_gyro * np.pi / 180  # Example bias
        self.scale_factors = np.array([s1, s2, s3])
        
        self.interp_coeff = 0.5
        self.a_gyr = 5.0
        self.b_gyr = 2.0
        self.a_acc = 22.0
        self.b_acc = 2.0
        
        self.particle_history = []
        self.particle_histories = []
        
        self.hist_depth = 2
        
    def initFromAccMag(self, accelerometer, magnetometer):
        self.quaternion = wahba_constrained(np.array([0, 0, 1]), accelerometer, np.array([0, 1, 0]), magnetometer)[0]
        for p in self.particle_filters:
            p.set_quaternion(self.quaternion)

f):
                                      
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
                    bias_gyro,        self.defacc = np.linalg.norm(accelerometer)
 defacc, a_acc, b_acc, interp_coeff, a_gyr, b_gyr, w_acc, w_mag):
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
        qp = integrate_rk4(quaternion, gyro_denoised, dt)
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
        h = quatern_prod(quaternion, quatern_prod(np.array([0, *mag]), quatern_conj(quaternion)))
        mr = np.array([np.linalg.norm([h[1], h[2]]), 0, h[3]])
        mr = mr / np.linalg.norm(mr)
        mag_mes_pred = R @ mr
        
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
        
        # if particle_result:
        #     self.predict_particle()
        #     if len(self.particle_history) > 0:
        #         self.quaternion = self.particle_history[-1]['quaternion']
            
        # accw = 0 if particle_result else self.w_acc
        
        s        self.test_multip = test_multip
elf.qu        self.acc_modif = 0.0
aternion = self.update_step(
            self.quaternion, gyroscope, accelerometer, magnetometer, dt,
            self.bias_gyro, self.defacc, self.a_acc, self.b_acc,
            self.interp_coeff, self.a_gyr, self.b_gyr, self.w_acc, self.w_mag
        )
        
        state = {'quaternion': self.quaternion,
                  'accelerometer': accelerometer,
                  'magnetometer': magnetometer,
                  'gyroscope': gyroscope,
                  'dt':        self.integ_mod = 0
        
 dt,
                  'particle_result': particle_result}
        
        #fifo append
        if len(self.particle_history) > self.hist_depth:
            self.particle_history.pop(0)
            
        self.particle_history.append(state)    
        

class JustaAHRSPureFastClean:
    """
    Justa AHRS Pure Fa        self.defacc = np.linalg.norm(accelerometer)
st implementation
    """
    
    def __init__(self, quaternion=None, gain=0.0528152, w_acc=0.00248, w_mag=1.35e-04):
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0]) if quaternion is None else np.array(quaternion)
        self.gain = gain
        self.w_acc = w_acc
        self.w_mag = w_mag
        
    def update(self, gyroscope        # from matplotlib import pyplot as plt
        # plt.figure()
        # min_x = np.min(points_x)
        # max_x = np.max(points_x)
        # test_p = np.linspace(min_x , max_x , 1000)
        # test_y = self.p_cubic(test_p)
        
        # plt.plot(points_x, points_y, "o", label="Nodes")
        # plt.plot(test_p, test_y, label=f"Local poly order {self.p_cubic.order}")
        # plt.show()
, accelerometer, magnetometer, dt):
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
        
        # Gyroscope integration
        q_dot = 0.5 * dt * quatern_prod(q, np.array([0, *gyroscope]))
        qp = q + q_dot
        qp = qp / np.linalg.norm(qp)
        
                c = 0.99
        self.integ_mod = self.integ_mod * c + acc_coef * (1-c)
        
        #clamp acc_c2 to [0, 1]
        acc_c2 = max(0, min(1, acc_c2))
        
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
        ca = np.cross(acc,        self.out_mod.append(na)
 acc_mes_pred)
        na = np.linalg.norm(ca)
        veca = ca / na
        
        phia = self.w_acc
            
        # Magnetometer correction
        cm = np.cross(mag, mag_mes_pred)
        n = np.linalg.norm(cm)
        vecm = cm / n
        
        phim = self.w_mag
            
        # Correction quaternion
        q_cor = np.array([1, *(veca * phia / 2 + vecm * phim / 2)])
        
        quat = quatern_prod(qp, q_cor)
        
        if quat[0] < 0:
            quat = -quat
            
        self.quaternion = quat / np.linalg.norm(quat)
