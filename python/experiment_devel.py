from dataset_loader import DatasetLoader
from filters import (JustaAHRSv2, JustaAHRSPure)
from utils import eval_filter_on_dataset, plot_dataset
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
import pickle

from quaternion_library import quatern_prod, quatern_conj

#evaluate filter on Justa dataset
def angular_velocity_finite_difference(q_curr, q_prev, dt):
    """
    Method 1: Finite difference using quaternion derivative.
    
    From: q_dot = 0.5 * q ⊗ [0, wx, wy, wz]
    Solve for omega: [0, wx, wy, wz] = 2 * q_conj ⊗ q_dot
    
    This is the most common and straightforward method.
    """
    # Ensure quaternions are normalized
    q_curr = q_curr / np.linalg.norm(q_curr)
    q_prev = q_prev / np.linalg.norm(q_prev)
    
    # Compute quaternion derivative (finite difference)
    q_dot = (q_curr - q_prev) / dt
    
    # if q_dot[0] < 0:
    #     q_dot = -q_dot
    
    # Conjugate of current quaternion
    q_conj = quatern_conj(q_curr)
    
    if q_conj[0] < 0:
        q_conj = -q_conj
    
    # Compute omega quaternion: [0, wx, wy, wz] = 2 * q_conj ⊗ q_dot
    omega_quat = 2.0 * quatern_prod(q_conj, q_dot)
    
    # Extract angular velocity (ignore w component, should be ~0)
    omega = omega_quat[1:4]
    
    if omega[0] < 0:
        omega = omega
    return omega


dataset_loader = DatasetLoader()
#dataset = dataset_loader.get_dataset_segment('Synth2', 0, 3000)
dataset = dataset_loader.load_dataset('Justa')

# qcor= np.array([ 9.99976582e-01, 3.87208520e-04,  -6.79134620e-03,  -7.49832763e-04])
qcor= np.array([ 9.99976582e-01, 3.87208520e-04,  -6.79134620e-03,  -7.49832763e-04])
#qcor= np.array([ 1, 0,  0,  0])

for i, t in enumerate(dataset['reference']):
    dataset['reference'][i] = quatern_prod(quatern_prod(quatern_conj(qcor), t), qcor)

#dataset = pickle.load( open('synthetic_rigid_body_sensor_offset.pkl', 'rb') )#
# plot_dataset(dataset)

# n ='01_undisturbed_slow_rotation_A.mat'
# dataset = dataset_loader.load_broad_dataset(n)

gyr_off_err = 0.0 # Example gyro offset error in deg/s #w_acc=0.00015, w_mag=1.35e-04
filter_instance = JustaAHRSv2(quaternion=dataset['reference'][1],gain=15.0928152, w_acc=0.000, w_mag=0,\
                                s1=0.000594, s2= 5.202019, s3=-1.849, \
                                bias_gyro=np.zeros(3)) # s1=1.0, s2=1.0, s3=0.98

filter_instance = JustaAHRSPure(quaternion=dataset['reference'][1], w_acc=0.01, w_mag=0.1)

#filter_instance = JustaAHRSPure(quaternion=dataset['reference'][1])

quaternion_result, angle_err = eval_filter_on_dataset(
    filter_instance, dataset, use_imu=False, use_square_err=False)

mean_error = np.mean(angle_err) #[3:600]

# defacc = np.mean(np.linalg.norm(dataset['gyroscope'][0:200,:], axis=1))
# print(f"Defacc: {defacc}")


# bias = np.mean(dataset['gyroscope'][0:500,:], axis=0)
# print(f"Gyroscope Bias: {bias* (180/np.pi)} deg/s")
# bias = np.mean(dataset['gyroscope'][0:400,:], axis=0)
# print(f"Gyroscope Bias: {bias* (180/np.pi)} deg/s")

plt.figure()
plt.plot(dataset['time'], angle_err, label='Angular Error', alpha=0.7)
# #plt.plot(dataset['time'][1:], filter_instance.coefs, label='Particle Result Coefs', alpha=0.7)
# #plt.plot(dataset['time'][0:500], dataset['gyroscope'][0:500,:], label='Gyroscope (scaled)')
# plt.xlabel('Time (s)')
# plt.ylabel('Angular Error (degrees)')

plt.show()

print(f"Mean Angular Error: {mean_error:.6f} degrees")