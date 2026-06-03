import time
from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from filters.justa_ahrs import JustaAHRSInvFast, JustaAHRSPure, JustaAHRSInvButterworth
from utils import angle_error, eval_filter_on_dataset, plot_dataset, qdiff, angle_diff_deg
import pickle
from quaternion_library import quatern_prod, quatern_conj

import sys
sys.path.append('python\\vqf_local\\vqf')  # folder containing vqf.pyx and vqf.pyxbld
import pyximport
pyximport.install(setup_args={"include_dirs": []}, language_level=3)

from vqf import VQF as PyVQF

DOF6 = False

VQF = False

dl = DatasetLoader()
# dataset_name = 'slow_v4.mat'
# dat = dl.load_sassari_dataset(dataset_name, 2)

test_datasets = {}
dl = DatasetLoader()
dat = dl.load_broad_dataset(file_name='07_undisturbed_fast_rotation_B.mat', mean_initial_samples=True)
start_index = dat['start_time']['index'] if dat.keys().__contains__('start_time') else 2000
bias = dat['gyroscope'][:start_index].mean(axis=0)
dat['gyroscope'] = dat['gyroscope'] - bias

if VQF:
    b = PyVQF(1.0/dat['mean_sampling_rate'], tauAcc=1.294, tauMag=1.44, motionBiasEstEnabled=False, restBiasEstEnabled=False, magDistRejectionEnabled=False, useJustaFilter=False)

    gyr = np.ascontiguousarray(dat['gyroscope'], dtype=np.float64)
    acc = np.ascontiguousarray(dat['accelerometer'], dtype=np.float64)
    mag = np.ascontiguousarray(dat['magnetometer'], dtype=np.float64)
    res = b.updateBatch(gyr, acc, mag)
    
    if DOF6:
        quaternion_result = res['quat6D']
    else:
        quaternion_result = res['quat9D']
else:
    # #j_filter = JustaAHRSInvButterworth()
    # j_filter = JustaAHRSlp2(w_acc=0.000168, w_mag=0.0, linMag=True, lp_stage=4)
    # #j_filter = JustaAHRSPure(w_acc=0.2, w_mag=0.2)

    # j_filter.initFromAccMag(dat['accelerometer'][0], dat['magnetometer'][0]) 

    # quaternion_result = eval_filter_on_dataset(j_filter, dat)
    
    b = PyVQF(1.0/dat['mean_sampling_rate'], tauAcc=1.294, tauMag=1.44, motionBiasEstEnabled=False, restBiasEstEnabled=False, magDistRejectionEnabled=False, useJustaFilter=True)

    gyr = np.ascontiguousarray(dat['gyroscope'], dtype=np.float64)
    acc = np.ascontiguousarray(dat['accelerometer'], dtype=np.float64)
    mag = np.ascontiguousarray(dat['magnetometer'], dtype=np.float64)
    res = b.updateBatch(gyr, acc, mag)
    
    if DOF6:
        quaternion_result = res['quat6D']
    else:
        quaternion_result = res['quat9D']
    
diff = angle_error(quaternion_result, dat['reference'], align_start=True, shift_samples=0, align_index=3000, use_imu=DOF6)
print(f"Mean angle error: {np.mean(diff):.4f} deg")

#noise
quaternion_result_noise = np.abs(np.diff(diff, axis=0))
print(f"Mean quaternion change: {quaternion_result_noise.mean():.6f}")


plt.plot(diff, label='Reference Norm')

# if not VQF:
#     plt.plot(b.coefs, label=[f"coef {i}" for i in range(len(b.coefs[0]))])
#     # plt.plot(np.array(j_filter.coefs)[:,:], label=[f"coef {i}" for i in range(3)])
plt.legend()
plt.show()