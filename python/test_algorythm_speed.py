import pickle
from time import time

import numpy as np

from quaternion_library import quatern_prod, quatern_conj
import matplotlib.pyplot as plt
import sys
sys.path.insert(0, 'python\\vqf_local')

from vqf import VQF
from dataset_loader import DatasetLoader

N_COPY = 100

BASIC_VQF = False

test_datasets = {}
dl = DatasetLoader()
dat = dl.load_broad_dataset(file_name='07_undisturbed_fast_rotation_B.mat', mean_initial_samples=True)

gyr = np.row_stack([dat['gyroscope']] * N_COPY)
acc = np.row_stack([dat['accelerometer']] * N_COPY)
mag = np.row_stack([dat['magnetometer']] * N_COPY)

gyr = np.ascontiguousarray(gyr, dtype=np.float64)
acc = np.ascontiguousarray(acc, dtype=np.float64)
mag = np.ascontiguousarray(mag, dtype=np.float64)

t_vqf = 0
t_justa = 0
t_base = 0

for i in range(10):
    if BASIC_VQF:
        b = VQF(1.0/dat['mean_sampling_rate'], tauAcc=1.1, tauMag=1.1, 
                    motionBiasEstEnabled=False, restBiasEstEnabled=False, magDistRejectionEnabled=False,
                    useJustaFilter=False, useAccLp=False)
    else:
        b = VQF(1.0/dat['mean_sampling_rate'], tauAcc=1.1, tauMag=1.1, 
                    motionBiasEstEnabled=True, restBiasEstEnabled=True, magDistRejectionEnabled=True,
                    useJustaFilter=False, useAccLp=False)

    time_start_vqf = time()
    res = b.updateBatch(gyr, acc, mag)  
    time_end_vqf = time()
    t_vqf += time_end_vqf - time_start_vqf
    
    b = VQF(1.0/dat['mean_sampling_rate'], tauAcc=1.1, tauMag=1.1, 
                motionBiasEstEnabled=False, restBiasEstEnabled=False, magDistRejectionEnabled=False,
                useJustaFilter=True, useAccLp=False, gyroIntegrationMethod=1)

    time_start_justa = time()
    res = b.updateBatch(gyr, acc, mag)  
    time_end_justa = time()
    t_justa += time_end_justa - time_start_justa
    b = VQF(1.0/dat['mean_sampling_rate'], tauAcc=1.1, tauMag=1.1, 
                motionBiasEstEnabled=False, restBiasEstEnabled=False, magDistRejectionEnabled=False,
                useJustaFilter=True, useAccLp=True)
    
    time_start_base = time()
    res = b.updateBatch(gyr, acc, mag)  
    time_end_base = time()
    t_base += time_end_base - time_start_base
    print(f"Iteration {i+1}:")

len_gyr = gyr.shape[0]*N_COPY

print(f"Time taken VQF clean: {(t_vqf-t_base)*1e9/len_gyr} ns")
print(f"Time taken Justa clean: {(t_justa-t_base)*1e9/len_gyr} ns")
print(f"Time taken Base: {t_base*1e9/len_gyr} ns")

print(f"Improvement: {(1 - ((t_justa - t_base) / (t_vqf - t_base))) * 100:.2f}%")
# plt.plot(res['quat9D'])
# plt.show()
