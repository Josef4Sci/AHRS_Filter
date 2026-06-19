import pickle
from time import perf_counter as time

import numpy as np

from quaternion_library import quatern_prod, quatern_conj
import matplotlib.pyplot as plt
import sys
sys.path.insert(0, 'python\\vqf_local')

from vqf import VQF
from dataset_loader import DatasetLoader
from utils import FilterType, get_optim_params_all

N_COPY = 100

test_datasets = {}
dl = DatasetLoader()
dat = dl.load_broad_dataset(file_name='07_undisturbed_fast_rotation_B.mat', mean_initial_samples=True)

gyr = np.row_stack([dat['gyroscope']] * N_COPY)
acc = np.row_stack([dat['accelerometer']] * N_COPY)
mag = np.row_stack([dat['magnetometer']] * N_COPY)

gyr = np.ascontiguousarray(gyr, dtype=np.float64)
acc = np.ascontiguousarray(acc, dtype=np.float64)
mag = np.ascontiguousarray(mag, dtype=np.float64)

filters = get_optim_params_all()

for filter_name, j_filter in filters.items():
    j_filter['times'] = []

t_base = []

for i in range(5):
    for filter_name, j_filter in filters.items():
        vqf_full = j_filter['filter_type'] == FilterType.FILTER_VQF
        if j_filter['filter_type'] == FilterType.FILTER_FAST_VQF or j_filter['filter_type'] == FilterType.FILTER_JUSTA_ORIG:
            gyro_int = 1 
        else:
            gyro_int = 0
            
        vq = VQF(1.0/dat['mean_sampling_rate'], 
                    tauAcc=j_filter['filter']['tauAcc'], tauMag=j_filter['filter']['tauMag'],
                    motionBiasEstEnabled=vqf_full, restBiasEstEnabled=vqf_full,
                    magDistRejectionEnabled=vqf_full, filterType=int(j_filter['filter_type']), gyroIntegrationMethod=gyro_int)
        start_time = time()
        res = vq.updateBatch(gyr, acc, mag) 
        time_end_vqf = time()
        j_filter['times'].append(time_end_vqf - start_time)
    
    
    b = VQF(1.0/dat['mean_sampling_rate'], tauAcc=1.1, tauMag=1.1, 
                motionBiasEstEnabled=False, restBiasEstEnabled=False, magDistRejectionEnabled=False,
                filterType=int(FilterType.FILTER_SKIP))
    
    time_start_base = time()
    res = b.updateBatch(gyr, acc, mag)  
    time_end_base = time()
    t_base.append(time_end_base - time_start_base)
    print(f"Iteration {i+1}:")

len_gyr = gyr.shape[0]
t_base_ns = np.mean(t_base)*1e9/len_gyr

for filter_name, j_filter in filters.items():
    t_vqf = np.mean(j_filter['times'])
    print(f"Filter: {filter_name}, time taken clean: {t_vqf*1e9/len_gyr - t_base_ns} ns")
    
# plt.plot(res['quat9D'])
# plt.show()
