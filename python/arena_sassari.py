from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt

import sys
sys.path.append('python\\vqf_local\\vqf')  # folder containing vqf.pyx and vqf.pyxbld
import pyximport
pyximport.install(setup_args={"include_dirs": []}, language_level=3)
from vqf import VQF

import numpy as np
import pandas as pd
from filters.justa_ahrs import JustaAHRSInvFast, JustaAHRSInv, JustaAHRSPure
from utils import angle_error, eval_filter_on_dataset, plot_dataset


RMSE = True

dataset_loader = DatasetLoader()
names = ['slow_v4.mat', 'medium_v4.mat', 'fast_v4.mat']
test_datasets={}
for i in range(6):
    for n in names:    
        dat = dataset_loader.load_sassari_dataset(n, i)
        bias = dat['gyroscope'][:500].mean(axis=0)
        dat['gyroscope']=dat['gyroscope']*np.array([1.015, 1.015, 1.01]) - bias
        test_datasets[n+str(i)] = dat
    
    
# single_test = 'fast_v4.mat0'
# test_datasets= {single_test: test_datasets[single_test]} # only first dataset

#plot_dataset(test_datasets[single_test])

test_filters = { # 9DOF
    'vqf': {'filter': {'tauAcc': 1.2, 'tauMag': 7.0}, 'errors': [], 'type': 1, 'justa': False},
    'justa_cpp': {'filter': {'tauAcc': 1.882, 'tauMag': 0.23}, 'errors': [], 'type': 1, 'justa': True},
    #'justa_inv':  {'filter': JustaAHRSInvFast(w_acc=1, w_mag=1), 'errors': [], 'type': 0},
}

single_dataset = len(test_datasets)==1

errors_filters = dict.fromkeys(test_filters.keys(), [])

for dataset_name, dat in test_datasets.items():
    print(f"Evaluating dataset: {dataset_name}")

    dat_cp = dat.copy()
    bias = dat_cp['gyroscope'][:dat_cp['start_time']['index']].mean(axis=0)
    dat_cp['gyroscope'] = dat_cp['gyroscope'] - bias
    
    for filter_name, j_filter in test_filters.items():       
        if j_filter['type'] == 1:  # vqf
            
            gyr = np.ascontiguousarray(dat_cp['gyroscope'], dtype=np.float64)
            acc = np.ascontiguousarray(dat_cp['accelerometer'], dtype=np.float64)
            mag = np.ascontiguousarray(dat_cp['magnetometer'], dtype=np.float64)
            
            vq = VQF(1/100, tauAcc=j_filter['filter']['tauAcc'], tauMag=j_filter['filter']['tauMag'],
                     motionBiasEstEnabled=False, restBiasEstEnabled=False,
                     magDistRejectionEnabled=False, useJustaFilter=j_filter['justa'])
                      
            res = vq.updateBatch(gyr, acc, mag)
            result = res['quat9D']
        else:
            j_filter['filter'].initFromAccMag(dat_cp['accelerometer'][0], dat_cp['magnetometer'][0])
            result = eval_filter_on_dataset(j_filter['filter'], dat_cp, use_imu=False, use_square_err=False)
        
        alignIndex = int(dat_cp['start_time']['index']*0.5)
        error_timeserie = angle_error(result, dat_cp['reference'], align_start=True, shift_samples=0, align_index=alignIndex)
        
        
        if RMSE:
            error = np.sqrt(np.nanmean(error_timeserie**2))
        else:
            error = np.nanmean(error_timeserie)
            
        print(f"{filter_name} Mean Error: {error:.2f} deg")
        if single_dataset:
            j_filter['errors'].append(error_timeserie)
        else:
            j_filter['errors'].append(error)

if single_dataset:
    for filter_name, filter in test_filters.items():
        plt.plot(filter['errors'][0], label=filter_name)
else:
    for filter_name, filter in test_filters.items():
        plt.plot(list(test_datasets.keys()), filter['errors'], label=filter_name)

plt.xticks(rotation=45)
plt.xlabel('Dataset')
plt.ylabel('Mean Error (deg)')
plt.legend()
plt.show()

for filter_name, filter in test_filters.items():
    print(f"{filter_name} Mean Error across datasets: {np.mean(filter['errors']):.2f} deg")

