from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt

import sys
sys.path.append('python\\vqf_local\\vqf')  # folder containing vqf.pyx and vqf.pyxbld
import pyximport
pyximport.install(setup_args={"include_dirs": []}, language_level=3)

from vqf import VQF as PyVQF
# from vqf_local.vqf.pyvqf import PyVQF
import numpy as np
import pandas as pd
from filters.justa_ahrs import JustaAHRSInvFast, JustaAHRSInv, JustaAHRSPure, JustaAHRSlp2
from utils import angle_error, eval_filter_on_dataset, plot_dataset

    
dataset_loader = DatasetLoader()
names = ['slow_v4.mat', 'medium_v4.mat', 'fast_v4.mat']
test_datasets={}
for i in range(1):
    for n in names:    
        dat = dataset_loader.load_sassari_dataset(n, i)
        # bias = dat['gyroscope'][:500].mean(axis=0)
        # dat['gyroscope']=dat['gyroscope']*np.array([1.015, 1.015, 1.01]) - bias
        test_datasets[n+str(i)] = dat
    
    
single_test = 'fast_v4.mat0'    
test_datasets= {single_test: test_datasets[single_test]} # only first dataset

test_filters = { # 9DOF
    'vqf': {'filter': {'tauAcc': 1.2, 'tauMag': 7.0}, 'errors': [], 'type': 1, 'justa': False},
    'justa_cpp': {'filter': {'tauAcc': 1.5, 'tauMag': 1.3}, 'errors': [], 'type': 1, 'justa': True},
    #'justa_inv':  {'filter': JustaAHRSInvFast(w_acc=1, w_mag=1), 'errors': [], 'type': 0},
}

single_dataset = len(test_datasets)==1

errors_filters = dict.fromkeys(test_filters.keys(), [])

for dataset_name, dat in test_datasets.items():
    print(f"Evaluating dataset: {dataset_name}")

    for filter_name, filter in test_filters.items():       
        if filter['type'] == 1:  # vqf
            # j_filter['filter'].
            gyr = np.ascontiguousarray(dat['gyroscope'], dtype=np.float64)
            acc = np.ascontiguousarray(dat['accelerometer'], dtype=np.float64)
            mag = np.ascontiguousarray(dat['magnetometer'], dtype=np.float64)
            vq = PyVQF(1.0/dat['mean_sampling_rate'], 
                       tauAcc=filter['filter']['tauAcc'], tauMag=filter['filter']['tauMag'],
                       motionBiasEstEnabled=True, restBiasEstEnabled=True, magDistRejectionEnabled=False,
                       useJustaFilter=filter['justa'], staticAccThreshold=0.9, staticGyrThreshold=0.5, staticMagThreshold=3.0, staticWindowSize=3, staticBlockForwardSteps=500) #useMag=True, useMagStepWhole=False, useMagStepLinear=True, useJustaFilter=filter['justa'], JustaFIlterVersionOld=True
            #vq.coeffs['gyrTs'] = 1.0/dat['mean_sampling_rate']            
            res = vq.updateBatch(gyr, acc, mag)
            result = res['quat9D']
        else:
            filter['filter'].initFromAccMag(dat['accelerometer'][0], dat['magnetometer'][0])
            result = eval_filter_on_dataset(filter['filter'], dat)
        
        alignIndex = int(dat['start_time']['index']*0.5)
        error_9D = angle_error(result, dat['reference'], align_start=True, shift_samples=0, align_index=alignIndex)
        
        print(f"{filter_name} Mean Error: {np.mean(error_9D):.2f} deg")
        if single_dataset:
            filter['errors'].append(error_9D)
        else:
            filter['errors'].append(np.mean(error_9D))

if single_dataset:
    for filter_name, filter in test_filters.items():
        plt.plot(filter['errors'][0], label=filter_name)
else:
    for filter_name, filter in test_filters.items():
        plt.plot(list(test_datasets.keys()), filter['errors'], label=filter_name)

plt.xlabel('Dataset')
plt.ylabel('Mean Error (deg)')
plt.legend()
plt.show()

for filter_name, filter in test_filters.items():
    print(f"{filter_name} Mean Error across datasets: {np.mean(filter['errors']):.2f} deg")

