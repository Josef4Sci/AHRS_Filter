from dataset_loader import DatasetLoader
from filters import (JustaAHRSPure, JustaAHRSInvFast)
from utils import eval_filter_on_dataset, plot_dataset, angle_error
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
import pickle

import sys
sys.path.append('python\\vqf_local\\vqf')   # folder containing vqf.pyx and vqf.pyxbld
import pyximport
pyximport.install(setup_args={"include_dirs": []}, language_level=3)
from vqf import VQF

from quaternion_library import quatern_prod, quatern_conj
from quaternion_library_jit import quaternion_rotate_vector, quatern_prod_single, quatern_conj_single

dataset = pickle.load( open('synthetic_rigid_body_sensor_offset.pkl', 'rb') )

#plot_dataset(dataset)

test_datasets = {}
test_datasets[ 'synth'] = dataset

test_filters = {
    # 'JustaAHRSv2': {'filter': JustaAHRSv2( w_acc=0.00034, w_mag=0.00022), 'errors': [], 'type': 0},
    #
    'JustaAHRSv4': {'filter': JustaAHRSInvFast(w_acc=0.7, w_mag=0.2), 'errors': [], 'type': 0},
    # 'JustaAHRSPure': {'filter': JustaAHRSPure(w_acc=1, w_mag=1), 'errors': [], 'type': 0},
    'vqf': {'filter': None, 'errors': [], 'type': 1, 'par1': False},
    'justa_cpp': {'filter': None, 'errors': [], 'type': 1, 'par1': True},
}


single_dataset = len(test_datasets)==1

errors_filters = dict.fromkeys(test_filters.keys(), [])

for dataset_name, dat in test_datasets.items():
    print(f"Evaluating dataset: {dataset_name}")

    for filter_name, j_filter in test_filters.items():       
        if j_filter['type'] == 1:  # vqf
            # j_filter['filter'].
            gyr = np.ascontiguousarray(dat['gyroscope'], dtype=np.float64)
            acc = np.ascontiguousarray(dat['accelerometer'], dtype=np.float64)
            mag = np.ascontiguousarray(dat['magnetometer'], dtype=np.float64)
            
            j_filter['filter']
            tauAcc = 0.7
            tauMag = 0.2 
            useAccStep = j_filter['par1']
            vq = VQF(1.0/dat['mean_sampling_rate'], tauAcc=tauAcc, tauMag=tauMag, useAccStepWhole=useAccStep)
            vq.setStepQuat(np.array([1,0,0,0], dtype=np.float64))
            #vq.coeffs['gyrTs'] = 1.0/dat['mean_sampling_rate']
            #measure time taken for updateBatch
            time_start = pd.Timestamp.now()
            res = vq.updateBatch(gyr, acc, mag)
            time_end = pd.Timestamp.now()
            time_diff = (time_end - time_start).total_seconds()
            print(f"vqf updateBatch time: {time_diff:.4f} seconds, tauAcc: {tauAcc}, tauMag: {tauMag}, useAccStep: {useAccStep}")
            result = res['quat9D']
        else:
            j_filter['filter'].initFromAccMag(dat['accelerometer'][0], dat['magnetometer'][0])
            result = eval_filter_on_dataset(j_filter['filter'], dat)
        
        alignIndex = int(dat['start_time']['index']*0.5)
        error_9D = angle_error(result, dat['reference'], align_start=True, shift_samples=0, align_index=alignIndex)
        
        print(f"{filter_name} Mean Error: {np.mean(error_9D):.2f} deg")
        if single_dataset:
            j_filter['errors'].append(error_9D)
        else:
            j_filter['errors'].append(np.mean(error_9D))

if single_dataset:
    for filter_name, j_filter in test_filters.items():
        plt.plot(j_filter['errors'][0], label=filter_name)
else:
    for filter_name, j_filter in test_filters.items():
        plt.plot(list(test_datasets.keys()), j_filter['errors'], label=filter_name)

plt.xlabel('Dataset')
plt.ylabel('Mean Error (deg)')
plt.legend()
plt.show()

for filter_name, j_filter in test_filters.items():
    print(f"{filter_name} Mean Error across datasets: {np.mean(j_filter['errors']):.2f} deg")