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


RMSE = True
DOF6 = False

dataset_loader = DatasetLoader()

sl = dataset_loader.load_justa_raw(0)
fast = dataset_loader.load_justa_raw(1)
dist = dataset_loader.load_justa_raw(2)
datasets = [sl, fast, dist]

test_datasets={'slow': sl} #, 'fast': fast, 'dist': dist

#plot_dataset(test_datasets['slow'])

if DOF6:
    test_filters = { # 6DOF
        'vqf': {'filter': {'tauAcc': 1.1, 'tauMag': 0}, 'errors': [], 'type': 1},
        'justa_cpp': {'filter': {'tauAcc': 1.1, 'tauMag': 0}, 'errors': [], 'type': 1, 'par1': True},
    }
else:
    test_filters = { # 9DOF
    'vqf': {'filter': {'tauAcc': 0.5, 'tauMag': 0.5}, 'errors': [], 'type': 1, 'par1': False},
    'justa_cpp': {'filter': {'tauAcc': 0.7, 'tauMag': 0.3}, 'errors': [], 'type': 1, 'par1': True},
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
                       useAccStepWhole= filter['par1'], useMag=True, useMagStepWhole=True, useMagStepLinear=True, 
                       useJustaFilter=True, JustaFIlterVersionOld=True)
            if DOF6:
                res = vq.updateBatch(gyr, acc)
                result = res['quat6D']
            else:
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

