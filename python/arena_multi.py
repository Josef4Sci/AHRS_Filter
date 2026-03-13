import time
from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt

from vqf import VQF as PyVQF
import numpy as np
import pandas as pd
from filters.justa_ahrs import JustaAHRSInvFast, JustaAHRSInv, JustaAHRSPure, JustaAHRSlp2
from utils import angle_error, eval_filter_on_dataset

RMSE = True

test_datasets = {}
dataset_loader = DatasetLoader()

sl = dataset_loader.load_justa_raw(0)
fast = dataset_loader.load_justa_raw(1)
# dist = dataset_loader.load_justa_raw(2)
# test_datasets['j_slow'] = sl
# test_datasets['j_fast'] = fast
# test_datasets['j_dist'] = dist

broad_white = dataset_loader.broad_white_list_datasets()
for i in range(4):
    file = broad_white[i]
    dat = dataset_loader.load_broad_dataset(file_name=file, mean_initial_samples=True)
    if dat is not None:
        test_datasets[file] = dat

dataset_loader = DatasetLoader()
names = ['slow_v4.mat', 'medium_v4.mat', 'fast_v4.mat']
for i in range(1):
    for n in names:    
        dat = dataset_loader.load_sassari_dataset(n, i)
        # bias = dat['gyroscope'][:500].mean(axis=0)
        # dat['gyroscope']=dat['gyroscope']*np.array([1.015, 1.015, 1.01]) - bias
        test_datasets[n+str(i)] = dat

test_filters = {
    #'JustaAHRSPureLpimplem+': {'filter': JustaAHRSlp2(w_acc=0.97287, w_mag=0.475, linMag=False, whole_mag=True, lp_stage=1), 'errors': [], 'type': 0},
    #'JustaAHRSlp2+': {'filter': JustaAHRSlp2(w_acc=0.9469, w_mag=0.13, linMag=False, whole_mag=False), 'errors': [], 'type': 0},
    #'JustaAHRSlp2wholeMag+': {'filter': JustaAHRSlp2(w_acc=0.9, w_mag=0.1, linMag=False, whole_mag=True), 'errors': [], 'type': 0},
    'JustaAHRSpureLin': {'filter': JustaAHRSPure(w_acc=0.63, w_mag=24.1, linMag=True, whole_mag=False), 'errors': [], 'type': 0},    
    'JustaAHRSpure': {'filter': JustaAHRSPure(w_acc=1.0, w_mag=3.44, linMag=False, whole_mag=False), 'errors': [], 'type': 0},    
    'JustaAHRSlp2lin+': {'filter': JustaAHRSlp2(w_acc=0.89, w_mag=0.49, linMag=True, whole_mag=False), 'errors': [], 'type': 0},
    #'JustaAHRSlp2linWhole+': {'filter': JustaAHRSlp2(w_acc=1.0777, w_mag=0.8, linMag=True, whole_mag=True), 'errors': [], 'type': 0},
    'vqf': {'filter': None, 'errors': [], 'type': 1},
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
            start_time = time.time()
            vq = PyVQF(1.0/dat['mean_sampling_rate'], tauAcc=1.1, tauMag=3.6, motionBiasEstEnabled=False, restBiasEstEnabled=False, magDistRejectionEnabled=False)

            res = vq.updateBatch(gyr, acc, mag)
            result = res['quat9D']
        else:
            j_filter['filter'].initFromAccMag(dat['accelerometer'][0], dat['magnetometer'][0])
            result = eval_filter_on_dataset(j_filter['filter'], dat)
        
        alignIndex = int(dat['start_time']['index']*0.5)
        error_9D = angle_error(result, dat['reference'], align_start=True, shift_samples=0, align_index=alignIndex)
        
        if RMSE:
            error_9D = np.sqrt(np.mean(error_9D**2))
        
        print(f"{filter_name} Mean Error {'RMSE' if RMSE else 'MAE'}: {np.mean(error_9D):.2f} deg")
        if single_dataset:
            j_filter['errors'].append(error_9D)
        else:
            j_filter['errors'].append(np.mean(error_9D))

if single_dataset:
    plt.figure()
    
    fig, axs = plt.subplots(4, 1, sharex=True, constrained_layout=True)
    
    # interval = [22000, 24000]
    interval = [0, -1]
    for filter_name, j_filter in test_filters.items():
        # subplot errors and 
        axs[0].plot(j_filter['errors'][0][interval[0]:interval[1]], label=filter_name)
    axs[0].legend()
    axs[1].plot(dat['gyroscope'][interval[0]:interval[1]])
    axs[1].legend(['x','y','z'])
    axs[2].plot(dat['accelerometer'][interval[0]:interval[1]])
    axs[2].legend(['x','y','z'])
    axs[3].plot(dat['magnetometer'][interval[0]:interval[1]])
    axs[3].legend(['x','y','z'])
    plt.show()
else:
    for filter_name, j_filter in test_filters.items():
        plt.plot(list(test_datasets.keys()), j_filter['errors'], label=filter_name)

    plt.xlabel('Dataset')
    plt.ylabel('Mean Error (deg)')
    plt.legend()
    plt.show()

for filter_name, j_filter in test_filters.items():
    print(f"{filter_name} Mean Error across datasets: {np.mean(j_filter['errors']):.2f} deg")


