import time
from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt

import sys
sys.path.append('python\\vqf_local\\vqf')  # folder containing vqf.pyx and vqf.pyxbld
import pyximport
pyximport.install(setup_args={"include_dirs": []}, language_level=3)

from vqf import VQF as PyVQF
import numpy as np
import pandas as pd
from filters.justa_ahrs import JustaAHRSInvFast, JustaAHRSInv, JustaAHRSPure
from utils import FilterType, angle_error, eval_filter_on_dataset, get_optim_params_all

RMSE = True
DOF6 = False

test_datasets = {}
dataset_loader = DatasetLoader()

# sl = dataset_loader.load_justa_raw(0)
# fast = dataset_loader.load_justa_raw(1)
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

#test_datasets = {'fast_v4.mat0':test_datasets['fast_v4.mat0']} # for quick testing

if DOF6:
    test_filters = { # 6DOF
        # 'JustaAHRSpureLin': {'filter': JustaAHRSPure(w_acc=0.43, no_mag=True), 'errors': [], 'type': 0},   
        # 'JustaAHRSinv': {'filter': JustaAHRSInvFast(w_acc=0.392, no_mag=True), 'errors': [], 'type': 0},
        'vqf': {'filter': {'tauAcc': 1.2, 'tauMag': 7.0}, 'errors': [], 'type': 1},
        'justa_cpp': {'filter': None, 'errors': [], 'type': 1, 'par1': True},
    }
else:
    test_filters = get_optim_params_all()
    
single_dataset = len(test_datasets)==1

errors_filters = dict.fromkeys(test_filters.keys(), [])

for dataset_name, dataset in test_datasets.items():
    print(f"Evaluating dataset: {dataset_name}")
    
    dat = dataset.copy()
    
    sta ='start_time'
    ir = "interest_range"
    if dat.keys().__contains__(ir):
        start = dat['time'] < dat[ir][0]
        stop = dat['time'] > dat[ir][1]
        start_index = np.where(start)[0][-1] + 1
        stop_index = np.where(stop)[0][0] - 1
    elif dat.keys().__contains__(sta):
        start_index = int(dat[sta]['index'])
        stop_index = -1
    else:
        start_index = 200
        stop_index = -1
    
    alignIndex = int(start_index*0.5)
    
    bias = dat['gyroscope'][:start_index].mean(axis=0)
    dat['gyroscope'] = dat['gyroscope'] - bias
    
    for filter_name, j_filter in test_filters.items():       
        if j_filter['type'] == 1:  # vqf
            # j_filter['filter'].
            gyr = np.ascontiguousarray(dat['gyroscope'], dtype=np.float64)
            acc = np.ascontiguousarray(dat['accelerometer'], dtype=np.float64)
            mag = np.ascontiguousarray(dat['magnetometer'], dtype=np.float64)
            start_time = time.time()
            vq = PyVQF(1.0/dat['mean_sampling_rate'], 
                       tauAcc=j_filter['filter']['tauAcc'], tauMag=j_filter['filter']['tauMag'],
                       motionBiasEstEnabled=False, restBiasEstEnabled=False, magDistRejectionEnabled=False,
                       filterType=int(j_filter['filter_type']))
            if DOF6:
                res = vq.updateBatch(gyr, acc)
                result = res['quat6D']
            else:
                res = vq.updateBatch(gyr, acc, mag)
                result = res['quat9D']
        else:
            j_filter['filter'].initFromAccMag(dat['accelerometer'][0], dat['magnetometer'][0])
            result = eval_filter_on_dataset(j_filter['filter'], dat)
        
        alignIndex = int(dat['start_time']['index']*0.5)
        angle_err = angle_error(result, dat['reference'], align_start=True, shift_samples=0, align_index=alignIndex, use_imu=DOF6)
        
        angle_err = angle_err[start_index:stop_index]   
                
        print(f"{filter_name} Mean Error {'RMSE' if RMSE else 'MAE'}: {np.mean(angle_err):.2f} deg")
        if single_dataset:
            if RMSE:
                j_filter['errors'].append(angle_err**2)
            else:
                j_filter['errors'].append(angle_err)
        else:   
            if RMSE:
                angle_err = np.sqrt(np.mean(angle_err**2))
            j_filter['errors'].append(np.mean(angle_err))

if single_dataset:
    plt.figure()
    
    fig, axs = plt.subplots(4, 1, sharex=True, constrained_layout=True)
    
    # interval = [22000, 24000]
    interval = [dat['start_time']['index'], -1]
    for filter_name, j_filter in test_filters.items():
        # subplot errors and 
        axs[0].plot(j_filter['errors'][0], label=filter_name)
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
    plt.ylabel('RMSE (deg)')
    plt.xticks(rotation=45)  # Rotate dataset names for better visibility
    #make some space for x labels
    plt.subplots_adjust(bottom=0.3)
    plt.legend()
    plt.show()

for filter_name, j_filter in test_filters.items():
    print(f"{filter_name} Mean Error across datasets: {np.mean(j_filter['errors']):.2f} deg")


