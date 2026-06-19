import time
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
from utils import angle_error, eval_filter_on_dataset, get_optim_params_all, get_color_sequence

test_datasets = {}
dl = DatasetLoader()

DOF6 = True
RMSE = True
FIX_BIAS = True

# broad_white = dl.broad_white_list_datasets()
# for i in range(4):
#     file = broad_white[i]
#     dat = dl.load_broad_dataset(file_name=file, mean_initial_samples=True)
#     if dat is not None:
#         test_datasets[file] = dat

# file = broad_white[1]
# dat = dl.load_broad_dataset(file_name=file, mean_initial_samples=True)
# if dat is not None:
#     test_datasets[file] = dat
# nan_ratio = {}
# black_list = dl.broad_black_under_thresh(0.2)

# print("Datasets with error jumps (black list):")
# for ind, file in enumerate(black_list):
            
#     file = black_list[ind]
#     dataset = dl.load_broad_dataset(file, bypass_black_list=True)
#     if dataset is None:
#         print(f"Dataset {file} skipped due to missing or invalid data.")
#         continue
    
#     nan_ratio_value = np.sum(np.isnan(dataset['reference'][:, 0])) / len(dataset['reference'][:, 0])
#     nan_ratio[file] = nan_ratio_value
#     print(f"Dataset: {file}, NaN ratio: {nan_ratio_value:.2%}")

#     # plt.figure()
#     # plt.plot(dataset['reference'], label='diff')
#     # plt.legend()
#     # plt.show()
# sys.exit()
# if diff_large is true, make true for consequentive N samples
# N=1000
# diff_large = np.convolve(diff_large, np.ones(N, dtype=bool), mode='same') > 0
# #shift half of N to the right, so that the large diff is marked from the start of the jump
# diff_large = np.roll(diff_large, N//2)
# diff_large = np.concatenate((diff_large, np.zeros(1, dtype=bool)))
# dataset['reference'][diff_large] = np.NAN
# test_datasets[file] = dataset

#test_datasets['02_undisturbed_slow_rotation_B.mat'] = dl.load_broad_dataset(file_name='02_undisturbed_slow_rotation_B.mat', mean_initial_samples=True)
# test_datasets['02_undisturbed_slow_rotation_B.mat'] = dl.load_broad_dataset(file_name='02_undisturbed_slow_rotation_B.mat', mean_initial_samples=True)
# test_datasets['03_undisturbed_slow_rotation_C.mat'] = dl.load_broad_dataset(file_name='03_undisturbed_slow_rotation_C.mat', mean_initial_samples=True)
#test_datasets[ '07_undisturbed_fast_rotation_B.mat'] = dl.load_broad_dataset(file_name='07_undisturbed_fast_rotation_B.mat', mean_initial_samples=True)

ind_sing = 4
black_list = dl.broad_black_under_thresh(0.2)
for ind, file in enumerate(black_list):   
    if ind_sing!=ind :
        continue
    dataset = dl.load_broad_dataset(file, bypass_black_list=True, mean_initial_samples=True)
 
    if dataset is not None:
        test_datasets[file] = dataset
        break

test_filters = get_optim_params_all()
single_dataset = len(test_datasets)==1

errors_filters = dict.fromkeys(test_filters.keys(), [])

for dataset_name, dat in test_datasets.items():
    print(f"Evaluating dataset: {dataset_name}")

    dat_cp = dat.copy()
    if FIX_BIAS:
        bias = dat_cp['gyroscope'][:dat_cp['start_time']['index']].mean(axis=0)
        dat_cp['gyroscope'] = dat_cp['gyroscope'] - bias
    
    for filter_name, j_filter in test_filters.items():       
        if j_filter['type'] == 1:  # vqf
            
            gyr = np.ascontiguousarray(dat_cp['gyroscope'], dtype=np.float64)
            acc = np.ascontiguousarray(dat_cp['accelerometer'], dtype=np.float64)
            mag = np.ascontiguousarray(dat_cp['magnetometer'], dtype=np.float64)
            start_time = time.time()
            vq = VQF(1.0/dat_cp['mean_sampling_rate'], tauAcc=j_filter['filter']['tauAcc'], tauMag=j_filter['filter']['tauMag'],
                     motionBiasEstEnabled=False, restBiasEstEnabled=False,
                     magDistRejectionEnabled=False, filterType=int(j_filter['filter_type']))
                      
            res = vq.updateBatch(gyr, acc, mag)
            result = res['quat9D']
        else:
            j_filter['filter'].initFromAccMag(dat_cp['accelerometer'][0], dat_cp['magnetometer'][0])
            result = eval_filter_on_dataset(j_filter['filter'], dat_cp['gyroscope'], dat_cp['accelerometer'], dat_cp['magnetometer'], dat_cp['mean_sampling_rate'])
        
        alignIndex = int(dat_cp['start_time']['index']*0.5)
        error_timeserie = angle_error(result, dat_cp['reference'], align_start=True, shift_samples=0, align_index=alignIndex, 
                                      yaw_only=False, use_imu=False)
        
        
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
    #plt.figure()
    
    # fig, axs = plt.subplots(2, 1, sharex=True, constrained_layout=True)
    
    # fig.suptitle(f"Jumping reference issue example in Broad dataset {list(test_datasets.keys())[0]}")
    # fig.supxlabel('Time (s)')

    # interval = [22000, 24000]
    colors = get_color_sequence()
    interval = [0, -1]
    i=0
    for filter_name, j_filter in test_filters.items():
        # subplot errors and 
        alb= 0.3 if 'Madgwick' in filter_name else 1.0
        plt.plot(dat['time'][interval[0]:interval[1]], j_filter['errors'][0][interval[0]:interval[1]],
                 label=filter_name, alpha=alb, color=colors[i])
        plt.ylabel('Error (deg)')
        i+=1

    plt.legend()
    # axs[0].legend()
    # axs[1].plot(dat['time'][interval[0]:interval[1]-1], diff[interval[0]:interval[1]], label='angle difference between reference at t and t+1')
    # axs[1].set_ylabel('Diff angle (deg)')
    # axs[1].legend()    


    # axs[1].plot(dat['gyroscope'][interval[0]:interval[1]])
    # axs[1].legend(['x','y','z'])
    # axs[2].plot(dat['accelerometer'][interval[0]:interval[1]])
    # axs[2].legend(['x','y','z'])
    # axs[3].plot(dat['magnetometer'][interval[0]:interval[1]])
    # axs[3].legend(['x','y','z'])
    plt.show()
else:
    for filter_name, j_filter in test_filters.items():
        plt.plot(list(test_datasets.keys()), j_filter['errors'], label=filter_name)

    plt.xlabel('Dataset')
    plt.ylabel('Mean Error (deg)')
    plt.legend()
    plt.show()

for filter_name, j_filter in test_filters.items():
    print(f"{filter_name} Mean Error across datasets: {np.nanmean(j_filter['errors']):.2f} deg")

