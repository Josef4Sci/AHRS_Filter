import time
import numpy as np
import matplotlib.pyplot as plt

import sys
import os

_python_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _python_dir)
sys.path.insert(1, os.path.join(_python_dir, 'vqf_local'))

from dataset_loader import DatasetLoader

from vqf import VQF
from utils import angle_error, get_optim_params_all, FilterType, get_color_sequence

RMSE = True
DISTURB_INCLUDED = True  # Set to True to include disturbed datasets, False for whitelisted datasets
base_fold = 'paper_j2\\figures\\'

test_datasets = {}
dataset_loader = DatasetLoader()

transparent = {'Madgwick'}
color_sequence = get_color_sequence()

if DISTURB_INCLUDED:
    black_list = dataset_loader.broad_black_under_thresh(0.2)
    for ind, file in enumerate(black_list):
        dataset = dataset_loader.load_broad_dataset(file, bypass_black_list=True, mean_initial_samples=True)
        if dataset is not None:
            test_datasets[file] = dataset

    test_filters = get_optim_params_all()
else:
    names = {'slow_v4.mat':'sassari_slow', 'medium_v4.mat':'sassari_medium', 'fast_v4.mat':'sassari_fast'}
    for i in range(1):
        for k, n in names.items():    
            dat = dataset_loader.load_sassari_dataset(k, i)
            test_datasets[n] = dat

    broad_white = dataset_loader.broad_white_list_datasets()
    for i in range(4):
        file = broad_white[i]
        dat = dataset_loader.load_broad_dataset(file_name=file, mean_initial_samples=True)
        if dat is not None:
            test_datasets['br_'+file] = dat
            
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

        # j_filter['filter'].
        gyr = np.ascontiguousarray(dat['gyroscope'], dtype=np.float64)
        acc = np.ascontiguousarray(dat['accelerometer'], dtype=np.float64)
        mag = np.ascontiguousarray(dat['magnetometer'], dtype=np.float64)
        
        vqf_full = j_filter['filter_type'] == FilterType.FILTER_VQF
        start_time = time.time()
        vq = VQF(1.0/dat['mean_sampling_rate'], 
                    tauAcc=j_filter['filter']['tauAcc'], tauMag=j_filter['filter']['tauMag'],
                    motionBiasEstEnabled=vqf_full, restBiasEstEnabled=vqf_full,
                    magDistRejectionEnabled=vqf_full, filterType=int(j_filter['filter_type']))

        res = vq.updateBatch(gyr, acc, mag)
        result = res['quat9D']
        
        alignIndex = int(dat['start_time']['index']*0.5)
        angle_err = angle_error(result, dat['reference'], align_start=True, shift_samples=0, align_index=alignIndex, use_imu=False)
        
        angle_err = angle_err[start_index:stop_index]   
                
        print(f"{filter_name} Mean Error {'RMSE' if RMSE else 'MAE'}: {np.nanmean(angle_err):.2f} deg")
        if single_dataset:
            if RMSE:
                j_filter['errors'].append(angle_err**2)
            else:
                j_filter['errors'].append(angle_err)
        else:   
            if RMSE:
                angle_err = np.sqrt(np.nanmean(angle_err**2))
            j_filter['errors'].append(np.nanmean(angle_err))

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
    import re

    def clean_label(name, max_line=16):
        name = re.sub(r'^\d+_', '', name)   # strip leading "02_"
        name = re.sub(r'\.\w+$', '', name)   # strip extension ".mat0"
        if len(name) > max_line:
            mid = len(name) // 2
            left = name.rfind('_', 0, mid)
            right = name.find('_', mid)
            if left == -1 and right == -1:
                pass
            elif left == -1:
                split_at = right
            elif right == -1:
                split_at = left
            else:
                split_at = left if (mid - left) <= (right - mid) else right
            name = name[:split_at] + '\n' + name[split_at + 1:]
        return name

    linestyles = ['-', '--', '-.', ':']
    markers = ['o', 's', '^', 'D']
    dataset_keys = list(test_datasets.keys())
    clean_keys = [clean_label(k) for k in dataset_keys]

    if DISTURB_INCLUDED:
        fig, ax = plt.subplots(figsize=(8, 5))
        fig.suptitle('Filter Performance Across Disturbed Takes (BROAD fixed)')
        
        for i, (filter_name, j_filter) in enumerate(test_filters.items()):
            ls = linestyles[i % len(linestyles)]
            mk = markers[i % len(markers)]
            color = color_sequence[i % len(color_sequence)]
            ax.plot(range(len(dataset_keys)), j_filter['errors'], label=filter_name,
                    linestyle=ls, marker=mk, color=color, markersize=4, alpha=0.3 if filter_name in transparent else 1.0)
    else:
        fig, ax = plt.subplots(figsize=(6, 5))
        fig.suptitle('Filter Performance Across Undisturbed Takes')
        
        for i, (filter_name, j_filter) in enumerate(test_filters.items()):
            ls = linestyles[i % len(linestyles)]
            mk = markers[i % len(markers)]
            color = color_sequence[i % len(color_sequence)]
            ax.plot(range(len(dataset_keys)), j_filter['errors'], label=filter_name,
                    linestyle=ls, marker=mk, color=color, markersize=4, alpha=0.3 if filter_name in transparent else 1.0)

    ax.set_ylabel('RMSE (deg)')
    ax.set_xticks(range(len(dataset_keys)))
    ax.set_xticklabels(clean_keys, rotation=90, ha='center')
    
    plt.subplots_adjust(bottom=0.4)
    plt.legend()
    filename = 'multi_arena_comparison.png' if DISTURB_INCLUDED else 'multi_arena_comparison_whitelist.png'
    plt.savefig(base_fold + filename, dpi=300)
    plt.show()

print("\nMean Errors Across Datasets:")
for filter_name, j_filter in test_filters.items():
    print(f"{filter_name} Mean: {np.mean(j_filter['errors']):.2f} deg")


