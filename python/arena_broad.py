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
from filters.justa_ahrs import JustaAHRSInvFast, JustaAHRSInv, JustaAHRSPure, JustaAHRSlp2
from utils import angle_error, eval_filter_on_dataset, plot_dataset

test_datasets = {}
dl = DatasetLoader()
        
# broad_white = dl.broad_white_list_datasets()
# for i in range(4):
#     file = broad_white[i]
#     dat = dl.load_broad_dataset(file_name=file, mean_initial_samples=True)
#     if dat is not None:
#         test_datasets[file] = dat

black_list = dl.black_list_error_jump
ind = 14
file = black_list[ind]
bl_threshold = [1.2, 1.2, 1.8, -1, 1.2, 1.2, 1.3, 1.2, 1.2, 2.3, 2.0, 2.0, -1.0, 2.2, 2.2]
dataset = dl.load_broad_dataset(file, bypass_black_list=True)

diff = angle_error(dataset['reference'], dataset['reference'], align_start=False, shift_samples=1)
diff_large = diff > bl_threshold[ind]


# if diff_large is true, make true for consequentive N samples
N=1000
diff_large = np.convolve(diff_large, np.ones(N, dtype=bool), mode='same') > 0
#shift half of N to the right, so that the large diff is marked from the start of the jump
diff_large = np.roll(diff_large, N//2)
diff_large = np.concatenate((diff_large, np.zeros(1, dtype=bool)))
dataset['reference'][diff_large] = np.NAN

test_datasets[file] = dataset

#test_datasets['03_undisturbed_slow_rotation_C.mat'] = dl.load_broad_dataset(file_name='03_undisturbed_slow_rotation_C.mat', mean_initial_samples=True)
#test_datasets[ '07_undisturbed_fast_rotation_B.mat'] = dl.load_broad_dataset(file_name='07_undisturbed_fast_rotation_B.mat', mean_initial_samples=True)

test_filters = {
    #'JustaAHRSv2': {'filter': JustaAHRSv2( w_acc=0.00034, w_mag=0.00022), 'errors': [], 'type': 0},
    #'JustaAHRSv4': {'filter': JustaAHRSv4(w_acc=1, w_mag=1), 'errors': [], 'type': 0},
    'JustaInvFast' : {'filter': JustaAHRSInvFast(w_acc=1, w_mag=1), 'errors': [], 'type': 0},
    #'JustaAHRSPureMagSetep': {'filter': JustaAHRSPure(w_acc=3.15, w_mag=2.15), 'errors': [], 'type': 0},
    # 'JustaAHRSPureMagLin': {'filter': JustaAHRSPure(w_acc=2, w_mag=30.15, linMag=True), 'errors': [], 'type': 0},
    # 'JustaAHRSlp2': {'filter': JustaAHRSlp2(w_acc=0.6, w_mag=1.0, linMag=False, whole_mag=False), 'errors': [], 'type': 0},
    # 'JustaAHRSlp2wholeMag': {'filter': JustaAHRSlp2(w_acc=0.6, w_mag=1.0, linMag=False, whole_mag=True), 'errors': [], 'type': 0},
    # 'JustaAHRSlp2lin': {'filter': JustaAHRSlp2(w_acc=0.6, w_mag=1.15, linMag=True), 'errors': [], 'type': 0},
    # 'JustaAHRSlp2linWhole': {'filter': JustaAHRSlp2(w_acc=0.6, w_mag=1.15, linMag=True, whole_mag=True), 'errors': [], 'type': 0},
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
            vq = VQF(1.0/dat['mean_sampling_rate'], tauAcc=0.994, tauMag=1.44, motionBiasEstEnabled=False, restBiasEstEnabled=False, magDistRejectionEnabled=False)
            vq.coeffs['gyrTs'] = 1.0/dat['mean_sampling_rate']            
            res = vq.updateBatch(gyr, acc, mag)
            result = res['quat9D']
        else:
            j_filter['filter'].initFromAccMag(dat['accelerometer'][0], dat['magnetometer'][0])
            result = eval_filter_on_dataset(j_filter['filter'], dat)
        
        alignIndex = int(dat['start_time']['index']*0.5)
        error_9D = angle_error(result, dat['reference'], align_start=True, shift_samples=0, align_index=alignIndex)
        
        print(f"{filter_name} Mean Error: {np.nanmean(error_9D):.2f} deg")
        if single_dataset:
            j_filter['errors'].append(error_9D)
        else:
            j_filter['errors'].append(np.nanmean(error_9D))

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
    print(f"{filter_name} Mean Error across datasets: {np.nanmean(j_filter['errors']):.2f} deg")

