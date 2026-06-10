import time
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import sys
import os

_python_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _python_dir)
sys.path.insert(1, os.path.join(_python_dir, 'vqf_local', 'vqf'))

import pyximport
pyximport.install(setup_args={"include_dirs": []}, language_level=3)

from vqf import VQF

from dataset_loader import DatasetLoader
from utils import angle_error

test_datasets = {}
dl = DatasetLoader()

FIX_BIAS = True
        
black_list = dl.black_list_error_jump
ind = 14
file = black_list[ind]
bl_threshold = [1.2, 1.2, 1.8, -1, 1.2, 1.2, 1.3, 1.2, 1.2, 2.3, 2.0, 2.0, -1.0, 2.2, 2.2]
dataset = dl.load_broad_dataset(file, bypass_black_list=True, skip_jump_fix=True)
test_datasets['01_undisturbed_slow_rotation_A.mat'] = dataset

diff = angle_error(dataset['reference'], dataset['reference'], align_start=False, shift_samples=1)
diff_large = diff > bl_threshold[ind]
# plt.plot(diff)
# plt.show()

# if diff_large is true, make true for consequentive N samples
# N=1000
# diff_large = np.convolve(diff_large, np.ones(N, dtype=bool), mode='same') > 0
# #shift half of N to the right, so that the large diff is marked from the start of the jump
# diff_large = np.roll(diff_large, N//2)
# diff_large = np.concatenate((diff_large, np.zeros(1, dtype=bool)))
# dataset['reference'][diff_large] = np.NAN
# test_datasets[file] = dataset

test_filters = { # 9DOF
'vqf': {'filter': {'tauAcc': 1.258081, 'tauMag': 9.9}, 'errors': [], 'type': 1, 'justa': False},
#'justa_cpp': {'filter': {'tauAcc': 2.295586, 'tauMag': 0.046651}, 'errors': [], 'type': 1, 'justa': True},
}

dat_cp = dataset.copy()
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
                    magDistRejectionEnabled=False, useJustaFilter=j_filter['justa'])
                    
        res = vq.updateBatch(gyr, acc, mag)
        result = res['quat9D']

alignIndex = int(dat_cp['start_time']['index']*0.5)
error_timeserie = angle_error(result, dat_cp['reference'], align_start=True, shift_samples=0, align_index=alignIndex)


fig, axs = plt.subplots(2, 1, sharex=True, constrained_layout=True)
#set fig size
fig.set_size_inches(8, 5)

fig.suptitle(f"Jumping reference issue example in Broad dataset {list(test_datasets.keys())[0]}")
fig.supxlabel('Time (s)')

# interval = [22000, 24000]
interval = [0, -1]
for filter_name, j_filter in test_filters.items():
    # subplot errors and 
    axs[0].plot(dat_cp['time'][interval[0]:interval[1]], error_timeserie[interval[0]:interval[1]], label='VQF error against reference')
    axs[0].set_ylabel('Error (deg)')

    
axs[0].legend()
axs[1].plot(dat_cp['time'][interval[0]:interval[1]-1], diff[interval[0]:interval[1]], label='angle difference between reference at t and t+1')
axs[1].set_ylabel('Diff angle (deg)')
axs[1].legend()    

base_fold = 'paper_j2\\figures\\'
plt.savefig(base_fold + 'broad_jump_issue.png', dpi=300)
