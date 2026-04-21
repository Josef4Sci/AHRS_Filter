import time
from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt

import numpy as np
import pandas as pd
from filters.justa_ahrs import JustaAHRSInvFast, JustaAHRSInv, JustaAHRSPure, JustaAHRSv3
from utils import angle_error, eval_filter_on_dataset


import sys
sys.path.append('python\\vqf_local\\vqf')   # folder containing vqf.pyx and vqf.pyxbld
import pyximport
pyximport.install(setup_args={"include_dirs": []}, language_level=3)
from vqf import VQF as BasicVQF

plot_result = True

dl = DatasetLoader()
dat = dl.load_dataset('Justa')

dat['gyroscope'] = dat['gyroscope']*np.array([0.977, 1.0, 1.0])
gn = np.linalg.norm(dat['gyroscope'], axis=1)
thresh_integ = 1.0
scale = 0.002
off = 0.5
integ = np.zeros(len(gn))
for i in range(1, len(gn)):
    new_value = integ[i-1] + scale * (gn[i] - thresh_integ)
    new_value = min(new_value, 1.0)  # Cap the value at 1.0
    new_value = max(new_value, 0.0)  # Ensure the value does not go below 0.0
    integ[i] = new_value

a_gyr = 5.0
b_gyr = 2.0
base = 1.0 / (1 + np.exp(b_gyr))
a = 1 - base - 1 / (1 + np.exp(gn*a_gyr-b_gyr)) 


multip = 1-a + 0.1 #np.linspace(0.9, 0.9, len(dat['gyroscope']))
multip[5500:6500] = 0.2

j_filter_orig = JustaAHRSPure(w_acc=0.5, w_mag= 0.5)

vqf_basic = BasicVQF(1.0/dat['mean_sampling_rate'], tauAcc=0.0, tauMag=0.0, useAccStepWhole=True)
gyr = np.ascontiguousarray(dat['gyroscope'], dtype=np.float64)
acc = np.ascontiguousarray(dat['accelerometer'], dtype=np.float64)
mag = np.ascontiguousarray(dat['magnetometer'], dtype=np.float64)
res = vqf_basic.updateBatch(gyr, acc, mag)

j_filter_orig.initFromAccMag(dat['accelerometer'][0], dat['magnetometer'][0]) #
quaternion_result_orig = eval_filter_on_dataset(j_filter_orig, dat)

shift = 0
window = 500

skip_start_for_comparison = 10

error_9D_orig = angle_error(quaternion_result_orig, dat['reference'], align_start=True, shift_samples=shift)
angle_err = angle_error(res['quat9D'], dat['reference'], align_start=True, shift_samples=shift)

print(f'Mean error orig: {np.mean(error_9D_orig[skip_start_for_comparison:]):.2f} deg')
print(f'Mean vqf error: {np.mean(angle_err[skip_start_for_comparison:]):.2f} deg')
# print(f'Mean diff error: {np.mean(np.abs(diff_error[window+skip_start_for_comparison:])):.4f} deg')

window = 20
if plot_result:
    plt.plot(pd.Series(error_9D_orig).rolling(window).mean(), label='orig')
    plt.plot(pd.Series(angle_err).rolling(window).mean(), label='VQF error')
    plt.plot(multip, label='Multip')
    plt.legend()
    plt.show()