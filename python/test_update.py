import time
from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt
from vqf.vqf.pyvqf import PyVQF
import numpy as np
import pandas as pd
from filters.justa_ahrs import JustaAHRSInvFast, JustaAHRSInv, JustaAHRSPure, JustaAHRSv2, JustaAHRSv3, JustaAHRSv4
from utils import angle_error, eval_filter_on_dataset, plot_dataset
import pickle
from quaternion_library import quatern_prod, quatern_conj
plot_result = True

dl = DatasetLoader()
# dataset_name = 'slow_v4.mat'
# dat = dl.load_sassari_dataset(dataset_name, 2)

dat0 = dl.load_justa_raw(0)
dat1 = dl.load_justa_raw(1)
dat2 = dl.load_justa_raw(2)

dat0_end_cut = -600

dt = dat0['time'][1] - dat0['time'][0]
tsh1 = dat0['time'][dat0_end_cut] + dt
ths2 = tsh1 + (dat1['time'][-1] - dat1['time'][0]) + dt

# join datasets
dat = {
    'time': np.concatenate((dat0['time'][:dat0_end_cut], dat1['time'] + tsh1, dat2['time'] + ths2)),
    'gyroscope': np.concatenate((dat0['gyroscope'][:dat0_end_cut], dat1['gyroscope'], dat2['gyroscope'])),
    'accelerometer': np.concatenate((dat0['accelerometer'][:dat0_end_cut], dat1['accelerometer'], dat2['accelerometer'])),
    'magnetometer': np.concatenate((dat0['magnetometer'][:dat0_end_cut], dat1['magnetometer'], dat2['magnetometer'])),
    'reference': np.concatenate((dat0['reference'][:dat0_end_cut], dat1['reference'], dat2['reference'])),
    'mean_sampling_rate': (dat0['mean_sampling_rate'] + dat1['mean_sampling_rate'] + dat2['mean_sampling_rate']) / 3
}



plot_dataset(dat)
#plot_dataset(dat1)
# stack_reference = np.vstack((dat0['reference'], dat1['reference']))

# stack_reference_shifted = np.roll(stack_reference, -1, axis=0)  # Shift by one sample
# stack_reference_shifted[0] = stack_reference[0]  # Set the first sample to the original first sample

# diff = angle_error(stack_reference, stack_reference_shifted, align_start=False, shift_samples=0)

# # norm_q = np.abs(np.diff(stack_reference[:,0], axis=0))
# plt.plot(diff, label='Reference Norm')
# plt.show()