import time
from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt

import numpy as np
import pandas as pd
from static_detector import StaticDetector



plot_result = True
variance = True

dl = DatasetLoader()
dat = dl.load_dataset('Justa')

# names = ['slow_v4.mat', 'medium_v4.mat', 'fast_v4.mat']
# test_datasets = {}
# for i in range(1):
#     for n in names:    
#         dat = dl.load_sassari_dataset(n, i)
#         # bias = dat['gyroscope'][:500].mean(axis=0)
#         # dat['gyroscope']=dat['gyroscope']*np.array([1.015, 1.015, 1.01]) - bias
#         test_datasets[n+str(i)] = dat
        
# broad_white = dl.broad_white_list_datasets()
# for i in range(4):
#     file = broad_white[i]
#     dat = dl.load_broad_dataset(file_name=file, mean_initial_samples=True)
#     if dat is not None:
#         test_datasets[file] = dat
        
# dat = test_datasets['02_undisturbed_slow_rotation_B.mat']

multip= 0.2
static_detector = StaticDetector(acc_threshold=0.07, gyr_threshold=0.3, mag_threshold=0.1, window_size=5, block_forward_steps=200)

detection = static_detector.updateBatch(dat['accelerometer'], dat['gyroscope'], dat['magnetometer'])

static_detector.reset()  # Reset the detector to ensure variance calculations start fresh
if variance:
    variances_agm = static_detector.varianceBatch(dat['accelerometer'], dat['gyroscope'], dat['magnetometer'])
else:
    variances_agm = static_detector.updateBatchSeparate(dat['accelerometer'], dat['gyroscope'], dat['magnetometer'])

# clamp max variance for better visualization
variances_agm = np.clip(variances_agm, 0, 5)

plt.plot(dat['gyroscope'][:, 0]*0.2, label='gyro x')
plt.plot(variances_agm[0, :], label='acc')
plt.plot(variances_agm[1, :], label='gyr')
plt.plot(variances_agm[2, :], label='mag')
plt.plot(detection, label='detection')
plt.legend()
plt.show()