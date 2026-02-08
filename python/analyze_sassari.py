from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt
import vqf
import numpy as np
import pandas as pd

from utils import angle_error

dl = DatasetLoader()
dat = dl.load_sassari_dataset('fast_v4.mat', 0)

b = vqf.BasicVQF(1.0/dat['mean_sampling_rate'])
#b.state['gyrQuat'] = dat['reference'][0]

bias = dat['gyroscope'][:500].mean(axis=0)

res_g = []
for i in range(len(dat['gyroscope'])):
    b.updateGyr(dat['gyroscope'][i]*np.array([1.015, 1.015, 1.01]) - bias)
    res_g.append(b.getQuat3D())
res_g = np.array(res_g)

gyro = np.zeros_like(dat['gyroscope'])


res = b.updateBatch(dat['gyroscope']*np.array([1.015, 1.015, 1.01]) - bias, dat['accelerometer'], dat['magnetometer'])
res_g = res['quat9D']
shift = 1
error_9D = angle_error(res_g, dat['reference'], align_start=True, shift_samples=shift)


diff_error = (pd.Series(error_9D) - pd.Series(error_9D).rolling(100).mean()).to_numpy()
print(f'Mean error: {np.mean(error_9D):.2f} deg')
print(f'Mean diff error: {np.mean(np.abs(diff_error[100:])):.4f} deg')
plt.plot(dat['time'][shift:], diff_error, label='Diff Error')
plt.plot(dat['time'][shift:], error_9D, label='Diff Error')
plt.show()