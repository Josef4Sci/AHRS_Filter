from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt
import vqf
import numpy as np
import pandas as pd
from filters.justa_ahrs import JustaAHRSInvFast, JustaAHRSInv, JustaAHRSPure
from utils import angle_error, eval_filter_on_dataset, plot_dataset

dl = DatasetLoader()
dataset_name = 'slow_v4.mat'
dataset_name = 'medium_v4.mat'
dataset_name = 'fast_v4.mat'
dat = dl.load_sassari_dataset(dataset_name, 2)

b = vqf.BasicVQF(1.0/dat['mean_sampling_rate'], tauAcc=0.994, tauMag=1.44)
#b.state['gyrQuat'] = dat['reference'][0]


j_filter = JustaAHRSInvFast( w_acc=0.00034, w_mag=0.00022)
#j_filter = JustaAHRSPure(w_acc=0.00068, w_mag=0.00044)

bias = dat['gyroscope'][:500].mean(axis=0)
dat['gyroscope']=dat['gyroscope']*np.array([1.015, 1.015, 1.01]) - bias

# res_g = []
# for i in range(len(dat['gyroscope'])):
#     b.updateGyr(dat['gyroscope'][i])
#     res_g.append(b.getQuat3D())
# res_g = np.array(res_g)

res = b.updateBatch(dat['gyroscope'], dat['accelerometer'], dat['magnetometer'])
#res_g = res['quat9D']

j_filter.initFromAccMag(dat['accelerometer'][0], dat['magnetometer'][0]) # 
#print(f'Initial: {j_filter.quaternion}')
quaternion_result = eval_filter_on_dataset(j_filter, dat)

shift = 1
window = 100

skip_start_for_comparison = 5000

# get index near start and stop
start = dat['time'] < dat['interest_range'][0]
stop = dat['time'] > dat['interest_range'][1]
start_index = np.where(start)[0][-1] + 1
stop_index = np.where(stop)[0][0] - 1

error_9D = angle_error(quaternion_result, dat['reference'], align_start=True, shift_samples=shift, align_index=100)
error_9D_vqf = angle_error(res['quat9D'], dat['reference'], align_start=True, shift_samples=shift, align_index=100)

error_9D = error_9D[start_index:stop_index]
error_9D_vqf = error_9D_vqf[start_index:stop_index]
time_plot = dat['time'][start_index:stop_index]

print(dataset_name)
diff_error_vqf = (pd.Series(error_9D_vqf) - pd.Series(error_9D_vqf).rolling(window).mean()).to_numpy()
print(f'Mean error vqf: {np.mean(error_9D_vqf[skip_start_for_comparison:]):.2f} deg')
print(f'Mean diff error vqf: {np.mean(np.abs(diff_error_vqf[window+skip_start_for_comparison:])):.4f} deg')

diff_error = (pd.Series(error_9D) - pd.Series(error_9D).rolling(window).mean()).to_numpy()
print(f'Mean error: {np.mean(error_9D[skip_start_for_comparison:]):.2f} deg')
print(f'Mean diff error: {np.mean(np.abs(diff_error[window+skip_start_for_comparison:])):.4f} deg')
plt.plot(time_plot, diff_error, label='Diff Error')
plt.plot(time_plot, error_9D, label='J error')
plt.plot(time_plot, error_9D_vqf, label='VQF error')
plt.legend()
plt.show()