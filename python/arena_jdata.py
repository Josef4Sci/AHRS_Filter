import time
from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt
import vqf
import numpy as np
import pandas as pd
from filters.justa_ahrs import JustaAHRSInvFast, JustaAHRSInv, JustaAHRSPure
from utils import angle_error, eval_filter_on_dataset

dl = DatasetLoader()
# dataset_name = 'slow_v4.mat'
# dat = dl.load_sassari_dataset(dataset_name, 2)

dat = dl.load_dataset('Justa')

b = vqf.BasicVQF(1.0/dat['mean_sampling_rate'], tauAcc=0.994, tauMag=1.44)
#b.state['gyrQuat'] = dat['reference'][0]


#j_filter = JustaAHRSInvFast( w_acc=0.00034, w_mag=0.00022)
j_filter = JustaAHRSPure(w_acc=0.00024, w_mag=0.00022)
j_filter = JustaAHRSInvFast( w_acc=0.0005, w_mag=0.00044)

# res_g = []
# for i in range(len(dat['gyroscope'])):
#     b.updateGyr(dat['gyroscope'][i])
#     res_g.append(b.getQuat3D())
# res_g = np.array(res_g)

gyr = np.ascontiguousarray(dat['gyroscope'], dtype=np.float64)
acc = np.ascontiguousarray(dat['accelerometer'], dtype=np.float64)
mag = np.ascontiguousarray(dat['magnetometer'], dtype=np.float64)
start_time = time.time()
res = b.updateBatch(gyr, acc, mag)
end_time = time.time()
print(f"Batch update took {end_time - start_time:.4f} seconds")
#res_g = res['quat9D']

j_filter.initFromAccMag(dat['accelerometer'][0], dat['magnetometer'][0]) # 
#print(f'Initial: {j_filter.quaternion}')
start_time = time.time()
quaternion_result = eval_filter_on_dataset(j_filter, dat)
end_time = time.time()
print(f"Justa filter evaluation took {end_time - start_time:.4f} seconds")

shift = 1
window = 100

skip_start_for_comparison = 10

error_9D = angle_error(quaternion_result, dat['reference'], align_start=True, shift_samples=shift)
error_9D_vqf = angle_error(res['quat9D'], dat['reference'], align_start=True, shift_samples=shift)

diff_error_vqf = (pd.Series(error_9D_vqf) - pd.Series(error_9D_vqf).rolling(window).mean()).to_numpy()
print(f'Mean error vqf: {np.mean(error_9D_vqf[skip_start_for_comparison:]):.2f} deg')
print(f'Mean diff error vqf: {np.mean(np.abs(diff_error_vqf[window+skip_start_for_comparison:])):.4f} deg')

diff_error = (pd.Series(error_9D) - pd.Series(error_9D).rolling(window).mean()).to_numpy()
print(f'Mean error: {np.mean(error_9D[skip_start_for_comparison:]):.2f} deg')
print(f'Mean diff error: {np.mean(np.abs(diff_error[window+skip_start_for_comparison:])):.4f} deg')
plt.plot(dat['time'][shift:], diff_error, label='Diff Error')
plt.plot(dat['time'][shift:], error_9D, label='J error')
plt.plot(dat['time'][shift:], error_9D_vqf, label='VQF error')
plt.legend()
plt.show()