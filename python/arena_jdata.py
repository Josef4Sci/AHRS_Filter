import time
from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt
from vqf.vqf.pyvqf import PyVQF
import numpy as np
import pandas as pd
from filters.justa_ahrs import JustaAHRSInvFast, JustaAHRSInv, JustaAHRSPure, JustaAHRSv2, JustaAHRSv3, JustaAHRSv4
from utils import angle_error, eval_filter_on_dataset, plot_dataset

plot_result = True

dl = DatasetLoader()
# dataset_name = 'slow_v4.mat'
# dat = dl.load_sassari_dataset(dataset_name, 2)

dat = dl.load_dataset('Justa')


b = PyVQF(1.0/dat['mean_sampling_rate'], tauAcc=1.68, tauMag=5.7, motionBiasEstEnabled=False, restBiasEstEnabled=False, magDistRejectionEnabled=False)
#b.state['gyrQuat'] = dat['reference'][0]

j_filter = JustaAHRSv4(w_acc=0.000137, w_mag= 0.0001)

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

# same_mag_test = np.tile(dat['magnetometer'][0], (len(dat['magnetometer']), 1))
# same_acc_test = np.tile(dat['accelerometer'][0], (len(dat['accelerometer']), 1))
# same_gyr_test = np.tile(np.zeros(3), (len(dat['accelerometer']), 1))
# dat['magnetometer'] = same_mag_test
# dat['accelerometer'] = same_acc_test   
# dat['gyroscope'] = same_gyr_test

#print(f'Initial: {j_filter.quaternion}')
start_time = time.time()
quaternion_result = eval_filter_on_dataset(j_filter, dat)
end_time = time.time()
print(f"Justa filter evaluation took {end_time - start_time:.4f} seconds")

shift = -1
window = 500

skip_start_for_comparison = 10

error_9D = angle_error(quaternion_result, dat['reference'], align_start=True, shift_samples=shift, align_index=-10)

# plt.plot(j_filter.coefs, label='Justa AHRS v3')
# plt.legend()
# plt.show()

error_9D_vqf = angle_error(res['quat9D'], dat['reference'], align_start=True, shift_samples=shift, align_index=-10)

diff_error_vqf = (pd.Series(error_9D_vqf) - pd.Series(error_9D_vqf).rolling(window).mean()).to_numpy()
print(f'Mean error vqf: {np.mean(error_9D_vqf[skip_start_for_comparison:]):.2f} deg')
print(f'Mean diff error vqf: {np.mean(np.abs(diff_error_vqf[window+skip_start_for_comparison:])):.4f} deg')

diff_error = (pd.Series(error_9D) - pd.Series(error_9D).rolling(window).mean()).to_numpy()
print(f'Mean error: {np.mean(error_9D[skip_start_for_comparison:]):.2f} deg')
print(f'Mean diff error: {np.mean(np.abs(diff_error[window+skip_start_for_comparison:])):.4f} deg')

window = 1
if plot_result:
    #plt.plot(pd.Series(np.abs(diff_error)).rolling(window).mean(), label='Diff Error')
    plt.plot(pd.Series(error_9D).rolling(window).mean() , label='J error')
    plt.plot(pd.Series(error_9D_vqf).rolling(window).mean() , label='VQF error')
    plt.legend()
    plt.show()