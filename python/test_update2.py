
import vqf
from dataset_loader import DatasetLoader
import scipy.io
from quaternion_library import quatern_prod, quatern_conj
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from utils import angle_error, eval_filter_on_dataset
from filters import (JustaAHRSv2, JustaAHRSPure)
import time

mat = scipy.io.loadmat('Datasets/Synthetic2.mat')

n ='01_undisturbed_slow_rotation_A.mat'
n ='23_undisturbed_fast_combined_360s.mat'
dl= DatasetLoader()
dat = dl.load_broad_dataset(n)

b = vqf.BasicVQF(1.0/dat['mean_sampling_rate'])

res_g = []
for i in range(len(dat['gyroscope'])):
    b.updateGyr(dat['gyroscope'][i])
    res_g.append(b.getQuat3D())
    
#res=b.updateBatch(dat['gyroscope'], dat['accelerometer'], dat['magnetometer'])

error_9D = angle_error(res_g, dat['reference'])
print(f'VQF 9D Mean Error: {np.mean(error_9D)} deg')

# dat_j = dl.load_dataset('Justa')


j_filter = JustaAHRSPure(quaternion=dat['reference'][0], w_acc=0.05, w_mag=0.02)


time_start=time.time()
quaternion_result, angle_err = eval_filter_on_dataset(j_filter, dat, use_imu=False)
time_end=time.time()


print(f'JustaAHRS Pure Mean Error: {np.mean(angle_err)} deg')

q_diff = quatern_prod(quatern_conj(dat['reference']), quaternion_result)

plt.figure()
plt.plot(dat['time'], angle_err, label='error angle')
plt.plot(dat['time'], error_9D, label='error angle')
# plt.plot(dat['time'], q_diff, label=['w', 'x', 'y', 'z'])
#plt.plot(dat['time'], j_filter.bias_history, label=['x', 'y', 'z'])
plt.legend()
plt.show()



