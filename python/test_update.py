import time
from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt
from vqf import PyVQF
import numpy as np
import pandas as pd
from filters.justa_ahrs import JustaAHRSInvFast, JustaAHRSInv, JustaAHRSPure, JustaAHRSv2, JustaAHRSv3, JustaAHRSv4
from utils import angle_error, eval_filter_on_dataset, plot_dataset, qdiff, angle_diff_deg
import pickle
from quaternion_library import quatern_prod, quatern_conj
plot_result = True

dl = DatasetLoader()
# dataset_name = 'slow_v4.mat'
# dat = dl.load_sassari_dataset(dataset_name, 2)

wl = dl.broad_white_list_datasets()
fil = wl[1]
print(f"Loading dataset: {fil}")
dat = dl.load_broad_dataset(file_name=fil)
j_filter = JustaAHRSv4(w_acc=0.00068, w_mag=0.00044)

j_filter.initFromAccMag(dat['accelerometer'][0], dat['magnetometer'][0]) 

qini = j_filter.quaternion 
print(f"Initial quaternion: {qini} from acc/mag: {angle_diff_deg(qdiff(qini, dat['reference'][0].reshape(1, -1)))} deg")



iters = 200
for i in range(iters):
    j_filter.update(
                    dat['gyroscope'][0],
                    dat['accelerometer'][0],
                    dat['magnetometer'][0],
                    0.01
                )
print(f"Quaternion after {iters} updates 0: {j_filter.quaternion}, diff {angle_diff_deg(qdiff(j_filter.quaternion, dat['reference'][0].reshape(1, -1)))} deg")

for i in range(iters):
    j_filter.update(
                    dat['gyroscope'][i],
                    dat['accelerometer'][i],
                    dat['magnetometer'][i],
                    0.01
                )
print(f"Quaternion after {iters} updates data: {j_filter.quaternion}, diff {angle_diff_deg(qdiff(j_filter.quaternion, dat['reference'][iters-1].reshape(1, -1)))} deg")

j_filter.initFromAccMag(np.mean(dat['accelerometer'][:300], axis=0), np.mean(dat['magnetometer'][:300], axis=0)) 
print(f"Initial quaternion with mean acc/mag: {j_filter.quaternion} from acc/mag: {angle_diff_deg(qdiff(j_filter.quaternion, dat['reference'][0].reshape(1, -1)))} deg")


print(f'Reference quaternion: {dat["reference"][0]}')

#plot_dataset(dat)

quaternion_result = eval_filter_on_dataset(j_filter, dat)
diff = angle_error(quaternion_result, dat['reference'], align_start=True, shift_samples=0, align_index=200)
plt.plot(diff, label='Reference Norm')
plt.show()