import time
from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt
from vqf import PyVQF
import numpy as np
import pandas as pd
from filters.justa_ahrs import JustaAHRSInvFast, JustaAHRSInv, JustaAHRSPure, JustaAHRSv2, JustaAHRSv3, JustaAHRSbezier
from utils import angle_error, eval_filter_on_dataset, plot_dataset, qdiff, angle_diff_deg
import pickle
from quaternion_library import quatern_prod, quatern_conj
plot_result = True

dl = DatasetLoader()
# dataset_name = 'slow_v4.mat'
# dat = dl.load_sassari_dataset(dataset_name, 2)

test_datasets = {}
dl = DatasetLoader()
#dat = dl.load_broad_dataset(file_name='07_undisturbed_fast_rotation_B.mat', mean_initial_samples=True)

dat = dl.load_justa_raw(1)

j_filter = JustaAHRSbezier()
j_filter = JustaAHRSPure(w_acc=0.2, w_mag=0.2)

j_filter.initFromAccMag(dat['accelerometer'][0], dat['magnetometer'][0]) 


quaternion_result = eval_filter_on_dataset(j_filter, dat)
diff = angle_error(quaternion_result, dat['reference'], align_start=True, shift_samples=0, align_index=200)
print(f"Mean angle error: {np.mean(diff):.4f} deg")

plt.plot(diff, label='Reference Norm')
plt.plot(j_filter.out_mod, label='Modif')
plt.show()