import time
from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt
from vqf import PyVQF
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

wl = dl.broad_white_list_datasets()
dat = dl.load_broad_dataset(wl[1])
j_filter = JustaAHRSv4(w_acc=0.00068, w_mag=0.00044)

j_filter.initFromAccMag(dat['accelerometer'][0], dat['magnetometer'][0]) 
quaternion_result = eval_filter_on_dataset(j_filter, dat)

#plot_dataset(dat)

diff = angle_error(quaternion_result, dat['reference'], align_start=False, shift_samples=0)
plt.plot(diff, label='Reference Norm')
plt.show()