import time
from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt
import vqf
import numpy as np
import pandas as pd
from filters.justa_ahrs import JustaAHRSInvFast, JustaAHRSInv, JustaAHRSPure, JustaAHRSv2, JustaAHRSv4
from utils import angle_error, eval_filter_on_dataset, plot_dataset

test_datasets = {}
dl = DatasetLoader()
        
# broad_white = dl.broad_white_list_datasets()
# for i in range(4):
#     file = broad_white[i]
#     dat = dl.load_broad_dataset(file_name=file, mean_initial_samples=True)
#     if dat is not None:
#         test_datasets[file] = dat


# test_datasets['03_undisturbed_slow_rotation_C.mat'] = dl.load_broad_dataset(file_name='03_undisturbed_slow_rotation_C.mat', mean_initial_samples=True)
test_datasets[ '07_undisturbed_fast_rotation_B.mat'] = dl.load_broad_dataset(file_name='07_undisturbed_fast_rotation_B.mat', mean_initial_samples=True)

test_filters = {
    'JustaAHRSv2': {'filter': JustaAHRSv2( w_acc=0.00034, w_mag=0.00022), 'errors': [], 'type': 0},
    'JustaAHRSv4': {'filter': JustaAHRSv4(w_acc=1, w_mag=1), 'errors': [], 'type': 0},
    'JustaAHRSPure': {'filter': JustaAHRSPure(w_acc=3.15, w_mag=2.15), 'errors': [], 'type': 0},
    'vqf': {'filter': None, 'errors': [], 'type': 1},
}

single_dataset = len(test_datasets)==1

errors_filters = dict.fromkeys(test_filters.keys(), [])

for dataset_name, dat in test_datasets.items():
    print(f"Evaluating dataset: {dataset_name}")

    for filter_name, j_filter in test_filters.items():       
        if j_filter['type'] == 1:  # vqf
            # j_filter['filter'].
            gyr = np.ascontiguousarray(dat['gyroscope'], dtype=np.float64)
            acc = np.ascontiguousarray(dat['accelerometer'], dtype=np.float64)
            mag = np.ascontiguousarray(dat['magnetometer'], dtype=np.float64)
            start_time = time.time()
            vq = vqf.BasicVQF(1.0/dat['mean_sampling_rate'], tauAcc=0.994, tauMag=1.44)
            vq.coeffs['gyrTs'] = 1.0/dat['mean_sampling_rate']            
            res = vq.updateBatch(gyr, acc, mag)
            result = res['quat9D']
        else:
            j_filter['filter'].initFromAccMag(dat['accelerometer'][0], dat['magnetometer'][0])
            result = eval_filter_on_dataset(j_filter['filter'], dat)
        
        alignIndex = int(dat['start_time']['index']*0.5)
        error_9D = angle_error(result, dat['reference'], align_start=True, shift_samples=0, align_index=alignIndex)
        
        print(f"{filter_name} Mean Error: {np.mean(error_9D):.2f} deg")
        if single_dataset:
            j_filter['errors'].append(error_9D)
        else:
            j_filter['errors'].append(np.mean(error_9D))

if single_dataset:
    for filter_name, j_filter in test_filters.items():
        plt.plot(j_filter['errors'][0], label=filter_name)
else:
    for filter_name, j_filter in test_filters.items():
        plt.plot(list(test_datasets.keys()), j_filter['errors'], label=filter_name)

plt.xlabel('Dataset')
plt.ylabel('Mean Error (deg)')
plt.legend()
plt.show()

for filter_name, j_filter in test_filters.items():
    print(f"{filter_name} Mean Error across datasets: {np.mean(j_filter['errors']):.2f} deg")

