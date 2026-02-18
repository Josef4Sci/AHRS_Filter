import time
from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt
import vqf
import numpy as np
import pandas as pd
from filters.justa_ahrs import JustaAHRSInvFast, JustaAHRSInv, JustaAHRSPure, JustaAHRSv2
from utils import angle_error, eval_filter_on_dataset

test_datasets = {}
dl = DatasetLoader()

dataset_names = ['slow_v4.mat', 'medium_v4.mat', 'fast_v4.mat']
for dataset_name in dataset_names:
    for unit_n in range(4):
        dat = dl.load_sassari_dataset(dataset_name, unit_n)
        test_datasets[dataset_name + f"_unit_{unit_n}"] = dat
        
# broad_white = dl.broad_white_list_datasets()
# for file in broad_white:
#     dat = dl.load_broad_dataset(file_name=file)
#     if dat is not None:
#         test_datasets[file] = dat


# b = vqf.BasicVQF(1.0/dat['mean_sampling_rate'], tauAcc=0.994, tauMag=1.44)
#b.state['gyrQuat'] = dat['reference'][0]

test_filters = {
    'JustaAHRSInvFast': {'filter': JustaAHRSInvFast( w_acc=0.00034, w_mag=0.00022), 'errors': [], 'type': 0},
    'JustaAHRSv2': {'filter': JustaAHRSv2(w_acc=0.001349, w_mag=0.0003 , delay_steps=5), 'errors': [], 'type': 0},
    # 'JustaAHRSPure': {'filter': JustaAHRSPure(w_acc=0.0019, w_mag=0.0002), 'errors': [], 'type': 0},
    'vqf': {'filter': None, 'errors': [], 'type': 1},
}

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
            vq.set
            res = vq.updateBatch(gyr, acc, mag)
            error_9D_vqf = angle_error(res['quat9D'], dat['reference'], align_start=True, shift_samples=0)
            print(f"{filter_name} Mean Error: {np.mean(error_9D_vqf):.2f} deg")
            j_filter['errors'].append(np.mean(error_9D_vqf))
        else:
            j_filter['filter'].initFromAccMag(dat['accelerometer'][0], dat['magnetometer'][0]) #
            j_filter_result = eval_filter_on_dataset(j_filter['filter'], dat)
            error_9D = angle_error(j_filter_result, dat['reference'], align_start=True, shift_samples=0)
            print(f"{filter_name} Mean Error: {np.mean(error_9D):.2f} deg")
            j_filter['errors'].append(np.mean(error_9D))

for filter_name, j_filter in test_filters.items():
    plt.plot(list(test_datasets.keys()), j_filter['errors'], label=filter_name)

plt.xlabel('Dataset')
plt.ylabel('Mean Error (deg)')
plt.legend()
plt.show()

for filter_name, j_filter in test_filters.items():
    print(f"{filter_name} Mean Error across datasets: {np.mean(j_filter['errors']):.2f} deg")

