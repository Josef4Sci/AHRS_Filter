from dataset_loader import DatasetLoader
import matplotlib.pyplot as plt
import vqf
import numpy as np
import pandas as pd
from filters.justa_ahrs import JustaAHRSInvFast, JustaAHRSInv, JustaAHRSPure
from utils import angle_error, eval_filter_on_dataset, plot_dataset


    
dataset_loader = DatasetLoader()
sensor = 0
names = ['slow_v4.mat', 'medium_v4.mat', 'fast_v4.mat']
test_datasets={}
for i in range(4):
    for n in names:    
        dat = dataset_loader.load_sassari_dataset(n, i)
        # bias = dat['gyroscope'][:500].mean(axis=0)
        # dat['gyroscope']=dat['gyroscope']*np.array([1.015, 1.015, 1.01]) - bias
        test_datasets[n+str(i)] = dat

test_filters = {
    'JustaAHRSPure': {'filter': JustaAHRSPure(w_acc=0.99, w_mag=0.99), 'errors': [], 'type': 0},
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

