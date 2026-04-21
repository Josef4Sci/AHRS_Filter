from dataset_loader import DatasetLoader
from utils import plot_dataset, angle_error, qdiff_single, quatern_conj_single, quatern_prod_single
import matplotlib.pyplot as plt
import numpy as np

loader = DatasetLoader()
black_list = loader.black_list_error_jump
file = black_list[0]
dataset = loader.load_broad_dataset(file, bypass_black_list=True)


print(f"Loaded dataset: {file}")
#plot_dataset(dataset)

diff = angle_error(dataset['reference'], dataset['reference'], align_start=False, shift_samples=1)
diff_large = diff > 1.2


# if diff_large is true, make true for consequentive N samples
N=100
diff_large = np.convolve(diff_large, np.ones(N, dtype=bool), mode='same') > 0
#shift half of N to the right, so that the large diff is marked from the start of the jump
diff_large = np.roll(diff_large, N//2)
diff_large = np.concatenate((diff_large, np.zeros(1, dtype=bool)))
dataset['reference'][diff_large] = np.NAN

# apply corrections cumulatively
corrected_quat = dataset['reference'].copy()
corrected_quat[diff_large] = np.NAN
    
plt.plot(dataset['time'][1:], diff)
plt.plot(dataset['time'], corrected_quat[:,0])
plt.plot(dataset['time'], diff_large)
# plt.plot(dataset['time'], dataset['reference'][:,0])
plt.show()