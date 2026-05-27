import numpy as np
from scipy import signal
from matplotlib import pyplot as plt
from dataset_loader import DatasetLoader

load_dataset = DatasetLoader()
dat = load_dataset.load_dataset('Justa')
gyro = dat['gyroscope']  # Use all gyroscope axes
fs = dat['mean_sampling_rate']  # Sampling frequency from the dataset
t = np.arange(len(gyro)) / fs  # Time vector based on the length of the data and sampling frequency
# Generate signal with high-frequency noise
# fs = 1000  # Sampling frequency (Hz)
# t = np.linspace(0, 1, fs)
# x = np.sin(2 * np.pi * 10 * t) + 0.5 * np.sin(2 * np.pi * 100 * t)

# Design Butterworth filter
order = 2
cutoff = 5  # Cutoff frequency (Hz)
nyquist = fs / 2
normalized_cutoff = cutoff / nyquist

# Get filter coefficients
b, a = signal.butter(order, normalized_cutoff, btype='low')

# Apply filter

# Iterate over time samples and filter all 3 axes together each step
zi = signal.lfilter_zi(b, a)[:, None] * np.ones(3) * 0.00001
y = np.zeros_like(gyro)
for i in range(gyro.shape[0]):
    y[i, :], zi = signal.lfilter(b, a, gyro[i, :][np.newaxis, :], axis=0, zi=zi)
    

# Plot results
plt.figure(figsize=(10, 4))
axis_labels = ['x', 'y', 'z']
for axis in range(gyro.shape[1]):
    plt.plot(t, gyro[:, axis], alpha=0.25, label=f'Noisy {axis_labels[axis]}')
    plt.plot(t, y[:, axis], linewidth=2, label=f'Filtered {axis_labels[axis]}')
plt.legend()
plt.grid(True)
plt.show()