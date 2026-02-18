import numpy as np
from scipy import signal
from matplotlib import pyplot as plt
from dataset_loader import DatasetLoader

load_dataset = DatasetLoader()
dat = load_dataset.load_dataset('Justa')
x = dat['gyroscope'][:, 0]  # Use the x-axis gyroscope data as an example
fs = dat['mean_sampling_rate']  # Sampling frequency from the dataset
t = np.arange(len(x)) / fs  # Time vector based on the length of the data and sampling frequency
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
zi = signal.lfilter_zi(b, a) * x[0]  # Scale by first input value

# Allocate output array
y = np.zeros(len(x))

# Filter sample by sample
for i in range(len(x)):
    # Pass single sample and get updated state
    y[i], zi = signal.lfilter(b, a, [x[i]], zi=zi)

# Plot results
plt.figure(figsize=(10, 4))
plt.plot(t, x, 'b-', alpha=0.3, label='Noisy signal')
plt.plot(t, y, 'r-', linewidth=2, label='Filtered signal')
plt.legend()
plt.grid(True)
plt.show()