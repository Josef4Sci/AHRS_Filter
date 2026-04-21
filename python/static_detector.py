import numpy as np


class StaticDetector:
    def __init__(self, acc_threshold=0.1, gyr_threshold=0.015, mag_threshold=3.0, window_size=3, block_forward_steps=500):
        self.acc_threshold = acc_threshold
        self.gyr_threshold = gyr_threshold
        self.mag_threshold = mag_threshold
        self.window_size = window_size
        self.block_forward_steps = block_forward_steps
        self.acc_buffer = []
        self.gyr_buffer = []
        self.mag_buffer = []
        self.last_static_index = -block_forward_steps  # Initialize to allow detection at the start

    def update(self, acc, gyr, mag):
        self.acc_buffer.append(acc)
        self.gyr_buffer.append(gyr)
        self.mag_buffer.append(mag)

        if len(self.acc_buffer) > self.window_size:
            self.acc_buffer.pop(0)
            self.gyr_buffer.pop(0)
            self.mag_buffer.pop(0)

        if self.is_static():
            self.last_static_index = 0  # Reset the static index counter
            return True
        else:             # If we are within the block forward steps after the last detected static period, consider it static
            if self.last_static_index >= 0 and self.last_static_index < self.block_forward_steps:
                self.last_static_index += 1
                return True

        return False

    def updateBatch(self, acc_batch, gyr_batch, mag_batch):
        outputs = np.zeros(len(acc_batch), dtype=bool)
        for i, (acc, gyr, mag) in enumerate(zip(acc_batch, gyr_batch, mag_batch)):
            outputs[i] = self.update(acc, gyr, mag)
        return outputs

    def is_static(self):
        if len(self.acc_buffer) < self.window_size:
            return False

        gyr_var = np.linalg.norm(self.gyr_buffer, axis=1)
        acc_diff = np.diff(self.acc_buffer, axis=0)
        acc_var = np.linalg.norm(acc_diff, axis=1)
        mag_diff = np.diff(self.mag_buffer, axis=0)
        mag_var = np.linalg.norm(mag_diff, axis=1)

        mean_gyr_var = np.mean(gyr_var)
        mean_acc_var = np.mean(acc_var)
        mean_mag_var = np.mean(mag_var)

        is_static = mean_acc_var < self.acc_threshold and mean_gyr_var < self.gyr_threshold and mean_mag_var < self.mag_threshold 
        return is_static
        