import numpy as np


class StaticDetector:
    def __init__(self, acc_threshold=0.1, gyr_threshold=0.015, mag_threshold=3.0, window_size=3, block_forward_steps=500):
        self.acc_threshold = acc_threshold
        self.gyr_threshold = gyr_threshold
        self.mag_threshold = mag_threshold
        print(f"Initialized StaticDetector with acc_threshold={acc_threshold}, gyr_threshold={gyr_threshold}, mag_threshold={mag_threshold}")
        self.window_size = window_size
        self.block_forward_steps = block_forward_steps
        self.acc_buffer = []
        self.gyr_buffer = []
        self.mag_buffer = []
        self.sample_index = -1
        self.last_motion_index = -block_forward_steps  # Track motion detections for blocking

    def reset(self):
        self.acc_buffer.clear()
        self.gyr_buffer.clear()
        self.mag_buffer.clear()
        self.sample_index = -1
        self.last_motion_index = -self.block_forward_steps

    def update(self, acc, gyr, mag):
        self.sample_index += 1
        self.acc_buffer.append(acc)
        self.gyr_buffer.append(gyr)
        self.mag_buffer.append(mag)

        if len(self.acc_buffer) > self.window_size:
            self.acc_buffer.pop(0)
            self.gyr_buffer.pop(0)
            self.mag_buffer.pop(0)
        
        # Update motion tracking after buffer is stable
        static_detected = self.is_static_gyr() and self.is_static_acc() and self.is_static_mag()
        if not static_detected:
            self.last_motion_index = self.sample_index


    def updateBatch(self, acc_batch, gyr_batch, mag_batch):
        outputs = self.updateBatchSeparate(acc_batch, gyr_batch, mag_batch)
        return np.all(outputs, axis=0)  # Combine acc, gyr, mag static flags into a single static flag per sample
    
    def updateBatchSeparate(self, acc_batch, gyr_batch, mag_batch):
        outputs = np.zeros((4, len(acc_batch)), dtype=bool)
        for i, (acc, gyr, mag) in enumerate(zip(acc_batch, gyr_batch, mag_batch)):
            self.update(acc, gyr, mag)            
            
            blocking = self.sample_index - self.last_motion_index < self.block_forward_steps
            sensor_states = [self.is_static_acc(), self.is_static_gyr(), self.is_static_mag(), not blocking]
            outputs[:, i] = sensor_states
            
        return outputs
    
    def varianceBatch(self, acc_batch, gyr_batch, mag_batch):
        outputs = np.zeros((3, len(acc_batch)), dtype=float)
        for i, (acc, gyr, mag) in enumerate(zip(acc_batch, gyr_batch, mag_batch)):
            self.update(acc, gyr, mag)            
            outputs[:, i] = [self.acc_variance()/self.acc_threshold, self.gyr_variance()/self.gyr_threshold, self.mag_variance()/self.mag_threshold]
        return outputs

    def gyr_variance(self):        
        gyr_var = np.linalg.norm(self.gyr_buffer, axis=1)
        mean_gyr_var = np.mean(gyr_var)
        return mean_gyr_var

    def acc_variance(self):
        if len(self.acc_buffer) < 2:
            return 0.0
        acc_diff = np.diff(self.acc_buffer, axis=0)
        acc_var = np.linalg.norm(acc_diff, axis=1)
        mean_acc_var = np.mean(acc_var)
        return mean_acc_var
    
    def mag_variance(self):
        if len(self.mag_buffer) < 2:
            return 0.0
        mag_diff = np.diff(self.mag_buffer, axis=0)
        mag_var = np.linalg.norm(mag_diff, axis=1)
        mean_mag_var = np.mean(mag_var)
        return mean_mag_var

    def _is_static_from(self, buffer, variance_fn, threshold):
        if len(buffer) < self.window_size:
            return False
        return variance_fn() < threshold

    def is_static_gyr(self):
        return self._is_static_from(self.gyr_buffer, self.gyr_variance, self.gyr_threshold)
    
    def is_static_acc(self):
        return self._is_static_from(self.acc_buffer, self.acc_variance, self.acc_threshold)

    def is_static_mag(self):
        return self._is_static_from(self.mag_buffer, self.mag_variance, self.mag_threshold)
    
    def is_static(self):
        # Block if we recently detected motion
        if self.sample_index - self.last_motion_index < self.block_forward_steps:
            return False
        
        return self.is_static_gyr() and self.is_static_acc() and self.is_static_mag()