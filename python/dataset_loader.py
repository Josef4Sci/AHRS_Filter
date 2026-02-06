"""
Dataset loader for AHRS filter testing
Reads CSV datasets and provides data in appropriate format
"""
import pandas as pd
import numpy as np
import os
import scipy.io


class DatasetLoader:
    """
    Load and manage AHRS datasets from CSV files
    """
    
    def __init__(self, base_path='Datasets', base_path_broad ='Datasets/broad/data_mat'):
        self.base_path = base_path
        self.base_path_broad = base_path_broad
        self.datasets = {}
        self.ms2g = 9.80665  # Conversion factor from m/s^2 to g
        
    def load_broad_dataset(self, file_name):   
        mat = scipy.io.loadmat(os.path.join(self.base_path_broad, file_name))
        
        quat = mat['opt_quat']
        # not nan
        valid_quat = ~np.isnan(quat).any(axis=1)
        length = np.sum(valid_quat)
        
        sr = mat['sampling_rate'].squeeze()
        dt = 1.0 / sr        
        timestamp = np.arange(0, length*dt, dt)[:length]
        data = {
            'time': timestamp,
            'gyroscope': mat[f'imu_gyr'][valid_quat,:],
            'accelerometer': mat[f'imu_acc'][valid_quat,:]/self.ms2g,
            'magnetometer': mat[f'imu_mag'][valid_quat,:],
            'reference': mat[f'opt_quat'][valid_quat,:],
            'mean_sampling_rate': sr
        }
        return data
       
    
       
        
    def load_dataset(self, dataset_name):
        """
        Load a specific dataset from CSV
        
        Args:
            dataset_name: Name of the dataset ('ALS', 'Justa', 'Synth2', 'Synth3')
            
        Returns:
            Dictionary containing time, sensor data, and reference quaternions
        """
        file_map = {
            'ALS': 'ALS_dataset.mat',
            'Justa': 'Justa_dataset.mat',
            'Synth2': 'Synthetic2.mat',
            'Synth3': 'Synthetic3.mat'
        }
        
        if dataset_name not in file_map:
            raise ValueError(f"Unknown dataset: {dataset_name}")
            
        file_path = os.path.join(self.base_path, file_map[dataset_name])
        
        
        mat = scipy.io.loadmat(file_path)
        
        timestamp = mat['time'].squeeze()
        mean_sampling_rate = 1.0 /  np.diff(timestamp).mean()
        
        # Extract data
        data = {
            'time': timestamp,
            'gyroscope': mat['Gyroscope'],
            'accelerometer': mat['Accelerometer'],
            'magnetometer': mat['Magnetometer'],
            'reference': mat['qViconReference'],
            'mean_sampling_rate': mean_sampling_rate
        }
        
        self.datasets[dataset_name] = data
        return data
        
    def load_all_datasets(self):
        """
        Load all available datasets
        
        Returns:
            Dictionary with all datasets
        """
        for name in ['ALS', 'Justa', 'Synth2', 'Synth3']:
            try:
                self.load_dataset(name)
            except Exception as e:
                print(f"Warning: Could not load {name}: {e}")
                
        return self.datasets
        
    def get_dataset_segment(self, dataset_name, start_idx=None, end_idx=None):
        """
        Get a segment of a dataset
        
        Args:
            dataset_name: Name of the dataset
            start_idx: Starting index (None = beginning)
            end_idx: Ending index (None = end)
            
        Returns:
            Dictionary containing the data segment
        """
        if dataset_name not in self.datasets:
            self.load_dataset(dataset_name)
            
        data = self.datasets[dataset_name]
        
        if start_idx is None:
            start_idx = 0
        if end_idx is None:
            end_idx = len(data['time'])
            
        segment = {
            'time': data['time'][start_idx:end_idx],
            'gyroscope': data['gyroscope'][start_idx:end_idx],
            'accelerometer': data['accelerometer'][start_idx:end_idx],
            'magnetometer': data['magnetometer'][start_idx:end_idx],
            'reference': data['reference'][start_idx:end_idx]
        }
        
        return segment
