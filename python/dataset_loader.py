"""
Dataset loader for AHRS filter testing
Reads CSV datasets and provides data in appropriate format
"""
import pandas as pd
import numpy as np
import os
import scipy.io
from quaternion_library import quatern_prod, quatern_conj


class DatasetLoader:
    """
    Load and manage AHRS datasets from CSV files
    """
    
    def __init__(self, base_path='Datasets', base_path_broad ='Datasets/broad/data_mat', base_path_sassari ='Datasets/mimu_optical_dataset_caruso_sassari-5.0'):
        self.base_path = base_path
        self.base_path_broad = base_path_broad
        self.base_path_sassari = base_path_sassari
        self.datasets = {}
        self.ms2g = 9.80665  # Conversion factor from m/s^2 to g
        self.black_list_error_jump = ['01_undisturbed_slow_rotation_A.mat', '04_undisturbed_slow_rotation_with_breaks_A.mat', '06_undisturbed_fast_rotation_A.mat', '08_undisturbed_fast_rotation_with_breaks_A.mat', '13_undisturbed_slow_translation_with_breaks_A.mat', '15_undisturbed_fast_translation_A.mat', '17_undisturbed_fast_translation_with_breaks_A.mat', '19_undisturbed_slow_combined_240s.mat', '20_undisturbed_slow_combined_360s.mat', '21_undisturbed_fast_combined.mat', '22_undisturbed_fast_combined_240s.mat', '23_undisturbed_fast_combined_360s.mat', '28_disturbed_stationary_magnet_A.mat', '29_disturbed_stationary_magnet_B.mat', '30_disturbed_stationary_magnet_C.mat', '31_disturbed_stationary_magnet_D.mat', '34_disturbed_attached_magnet_3cm.mat', '35_disturbed_attached_magnet_4cm.mat', '36_disturbed_attached_magnet_5cm.mat', '37_disturbed_office_A.mat', '38_disturbed_office_B.mat', '39_disturbed_mixed.mat']
        
    
    def broad_white_list_datasets(self):
        return ['02_undisturbed_slow_rotation_B.mat', '03_undisturbed_slow_rotation_C.mat', '05_undisturbed_slow_rotation_with_breaks_B.mat', '07_undisturbed_fast_rotation_B.mat', '09_undisturbed_fast_rotation_with_breaks_B.mat', '10_undisturbed_slow_translation_A.mat', '11_undisturbed_slow_translation_B.mat', '12_undisturbed_slow_translation_C.mat', '14_undisturbed_slow_translation_with_breaks_B.mat', '16_undisturbed_fast_translation_B.mat', '18_undisturbed_fast_translation_with_breaks_B.mat', '24_disturbed_tapping_A.mat', '25_disturbed_tapping_B.mat', '26_disturbed_phone_vibration_A.mat', '27_disturbed_phone_vibration_B.mat', '32_disturbed_attached_magnet_1cm.mat', '33_disturbed_attached_magnet_2cm.mat']
    
    def load_broad_dataset(self, file_name):   

        if file_name in self.black_list_error_jump:
            return None

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
       

    def load_sassari_dataset(self, file_name, unit_n):   

        units = ['AP1', 'AP2', 'SH1', 'SH2', 'XS1', 'XS2']

        mat = scipy.io.loadmat(os.path.join(self.base_path_sassari, file_name))
        
        return self.load_sassari_unit(mat, units[unit_n])
    
    def load_sassari_unit(self, mat, unit_name):
        
        ref_key = 'Qs'
        quat_glob = mat[ref_key]

        unit_raw = mat[unit_name]
        raw_time = unit_raw[:,0]
        smoth_time = np.array(pd.Series(raw_time).rolling(200, center=True).mean())

        acc = unit_raw[:,1:4]
        gyr = unit_raw[:,4:7]
        mag = unit_raw[:,7:10]
        # quat_local = unit_raw[:,10:14]

        dt = np.mean(np.diff(raw_time))
        sr = 1/dt
        valid_data = ~np.isnan(smoth_time)

        data = {
            'time': smoth_time[valid_data],
            'gyroscope': gyr[valid_data,:],
            'accelerometer': acc[valid_data,:]/self.ms2g,
            'magnetometer': mag[valid_data,:],
            'reference': quat_glob[valid_data,:],
            'mean_sampling_rate': sr
        }
        
        return data
    
        # file_path = os.path.join(self.base_path_sassari, file_map[unit_name])
        # return self.load_sassari_dataset(file_path, unit_name)

        
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
