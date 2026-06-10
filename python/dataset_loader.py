"""
Dataset loader for AHRS filter testing
Reads CSV datasets and provides data in appropriate format
"""
import pandas as pd
import numpy as np
import os
import scipy.io
import matplotlib.pyplot as plt
from utils import angle_error
from quaternion_library import quatern_prod, quatern_conj
from load_raw_justa import fix_coordinate_system, fix_magnet_alignment,\
    get_measurement_files, load_raw_justa, interpolate_vicon_to_imu, add_time_from_start, fix_negative_qw,\
    fix_ref_drops

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
        self.black_list_error_nan_ratio = {'01_undisturbed_slow_rotation_A.mat': 0.2186762478424742, '04_undisturbed_slow_rotation_with_breaks_A.mat': 0.25715604801477376, '06_undisturbed_fast_rotation_A.mat': 0.43626279559153974, '08_undisturbed_fast_rotation_with_breaks_A.mat': 0.9999818524970964, '13_undisturbed_slow_translation_with_breaks_A.mat': 0.04658960682503077, '15_undisturbed_fast_translation_A.mat': 0.32870211549456835, '17_undisturbed_fast_translation_with_breaks_A.mat': 0.3672212260078369, '19_undisturbed_slow_combined_240s.mat': 0.25412614550215074, '20_undisturbed_slow_combined_360s.mat': 0.016654176034640686, '21_undisturbed_fast_combined.mat': 0.5407503989413459, '22_undisturbed_fast_combined_240s.mat': 0.1286645140985441, '23_undisturbed_fast_combined_360s.mat': 0.17178936145401236, '28_disturbed_stationary_magnet_A.mat': 0.9999804626445764, '29_disturbed_stationary_magnet_B.mat': 0.6084755738669806, '30_disturbed_stationary_magnet_C.mat': 0.08876662834619806, '31_disturbed_stationary_magnet_D.mat': 0.1297805771684986, '34_disturbed_attached_magnet_3cm.mat': 0.044644850216527525, '35_disturbed_attached_magnet_4cm.mat': 0.09310120100549298, '36_disturbed_attached_magnet_5cm.mat': 0.04664505445810108, '37_disturbed_office_A.mat': 0.1104566027689031, '38_disturbed_office_B.mat': 0.059411330145028966, '39_disturbed_mixed.mat': 0.039620436221002794}
        thr = [1.2, 1.2, 1.8, -1, 1.2, 1.2, 1.3, 1.2, 1.2, 2.3, 2.0, 2.0, -1.0,\
                                    2.2, 3.2, 3.5, 1.25, 1.5, 1.9, 1.8, 1.7, 6.5]
        # make dict for error jump threshold
        self.black_thresholds = dict(zip(self.black_list_error_jump, thr)) 

    def broad_white_list_datasets(self):
        return ['02_undisturbed_slow_rotation_B.mat', '03_undisturbed_slow_rotation_C.mat', '05_undisturbed_slow_rotation_with_breaks_B.mat', '07_undisturbed_fast_rotation_B.mat', '09_undisturbed_fast_rotation_with_breaks_B.mat', '10_undisturbed_slow_translation_A.mat', '11_undisturbed_slow_translation_B.mat', '12_undisturbed_slow_translation_C.mat', '14_undisturbed_slow_translation_with_breaks_B.mat', '16_undisturbed_fast_translation_B.mat', '18_undisturbed_fast_translation_with_breaks_B.mat', '24_disturbed_tapping_A.mat', '25_disturbed_tapping_B.mat', '26_disturbed_phone_vibration_A.mat', '27_disturbed_phone_vibration_B.mat', '32_disturbed_attached_magnet_1cm.mat', '33_disturbed_attached_magnet_2cm.mat']
    
    def broad_start_list_datasets(self):
        len_wh = len(self.broad_white_list_datasets())
        first_part_st = [35, 34, 30, 23]
        next_st = 10
        missing = len_wh - len(first_part_st)
        all_starts = first_part_st + [next_st for i in range(missing)]
        return all_starts

    def broad_black_under_thresh(self, threshold):
        # return list of datasets from black list with error jump threshold above given threshold
        select = {k: v for k, v in self.black_list_error_nan_ratio.items() if v < threshold}
        return list(select.keys())


    def load_broad_dataset(self, file_name, mean_initial_samples = False, bypass_black_list = False, skip_jump_fix = False):

        if file_name in self.black_list_error_jump and not bypass_black_list:
            return None

        wh = self.broad_white_list_datasets()
        starts = self.broad_start_list_datasets()
        st_dict = dict(zip(wh, starts))
        current_st = st_dict.get(file_name)
        if current_st is None:
            current_st = 10

        mat = scipy.io.loadmat(os.path.join(self.base_path_broad, file_name))
        
        quat = mat['opt_quat']
        # not nan
        valid_quat = ~np.isnan(quat).any(axis=1)
        length = np.sum(valid_quat)
        
        sr = mat['sampling_rate'].squeeze()
        dt = 1.0 / sr        
        timestamp = np.arange(0, length*dt, dt)[:length]

        gyr = mat[f'imu_gyr'][valid_quat,:]
        acc = mat[f'imu_acc'][valid_quat,:]/self.ms2g
        mag = mat[f'imu_mag'][valid_quat,:]

        st_idx = np.argmin(np.abs(timestamp - current_st))
        if mean_initial_samples:
            # index of start of movement, nearest to current_st in timestamp
            gyr[:st_idx] = gyr[:st_idx].mean(axis=0)
            acc[:st_idx] = acc[:st_idx].mean(axis=0)
            mag[:st_idx] = mag[:st_idx].mean(axis=0)
            
            
        reference = quat[valid_quat,:]
        if file_name in self.black_list_error_jump and not skip_jump_fix:
            
            threshold = self.black_thresholds.get(file_name, None)
            if threshold is None or threshold < 0:
                return None
            
            diff = angle_error(reference, reference, align_start=False, shift_samples=1)
            diff_large = diff > threshold
            N=6000
            diff_large = np.convolve(diff_large, np.ones(N, dtype=bool), mode='same') > 0
            #shift half of N to the right, so that the large diff is marked from the start of the jump
            diff_large = np.roll(diff_large, N//2)
            diff_large = np.concatenate((diff_large, np.zeros(1, dtype=bool)))
            reference[diff_large] = np.NAN

        data = {
            'time': timestamp,
            'gyroscope': gyr,
            'accelerometer': acc,
            'magnetometer': mag,
            'reference': reference,
            'mean_sampling_rate': sr,
            'start_time': {'seconds': current_st, 'index': st_idx},
            'static_detection': { 'acc_threshold': 0.07, 'gyr_threshold': 0.1, 'mag_threshold': 9}
        }
        return data
       

    def load_sassari_dataset(self, file_name, unit_n):   

        units = ['AP1', 'AP2', 'SH1', 'SH2', 'XS1', 'XS2']

        speed = file_name.split('_')[0]
        speeds = {'slow': [60, 220], 'medium': [66, 170], 'fast': [70, 150]}
        start = speeds[speed][0]
        if speed not in speeds.keys():
            raise ValueError(f"Unknown speed in file name: {file_name}")

        mat = scipy.io.loadmat(os.path.join(self.base_path_sassari, file_name))
        
        dat = self.load_sassari_unit(mat, units[unit_n])
        
        dat['interest_range'] = (speeds[speed][0], speeds[speed][1])
        
        st_idx = np.argmin(np.abs(dat['time'] - start))
        dat['start_time'] = {'seconds': start, 'index': st_idx}
        dat['static_detection'] = { 'acc_threshold': 0.002, 'gyr_threshold': 0.05, 'mag_threshold': 0.04}

        return dat
    
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

        dt = np.mean(np.diff(raw_time)) # the time values seems inaccurate
        sr = 100
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
            'mean_sampling_rate': mean_sampling_rate,
            'static_detection': { 'acc_threshold': 0.07, 'gyr_threshold': 0.3, 'mag_threshold': 0.1}
        }
        
        self.datasets[dataset_name] = data
        return data

    def load_justa_raw(self, num, extend_by = 0):
        fold =  './Datasets/raw_j/'
        files_m = get_measurement_files(fold)
        name, files = list(files_m.items())[ num]
        pd_rot, dat_imu = load_raw_justa(files[1], files[0])

        interp_quats = interpolate_vicon_to_imu(pd_rot, dat_imu)
        interp_quats = fix_negative_qw(interp_quats)
        interp_quats = fix_ref_drops(interp_quats)
        
        dat_imu = add_time_from_start(dat_imu)
        #dat_imu = fix_magnet_alignment(dat_imu) # already fixed in raw data
        dat_imu = fix_coordinate_system(dat_imu)

        sampling_rate = 1.0 / np.diff(dat_imu['time_from_start']).mean()
        
        starts = [7.0, 0.5, 2]
        start_time = starts[num]
        st_idx = np.argmin(np.abs(dat_imu['time_from_start'] - start_time))
        st = {'seconds': start_time, 'index': st_idx}
        
        data = {
            'time': dat_imu['time_from_start'].values,
            'gyroscope': dat_imu[['gyr_x', 'gyr_y', 'gyr_z']].values,
            'accelerometer': dat_imu[['acc_x', 'acc_y', 'acc_z']].values,
            'magnetometer': dat_imu[['mag_x', 'mag_y', 'mag_z']].values,
            'reference': interp_quats,
            'mean_sampling_rate': sampling_rate,
            'dataset': name,
            'start_time': st
        }

        if extend_by > 0:
            last_time = data['time'][-1]
            dt = 1.0 / sampling_rate
            extra_time = np.arange(last_time + dt, last_time + dt*(extend_by+2), dt)
            gyro_mean_last_10 = np.zeros(3) # data['gyroscope'][-10:].mean(axis=0)
            acc_mean_last_10 = data['accelerometer'][-10:].mean(axis=0)
            mag_mean_last_10 = data['magnetometer'][-10:].mean(axis=0)

            data['time'] = np.concatenate((data['time'], extra_time[:extend_by]))
            data['gyroscope'] = np.concatenate((data['gyroscope'], np.tile(gyro_mean_last_10, (extend_by, 1))))
            data['accelerometer'] = np.concatenate((data['accelerometer'], np.tile(acc_mean_last_10, (extend_by, 1))))
            data['magnetometer'] = np.concatenate((data['magnetometer'], np.tile(mag_mean_last_10, (extend_by, 1))))
            data['reference'] = np.concatenate((data['reference'], np.tile(data['reference'][-1], (extend_by, 1))))

        return data
    
    def all_raw_justa(self):
        #bad idea
        dat0 = self.load_justa_raw(0)
        dat1 = self.load_justa_raw(1)
        dat2 = self.load_justa_raw(2)

        dat0_end_cut = -600

        dt = dat0['time'][1] - dat0['time'][0]
        tsh1 = dat0['time'][dat0_end_cut] + dt
        ths2 = tsh1 + (dat1['time'][-1] - dat1['time'][0]) + dt

        # join datasets
        dat = {
            'time': np.concatenate((dat0['time'][:dat0_end_cut], dat1['time'] + tsh1, dat2['time'] + ths2)),
            'gyroscope': np.concatenate((dat0['gyroscope'][:dat0_end_cut], dat1['gyroscope'], dat2['gyroscope'])),
            'accelerometer': np.concatenate((dat0['accelerometer'][:dat0_end_cut], dat1['accelerometer'], dat2['accelerometer'])),
            'magnetometer': np.concatenate((dat0['magnetometer'][:dat0_end_cut], dat1['magnetometer'], dat2['magnetometer'])),
            'reference': np.concatenate((dat0['reference'][:dat0_end_cut], dat1['reference'], dat2['reference'])),
            'mean_sampling_rate': (dat0['mean_sampling_rate'] + dat1['mean_sampling_rate'] + dat2['mean_sampling_rate']) / 3
        }
        return dat

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
