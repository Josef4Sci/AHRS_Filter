
import scipy.io
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import re
from collections import defaultdict
import os

BASE_FOLDER = './Datasets/raw_j/'

def load_raw_justa(vic_path, imu_path):
    dat_vic = pd.read_csv(BASE_FOLDER + vic_path)
    dat_vic['time'] = pd.to_datetime(dat_vic['time'], format='%Y/%m/%d/%H:%M:%S.%f')
    pd_rot = dat_vic[['time','.transform.rotation.x','.transform.rotation.y','.transform.rotation.z','.transform.rotation.w']]

    columns = [ 'time', 'un1', 'un2', 'acc_x', 'acc_y', 'acc_z', 'gyr_x', 'gyr_y', 'gyr_z', 'mag_x', 'mag_y', 'mag_z',  'a','b','c','d']
    dat_imu = pd.read_csv(BASE_FOLDER + imu_path, delimiter=';', names=columns)
    acc_mult = 0.00098/16
    deg2rad = np.pi/180
    gyr_mult = deg2rad*2000/32767
    mag_mult = 1/1000

    dat_imu['time'] = pd.to_datetime(dat_imu['time'], format='%Y/%m/%d/%H:%M:%S.%f')
    dat_imu['acc_x'] = dat_imu['acc_x'] * acc_mult
    dat_imu['acc_y'] = dat_imu['acc_y'] * acc_mult
    dat_imu['acc_z'] = dat_imu['acc_z'] * acc_mult
    dat_imu['gyr_x'] = dat_imu['gyr_x'] * gyr_mult
    dat_imu['gyr_y'] = dat_imu['gyr_y'] * gyr_mult
    dat_imu['gyr_z'] = dat_imu['gyr_z'] * gyr_mult
    dat_imu['mag_x'] = dat_imu['mag_x'] * mag_mult
    dat_imu['mag_y'] = dat_imu['mag_y'] * mag_mult
    dat_imu['mag_z'] = dat_imu['mag_z'] * mag_mult
    
    return pd_rot, dat_imu


def interpolate_vicon_to_imu(pd_rot, dat_imu):
    # Interpolate Vicon data to match IMU timestamps
    imu_times = dat_imu['time'].values.astype(np.int64)  # Convert to nanoseconds
    vicon_times = pd_rot['time'].values.astype(np.int64)  # Convert to nanoseconds
    vicon_quats = pd_rot[['.transform.rotation.w','.transform.rotation.x', '.transform.rotation.y', '.transform.rotation.z']].values     # Extract quaternion components

    interp_quats = np.zeros((len(imu_times), 4))
    for i in range(4):
        interp_quats[:, i] = np.interp(imu_times, vicon_times, vicon_quats[:, i])
    
    return interp_quats

def add_time_from_start(dat_imu):
    dat_imu['time_from_start'] = (dat_imu['time'] - dat_imu['time'].iloc[0]).dt.total_seconds()
    return dat_imu

def fix_negative_qw(quaternions):
    # Ensure qw is non-negative
    for i in range(len(quaternions)):
        if quaternions[i, 0] < 0:  # Assuming qw is the first component
            quaternions[i] = -quaternions[i]
    return quaternions

def get_measurement_files(base_folder):
    # Regex pattern: capture everything before the date pattern
    pattern = r'^(.+?)-\d{4}-\d{2}-\d{2}'

    # Group files by measurement name
    measurements = defaultdict(list)

    for filename in os.listdir(base_folder):
        match = re.match(pattern, filename)
        if match:
            measurement_name = match.group(1)
            measurements[measurement_name].append(filename)

    return measurements

def fix_magnet_alignment(dat_imu):
    #sensor in bmx055 has switched axis for magnetometer, fix by m_x_fix = -m_y and m_y_fix = m_x
    mx_copy = dat_imu['mag_x'].copy()
    my_copy = dat_imu['mag_y'].copy()
    dat_imu['mag_x'], dat_imu['mag_y'] = -my_copy, mx_copy
    return dat_imu

if __name__ == '__main__':
    measurements = get_measurement_files(BASE_FOLDER)
    name, files = list(measurements.items())[0]

    pd_rot, dat_imu = load_raw_justa(files[1], files[0])
    interp_quats = interpolate_vicon_to_imu(pd_rot, dat_imu)
    fixed_q = fix_negative_qw(interp_quats)
    dat_imu = add_time_from_start(dat_imu)

    plt.plot(dat_imu['time_from_start'], dat_imu['gyr_x'], label='gyr_x')
    plt.plot(dat_imu['time_from_start'], dat_imu['gyr_y'], label='gyr_y')
    plt.plot(dat_imu['time_from_start'], dat_imu['gyr_z'], label='gyr_z')
    plt.plot(dat_imu['time_from_start'], fixed_q, label=['qw', 'qx', 'qy', 'qz'])

    # plt.plot(pd_rot[:,0], pd_rot[:,1], label='x')
    # plt.plot(pd_rot[:,0], pd_rot[:,2], label='y')
    # plt.plot(pd_rot[:,0], pd_rot[:,3], label='z')
    # plt.plot(pd_rot[:,0], pd_rot[:,4], label='w')
    plt.legend()
    plt.show()