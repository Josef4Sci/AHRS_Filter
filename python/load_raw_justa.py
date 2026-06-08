
import scipy.io
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import re
from collections import defaultdict
import os

from utils import angle_error
from quaternion_library import quatern_prod, quatern_conj

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
    mag_mult = -1/1000

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

def fix_coordinate_system(dat_imu):
    # quternion is from optimization
    qr_fix =np.array([ 0.93107197,  -0.01282704,  -0.01901651, 0.36335445])
    
    acc_meas = np.array([dat_imu['acc_x'], dat_imu['acc_y'], dat_imu['acc_z']]).T
    mag_meas = np.array([dat_imu['mag_x'], dat_imu['mag_y'], dat_imu['mag_z']]).T
    gyr_meas = np.array([dat_imu['gyr_x'], dat_imu['gyr_y'], dat_imu['gyr_z']]).T

    qr_stack = np.tile(qr_fix, (len(dat_imu), 1))
    acc_fixed = quatern_prod(quatern_prod(qr_stack, np.hstack((np.tile([0], (len(dat_imu), 1)), acc_meas))), quatern_conj(qr_stack))
    mag_fixed = quatern_prod(quatern_prod(qr_stack, np.hstack((np.tile([0], (len(dat_imu), 1)), mag_meas))), quatern_conj(qr_stack))
    gyr_fixed = quatern_prod(quatern_prod(qr_stack, np.hstack((np.tile([0], (len(dat_imu), 1)), gyr_meas))), quatern_conj(qr_stack))

    dat_imu['acc_x'], dat_imu['acc_y'], dat_imu['acc_z'] = acc_fixed[:, 1], acc_fixed[:, 2], acc_fixed[:, 3]
    dat_imu['mag_x'], dat_imu['mag_y'], dat_imu['mag_z'] = mag_fixed[:, 1], mag_fixed[:, 2], mag_fixed[:, 3]
    dat_imu['gyr_x'], dat_imu['gyr_y'], dat_imu['gyr_z'] = gyr_fixed[:, 1], gyr_fixed[:, 2], gyr_fixed[:, 3]
    return dat_imu

def fix_ref_drops(interp_quats):
    
    norm_q = np.linalg.norm(interp_quats, axis=1)
    interpolate_indices = np.where(norm_q < 0.999)[0]    
    interpolate_indices2 = np.where(norm_q > 1.01)[0]
    
    for idx in np.concatenate((interpolate_indices, interpolate_indices2)):
        if idx == 0 or idx == len(interp_quats) - 1:
            continue  # Skip if it's the first or last index
        # Linear interpolation of quaternions (not ideal, but a simple fix)
        interp_quats[idx] = (interp_quats[idx - 1] + interp_quats[idx + 1]) / 2
        interp_quats[idx] /= np.linalg.norm(interp_quats[idx])  # Normalize to unit quaternion

    shifted = np.roll(interp_quats, 1, axis=0)  # Shift by one sample
    shifted[0] = interp_quats[0]  # Set the first sample to the original first sample

    diff = angle_error(interp_quats, shifted, align_start=False, shift_samples=0)
    interp_diff = np.where(diff > 3)[0]
    for idx in interp_diff:
        if idx == 0 or idx == len(interp_quats) - 1:
            continue  # Skip if it's the first or last index
        # Linear interpolation of quaternions (not ideal, but a simple fix)
        interp_quats[idx] = (interp_quats[idx - 1] + interp_quats[idx + 1]) / 2
        interp_quats[idx] /= np.linalg.norm(interp_quats[idx])  # Normalize to unit quaternion

    return interp_quats

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