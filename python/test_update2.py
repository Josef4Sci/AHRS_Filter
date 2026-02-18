
import scipy.io
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

fol = 'C:\\Users\\josef\\Desktop\\AHRS_Filter\\Datasets\\raw_j\\'
path ="mereni1_rotace1-2018-10-02-14-03-23-vicon-IMUframe-IMUframe.csv"
dat_vic = pd.read_csv(fol + path)
dat_vic['time'] = pd.to_datetime(dat_vic['time'], format='%Y/%m/%d/%H:%M:%S.%f')
pd_rot = dat_vic[['time','.transform.rotation.x','.transform.rotation.y','.transform.rotation.z','.transform.rotation.w']].to_numpy()

imu_file = 'mereni1_rotace1-2018-10-02-14-03-23-arr_format.csv'
columns = [ 'time', 'un1', 'un2', 'acc_x', 'acc_y', 'acc_z', 'gyr_x', 'gyr_y', 'gyr_z', 'mag_x', 'mag_y', 'mag_z',  'a','b','c','d']
dat_imu = pd.read_csv(fol + imu_file, delimiter=';', names=columns)
acc_mult = 0.00098/16
deq2rad = np.pi/180
gyr_mult = deq2rad*2000/32767
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

plt.plot(dat_imu['time'], dat_imu['gyr_x'], label='gyr_x')
plt.plot(dat_imu['time'], dat_imu['gyr_y'], label='gyr_y')
plt.plot(dat_imu['time'], dat_imu['gyr_z'], label='gyr_z')

plt.plot(pd_rot[:,0], pd_rot[:,1], label='x')
plt.plot(pd_rot[:,0], pd_rot[:,2], label='y')
plt.plot(pd_rot[:,0], pd_rot[:,3], label='z')
plt.plot(pd_rot[:,0], pd_rot[:,4], label='w')
plt.legend()
plt.show()