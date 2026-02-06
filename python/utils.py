import numpy as np
from quaternion_library import quatern_conj, quatern_prod
from matplotlib import pyplot as plt


def angle_error(q_est, q_ref, use_imu=False):
    
    q_err = quatern_prod(q_ref, quatern_conj(q_est))
    
    # Ensure all quaternions have positive w component
    q_err[q_err[:, 0] < 0] = -q_err[q_err[:, 0] < 0]
    
    # Calculate angular error
    if use_imu:
        angle_error = np.abs(2 * np.arctan2(
            np.linalg.norm(q_err[:, 1:4], axis=1),
            q_err[:, 0]
        ) * 180 / np.pi)
    else:
        angle_error = np.abs(2 * np.arctan2(
            np.linalg.norm(q_err[:, 1:4], axis=1),
            q_err[:, 0]
        ) * 180 / np.pi)
        
    return angle_error

def eval_filter_on_dataset(filter_instance, dataset, use_imu=False, use_square_err=False):
    # Reset quaternion to reference at start
    filter_instance.quaternion = dataset['reference'][0].copy()
    
    # Run filter through data
    quaternion_result = np.zeros((len(dataset['time']), 4))
        
    quaternion_result[0] = filter_instance.quaternion
    
    for t in range(1, len(dataset['time'])):
        # Calculate sample period
       
        dt = dataset['time'][t] - dataset['time'][t-1]
        
        # Update filter
        if use_imu:
            filter_instance.update_imu(
                dataset['gyroscope'][t],
                dataset['accelerometer'][t],
                dt
            )
        else:
            filter_instance.update(
                dataset['gyroscope'][t],
                dataset['accelerometer'][t],
                dataset['magnetometer'][t],
                dt
            )
            
        quaternion_result[t] = filter_instance.quaternion
        
    # Calculate error
    q_ref = dataset['reference']
    
    angle_err = angle_error(quaternion_result, q_ref, use_imu=use_imu)
        
    # Use RMS or absolute error
    if use_square_err:
        angle_err = angle_err ** 2
        
    return quaternion_result, angle_err


def plot_dataset(dataset, angle_error=None):
    plt.figure(figsize=(12, 8))
    
    size = 5 if angle_error is not None else 4
    
    plt.subplot(size, 1, 1)
    plt.plot(dataset['time'], dataset['gyroscope'])
    plt.title('Gyroscope')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend(['gx', 'gy', 'gz'])
    
    plt.subplot(size, 1, 2)
    plt.plot(dataset['time'], dataset['accelerometer'])
    plt.title('Accelerometer')
    plt.ylabel('Acceleration (m/s²)')
    plt.legend(['ax', 'ay', 'az'])
    
    plt.subplot(size, 1, 3)
    plt.plot(dataset['time'], dataset['magnetometer'])
    plt.title('Magnetometer')
    plt.ylabel('Magnetic Field (µT)')
    plt.legend(['mx', 'my', 'mz'])
    
    plt.subplot(size, 1, 4)
    plt.plot(dataset['time'], dataset['reference'], label='Reference')
    plt.title('Quaternion Reference')
    plt.ylabel('Quaternion')
    plt.legend(['w', 'x', 'y', 'z'])
    
    if angle_error is not None:
        plt.subplot(size, 1, 5)
        plt.plot(dataset['time'], angle_error)
        plt.title('Angular Error')
        plt.xlabel('Time (s)')
        plt.ylabel('Error (degrees)')
    
    plt.tight_layout()
    plt.show()