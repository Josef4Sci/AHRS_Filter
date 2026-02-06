from dataset_loader import DatasetLoader
from filters import (JustaAHRSv2)
from utils import eval_filter_on_dataset, plot_dataset
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
from quaternion_library import quatern_prod, quatern_conj, quaternion_rotate_vector, integrate_rk4
import pickle

def create_sensor_data_simulation():
    """
    Generate synthetic IMU sensor data (accelerometer, magnetometer, gyroscope)
    with realistic noise, bias, and scale errors.
    
    Returns:
    --------
    data : dict
        Dictionary containing:
        - 'accelerometer': np.ndarray, shape (N, 3) in G
        - 'magnetometer': np.ndarray, shape (N, 3) in Tesla
        - 'gyroscope': np.ndarray, shape (N, 3) in rad/s
        - 'quaternion_reference': np.ndarray, shape (N, 4) [w, x, y, z]
        - 'time': np.ndarray, shape (N,) in seconds
    """
    
    # Parameters
    wahba_angle = 25.52  # degrees - magnetic inclination angle
    mag_intensity = 50e-3  # Tesla
    
    # Sensor errors
    acc_scale_err = 1e-2
    mag_scale_err = 5e-2
    gyr_scale_err = 5e-3
    
    acc_noise_err = 1e-2  # G
    mag_noise_err = 2e-4  # T
    gyr_noise_err = 0.1   # deg/s
    
    gyr_off_err = 0.2     # deg/s
    mag_off_err = 7e-4    # T
    acc_off_err = 3e-3    # G
    
    # Precompute reference vectors
    acc_null = np.array([0.0, 0.0, 1.0])  # Gravity in world frame (pointing down)
    
    # Compute magnetic field reference (rotated by wahba angle)
    wahba_rad = np.deg2rad(wahba_angle)
    q_temp = np.array([
        np.cos(wahba_rad / 2),
        0.0,
        np.sin(wahba_rad / 2),
        0.0
    ])
    mag_null = quaternion_rotate_vector(q_temp, acc_null) * mag_intensity
    
    # Simulation parameters
    leng = 12000
    dt = 0.0039  # seconds (approximately 256 Hz)
    
    # Preallocate arrays
    generated_mag = np.zeros((leng, 3))
    generated_acc = np.zeros((leng, 3))
    generated_gyr = np.zeros((leng, 3))
    quat = np.zeros((leng, 4))
    time = np.zeros(leng)
    
    # Initial conditions
    quat[0] = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
    time[0] = 0.0
    
    # Rotation parameters
    angle = np.deg2rad(40 * dt)  # Rotation angle per step
    
    # Sensor bias offsets
    mag_offset = np.array([mag_off_err, -mag_off_err, mag_off_err / 2])
    acc_offset = np.array([-acc_off_err, acc_off_err, acc_off_err / 2])
    gyr_offset = np.array([-gyr_off_err, gyr_off_err, gyr_off_err / 2])
    
    # Simulation phases
    phase = 1
    
    # Generate data
    for i in range(leng - 1):
        # Phase transition: stop rotation between 15-20s when acc.x is small
        if phase == 1 and 15 < time[i] < 20 and i > 0:
            if abs(generated_acc[i - 1, 0]) < 0.01:
                phase = 2
        
        # Generate quaternion trajectory
        if phase == 1:
            # Rotating phase
            direction = np.array([0.1, 0.2, 0.05])
            direction = direction / np.linalg.norm(direction)
            
            q_delta = np.array([
                np.cos(angle / 2),
                direction[0] * np.sin(angle / 2),
                direction[1] * np.sin(angle / 2),
                direction[2] * np.sin(angle / 2)
            ])
            q_delta = q_delta / np.linalg.norm(q_delta)
            
            quat[i + 1] = quatern_prod(quat[i], q_delta)
            
            # Ensure positive scalar part
            if quat[i + 1, 0] < 0:
                quat[i + 1] = -quat[i + 1]
        
        elif phase == 2:
            # Static phase
            quat[i + 1] = quat[i].copy()
        
        # Generate magnetometer reading (in body frame)
        mag_body = quaternion_rotate_vector(quat[i], mag_null)
        generated_mag[i] = (
            -(1 + mag_scale_err) * mag_body +
            np.random.randn(3) * mag_noise_err +
            mag_offset
        )
        
        # Generate accelerometer reading (in body frame)
        acc_body = quaternion_rotate_vector(quat[i], acc_null)
        generated_acc[i] = (
            (1 + acc_scale_err) * acc_body +
            np.random.randn(3) * acc_noise_err +
            acc_offset
        )
        
        # Update time
        time[i + 1] = time[i] + dt
        
        # Compute angular velocity from quaternion difference
        q_diff = quatern_prod(quatern_conj(quat[i]), quat[i + 1])
        
        # Ensure positive scalar part
        if q_diff[0] < 0:
            q_diff = -q_diff
        
        # Extract angular velocity (small angle approximation)
        omega = 2.0 * q_diff[1:4] / dt  # rad/s
        
        # Generate gyroscope reading with errors
        generated_gyr[i] = (
            (1 + gyr_scale_err) * omega +
            np.random.randn(3) * np.deg2rad(gyr_noise_err) +
            np.deg2rad(gyr_offset)
        )
    
    # Handle last sample
    generated_mag[-1] = generated_mag[-2]
    generated_acc[-1] = generated_acc[-2]
    generated_gyr[-1] = generated_gyr[-2]
    
    # Add external acceleration disturbance (simulating movement)
    intA = slice(6000, 8001)
    generated_acc[intA, 1] += 0.5 * np.sin(np.linspace(0, 6 * np.pi, 2001))
    
    # Add magnetic disturbance (simulating magnetic interference)
    intM = slice(9000, 11001)
    generated_mag[intM, 0] += mag_intensity * 0.5 * np.sin(np.linspace(0, 6 * np.pi, 2001))
    
    # Return data dictionary
    data = {
        'accelerometer': generated_acc,
        'magnetometer': generated_mag,
        'gyroscope': generated_gyr,
        'reference': quat,
        'time': time
    }
    
    return data


def compute_rotational_acceleration(omega, omega_dot, sensor_pose):
    """
    Compute acceleration due to sensor being offset from rotation center.
    
    For a rigid body rotating about the origin:
    a = omega_dot × r + omega × (omega × r)
    
    where:
    - omega: angular velocity (rad/s)
    - omega_dot: angular acceleration (rad/s²)
    - r: sensor position relative to rotation center
    
    The first term is tangential acceleration (from angular acceleration)
    The second term is centripetal acceleration (from angular velocity)
    
    Parameters:
    -----------
    omega : np.ndarray, shape (3,)
        Angular velocity in world frame [wx, wy, wz] rad/s
    omega_dot : np.ndarray, shape (3,)
        Angular acceleration in world frame [ax, ay, az] rad/s²
    sensor_pose : np.ndarray, shape (3,)
        Sensor position in world frame [x, y, z] meters
    
    Returns:
    --------
    a_rotational : np.ndarray, shape (3,)
        Rotational acceleration in world frame (m/s²)
    """
    # Tangential acceleration: omega_dot × r
    a_tangential = np.cross(omega_dot, sensor_pose)
    
    # Centripetal acceleration: omega × (omega × r)
    a_centripetal = np.cross(omega, np.cross(omega, sensor_pose))
    
    # Total rotational acceleration
    a_rotational = a_tangential + a_centripetal
    
    return a_rotational


def create_sensor_data_simulation_rigid_body(sensor_pose=np.array([0.0, 0.0, 0.0]),\
                                             gyro_offset=0.0,\
                                             gyro_noise=0.0,
                                             gyro_scale=0.0):
    """
    Generate synthetic IMU sensor data with rigid body rotational acceleration.
    
    Parameters:
    -----------
    sensor_pose : np.ndarray, shape (3,)
        Sensor position relative to rotation center [x, y, z] in meters
        Default [0, 0, 0] means sensor is at rotation center (no extra acceleration)
        Example: [0.1, 0.0, 0.0] means sensor is 10cm along x-axis from center
    
    Returns:
    --------
    data : dict
        Dictionary containing:
        - 'accelerometer': np.ndarray, shape (N, 3) in G
        - 'magnetometer': np.ndarray, shape (N, 3) in Tesla
        - 'gyroscope': np.ndarray, shape (N, 3) in rad/s
        - 'quaternion_reference': np.ndarray, shape (N, 4) [w, x, y, z]
        - 'time': np.ndarray, shape (N,) in seconds
        - 'omega_world': np.ndarray, shape (N, 3) angular velocity in world frame
        - 'acceleration_world': np.ndarray, shape (N, 3) total acceleration in world frame
    """
    
    # Parameters
    wahba_angle = 25.52  # degrees - magnetic inclination angle
    mag_intensity = 50e-3  # Tesla
    
    # Sensor errors
    acc_scale_err = 1e-2
    mag_scale_err = 5e-2
    gyr_scale_err = gyro_scale # 5e-3
    
    acc_noise_err = 1e-2  # G
    mag_noise_err = 2e-4  # T
    gyr_noise_err = gyro_noise #  0.1   # deg/s
    
    gyr_off_err = gyro_offset #0.2     # deg/s
    mag_off_err = 7e-4    # T
    acc_off_err = 3e-3    # G
    
    # Precompute reference vectors
    acc_null = np.array([0.0, 0.0, 1.0])  # Gravity in world frame (pointing down, 1G)
    
    # Compute magnetic field reference (rotated by wahba angle)
    wahba_rad = np.deg2rad(wahba_angle)
    q_temp = np.array([
        np.cos(wahba_rad / 2),
        0.0,
        np.sin(wahba_rad / 2),
        0.0
    ])
    mag_null = quaternion_rotate_vector(q_temp, acc_null) * mag_intensity
    
    # Simulation parameters
    leng = 12000
    dt = 0.0039  # seconds (approximately 256 Hz)
    
    # Preallocate arrays
    generated_mag = np.zeros((leng, 3))
    generated_acc = np.zeros((leng, 3))
    generated_gyr = np.zeros((leng, 3))
    quat = np.zeros((leng, 4))
    time = np.zeros(leng)
    omega_world = np.zeros((leng, 3))  # Angular velocity in world frame
    acceleration_world = np.zeros((leng, 3))  # Total acceleration in world frame
    
    # Initial conditions
    quat[0] = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
    time[0] = 0.0
    
    # Rotation parameters
    angle = np.deg2rad(40 * dt)  # Rotation angle per step
    
    # Sensor bias offsets
    mag_offset = np.array([mag_off_err, -mag_off_err, mag_off_err / 2])
    acc_offset = np.array([-acc_off_err, acc_off_err, acc_off_err / 2])
    gyr_offset = np.array([-gyr_off_err, gyr_off_err, gyr_off_err / 2])
    
    # Rotation direction (constant in world frame)
    direction = np.array([0.1, 0.2, 0.05])
    direction = direction / np.linalg.norm(direction)
    
    # Constant angular velocity magnitude in world frame (rad/s)
    omega_magnitude = angle / dt
    
    # Simulation phases
    phase = 1
    
    # Generate data
    for i in range(leng - 1):
        # Phase transition: stop rotation between 15-20s when acc.x is small
        if phase == 1 and 15 < time[i] < 20 and i > 0:
            if abs(generated_acc[i - 1, 0]) < 0.01:
                phase = 2
        
        # Compute angular velocity in world frame
        if phase == 1:
            # Rotating phase - constant angular velocity in world frame
            omega_world[i] = omega_magnitude * direction
            
            # Angular acceleration in world frame (zero for constant rotation)
            omega_dot_world = np.array([0.0, 0.0, 0.0])
            
            # Generate quaternion using RK4
            # First, rotate sensor pose to current orientation
            sensor_pose_rotated = quaternion_rotate_vector(quat[i], sensor_pose)
            
            # Compute angular velocity in body frame for RK4
            omega_body = quaternion_rotate_vector(quatern_conj(quat[i]), omega_world[i])
            
            # Integrate quaternion using RK4
            quat[i + 1] = integrate_rk4(quat[i], omega_body, dt)
            
            # Ensure positive scalar part
            if quat[i + 1, 0] < 0:
                quat[i + 1] = -quat[i + 1]
            
        elif phase == 2:
            # Static phase - no rotation
            omega_world[i] = np.array([0.0, 0.0, 0.0])
            omega_dot_world = np.array([0.0, 0.0, 0.0])
            quat[i + 1] = quat[i].copy()
        
        # Compute rotational acceleration due to off-center sensor placement
        # Position in world frame (rotates with the body)
        sensor_pose_world = quaternion_rotate_vector(quat[i], sensor_pose)
        
        # Acceleration due to rotation (in world frame)
        a_rotational_world = compute_rotational_acceleration(
            omega_world[i], 
            omega_dot_world, 
            sensor_pose_world
        )
        
        # Total acceleration in world frame = gravity + rotational acceleration
        # (in G units, so divide rotational by 9.81)
        acceleration_world[i] = acc_null + a_rotational_world / 9.81
        
        # Transform total acceleration to body frame
        acc_body = quaternion_rotate_vector(quat[i], acceleration_world[i])
        
        # Generate accelerometer reading (in body frame)
        generated_acc[i] = (
            (1 + acc_scale_err) * acc_body +
            np.random.randn(3) * acc_noise_err +
            acc_offset
        )
        
        # Generate magnetometer reading (in body frame)
        mag_body = quaternion_rotate_vector(quat[i], mag_null)
        generated_mag[i] = (
            -(1 + mag_scale_err) * mag_body +
            np.random.randn(3) * mag_noise_err +
            mag_offset
        )
        
        # Update time
        time[i + 1] = time[i] + dt
        
        # Angular velocity in body frame
        omega_body = quaternion_rotate_vector(quatern_conj(quat[i]), omega_world[i])
        
        # Generate gyroscope reading with errors
        generated_gyr[i] = (
            (1 + gyr_scale_err) * omega_body +
            np.random.randn(3) * np.deg2rad(gyr_noise_err) +
            np.deg2rad(gyr_offset)
        )
    
    # Handle last sample
    omega_world[-1] = omega_world[-2]
    acceleration_world[-1] = acceleration_world[-2]
    generated_mag[-1] = generated_mag[-2]
    generated_acc[-1] = generated_acc[-2]
    generated_gyr[-1] = generated_gyr[-2]
    
    # Add external acceleration disturbance (simulating movement)
    intA = slice(6000, 8001)
    generated_acc[intA, 1] += 0.5 * np.sin(np.linspace(0, 6 * np.pi, 2001))
    
    # Add magnetic disturbance (simulating magnetic interference)
    intM = slice(9000, 11001)
    generated_mag[intM, 0] += mag_intensity * 0.5 * np.sin(np.linspace(0, 6 * np.pi, 2001))
    
    # Return data dictionary
    data = {
        'accelerometer': generated_acc,
        'magnetometer': generated_mag,
        'gyroscope': generated_gyr,
        'reference': quat,
        'time': time,
        'omega_world': omega_world,
        'acceleration_world': acceleration_world,
        'sensor_pose': sensor_pose
    }
    
    return data


#data = create_sensor_data_simulation()
data = create_sensor_data_simulation_rigid_body(sensor_pose=np.array([1.2, 1.2, 1.0]),\
                                                gyro_noise=0.1, gyro_offset=0.2, gyro_scale=5e-3 )

plt.plot(data['time'], np.linalg.norm(data['accelerometer'], axis=1), label='Acc World Magnitude')
plt.show()

pickle.dump(data, open('synthetic_rigid_body_sensor_offset.pkl', 'wb'))
#plot_dataset(data)