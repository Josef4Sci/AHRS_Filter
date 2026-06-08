from dataset_loader import DatasetLoader
from utils import eval_filter_on_dataset, plot_dataset
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
from quaternion_library_jit import quatern_prod_single, quatern_conj_single, quaternion_rotate_vector, integrate_rk4
import pickle

def create_sensor_data_simulation_in_rot_certer():
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
        0.0,
        np.cos(wahba_rad / 2),
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


class SimSensorParameters:
    
    def __init__(self, wahba_angle=25.52, mag_intensity=50e-3, acc_scale_err=1e-2,
                 mag_scale_err=5e-2, gyr_scale_err=5e-3, acc_noise_err=1e-2, mag_noise_err=2e-4,
                 gyr_noise_err=0.1, gyr_off_err=0.002, mag_off_err=7e-4, acc_off_err=3e-3):

        # Parameters
        self.wahba_angle = wahba_angle  # degrees - magnetic inclination angle
        self.mag_intensity = mag_intensity  # Tesla
        
        # Sensor errors
        self.acc_scale_err = acc_scale_err
        self.mag_scale_err = mag_scale_err
        self.gyr_scale_err = gyr_scale_err
        
        self.acc_noise_err = acc_noise_err  # G
        self.mag_noise_err = mag_noise_err  # T
        self.gyr_noise_err = gyr_noise_err #  deg/s
        
        self.gyr_off_err = gyr_off_err #0.2     # deg/s
        self.mag_off_err = mag_off_err# 7e-4    # T
        self.acc_off_err = acc_off_err    # G
        
        # Precompute reference vectors
        self.acc_null = np.array([0.0, 0.0, 1.0])  # Gravity in world frame (pointing down, 1G)
        
        # Compute magnetic field reference (rotated by wahba angle)
        wahba_rad = np.deg2rad(self.wahba_angle)
        q_temp = np.array([
            np.cos(wahba_rad / 2),
            np.sin(wahba_rad / 2),
            0.0,
            0.0
        ])
        self.mag_null = quaternion_rotate_vector(q_temp, self.acc_null) * self.mag_intensity

def sim_rigid_body_rot_with_disturb(sensor_pose=np.array([0.0, 0.0, 0.0]), sen_params=SimSensorParameters(),
                                            rot_axis=np.array([0.1, 0.2, 0.05]), dist_mag_intensity=0.01, dist_acc_intensity=0.02,
                                            omega_world_ext=None, quat_ref_ext=None, time_ext=None):
    """
    Generate synthetic IMU sensor data with rigid body rotational acceleration.

    Parameters:
    -----------
    sensor_pose : np.ndarray, shape (3,)
        Sensor position relative to rotation center [x, y, z] in meters
    omega_world_ext : np.ndarray, shape (N, 3), optional
        External angular velocity in world frame. When provided together with
        quat_ref_ext and time_ext, the internal motion generation is skipped and
        these arrays drive the simulation instead.
    quat_ref_ext : np.ndarray, shape (N, 4), optional
        External reference quaternions [w, x, y, z].
    time_ext : np.ndarray, shape (N,), optional
        External time vector in seconds.

    Returns:
    --------
    data : dict
        Dictionary containing:
        - 'accelerometer': np.ndarray, shape (N, 3) in G
        - 'magnetometer': np.ndarray, shape (N, 3) in Tesla
        - 'gyroscope': np.ndarray, shape (N, 3) in rad/s
        - 'reference': np.ndarray, shape (N, 4) [w, x, y, z]
        - 'time': np.ndarray, shape (N,) in seconds
        - 'omega_world': np.ndarray, shape (N, 3) angular velocity in world frame
        - 'acceleration_world': np.ndarray, shape (N, 3) total acceleration in world frame
    """
    use_ext = omega_world_ext is not None and quat_ref_ext is not None and time_ext is not None

    if use_ext:
        leng = len(time_ext)
        time = time_ext.copy()
        quat = quat_ref_ext.copy()
        omega_world = omega_world_ext.copy()
        # angular acceleration via central differences
        omega_dot_arr = np.zeros((leng, 3))
        for i in range(1, leng - 1):
            span = time[i + 1] - time[i - 1]
            if span > 0:
                omega_dot_arr[i] = (omega_world[i + 1] - omega_world[i - 1]) / span
        omega_dot_arr[0] = omega_dot_arr[1]
        omega_dot_arr[-1] = omega_dot_arr[-2]
    else:
        leng = 12000
        dt = 0.0039  # seconds (approximately 256 Hz)
        quat = np.zeros((leng, 4))
        time = np.zeros(leng)
        omega_world = np.zeros((leng, 3))
        quat[0] = np.array([1.0, 0.0, 0.0, 0.0])
        time[0] = 0.0

    # Preallocate sensor arrays
    generated_mag = np.zeros((leng, 3))
    generated_acc = np.zeros((leng, 3))
    generated_gyr = np.zeros((leng, 3))
    acceleration_world = np.zeros((leng, 3))

    # Sensor bias offsets
    mag_offset = np.array([sen_params.mag_off_err, -sen_params.mag_off_err, sen_params.mag_off_err / 2])
    acc_offset = np.array([-sen_params.acc_off_err, sen_params.acc_off_err, sen_params.acc_off_err / 2])
    gyr_offset = np.array([-sen_params.gyr_off_err, sen_params.gyr_off_err, sen_params.gyr_off_err / 2])

    if not use_ext:
        # Rotation parameters for internal motion generation
        angle = np.deg2rad(40 * dt)
        direction = rot_axis / np.linalg.norm(rot_axis)
        omega_magnitude = angle / dt
        phase = 1

    sensor_pose_world = np.zeros(3)

    # Generate data
    for i in range(leng - 1):
        if use_ext:
            omega_dot_world = omega_dot_arr[i]
        else:
            # Phase transition: stop rotation between 15-20s when acc.x is small
            if phase == 1 and 15 < time[i] < 20 and i > 0:
                if abs(generated_acc[i - 1, 0]) < 0.01:
                    phase = 2

            if phase == 1:
                omega_world[i] = omega_magnitude * direction
                omega_dot_world = np.array([0.0, 0.0, 0.0])
                omega_body = quaternion_rotate_vector(quatern_conj_single(quat[i]), omega_world[i])
                quat[i + 1] = integrate_rk4(quat[i], omega_body, dt)
                if quat[i + 1, 0] < 0:
                    quat[i + 1] = -quat[i + 1]
            elif phase == 2:
                omega_world[i] = np.array([0.0, 0.0, 0.0])
                omega_dot_world = np.array([0.0, 0.0, 0.0])
                quat[i + 1] = quat[i].copy()

            time[i + 1] = time[i] + dt

        # Compute rotational acceleration due to off-center sensor placement
        sensor_pose_world = quaternion_rotate_vector(quat[i], sensor_pose)
        a_rotational_world = compute_rotational_acceleration(
            omega_world[i],
            omega_dot_world,
            sensor_pose_world
        )

        # Total acceleration in world frame = gravity + rotational acceleration (in G)
        acceleration_world[i] = sen_params.acc_null + a_rotational_world / 9.81

        # Transform total acceleration to body frame.
        # External case: q is body-to-world (B2W), so world→body needs q_conj.
        # Internal case: uses the simulation's own (inverted) convention to stay
        # compatible with filters trained on internal data.
        q_w2b = quatern_conj_single(quat[i]) if use_ext else quat[i]
        acc_body = quaternion_rotate_vector(q_w2b, acceleration_world[i])
        generated_acc[i] = (
            (1 + sen_params.acc_scale_err) * acc_body +
            np.random.randn(3) * sen_params.acc_noise_err +
            acc_offset
        )

        # Generate magnetometer reading (in body frame)
        mag_body = quaternion_rotate_vector(q_w2b, sen_params.mag_null)
        generated_mag[i] = (
            -(1 + sen_params.mag_scale_err) * mag_body +
            np.random.randn(3) * sen_params.mag_noise_err +
            mag_offset
        )

        # Angular velocity in body frame for gyroscope
        omega_body = quaternion_rotate_vector(quatern_conj_single(quat[i]), omega_world[i])
        generated_gyr[i] = (
            (1 + sen_params.gyr_scale_err) * omega_body +
            np.random.randn(3) * np.deg2rad(sen_params.gyr_noise_err) + gyr_offset
        )

    # Handle last sample
    omega_world[-1] = omega_world[-2]
    acceleration_world[-1] = acceleration_world[-2]
    generated_mag[-1] = generated_mag[-2]
    generated_acc[-1] = generated_acc[-2]
    generated_gyr[-1] = generated_gyr[-2]

    # Add disturbances at proportional positions within the data
    distA_start = int(0.500 * leng)
    distA_end   = min(int(0.667 * leng) + 1, leng)
    distA_len   = distA_end - distA_start
    if distA_len > 1:
        generated_acc[distA_start:distA_end, 1] += (
            dist_acc_intensity * 0.5 * np.sin(np.linspace(0, 6 * np.pi, distA_len))
        )

    distM_start = int(0.750 * leng)
    distM_end   = min(int(0.917 * leng) + 1, leng)
    distM_len   = distM_end - distM_start
    if distM_len > 1:
        generated_mag[distM_start:distM_end, 0] += (
            dist_mag_intensity * 0.5 * np.sin(np.linspace(0, 6 * np.pi, distM_len))
        )
    
    # Return data dictionary
    data = {
        'accelerometer': generated_acc,
        'magnetometer': generated_mag,
        'gyroscope': generated_gyr,
        'reference': quat,
        'time': time,
        'omega_world': omega_world,
        'acceleration_world': acceleration_world,
        'sensor_pose': sensor_pose_world
    }
    
    return data

def sim_rigid_body_chirp_rot(sensor_pose=np.array([0.0, 0.0, 0.0]), sen_params=SimSensorParameters(), rot_axis=np.array([0.1, 0.2, 0.05]),
                             max_omega=10.0):
    """
    Generate synthetic IMU sensor data with rigid body rotational acceleration. The rotation frequency will chirp from low to high 
    and than slow down again, simulating a more dynamic movement. Omega magnitude will get triangular shape.
    
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
    # Simulation parameters
    leng = 12000
    dt = 0.0039  # seconds (approximately 256 Hz)
    
    # Preallocate arrays
    generated_mag = np.zeros((leng, 3))
    generated_acc = np.zeros((leng, 3))
    generated_gyr = np.zeros((leng, 3))
    omega_magnitudes = np.zeros((leng, 1))
    quat = np.zeros((leng, 4))
    time = np.zeros(leng)
    omega_world = np.zeros((leng, 3))  # Angular velocity in world frame
    omega_dot_world = np.zeros((leng, 3))  # Angular velocity in world frame
    acceleration_world = np.zeros((leng, 3))  # Total acceleration in world frame
    
    # Initial conditions
    quat[0] = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
    time[0] = 0.0
    
    # Rotation direction (constant in world frame)
    direction = rot_axis
    direction = direction / np.linalg.norm(direction)
    
    # Sensor bias offsets
    mag_offset = np.array([sen_params.mag_off_err, -sen_params.mag_off_err, sen_params.mag_off_err / 2])
    acc_offset = np.array([-sen_params.acc_off_err, sen_params.acc_off_err, sen_params.acc_off_err / 2])
    gyr_offset = np.array([-sen_params.gyr_off_err, sen_params.gyr_off_err, sen_params.gyr_off_err / 2])
    
    chirp_period = 10.0  # seconds for one full chirp cycle (low to high to low)
    chirp_start_time = 20.0  # seconds when chirp starts

    second_chirp_start_time = chirp_start_time + chirp_period  # seconds when second chirp starts

    # Generate data
    for i in range(leng - 1):
        # Phase transition: stop rotation between 15-20s when acc.x is small
        time[i + 1] = time[i] + dt

        if time[i] < chirp_start_time:
            omega_magnitude = 0
        elif time[i] >= chirp_start_time and time[i] < second_chirp_start_time:
            omega_magnitude = max_omega * (time[i] - chirp_start_time) / chirp_period
        elif time[i] >= second_chirp_start_time and time[i] < second_chirp_start_time + chirp_period:
            omega_magnitude = max_omega * (1 - (time[i] - second_chirp_start_time) / chirp_period)
        else:
            omega_magnitude = 0
        
        omega_magnitudes[i] = omega_magnitude
        # Generate data for each sensor
        # Compute angular velocity in world frame
        omega_world[i] = omega_magnitude * direction
        omega_diff = omega_world[i] - omega_world[i - 1] if i > 0 else np.array([0.0, 0.0, 0.0])
        omega_dot_world[i] = omega_diff / dt


        sensor_pose_world = quaternion_rotate_vector(quat[i], sensor_pose)
        
        # Acceleration due to rotation (in world frame)
        a_rotational_world = compute_rotational_acceleration(
            omega_world[i], 
            omega_dot_world[i], 
            sensor_pose_world
        )

        acceleration_world[i] = sen_params.acc_null + a_rotational_world / 9.81
        
        # Transform total acceleration to body frame
        acc_body = quaternion_rotate_vector(quat[i], acceleration_world[i])
        
        # Generate accelerometer reading (in body frame)
        generated_acc[i] = (
            (1 + sen_params.acc_scale_err) * acc_body +
            np.random.randn(3) * sen_params.acc_noise_err +
            acc_offset
        )
        
        # Generate magnetometer reading (in body frame)
        mag_body = quaternion_rotate_vector(quat[i], sen_params.mag_null)
        generated_mag[i] = (
            -(1 + sen_params.mag_scale_err) * mag_body +
            np.random.randn(3) * sen_params.mag_noise_err +
            mag_offset
        )

        omega_body = quaternion_rotate_vector(quatern_conj_single(quat[i]), omega_world[i])
            
        # Integrate quaternion using RK4
        quat[i + 1] = integrate_rk4(quat[i], omega_body, dt)
        
        # Ensure positive scalar part
        if quat[i + 1, 0] < 0:
            quat[i + 1] = -quat[i + 1]

        generated_gyr[i] = (
            (1 + sen_params.gyr_scale_err) * omega_body +
            np.random.randn(3) * np.deg2rad(sen_params.gyr_noise_err) + gyr_offset
        )
    
    # Handle last sample
    omega_world[-1] = omega_world[-2]
    omega_dot_world[-1] = omega_dot_world[-2]
    acceleration_world[-1] = acceleration_world[-2]
    generated_mag[-1] = generated_mag[-2]
    generated_gyr[-1] = generated_gyr[-2]
    generated_acc[-1] = generated_acc[-2]

    st_idx = np.searchsorted(time, chirp_start_time)

    data = {
        'accelerometer': generated_acc,
        'magnetometer': generated_mag,
        'gyroscope': generated_gyr,
        'reference': quat,
        'time': time,
        'omega_world': omega_world,
        'omega_dot_world': omega_dot_world,
        'acceleration_world': acceleration_world,
        'sensor_pose': sensor_pose_world,
        'omega_magnitudes': omega_magnitudes,
        'start_time':  {'seconds': chirp_start_time, 'index': st_idx},
        'mean_sampling_rate': 1.0 / dt
    }
    return data

def generate_from_real_reference(data_real, sensor_pose=np.array([0.0, 0.0, 0.0]), sen_params=SimSensorParameters(mag_intensity=1), rot_axis=np.array([0.1, 0.2, 0.05]),
                             max_omega=10.0):
    # Load real dataset

    
    # Extract reference quaternion and time
    quat_ref = data_real['reference']
    time = data_real['time']
    
    # Compute angular velocity in WORLD frame from quaternion reference.
    # q_i ⊗ q_{i-1}^{-1} gives the incremental rotation in world frame (B2W convention),
    # whereas q_{i-1}^{-1} ⊗ q_i would give body-frame omega.
    omega_world = np.zeros((len(time), 3))
    for i in range(1, len(time)):
        q_diff = quatern_prod_single(quat_ref[i], quatern_conj_single(quat_ref[i - 1]))
        if q_diff[0] < 0:
            q_diff = -q_diff
        omega_world[i] = 2.0 * q_diff[1:4] / (time[i] - time[i - 1])
    
    # Generate synthetic sensor data driven by the real reference motion
    data_synthetic = sim_rigid_body_rot_with_disturb(
        sensor_pose=sensor_pose,
        sen_params=sen_params,
        rot_axis=rot_axis,
        dist_mag_intensity=0.01,
        dist_acc_intensity=0.02,
        omega_world_ext=omega_world,
        quat_ref_ext=quat_ref,
        time_ext=time,
    )
    
    return data_synthetic

#data = create_sensor_data_simulation()
# data = sim_rigid_body_rot_with_disturb(sensor_pose=np.array([1.2, 1.2, 1.0]),\
#                                                 gyro_noise=0.1, gyro_offset=0.2, gyro_scale=5e-3,
#                                                 rot_axis=np.array([0.0, 0.0, 1.0]))

# data = sim_rigid_body_chirp_rot(sensor_pose=np.array([0.1, 0.0, 0.0]),\
#                                                 rot_axis=np.array([0.1, 0.2, 0.05]), max_omega=5.0)
dataset_loader = DatasetLoader()
data_real = dataset_loader.load_justa_raw(1)
data = generate_from_real_reference(data_real, sensor_pose=np.array([0.1, 0.0, 0.0]),\
                                                rot_axis=np.array([0.1, 0.2, 0.05]), max_omega=5.0)

#pickle.dump(data, open('synthetic_rigid_body_sensor_offset.pkl', 'wb'))
#plot_dataset(data)


# compare acc
fig, ax = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

show_acc = False
show_mag = True
if show_acc:
    ax[0].plot(data['time'], data['accelerometer'][:, 0], label='Acc X')
    ax[0].plot(data['time'], data_real['accelerometer'][:, 0], label='Acc real X')
    ax[1].plot(data['time'], data['accelerometer'][:, 1], label='Acc Y')
    ax[1].plot(data['time'], data_real['accelerometer'][:, 1], label='Acc real Y')
    ax[2].plot(data['time'], data['accelerometer'][:, 2], label='Acc Z')
    ax[2].plot(data['time'], data_real['accelerometer'][:, 2], label='Acc real Z')
else:
    ax[0].plot(data['time'], data['magnetometer'][:, 0], label='Mag X')
    ax[0].plot(data['time'], data_real['magnetometer'][:, 0], label='Mag real X')
    ax[1].plot(data['time'], data['magnetometer'][:, 1], label='Mag Y')
    ax[1].plot(data['time'], data_real['magnetometer'][:, 1], label='Mag real Y')
    ax[2].plot(data['time'], data['magnetometer'][:, 2], label='Mag Z')
    ax[2].plot(data['time'], data_real['magnetometer'][:, 2], label='Mag real Z')

plt.legend()
plt.show()