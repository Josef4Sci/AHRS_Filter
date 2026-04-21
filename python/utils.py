import numpy as np
from quaternion_library_jit import fast_cross, fast_norm, quatern_conj_single, quatern_prod_single, quatern_prod, quatern_conj, quaternion_rotate_vector
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation, Slerp


def interpolate_with_scipy(quaternions, weights):
    """
    Use scipy's Slerp for robust interpolation.
    
    Args:
        quaternions: array of shape (N, 4) as [w, x, y, z] (scipy convention)
        weights: array of shape (N,) with weights (should sum to 1)
    """
    # Normalize weights
    weights = np.array(weights)
    weights = weights / np.sum(weights)
    
    # Create time points for each quaternion
    times = np.arange(len(quaternions))
    
    # Create Slerp interpolator
    rotations = Rotation.from_quat(quaternions, scalar_first=True)  # expects [w, x, y, z]
    slerp = Slerp(times, rotations)
    
    # Compute weighted interpolation point
    weighted_time = np.sum(times * weights)
    
    # Interpolate
    result_rotation = slerp(weighted_time)
    
    return result_rotation.as_quat(scalar_first=True)  # Return in [w, x, y, z] format

def qdiff(q1, q2):
    return quatern_prod(q1, quatern_conj(q2))

def qdiff_single(q1, q2):
    return quatern_prod_single(q1, quatern_conj_single(q2))

def angle_diff_deg(qdiff):
    angle_error = np.abs(2 * np.arctan2(
        np.linalg.norm(qdiff[:, 1:4], axis=1),
        qdiff[:, 0]
    ) * 180 / np.pi)
    return angle_error



def vector_angle_deg(a, b):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)

    na = np.linalg.norm(a)
    nb = np.linalg.norm(b)
    if na == 0 or nb == 0:
        raise ValueError("Angle is undefined for a zero-length vector")

    cos_theta = np.dot(a, b) / (na * nb)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)  # numerical safety
    return np.degrees(np.arccos(cos_theta))

def angle_error(q_est, q_ref, use_imu=False, align_start=False, shift_samples=0, align_index=20):
    
    min_norm = np.min(np.linalg.norm(q_est, axis=1))
    if min_norm < 0.9 or min_norm > 1.1:
        print(f"Warning: Quaternion norms deviate from 1 (min norm: {min_norm:.3f}). Check if quaternions are normalized and in [w, x, y, z] format.")
        assert False, "Quaternion norms are not close to 1. Check input quaternions."
    
    if align_start:
        fix_heading_quat = quatern_prod_single(q_ref[align_index,:], quatern_conj_single(q_est[align_index,:]))
        fix_heading_quat_array = np.tile(fix_heading_quat, (q_ref.shape[0], 1))
        q_est_in = quatern_prod(fix_heading_quat_array, q_est)
    else:
        q_est_in = q_est

    if not use_imu:
        # implement shift by multiplying with conjugate of reference at shift_samples
        if shift_samples > 0:        
            q_err = qdiff(q_ref[shift_samples:,:], q_est_in[0:-(shift_samples),:])
        elif shift_samples < 0:
            q_err = qdiff(q_ref[0:shift_samples,:], q_est_in[-shift_samples:,:])
        else:
            q_err = qdiff(q_ref, q_est_in)
        
        # Ensure all quaternions have positive w component
        q_err[q_err[:, 0] < 0] = -q_err[q_err[:, 0] < 0]
        
        # Calculate angular error
        angle_error = angle_diff_deg(q_err)    
    else:
        acc_ref = np.array([0, 0, 1])
        angle_error = np.zeros(q_est_in.shape[0])
        for i in range(q_est_in.shape[0]):
            est_acc = quaternion_rotate_vector(q_est_in[i], acc_ref)
            ref_acc = quaternion_rotate_vector(q_ref[i], acc_ref)
            angle_error[i] = vector_angle_deg(est_acc, ref_acc)
        
    return angle_error

def eval_filter_on_dataset(filter_instance, dataset, use_imu=False, use_square_err=False):
    # Reset quaternion to reference at start
    #filter_instance.quaternion = dataset['reference'][0].copy()
    
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
        
    return quaternion_result


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



def vectors_to_quaternion(v1, v2):
    """
    Find quaternion that rotates v1 to v2 exactly.
    
    Parameters:
    -----------
    v1, v2 : array-like, shape (3,)
        Normalized 3D vectors
        
    Returns:
    --------
    q : array, shape (4,)
        Quaternion [w, x, y, z]
    """
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    
    # Handle parallel/anti-parallel cases
    dot = np.dot(v1, v2)
    if dot > 0.9999:
        return np.array([1, 0, 0, 0])  # Identity rotation
    elif dot < -0.9999:
        # 180-degree rotation around perpendicular axis
        axis = np.array([1, 0, 0]) if abs(v1[0]) < 0.9 else np.array([0, 1, 0])
        axis = np.cross(v1, axis)
        axis /= np.linalg.norm(axis)
        return np.array([0, *axis])
    
    # General case
    axis = np.cross(v1, v2)
    axis_norm = np.linalg.norm(axis)
    angle = np.arctan2(axis_norm, dot)
    
    if axis_norm > 0:
        axis = axis / axis_norm
    
    half_angle = angle / 2
    return np.array([
        np.cos(half_angle),
        axis[0] * np.sin(half_angle),
        axis[1] * np.sin(half_angle),
        axis[2] * np.sin(half_angle)
    ])


def wahba_constrained(ref1, obs1, ref2, obs2):
    """
    Solve weighted Wahba problem with exact fit constraint.
    
    Parameters:
    -----------
    ref1 : array, shape (3,)
        First reference vector (will fit EXACTLY)
    obs1 : array, shape (3,)
        First observation vector
    ref2 : array, shape (3,)
        Second reference vector (minimize error with weight)
    obs2 : array, shape (3,)
        Second observation vector
        
    Returns:
    --------
    q_best : array, shape (4,)
        Optimal quaternion [w, x, y, z]
    error : float
        Error for second vector
    """
    # Normalize all vectors
    ref1 = ref1 / np.linalg.norm(ref1)
    obs1 = obs1 / np.linalg.norm(obs1)
    ref2 = ref2 / np.linalg.norm(ref2)
    obs2 = obs2 / np.linalg.norm(obs2)
    
    # Step 1: Get rotation that maps obs1 to ref1 exactly
    q_exact = vectors_to_quaternion(obs1, ref1)
    
    # Step 2: Parameterize additional rotation around ref1 axis
    # Total rotation = R(ref1, theta) @ R_exact
    
    def cost_function(theta):
        # Create rotation around ref1 axis
        axis_rot = Rotation.from_rotvec(theta[0] * ref1)
        base_rot = Rotation.from_quat(q_exact, scalar_first=True)  # [w, x, y, z] format
        
        # Combine rotations
        total_rot = axis_rot * base_rot
        
        # Apply to obs2
        rotated_obs2 = total_rot.apply(obs2)
        
        # Weighted error
        error = np.linalg.norm(rotated_obs2 - ref2)**2
        return error
    
    from scipy.spatial.transform import Rotation
    from scipy.optimize import minimize
    # Optimize angle around ref1 axis
    result = minimize(cost_function, x0=[0.0], method='L-BFGS-B')
    
    # Build final rotation
    theta_opt = result.x[0]
    axis_rot = Rotation.from_rotvec(theta_opt * ref1)
    base_rot = Rotation.from_quat(q_exact, scalar_first=True)
    final_rot = axis_rot * base_rot
    
    q_best = final_rot.as_quat(scalar_first=True)  # [w, x, y, z]
    
    # Verify and compute final error
    rotated2 = final_rot.apply(obs2)
    error2 = np.linalg.norm(rotated2 - ref2)
    
    return q_best, error2
