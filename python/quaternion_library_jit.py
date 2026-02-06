"""
Quaternion library functions for AHRS filters
Ported from MATLAB to Python
"""
import numpy as np
from numba import jit

@jit(nopython=True, cache=True, fastmath=True, inline='always')
def fast_norm(v):
    """Faster norm calculation for small vectors"""
    return np.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])

@jit(nopython=True, cache=True, fastmath=True, inline='always')
def fast_normalize_3d(v):
    """Fast normalization with early return check"""
    norm = fast_norm(v)
    if norm < 1e-10:  # Avoid division by zero
        return np.zeros(3, dtype=np.float64), False
    return v / norm, True

@jit(nopython=True, cache=True, fastmath=True, inline='always')
def fast_normalize_4d(v):
    """Fast 4D normalization"""
    norm = np.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3])
    return v / norm

@jit(nopython=True, cache=True)
def quaternion_derivative(q, gyro):
    w, x, y, z = q
    wx, wy, wz = gyro
    q_dot = 0.5 * np.array([
        -x*wx - y*wy - z*wz,
            w*wx + y*wz - z*wy,
            w*wy - x*wz + z*wx,
            w*wz + x*wy - y*wx
    ])
    return q_dot

@jit(nopython=True, cache=True)
def integrate_rk4(q, gyro, dt):
    """
    4th order Runge-Kutta integration for quaternion from previous answer.
    
    Parameters:
    -----------
    q : np.ndarray, shape (4,)
        Current quaternion [w, x, y, z]
    gyro : np.ndarray, shape (3,)
        Angular velocity [wx, wy, wz] in rad/s
    dt : float
        Time step in seconds
    
    Returns:
    --------
    q_new : np.ndarray, shape (4,)
        Updated quaternion [w, x, y, z], normalized
    """
    
    k1 = quaternion_derivative(q, gyro)
    
    q2 = q + 0.5 * k1 * dt
    q2 = q2 / np.linalg.norm(q2)
    k2 = quaternion_derivative(q2, gyro)
    
    q3 = q + 0.5 * k2 * dt
    q3 = q3 / np.linalg.norm(q3)
    k3 = quaternion_derivative(q3, gyro)
    
    q4 = q + k3 * dt
    q4 = q4 / np.linalg.norm(q4)
    k4 = quaternion_derivative(q4, gyro)
    
    q_new = q + (k1 + 2*k2 + 2*k3 + k4) * dt / 6
    q_new = q_new / np.linalg.norm(q_new)
    
    return q_new

@jit(nopython=True, cache=True)
def quaternion_rotate_vector(q, v):
    """
    Rotate vector v by quaternion q.
    
    v_rotated = q^-1 ⊗ [0, v] ⊗ q
    
    Parameters:
    -----------
    q : np.ndarray, shape (4,)
        Quaternion [w, x, y, z]
    v : np.ndarray, shape (3,)
        Vector to rotate
    
    Returns:
    --------
    v_rotated : np.ndarray, shape (3,)
        Rotated vector
    """
    q_conj = quatern_conj_single(q)
    v_quat = np.array([0.0, v[0], v[1], v[2]])
    result = quatern_prod_single(q_conj, quatern_prod_single(v_quat, q))
    return result[1:4]

@jit(nopython=True, cache=True)
def quatern_prod(a, b):
    """
    Calculates the quaternion product of quaternion a and b.
    Quaternion format: [w, x, y, z]
    
    Args:
        a: First quaternion(s) as numpy array
        b: Second quaternion(s) as numpy array
        
    Returns:
        Product quaternion ab
    """
    if a.ndim == 1:
        a = a.reshape(1, -1)
    if b.ndim == 1:
        b = b.reshape(1, -1)
    
    ab = np.zeros((len(a), 4))
    
    ab[:, 0] = a[:, 0] * b[:, 0] - a[:, 1] * b[:, 1] - a[:, 2] * b[:, 2] - a[:, 3] * b[:, 3]
    ab[:, 1] = a[:, 0] * b[:, 1] + a[:, 1] * b[:, 0] + a[:, 2] * b[:, 3] - a[:, 3] * b[:, 2]
    ab[:, 2] = a[:, 0] * b[:, 2] - a[:, 1] * b[:, 3] + a[:, 2] * b[:, 0] + a[:, 3] * b[:, 1]
    ab[:, 3] = a[:, 0] * b[:, 3] + a[:, 1] * b[:, 2] - a[:, 2] * b[:, 1] + a[:, 3] * b[:, 0]
    
    return ab

@jit(nopython=True, cache=True)
def quatern_prod_single(a, b):
    """
    Calculates the quaternion product of quaternion a and b.
    Quaternion format: [w, x, y, z]
    
    Args:
        a: First quaternion(s) as numpy array
        b: Second quaternion(s) as numpy array
        
    Returns:
        Product quaternion ab
    """

    ab = np.zeros(4)
    
    ab[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3]
    ab[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2]
    ab[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1]
    ab[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
    
    return ab


@jit(nopython=True, cache=True)
def quatern_conj_single(q):
    """
    Calculate quaternion conjugate.
    
    Args:
        q: Quaternion [w, x, y, z]
        
    Returns:
        Conjugate quaternion
    """
    quat_c = np.copy(q)
    quat_c[1:] = -quat_c[1:]
    return quat_c

@jit(nopython=True, cache=True)
def quatern_conj(q):
    """
    Calculate quaternion conjugate.
    
    Args:
        q: Quaternion(s) as numpy array [w, x, y, z]
        
    Returns:
        Conjugate quaternion
    """
    quat_c = np.copy(q)
    quat_c[:, 1:] = -quat_c[:, 1:]
    return quat_c


def measurement_quaternion_acc_mag(acc, mag, mag_r, q_):
    """
    Fast Kalman Filter for Attitude Estimation measurement function
    Author: Jin Wu
    
    Args:
        acc: Accelerometer measurement [ax, ay, az]
        mag: Magnetometer measurement [mx, my, mz]
        mag_r: Reference magnetic field [mN, 0, mD]
        q_: Prior quaternion [q0, q1, q2, q3]
        
    Returns:
        q: Measurement quaternion
        Jacob: Jacobian matrix
    """
    ax, ay, az = acc
    mx, my, mz = mag
    mN, _, mD = mag_r
    q0, q1, q2, q3 = q_
    
    q = np.zeros(4)
    Jacob = np.zeros((4, 6))
    
    q[0] = ((ay*mD*my + (1 + az)*(1 + mN*mx + mD*mz) + ax*(mD*mx - mN*mz))*q0 + 
            ((mD + az*mD - ax*mN)*my + ay*(1 + mN*mx - mD*mz))*q1 + 
            (ay*mN*my + ax*(-1 + mN*mx + mD*mz) + (1 + az)*(-(mD*mx) + mN*mz))*q2 + 
            (-((ax*mD + mN + az*mN)*my) + ay*(mD*mx + mN*mz))*q3)
    
    q[1] = (((mD - az*mD - ax*mN)*my + ay*(1 + mN*mx + mD*mz))*q0 + 
            (ay*mD*my - (-1 + az)*(1 + mN*mx - mD*mz) + ax*(mD*mx + mN*mz))*q1 + 
            ((ax*mD + mN - az*mN)*my + ay*(-(mD*mx) + mN*mz))*q2 + 
            (-(ay*mN*my) + ax*(1 - mN*mx + mD*mz) - (-1 + az)*(mD*mx + mN*mz))*q3)
    
    q[2] = ((-(ay*mN*my) - ax*(1 + mN*mx + mD*mz) + (-1 + az)*(mD*mx - mN*mz))*q0 + 
            ((-(ax*mD) + mN - az*mN)*my + ay*(mD*mx + mN*mz))*q1 + 
            (ay*mD*my + (-1 + az)*(-1 + mN*mx + mD*mz) + ax*(mD*mx - mN*mz))*q2 + 
            ((mD - az*mD + ax*mN)*my + ay*(1 - mN*mx + mD*mz))*q3)
    
    q[3] = (ax*(q1 + mN*mx*q1 + mN*my*q2 + mN*mz*q3 + mD*(my*q0 - mz*q1 + mx*q3)) + 
            (1 + az)*(mD*mx*q1 + mD*my*q2 + q3 + mD*mz*q3 - mN*(my*q0 - mz*q1 + mx*q3)) + 
            ay*(mN*mz*q0 + mN*my*q1 + q2 - mN*mx*q2 - mD*(mx*q0 + mz*q2 - my*q3)))
    
    # Jacobian matrix
    Jacob[0, 0] = -q2 - mN*(mz*q0 + my*q1 - mx*q2) + mD*(mx*q0 + mz*q2 - my*q3)
    Jacob[0, 1] = q1 + mN*mx*q1 + mN*my*q2 + mN*mz*q3 + mD*(my*q0 - mz*q1 + mx*q3)
    Jacob[0, 2] = q0 + mN*mx*q0 + mD*mz*q0 + mD*my*q1 - mD*mx*q2 + mN*mz*q2 - mN*my*q3
    Jacob[0, 3] = (ax*mD + mN + az*mN)*q0 + ay*mN*q1 + (-((1 + az)*mD) + ax*mN)*q2 + ay*mD*q3
    Jacob[0, 4] = ay*mD*q0 + (mD + az*mD - ax*mN)*q1 + ay*mN*q2 - (ax*mD + mN + az*mN)*q3
    Jacob[0, 5] = mD*(q0 + az*q0 - ay*q1 + ax*q2) + mN*(-(ax*q0) + q2 + az*q2 + ay*q3)
    
    Jacob[1, 0] = q3 - mN*(my*q0 - mz*q1 + mx*q3) + mD*(mx*q1 + my*q2 + mz*q3)
    Jacob[1, 1] = q0 + mN*mx*q0 + mD*mz*q0 + mD*my*q1 - mD*mx*q2 + mN*mz*q2 - mN*my*q3
    Jacob[1, 2] = -((1 + mN*mx)*q1) - mD*(my*q0 - mz*q1 + mx*q3) - mN*(my*q2 + mz*q3)
    Jacob[1, 3] = ay*(mN*q0 - mD*q2) - (-1 + az)*(mN*q1 + mD*q3) + ax*(mD*q1 - mN*q3)
    Jacob[1, 4] = mD*(q0 - az*q0 + ay*q1 + ax*q2) - mN*(ax*q0 + (-1 + az)*q2 + ay*q3)
    Jacob[1, 5] = ay*(mD*q0 + mN*q2) + mD*((-1 + az)*q1 + ax*q3) + mN*(ax*q1 + q3 - az*q3)
    
    Jacob[2, 0] = -((1 + mN*mx + mD*mz)*q0) - mD*my*q1 + mD*mx*q2 - mN*mz*q2 + mN*my*q3
    Jacob[2, 1] = q3 - mN*(my*q0 - mz*q1 + mx*q3) + mD*(mx*q1 + my*q2 + mz*q3)
    Jacob[2, 2] = -q2 - mN*(mz*q0 + my*q1 - mx*q2) + mD*(mx*q0 + mz*q2 - my*q3)
    Jacob[2, 3] = mD*((-1 + az)*q0 + ay*q1 + ax*q2) - mN*(ax*q0 + q2 - az*q2 + ay*q3)
    Jacob[2, 4] = ay*(-(mN*q0) + mD*q2) - (-1 + az)*(mN*q1 + mD*q3) + ax*(-(mD*q1) + mN*q3)
    Jacob[2, 5] = mN*(q0 - az*q0 + ay*q1) - ax*(mD*q0 + mN*q2) + mD*((-1 + az)*q2 + ay*q3)
    
    Jacob[3, 0] = mN*(mx*q1 + my*q2 + mz*q3) - mD*(my*q0 - mz*q1 + mx*q3)
    Jacob[3, 1] = mN*mx*q0 - mD*mz*q0 - mD*my*q1 + mD*mx*q2 - mN*mz*q2 + mN*my*q3
    Jacob[3, 2] = q1 + mN*(-(mz*q0) - my*q1 + mx*q2) + mD*(mx*q0 + mz*q2 - my*q3)
    Jacob[3, 3] = ay*(mD*q0 + mN*q2) + mN*((1 + az)*q1 + ax*q3) + mD*(ax*q1 - q3 + az*q3)
    Jacob[3, 4] = mN*(q0 + az*q0 + ay*q1) - ax*(mD*q0 + mN*q2) - mD*((1 + az)*q2 + ay*q3)
    Jacob[3, 5] = ay*(mN*q0 - mD*q2) + (1 + az)*(mD*q1 + mN*q3) + ax*(-(mD*q1) + mN*q3)
    
    return q, Jacob


def kalman_update(xk_1, yk, Pk_1, Phi_k, Xi_k, Eps_k):
    """
    Kalman filter update step
    
    Args:
        xk_1: Previous state estimate
        yk: Measurement
        Pk_1: Previous covariance
        Phi_k: State transition matrix
        Xi_k: Process noise covariance
        Eps_k: Measurement noise covariance
        
    Returns:
        xk: Updated state estimate
        Pk: Updated covariance
    """
    x_ = Phi_k @ xk_1
    Pk_ = Phi_k @ Pk_1 @ Phi_k.T + Xi_k
    Gk = Pk_ @ np.linalg.inv(Pk_ + Eps_k)
    Pk = (np.eye(4) - Gk) @ Pk_
    xk = x_ + Gk @ (yk - x_)
    
    return xk, Pk
