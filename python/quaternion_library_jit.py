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

@jit(nopython=True, cache=True,fastmath=True, inline='always')
def fast_cross(a, b):
    """Fast cross product for 3D vectors"""
    return np.array([
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    ], dtype=np.float64)

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


# Method 1: Simple Euler Integration (fastest, least accurate)
@jit(nopython=True, cache=True)
def integrate_euler(q, gyro, dt):
    """Simple Euler integration: q_new = q + q_dot * dt"""
    q_dot = quaternion_derivative(q, gyro)
    q_new = q + q_dot * dt    
    return fast_normalize_4d(q_new)


# Method 2: Midpoint Method (good balance)
@jit(nopython=True, cache=True)
def integrate_midpoint(q, gyro, dt):
    """Midpoint integration (2nd order Runge-Kutta)"""
    # First estimate
    k1 = quaternion_derivative(q, gyro)
    
    # Midpoint
    q_mid = q + 0.5 * k1 * dt
    q_mid = fast_normalize_4d(q_mid)  # Normalize midpoint
    
    # Second estimate
    k2 = quaternion_derivative(q_mid, gyro)
    
    q_new = q + k2 * dt
    return fast_normalize_4d(q_new)

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
    q2 = fast_normalize_4d(q2)
    k2 = quaternion_derivative(q2, gyro)
    
    q3 = q + 0.5 * k2 * dt
    q3 = fast_normalize_4d(q3)
    k3 = quaternion_derivative(q3, gyro)
    
    q4 = q + k3 * dt
    q4 = fast_normalize_4d(q4)
    k4 = quaternion_derivative(q4, gyro)
    
    q_new = q + (k1 + 2*k2 + 2*k3 + k4) * dt / 6
    q_new = fast_normalize_4d(q_new)
    
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
