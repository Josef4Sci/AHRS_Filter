"""
Quaternion library functions for AHRS filters
Ported from MATLAB to Python
"""
import numpy as np


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
    
    if len(ab) == 1:
        return ab[0]
    return ab


def quatern_conj(q):
    """
    Calculate quaternion conjugate.
    
    Args:
        q: Quaternion(s) as numpy array [w, x, y, z]
        
    Returns:
        Conjugate quaternion
    """
    if q.ndim == 1:
        return np.array([q[0], -q[1], -q[2], -q[3]])
    else:
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
