import pickle
import matplotlib.pyplot as plt
import pandas as pd
import scipy.io
import numpy as np

from utils import angle_error
from scipy.optimize import minimize, differential_evolution
from quaternion_library import quatern_conj_single, quatern_prod_single, quatern_prod, quatern_conj

#with open('quaternion_comparison.pkl', 'wb') as f:

def align_quaternions(data, q_left, q_right):
    #normalize quaternions
    q_left = q_left / np.linalg.norm(q_left)
    q_right = q_right / np.linalg.norm(q_right)
    
    quaternion_justa =  data['quaternion_justa']
    quaternion_vqf = data['quaternion_vqf']
    
    quats_left = np.tile(q_left, (len(quaternion_justa), 1))
    quats_right = np.tile(q_right, (len(quaternion_justa), 1))
    
    # Apply the transformation to the VQF quaternion
    transformed_j = quatern_prod(quatern_prod(quats_left, quaternion_justa), quats_right)
    transformed_vqf = quatern_prod(quatern_prod(quats_left, quatern_conj(quaternion_vqf)), quats_right)
    return transformed_j, transformed_vqf
    

def objective_function(params, data):
    """
    Find out two quaternions, left and right side product to minimize the error.
    """
    q_left = np.array([params[0], params[1], params[2], params[3]])
    q_right = np.array([params[4], params[5], params[6], params[7]])
    
    reference = data['reference']
    transformed_j, transformed_vqf = align_quaternions(data, q_left, q_right)
    
    angle_err = angle_error(transformed_j, reference, align_start=False, shift_samples=0)
    angle_err_vqf = angle_error(transformed_vqf, reference, align_start=False, shift_samples=0)
    
    #print(f"Quaternion left: {q_left}, Quaternion right: {q_right}, Mean angle error: {np.mean(angle_err):.4f} deg")
    return np.mean(angle_err_vqf)
    

f = open('quaternion_comparison.pkl', 'rb')
data = pickle.load(f)

initial_guess = [1, 0, 0, 0, 1, 0, 0, 0]

bounds = [(0, 1.0), (-1.0, 1.0),(-1.0, 1.0),(-1.0, 1.0), (0.0, 1.0),(-1.0, 1.0),(-1.0, 1.0),(-1.0, 1.0)]
    
result = minimize(
    objective_function,
    initial_guess,
    method='L-BFGS-B',
    bounds=bounds,
    args=(data,),        
    options={'maxiter': 2000, 'xatol': 1e-6, 'fatol': 1e-6, 'disp': True}
    #bounds=[(0, 1.0), (0, 1.0), (0.0, 1.0)]
)

q_left_opt = np.array(result.x[0:4])
q_right_opt = np.array(result.x[4:8])

transformed_j, transformed_vqf = align_quaternions(data, q_left_opt, q_right_opt)

print(f"Optimized Quaternion left: {q_left_opt} \n Optimized Quaternion right: {q_right_opt} \n Mean angle error: {result.fun:.4f} deg")


angle_err = angle_error(transformed_j, data['reference'], align_start=True, shift_samples=0)
angle_error_vqf = angle_error(transformed_vqf, data['reference'], align_start=True, shift_samples=0)
plt.plot(angle_err, label='Transformed Justa Error')
plt.plot(angle_error_vqf, label='Transformed VQF Error')
plt.legend()
plt.show()