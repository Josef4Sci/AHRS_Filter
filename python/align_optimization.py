import pickle
import matplotlib.pyplot as plt
import pandas as pd
import scipy.io
import numpy as np

from filters.justa_ahrs import JustaAHRSv4
from dataset_loader import DatasetLoader
from utils import angle_error, eval_filter_on_dataset
from scipy.optimize import minimize, differential_evolution
from quaternion_library import quatern_conj_single, quatern_prod_single, quatern_prod, quatern_conj
from vqf import PyVQF

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
    transformed_vqf = quatern_prod(quatern_prod(quats_left, quaternion_vqf), quats_right)
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
    return np.mean(angle_err_vqf) + np.mean(angle_err)
    

# f = open('quaternion_comparison.pkl', 'rb')
# data = pickle.load(f)
dl = DatasetLoader()
# dataset_name = 'slow_v4.mat'
# dat = dl.load_sassari_dataset(dataset_name, 2)


q_l = []
q_r = []
angle_errs = []

broad_white = dl.broad_white_list_datasets()

for i in range(4):
    # dataset_name = 'medium_v4.mat'
    # dataset_name = 'fast_v4.mat'
    # data = dl.load_sassari_dataset(dataset_name, 2)
    
    
    file = broad_white[i]
    data = dl.load_broad_dataset(file_name=file)
    
    b = PyVQF(1.0/data['mean_sampling_rate'], tauAcc=0.994, tauMag=1.44, motionBiasEstEnabled=False, restBiasEstEnabled=False, magDistRejectionEnabled=False)
    #b.state['gyrQuat'] = dat['reference'][0]

    j_filter = JustaAHRSv4(w_acc=0.001379, w_mag= 0.000072)

    gyr = np.ascontiguousarray(data['gyroscope'], dtype=np.float64)
    acc = np.ascontiguousarray(data['accelerometer'], dtype=np.float64)
    mag = np.ascontiguousarray(data['magnetometer'], dtype=np.float64)
    res = b.updateBatch(gyr, acc, mag)
    j_filter.initFromAccMag(data['accelerometer'][0], data['magnetometer'][0])
    quaternion_result = eval_filter_on_dataset(j_filter, data)

    #optim
    initial_guess = [1, 0, 0, 0, 1, 0, 0, 0]
    bounds = [(0, 1.0), (-1.0, 1.0),(-1.0, 1.0),(-1.0, 1.0), (0.0, 1.0),(-1.0, 1.0),(-1.0, 1.0),(-1.0, 1.0)]

    pikle_data = {
        'time': data['time'],
        'quaternion_justa': quaternion_result,
        'quaternion_vqf': res['quat9D'],
        'reference': data['reference']
    }

    result = minimize(
        objective_function,
        initial_guess,
        method='L-BFGS-B',
        bounds=bounds,
        args=(pikle_data,),        
        options={'maxiter': 2000, 'xatol': 1e-6, 'fatol': 1e-6, 'disp': True}
        #bounds=[(0, 1.0), (0, 1.0), (0.0, 1.0)]
    )

    q_left_opt = np.array(result.x[0:4])
    q_right_opt = np.array(result.x[4:8])


    q_left_opt = q_left_opt / np.linalg.norm(q_left_opt)
    q_right_opt = q_right_opt / np.linalg.norm(q_right_opt)

    print(f"Optimized Quaternion left: {q_left_opt} \n Optimized Quaternion right: {q_right_opt} \n Mean angle error: {result.fun:.4f} deg")
    q_l.append(q_left_opt)
    q_r.append(q_right_opt)
    angle_errs.append(result.fun)

    if False:
        transformed_j, transformed_vqf = align_quaternions(data, q_left_opt, q_right_opt)
        angle_err = angle_error(transformed_j, data['reference'], align_start=True, shift_samples=0)
        angle_error_vqf = angle_error(transformed_vqf, data['reference'], align_start=True, shift_samples=0)
        plt.plot(angle_err, label='Transformed Justa Error')
        plt.plot(angle_error_vqf, label='Transformed VQF Error')
        plt.legend()
        plt.show()


q_l_mean = np.mean(q_l, axis=0)
q_r_mean = np.mean(q_r, axis=0)

q_l_stdev = np.std(q_l, axis=0)
q_r_stdev = np.std(q_r, axis=0)
print('Mean:')
print(f"Quaternion left: {q_l_mean} \n Quaternion right: {q_r_mean} \n Mean angle error: {np.mean(angle_errs):.4f} deg")
print('Standard Deviation:')
print(f"Quaternion left: {q_l_stdev} \n Quaternion right: {q_r_stdev} \n Mean angle error: {np.std(angle_errs):.4f} deg")