import pickle
import numpy as np
from scipy.optimize import minimize, differential_evolution
from vqf.vqf.basicvqf import BasicVQF
from dataset_loader import DatasetLoader
from filters import (JustaAHRSv2, JustaAHRSInvFast, JustaAHRSPure, JustaAHRSv3, JustaAHRSv4)

from utils import angle_error, eval_filter_on_dataset
from matplotlib import pyplot as plt
import pandas as pd

# Define the objective function to minimize
def objective_function(params, dataset):
    """
    Objective function that returns the mean error for given s1, s2, s3 parameters.
    
    Parameters:
    -----------
    params : array-like
        [s1, s2] parameters to optimize
    """
    # s1, s2, s3 = params
    s1, s2, s3 = params
    
    # Create filter instance with optimized parameters
    #filter_instance = JustaAHRSPure( w_acc=s1, w_mag=s2)
    filter_instance = JustaAHRSv4(w_acc=s1, w_mag=s2, cut_off=s3)
    # filter_instance = JustaAHRSPure( w_acc=0, w_mag=0, gyro_scale=np.array([s1, s2, s3]) )
    # #filter_instance = JustaAHRSv2(w_acc=0.00143, w_mag=0.0001, delay_steps=10, a_gyr=s1, b_gyr=s2)

    filter_instance.initFromAccMag(dataset['accelerometer'][0], dataset['magnetometer'][0]) # Initialize with first measurement
    
    # # Evaluate filter
    quaternion_result = eval_filter_on_dataset(
        filter_instance, dataset, use_imu=False, use_square_err=False
    )

    angle_err = angle_error(quaternion_result, dataset['reference'], use_imu=False, align_start=True, shift_samples=0)

    # window = 500
    # shift = -1
    # skip_start_for_comparison = 10

    # error_9D = angle_error(quaternion_result, dataset['reference'], align_start=True, shift_samples=shift)
    # diff_error = (pd.Series(error_9D) - pd.Series(error_9D).rolling(window).mean()).to_numpy()
    # men_diff =np.mean(np.abs(diff_error[window+skip_start_for_comparison:]))
    # # print(f'Mean error: {np.mean(error_9D[skip_start_for_comparison:]):.2f} deg')
    # # print(f'Mean diff error: {men_diff:.4f} deg')
    # mean_error = men_diff
        
    # gyr = np.ascontiguousarray(dataset['gyroscope'], dtype=np.float64)
    # acc = np.ascontiguousarray(dataset['accelerometer'], dtype=np.float64)
    # mag = np.ascontiguousarray(dataset['magnetometer'], dtype=np.float64)
    # b = BasicVQF(1.0/dataset['mean_sampling_rate'], tauAcc=s1, tauMag=s2, motionBiasEstEnabled=True, restBiasEstEnabled=True, magDistRejectionEnabled=False)
    # #b= vqf.VQF(1.0/dataset['mean_sampling_rate'], tauAcc=s1, tauMag=s2)
    # res = b.updateBatch(gyr, acc, mag)
    # shift = 1
    # angle_err = angle_error(res['quat9D'], dataset['reference'], align_start=True, shift_samples=shift)
    
    # Use RMS or absolute error
    # if use_square_err:
    #     angle_err = angle_err ** 2
    
    mean_error = np.mean(angle_err)
    #mean_error = np.mean(pd.Series( np.diff(angle_err)).rolling(20).mean().abs())
    
    print(f"s1={s1:.6f}, s2={s2:.6f}, s3={s3:.6f} -> mean_error={mean_error:.6f}")
    #print(f"s1={s1:.6f}, s2={s2:.6f} -> mean_error={mean_error:.6f}")
    
    return mean_error
    

# Method 1: Nelder-Mead (simplex) - Good for local optimization
def optimize_nelder_mead(dataset):
    """Local optimization using Nelder-Mead method"""
    initial_guess = [0.248, 1.35e-04, 2.0]  # Starting from your current values
    # initial_guess = [0.9,1]
    #initial_guess = [1, 0.0001]
    
    result = minimize(
        objective_function,
        initial_guess,
        args=(dataset,),        
        method='Nelder-Mead',
        options={'maxiter': 200, 'xatol': 1e-6, 'fatol': 1e-6, 'disp': True}
    )
    
    return result


# Method 2: L-BFGS-B - Bounded optimization with gradients
def optimize_lbfgsb(dataset, bounds=None):
    """Bounded optimization using L-BFGS-B"""
    initial_guess = [1.05, 1.0, 1.0]
    
    if bounds is None:
        # Define reasonable bounds for s1, s2, s3
        bounds = [(0.1, 5.0), (0.1, 5.0), (0.1, 5.0)]
    
    result = minimize(
        objective_function,
        initial_guess,
        method='L-BFGS-B',
        bounds=bounds,
        args=(dataset,),    
        options={'maxiter': 100, 'disp': True}
    )
    
    return result


# Method 3: Differential Evolution - Global optimization (more robust but slower)
def optimize_differential_evolution(bounds=None):
    """Global optimization using Differential Evolution"""
    if bounds is None:
        # Define reasonable bounds for s1, s2, s3
        bounds = [(0.1, 5.0), (0.1, 5.0), (0.1, 5.0)]
    
    result = differential_evolution(
        objective_function,
        bounds,
        strategy='best1bin',
        maxiter=50,
        popsize=10,
        tol=1e-6,
        disp=True,
        polish=True,  # Refine with L-BFGS-B at the end
        seed=42  # For reproducibility
    )
    
    return result


# Run optimization
if __name__ == "__main__":
    print("=" * 60)
    print("Starting optimization...")
    print("=" * 60)
    
    dataset_loader = DatasetLoader()
    dataset = dataset_loader.load_dataset('Justa')
    #dataset = pickle.load( open('synthetic_rigid_body_sensor_offset.pkl', 'rb') )#  

    dataset = dataset_loader.load_sassari_dataset('medium_v4.mat', 0)
    # bias = dataset['gyroscope'][:500].mean(axis=0)
    # dataset['gyroscope']=dataset['gyroscope']*np.array([1.015, 1.015, 1.01]) - bias
    
    # Option 1: Fast local optimization (recommended to try first)
    print("\n### Method 1: Nelder-Mead (Local Optimization) ###")
    result = optimize_nelder_mead(dataset)
    
    # Option 2: Bounded local optimization
    # print("\n### Method 2: L-BFGS-B (Bounded Optimization) ###")
    # result = optimize_lbfgsb()
    
    #Option 3: Global optimization (more thorough but slower)
    # print("\n### Method 3: Differential Evolution (Global Optimization) ###")
    # result = optimize_differential_evolution()
    
    print("\n" + "=" * 60)
    print("Optimization Complete!")
    print("=" * 60)
    print(f"Optimal s1: {result.x[0]:.6f}")
    print(f"Optimal s2: {result.x[1]:.6f}")
    # print(f"Optimal s3: {result.x[2]:.6f}")
    print(f"Minimum mean error: {result.fun:.6f}")
    print(f"Success: {result.success}")
    print(f"Message: {result.message}")
    
    # Test the optimal parameters
    print("\n" + "=" * 60)
    print("Verifying optimal parameters...")
    print("=" * 60)
    
    optimal_filter = JustaAHRSInvFast(
        w_acc=result.x[0], 
        w_mag=result.x[1])
    
    optimal_filter.initFromAccMag(dataset['accelerometer'][0], dataset['magnetometer'][0])
    
    quaternion_result = eval_filter_on_dataset(
        optimal_filter, dataset, use_imu=False, use_square_err=False
    )
    error_9D = angle_error(quaternion_result, dataset['reference'], align_start=True, shift_samples=1)
    final_mean_error = np.mean(error_9D)

    # b = vqf.BasicVQF(1.0/dataset['mean_sampling_rate'], tauAcc=result.x[0], tauMag=result.x[1])
    # gyr = np.ascontiguousarray(dataset['gyroscope'], dtype=np.float64)
    # acc = np.ascontiguousarray(dataset['accelerometer'], dtype=np.float64)
    # mag = np.ascontiguousarray(dataset['magnetometer'], dtype=np.float64)
    # res = b.updateBatch(gyr, acc, mag)
    # shift = 1
    # skip_start_for_comparison = 5000

    # error_9D_vqf = angle_error(res['quat9D'], dataset['reference'], align_start=True, shift_samples=shift)
    # final_mean_error = np.mean(error_9D_vqf)

    
    print(f"Final mean error: {final_mean_error:.6f}")