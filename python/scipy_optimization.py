import pickle
import numpy as np
from scipy.optimize import minimize, differential_evolution
from vqf import VQF as PyVQF
# from vqf_local.vqf.pyvqf import PyVQF
from dataset_loader import DatasetLoader
from filters import (JustaAHRSv2, JustaAHRSInvFast, JustaAHRSPure, JustaAHRSv3)

from turbo_data import TuRBO_BO
from utils import angle_error, eval_filter_on_dataset
from matplotlib import pyplot as plt
import pandas as pd

SWITCH_VQF = True
RMSE = True

# Define the objective function to minimize
def objective_function(params, datasets):
    """
    Objective function that returns the mean error for given s1, s2, s3 parameters.
    
    Parameters:
    -----------
    params : array-like
        [s1, s2] parameters to optimize
    """
    # s1, s2, s3 = params
    N = params.shape[0]
    
    err = 0.0
    for dataset in datasets:

        shift = 0

        sta ='start_time'
        ir = "interest_range"
        if dataset.keys().__contains__(ir):
            start = dataset['time'] < dataset[ir][0]
            stop = dataset['time'] > dataset[ir][1]
            start_index = np.where(start)[0][-1] + 1
            stop_index = np.where(stop)[0][0] - 1
        elif dataset.keys().__contains__(sta):
            start_index = int(dataset[sta]['index'])
            stop_index = -1
        else:
            start_index = 200
            stop_index = -1
        
        alignIndex = int(start_index*0.5)
        
        bias = dataset['gyroscope'][:start_index].mean(axis=0)
        dataset_g = dataset['gyroscope'] - bias

        # Create filter instance with optimized parameters
        if SWITCH_VQF:
            gyr = np.ascontiguousarray(dataset_g, dtype=np.float64)
            acc = np.ascontiguousarray(dataset['accelerometer'], dtype=np.float64)
            mag = np.ascontiguousarray(dataset['magnetometer'], dtype=np.float64)
            b = PyVQF(1.0/dataset['mean_sampling_rate'], tauAcc=params[0], tauMag=params[1], motionBiasEstEnabled=False, restBiasEstEnabled=False, magDistRejectionEnabled=False)
            res = b.updateBatch(gyr, acc, mag)
            quaternion_result = res['quat9D']

        else:
            filter_instance = JustaAHRSPure(w_acc=params[0], w_mag=np.abs(params[1]), linMag=True)
            filter_instance.initFromAccMag(dataset['accelerometer'][0], dataset['magnetometer'][0]) # Initialize with first measurement
            quaternion_result = eval_filter_on_dataset(filter_instance, dataset, use_imu=False, use_square_err=False)

            
        angle_err = angle_error(quaternion_result, dataset['reference'], align_start=True, shift_samples=shift, align_index=alignIndex)
        
        angle_err = angle_err[start_index:stop_index]        
        
        if RMSE:
            mean_error = np.sqrt(np.mean(angle_err**2))
        else:
            mean_error = np.mean(angle_err)
        err += mean_error
    
    for i in range(N):
        print(f"s{i+1}={params[i]:.6f}", end=' ')
    print(f"-> mean_error={(err/len(datasets)):.6f}")
    
    return err
    

# Method 1: Nelder-Mead (simplex) - Good for local optimization
def optimize_nelder_mead(datasets):
    """Local optimization using Nelder-Mead method"""

    if SWITCH_VQF:
        initial_guess = [1, 1.0]
    else:
        initial_guess = [0.0004, 1.35e-04]  # Starting from your current values
        initial_guess = [1, 100]
    #
    
    result = minimize(
        objective_function,
        initial_guess,
        args=(datasets,),        
        method='Nelder-Mead',
        options={'maxiter': 50, 'xatol': 1e-6, 'fatol': 1e-6, 'disp': True}
        #bounds=[(0, 1.0), (0, 1.0), (0.0, 1.0)]
    )
    
    return result

def turbo_bo_optimize(dataset):
    
    pbounds = {"x1": (0, 0.01), "x2": (0, 0.01)} #, "x3": (0, 1)

    turbo_bo = TuRBO_BO(
        f=objective_function,
        pbounds=pbounds,
        dataset=dataset
    )

    best_pt, best_val = turbo_bo.run(n_iter=5)
    # scipy like result object
    class Result:
        def __init__(self, x, fun):
            self.x = x
            self.fun = fun
            self.success = True
            self.message = "Optimization completed successfully."
            
    return Result(best_pt, best_val.squeeze())

# Method 2: L-BFGS-B - Bounded optimization with gradients
def optimize_lbfgsb(dataset, bounds=None):
    """Bounded optimization using L-BFGS-B"""
    initial_guess = [0.0004, 1.35e-04, 0.5] 
    
    if bounds is None:
        # Define reasonable bounds for s1, s2, s3
        bounds = [(0.0, 5.0), (0.0, 5.0), (0.0, 1.0)]
    
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
    # dataset = dataset_loader.load_dataset('Justa')
    # sl = dataset_loader.load_justa_raw(0)
    # fast = dataset_loader.load_justa_raw(1)
    # dist = dataset_loader.load_justa_raw(2)
    # datasets = [sl, fast, dist]
    # sl = dataset_loader.load_sassari_dataset('slow_v4.mat', 0)
    # med = dataset_loader.load_sassari_dataset('medium_v4.mat', 0)
    # fast = dataset_loader.load_sassari_dataset('fast_v4.mat', 0)
    # datasets = [sl, med, fast]
    
    test_datasets = {}
    broad_white = dataset_loader.broad_white_list_datasets()
    for i in range(4):
        file = broad_white[i]
        dat = dataset_loader.load_broad_dataset(file_name=file, mean_initial_samples=True)
        if dat is not None:
            test_datasets[file] = dat

    datasets = list(test_datasets.values())

    #dataset = pickle.load( open('synthetic_rigid_body_sensor_offset.pkl', 'rb') )#  

    # dataset = dataset_loader.load_sassari_dataset('medium_v4.mat', 0)
    # bias = dataset['gyroscope'][:500].mean(axis=0)
    # dataset['gyroscope']=dataset['gyroscope']*np.array([1.015, 1.015, 1.01]) - bias
    
    # # Option 1: Fast local optimization (recommended to try first)
    print("\n### Method 1: Nelder-Mead (Local Optimization) ###")
    result = optimize_nelder_mead(datasets)
    
    # # Option 2: Bounded local optimization
    # print("\n### Method 2: L-BFGS-B (Bounded Optimization) ###")
    # result = optimize_lbfgsb(dataset)
    
    #Option 3: Global optimization (more thorough but slower)
    # print("\n### Method 3: Differential Evolution (Global Optimization) ###")
    # result = optimize_differential_evolution()
    
    # Option 4: TuRBO Bayesian Optimization
    # print("\n### Method 4: TuRBO Bayesian Optimization ###")
    # result = turbo_bo_optimize(dataset)
    
    # print("\n" + "=" * 60)
    # print("Optimization Complete!")
    # print("=" * 60)
    # print(f"Optimal s1: {result.x[0]:.6f}")
    # print(f"Optimal s2: {result.x[1]:.6f}")
    # # print(f"Optimal s3: {result.x[2]:.6f}")
    # print(f"Minimum mean error: {result.fun:.6f}")
    # print(f"Success: {result.success}")
    # print(f"Message: {result.message}")
    
    # # Test the optimal parameters
    # print("\n" + "=" * 60)
    # print("Verifying optimal parameters...")
    # print("=" * 60)
    
    # optimal_filter = JustaAHRSInvFast(
    #     w_acc=result.x[0], 
    #     w_mag=result.x[1])
    
    # optimal_filter.initFromAccMag(dataset['accelerometer'][0], dataset['magnetometer'][0])
    
    # quaternion_result = eval_filter_on_dataset(
    #     optimal_filter, dataset, use_imu=False, use_square_err=False
    # )
    # error_9D = angle_error(quaternion_result, dataset['reference'], align_start=True, shift_samples=1)
    # final_mean_error = np.mean(error_9D)

    # b = vqf.BasicVQF(1.0/dataset['mean_sampling_rate'], tauAcc=result.x[0], tauMag=result.x[1])
    # gyr = np.ascontiguousarray(dataset['gyroscope'], dtype=np.float64)
    # acc = np.ascontiguousarray(dataset['accelerometer'], dtype=np.float64)
    # mag = np.ascontiguousarray(dataset['magnetometer'], dtype=np.float64)
    # res = b.updateBatch(gyr, acc, mag)
    # shift = 1
    # skip_start_for_comparison = 5000

    # error_9D_vqf = angle_error(res['quat9D'], dataset['reference'], align_start=True, shift_samples=shift)
    # final_mean_error = np.mean(error_9D_vqf)

    
   # print(f"Final mean error: {final_mean_error:.6f}")