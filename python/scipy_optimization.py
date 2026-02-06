import pickle
import numpy as np
from scipy.optimize import minimize, differential_evolution
from dataset_loader import DatasetLoader
from filters import (JustaAHRSv2)

from utils import eval_filter_on_dataset
from matplotlib import pyplot as plt
import pandas as pd

# Define the objective function to minimize
def objective_function(params, dataset):
    """
    Objective function that returns the mean error for given s1, s2, s3 parameters.
    
    Parameters:
    -----------
    params : array-like
        [s1, s2, s3] parameters to optimize
    """
    s1, s2, s3 = params
    
    gyr_off_err = 0.336  # Example gyro offset error in deg/s
    bg=[-gyr_off_err, gyr_off_err, gyr_off_err / 2]
    
    # Create filter instance with optimized parameters
    filter_instance = JustaAHRSv2(
        gain=12.928152, 
        w_acc=0.00248, 
        w_mag=1.35e-04, 
        s1=s1, 
        s2=s2, 
        s3=s3,
        bias_gyro=np.zeros(3)
    )
    
    # Evaluate filter
    quaternion_result, angle_err = eval_filter_on_dataset(
        filter_instance, dataset, use_imu=False, use_square_err=False
    )
    
    #mean_error = np.mean(angle_error)
    mean_error = np.mean(pd.Series( np.diff(angle_err)).rolling(20).mean().abs())
    
    print(f"s1={s1:.6f}, s2={s2:.6f}, s3={s3:.6f} -> mean_error={mean_error:.6f}")
    
    return mean_error
    

# Method 1: Nelder-Mead (simplex) - Good for local optimization
def optimize_nelder_mead(dataset):
    """Local optimization using Nelder-Mead method"""
    initial_guess = [0.00248, 1.35e-04, -2.0]  # Starting from your current values
    
    result = minimize(
        objective_function,
        initial_guess,
        args=(dataset,),        
        method='Nelder-Mead',
        options={'maxiter': 30, 'xatol': 1e-6, 'fatol': 1e-6, 'disp': True}
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
    #dataset = dataset_loader.load_dataset('Justa')
    dataset = pickle.load( open('synthetic_rigid_body_sensor_offset.pkl', 'rb') )#  
    
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
    print(f"Optimal s3: {result.x[2]:.6f}")
    print(f"Minimum mean error: {result.fun:.6f}")
    print(f"Success: {result.success}")
    print(f"Message: {result.message}")
    
    # Test the optimal parameters
    print("\n" + "=" * 60)
    print("Verifying optimal parameters...")
    print("=" * 60)
    gyr_off_err = 0.0  # Example gyro offset error in deg/s
    optimal_filter = JustaAHRSv2(
        gain=12.928152,
        w_acc=0.00248, 
        w_mag=1.35e-04,
        s1=result.x[0],
        s2=result.x[1],
        s3=result.x[2],
        bias_gyro=np.array([-gyr_off_err, gyr_off_err, gyr_off_err / 2])
    )
    quaternion_result, angle_err = eval_filter_on_dataset(
        optimal_filter, dataset, use_imu=False, use_square_err=False
    )
    final_mean_error = np.mean(angle_err)
    print(f"Final mean error: {final_mean_error:.6f}")