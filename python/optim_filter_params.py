"""
Filter Parameter Optimization Module
Python implementation of MATLAB optimFilterParams function

This module optimizes AHRS filter parameters by minimizing orientation error
against reference data from motion capture systems.
"""
import numpy as np
import sys
import os

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from utils import eval_filter_on_dataset
from quaternion_library import quatern_prod, quatern_conj
from dataset_loader import DatasetLoader
from filters import (
    MadgwickAHRS, JustaAHRSPure, JustaAHRSv2,
    ValentiAHRS, WilsonMadgwickAHRS, AdmirallWilsonAHRS,
    YoungSooSuhAHRS, JinWuKFAHRS
)


class FilterOptimizer:
    """
    Optimize AHRS filter parameters using grid search
    """
    
    def __init__(self, filter_instance, dataset_name='Justa', use_rms=True, use_imu=False):
        """
        Initialize optimizer
        
        Args:
            filter_instance: Instance of an AHRS filter
            dataset_name: Name of dataset to use for optimization
            use_rms: Whether to use RMS error (True) or absolute error (False)
            use_imu: Whether to use IMU-only mode (no magnetometer)
        """
        self.filter = filter_instance
        self.dataset_name = dataset_name
        self.use_rms = use_rms
        self.use_imu = use_imu
        self.loader = DatasetLoader()
        
        # Load dataset
        self.data = self.loader.load_dataset(dataset_name)
        
        # Determine dataset segment based on dataset type
        self.start_idx, self.end_idx = self._get_dataset_range()
        
    def _get_dataset_range(self):
        """
        Get the appropriate data range for the dataset
        
        Returns:
            start_idx, end_idx tuple
        """
        if self.dataset_name == 'Justa':
            # For Justa dataset, can use different segments
            # Default: use full dataset
            return 0, len(self.data['time']) - 1
        else:
            return 0, len(self.data['time']) - 1
            
    def _get_filter_params(self):
        """
        Get current filter parameters based on filter type
        
        Returns:
            Dictionary with parameter names and values
        """
        filter_class = self.filter.__class__.__name__
        
        if filter_class == 'MadgwickAHRS':
            return {'beta': self.filter.beta}, 1
        elif filter_class == 'WilsonMadgwickAHRS':
            return {'beta': self.filter.beta}, 1
        elif filter_class == 'AdmirallWilsonAHRS':
            return {'beta': self.filter.beta}, 1
        elif filter_class == 'JustaAHRSv2':
            return {
                'w_acc': self.filter.w_acc,
                'w_mag': self.filter.w_mag,
                'gain': self.filter.gain
            }, 3
        elif filter_class == 'JustaAHRSPureFastClean':
            return {
                'w_acc': self.filter.w_acc,
                'w_mag': self.filter.w_mag,
                'gain': self.filter.gain
            }, 3
        elif filter_class == 'JustaAHRSPure':
            return {
                'w_acc': self.filter.w_acc,
                'w_mag': self.filter.w_mag
            }, 2
        elif filter_class == 'ValentiAHRS':
            return {
                'w_acc': self.filter.w_acc,
                'w_mag': self.filter.w_mag
            }, 2
        elif filter_class == 'JinWuKFAHRS':
            return {
                'sigma_a': self.filter.sigma_a[0, 0],
                'sigma_m': self.filter.sigma_m[0, 0]
            }, 2
        elif filter_class == 'YoungSooSuhAHRS':
            return {
                'rg': self.filter.rg,
                'ra': self.filter.ra,
                'rm': self.filter.rm
            }, 3
        else:
            raise ValueError(f"Unknown filter type: {filter_class}")
            
    def _set_filter_params(self, params):
        """
        Set filter parameters based on filter type
        
        Args:
            params: Dictionary with parameter values
        """
        filter_class = self.filter.__class__.__name__
        
        if filter_class == 'MadgwickAHRS':
            self.filter.beta = params.get('beta', self.filter.beta)
        elif filter_class == 'WilsonMadgwickAHRS':
            self.filter.beta = params.get('beta', self.filter.beta)
        elif filter_class == 'AdmirallWilsonAHRS':
            self.filter.beta = params.get('beta', self.filter.beta)
        elif filter_class == 'JustaAHRSv2' or filter_class == 'JustaAHRSPureFastClean':
            self.filter.w_acc = params.get('w_acc', self.filter.w_acc)
            self.filter.w_mag = params.get('w_mag', self.filter.w_mag)
            self.filter.gain = params.get('gain', self.filter.gain)
        elif filter_class == 'JustaAHRSPure' or filter_class == 'ValentiAHRS':
            self.filter.w_acc = params.get('w_acc', self.filter.w_acc)
            self.filter.w_mag = params.get('w_mag', self.filter.w_mag)
        elif filter_class == 'JinWuKFAHRS':
            if 'sigma_a' in params:
                self.filter.sigma_a = np.eye(3) * params['sigma_a']
            if 'sigma_m' in params:
                self.filter.sigma_m = np.eye(3) * params['sigma_m']
        elif filter_class == 'YoungSooSuhAHRS':
            self.filter.rg = params.get('rg', self.filter.rg)
            self.filter.ra = params.get('ra', self.filter.ra)
            self.filter.rm = params.get('rm', self.filter.rm)
            
    def _generate_test_values(self, current_value, param_name):
        """
        Generate test values around current parameter value
        
        Args:
            current_value: Current parameter value
            param_name: Name of the parameter
            
        Returns:
            List of test values
        """
        test_values = [
            current_value * 0.3,
            current_value * 0.6,
            current_value * 0.9,
            current_value * 0.95,
            current_value * 1.05,
            current_value * 1.1,
            current_value * 1.4,
            current_value * 1.7,
            np.random.rand() * current_value * 2
        ]
        
        # Ensure no negative values
        test_values = [max(0, v) for v in test_values]
        
        return test_values
        
    def _evaluate_filter(self, params):
        """
        Evaluate filter with given parameters
        
        Args:
            params: Dictionary of parameter values
            
        Returns:
            Mean error
        """
        # Set parameters
        self._set_filter_params(params)
        
        quaternion_result, angle_err = eval_filter_on_dataset(
            self.filter, self.data, use_imu=self.use_imu, use_square_err=self.use_rms)

        return np.mean(angle_err)
        
    def optimize(self, max_iterations=1):
        """
        Optimize filter parameters
        
        Args:
            max_iterations: Number of optimization iterations
            
        Returns:
            Dictionary with optimized parameters and error
        """
        current_params, num_params = self._get_filter_params()
        param_names = list(current_params.keys())
        
        print(f"Optimizing {self.filter.__class__.__name__}")
        print(f"Initial parameters: {current_params}")
        
        for iteration in range(max_iterations):
            print(f"\nIteration {iteration + 1}/{max_iterations}")
            
            # Optimize each parameter independently
            for param_idx, param_name in enumerate(param_names):
                current_value = current_params[param_name]
                test_values = self._generate_test_values(current_value, param_name)
                
                errors = []
                for test_value in test_values:
                    # Create test parameter set
                    test_params = current_params.copy()
                    test_params[param_name] = test_value
                    
                    # Evaluate
                    error = self._evaluate_filter(test_params)
                    errors.append(error)
                    
                # Find best value
                best_idx = np.argmin(errors)
                best_value = test_values[best_idx]
                best_error = errors[best_idx]
                
                current_params[param_name] = best_value
                print(f"  {param_name}: {current_value:.6f} -> {best_value:.6f} (error: {best_error:.6f})")
                
        # Set final optimized parameters
        self._set_filter_params(current_params)
        
        final_error = self._evaluate_filter(current_params)
        
        result = {
            'filter': self.filter,
            'parameters': current_params,
            'error': final_error
        }
        
        print(f"\nOptimization complete!")
        print(f"Final parameters: {current_params}")
        print(f"Final error: {final_error:.6f}")
        
        return result
