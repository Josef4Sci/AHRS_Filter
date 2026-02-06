"""
Main script to demonstrate AHRS filter optimization
This is equivalent to the MATLAB optimFilterParams usage

Usage:
    python main_optimize.py
"""
import numpy as np
import sys
import os

# Add current directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from optim_filter_params import FilterOptimizer
from filters import (
    MadgwickAHRS, JustaAHRSPure, JustaAHRSPureFast,
    ValentiAHRS, WilsonMadgwickAHRS, AdmirallWilsonAHRS,
    YoungSooSuhAHRS, JinWuKFAHRS
)


def optimize_all_filters(dataset_name='Justa', use_rms=True, use_imu=False):
    """
    Optimize all available filters on a given dataset
    
    Args:
        dataset_name: Name of dataset ('Justa', 'ALS', 'Synth2', 'Synth3')
        use_rms: Use RMS error metric
        use_imu: Use IMU-only mode (no magnetometer)
    """
    # Define filters to optimize
    filters = [
        ('MadgwickAHRS', MadgwickAHRS(beta=0.1)),
        ('JustaAHRSPureFast', JustaAHRSPureFast(gain=0.0528152, w_acc=0.00248, w_mag=1.35e-04)),
        ('JustaAHRSPure', JustaAHRSPure(w_acc=0.00248, w_mag=1.35e-04)),
        ('ValentiAHRS', ValentiAHRS(w_acc=0.01, w_mag=0.01)),
        ('WilsonMadgwickAHRS', WilsonMadgwickAHRS(beta=0.1)),
        ('AdmirallWilsonAHRS', AdmirallWilsonAHRS(beta=0.1)),
        ('YoungSooSuhAHRS', YoungSooSuhAHRS(rg=0.317, ra=0.0004156, rm=0.00057)),
        ('JinWuKFAHRS', JinWuKFAHRS(sigma_a=3, sigma_m=500)),
    ]
    
    results = {}
    
    for filter_name, filter_instance in filters:
        print(f"\n{'='*60}")
        print(f"Optimizing {filter_name}")
        print(f"{'='*60}")
        
        try:
            optimizer = FilterOptimizer(
                filter_instance,
                dataset_name=dataset_name,
                use_rms=use_rms,
                use_imu=use_imu
            )
            
            result = optimizer.optimize(max_iterations=1)
            results[filter_name] = result
            
        except Exception as e:
            print(f"Error optimizing {filter_name}: {e}")
            import traceback
            traceback.print_exc()
            
    # Print summary
    print(f"\n{'='*60}")
    print("OPTIMIZATION SUMMARY")
    print(f"{'='*60}")
    print(f"Dataset: {dataset_name}")
    print(f"Metric: {'RMS' if use_rms else 'Absolute'}")
    print(f"Mode: {'IMU' if use_imu else 'MARG'}")
    print()
    
    # Sort by error
    sorted_results = sorted(results.items(), key=lambda x: x[1]['error'])
    
    for filter_name, result in sorted_results:
        print(f"{filter_name:25s} - Error: {result['error']:.6f}")
        for param_name, param_value in result['parameters'].items():
            print(f"  {param_name:15s}: {param_value:.6e}")
        print()
        
    return results


def optimize_single_filter(filter_name='MadgwickAHRS', dataset_name='Justa', 
                           use_rms=True, use_imu=False):
    """
    Optimize a single filter
    
    Args:
        filter_name: Name of the filter to optimize
        dataset_name: Name of dataset
        use_rms: Use RMS error metric
        use_imu: Use IMU-only mode
    """
    # Create filter instance
    filter_map = {
        'MadgwickAHRS': MadgwickAHRS(beta=0.1),
        'JustaAHRSPureFast': JustaAHRSPureFast(gain=0.0528152, w_acc=0.00248, w_mag=1.35e-04),
        'JustaAHRSPure': JustaAHRSPure(w_acc=0.00248, w_mag=1.35e-04),
        'ValentiAHRS': ValentiAHRS(w_acc=0.01, w_mag=0.01),
        'WilsonMadgwickAHRS': WilsonMadgwickAHRS(beta=0.1),
        'AdmirallWilsonAHRS': AdmirallWilsonAHRS(beta=0.1),
        'YoungSooSuhAHRS': YoungSooSuhAHRS(rg=0.317, ra=0.0004156, rm=0.00057),
        'JinWuKFAHRS': JinWuKFAHRS(sigma_a=3, sigma_m=500),
    }
    
    if filter_name not in filter_map:
        print(f"Unknown filter: {filter_name}")
        print(f"Available filters: {', '.join(filter_map.keys())}")
        return None
        
    filter_instance = filter_map[filter_name]
    
    print(f"Optimizing {filter_name} on {dataset_name} dataset")
    
    optimizer = FilterOptimizer(
        filter_instance,
        dataset_name=dataset_name,
        use_rms=use_rms,
        use_imu=use_imu
    )
    
    result = optimizer.optimize(max_iterations=1)
    
    return result


if __name__ == '__main__':
    # Example 1: Optimize a single filter
    print("Example 1: Optimize JustaAHRSPureFast filter on Justa dataset")
    result = optimize_single_filter('JustaAHRSPureFast', 'Justa', use_rms=True, use_imu=False)
    
    # Example 2: Optimize all filters (uncomment to run)
    # print("\n\nExample 2: Optimize all filters")
    # results = optimize_all_filters('Justa', use_rms=True, use_imu=False)
    
    # Example 3: Optimize on different dataset (uncomment to run)
    # print("\n\nExample 3: Optimize on ALS dataset")
    # result = optimize_single_filter('JustaAHRSPureFast', 'ALS', use_rms=True, use_imu=False)
