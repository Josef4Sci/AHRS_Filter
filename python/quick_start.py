"""
Quick Start Guide - Python AHRS Filter Optimization

This script shows the most common use cases in a simple, copy-paste format.
"""

# =============================================================================
# EXAMPLE 1: Use a filter with real sensor data
# =============================================================================

def example_1_basic_filter_usage():
    """Use Madgwick filter with sensor data"""
    from filters import MadgwickAHRS
    import numpy as np
    
    # Create filter
    ahrs = MadgwickAHRS(beta=0.1)
    dt = 1.0 / 100.0  # 100 Hz sample period
    
    # Example sensor readings (replace with your actual data)
    gyroscope = np.array([0.1, 0.05, -0.02])      # rad/s
    accelerometer = np.array([0.0, 0.0, 9.81])    # m/s^2
    magnetometer = np.array([20.0, 5.0, 45.0])    # μT
    
    # Normalize accelerometer and magnetometer
    accelerometer = accelerometer / np.linalg.norm(accelerometer)
    magnetometer = magnetometer / np.linalg.norm(magnetometer)
    
    # Update filter
    ahrs.update(gyroscope, accelerometer, magnetometer, dt)
    
    # Get orientation as quaternion [w, x, y, z]
    print(f"Orientation quaternion: {ahrs.quaternion}")
    
    # Convert to Euler angles if needed (roll, pitch, yaw)
    w, x, y, z = ahrs.quaternion
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
    pitch = np.arcsin(2*(w*y - z*x))
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
    
    print(f"Roll: {np.degrees(roll):.2f}°")
    print(f"Pitch: {np.degrees(pitch):.2f}°")
    print(f"Yaw: {np.degrees(yaw):.2f}°")


# =============================================================================
# EXAMPLE 2: Optimize a filter on a dataset
# =============================================================================

def example_2_optimize_single_filter():
    """Optimize Madgwick filter parameters"""
    from optim_filter_params import FilterOptimizer
    from filters import MadgwickAHRS
    
    # Create filter with initial parameters
    filter_instance = MadgwickAHRS(beta=0.1)
    
    # Create optimizer
    optimizer = FilterOptimizer(
        filter_instance,
        dataset_name='Justa',  # Options: 'Justa', 'ALS', 'Synth2', 'Synth3'
        use_rms=True,          # Use RMS error
        use_imu=False          # Use magnetometer (MARG mode)
    )
    
    # Run optimization
    result = optimizer.optimize(max_iterations=1)
    
    print(f"Optimized beta: {result['parameters']['beta']:.6f}")
    print(f"Final error: {result['error']:.6f}")


# =============================================================================
# EXAMPLE 3: Compare multiple filters
# =============================================================================

def example_3_compare_filters():
    """Compare different filters on same dataset"""
    from optim_filter_params import FilterOptimizer
    from filters import MadgwickAHRS, JustaAHRSPureFast, ValentiAHRS
    
    filters = [
        ('Madgwick', MadgwickAHRS(beta=0.1)),
        ('Justa Fast', JustaAHRSPureFast(gain=0.05, w_acc=0.002, w_mag=0.0001)),
        ('Valenti', ValentiAHRS(w_acc=0.01, w_mag=0.01))
    ]
    
    results = {}
    
    for name, filter_instance in filters:
        print(f"\nOptimizing {name}...")
        optimizer = FilterOptimizer(filter_instance, dataset_name='Justa')
        result = optimizer.optimize(max_iterations=1)
        results[name] = result['error']
        
    # Print comparison
    print("\n" + "="*50)
    print("Filter Comparison:")
    print("="*50)
    for name, error in sorted(results.items(), key=lambda x: x[1]):
        print(f"{name:20s}: {error:.6f}")


# =============================================================================
# EXAMPLE 4: Load and process a dataset
# =============================================================================

def example_4_load_dataset():
    """Load and inspect a dataset"""
    from dataset_loader import DatasetLoader
    
    # Create loader
    loader = DatasetLoader()
    
    # Load specific dataset
    data = loader.load_dataset('Justa')
    
    print(f"Dataset length: {len(data['time'])} samples")
    print(f"Duration: {data['time'][-1]:.2f} seconds")
    print(f"\nFirst gyroscope reading: {data['gyroscope'][0]}")
    print(f"First accelerometer reading: {data['accelerometer'][0]}")
    print(f"First magnetometer reading: {data['magnetometer'][0]}")
    print(f"First reference quaternion: {data['reference'][0]}")
    
    # Get a segment
    segment = loader.get_dataset_segment('Justa', start_idx=100, end_idx=200)
    print(f"\nSegment length: {len(segment['time'])} samples")


# =============================================================================
# EXAMPLE 5: Process your own CSV data
# =============================================================================

def example_5_custom_csv():
    """Process custom CSV file"""
    import pandas as pd
    import numpy as np
    from filters import MadgwickAHRS
    
    # Read your CSV file
    # Expected columns: time, gx, gy, gz, ax, ay, az, mx, my, mz
    df = pd.read_csv('your_data.csv')  # Replace with your file
    
    # Create filter
    ahrs = MadgwickAHRS(beta=0.1)
    
    # Process each row
    orientations = []
    
    for i in range(len(df)):
        # Get sensor data
        gyro = df.loc[i, ['gx', 'gy', 'gz']].values
        accel = df.loc[i, ['ax', 'ay', 'az']].values
        mag = df.loc[i, ['mx', 'my', 'mz']].values
        
        # Calculate sample period
        if i == 0:
            dt = df.loc[0, 'time']
        else:
            dt = df.loc[i, 'time'] - df.loc[i-1, 'time']
        
        # Normalize accelerometer and magnetometer
        accel = accel / np.linalg.norm(accel)
        mag = mag / np.linalg.norm(mag)
        
        # Update filter
        ahrs.update(gyro, accel, mag, dt)
        
        # Store result
        orientations.append(ahrs.quaternion.copy())
    
    # Convert to array
    orientations = np.array(orientations)
    print(f"Processed {len(orientations)} samples")


# =============================================================================
# EXAMPLE 6: Use all filters with batch optimization
# =============================================================================

def example_6_optimize_all():
    """Optimize all available filters"""
    from main_optimize import optimize_all_filters
    
    # Run optimization on all filters
    results = optimize_all_filters(
        dataset_name='Justa',
        use_rms=True,
        use_imu=False
    )
    
    # Results are printed automatically
    # You can also access them programmatically:
    for filter_name, result in results.items():
        print(f"\n{filter_name}:")
        print(f"  Parameters: {result['parameters']}")
        print(f"  Error: {result['error']:.6f}")


# =============================================================================
# EXAMPLE 7: Real-time simulation
# =============================================================================

def example_7_realtime_simulation():
    """Simulate real-time filter updates"""
    from filters import MadgwickAHRS
    from dataset_loader import DatasetLoader
    import time
    
    # Load dataset
    loader = DatasetLoader()
    data = loader.load_dataset('Justa')
    
    # Create filter
    ahrs = MadgwickAHRS(beta=0.1)
    ahrs.quaternion = data['reference'][0].copy()
    
    # Simulate real-time processing
    print("Starting real-time simulation...")
    start_time = time.time()
    
    for i in range(min(100, len(data['time']))):  # First 100 samples
        # Calculate dt
        if i == 0:
            dt = data['time'][0]
        else:
            dt = data['time'][i] - data['time'][i-1]
        
        # Update
        ahrs.update(
            data['gyroscope'][i],
            data['accelerometer'][i] / np.linalg.norm(data['accelerometer'][i]),
            data['magnetometer'][i] / np.linalg.norm(data['magnetometer'][i]),
            dt
        )
        
        # Print every 10 samples
        if i % 10 == 0:
            print(f"Sample {i}: q = [{ahrs.quaternion[0]:.4f}, {ahrs.quaternion[1]:.4f}, "
                  f"{ahrs.quaternion[2]:.4f}, {ahrs.quaternion[3]:.4f}]")
    
    elapsed = time.time() - start_time
    print(f"\nProcessed 100 samples in {elapsed:.3f} seconds")
    print(f"Average rate: {100/elapsed:.1f} Hz")


# =============================================================================
# Run examples
# =============================================================================

if __name__ == '__main__':
    import sys
    
    print("="*70)
    print("AHRS Filter Quick Start Examples")
    print("="*70)
    
    # Uncomment the example you want to run:
    
    print("\nExample 1: Basic filter usage")
    print("-"*70)
    example_1_basic_filter_usage()
    
    # print("\nExample 2: Optimize single filter")
    # print("-"*70)
    # example_2_optimize_single_filter()
    
    # print("\nExample 3: Compare filters")
    # print("-"*70)
    # example_3_compare_filters()
    
    # print("\nExample 4: Load dataset")
    # print("-"*70)
    # example_4_load_dataset()
    
    # print("\nExample 5: Custom CSV")
    # print("-"*70)
    # example_5_custom_csv()  # Need to create your_data.csv first
    
    # print("\nExample 6: Optimize all filters")
    # print("-"*70)
    # example_6_optimize_all()
    
    # print("\nExample 7: Real-time simulation")
    # print("-"*70)
    # example_7_realtime_simulation()
