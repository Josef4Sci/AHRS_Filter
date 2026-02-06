import numpy as np
import time
from numba import jit
from quaternion_library_jit import quatern_prod, quatern_conj, quaternion_rotate_vector, quatern_prod_single, fast_normalize_3d, fast_normalize_4d

# Your optimized function (paste your actual implementation here)
@jit(nopython=True, cache=True, fastmath=True)
def update_step_optimized(quaternion, gyroscope, accelerometer, magnetometer, 
                            dt, w_acc, w_mag, bias_gyro):
    """
    Heavily optimized update step.
    
    Key optimizations:
    1. Remove redundant astype() calls - inputs should already be float64
    2. Replace np.linalg.norm() with manual calculation (faster for small vectors)
    3. Combine operations to reduce intermediate allocations
    4. Use fastmath=True for aggressive optimizations
    5. Inline small helper functions
    6. Reuse calculations
    """
    
    # ============================================
    # OPTIMIZATION 1: Remove astype if inputs are already float64
    # Pass float64 arrays from outside instead
    # ============================================
    q = quaternion  # Remove .astype() - ensure caller passes float64
    
    # ============================================
    # OPTIMIZATION 2: Fast normalize with early exit
    # ============================================
    acc, valid_acc = fast_normalize_3d(accelerometer)
    if not valid_acc:
        return q
    
    mag, valid_mag = fast_normalize_3d(magnetometer)
    if not valid_mag:
        return q
    
    # ============================================
    # OPTIMIZATION 3: Gyroscope prediction - combine operations
    # ============================================
    wt = (gyroscope - bias_gyro) * (dt * 0.5)  # Multiply once
    
    q_dot_x = np.sin(wt[0])
    q_dot_y = np.sin(wt[1])
    q_dot_z = np.sin(wt[2])
    
    # Optimization: reuse sum calculation
    sin_sum_sq = q_dot_x*q_dot_x + q_dot_y*q_dot_y + q_dot_z*q_dot_z
    q_dot_w = np.sqrt(1.0 - sin_sum_sq)
    
    # Direct array creation without intermediate variable
    qp = quatern_prod_single(q, np.array([q_dot_w, q_dot_x, q_dot_y, q_dot_z], dtype=np.float64))
    
    # ============================================
    # OPTIMIZATION 4: Predicted accelerometer - use static array
    # ============================================
    # Predefine ar to avoid allocation
    acc_mes_pred = quaternion_rotate_vector(qp, np.array([0.0, 0.0, 1.0], dtype=np.float64))
    
    # ============================================
    # OPTIMIZATION 5: Magnetic reference - combine operations
    # ============================================
    mr_z = acc_mes_pred[0]*mag[0] + acc_mes_pred[1]*mag[1] + acc_mes_pred[2]*mag[2]  # Manual dot product
    mr_x = np.sqrt(1.0 - mr_z*mr_z)
    
    mag_mes_pred = quaternion_rotate_vector(qp, np.array([0.0, mr_x, mr_z], dtype=np.float64))
    
    # ============================================
    # OPTIMIZATION 6: Accelerometer correction - avoid double normalization
    # ============================================
    # Cross product: ca = acc × acc_mes_pred
    ca_x = acc[1]*acc_mes_pred[2] - acc[2]*acc_mes_pred[1]
    ca_y = acc[2]*acc_mes_pred[0] - acc[0]*acc_mes_pred[2]
    ca_z = acc[0]*acc_mes_pred[1] - acc[1]*acc_mes_pred[0]
    
    ca_norm = np.sqrt(ca_x*ca_x + ca_y*ca_y + ca_z*ca_z)
    
    # Avoid division if norm is too small
    if ca_norm > 1e-10:
        inv_ca_norm = 1.0 / ca_norm
        vec_a_x = ca_x * inv_ca_norm
        vec_a_y = ca_y * inv_ca_norm
        vec_a_z = ca_z * inv_ca_norm
    else:
        vec_a_x = vec_a_y = vec_a_z = 0.0
    
    # ============================================
    # OPTIMIZATION 7: Magnetometer correction - same optimization
    # ============================================
    cm_x = mag[1]*mag_mes_pred[2] - mag[2]*mag_mes_pred[1]
    cm_y = mag[2]*mag_mes_pred[0] - mag[0]*mag_mes_pred[2]
    cm_z = mag[0]*mag_mes_pred[1] - mag[1]*mag_mes_pred[0]
    
    cm_norm = np.sqrt(cm_x*cm_x + cm_y*cm_y + cm_z*cm_z)
    
    if cm_norm > 1e-10:
        inv_cm_norm = 1.0 / cm_norm
        vec_b_x = cm_x * inv_cm_norm
        vec_b_y = cm_y * inv_cm_norm
        vec_b_z = cm_z * inv_cm_norm
    else:
        vec_b_x = vec_b_y = vec_b_z = 0.0
    
    # ============================================
    # OPTIMIZATION 8: Combined correction - fuse operations
    # ============================================
    w_acc_half = w_acc * 0.5
    w_mag_half = w_mag * 0.5
    
    im_x = vec_a_x * w_acc_half + vec_b_x * w_mag_half
    im_y = vec_a_y * w_acc_half + vec_b_y * w_mag_half
    im_z = vec_a_z * w_acc_half + vec_b_z * w_mag_half
    
    im_norm = np.sqrt(im_x*im_x + im_y*im_y + im_z*im_z)
    
    # sinc optimization: sinc(x) = sin(πx)/(πx)
    # For small x, sinc(x) ≈ 1
    if im_norm < 1e-10:
        sinc_val = 1.0
    else:
        sinc_val = np.sinc(im_norm / np.pi)
    
    im2_x = im_x * sinc_val
    im2_y = im_y * sinc_val
    im2_z = im_z * sinc_val
    
    im2_sum_sq = im2_x*im2_x + im2_y*im2_y + im2_z*im2_z
    q_cor = np.array([np.sqrt(1.0 - im2_sum_sq), im2_x, im2_y, im2_z], dtype=np.float64)
    
    # ============================================
    # OPTIMIZATION 9: Final quaternion
    # ============================================
    quat = quatern_prod_single(qp, q_cor)
    
    # Sign check and normalize in one step
    if quat[0] < 0.0:
        quat = -quat
    
    return fast_normalize_4d(quat)

# Prepare test data
def prepare_test_data():
    """Create realistic test data"""
    q = np.array([1., 0., 0., 0.], dtype=np.float64)
    gyro = np.array([0.1, 0.05, -0.02], dtype=np.float64)  # rad/s
    acc = np.array([0., 0., 9.81], dtype=np.float64)  # m/s^2
    mag = np.array([0.3, 0., 0.6], dtype=np.float64)  # normalized
    bias = np.zeros(3, dtype=np.float64)
    dt = 0.01  # 100 Hz
    w_acc = 0.5
    w_mag = 0.5
    
    return q, gyro, acc, mag, bias, dt, w_acc, w_mag

def benchmark(func, iterations=10000):
    """
    Benchmark a function with proper warmup.
    
    Args:
        func: Function to benchmark
        iterations: Number of iterations to run
        
    Returns:
        dict with timing statistics
    """
    q, gyro, acc, mag, bias, dt, w_acc, w_mag = prepare_test_data()
    
    print(f"Warming up (compiling JIT)...")
    # Warmup - first call triggers compilation
    warmup_start = time.perf_counter()
    _ = func(q, gyro, acc, mag, dt, w_acc, w_mag, bias)
    warmup_time = time.perf_counter() - warmup_start
    print(f"Warmup/compilation took: {warmup_time:.4f}s\n")
    
    # Run a few more warmup iterations
    for _ in range(100):
        _ = func(q, gyro, acc, mag, dt, w_acc, w_mag, bias)
    
    print(f"Running {iterations:,} iterations...")
    
    # Main benchmark
    times = []
    for i in range(iterations):
        start = time.perf_counter()
        result = func(q, gyro, acc, mag, dt, w_acc, w_mag, bias)
        end = time.perf_counter()
        times.append(end - start)
    
    times = np.array(times)
    
    # Calculate statistics
    total_time = np.sum(times)
    mean_time = np.mean(times)
    median_time = np.median(times)
    min_time = np.min(times)
    max_time = np.max(times)
    std_time = np.std(times)
    p95_time = np.percentile(times, 95)
    p99_time = np.percentile(times, 99)
    
    return {
        'total': total_time,
        'mean': mean_time,
        'median': median_time,
        'min': min_time,
        'max': max_time,
        'std': std_time,
        'p95': p95_time,
        'p99': p99_time,
        'warmup': warmup_time,
        'iterations': iterations,
        'times': times
    }

def print_results(stats):
    """Print benchmark results in a nice format"""
    print("\n" + "="*70)
    print("BENCHMARK RESULTS")
    print("="*70)
    print(f"Total iterations    : {stats['iterations']:,}")
    print(f"Warmup/compile time : {stats['warmup']:.4f}s")
    print(f"Total runtime       : {stats['total']:.4f}s")
    print("-"*70)
    print(f"Mean time per call  : {stats['mean']*1e6:8.2f} µs")
    print(f"Median time         : {stats['median']*1e6:8.2f} µs")
    print(f"Min time            : {stats['min']*1e6:8.2f} µs")
    print(f"Max time            : {stats['max']*1e6:8.2f} µs")
    print(f"Std deviation       : {stats['std']*1e6:8.2f} µs")
    print(f"95th percentile     : {stats['p95']*1e6:8.2f} µs")
    print(f"99th percentile     : {stats['p99']*1e6:8.2f} µs")
    print("-"*70)
    print(f"Throughput          : {stats['iterations']/stats['total']:,.0f} calls/sec")
    print(f"Hz (frequency)      : {stats['iterations']/stats['total']:,.0f} Hz")
    print("="*70)

def plot_histogram(stats, save_path='timing_histogram.png'):
    """Optional: Plot timing distribution (requires matplotlib)"""
    try:
        import matplotlib.pyplot as plt
        
        times_us = stats['times'] * 1e6  # Convert to microseconds
        
        plt.figure(figsize=(12, 6))
        
        # Histogram
        plt.subplot(1, 2, 1)
        plt.hist(times_us, bins=50, edgecolor='black', alpha=0.7)
        plt.axvline(stats['mean']*1e6, color='r', linestyle='--', label=f'Mean: {stats["mean"]*1e6:.2f} µs')
        plt.axvline(stats['median']*1e6, color='g', linestyle='--', label=f'Median: {stats["median"]*1e6:.2f} µs')
        plt.xlabel('Time (µs)')
        plt.ylabel('Frequency')
        plt.title('Execution Time Distribution')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Time series
        plt.subplot(1, 2, 2)
        plt.plot(times_us, alpha=0.5, linewidth=0.5)
        plt.axhline(stats['mean']*1e6, color='r', linestyle='--', label='Mean')
        plt.xlabel('Iteration')
        plt.ylabel('Time (µs)')
        plt.title('Execution Time Over Iterations')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=150)
        print(f"\nHistogram saved to: {save_path}")
        
    except ImportError:
        print("\nMatplotlib not available. Skipping histogram plot.")

# Main execution
if __name__ == "__main__":
    print("Starting benchmark for update_step_optimized()...")
    print("-"*70)
    
    # Run benchmark
    stats = benchmark(update_step_optimized, iterations=10000)
    
    # Print results
    print_results(stats)
    
    # Optional: plot histogram
    plot_histogram(stats)
    
    # Additional: Test with different data sizes
    print("\n\nTesting with batch processing...")
    print("-"*70)
    
    # Test if processing 1000 samples sequentially
    q, gyro, acc, mag, bias, dt, w_acc, w_mag = prepare_test_data()
    
    warmup = update_step_optimized(q, gyro, acc, mag, dt, w_acc, w_mag, bias)
    
    batch_size = 1000
    start = time.perf_counter()
    for _ in range(batch_size):
        result = update_step_optimized(q, gyro, acc, mag, dt, w_acc, w_mag, bias)
        q = result  # Update for next iteration
    batch_time = time.perf_counter() - start
    
    print(f"Processing {batch_size} samples sequentially:")
    print(f"  Total time: {batch_time:.4f}s")
    print(f"  Per sample: {batch_time/batch_size*1e6:.2f} µs")
    print(f"  Frequency : {batch_size/batch_time:.0f} Hz")
    print("="*70)