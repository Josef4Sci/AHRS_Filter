"""
Simple test script to verify filter implementations
"""
import numpy as np
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from filters import (
    MadgwickAHRS, JustaAHRSPure, JustaAHRSPureFast,
    ValentiAHRS, WilsonMadgwickAHRS, AdmirallWilsonAHRS,
    YoungSooSuhAHRS, JinWuKFAHRS
)


def test_filter(filter_instance, filter_name):
    """Test a single filter with dummy data"""
    print(f"Testing {filter_name}...")
    
    # Create dummy sensor data
    gyroscope = np.array([0.1, 0.05, -0.02])  # rad/s
    accelerometer = np.array([0, 0, 9.81])     # m/s^2
    magnetometer = np.array([0.5, 0, 0.866])   # arbitrary units
    
    # Normalize accelerometer and magnetometer
    accelerometer = accelerometer / np.linalg.norm(accelerometer)
    magnetometer = magnetometer / np.linalg.norm(magnetometer)
    
    filter_instance.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # Reset quaternion
    
    try:
        # Update filter
        filter_instance.update(gyroscope, accelerometer, magnetometer)
        
        # Check quaternion
        q = filter_instance.quaternion
        q_norm = np.linalg.norm(q)
        
        print(f"  Initial quaternion: {q}")
        print(f"  Quaternion norm: {q_norm:.6f}")
        
        if abs(q_norm - 1.0) > 1e-6:
            print(f"  WARNING: Quaternion not normalized!")
            return False
            
        # Run a few more iterations
        for i in range(10):
            filter_instance.update(gyroscope, accelerometer, magnetometer)
            
        q = filter_instance.quaternion
        print(f"  After 10 steps: {q}")
        print(f"  ✓ {filter_name} passed\n")
        return True
        
    except Exception as e:
        print(f"  ✗ {filter_name} failed: {e}\n")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Test all filters"""
    print("="*60)
    print("AHRS Filter Implementation Test")
    print("="*60)
    print()
    
    filters = [
        (MadgwickAHRS(beta=0.1), "MadgwickAHRS"),
        (JustaAHRSPureFast(gain=0.05, w_acc=0.002, w_mag=0.0001), "JustaAHRSPureFast"),
        (JustaAHRSPure(w_acc=0.002, w_mag=0.0001), "JustaAHRSPure"),
        (ValentiAHRS(w_acc=0.01, w_mag=0.01), "ValentiAHRS"),
        (WilsonMadgwickAHRS(beta=0.1), "WilsonMadgwickAHRS"),
        (AdmirallWilsonAHRS(beta=0.1), "AdmirallWilsonAHRS"),
        (YoungSooSuhAHRS(rg=0.3, ra=0.0004, rm=0.0005), "YoungSooSuhAHRS"),
        (JinWuKFAHRS(sigma_a=3, sigma_m=500), "JinWuKFAHRS"),
    ]
    
    passed = 0
    failed = 0
    
    for filter_instance, filter_name in filters:
        if test_filter(filter_instance, filter_name):
            passed += 1
        else:
            failed += 1
            
    print("="*60)
    print(f"Test Results: {passed} passed, {failed} failed")
    print("="*60)
    
    return failed == 0


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
