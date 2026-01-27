# AHRS Filter Optimization - Python Implementation

This is a Python implementation of the MATLAB AHRS filter optimization framework. It includes reimplementations of all major filters and the optimization algorithm.

## Features

- **Multiple AHRS Filters Implemented:**
  - Madgwick AHRS
  - Justa AHRS (Pure and Fast variants)
  - Valenti AHRS
  - Wilson-Madgwick AHRS
  - Admirall-Wilson AHRS
  - YoungSoo Suh AHRS
  - Jin Wu Kalman Filter AHRS

- **Dataset Support:**
  - ALS dataset
  - Justa dataset
  - Synthetic datasets (Synth2, Synth3)
  - CSV format for easy data handling

- **Optimization Features:**
  - Grid search parameter optimization
  - RMS or absolute error metrics
  - IMU-only or MARG mode
  - Compatible with MATLAB results

## Installation

1. Install required packages:
```bash
pip install -r ../requirements.txt
```

## Project Structure

```
python/
├── filters/                    # AHRS filter implementations
│   ├── __init__.py
│   ├── madgwick_ahrs.py       # Madgwick filter
│   ├── justa_ahrs.py          # Justa filters
│   ├── valenti_ahrs.py        # Valenti filter
│   ├── wilson_ahrs.py         # Wilson filters
│   ├── youngsoo_suh_ahrs.py   # YoungSoo Suh filter
│   └── jinwu_kf_ahrs.py       # Jin Wu Kalman filter
├── quaternion_library.py      # Quaternion operations
├── dataset_loader.py          # CSV dataset loader
├── optim_filter_params.py     # Parameter optimization
├── main_optimize.py           # Main example script
└── README.md                  # This file
```

## Usage

### Basic Example - Optimize Single Filter

```python
from main_optimize import optimize_single_filter

# Optimize Madgwick filter on Justa dataset
result = optimize_single_filter(
    filter_name='MadgwickAHRS',
    dataset_name='Justa',
    use_rms=True,
    use_imu=False
)

print(f"Optimized parameters: {result['parameters']}")
print(f"Final error: {result['error']}")
```

### Optimize All Filters

```python
from main_optimize import optimize_all_filters

results = optimize_all_filters(
    dataset_name='Justa',
    use_rms=True,
    use_imu=False
)
```

### Use a Filter Directly

```python
from filters import MadgwickAHRS
import numpy as np

# Create filter instance
ahrs = MadgwickAHRS(sample_period=1/256, beta=0.1)

# Update with sensor data
gyroscope = np.array([0.1, 0.05, -0.02])  # rad/s
accelerometer = np.array([0, 0, 1])        # normalized
magnetometer = np.array([0.5, 0, 0.866])   # normalized

ahrs.update(gyroscope, accelerometer, magnetometer)

# Get orientation quaternion
print(f"Quaternion: {ahrs.quaternion}")
```

### Custom Optimization

```python
from optim_filter_params import FilterOptimizer
from filters import JustaAHRSPureFast

# Create filter
filter_instance = JustaAHRSPureFast(
    gain=0.05,
    w_acc=0.002,
    w_mag=0.0001
)

# Create optimizer
optimizer = FilterOptimizer(
    filter_instance,
    dataset_name='ALS',
    use_rms=True,
    use_imu=False
)

# Run optimization
result = optimizer.optimize(max_iterations=1)
```

## Available Filters

### 1. MadgwickAHRS
- **Parameters:** `beta`
- **Reference:** http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

### 2. JustaAHRSPureFast
- **Parameters:** `gain`, `w_acc`, `w_mag`
- **Reference:** Sensors, 2020, 20.14: 3824

### 3. JustaAHRSPure
- **Parameters:** `w_acc`, `w_mag`
- **Reference:** Sensors, 2020, 20.14: 3824

### 4. ValentiAHRS
- **Parameters:** `w_acc`, `w_mag`
- **Reference:** Sensors 15(8):19302-19330, 2015

### 5. WilsonMadgwickAHRS
- **Parameters:** `beta`
- **Reference:** Case study on robot teleoperation

### 6. AdmirallWilsonAHRS
- **Parameters:** `beta`
- **Reference:** Improved formulation of IMU/MARG orientation

### 7. YoungSooSuhAHRS
- **Parameters:** `rg`, `ra`, `rm`
- **Reference:** IEEE Trans. Aerospace Electronic Systems, 2019

### 8. JinWuKFAHRS
- **Parameters:** `sigma_a`, `sigma_m`
- **Reference:** Journal of Sensors, 2017

## Dataset Format

CSV files should have the following columns:
- `time`: Timestamp
- `gx`, `gy`, `gz`: Gyroscope (rad/s)
- `ax`, `ay`, `az`: Accelerometer
- `mx`, `my`, `mz`: Magnetometer
- `ref1`, `ref2`, `ref3`, `ref4`: Reference quaternion [w, x, y, z]

## Comparison with MATLAB

This Python implementation closely follows the MATLAB version:
- Same filter algorithms
- Same optimization strategy
- Compatible parameter values
- Similar error metrics

Key differences:
- Python uses NumPy arrays instead of MATLAB matrices
- Object-oriented design for filters
- More modular code structure

## Running the Examples

```bash
# Navigate to python directory
cd python

# Run main optimization example
python main_optimize.py
```

## Extending

### Adding a New Filter

1. Create a new file in `filters/` directory
2. Implement the filter class with:
   - `__init__()` method with parameters
   - `update()` method for MARG data
   - Optional `update_imu()` for IMU-only
3. Add to `filters/__init__.py`
4. Add parameter mapping in `optim_filter_params.py`

### Adding a New Dataset

1. Place CSV file in `../Datasets/csv/`
2. Add entry to `DatasetLoader.load_dataset()`
3. Use with any filter or optimizer

## License

Same as the parent MATLAB project.

## Authors

Python implementation by GitHub Copilot, based on original MATLAB code by Josef Justa.
