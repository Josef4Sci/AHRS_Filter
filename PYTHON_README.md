# 🐍 Python Implementation Available!

## NEW: Complete Python Port

All MATLAB filters have been reimplemented in Python! The Python version includes:

✅ **All 8 AHRS Filters**
- Madgwick AHRS
- Justa AHRS (Pure & Fast variants)
- Valenti AHRS
- Wilson-Madgwick AHRS
- Admirall-Wilson AHRS
- YoungSoo Suh AHRS
- Jin Wu Kalman Filter AHRS

✅ **Full Optimization Framework**
- Parameter optimization (similar to MATLAB)
- CSV dataset support
- Grid search algorithm
- RMS/Absolute error metrics

✅ **Easy to Use**
- Object-oriented design
- Comprehensive documentation
- Ready-to-run examples
- Unit tests included

## Quick Start (Python)

```bash
# Install dependencies
pip install -r requirements.txt

# Test filters
cd python
python test_filters.py

# Run examples
python quick_start.py

# Optimize parameters
python main_optimize.py
```

## Python Documentation

- **[python/INDEX.md](python/INDEX.md)** - Navigation guide (start here!)
- **[python/README.md](python/README.md)** - Complete usage guide
- **[python/quick_start.py](python/quick_start.py)** - 7 ready-to-run examples
- **[python/PROJECT_COMPLETE.md](python/PROJECT_COMPLETE.md)** - Implementation summary

## Python vs MATLAB

| Feature | MATLAB | Python |
|---------|--------|--------|
| All filters | ✅ | ✅ |
| Optimization | ✅ | ✅ |
| GUI | ✅ | ❌ |
| CSV support | ❌ | ✅ |
| MAT support | ✅ | ❌ |
| License required | ✅ | ❌ |
| Easy integration | Good | Excellent |

---

# AHRS_Filter (Original MATLAB)

This repository contains new AHRS filters (different variations of JustaAHRS) and new dataset with 9-DOF inertial measurement unit (3x accelerometer, 3x magnetometer, 3x gyroscope) with VICON reference. The filter responses can be compared to the well-known methods in MATLAB gui application which is also included in repository (screen below).

<p align="center">
  <img src="https://user-images.githubusercontent.com/49363434/66477762-21d4c980-ea99-11e9-9d34-b125b1880f6f.png" width="600" title="Screen1">
</p>

The implemented optimization methods are robust against stuck in local minimums (are not so precise)

## Please Cite:
JUSTA, Josef; ŠMÍDL, Václav; HAMÁČEK, Aleš. Fast AHRS Filter for Accelerometer, Magnetometer, and Gyroscope Combination with Separated Sensor Corrections. Sensors, 2020, 20.14: 3824.

## Choose Your Version

- **🐍 Python**: Go to [python/](python/) directory
- **📊 MATLAB**: Continue reading below
