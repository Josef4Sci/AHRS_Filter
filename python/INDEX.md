# Python AHRS Filters - Navigation Index

## 📚 Documentation Files

Start here based on what you need:

### 🚀 Quick Start
- **[DONE.md](DONE.md)** - Quick overview of what was created
- **[quick_start.py](quick_start.py)** - 7 ready-to-run examples
- **[README.md](README.md)** - Main documentation and usage guide

### 📖 Detailed Guides
- **[COMPLETE_OVERVIEW.md](COMPLETE_OVERVIEW.md)** - Comprehensive reference
- **[IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md)** - Implementation details

### 🔬 Testing & Examples
- **[test_filters.py](test_filters.py)** - Unit tests for all filters
- **[main_optimize.py](main_optimize.py)** - Optimization examples

## 💻 Code Files

### Core Functionality
- **[quaternion_library.py](quaternion_library.py)** - Quaternion math operations
- **[dataset_loader.py](dataset_loader.py)** - Advanced CSV dataset loader
- **[load_datasets.py](load_datasets.py)** - Simple CSV loader
- **[optim_filter_params.py](optim_filter_params.py)** - Parameter optimization engine

### Filter Implementations
All in the `filters/` directory:
- **[madgwick_ahrs.py](filters/madgwick_ahrs.py)** - Madgwick AHRS
- **[justa_ahrs.py](filters/justa_ahrs.py)** - Justa filters (Pure & Fast)
- **[valenti_ahrs.py](filters/valenti_ahrs.py)** - Valenti AHRS
- **[wilson_ahrs.py](filters/wilson_ahrs.py)** - Wilson filters (2 variants)
- **[youngsoo_suh_ahrs.py](filters/youngsoo_suh_ahrs.py)** - YoungSoo Suh AHRS
- **[jinwu_kf_ahrs.py](filters/jinwu_kf_ahrs.py)** - Jin Wu Kalman Filter

## 🎯 What Should I Read First?

### If you want to...

#### ...quickly test if it works
1. Install: `pip install -r ../requirements.txt`
2. Test: `python test_filters.py`
3. Run: `python quick_start.py`

#### ...understand how to use a filter
1. Read: **[README.md](README.md)** → "Usage" section
2. See: **[quick_start.py](quick_start.py)** → Example 1
3. Try: Copy example and modify for your needs

#### ...optimize filter parameters
1. Read: **[README.md](README.md)** → "Optimization" section
2. See: **[main_optimize.py](main_optimize.py)**
3. Run: `python main_optimize.py`

#### ...load your own CSV data
1. Read: **[README.md](README.md)** → "Dataset Format"
2. See: **[quick_start.py](quick_start.py)** → Example 5
3. Use: **[dataset_loader.py](dataset_loader.py)**

#### ...understand the implementation
1. Read: **[IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md)**
2. Check: **[COMPLETE_OVERVIEW.md](COMPLETE_OVERVIEW.md)**
3. Review: Individual filter source files

#### ...add a new filter
1. Read: **[COMPLETE_OVERVIEW.md](COMPLETE_OVERVIEW.md)** → "Extending"
2. Copy: Any filter file as template
3. Add: To `filters/__init__.py` and `optim_filter_params.py`

## 📊 Filter Quick Reference

| Filter | File | Parameters | Best For |
|--------|------|------------|----------|
| Madgwick | [madgwick_ahrs.py](filters/madgwick_ahrs.py) | beta | General purpose |
| Justa Fast | [justa_ahrs.py](filters/justa_ahrs.py) | gain, w_acc, w_mag | High speed |
| Justa Pure | [justa_ahrs.py](filters/justa_ahrs.py) | w_acc, w_mag | Accuracy |
| Valenti | [valenti_ahrs.py](filters/valenti_ahrs.py) | w_acc, w_mag | IMU/MARG |
| Wilson-Madgwick | [wilson_ahrs.py](filters/wilson_ahrs.py) | beta | MARG only |
| Admirall-Wilson | [wilson_ahrs.py](filters/wilson_ahrs.py) | beta | Improved |
| YoungSoo Suh | [youngsoo_suh_ahrs.py](filters/youngsoo_suh_ahrs.py) | rg, ra, rm | Separated sensors |
| Jin Wu KF | [jinwu_kf_ahrs.py](filters/jinwu_kf_ahrs.py) | sigma_a, sigma_m | Kalman-based |

## 🔧 Common Tasks

### Run a Filter
```python
from filters import MadgwickAHRS
ahrs = MadgwickAHRS(beta=0.1)
ahrs.update(gyro, accel, mag)
print(ahrs.quaternion)
```
📖 See: [quick_start.py](quick_start.py) Example 1

### Optimize Parameters
```python
from main_optimize import optimize_single_filter
result = optimize_single_filter('MadgwickAHRS', 'Justa')
```
📖 See: [main_optimize.py](main_optimize.py)

### Load Dataset
```python
from dataset_loader import DatasetLoader
loader = DatasetLoader()
data = loader.load_dataset('Justa')
```
📖 See: [dataset_loader.py](dataset_loader.py)

### Test Everything
```bash
python test_filters.py
```
📖 See: [test_filters.py](test_filters.py)

## 🆘 Troubleshooting

| Problem | Solution | Reference |
|---------|----------|-----------|
| Import errors | Run from `python/` directory | [README.md](README.md) |
| CSV not found | Check `../Datasets/csv/` exists | [dataset_loader.py](dataset_loader.py) |
| Filter fails | Run `test_filters.py` first | [test_filters.py](test_filters.py) |
| Need examples | Check `quick_start.py` | [quick_start.py](quick_start.py) |

## 📦 Dependencies

Install with: `pip install -r ../requirements.txt`

Required:
- numpy >= 1.20.0
- pandas >= 1.3.0
- scipy >= 1.7.0
- matplotlib >= 3.4.0 (optional)

## 🏁 Getting Started Checklist

- [ ] Install dependencies: `pip install -r ../requirements.txt`
- [ ] Test installation: `python test_filters.py`
- [ ] Read main docs: [README.md](README.md)
- [ ] Run examples: `python quick_start.py`
- [ ] Try optimization: `python main_optimize.py`
- [ ] Read your filter's source: `filters/madgwick_ahrs.py` (or whichever you need)
- [ ] Load your data: Use [dataset_loader.py](dataset_loader.py)
- [ ] Start coding! 🚀

## 📞 Help

1. **First**: Read [README.md](README.md)
2. **Examples**: Check [quick_start.py](quick_start.py)
3. **Details**: See [COMPLETE_OVERVIEW.md](COMPLETE_OVERVIEW.md)
4. **Test**: Run [test_filters.py](test_filters.py)

---

**Quick Links**:
[DONE.md](DONE.md) | 
[README.md](README.md) | 
[quick_start.py](quick_start.py) | 
[test_filters.py](test_filters.py) | 
[Filters](filters/)

**Last Updated**: January 27, 2026
