# Project Organization Summary

This document describes the reorganized structure of the Thesis project.

## Recent Changes (January 19, 2026)

All README files have been consolidated into the `README/` directory, and all test and example files have been moved to the `tests/` directory for better organization.

## Directory Structure

```
Thesis/
├── README.md                          # Main project README
├── PROJECT_ORGANIZATION.md            # This file
├── LICENSE                            # Project license
├── requirements.txt                   # Python dependencies
├── setup.py                          # Package setup
├── environment.yml                    # Conda environment
├── setup_venv.ps1                    # Virtual environment setup script
│
├── README/                           # Documentation directory
│   ├── INDEX.md                      # Documentation index
│   ├── README_EMG.md                 # EMG module documentation
│   ├── README_IMU.md                 # IMU module documentation
│   └── IMU_MODULE_SUMMARY.md         # IMU module quick reference
│
├── src/                              # Source code
│   ├── __init__.py                   # Package initialization
│   ├── emg_acquisition.py            # EMG acquisition module
│   ├── imu_acquisition.py            # IMU acquisition module
│   └── example.py                    # Basic examples
│
├── tests/                            # Tests and examples
│   ├── README.md                     # Tests/examples documentation
│   ├── __init__.py                   # Package initialization
│   ├── test_emg.py                   # EMG module tests
│   ├── test_imu.py                   # IMU module tests
│   ├── test_example.py               # Example tests
│   ├── examples_emg_usage.py         # EMG interactive examples
│   ├── examples_imu_usage.py         # IMU interactive examples
│   └── emg_example.py                # Simple EMG example
│
├── scripts/                          # Standalone scripts
│   ├── EMG_testing.py                # EMG device testing script
│   ├── IMU_testing.py                # IMU device testing script (with visualization)
│   └── colab_setup.ipynb             # Google Colab setup
│
└── tmsi-python-interface/            # TMSi Python SDK
    └── TMSiSDK/                      # SDK modules
        └── device/
            └── devices/
                └── legacy/           # Legacy device support (Porti7/REFA)
```

## File Locations

### Documentation

All documentation is now in the `README/` directory:

| File | Description | Location |
|------|-------------|----------|
| Main README | Project overview | `README.md` (root) |
| EMG Documentation | Complete EMG module guide | `README/README_EMG.md` |
| IMU Documentation | Complete IMU module guide | `README/README_IMU.md` |
| IMU Summary | Quick reference for IMU | `README/IMU_MODULE_SUMMARY.md` |
| Documentation Index | Guide to all docs | `README/INDEX.md` |

### Tests and Examples

All tests and examples are now in the `tests/` directory:

| File | Type | Description | Run Command |
|------|------|-------------|-------------|
| `test_emg.py` | Test | EMG module test suite | `python tests/test_emg.py` |
| `test_imu.py` | Test | IMU module test suite | `python tests/test_imu.py` |
| `test_example.py` | Test | Basic test examples | `python tests/test_example.py` |
| `examples_emg_usage.py` | Examples | Interactive EMG examples | `python tests/examples_emg_usage.py` |
| `examples_imu_usage.py` | Examples | Interactive IMU examples | `python tests/examples_imu_usage.py` |
| `emg_example.py` | Example | Simple EMG demo | `python tests/emg_example.py` |

### Source Code

Core modules remain in the `src/` directory:

| File | Description |
|------|-------------|
| `emg_acquisition.py` | EMG acquisition module with `EMGDevice` class |
| `imu_acquisition.py` | IMU acquisition module with `IMUDevice` class |
| `__init__.py` | Package exports |

### Scripts

Standalone testing scripts with visualization remain in `scripts/`:

| File | Description |
|------|-------------|
| `EMG_testing.py` | EMG device connection and data acquisition test |
| `IMU_testing.py` | IMU orientation tracking with VPython visualization |

## Benefits of New Organization

### 1. Clear Separation of Concerns
- **Documentation**: All in `README/`
- **Tests & Examples**: All in `tests/`
- **Source Code**: All in `src/`
- **Scripts**: Standalone utilities in `scripts/`

### 2. Easier Navigation
- Find all docs in one place
- Find all tests/examples in one place
- No confusion about file locations

### 3. Better Discoverability
- New users can go straight to `README/` for documentation
- Developers can go to `tests/` for examples
- Clear separation between library code (`src/`) and usage examples (`tests/`)

### 4. Standard Python Project Structure
Follows Python best practices:
```
project/
├── README.md
├── src/
├── tests/
└── docs/ (or README/ in our case)
```

## Quick Start Guide

### 1. Documentation
```bash
# Read documentation
cat README/INDEX.md           # Documentation index
cat README/README_EMG.md      # EMG guide
cat README/README_IMU.md      # IMU guide
```

### 2. Run Examples
```bash
# EMG examples
python tests/examples_emg_usage.py

# IMU examples
python tests/examples_imu_usage.py

# Simple EMG example
python tests/emg_example.py
```

### 3. Run Tests
```bash
# Test EMG module
python tests/test_emg.py

# Test IMU module
python tests/test_imu.py
```

### 4. Use Modules in Code
```python
# Import from src package
from src import EMGDevice, IMUDevice, acquire_emg_data, acquire_imu_data

# Use the modules
emg = EMGDevice()
imu = IMUDevice(port="/dev/ttyUSB0")
```

## Migration Notes

### Files Moved

**To `README/` directory:**
- `src/README_EMG.md` → `README/README_EMG.md`
- `src/README_IMU.md` → `README/README_IMU.md`
- `IMU_MODULE_SUMMARY.md` → `README/IMU_MODULE_SUMMARY.md`

**To `tests/` directory:**
- `src/test_emg.py` → `tests/test_emg.py`
- `src/test_imu.py` → `tests/test_imu.py`
- `examples_emg_usage.py` → `tests/examples_emg_usage.py`
- `examples_imu_usage.py` → `tests/examples_imu_usage.py`
- `src/emg_example.py` → `tests/emg_example.py`

### No Breaking Changes

The module imports remain unchanged:
```python
from src import EMGDevice, IMUDevice  # Still works!
```

Only the locations of documentation and examples have changed.

## Additional Resources

- **Main README**: `README.md` - Project overview
- **Documentation Index**: `README/INDEX.md` - Guide to all documentation
- **Tests README**: `tests/README.md` - Guide to tests and examples
- **TMSi Interface**: `tmsi-python-interface/` - TMSi SDK with legacy device support

---

**Last Updated**: January 19, 2026

**Note**: This organization makes the project more maintainable and follows Python packaging best practices.
