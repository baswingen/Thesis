# IMU Acquisition Module - Summary

## Overview

A native Python module for IMU acquisition lives in `src/imu_acquisition.py`.

It now supports:
- **Dual BMI160** (ASCII protocol + Mahony)
- **Single BNO085** (quaternion CSV)
- **Dual BNO085 (RVC)** (yaw/pitch/roll CSV) — this is the **primary/default IMU setup** for the program.

The VPython visualizer for dual BNO085 is now a native module at `src/bno085_dual_visualizer.py` (with a backward-compatible launcher kept in `setup_scripts/dual_BNO085_testing.py`).

## Files Created

### 1. `src/imu_acquisition.py` (493 lines)
Main module containing:
- **Classes:**
  - `IMUDevice`: High-level interface for dual IMU sensors
  - `MahonyIMU`: Mahony AHRS filter for BMI160 orientation estimation
  - `IMUReading`: Data structure for IMU samples
  - `IMUCalibration`: Data structure for calibration data

- **Functions:**
  - `acquire_imu_data()`: Convenience function for quick acquisition
  - `parse_imu_line()`: Parse serial data
  - Quaternion utilities: `quat_mul()`, `quat_conj()`, `quat_norm()`, `quat_inv()`, `rotate_vec_by_quat()`, `quat_to_euler()`

### 2. `src/__init__.py` (Updated)
Exports:
- `IMUDevice`
- `acquire_imu_data`
- `IMUReading`
- `IMUCalibration`
- `MadgwickIMU`

### 3. `src/README_IMU.md` (292 lines)
Comprehensive documentation including:
- Quick start guide
- API reference
- Usage examples
- Calibration instructions
- Troubleshooting guide
- Performance notes

### 4. `src/test_imu.py` (449 lines)
Test suite covering:
- Line parsing
- Madgwick filter
- Serial connection
- Gyro calibration
- Orientation filtering
- Convenience function

### 5. `examples_imu_usage.py` (617 lines)
Six complete examples:
1. Basic data acquisition
2. Calibration and bias correction
3. Orientation tracking
4. Relative orientation between IMUs
5. Convenience function usage
6. Save and load data

## Quick Start

### Import the Module

```python
from src import IMUDevice, acquire_imu_data
```

### Basic Usage

```python
# Connect to IMU device
device = IMUDevice(port="/dev/cu.usbmodem9888E00A0CC02")
device.connect()

# Calibrate (IMUs must be flat and still)
calibration = device.calibrate_gyro(num_samples=150)

# Enable orientation filtering
device.enable_orientation_filter(beta=0.08)

# Read samples
for i in range(100):
    data = device.read_sample_with_orientation()
    if data and 'imu1_euler' in data:
        euler = data['imu1_euler']
        print(f"Roll={euler[0]:.2f}, Pitch={euler[1]:.2f}, Yaw={euler[2]:.2f}")

device.disconnect()
```

### Convenience Function

```python
# Acquire 10 seconds of data with automatic calibration
readings, calibration = acquire_imu_data(
    port="/dev/cu.usbmodem9888E00A0CC02",
    duration=10.0,
    calibrate=True,
    enable_filter=True
)
```

## Key Features

### 1. Robust Serial Communication
- Auto-reconnect on errors
- Stall detection and recovery
- Configurable timeouts

### 2. Gyro Calibration
- Measures and removes gyro bias
- Reduces orientation drift
- Saves/loads calibration data

### 3. Orientation Estimation
- Madgwick AHRS filter
- Fuses gyro + accel data
- Outputs quaternions or Euler angles
- Configurable filter gain (beta)

### 4. Easy Integration
- Simple, intuitive API
- Convenience functions for common tasks
- Comprehensive error handling
- Well-documented

## Testing

Run the test suite:

```bash
python src/test_imu.py
```

Or from project root:

```bash
python -m src.test_imu
```

## Examples

Run interactive examples:

```bash
python examples_imu_usage.py
```

## Comparison with Original Script

| Feature | `scripts/IMU_testing.py` | `src/imu_acquisition.py` |
|---------|--------------------------|--------------------------|
| Visualization | VPython (built-in) | Separate (can be added) |
| API | Script-level | Module-level classes |
| Reusability | Low | High |
| Testing | None | Comprehensive suite |
| Documentation | Comments | Full README + docstrings |
| Data export | Manual | Built-in functions |
| Calibration | Integrated with viz | Standalone method |
| Import | N/A | `from src import IMUDevice` |

## Integration with Existing Code

The original `scripts/IMU_testing.py` remains unchanged and functional. The new module provides the same core functionality in a reusable format.

To use the new module in your project:

```python
# Option 1: Import specific classes
from src import IMUDevice, MadgwickIMU

# Option 2: Import convenience function
from src import acquire_imu_data

# Option 3: Import everything
import src
device = src.IMUDevice(port="...")
```

## Next Steps

1. **Test the module** with your hardware:
   ```bash
   python src/test_imu.py
   ```

2. **Try the examples**:
   ```bash
   python examples_imu_usage.py
   ```

3. **Read the documentation**:
   - `src/README_IMU.md` - Complete guide
   - `src/imu_acquisition.py` - API docstrings

4. **Integrate into your workflow**:
   - Use `IMUDevice` for custom acquisition logic
   - Use `acquire_imu_data()` for quick data collection
   - Combine with EMG acquisition for multi-sensor experiments

## Dependencies

All dependencies are already satisfied:
- `numpy` - Array operations
- `pyserial` - Serial communication (already used for IMU testing)
- `dataclasses` - Data structures (Python 3.7+ standard library)

## Notes

- **Serial Port**: Update `PORT` in examples to match your device
- **Calibration**: Always calibrate with IMUs flat and still for best results
- **Beta Parameter**: Default 0.08 works well for most cases; adjust for your application
- **Sample Rate**: Depends on Arduino firmware (typically 20-100 Hz)

## Support

For questions or issues:
1. Check `src/README_IMU.md` for detailed documentation
2. Run test suite to verify functionality
3. Review examples for usage patterns
4. Check troubleshooting section in README

---

**Status**: ✅ Complete and ready to use

**Date**: January 19, 2026
