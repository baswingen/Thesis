# IMU Module Migration Guide

The IMU acquisition module has been updated to use the new ASCII protocol with dual BMI160 support.

## What Changed

### Old Module (Binary Protocol)
- Used binary serial protocol (500000 baud)
- Madgwick filter only
- Single format string parsing
- Less robust error handling

### New Module (ASCII Protocol)
- Uses ASCII serial protocol (230400 baud) with XOR checksum
- Mahony filter with adaptive gains
- Dual BMI160 support built-in
- Robust auto-reconnection
- Better calibration

## API Compatibility

### ✅ Fully Compatible

The new module maintains **backward compatibility** with the old API:

```python
# All of these still work:
from src.imu_acquisition import IMUDevice, acquire_imu_data
from src import IMUDevice, IMUReading, IMUCalibration

# Old code continues to work
with IMUDevice() as imu:
    imu.calibrate()
    reading = imu.read()
```

### 🆕 New Features

1. **IMUConfig for advanced configuration**:
```python
from src.imu_acquisition import IMUDevice, IMUConfig

config = IMUConfig(
    port='/dev/ttyUSB0',
    baud=230400,
    kp=2.0,  # Mahony gains
    ki=0.01,
    adaptive_gains=True
)

with IMUDevice(config) as imu:
    imu.calibrate()
```

2. **Dual IMU data in single reading**:
```python
reading = imu.read()
print(reading.gyro1, reading.accel1)  # IMU 1
print(reading.gyro2, reading.accel2)  # IMU 2
print(reading.euler1)  # IMU 1 orientation
print(reading.euler2)  # IMU 2 orientation
```

3. **Mahony filter instead of Madgwick**:
```python
# Automatically used when use_mahony=True (default)
# Better performance and accuracy
```

4. **Stream interface**:
```python
# Duration-based
for reading in imu.read_stream(duration=10.0):
    process(reading)

# Sample-based
for reading in imu.read_stream(max_samples=1000):
    process(reading)
```

## Migration Examples

### Example 1: Basic Usage (No Changes Needed)

**Old Code:**
```python
from src.imu_acquisition import IMUDevice

with IMUDevice() as imu:
    imu.calibrate(samples=200)
    
    for i in range(100):
        reading = imu.read()
        if reading:
            print(reading.euler1)
```

**New Code:**
```python
# Identical! No changes needed
from src.imu_acquisition import IMUDevice

with IMUDevice() as imu:
    imu.calibrate(samples=200)
    
    for i in range(100):
        reading = imu.read()
        if reading:
            print(reading.euler1)
```

### Example 2: Using New Stream API

**Old Code:**
```python
from src.imu_acquisition import IMUDevice

with IMUDevice() as imu:
    imu.calibrate()
    
    readings = []
    for i in range(100):
        reading = imu.read()
        if reading:
            readings.append(reading)
```

**New Code (Cleaner):**
```python
from src.imu_acquisition import IMUDevice

with IMUDevice() as imu:
    imu.calibrate()
    
    # Simpler and more Pythonic
    readings = list(imu.read_stream(max_samples=100))
```

### Example 3: Custom Configuration

**Old Code:**
```python
from src.imu_acquisition import IMUDevice

# Configuration was done via attributes
imu = IMUDevice()
# ... set attributes ...
imu.connect()
```

**New Code (Better):**
```python
from src.imu_acquisition import IMUDevice, IMUConfig

# Configuration via dataclass
config = IMUConfig(
    port='/dev/ttyUSB0',
    baud=230400,
    kp=2.0,
    adaptive_gains=True
)

with IMUDevice(config) as imu:
    imu.calibrate()
```

### Example 4: Accessing Both IMUs

**Old Code:**
```python
# Old module only had IMU1 data easily accessible
reading = imu.read()
gyro = reading.gyro1
accel = reading.accel1
```

**New Code:**
```python
# New module has both IMUs
reading = imu.read()
gyro1, gyro2 = reading.gyro1, reading.gyro2
accel1, accel2 = reading.accel1, reading.accel2
euler1, euler2 = reading.euler1, reading.euler2
```

## Breaking Changes

### ⚠️ Minor Incompatibilities

1. **Filter Class Name**
   - Old: `MadgwickIMU`
   - New: `MahonyIMU`
   - **Fix**: Use `MahonyIMU` or access via `IMUDevice.filter1`

2. **Serial Protocol**
   - Old: Binary (500000 baud)
   - New: ASCII (230400 baud)
   - **Fix**: Update Arduino sketch to use new protocol

3. **Baud Rate**
   - Default changed from 500000 to 230400
   - **Fix**: Specify in config if needed

## Arduino Sketch Requirements

The new module requires the ASCII protocol Arduino sketch:

```cpp
// Arduino output format:
$seq,us,ax1,ay1,az1,gx1,gy1,gz1,ok1,ax2,ay2,az2,gx2,gy2,gz2,ok2*HH

// Example:
$123,456789,1000,2000,3000,100,200,300,1,1100,2200,3300,110,220,330,1*5A
```

See `arduino/IMU_sketch/` for the full sketch.

## Testing Your Migration

1. **Verify imports work**:
```bash
python tests/test_bmi160_import.py
```

2. **Test basic functionality**:
```python
from src.imu_acquisition import IMUDevice

with IMUDevice() as imu:
    imu.calibrate(samples=50)
    reading = imu.read()
    print("✓ Module works!" if reading else "✗ No data")
```

3. **Check your existing code**:
```bash
# Search for old API usage
grep -r "from.*imu_acquisition.*import" your_code/
grep -r "MadgwickIMU" your_code/
```

## Compatibility Matrix

| Feature | Old Module | New Module | Compatible |
|---------|-----------|------------|------------|
| `IMUDevice` class | ✅ | ✅ | ✅ Yes |
| `acquire_imu_data()` | ✅ | ✅ | ✅ Yes |
| `IMUReading` | ✅ | ✅ | ✅ Yes |
| `IMUCalibration` | ✅ | ✅ | ✅ Yes |
| `.read()` method | ✅ | ✅ | ✅ Yes |
| `.calibrate()` | ✅ | ✅ | ✅ Yes |
| Context manager | ✅ | ✅ | ✅ Yes |
| `MadgwickIMU` | ✅ | ❌ | ⚠️ Use `MahonyIMU` |
| Binary protocol | ✅ | ❌ | ⚠️ Use ASCII |
| Single IMU only | ✅ | ❌ | ✅ Access via `.gyro1` |

## Benefits of New Module

1. **Better Accuracy**: Mahony filter with adaptive gains
2. **More Robust**: Checksum validation prevents bad data
3. **Easier Debugging**: ASCII protocol is human-readable
4. **Dual IMU Support**: Both IMUs in one reading
5. **Auto-Reconnection**: Handles serial errors gracefully
6. **Better Documentation**: Comprehensive docstrings
7. **Modern API**: Streaming interface, dataclasses, type hints

## Need Help?

Check these resources:
- Module demo: `python -m src.imu_acquisition`
- Usage examples: `tests/example_bmi160_dual_usage.py`
- Import test: `tests/test_bmi160_import.py`
- Documentation: `README/README_IMU.md`

## Rollback (If Needed)

If you need the old binary protocol module temporarily:

1. The old module is preserved as `src/imu_acquisition_legacy.py`
2. You can also access via git history:
   ```bash
   git log -- src/imu_acquisition.py
   ```

However, we recommend migrating to the new module as it's more robust and feature-rich.
