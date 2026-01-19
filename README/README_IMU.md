# IMU Acquisition Module

Native Python module for acquiring data from BMI160 dual IMU sensors via serial communication. This module provides high-level interfaces for IMU data acquisition, gyro calibration, and orientation estimation using the Madgwick filter.

## Features

- **Serial Communication**: Robust serial connection with auto-reconnect
- **Data Parsing**: Automatic parsing of dual IMU sensor data
- **Gyro Calibration**: Built-in gyroscope bias calibration
- **Orientation Estimation**: Madgwick AHRS filter for 6-DOF orientation
- **Quaternion Math**: Full quaternion utilities for 3D rotations
- **Easy API**: Simple high-level functions for common tasks

## Quick Start

### Basic Usage

```python
from src import IMUDevice

# Connect to IMU device
device = IMUDevice(port="/dev/cu.usbmodem9888E00A0CC02")  # Update port
device.connect()

# Calibrate gyroscope (IMUs must be flat and still!)
calibration = device.calibrate_gyro(num_samples=150)

# Enable orientation filtering
device.enable_orientation_filter(beta=0.08)

# Read samples
import time
for i in range(100):
    data = device.read_sample_with_orientation()
    if data:
        reading = data['reading']
        print(f"Sample {i}: IMU1 Gyro = {reading.imu1_gyro}")
        
        if 'imu1_euler' in data:
            euler = data['imu1_euler']
            print(f"  Roll={euler[0]:.2f}, Pitch={euler[1]:.2f}, Yaw={euler[2]:.2f}")
    
    time.sleep(0.01)

# Disconnect
device.disconnect()
```

### Convenience Function

```python
from src import acquire_imu_data

# Acquire 10 seconds of data with automatic calibration
readings, calibration = acquire_imu_data(
    port="/dev/cu.usbmodem9888E00A0CC02",
    duration=10.0,
    calibrate=True,
    enable_filter=True
)

print(f"Acquired {len(readings)} samples")
```

## Hardware Setup

### Expected Data Format

The module expects serial data in the following format:

```
t_ms | gx1 gy1 gz1 ax1 ay1 az1 | gx2 gy2 gz2 ax2 ay2 az2
```

Where:
- `t_ms`: Timestamp in milliseconds
- `gx1, gy1, gz1`: IMU1 gyroscope readings (deg/s)
- `ax1, ay1, az1`: IMU1 accelerometer readings (g)
- `gx2, gy2, gz2`: IMU2 gyroscope readings (deg/s)
- `ax2, ay2, az2`: IMU2 accelerometer readings (g)

Example line:
```
1234 | 0.12 -0.34 0.56 0.01 -0.02 0.98 | -0.15 0.22 -0.44 0.00 0.01 1.02
```

### Serial Port Configuration

**Windows:**
```python
device = IMUDevice(port="COM3", baud=115200)
```

**macOS/Linux:**
```python
device = IMUDevice(port="/dev/cu.usbmodem9888E00A0CC02", baud=115200)
```

## API Reference

### Classes

#### `IMUDevice`

Main class for interfacing with dual IMU sensors.

**Constructor:**
```python
IMUDevice(port: str, baud: int = 115200, 
          auto_reconnect: bool = True, 
          stall_timeout_s: float = 3.0)
```

**Methods:**

- `connect()`: Open serial connection
- `disconnect()`: Close serial connection
- `is_connected() -> bool`: Check connection status
- `read_sample(debug=False) -> Optional[IMUReading]`: Read single sample
- `calibrate_gyro(num_samples=150, timeout_s=60.0) -> IMUCalibration`: Calibrate gyro bias
- `enable_orientation_filter(beta=0.08)`: Enable Madgwick filter
- `disable_orientation_filter()`: Disable orientation filtering
- `get_orientation() -> Optional[Tuple[np.ndarray, np.ndarray]]`: Get quaternions
- `get_euler_angles() -> Optional[Tuple[...]]`: Get Euler angles (radians)
- `read_sample_with_orientation(...) -> Optional[Dict]`: Read sample and update filter

#### `IMUReading`

Data structure for a single IMU reading.

**Attributes:**
- `timestamp_ms: int`: Timestamp in milliseconds
- `timestamp_s: float`: Timestamp in seconds (property)
- `imu1_gyro: np.ndarray`: IMU1 gyroscope [gx, gy, gz] in deg/s
- `imu1_accel: np.ndarray`: IMU1 accelerometer [ax, ay, az] in g
- `imu2_gyro: np.ndarray`: IMU2 gyroscope [gx, gy, gz] in deg/s
- `imu2_accel: np.ndarray`: IMU2 accelerometer [ax, ay, az] in g

#### `IMUCalibration`

Calibration data for gyroscope bias.

**Attributes:**
- `gyro_bias_imu1: np.ndarray`: IMU1 bias [gx, gy, gz] in deg/s
- `gyro_bias_imu2: np.ndarray`: IMU2 bias [gx, gy, gz] in deg/s
- `num_samples: int`: Number of samples used for calibration

#### `MadgwickIMU`

Madgwick AHRS filter for orientation estimation.

**Constructor:**
```python
MadgwickIMU(beta: float = 0.1)
```

**Methods:**
- `update(gx, gy, gz, ax, ay, az, dt, use_accel=True) -> np.ndarray`: Update with new data
- `get_quaternion() -> np.ndarray`: Get current quaternion [w, x, y, z]
- `get_euler() -> Tuple[float, float, float]`: Get Euler angles (roll, pitch, yaw) in radians
- `reset()`: Reset to identity orientation

**Parameters:**
- `beta`: Filter gain (0.01-0.3)
  - Higher values = faster convergence but more noise
  - Lower values = smoother but slower convergence
  - Typical range: 0.05-0.15

### Functions

#### `acquire_imu_data()`

Convenience function for quick data acquisition.

```python
acquire_imu_data(
    port: str,
    duration: float = 10.0,
    baud: int = 115200,
    calibrate: bool = True,
    enable_filter: bool = False,
    filter_beta: float = 0.08
) -> Tuple[List[IMUReading], Optional[IMUCalibration]]
```

**Args:**
- `port`: Serial port
- `duration`: Acquisition duration in seconds
- `baud`: Baud rate
- `calibrate`: Perform gyro calibration before acquisition
- `enable_filter`: Enable Madgwick orientation filtering
- `filter_beta`: Madgwick filter gain

**Returns:**
- Tuple of (list_of_readings, calibration_data)

### Quaternion Utilities

The module provides several quaternion utility functions:

- `quat_mul(q, r)`: Quaternion multiplication
- `quat_conj(q)`: Quaternion conjugate
- `quat_norm(q)`: Normalize quaternion
- `quat_inv(q)`: Quaternion inverse
- `rotate_vec_by_quat(v, q)`: Rotate 3D vector by quaternion
- `quat_to_euler(q)`: Convert quaternion to Euler angles (roll, pitch, yaw)

## Usage Examples

### Example 1: Simple Data Acquisition

```python
from src import IMUDevice
import time

device = IMUDevice(port="/dev/cu.usbmodem9888E00A0CC02")
device.connect()

print("Reading 10 samples...")
for i in range(10):
    reading = device.read_sample()
    if reading:
        print(f"Sample {i}:")
        print(f"  Timestamp: {reading.timestamp_ms} ms")
        print(f"  IMU1 Gyro: {reading.imu1_gyro}")
        print(f"  IMU1 Accel: {reading.imu1_accel}")
    time.sleep(0.1)

device.disconnect()
```

### Example 2: Calibration and Orientation Tracking

```python
from src import IMUDevice
import numpy as np
import time

# Connect and calibrate
device = IMUDevice(port="/dev/cu.usbmodem9888E00A0CC02")
device.connect()

print("\n=== CALIBRATION ===")
print("Place both IMUs FLAT on a surface and keep them STILL")
time.sleep(3)

calibration = device.calibrate_gyro(num_samples=150)

# Enable orientation filtering
device.enable_orientation_filter(beta=0.08)

# Track orientation for 10 seconds
print("\n=== TRACKING ===")
start_time = time.time()

while (time.time() - start_time) < 10.0:
    data = device.read_sample_with_orientation()
    
    if data and 'imu1_euler' in data:
        # Get Euler angles and convert to degrees
        euler1 = np.rad2deg(data['imu1_euler'])
        euler2 = np.rad2deg(data['imu2_euler'])
        
        print(f"IMU1: Roll={euler1[0]:6.1f}° Pitch={euler1[1]:6.1f}° Yaw={euler1[2]:6.1f}°  "
              f"IMU2: Roll={euler2[0]:6.1f}° Pitch={euler2[1]:6.1f}° Yaw={euler2[2]:6.1f}°",
              end='\r')
    
    time.sleep(0.05)

print("\n\nDone!")
device.disconnect()
```

### Example 3: Save Data to File

```python
from src import acquire_imu_data
import numpy as np

# Acquire data
readings, calibration = acquire_imu_data(
    port="/dev/cu.usbmodem9888E00A0CC02",
    duration=30.0,
    calibrate=True,
    enable_filter=True
)

# Extract data for saving
timestamps = [r['reading'].timestamp_s for r in readings]
imu1_gyro = np.array([r['reading'].imu1_gyro for r in readings])
imu1_accel = np.array([r['reading'].imu1_accel for r in readings])
imu1_quat = np.array([r['imu1_quat'] for r in readings])

# Save to file
np.savez('imu_data.npz',
         timestamps=timestamps,
         imu1_gyro=imu1_gyro,
         imu1_accel=imu1_accel,
         imu1_quat=imu1_quat,
         gyro_bias=calibration.gyro_bias_imu1)

print(f"Saved {len(readings)} samples to imu_data.npz")

# Load later
data = np.load('imu_data.npz')
print(f"Loaded {len(data['timestamps'])} samples")
```

### Example 4: Real-time Analysis

```python
from src import IMUDevice
import numpy as np
import time

device = IMUDevice(port="/dev/cu.usbmodem9888E00A0CC02")
device.connect()
device.calibrate_gyro()
device.enable_orientation_filter(beta=0.08)

# Collect data for analysis
gyro_buffer = []
accel_buffer = []
buffer_size = 100

print("Collecting real-time statistics...")

while True:
    data = device.read_sample_with_orientation()
    
    if data:
        reading = data['reading']
        
        # Add to buffers
        gyro_buffer.append(reading.imu1_gyro)
        accel_buffer.append(reading.imu1_accel)
        
        # Keep buffer size fixed
        if len(gyro_buffer) > buffer_size:
            gyro_buffer.pop(0)
            accel_buffer.pop(0)
        
        # Calculate statistics
        if len(gyro_buffer) == buffer_size:
            gyro_arr = np.array(gyro_buffer)
            accel_arr = np.array(accel_buffer)
            
            gyro_mean = np.mean(gyro_arr, axis=0)
            gyro_std = np.std(gyro_arr, axis=0)
            accel_mean = np.mean(accel_arr, axis=0)
            accel_mag = np.linalg.norm(accel_mean)
            
            print(f"Gyro Mean: [{gyro_mean[0]:6.2f}, {gyro_mean[1]:6.2f}, {gyro_mean[2]:6.2f}] deg/s  "
                  f"Std: [{gyro_std[0]:5.2f}, {gyro_std[1]:5.2f}, {gyro_std[2]:5.2f}]  "
                  f"Accel Mag: {accel_mag:.3f}g", end='\r')
    
    time.sleep(0.01)
```

## Calibration

### Why Calibrate?

Gyroscopes have a small constant bias (offset) that varies with temperature and other factors. This bias causes orientation drift over time. Calibration measures and removes this bias.

### Calibration Procedure

1. **Position**: Place BOTH IMUs flat on a stable surface
2. **Orientation**: The chip/components should face UP
3. **Stability**: Keep them completely STILL during calibration
4. **Duration**: Typically 5-10 seconds (150 samples at ~30 Hz)

```python
# Calibrate before use
device.connect()
calibration = device.calibrate_gyro(num_samples=150)

# Save calibration for later use
import pickle
with open('imu_calibration.pkl', 'wb') as f:
    pickle.dump(calibration, f)

# Load saved calibration
with open('imu_calibration.pkl', 'rb') as f:
    calibration = pickle.load(f)
device.set_calibration(calibration)
```

### When to Recalibrate

- At the start of each session
- If the device has warmed up or cooled down
- If you notice significant drift in orientation
- After moving the device to a different location

## Orientation Filtering

### Madgwick Filter

The Madgwick filter fuses gyroscope and accelerometer data to estimate orientation:

- **Gyroscope**: Provides fast orientation updates but drifts over time
- **Accelerometer**: Provides gravity reference to correct drift

### Filter Parameters

**Beta (β)**: Controls filter convergence speed and noise sensitivity

- **Low beta (0.01-0.05)**: 
  - Very smooth output
  - Slow convergence
  - Good for slow movements
  
- **Medium beta (0.05-0.15)**: 
  - Balanced performance
  - Typical for most applications
  - **Recommended: 0.08**
  
- **High beta (0.15-0.3)**: 
  - Fast convergence
  - More noise
  - Good for fast movements

```python
# Slow, smooth tracking
device.enable_orientation_filter(beta=0.03)

# Fast, responsive tracking
device.enable_orientation_filter(beta=0.15)
```

### Accelerometer Gating

The filter automatically gates accelerometer correction when the device is accelerating:

```python
data = device.read_sample_with_orientation(
    accel_gate_min_g=0.85,  # Only use accel if magnitude > 0.85g
    accel_gate_max_g=1.15   # Only use accel if magnitude < 1.15g
)
```

This prevents orientation errors during rapid movement.

## Troubleshooting

### No Data Received

```
[WARN] Serial stalled for > 3.0s
```

**Solutions:**
- Check serial port name
- Verify baud rate matches device
- Check USB cable connection
- Try unplugging and replugging the device
- Check if another program is using the port

### Parse Failures

```
Parse fail: Expected 3 parts, got 2
```

**Solutions:**
- Verify data format matches expected format
- Enable debug mode: `device.read_sample(debug=True)`
- Check for firmware issues
- Verify IMU is sending data correctly

### Calibration Timeout

```
Calibration failed: only 23 samples collected
```

**Solutions:**
- Ensure IMUs are powered on
- Check serial connection
- Verify data is being transmitted
- Increase timeout: `device.calibrate_gyro(timeout_s=120.0)`

### Orientation Drift

**Solutions:**
- Recalibrate gyroscope
- Increase filter beta for faster convergence
- Check for magnetic interference
- Ensure accelerometer gating is enabled

## Advanced Topics

### Relative Orientation

Calculate relative orientation between two IMUs:

```python
from src.imu_acquisition import quat_mul, quat_inv, quat_to_euler
import numpy as np

# Get orientations
data = device.read_sample_with_orientation()
q1 = data['imu1_quat']
q2 = data['imu2_quat']

# Calculate relative rotation (IMU2 relative to IMU1)
q_rel = quat_mul(quat_inv(q1), q2)

# Convert to Euler angles
roll, pitch, yaw = quat_to_euler(q_rel)
print(f"Relative: Roll={np.rad2deg(roll):.1f}°, "
      f"Pitch={np.rad2deg(pitch):.1f}°, "
      f"Yaw={np.rad2deg(yaw):.1f}°")
```

### Custom Sampling Rate

```python
# Read at specific rate
import time

target_rate = 50  # Hz
period = 1.0 / target_rate

while True:
    start = time.time()
    
    data = device.read_sample_with_orientation()
    if data:
        # Process data
        pass
    
    # Maintain rate
    elapsed = time.time() - start
    if elapsed < period:
        time.sleep(period - elapsed)
```

### Multiple Devices

```python
# Connect to multiple IMU devices
device1 = IMUDevice(port="/dev/cu.usbmodem001")
device2 = IMUDevice(port="/dev/cu.usbmodem002")

device1.connect()
device2.connect()

device1.calibrate_gyro()
device2.calibrate_gyro()

# Read from both
while True:
    reading1 = device1.read_sample()
    reading2 = device2.read_sample()
    # Process both readings
```

## Performance

- **Sample Rate**: Typically 20-100 Hz depending on Arduino firmware
- **Latency**: < 10ms for serial communication
- **Filter Update**: ~0.1ms per update (negligible)
- **CPU Usage**: Very low (~1% on modern hardware)

## Dependencies

- `numpy`: Array operations
- `pyserial`: Serial communication
- `dataclasses`: Data structures (Python 3.7+)

## Integration with Visualization

The module is designed to work seamlessly with VPython or other visualization tools:

```python
from src import IMUDevice
from vpython import box, vector, rate

# Setup IMU
device = IMUDevice(port="/dev/cu.usbmodem9888E00A0CC02")
device.connect()
device.calibrate_gyro()
device.enable_orientation_filter()

# Create visual object
body = box(pos=vector(0,0,0), length=2, height=0.5, width=1)

# Animation loop
while True:
    rate(60)
    
    data = device.read_sample_with_orientation()
    if data and 'imu1_quat' in data:
        # Update visualization
        q = data['imu1_quat']
        # ... update body orientation from quaternion
```

See `scripts/IMU_testing.py` for a complete visualization example.

## License

Part of Master Thesis project, January 2026.
