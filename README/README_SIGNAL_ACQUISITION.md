# Synchronized IMU-EMG Signal Acquisition

Real-time synchronized acquisition and visualization of dual BMI160 IMU data (~200Hz) and TMSi EMG data (~2000Hz+) with timestamp-based alignment.

## Overview

The `signal_acquisition_testing.py` script provides a complete system for:
- **Dual IMU acquisition** via Arduino with auto-detection
- **Multi-channel EMG acquisition** via TMSi device
- **Synchronized timestamping** for data alignment
- **Real-time visualization** with matplotlib
- **Thread-safe buffering** for high-throughput data

## Features

✅ **Cross-platform Arduino auto-detection** - No manual port configuration needed  
✅ **Synchronized data streams** - IMU and EMG data aligned by timestamp  
✅ **Real-time plots** - Live visualization of orientation and EMG signals  
✅ **Thread-safe** - Separate threads for IMU, EMG, and visualization  
✅ **Configurable** - Easy to customize sampling, display, and buffering  
✅ **Robust** - Auto-reconnection and error handling

## Hardware Requirements

### IMU System
- Arduino Uno R4 (or compatible) with dual BMI160 IMUs
- USB connection
- Works on **Windows, macOS, and Linux** with auto-detection

### EMG System (Optional)
- TMSi Porti7 or REFA device
- USB connection
- **Windows only** (TMSi SDK is Windows-specific)
- EMG electrodes configured as differential pairs

### Mixed Setup
You can run:
- **IMU only** - Set `ENABLE_EMG = False` (works on all platforms)
- **EMG only** - Set `ENABLE_IMU = False` (Windows only)
- **Both** - Full synchronized acquisition (Windows only)

## Quick Start

### 1. Prerequisites

Install required packages:
```bash
pip install pyserial numpy matplotlib
```

For EMG acquisition (Windows only):
```bash
# TMSi interface is included in src/
# No additional installation needed
```

### 2. Upload Arduino Sketch

Upload `arduino/IMU_sketch_I2C/IMU_sketch_I2C.ino` to your Arduino:
1. Open in Arduino IDE
2. Select board: Arduino Uno R4
3. Upload

### 3. Configure the Script

Edit `scripts/signal_acquisition_testing.py`:

```python
# Enable/disable devices
ENABLE_EMG = False   # Set True if you have TMSi device (Windows only)
ENABLE_IMU = True    # Set True if you have Arduino with IMUs

# IMU settings (auto-detection enabled by default)
IMU_PORT = None      # None = auto-detect (recommended)
IMU_BAUD = 230400

# EMG settings (if using TMSi)
EMG_DIFFERENTIAL_PAIRS = [
    (0, 1),  # Pair 1: Channel 0 (+) - Channel 1 (-)
    (2, 3),  # Pair 2: Channel 2 (+) - Channel 3 (-)
]

# Acquisition settings
ACQUISITION_DURATION = 60.0    # seconds (None = run until Ctrl+C)
PLOT_WINDOW_SECONDS = 5.0      # Time window to display
```

### 4. Run the Script

```bash
python scripts/signal_acquisition_testing.py
```

The script will:
1. Auto-detect Arduino (if `ENABLE_IMU = True`)
2. Connect to TMSi device (if `ENABLE_EMG = True`)
3. Calibrate IMUs (place flat and still!)
4. Start synchronized data acquisition
5. Display real-time plots

Press **Ctrl+C** to stop acquisition and see statistics.

## Configuration Options

### System Configuration

```python
# Enable/disable devices independently
ENABLE_EMG = False   # TMSi EMG acquisition (Windows only)
ENABLE_IMU = True    # Dual BMI160 IMU acquisition (cross-platform)
```

### EMG Configuration

```python
# Differential pairs: (positive_channel, negative_channel)
EMG_DIFFERENTIAL_PAIRS = [
    (0, 1),  # Muscle pair 1
    (2, 3),  # Muscle pair 2
    # Add more pairs as needed
]

EMG_CONNECTION_TYPE = 'usb'  # 'usb', 'bluetooth', 'network', 'wifi'
EMG_SAMPLE_RATE = None       # None = use device default (2048 or 4096 Hz)
```

### IMU Configuration

```python
# Arduino auto-detection (recommended)
IMU_PORT = None  # None = auto-detect Arduino on any platform
IMU_BAUD = 230400
IMU_CALIBRATION_SAMPLES = 200

# Or specify port manually:
# IMU_PORT = "COM3"                     # Windows
# IMU_PORT = "/dev/cu.usbmodem14201"    # macOS
# IMU_PORT = "/dev/ttyACM0"             # Linux
```

### Acquisition Settings

```python
ACQUISITION_DURATION = 60.0    # seconds (None = indefinite)
PLOT_WINDOW_SECONDS = 5.0      # Time window displayed in plots
PLOT_UPDATE_INTERVAL = 33      # milliseconds (~30 FPS)
MAX_BUFFER_SIZE = 100000       # Maximum samples in memory
```

### Visualization Settings

```python
SHOW_IMU1 = True              # Show IMU1 orientation plot
SHOW_IMU2 = True              # Show IMU2 orientation plot
EMG_PLOT_SCALE = 'auto'       # 'auto' or specific range like (-500, 500)
```

### Debug Settings

```python
VERBOSE = False                # Print detailed acquisition info
SHOW_TIMING_STATS = True       # Display timing statistics
```

## Usage Examples

### IMU Only (Cross-Platform)

Perfect for testing on macOS/Linux or without EMG hardware:

```python
ENABLE_EMG = False
ENABLE_IMU = True
IMU_PORT = None  # Auto-detect
```

Run:
```bash
python scripts/signal_acquisition_testing.py
```

### EMG Only (Windows)

Test TMSi device without IMUs:

```python
ENABLE_EMG = True
ENABLE_IMU = False

EMG_DIFFERENTIAL_PAIRS = [(0, 1), (2, 3)]
```

### Full Synchronized Acquisition (Windows)

Both IMU and EMG with timestamp synchronization:

```python
ENABLE_EMG = True
ENABLE_IMU = True

IMU_PORT = None  # Auto-detect
EMG_DIFFERENTIAL_PAIRS = [(0, 1), (2, 3), (4, 5), (6, 7)]
```

### Long-Duration Recording

```python
ACQUISITION_DURATION = 300.0  # 5 minutes
PLOT_WINDOW_SECONDS = 10.0    # Show last 10 seconds
MAX_BUFFER_SIZE = 500000      # Larger buffer
```

## Real-Time Visualization

The script creates a matplotlib window with subplots:

### IMU Plots (if enabled)
- **IMU1 Orientation**: Roll, Pitch, Yaw over time
- **IMU2 Orientation**: Roll, Pitch, Yaw over time
- Color-coded: Roll (red), Pitch (green), Yaw (blue)

### EMG Plots (if enabled)
- **Differential Pairs**: Each pair in separate subplot
- Auto-scaling or fixed range
- High-frequency EMG signals (~2000 Hz)

All plots show the last N seconds (configurable via `PLOT_WINDOW_SECONDS`).

## Data Structure

### IMU Data
```python
@dataclass
class IMUReading:
    timestamp: float       # Python time.time()
    t_us: int             # Arduino microseconds
    seq: int              # Sequence number
    
    gyro1: np.ndarray     # [gx, gy, gz] in rad/s
    accel1: np.ndarray    # [ax, ay, az] in g
    quat1: np.ndarray     # [w, x, y, z] quaternion
    euler1: tuple         # (roll, pitch, yaw) in degrees
    
    gyro2: np.ndarray     # Same for IMU2
    accel2: np.ndarray
    quat2: np.ndarray
    euler2: tuple
```

### EMG Data
```python
@dataclass
class EMGData:
    timestamp: float      # Python time.time()
    pairs: np.ndarray     # Differential pair values
    sample_count: int     # Samples in this chunk
```

## Thread Architecture

```
Main Thread
│
├─► IMU Thread (daemon)
│   ├─ Connect to Arduino (auto-detect)
│   ├─ Calibrate gyros
│   ├─ Read stream (~200 Hz)
│   └─ Buffer timestamped data
│
├─► EMG Thread (daemon)
│   ├─ Connect to TMSi device
│   ├─ Configure differential pairs
│   ├─ Read stream (~2000 Hz)
│   └─ Buffer timestamped data
│
└─► Visualization Thread (main)
    ├─ Matplotlib animation
    ├─ Query recent data from buffers
    └─ Update plots (~30 FPS)
```

All threads use **thread-safe buffers** for data exchange.

## Performance

### Typical Rates
- **IMU**: ~200 Hz (Arduino sampling rate)
- **EMG**: ~2000-4096 Hz (TMSi device rate)
- **Visualization**: ~30 FPS (plot update rate)

### Latency
- **IMU**: 5-10 ms (serial + processing)
- **EMG**: 10-20 ms (USB + processing)
- **Total**: <50 ms end-to-end

### Synchronization
- Both streams use `time.time()` for timestamping
- Sub-millisecond alignment possible
- Suitable for real-time analysis and control

## Troubleshooting

### Arduino Not Detected

**Issue**: `RuntimeError: No serial port found`

**Solutions**:
1. Check USB cable is connected
2. Verify Arduino is powered and programmed
3. Try unplugging and replugging
4. Manually specify port:
   ```python
   IMU_PORT = "COM3"  # Windows
   IMU_PORT = "/dev/cu.usbmodem14201"  # macOS
   ```

### IMU Calibration Fails

**Issue**: Calibration error or high drift

**Solutions**:
1. Place IMUs completely flat on stable surface
2. Ensure no vibration during calibration
3. Wait for IMUs to warm up (2 minutes)
4. Increase calibration samples:
   ```python
   IMU_CALIBRATION_SAMPLES = 500
   ```

### EMG Device Not Found

**Issue**: `TMSi device not found` (Windows only)

**Solutions**:
1. Check TMSi device is connected via USB
2. Verify TMSi drivers are installed
3. Try different USB port
4. Check Windows Device Manager for device

### Choppy Visualization

**Issue**: Plot update is slow or laggy

**Solutions**:
1. Reduce plot update rate:
   ```python
   PLOT_UPDATE_INTERVAL = 100  # Update every 100ms
   ```
2. Reduce plot window:
   ```python
   PLOT_WINDOW_SECONDS = 3.0  # Show less data
   ```
3. Disable unused plots:
   ```python
   SHOW_IMU2 = False  # If only using one IMU
   ```

### High Memory Usage

**Issue**: Script uses too much memory during long recordings

**Solutions**:
1. Reduce buffer size:
   ```python
   MAX_BUFFER_SIZE = 50000  # Keep fewer samples
   ```
2. Use shorter plot window:
   ```python
   PLOT_WINDOW_SECONDS = 3.0
   ```

### Platform-Specific Issues

#### macOS
- EMG not supported (Windows only)
- Set `ENABLE_EMG = False`
- IMU works perfectly with auto-detection

#### Linux
- EMG not supported (Windows only)
- May need to add user to `dialout` group:
  ```bash
  sudo usermod -a -G dialout $USER
  # Log out and back in
  ```

#### Windows
- Full support for both IMU and EMG
- TMSi SDK requires Windows

## Output Statistics

When you stop acquisition (Ctrl+C), the script displays:

```
ACQUISITION STATISTICS
======================================================================

IMU:
  Total samples: 12450
  Duration: 62.35 seconds
  Average rate: 199.7 Hz
  Buffer size: 12450

EMG:
  Total samples: 128000
  Total chunks: 640
  Duration: 62.35 seconds
  Average rate: 2052.8 Hz
  Buffer size: 100000

======================================================================
✓ SHUTDOWN COMPLETE
======================================================================
```

## Advanced Usage

### Custom Data Processing

You can access the raw buffers for custom processing:

```python
# In your own code
from scripts.signal_acquisition_testing import TimestampedBuffer

# Access IMU data
imu_data = imu_buffer.get_recent(window_seconds=5.0)
for timestamp, reading in imu_data:
    # Process IMU data
    print(f"IMU1 Euler: {reading.euler1}")

# Access EMG data
emg_data = emg_buffer.get_recent(window_seconds=5.0)
for timestamp, data in emg_data:
    # Process EMG data
    print(f"EMG Pairs: {data.pairs}")
```

### Save Data to File

Add data logging (future enhancement):

```python
import csv

# In acquisition thread
with open('imu_data.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['timestamp', 'roll1', 'pitch1', 'yaw1', ...])
    
    for reading in device.read_stream():
        writer.writerow([
            reading.timestamp,
            reading.euler1[0],
            reading.euler1[1],
            reading.euler1[2],
            # ...
        ])
```

## Related Scripts

- **`IMU_testing.py`** - IMU-only acquisition with VPython 3D visualization
- **`EMG_testing.py`** - EMG-only acquisition and processing
- **`signal_acquisition_testing.py`** - This script (synchronized IMU+EMG)

## Technical Details

### Arduino Protocol
- **ASCII format**: `$seq,t_us,ax1,ay1,...,checksum`
- **Baud rate**: 230400 (configurable)
- **Sample rate**: ~200 Hz (Arduino-side)
- **Checksum**: XOR validation

### EMG Protocol
- **TMSi SDK**: Native Python interface
- **Sample rate**: Device-dependent (2048-4096 Hz)
- **Differential pairs**: Calculated as `ch_pos - ch_neg`

### Synchronization
- All data timestamped with `time.time()` (system clock)
- Thread-safe deque buffers with locks
- Query by time window for aligned data

## References

- [BMI160 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi160/)
- [Mahony Filter Paper](http://ieeexplore.ieee.org/document/4608934/)
- [TMSi Documentation](https://www.tmsi.com/)

## License

Same as parent project (see main LICENSE file).

## Support

For issues or questions:
1. Check this README first
2. Verify hardware connections
3. Test each device independently (`ENABLE_EMG/IMU = False`)
4. Check terminal output for detailed error messages
