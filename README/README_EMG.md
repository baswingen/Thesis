# EMG Acquisition Module

Native Python interface for acquiring EMG signals from TMSi Porti7/REFA devices.

## Quick Start

### Basic Usage

```python
from src import EMGDevice

# Create and connect
emg = EMGDevice(connection_type='usb')
emg.connect()

# Acquire data for 10 seconds
emg.start_acquisition(duration=10.0)

# Get the data
data = emg.get_data()
print(f"Acquired {data.shape[0]} samples from {data.shape[1]} channels")

# Save data
emg.export_data('my_emg_data.npy')

# Cleanup
emg.disconnect()
```

### One-Line Quick Acquisition

```python
from src import acquire_emg_data

# Acquire 5 seconds of data and save it
data, info = acquire_emg_data(duration=5.0, export_to='data.npy')
```

### Context Manager (Automatic Cleanup)

```python
from src import EMGDevice

with EMGDevice() as emg:
    emg.connect()
    emg.start_acquisition(duration=10.0)
    data = emg.get_data()
    emg.export_data('data.csv')
# Automatically disconnects
```

## API Reference

### EMGDevice Class

#### Initialization
```python
emg = EMGDevice(connection_type='usb')
```
**Parameters:**
- `connection_type`: 'usb', 'bluetooth', 'network', 'wifi', 'optical'

#### Methods

##### `discover_devices() -> List[str]`
Find all available TMSi devices.

##### `connect(device_index=0) -> dict`
Connect to device and return device info.

##### `disconnect()`
Disconnect and cleanup.

##### `configure_acquisition(sample_rate=None, reference_calculation=True)`
Configure acquisition parameters.
- `sample_rate`: Desired rate in Hz (None = device default)
- `reference_calculation`: Enable common average reference

##### `start_acquisition(duration=None)`
Start acquiring data.
- `duration`: Time in seconds (None = continuous until stopped)

##### `stop_acquisition()`
Stop data acquisition.

##### `get_samples(blocking=False) -> np.ndarray`
Get latest samples.
- Returns: Array of shape (num_samples, num_channels)

##### `get_data() -> np.ndarray`
Get all acquired data as single array.

##### `export_data(filename, data=None)`
Export data to file (.npy, .csv, or .txt).

##### `get_device_info() -> dict`
Get device information dictionary.

##### `get_channel_names() -> List[str]`
Get list of channel names.

##### `get_channel_info() -> List[dict]`
Get detailed channel information.

##### `clear_buffer()`
Clear internal sample buffer.

## Examples

### Example 1: Continuous Acquisition

```python
from src import EMGDevice
import time

emg = EMGDevice()
emg.connect()

# Configure for 2000 Hz
emg.configure_acquisition(sample_rate=2000)

# Start continuous acquisition
emg.start_acquisition()

# Collect for 30 seconds
for i in range(30):
    samples = emg.get_samples()
    if samples is not None:
        print(f"Got {samples.shape[0]} samples")
    time.sleep(1)

emg.stop_acquisition()
data = emg.get_data()
emg.export_data('continuous_data.npy')
emg.disconnect()
```

### Example 2: Multiple Acquisitions

```python
from src import EMGDevice

emg = EMGDevice()
emg.connect()

# Acquire multiple trials
for trial in range(5):
    print(f"Trial {trial + 1}")
    emg.clear_buffer()  # Clear previous data
    
    emg.start_acquisition(duration=5.0)
    data = emg.get_data()
    
    emg.export_data(f'trial_{trial + 1}.csv')

emg.disconnect()
```

### Example 3: Channel Selection

```python
from src import EMGDevice

emg = EMGDevice()
emg.connect()

# Get channel information
channels = emg.get_channel_names()
print(f"Available channels: {channels}")

# Acquire data
emg.start_acquisition(duration=10.0)
data = emg.get_data()

# Extract specific channels (e.g., first 8 EMG channels)
emg_data = data[:, :8]
print(f"EMG data shape: {emg_data.shape}")

emg.disconnect()
```

### Example 4: Real-Time Processing

```python
from src import EMGDevice
import numpy as np

emg = EMGDevice()
emg.connect()
emg.start_acquisition()

try:
    while True:
        samples = emg.get_samples(blocking=False)
        
        if samples is not None:
            # Real-time processing
            rms = np.sqrt(np.mean(samples**2, axis=0))
            print(f"RMS values: {rms[:5]}")  # First 5 channels
        
except KeyboardInterrupt:
    print("Stopped by user")

emg.stop_acquisition()
emg.disconnect()
```

## Data Format

### Acquired Data
- Type: NumPy array
- Shape: `(num_samples, num_channels)`
- Values: Physical units (e.g., µV for EMG)
- NaN: Indicates disconnected/overflow channels

### Device Info Dictionary
```python
{
    'name': 'USB 207110023',
    'serial_number': 207110023,
    'num_channels': 38,
    'sample_rate': 2000,
    'connection_type': 'usb',
    'is_acquiring': False
}
```

## File Formats

### NumPy (.npy)
- Binary format, fastest
- Preserves exact precision
- Use `np.load('file.npy')` to read

### CSV (.csv)
- Text format, comma-separated
- Includes channel names as header
- Easily imported to Excel/MATLAB

### Text (.txt)
- Tab-separated text format
- Includes channel names as header

## Error Handling

```python
from src import EMGDevice

try:
    emg = EMGDevice()
    emg.connect()
    emg.start_acquisition(duration=10.0)
    data = emg.get_data()
except RuntimeError as e:
    print(f"Error: {e}")
finally:
    if emg._is_connected:
        emg.disconnect()
```

## Tips

1. **Use context manager** for automatic cleanup
2. **Clear buffer** between trials with `emg.clear_buffer()`
3. **Check for NaN** values in disconnected channels
4. **Use blocking=True** in `get_samples()` for reliable data
5. **Export regularly** to avoid memory issues with long acquisitions

## Troubleshooting

### No devices found
- Check device is powered on and connected
- Verify USB/Bluetooth connection
- Check TMSi drivers are installed

### Sample rate issues
- Device may adjust requested rate to nearest supported value
- Check actual rate with `emg.sample_rate` after configuration

### Memory issues
- For long acquisitions, export data periodically
- Use `clear_buffer()` between trials
- Consider using continuous mode with manual sample collection

## Requirements

- Python 3.8+
- NumPy
- TMSi Python Interface (in `tmsi-python-interface/`)
- TMSiSDK.dll (in legacy device directory)
- TMSi drivers installed (Windows)
