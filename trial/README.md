# Trial Data Collection System

Comprehensive trial data collection system for synchronized EMG and IMU signals with GUI visualization and HDF5 storage.

## Features

- **Synchronized Acquisition**: Real-time EMG and IMU data collection
- **Interactive GUI**: Tkinter-based interface with live signal plots
- **Keyboard Control**: Simple keyboard shortcuts for trial progression
- **HDF5 Storage**: ML-ready data format compatible with PyTorch/Keras
- **Preprocessing Pipeline**: Bandpass, notch filtering, envelope extraction
- **Flexible Protocols**: Configurable exercise sequences with repetitions
- **Comprehensive Metadata**: Full trial, participant, and device information

## Quick Start

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

Required packages:
- `h5py` - HDF5 file format
- `matplotlib` - Plotting and visualization
- `numpy` - Numerical computing
- `scipy` - Signal processing
- Other standard dependencies (see requirements.txt)

### 2. Configure Trial

Edit `trial/setup_trial.py`:

```python
# Participant info
PARTICIPANT_ID = "P001"
SESSION_ID = "S001"

# EMG configuration
EMG_CONFIG = {
    'sample_rate': 2048,
    'raw_channels': [0, 1, 2, 3],
    # ...
}

# Trial exercises
TRIAL_EXERCISES = [
    {
        'name': 'Wrist_Flexion',
        'duration': 5.0,
        'instruction': 'Flex your wrist and hold',
        'rest_after': 3.0,
        'repetitions': 3,
    },
    # ...more exercises...
]
```

### 3. Run Trial Collection

**With hardware (EMG + IMU devices):**
```bash
python -m trial.trial_manager
```

**Mock mode (testing without hardware):**
```bash
python -m trial.trial_manager --mock
```

### 4. Keyboard Controls

During trial collection:

- **SPACE** - Start/stop current trial
- **N** - Next trial
- **R** - Repeat current trial
- **C** - Calibrate IMU
- **Q** - Quit session

## File Structure

```
trial/
├── setup_trial.py          # Configuration (edit this!)
├── trial_manager.py        # Core orchestration
├── trial_gui.py            # GUI interface
├── data_storage.py         # HDF5 storage
├── trial_protocols.py      # Protocol definitions
├── test_simple.py          # Basic tests (no GUI)
├── test_trial_system.py    # Full integration tests
└── README.md               # This file

database/
├── participant_001/
│   └── session_001/
│       ├── metadata.json
│       ├── trial_001_raw.h5
│       ├── trial_001_preprocessed.h5
│       └── ...
└── README.md
```

## Configuration Guide

### Participant Information

```python
PARTICIPANT_ID = "P001"  # Unique participant ID
PARTICIPANT_INFO = {
    'age': 25,
    'gender': 'M',
    'handedness': 'right',
    'medical_notes': '',
}
```

### EMG Settings

```python
EMG_CONFIG = {
    'connection_type': 'usb',        # Connection method
    'sample_rate': 2048,             # Sampling rate (Hz)
    'raw_channels': [0, 1, 2, 3],    # Channel indices to record
    'differential_pairs': None,      # Optional: [(0,1), (2,3)]
    'channel_names': ['CH1', ...],   # Optional channel labels
}
```

### IMU Settings

```python
IMU_CONFIG = {
    'port': None,                    # Auto-detect serial port
    'calibration_samples': 200,      # Samples for gyro calibration
    'auto_calibrate_on_start': True, # Auto-calibrate on startup
    'use_mahony': True,              # Enable orientation filter
}
```

### Trial Protocol

```python
TRIAL_EXERCISES = [
    {
        'name': 'Exercise_Name',       # Used in filenames
        'duration': 5.0,               # Trial duration (seconds)
        'instruction': 'Instructions', # Shown to participant
        'rest_after': 3.0,             # Rest after trial (seconds)
        'repetitions': 3,              # Number of repetitions
    },
    # ... more exercises
]
```

### Preprocessing

```python
PREPROCESSING = {
    'bandpass_low': 20.0,       # Bandpass low cutoff (Hz)
    'bandpass_high': 450.0,     # Bandpass high cutoff (Hz)
    'notch_freq': 50.0,         # Powerline frequency (50 or 60 Hz)
    'envelope_cutoff': 10.0,    # Envelope smoothing (Hz)
    'mvc_normalization': False, # MVC normalization (after calibration)
}
```

## Data Format

### HDF5 File Structure

**Raw Data** (`trial_XXX_raw.h5`):
```
├── metadata/          # Trial information
├── emg/
│   ├── timestamps     # (N,) float64 - seconds
│   ├── raw           # (N, C) float32 - microvolts
│   └── channel_names # String array
├── imu/
│   ├── timestamps    # (M,) float64 - seconds
│   ├── accel1        # (M, 3) float32 - g
│   ├── gyro1         # (M, 3) float32 - rad/s
│   ├── quat1         # (M, 4) float32 - quaternion
│   └── ... (IMU2 data)
└── sync/             # Synchronization info
```

**Preprocessed Data** (`trial_XXX_preprocessed.h5`):
```
├── metadata/
├── emg/
│   ├── timestamps
│   ├── filtered      # Bandpass + notch filtered
│   ├── envelope      # Rectified + smoothed
│   └── normalized    # MVC-normalized (if applicable)
└── imu/ (optional copy)
```

### Loading Data in Python

**PyTorch:**
```python
import h5py
import torch

with h5py.File('trial_001_preprocessed.h5', 'r') as f:
    emg_data = torch.from_numpy(f['emg/filtered'][:])
    timestamps = f['emg/timestamps'][:]

print(emg_data.shape)  # (samples, channels)
```

**Keras/TensorFlow:**
```python
import h5py

with h5py.File('trial_001_raw.h5', 'r') as f:
    emg_data = f['emg/raw'][:]
    imu_data = f['imu/accel1'][:]
```

## Protocols

### Built-in Protocols

Select protocol in `setup_trial.py`:

```python
PROTOCOL_NAME = 'basic'  # 'basic', 'full', 'gesture_recognition', 'mvc_calibration'
```

Available protocols:
- **basic**: Essential hand/wrist movements (5 exercises)
- **pronation_supination**: Forearm rotation (3 exercises)
- **mvc_calibration**: Maximum voluntary contraction (3 exercises)
- **gesture_recognition**: Common gestures (6 exercises)
- **full**: Comprehensive movement set (9 exercises)
- **custom**: Use `TRIAL_EXERCISES` in setup_trial.py

### Creating Custom Protocols

```python
from trial.trial_protocols import create_custom_protocol

protocol = create_custom_protocol(
    exercises=['Rest', 'Movement_A', 'Movement_B'],
    durations=[5, 3, 3],
    instructions=['Relax', 'Do A', 'Do B'],
    rest_after=2.0,
    repetitions=3
)
```

## Testing

### Basic Test (No GUI)

Tests configuration and protocols without requiring matplotlib:

```bash
python trial/test_simple.py
```

### Full Integration Test

Tests entire system including GUI (requires all dependencies):

```bash
python trial/test_trial_system.py
```

### Manual Testing

Test with mock data (no hardware needed):

```bash
python -m trial.trial_manager --mock
```

## Troubleshooting

### Import Errors

If you see `ModuleNotFoundError`:
```bash
pip install -r requirements.txt
```

### Unicode Encoding Issues (Windows)

Unicode characters (✓, ⚠) may not display correctly in Windows console. The system handles this automatically by falling back to ASCII equivalents.

### Device Connection Issues

**EMG Device Not Found:**
- Check USB connection
- Verify TMSi drivers are installed
- Try different USB port

**IMU Device Not Found:**
- Check Arduino serial connection
- Verify correct sketch is uploaded
- Check baud rate (default: 230400)

**Calibration Failed:**
- Ensure IMUs are flat and stationary
- Wait for sensors to stabilize
- Retry calibration with 'C' key

### Data Storage Issues

**Permission Denied:**
- Check database directory permissions
- Ensure sufficient disk space

**File Size Large:**
- HDF5 files use compression by default
- Adjust compression level in `setup_trial.py`

## Performance Tips

1. **Reduce Plot Update Rate**: Increase `plot_update_interval_ms` in GUI config
2. **Use Fewer Channels**: Select only necessary EMG channels
3. **Compression**: Enable GZIP compression (default)
4. **Buffer Sizes**: Adjust buffer sizes for longer trials

## System Requirements

- **Python**: 3.8 or higher
- **OS**: Windows, macOS, Linux
- **RAM**: 4 GB minimum, 8 GB recommended
- **Storage**: ~10 MB per participant session
- **Display**: 1400x900 or higher recommended

## Architecture

```
┌─────────────┐
│   GUI       │  ← User interaction
└──────┬──────┘
       │
┌──────▼──────────────────┐
│   Trial Manager         │  ← State machine & coordination
└──────┬──────────────────┘
       │
       ├─────────────┬─────────────┬──────────────┐
       │             │             │              │
┌──────▼──────┐ ┌───▼────┐  ┌────▼─────┐  ┌────▼─────┐
│ Acquisition │ │Preproc │  │ Storage  │  │ Protocols│
└─────────────┘ └────────┘  └──────────┘  └──────────┘
       │
       ├─────────────┬
       │             │
   ┌───▼───┐    ┌───▼───┐
   │  EMG  │    │  IMU  │
   └───────┘    └───────┘
```

## Support

For issues or questions:
1. Check this README
2. Review database/README.md for data format details
3. Run tests: `python trial/test_simple.py`
4. Check trial logs in session directory

## Citation

If you use this system in your research, please cite:

```
EMG+IMU Trial Data Collection System
Version 1.0.0
https://github.com/your-repo
```

## License

See LICENSE file in repository root.

## Changelog

### Version 1.0.0 (2026-01-23)
- Initial release
- Synchronized EMG+IMU acquisition
- GUI interface with real-time visualization
- HDF5 storage with comprehensive metadata
- Configurable protocols
- Preprocessing pipeline
- Mock mode for testing
