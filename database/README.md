# Trial Data Database

This directory contains all trial data collected from participants.

## Directory Structure

```
database/
├── participant_001/
│   ├── session_001/
│   │   ├── metadata.json           # Session metadata
│   │   ├── trial_001_raw.h5        # Raw EMG + IMU data
│   │   ├── trial_001_preprocessed.h5  # Preprocessed EMG
│   │   ├── trial_002_raw.h5
│   │   ├── trial_002_preprocessed.h5
│   │   └── ...
│   ├── session_002/
│   │   └── ...
│   └── participant_info.txt        # Optional participant notes
├── participant_002/
│   └── ...
└── README.md (this file)
```

## File Formats

### HDF5 Files

All trial data is stored in HDF5 format for efficient storage and ML compatibility.

#### Raw Data Files (`trial_XXX_raw.h5`)

Contains synchronized raw EMG and IMU data:

```
trial_001_raw.h5
├── metadata/
│   ├── participant_id: "P001"
│   ├── session_id: "S001"
│   ├── trial_number: 1
│   ├── exercise_name: "Wrist_Flexion"
│   ├── duration: 5.0
│   └── timestamp_start: "2026-01-23T10:30:00"
├── emg/
│   ├── timestamps (N,) float64     # Synchronized timestamps (seconds)
│   ├── raw (N, C) float32          # Raw EMG channels (microvolts)
│   └── channel_names [str]         # Channel names
├── imu/
│   ├── timestamps (M,) float64     # Synchronized timestamps
│   ├── accel1 (M, 3) float32       # IMU1 acceleration [ax, ay, az] (g)
│   ├── gyro1 (M, 3) float32        # IMU1 angular velocity [gx, gy, gz] (rad/s)
│   ├── accel2 (M, 3) float32       # IMU2 acceleration
│   ├── gyro2 (M, 3) float32        # IMU2 angular velocity
│   ├── quat1 (M, 4) float32        # IMU1 orientation quaternion [w, x, y, z]
│   └── quat2 (M, 4) float32        # IMU2 orientation quaternion
└── sync/
    └── time_offset: float64        # Synchronization offset
```

#### Preprocessed Data Files (`trial_XXX_preprocessed.h5`)

Contains preprocessed EMG data ready for ML:

```
trial_001_preprocessed.h5
├── metadata/
│   ├── trial_number: 1
│   ├── exercise_name: "Wrist_Flexion"
│   └── raw_file: "trial_001_raw.h5"
├── emg/
│   ├── timestamps (N,) float64
│   ├── filtered (N, C) float32     # Bandpass + notch filtered
│   ├── envelope (N, C) float32     # Rectified + smoothed
│   └── normalized (N, C) float32   # MVC-normalized (if applicable)
└── imu/ (optional copy of IMU data)
```

### Metadata JSON File (`metadata.json`)

Session-level metadata:

```json
{
  "participant": {
    "id": "P001",
    "age": 25,
    "gender": "M",
    "handedness": "right"
  },
  "session": {
    "id": "S001",
    "date": "2026-01-23",
    "time_start": "10:30:00",
    "experimenter": "Researcher Name"
  },
  "emg_config": {
    "device": "TMSi Porti7",
    "sample_rate": 2048,
    "channels_used": [0, 1, 2, 3],
    "channel_names": ["EMG_CH1", "EMG_CH2", "EMG_CH3", "EMG_CH4"]
  },
  "imu_config": {
    "device": "Dual BMI160",
    "sample_rate": 200,
    "calibration_samples": 200
  },
  "preprocessing": {
    "bandpass": [20.0, 450.0],
    "notch": 50.0,
    "envelope_cutoff": 10.0
  }
}
```

## Loading Data

### PyTorch

```python
import h5py
import torch

# Load preprocessed EMG data
with h5py.File('trial_001_preprocessed.h5', 'r') as f:
    emg_filtered = torch.from_numpy(f['emg/filtered'][:])
    emg_envelope = torch.from_numpy(f['emg/envelope'][:])
    timestamps = f['emg/timestamps'][:]

print(f"EMG shape: {emg_filtered.shape}")  # (samples, channels)
```

### Keras/TensorFlow

```python
import h5py
import numpy as np

# Load raw IMU data
with h5py.File('trial_001_raw.h5', 'r') as f:
    accel = f['imu/accel1'][:]
    gyro = f['imu/gyro1'][:]
    timestamps = f['imu/timestamps'][:]

print(f"IMU shape: {accel.shape}")  # (samples, 3)
```

### NumPy (Direct)

```python
import h5py

# Load and inspect file structure
with h5py.File('trial_001_raw.h5', 'r') as f:
    # List all datasets
    def print_structure(name, obj):
        print(name)
    f.visititems(print_structure)
    
    # Load specific dataset
    emg_data = f['emg/raw'][:]
    sample_rate = f['emg'].attrs['sample_rate']
```

## Data Processing Pipeline

1. **Raw Data** → Synchronized EMG + IMU from hardware
2. **Preprocessing** → Bandpass filter, notch filter, envelope extraction
3. **Normalization** → MVC normalization (optional)
4. **Windowing** → Create time windows for ML (done during training)
5. **Feature Extraction** → Extract features (time/frequency domain)
6. **Model Training** → Train ML model (PyTorch/Keras)

## File Naming Convention

- Participants: `participant_001`, `participant_002`, etc.
- Sessions: `session_001`, `session_002`, etc.
- Trials: `trial_001_raw.h5`, `trial_001_preprocessed.h5`, etc.

Trial numbers are 1-based and sequential within each session.

## Data Organization Best Practices

1. **One session per directory**: Each recording session gets its own folder
2. **Keep raw data**: Always save raw data for reprocessing
3. **Metadata is crucial**: Include complete metadata for reproducibility
4. **Backup regularly**: HDF5 files are binary; keep backups
5. **Document anomalies**: Note any issues in participant notes

## Storage Requirements

Typical storage per trial (5 seconds):
- Raw EMG (4 channels @ 2 kHz): ~160 KB
- Raw IMU (6-axis × 2 @ 200 Hz): ~10 KB
- Preprocessed EMG: ~160 KB
- **Total per trial**: ~330 KB

For a full session (20 trials):
- **Total**: ~6.6 MB per participant per session

## Accessing Metadata

### From Python

```python
import json
from pathlib import Path

# Load session metadata
with open('database/participant_001/session_001/metadata.json') as f:
    metadata = json.load(f)

print(f"Participant: {metadata['participant']['id']}")
print(f"Session date: {metadata['session']['date']}")
```

### From HDF5 Attributes

```python
import h5py

with h5py.File('trial_001_raw.h5', 'r') as f:
    # Read attributes
    trial_num = f['metadata'].attrs['trial_number']
    exercise = f['metadata'].attrs['exercise_name']
    
    print(f"Trial {trial_num}: {exercise}")
```

## Data Quality Checks

Before using data for ML, verify:

1. **Completeness**: All expected trials present
2. **Sample rate**: Consistent sample rate across trials
3. **Signal quality**: No excessive saturation or noise
4. **Synchronization**: EMG and IMU timestamps aligned
5. **Metadata**: All metadata fields populated

Use the provided verification scripts:

```bash
python -m trial.verify_data database/participant_001/session_001/
```

## Privacy and Ethics

- **Do NOT commit this folder to git** (already in .gitignore)
- Keep participant data secure and confidential
- Follow institutional ethics guidelines
- Anonymize data before sharing
- Obtain proper consent forms

## Support

For issues with data format or loading:
- Check HDF5 file integrity: `h5py.File(path, 'r').keys()`
- Verify file sizes (corrupted files are usually very small)
- Check trial logs in `trial_log.txt`

For questions, contact the research team.
