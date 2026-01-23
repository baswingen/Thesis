# EMG Signal Preprocessing Module

Complete preprocessing pipeline for EMG signals with MVC normalization, integrated as a native module in the codebase.

## Location

**Module:** `src/emg_processing.py`

The preprocessing capabilities have been added to the existing EMG processing module, making them easily accessible throughout the codebase.

## Features

### EMGPreprocessor Class

Complete preprocessing pipeline that combines:
1. **Bandpass filtering** (20-450 Hz) - Removes DC offset and high-frequency noise
2. **Notch filtering** (50/60 Hz) - Removes powerline interference  
3. **Rectification + Smoothing** - Extracts envelope via `EMGEnvelopeExtractor`
4. **MVC Normalization** - Normalizes to 0-1 range based on Maximum Voluntary Contraction

All filters are **stateful**, avoiding edge artifacts in real-time processing.

### MVC Calibration Functions

- `calibrate_mvc_from_data()` - Calculate MVC from recorded data
- `calibrate_mvc_interactive()` - Interactive calibration with user prompts

## Quick Start

### Basic Usage

```python
from src.emg_processing import EMGPreprocessor

# Create preprocessor
preprocessor = EMGPreprocessor(
    fs=1024,              # Sample rate (Hz)
    mvc_value=100.0,      # MVC value for normalization
    bandpass_low=20.0,    # Low cutoff (Hz)
    bandpass_high=450.0,  # High cutoff (Hz)
    notch_freq=50.0,      # Notch frequency (Hz)
    envelope_cutoff=10.0  # Envelope smoothing (Hz)
)

# Process EMG signal
normalized_signal = preprocessor.process(raw_emg)

# Or get all processing stages
result = preprocessor.process(raw_emg, return_all_stages=True)
# result contains: 'raw', 'filtered', 'envelope', 'normalized'
```

### With MVC Calibration

```python
from src.emg_processing import EMGPreprocessor, calibrate_mvc_from_data

# Step 1: Record maximum contraction
mvc_recording = acquire_max_contraction(duration=5.0)

# Step 2: Calculate MVC value
mvc_value = calibrate_mvc_from_data(
    mvc_recording,
    fs=1024,
    percentile=95.0  # Use 95th percentile to avoid outliers
)

# Step 3: Create preprocessor with MVC
preprocessor = EMGPreprocessor(fs=1024, mvc_value=mvc_value)

# Step 4: Process signals
normalized = preprocessor.process(raw_emg)
```

### Real-Time Streaming

```python
from src.emg_processing import EMGPreprocessor

preprocessor = EMGPreprocessor(fs=1024, mvc_value=50.0)

# Process incoming chunks (filter states maintained between calls)
while acquiring:
    chunk = get_next_emg_chunk()
    normalized = preprocessor.process(chunk)
    # Use normalized signal for ML, visualization, etc.
```

## API Reference

### EMGPreprocessor

```python
EMGPreprocessor(
    fs: float,                          # Sample rate (Hz)
    mvc_value: Optional[float] = None,  # MVC value for normalization
    bandpass_low: float = 20.0,         # Bandpass low cutoff (Hz)
    bandpass_high: float = 450.0,       # Bandpass high cutoff (Hz)
    notch_freq: Optional[float] = 50.0, # Notch frequency (Hz), None to disable
    envelope_cutoff: float = 10.0,      # Envelope cutoff (Hz)
    bandpass_order: int = 4,            # Bandpass filter order
    envelope_order: int = 4             # Envelope filter order
)
```

**Methods:**
- `process(raw_emg, return_all_stages=False)` - Process EMG signal
- `set_mvc_value(mvc_value)` - Update MVC value
- `reset()` - Reset filter states

### calibrate_mvc_from_data()

```python
calibrate_mvc_from_data(
    emg_data: np.ndarray,               # Raw EMG from max contraction
    fs: float,                          # Sample rate (Hz)
    percentile: float = 95.0,           # Percentile for MVC (95th recommended)
    bandpass_low: float = 20.0,         # Preprocessing parameters...
    bandpass_high: float = 450.0,
    notch_freq: Optional[float] = 50.0,
    envelope_cutoff: float = 10.0
) -> float
```

Returns the MVC value suitable for normalization.

## Complete Example: Integration with Synchronized Acquisition

```python
from src.synchronized_acquisition import SynchronizedAcquisition, SyncConfig
from src.emg_processing import EMGPreprocessor, calibrate_mvc_from_data
import numpy as np

# Initialize acquisition
sync_config = SyncConfig(...)  # Your config
sync_acq = SynchronizedAcquisition(sync_config)
sync_acq.start()

# Step 1: MVC Calibration
print("Perform maximum contraction for 5 seconds...")
time.sleep(2)

mvc_samples = []
start = time.perf_counter()
while time.perf_counter() - start < 5.0:
    chunks = sync_acq.emg_buffer.get_recent(now_t=time.perf_counter(), window_s=0.5)
    for t, chunk in chunks:
        if 'pairs' in chunk.data:
            mvc_samples.append(chunk.data['pairs'][:, 0])
    time.sleep(0.05)

mvc_data = np.concatenate(mvc_samples)
mvc_value = calibrate_mvc_from_data(mvc_data, fs=1024)
print(f"MVC value: {mvc_value:.4f}")

# Step 2: Create preprocessor
preprocessor = EMGPreprocessor(fs=1024, mvc_value=mvc_value)

# Step 3: Real-time preprocessing
while True:
    chunks = sync_acq.emg_buffer.get_recent(now_t=time.perf_counter(), window_s=1.0)
    for t, chunk in chunks:
        if 'pairs' in chunk.data:
            raw_emg = chunk.data['pairs'][:, 0]
            normalized = preprocessor.process(raw_emg)
            # Use normalized signal for ML, control, etc.
```

## Default Parameters

The default preprocessing parameters are based on EMG literature:

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `bandpass_low` | 20 Hz | Remove DC offset and motion artifacts |
| `bandpass_high` | 450 Hz | Remove high-frequency noise |
| `notch_freq` | 50 Hz | Remove powerline (50 Hz EU, 60 Hz US) |
| `envelope_cutoff` | 10 Hz | Smooth muscle activation envelope |
| `mvc_percentile` | 95% | Avoid outliers in MVC calculation |

## Pipeline Visualization

```
Raw EMG Signal
     ↓
Bandpass Filter (20-450 Hz)
     ↓
Notch Filter (50 Hz)
     ↓
Rectification (abs value)
     ↓
Low-Pass Filter (10 Hz) - Envelope
     ↓
MVC Normalization (÷ MVC value)
     ↓
Normalized Signal (0-1 range)
```

## IMU Preprocessing

IMU signals from `src/imu_acquisition.py` already include:
- **Gyroscope bias calibration** during initialization
- **AHRS filtering** (Mahony filter) for orientation estimation
- **Clean sensor data** from BMI160

Therefore, **no additional preprocessing is needed for IMU**. Use the signals directly:
- `reading.gyro1`, `reading.gyro2` - Angular velocity (rad/s)
- `reading.accel1`, `reading.accel2` - Acceleration (g)
- `reading.euler1`, `reading.euler2` - Orientation (roll, pitch, yaw in degrees)

## Examples

See `tests/examples_emg_preprocessing.py` for comprehensive usage examples:
1. Basic preprocessing without MVC
2. Preprocessing with MVC normalization
3. Accessing all processing stages
4. Real-time streaming simulation
5. Dynamic MVC updates
6. Using individual filter components

## Integration with Existing Scripts

The preprocessing module is already integrated into:
- **`scripts/signal_preprocessing.py`** - Real-time preprocessing with visualization
- **`src/emg_processing.py`** - Native module (can be imported anywhere)

## Benefits

✅ **Centralized** - All preprocessing in one native module  
✅ **Reusable** - Easy to import and use anywhere in codebase  
✅ **Stateful filters** - Optimized for real-time streaming  
✅ **ML-ready output** - Normalized, smoothed, filtered signals  
✅ **Well-documented** - Clear API and usage examples  
✅ **Flexible** - Configurable parameters for different use cases

## Related Documentation

- [EMG Acquisition](README_EMG.md)
- [IMU Acquisition](README_IMU.md)
- [Synchronized Acquisition](README_SIGNAL_ACQUISITION.md)
- [Example Usage](../tests/examples_emg_preprocessing.py)
