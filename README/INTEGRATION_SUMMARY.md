# Synchronized Visualization Integration Summary

## What Was Integrated

The hardware-synchronized visualization tool has been fully integrated into the trial manager and GUI, allowing researchers to test and verify sensor connections before running trials.

## Changes Made

### 1. GUI Enhancements (`trial/trial_gui.py`)

**Added:**
- "Test Hardware" button in status panel
- Keyboard shortcut: `T` key
- Visual indicator and help text
- Callback handler for button clicks

**Location:** Bottom status panel, next to signal quality indicators

**Appearance:**
```
┌──────────────────────────────────────────┐
│  [🔍 Test Hardware (T)]                  │
│  Launch visualization to verify signals  │
└──────────────────────────────────────────┘
```

### 2. Trial Manager Integration (`trial/trial_manager.py`)

**Added:**
- `_handle_test_hardware()` method
- State validation (only in IDLE/READY)
- Subprocess launcher for visualization
- Configuration-aware (dummy vs real signals)
- User feedback dialogs

**Features:**
- Non-blocking (separate process)
- Automatically detects dummy/real signal mode
- Shows helpful instructions to user
- Error handling with fallback manual command

### 3. Visualization Tool (`src/synchronized_visualization.py`)

**Enhanced:**
- Based on `signal_acquisition_testing.py` protocol
- Hardware-synchronized timestamps
- Counter-based EMG timing
- IMU hardware timestamp sync
- Dual IMU orientation display
- EMG differential pairs

**Architecture:**
```
┌─────────────────────────────────────┐
│  IMU Acquisition Thread             │
│  - Hardware timestamp sync          │
│  - Auto-reconnect on reset          │
│  - Gyroscope calibration            │
└─────────────────────────────────────┘
           ↓ (synchronized)
┌─────────────────────────────────────┐
│  EMG Acquisition Thread             │
│  - Counter-based timing             │
│  - Differential pair calculation    │
│  - Wrap detection (16/32-bit)       │
└─────────────────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│  Real-time Plotter                  │
│  - Synchronized time axis           │
│  - Auto-scaling                     │
│  - Live statistics                  │
└─────────────────────────────────────┘
```

## Usage Workflow

### During Trial Setup

1. **Start Trial Manager**
   ```powershell
   python -m trial.trial_manager
   ```

2. **Wait for "SYSTEM READY"**
   - Devices initialized
   - GUI loaded

3. **Press `T` or Click "Test Hardware"**
   - Separate visualization window opens
   - Shows real-time EMG + IMU signals

4. **Verify Hardware:**
   - EMG differential pairs showing muscle activity
   - IMU orientation tracking movement
   - Stable sample rates
   - Good signal quality

5. **Close Visualization Window**
   - Returns to trial manager
   - Ready to begin trials

6. **Press `SPACE` to Start First Trial**

### Standalone Testing

```powershell
# Real hardware
python -m src.synchronized_visualization

# Dummy signals
python -m src.synchronized_visualization --dummy

# 60-second test
python -m src.synchronized_visualization --duration 60
```

## Key Features

### ✅ Hardware Verification
- **EMG Electrodes:** Verify connections, check signal quality
- **IMU Sensors:** Confirm orientation tracking, check calibration
- **Synchronization:** Ensure proper timing across sensors

### ✅ Non-Blocking Operation
- Runs in separate process
- Trial manager stays responsive
- Can launch multiple tests if needed

### ✅ Configuration-Aware
- Automatically uses dummy or real signals
- Respects `use_dummy_signals` setting
- Matches trial manager configuration

### ✅ State-Safe
- Only available in IDLE or READY states
- Prevents interference with running trials
- User-friendly error messages

### ✅ User-Friendly
- Clear instructions and feedback
- Visual button in GUI
- Keyboard shortcut for quick access
- Helpful error messages

## Configuration

### EMG Differential Pairs

Edit `src/synchronized_visualization.py`:

```python
EMG_DIFFERENTIAL_PAIRS = [
    (0, 1),  # Biceps: Ch0 (+) - Ch1 (-)
    (2, 3),  # Triceps: Ch2 (+) - Ch3 (-)
    (4, 5),  # Forearm: Ch4 (+) - Ch5 (-)
]
```

### Plot Settings

```python
PLOT_WINDOW_SECONDS = 5.0  # Display window
PLOT_UPDATE_INTERVAL = 33  # ~30 FPS
EMG_PLOT_SCALE = 'auto'    # Auto-scaling
```

### Hardware Settings

```python
IMU_PORT = None              # Auto-detect
IMU_BAUD = 230400
IMU_CALIBRATION_SAMPLES = 200
EMG_CONNECTION_TYPE = 'usb'
EMG_SAMPLE_RATE = 2048       # Hz
```

## Benefits

### For Researchers
1. **Pre-Trial Validation:** Catch hardware issues before participant arrives
2. **Quick Troubleshooting:** Diagnose connection problems quickly
3. **Training Tool:** Demonstrate system to new researchers
4. **Documentation:** Capture signal quality for records

### For Participants
1. **Reduced Wait Time:** Hardware verified before session
2. **Better Experience:** Fewer interruptions during trials
3. **Higher Quality Data:** Optimal sensor placement confirmed

### For Data Quality
1. **Early Detection:** Find bad electrodes before recording
2. **Calibration Verification:** Ensure IMU properly calibrated
3. **Synchronization Check:** Confirm proper timing
4. **Signal Baseline:** Establish expected signal ranges

## Technical Details

### Synchronization Protocol

**IMU (Hardware Timestamps):**
```python
# Establish time offset on first sample
time_offset = receive_time - (arduino_micros / 1_000_000)

# All subsequent samples use hardware timestamp
synchronized_time = (arduino_micros / 1_000_000) + time_offset
```

**EMG (Counter-Based):**
```python
# Unwrap counter (handles 16-bit/32-bit wrap)
counter_unwrapped = unwrap_counter(raw_counter)

# Map counter to time domain
sample_time = (counter * sample_period) + time_offset

# Adapt offset for drift tracking
time_offset = 0.995 * old_offset + 0.005 * new_offset
```

### Thread Safety

- `TimestampedBuffer`: Thread-safe with `threading.Lock`
- Separate threads for IMU and EMG acquisition
- Synchronized start via `threading.Event`
- Safe cleanup on exit

### Process Isolation

- Visualization runs in separate process via `subprocess.Popen`
- Won't block trial manager GUI
- Clean separation of concerns
- Independent error handling

## Files Modified/Created

### Modified
- `trial/trial_gui.py` - Added button and keyboard handler
- `trial/trial_manager.py` - Added launch handler

### Created
- `src/synchronized_visualization.py` - Main visualization tool
- `README/HARDWARE_TESTING.md` - User documentation
- `README/INTEGRATION_SUMMARY.md` - This file

## Testing Checklist

- [ ] Button appears in trial manager GUI
- [ ] Press `T` key launches visualization
- [ ] Click button launches visualization
- [ ] Visualization opens in separate window
- [ ] EMG differential pairs display correctly
- [ ] IMU orientation tracks movement
- [ ] Closing visualization returns to trial manager
- [ ] Works in both dummy and real hardware modes
- [ ] Only available in IDLE/READY states
- [ ] Error handling shows helpful messages

## Future Enhancements

Potential improvements:

1. **Save Snapshots:** Capture signal screenshots during test
2. **Signal Quality Metrics:** Automated SNR calculation
3. **Calibration Wizard:** Step-by-step electrode placement guide
4. **Historical Comparison:** Compare current vs previous sessions
5. **Multi-Session View:** Test multiple configurations
6. **Export Reports:** Generate signal quality reports

## Support

For issues or questions:

1. Check `README/HARDWARE_TESTING.md` for troubleshooting
2. Verify hardware connections
3. Test with dummy signals to isolate hardware issues
4. Check terminal output for error messages
5. Run visualization standalone for detailed debugging

---

**Integration Complete!** ✅

The system now provides comprehensive hardware testing capabilities directly from the trial manager interface, improving workflow efficiency and data quality.
