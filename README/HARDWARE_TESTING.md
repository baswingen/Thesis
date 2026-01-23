# Hardware Testing and Visualization

## Quick Hardware Test

Before running full trials, you can test your hardware setup using the integrated visualization tool.

### From Trial Manager

1. **Start the trial manager:**
   ```powershell
   python -m trial.trial_manager
   ```

2. **Press `T` key or click "Test Hardware" button**
   - A separate window will open showing real-time signals
   - Verify EMG electrode connections
   - Check IMU sensor readings
   - Ensure proper signal quality

3. **Close the visualization window when done**
   - Return to the trial manager to begin trials

### Standalone Visualization

You can also run the visualization tool independently:

```powershell
# Test with real hardware
python -m src.synchronized_visualization

# Test with dummy signals (no hardware required)
python -m src.synchronized_visualization --dummy

# Run for specific duration
python -m src.synchronized_visualization --duration 60
```

## What to Check

### EMG Signals
- ✅ **Differential pairs showing activity** when muscles contract
- ✅ **Low baseline noise** when relaxed
- ✅ **Stable sample rate** (should be 2048 Hz or configured rate)
- ❌ **Flat lines** = disconnected electrodes
- ❌ **60Hz noise** = poor electrode contact
- ❌ **Saturation** = amplifier overload

### IMU Signals
- ✅ **Orientation changes** when moving IMU
- ✅ **Stable readings** when stationary
- ✅ **Sample rate ~200 Hz**
- ❌ **No data** = disconnected Arduino
- ❌ **Erratic values** = poor calibration

### Synchronization
- ✅ **Both signals updating** in real-time
- ✅ **Time axis aligned** across all plots
- ✅ **Consistent timing** (check status panel)

## Visualization Features

### Display Panels

**Top Panel - EMG Differential Pairs:**
- Shows configured muscle pairs
- Auto-scaling amplitude
- Real-time signal updates

**Middle Panel - IMU1 Orientation:**
- Roll, Pitch, Yaw angles
- From first IMU sensor
- Degrees (-180 to +180)

**Bottom Panel - IMU2 Orientation:**
- Roll, Pitch, Yaw angles
- From second IMU sensor
- Degrees (-180 to +180)

### Configuration

Edit `src/synchronized_visualization.py` to configure:

```python
# EMG differential pairs (electrode connections)
EMG_DIFFERENTIAL_PAIRS = [
    (0, 1),  # Muscle 1: Channel 0 (+) - Channel 1 (-)
    (2, 3),  # Muscle 2: Channel 2 (+) - Channel 3 (-)
]

# Plot settings
PLOT_WINDOW_SECONDS = 5.0  # Time window displayed
PLOT_UPDATE_INTERVAL = 33  # Update rate (ms)
```

## Troubleshooting

### Visualization Won't Launch
- Check Python environment is activated
- Verify matplotlib is installed: `pip install matplotlib`
- Try running standalone: `python -m src.synchronized_visualization`

### No EMG Signal
1. Check TMSi device is connected (USB)
2. Verify electrodes are properly attached
3. Ensure skin preparation is adequate
4. Check differential pair configuration matches your setup

### No IMU Signal
1. Check Arduino is connected (USB)
2. Verify BMI160 sensors are powered
3. Check serial port detection
4. Try manual port: edit `IMU_PORT` in visualization config

### Poor Signal Quality
- **EMG:** Re-apply electrodes, improve skin contact
- **IMU:** Recalibrate (place flat and still during calibration)
- **Both:** Check cables and connections

## Best Practices

1. **Always test hardware before trials**
   - Prevents wasted participant time
   - Identifies issues early

2. **Verify signal quality**
   - EMG should show clear muscle activity
   - IMU should track movement accurately

3. **Check synchronization**
   - Both signals should update smoothly
   - No large timing gaps

4. **Document any issues**
   - Note in session metadata
   - Record signal quality problems

## Integration with Trial System

The hardware test is fully integrated:

- **Non-blocking:** Trial system continues running
- **State-aware:** Only available when idle or ready
- **Configuration-aware:** Uses dummy/real signals based on setup
- **Separate process:** Won't interfere with trial data collection

Press `T` anytime during IDLE or READY states to launch the test!
