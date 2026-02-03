# PRBS Timeline Normalization Fix

**Date**: 2026-02-03  
**Status**: Implemented  
**File**: `setup_scripts/prbs_sync_validation.py`

## Problem Summary

The real hardware synchronization test (Test 5) was failing with `[NO SYNC STATE]` even though:
- COUNTER channel was successfully detected and used
- Device timestamps were correctly extracted (EMG from COUNTER, IMU from BNO085)
- Plenty of data was collected (thousands of samples)
- Simulated tests were producing results (albeit with systematic errors)

## Root Cause: PRBS Phase Mismatch

The issue was that EMG and IMU device clocks started at **vastly different absolute times**:

```
EMG device time:  0.029 seconds  (TMSi COUNTER starts near zero)
IMU device time:  4658.562 seconds  (Arduino micros() since last reset)
Time difference:  4658 seconds = 78 minutes
```

### Why This Broke Synchronization

The PRBS generator was being queried with these absolute device times:

```python
# EMG
marker_emg = prbs.get_marker_at_time(0.029)  # Samples PRBS at t=0.029s

# IMU  
marker_imu = prbs.get_marker_at_time(4658.562)  # Samples PRBS at t=4658.562s
```

With a PRBS period of `255 bits / 10 Hz = 25.5 seconds`, the devices were sampling:
- **182 complete PRBS cycles apart** (4658s / 25.5s = 182.7)
- Completely **uncorrelated phases** of the sequence
- Cross-correlation returned **no valid peak** → `[NO SYNC STATE]`

## Solution: Normalize PRBS Timeline

Normalize both device timestamps to a **common PRBS reference timeline** before generating markers:

```python
# Track earliest timestamp from either device
prbs_ref_time = None  # Will be set to first EMG or IMU timestamp

# For EMG
if prbs_ref_time is None:
    prbs_ref_time = emg_device_t  # e.g., 0.029s
    
emg_prbs_t = emg_device_t - prbs_ref_time  # Normalized to 0.000s
marker_emg = prbs.get_marker_at_time(emg_prbs_t)

# For IMU  
imu_prbs_t = imu_device_t - prbs_ref_time  # e.g., 4658.562 - 0.029 = 4658.533s
marker_imu = prbs.get_marker_at_time(imu_prbs_t)
```

### Key Insight

The **PRBS offset** between devices is preserved in the normalized timeline:
- If EMG and IMU are 150ms apart in real time
- The normalized PRBS times will also be 150ms apart
- Cross-correlation will correctly find this 150ms offset

### What Doesn't Break

**Correlation timestamps** still use host-relative common time:
```python
# Establish offset mapping (keeps absolute time relationship)
emg_to_common_offset = host_time - emg_device_t  
imu_to_common_offset = host_time - imu_device_t

# Use for correlation (unchanged)
emg_common_t = emg_device_t + emg_to_common_offset
imu_common_t = imu_device_t + imu_to_common_offset

# But markers use normalized PRBS timeline
sync_engine.add_emg_data(emg_common_t, marker_emg)
sync_engine.add_imu_data(imu_common_t, marker_imu)
```

## Implementation Details

### Code Changes

**1. Added PRBS reference time tracking** (line ~650):
```python
# PRBS timeline normalization
prbs_ref_time = None  # Will be set to earliest device timestamp seen
```

**2. Updated EMG marker generation** (line ~700):
```python
# Establish PRBS reference time (earliest timestamp from either device)
if prbs_ref_time is None:
    prbs_ref_time = emg_device_t
    print(f"  [PRBS] Reference time set to EMG start: {prbs_ref_time:.3f}s")

# Normalize device time to PRBS reference timeline
emg_prbs_t = emg_device_t - prbs_ref_time
marker = prbs.get_marker_at_time(emg_prbs_t)
```

**3. Updated IMU marker generation** (line ~730):
```python
# Establish PRBS reference time (earliest timestamp from either device)
if prbs_ref_time is None:
    prbs_ref_time = imu_device_t
    print(f"  [PRBS] Reference time set to IMU start: {prbs_ref_time:.3f}s")

# Normalize device time to PRBS reference timeline
imu_prbs_t = imu_device_t - prbs_ref_time
marker = prbs.get_marker_at_time(imu_prbs_t)
```

**4. Updated function docstring** to reflect normalization approach

## Expected Results After Fix

With PRBS timeline normalization:
- ✅ EMG and IMU markers should correlate successfully
- ✅ Synchronization engine should produce valid offset estimates
- ✅ Real hardware test should report actual device clock offset (not `[NO SYNC STATE]`)

However, the **33ms systematic error** from simulated tests suggests:
- ⚠ The offset magnitude may still have ~30-50ms bias
- ⚠ This is a separate issue in the correlation/drift tracking logic

## Testing

Run the validation script with real hardware:
```powershell
python setup_scripts/prbs_sync_validation.py
```

Expected output for Test 5:
```
[PRBS] Reference time set to EMG start: 0.029s
[IMU] Timestamp baseline: device_t=4658.562s
...
t=10.0s  offset=XXX.XX ms  confidence=0.XXX  (EMG chunks=XXXX, IMU pkts=XXXX)
```

## Related Issues

- **Simulated Test Accuracy**: 33ms systematic error (separate from phase mismatch)
- **Drift Tracking**: 50ms saturation threshold too restrictive
- **Desync Detection**: 42ms error in detecting 150ms jump

## References

- Original issue discovered: 2026-02-03
- Related document: `README/PRBS_VALIDATION_STATUS.md`
- Related document: `README/PRBS_SYNC_FIXES.md`
- Implementation: `setup_scripts/prbs_sync_validation.py`, Test 5
