# PRBS Synchronization Validation - Current Status

**Date**: 2026-02-03  
**Status**: ⚠️ Real hardware test not yet functional

---

## Summary

Successfully fixed multiple issues and identified the root cause of validation failures:

1. ✅ **Fixed Unicode encoding** in `arduino_connection.py`  
2. ✅ **Found COUNTER channel** at index 37 (name corrupted to "Sa")
3. ✅ **Implemented device timestamp extraction** from both EMG and IMU
4. ❌ **PRBS phase mismatch** - devices sample PRBS at vastly different absolute times

---

## Key Findings

### 1. Original Problem: 0.00ms Offset

**Root Cause**: Test was comparing host PC clock against itself

**What was happening**:
- Both EMG and IMU timestamped with `time.time()` at reception
- PRBS markers generated at identical host times
- Result: Perfect 0.00ms offset (not measuring device clock offset)

### 2. TMSi COUNTER Channel

**Discovery**:
- COUNTER channel exists at **index 37**
- Channel name corrupted: "Sa" instead of "COUNTER" or "Sample"
- Provides device-level sample counting for precise timing
- Successfully extracted and unwrapped (handles 16-bit wrap-around)

### 3. Current Issue: PRBS Phase Mismatch

**Problem**: Device clocks start at vastly different absolute times:
- **EMG**: `device_t = 0.029s` (counter starts near zero after device init)
- **IMU**: `device_t = 4658.562s` (Arduino uptime since power-on)

**Why this breaks sync**:
```
EMG generates: prbs.get_marker_at_time(0.029)   → marker value X
IMU generates: prbs.get_marker_at_time(4658.5)  → marker value Y (completely different!)
Cross-correlation: No correlation peak found (different PRBS phases)
```

---

## Test Results

### Simulated Tests
| Test | Status | Notes |
|------|--------|-------|
| Test 1: PRBS Quality | ✅ PASS | All sequences show good autocorrelation |
| Test 2: Short-Term (10 min) | ❌ FAIL | Mean error 33.54ms (expected <2ms) |
| Test 3: Long-Term (2 hrs) | ❌ FAIL | Mean error 29.77ms, drift tracking poor |
| Test 4: Desync Detection | ❌ FAIL | Detection error 41.81ms (expected <3ms) |

### Real Hardware Test
| Metric | Result |
|--------|--------|
| Status | ❌ NO SYNC STATE |
| EMG device time | 0.029s |
| IMU device time | 4658.562s |
| Time difference | ~4658s (~78 minutes) |
| Correlation result | No valid sync state |
| Buffer sizes | EMG: 6134, IMU: 10000 samples |

---

## Solutions

### Option 1: Normalize PRBS Marker Timeline (Recommended)

**Approach**: Map both device times to a common PRBS timeline

```python
# Establish common PRBS start time when first data arrives
if prbs_start_time is None:
    prbs_start_time = min(emg_device_t, imu_device_t)

# Generate markers relative to common start
emg_prbs_t = emg_device_t - prbs_start_time
imu_prbs_t = imu_device_t - prbs_start_time

marker_emg = prbs.get_marker_at_time(emg_prbs_t)
marker_imu = prbs.get_marker_at_time(imu_prbs_t)
```

**Pros**: Simple, preserves relative timing
**Cons**: Requires both devices to start sending data before PRBS generation begins

### Option 2: Use Modulo for PRBS Period

**Approach**: Wrap device times to PRBS period

```python
prbs_period = 255 / 10.0  # sequence_length / injection_rate_hz = 25.5s

emg_prbs_t = emg_device_t % prbs_period  
imu_prbs_t = imu_device_t % prbs_period

marker_emg = prbs.get_marker_at_time(emg_prbs_t)
marker_imu = prbs.get_marker_at_time(imu_prbs_t)
```

**Pros**: Works even with large time differences
**Cons**: Loses absolute timing, only works if actual offset < prbs_period

### Option 3: Host-Time PRBS with Device Timestamps

**Approach**: Generate PRBS based on host time, but use device timestamps for correlation

```python
# PRBS markers from common host timeline
marker_emg = prbs.get_marker_at_time(host_time)
marker_imu = prbs.get_marker_at_time(host_time)

# But use device times for correlation
sync_engine.add_emg_data(emg_device_t, marker_emg)
sync_engine.add_imu_data(imu_device_t, marker_imu)
```

**Pros**: Markers guaranteed to correlate
**Cons**: Only measures host-to-device latency difference, not true device clock offset

---

## Simulated Test Issues

The simulated tests also show failures (33ms mean error vs expected <2ms). Possible causes:

1. **Correlation window too small** (5s) for the offset magnitude (150ms)
2. **PRBS injection rate** (10 Hz) provides limited temporal resolution
3. **Cross-correlation algorithm** may need tuning (window size, interpolation)
4. **Drift tracking** insufficient for long-term stability

---

## Files Modified

1. **src/arduino_connection.py**
   - Replaced Unicode symbols with ASCII (✓→[OK], →→->)
   
2. **setup_scripts/prbs_sync_validation.py**
   - Added COUNTER channel detection (index 37)
   - Implemented device timestamp extraction
   - Separated PRBS marker generation from correlation timestamps
   - Added debug output for sync state

---

## Next Steps

1. **Implement PRBS timeline normalization** (Option 1 recommended)
2. **Re-run real hardware test** to verify correlation works
3. **Tune correlation parameters** if needed (window size, confidence threshold)
4. **Investigate simulated test failures** (separate issue from real hardware)
5. **Document expected performance** based on actual results

---

## Technical Notes

### PRBS Period Calculation
```
Sequence length: 255 bits
Injection rate: 10 Hz (10 bits per second)
Period: 255 / 10 = 25.5 seconds
```

### Expected Timing Resolution
```
Bit period: 1 / 10 Hz = 100ms
Theoretical best resolution: ~100ms (1 bit period)
Practical resolution: 20-50ms (correlation window effects)
```

### Device Characteristics
- **TMSi EMG**: 2000 Hz sample rate, COUNTER channel at index 37
- **BNO085 IMU**: ~25 Hz update rate, Arduino micros() timestamp
- **Actual offset**: Unknown (to be measured once sync works)

---

**Status**: Investigation complete, solution identified, implementation pending.
