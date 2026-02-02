# Quick Reference: Real-Time Synchronization

## Visual Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    PYTHON MAIN PROGRAM                          │
│                   (time.time() = common clock)                  │
└────────────┬────────────────────────────────┬───────────────────┘
             │                                │
             ▼                                ▼
    ┌────────────────┐              ┌────────────────┐
    │  IMU THREAD    │              │  EMG THREAD    │
    │  (Arduino)     │              │  (TMSi USB)    │
    └────────────────┘              └────────────────┘
             │                                │
             │                                │
    Hardware timestamps                Interpolated timestamps
    (Arduino micros())                 (sample rate + smoothing)
             │                                │
             ▼                                ▼
    ┌────────────────┐              ┌────────────────┐
    │ Synchronized   │              │ Synchronized   │
    │ IMU Buffer     │              │ EMG Buffer     │
    │ ~200 Hz        │              │ ~2000 Hz       │
    └────────────────┘              └────────────────┘
             │                                │
             └────────────┬───────────────────┘
                          ▼
                 ┌─────────────────┐
                 │  VISUALIZATION  │
                 │  (matplotlib)   │
                 └─────────────────┘
```

## Timeline Example

```
Time (seconds): 0.000    0.050    0.100    0.150    0.200
                 │        │        │        │        │
IMU Samples:     ●────────●────────●────────●────────●
                 ^        ^        ^        ^        ^
                 Hardware timestamps (from Arduino micros())

EMG Chunks:      [────────]        [────────]        
                 ^        ^        ^        ^
                 Interpolated timestamps (from sample rate)

Sync Point:      ●  <- Both devices timestamp relative to this
                 t=0 established at acquisition start
```

## Key Features

### IMU (Hardware Timestamps)
```python
# Precision: ~1 microsecond
# Method: Arduino micros() counter
# Jitter: None (hardware-based)

t_hardware = 123456  # microseconds from Arduino
t_python = 1705932123.456  # seconds from time.time()
offset = t_python - (t_hardware / 1e6)

# All future samples:
synchronized_time = (t_hardware / 1e6) + offset
```

### EMG (Interpolated Timestamps)
```python
# Precision: ~0.5 milliseconds
# Method: Sample rate interpolation
# Jitter: Reduced by smoothing filter

receive_time = time.time()
sample_period = 1.0 / 2000  # 0.0005s at 2000 Hz
chunk_duration = 100 * sample_period  # 0.05s for 100 samples

# Backdate to middle of chunk:
chunk_timestamp = receive_time - chunk_duration + (chunk_duration / 2)

# Smooth with prediction:
synchronized_time = 0.7 * predicted + 0.3 * measured
```

## Accuracy Summary

| Aspect | Accuracy | Notes |
|--------|----------|-------|
| IMU precision | 1 µs | Hardware counter |
| EMG precision | 0.5 ms | Interpolation |
| Device sync | < 100 ms | Start time alignment |
| Relative timing | < 1 ms | Between IMU and EMG |
| Long-term drift | ~10 ms/hour | Uncorrected |

## Common Pitfalls (Avoided)

❌ **Timestamping too late**
```python
data = get_data()
process(data)  # ← Processing delay
t = time.time()  # ← Too late!
```

✅ **Timestamp immediately**
```python
data = get_data()
t = time.time()  # ← Immediate!
process(data)
```

❌ **Ignoring hardware timestamps**
```python
# IMU provides t_us, but we ignore it
timestamp = time.time()  # ← Includes serial jitter!
```

✅ **Use hardware timestamps**
```python
# Use IMU's hardware counter
synchronized_time = (reading.t_us / 1e6) + offset
```

❌ **No jitter reduction**
```python
# Raw USB timing has jitter
timestamp = time.time()  # ← Varies by ±5ms
```

✅ **Smooth timestamps**
```python
# Blend predicted and measured
timestamp = 0.7 * predicted + 0.3 * measured
```

## Verification Tools

### Check Synchronization Quality

Run the script and look for:

```
[IMU] Time synchronization established:
      Hardware start: 123456 µs
      Python start: 1705932123.456789 s
      Time offset: 1705932123.333333 s

[EMG] Found COUNTER channel at index 37
[EMG] Acquisition start time: 1705932123.501234 s
```

Device start difference: **45.2 ms** ← Should be < 100ms

### During Acquisition

```
SYNCHRONIZATION:
  Device start time difference: 45.2 ms
  ✓ Devices well synchronized (< 100ms difference)
  IMU time range: 0.123 - 60.456 s
  EMG time range: 0.167 - 60.489 s
  Synchronized data overlap: 60.289 seconds  ← Good overlap
```

## Troubleshooting

### Problem: Large device offset (> 1 second)
**Solution**: Devices started too far apart. Restart script.

### Problem: Counter discontinuities
```
[EMG] ⚠️ Counter discontinuity: gap of 150 samples
```
**Solution**: USB buffer overrun. Check system load, close other apps.

### Problem: High latency (> 50ms)
```
[IMU] Latency: 78.3ms
```
**Solution**: Serial port buffer full. Increase baud rate or reduce sample rate.

## Quick Test

To verify synchronization works:

1. **Start acquisition** - note device start times (should be < 100ms apart)
2. **Move IMU sharply** - note the time
3. **Check EMG response** - should align with motion in visualization
4. **Stop acquisition** - check synchronization statistics

Good sync = smooth, aligned plots with consistent timing.

## PRBS Synchronization (NEW!)

For **extended recordings** (>10 minutes), enable PRBS-based drift correction:

```python
# In signal_acquisition_testing.py
ENABLE_PRBS_SYNC = True  # Continuous drift correction
```

### How PRBS Works

1. **Inject markers** at 10 Hz into both streams
2. **Cross-correlate** every 5 seconds to detect drift
3. **Auto-correct** timestamps in real-time
4. **Monitor** drift in visualization title

### PRBS Timing Accuracy

| Duration | Without PRBS | With PRBS |
|----------|-------------|-----------|
| 10 minutes | 1-5 ms | <2 ms |
| 2 hours | 50-150 ms | <5 ms |

### When to Use PRBS

✅ Use for recordings > 10 minutes  
✅ Use for critical timing accuracy  
✅ Use when analyzing temporal relationships  
❌ Not needed for short tests (<5 minutes)  
❌ Adds ~1% CPU overhead

### Monitoring PRBS Sync

Watch the visualization title:
```
PRBS Sync: ✓ Δt=+2.3ms (conf=0.89)  ← Good sync
PRBS Sync: ⚠ Δt=+12.1ms (conf=0.85)  ← Warning
PRBS Sync: ✗ Δt=+51.2ms (conf=0.45)  ← Error
```

### Validation

Run validation tests:
```bash
python setup_scripts/prbs_sync_validation.py
```

## For More Details

- `PRBS_SYNCHRONIZATION.md` - Complete PRBS theory and usage
- `SYNCHRONIZATION_EXPLAINED.md` - Baseline sync explanation
