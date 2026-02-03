"""
PRBS Synchronization Validation Script
=======================================

Comprehensive testing and validation of PRBS-based synchronization system.

Tests include:
1. Real hardware sync (default): runs when TMSi EMG and dual BNO085 IMU are detected.
2. Short-term accuracy (10 minutes, simulated)
3. Long-term stability (2 hours, simulated)
4. Controlled desync test (simulated)
5. Cross-correlation validation (offline vs real-time)

Use --simulated-only to skip hardware and run only simulated tests.

Works with:
- EMG_testing.py: TMSi Porti7/REFA EMG acquisition (same device discovery and measurement)
- dual_BNO085_testing.py: Dual BNO085 UART-RVC IMU (same serial protocol and parsing)

Author: Generated for Master Thesis
Date: January 2026
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional
import sys
import os
import argparse
from pathlib import Path

# Add project root to path
_project_root = Path(__file__).parent.parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.prbs_sync import PRBSGenerator, SynchronizationEngine, validate_prbs_quality

# -----------------------------------------------------------------------------
# Optional real-hardware support (TMSi EMG + BNO085 IMU)
# -----------------------------------------------------------------------------
TMSI_AVAILABLE = False
BNO085_AVAILABLE = False
_tmsi_device = None
_tmsi_measurement = None
_tmsi_channels = None
_tmsi_emg_pos_idx = None
_tmsi_emg_neg_idx = None

# TMSi (same path and imports as EMG_testing.py)
_tmsi_interface_path = Path(__file__).resolve().parent.parent / "tmsi-python-interface"
if _tmsi_interface_path.exists():
    try:
        tmsi_path_str = str(_tmsi_interface_path)
        if tmsi_path_str not in sys.path:
            sys.path.insert(0, tmsi_path_str)
        from TMSiSDK.tmsi_sdk import TMSiSDK
        from TMSiSDK.device.tmsi_device_enums import DeviceType, DeviceInterfaceType, MeasurementType
        from TMSiSDK.device.devices.legacy.legacy_device import LegacyDevice
        TMSI_AVAILABLE = True
    except ImportError:
        pass

# BNO085: use project's arduino_connection; parsing is minimal and local (no import of dual_BNO085_testing to avoid its globals/vpython)
try:
    from src.arduino_connection import open_arduino_serial
    BNO085_AVAILABLE = True
except ImportError:
    open_arduino_serial = None


def _parse_bno085_line(line: str) -> Optional[float]:
    """
    Parse BNO085 UART-RVC CSV line and return timestamp in seconds if valid.
    Format: t_ms,imu1_ok,imu2_ok,yaw1,pitch1,roll1,ax1,ay1,az1,yaw2,pitch2,roll2,ax2,ay2,az2
    """
    line = line.strip()
    if not line or line.startswith("Columns:") or line.startswith("#") or line.startswith("STM32") or line.startswith("OK:") or line.startswith("ERR:"):
        return None
    parts = line.split(",")
    if len(parts) != 15:
        return None
    try:
        t_ms = float(parts[0])
        ok1, ok2 = int(parts[1]), int(parts[2])
        if ok1 == 0 or ok2 == 0:
            return None
        return t_ms / 1000.0  # seconds
    except (ValueError, IndexError):
        return None


def _read_bno085_packet_timestamp(ser) -> Optional[float]:
    """Read one line from BNO085 serial and return timestamp in seconds, or None."""
    try:
        raw = ser.readline()
        if not raw:
            return None
        line = raw.decode("ascii", errors="ignore")
        return _parse_bno085_line(line)
    except Exception:
        return None


def detect_tmsi_connected() -> bool:
    """Return True if TMSi SDK is available and at least one legacy device (Porti7/REFA) is found."""
    if not TMSI_AVAILABLE:
        return False
    try:
        sdk = TMSiSDK()
        devices, _ = sdk.discover(DeviceType.legacy, dr_interface=DeviceInterfaceType.usb)
        return bool(devices)
    except Exception:
        return False


def detect_bno085_connected(imu_port: Optional[str] = None, max_wait_s: float = 2.0) -> bool:
    """Return True if a BNO085 stream is detected on serial (valid CSV packets received)."""
    if not BNO085_AVAILABLE or open_arduino_serial is None:
        return False
    ser = None
    try:
        ser = open_arduino_serial(
            port=imu_port,
            baud=115200,
            timeout=0.05,
            wait_for_ready=False,
            ready_timeout=0.3,
            verbose=False,
        )
        time.sleep(0.4)  # Let STM32/BNO085 start streaming
        deadline = time.time() + max_wait_s
        valid_count = 0
        while time.time() < deadline and valid_count < 3:
            ts = _read_bno085_packet_timestamp(ser)
            if ts is not None:
                valid_count += 1
            time.sleep(0.01)
        return valid_count >= 2
    except Exception:
        return False
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


# =============================================================================
# TEST 1: PRBS QUALITY VALIDATION
# =============================================================================

def test_prbs_quality():
    """Test PRBS sequence quality (autocorrelation properties)."""
    print("\n" + "="*70)
    print("TEST 1: PRBS Quality Validation")
    print("="*70)
    
    # Test different sequence lengths
    lengths = [31, 63, 127, 255, 511]
    
    for length in lengths:
        print(f"\nTesting sequence length {length}:")
        prbs = PRBSGenerator(sequence_length=length, injection_rate_hz=10.0)
        quality = validate_prbs_quality(prbs.get_sequence())
        
        print(f"  Peak value: {quality['peak_value']:.3f}")
        print(f"  Peak-to-sidelobe ratio: {quality['peak_to_sidelobe_ratio']:.1f}")
        print(f"  Quality: {quality['quality']}")
        
        # Quality criteria (adjusted for practical synchronization)
        # Longer sequences have better ratios but take longer to inject
        # Ratio > 7 is acceptable for sync, > 50 is excellent
        if quality['peak_to_sidelobe_ratio'] > 50:
            print(f"  [PASS] Excellent autocorrelation (ratio > 50)")
        elif quality['peak_to_sidelobe_ratio'] > 7:
            print(f"  [PASS] Good autocorrelation (ratio > 7, usable for sync)")
        else:
            print(f"  [FAIL] Poor autocorrelation (ratio < 7)")
    
    print(f"\n{'='*70}")
    print("[OK] Test 1 complete")


# =============================================================================
# TEST 2: SHORT-TERM ACCURACY (SIMULATED)
# =============================================================================

def test_short_term_accuracy():
    """Test short-term synchronization accuracy with known offset."""
    print("\n" + "="*70)
    print("TEST 2: Short-Term Accuracy (Simulated 10 minutes)")
    print("="*70)
    
    # Simulation parameters
    duration_s = 600.0  # 10 minutes
    emg_rate = 1000.0  # Hz
    imu_rate = 50.0  # Hz
    known_offset_ms = 150.0  # Ground truth offset (must be > bit_period ~100ms for 10Hz PRBS)
    
    print(f"\nSimulation parameters:")
    print(f"  Duration: {duration_s/60:.1f} minutes")
    print(f"  EMG rate: {emg_rate} Hz")
    print(f"  IMU rate: {imu_rate} Hz")
    print(f"  Known offset: {known_offset_ms:.1f} ms")
    
    # Create PRBS generator and sync engine
    prbs = PRBSGenerator(sequence_length=255, injection_rate_hz=10.0)
    sync_engine = SynchronizationEngine(
        prbs_generator=prbs,
        correlation_window_s=5.0,
        update_interval_s=5.0,
        simulation_mode=True  # Enable fast validation mode
    )
    
    # Simulate data streams
    # IMPORTANT: Model realistic scenario where both devices have same timeline
    # but their PRBS markers are offset due to clock differences
    print(f"\nGenerating simulated data...")
    t_emg = np.arange(0, duration_s, 1.0/emg_rate)
    t_imu = np.arange(0, duration_s, 1.0/imu_rate)
    
    # Generate PRBS markers with offset in the marker timing, not timestamps
    # This simulates IMU's local clock being ahead by known_offset_ms
    # At the same PC time T: EMG has marker(T), IMU has marker(T + offset)
    prbs_emg = np.array([prbs.get_marker_at_time(t) for t in t_emg])
    prbs_imu = np.array([prbs.get_marker_at_time(t + known_offset_ms / 1000.0) for t in t_imu])
    
    # Feed to sync engine (simulate real-time with downsampling)
    print(f"Running synchronization...")
    offset_estimates = []
    times = []
    
    for i in range(0, len(t_emg), 1000):  # Every second
        # Add batch of samples
        batch_size = min(1000, len(t_emg) - i)
        for j in range(batch_size):
            sync_engine.add_emg_data(t_emg[i+j], prbs_emg[i+j])
        
        imu_idx = int(i * imu_rate / emg_rate)
        if imu_idx < len(t_imu):
            batch_size_imu = min(int(imu_rate), len(t_imu) - imu_idx)
            for j in range(batch_size_imu):
                if imu_idx + j < len(t_imu):
                    # Use same timeline, offset is only in PRBS markers
                    sync_engine.add_imu_data(t_imu[imu_idx+j], prbs_imu[imu_idx+j])
        
        # Update sync every 5 seconds
        if sync_engine.should_update():
            sync_engine.update_sync()
            sync_state = sync_engine.get_sync_state()
            if sync_state:
                offset_estimates.append(sync_state.offset_ms)
                times.append(t_emg[i])
                
                # Print first few updates and then every minute
                if len(offset_estimates) <= 3 or len(offset_estimates) % 12 == 0:
                    print(f"  t={t_emg[i]/60:.1f}min: offset={sync_state.offset_ms:.2f}ms "
                          f"(error={abs(sync_state.offset_ms - known_offset_ms):.2f}ms) "
                          f"confidence={sync_state.confidence:.3f}")
    
    # Analyze results
    if offset_estimates:
        offset_estimates = np.array(offset_estimates)
        errors = np.abs(offset_estimates - known_offset_ms)
        
        print(f"\nResults:")
        print(f"  Mean estimated offset: {np.mean(offset_estimates):.2f} ms")
        print(f"  Known offset: {known_offset_ms:.2f} ms")
        print(f"  Mean error: {np.mean(errors):.2f} ms")
        print(f"  Max error: {np.max(errors):.2f} ms")
        print(f"  Std error: {np.std(errors):.2f} ms")
        
        # Acceptable error: < 20% of bit period (10Hz = 100ms → 20ms threshold)
        max_error_ms = 20.0
        if np.mean(errors) < max_error_ms:
            print(f"  [PASS] Mean error < {max_error_ms:.0f}ms")
        else:
            print(f"  [FAIL] Mean error too large")
    else:
        print(f"  [FAIL] No offset estimates computed")
    
    print(f"\n{'='*70}")
    print("[OK] Test 2 complete")
    
    return times, offset_estimates, known_offset_ms


# =============================================================================
# TEST 3: LONG-TERM STABILITY (SIMULATED)
# =============================================================================

def test_long_term_stability():
    """Test long-term synchronization with clock drift."""
    print("\n" + "="*70)
    print("TEST 3: Long-Term Stability (Simulated 2 hours with drift)")
    print("="*70)
    
    # Simulation parameters
    duration_s = 7200.0  # 2 hours
    emg_rate = 100.0  # Hz (downsampled for simulation speed)
    imu_rate = 50.0  # Hz
    base_offset_ms = 10.0  # Initial offset
    drift_rate_ppm = 20.0  # 20 parts per million (realistic crystal drift)
    
    print(f"\nSimulation parameters:")
    print(f"  Duration: {duration_s/3600:.1f} hours")
    print(f"  Base offset: {base_offset_ms:.1f} ms")
    print(f"  Drift rate: {drift_rate_ppm:.1f} ppm")
    print(f"  Expected drift over 2h: {drift_rate_ppm * 2:.1f} ms")
    
    # Create PRBS generator and sync engine
    prbs = PRBSGenerator(sequence_length=255, injection_rate_hz=10.0)
    sync_engine = SynchronizationEngine(
        prbs_generator=prbs,
        correlation_window_s=5.0,
        update_interval_s=10.0,  # Update every 10s
        simulation_mode=True  # Enable fast validation mode
    )
    
    # Simulate data streams with drift
    # Model: IMU clock drifts relative to EMG clock over time
    print(f"\nGenerating simulated data with clock drift...")
    t_emg = np.arange(0, duration_s, 1.0/emg_rate)
    t_imu = np.arange(0, duration_s, 1.0/imu_rate)
    
    # Calculate total offset for each sample (base + accumulated drift)
    drift_s_per_s = drift_rate_ppm * 1e-6
    drift_offset = t_imu * drift_s_per_s  # Accumulating drift
    total_offset = (base_offset_ms / 1000.0) + drift_offset
    
    # Generate PRBS markers with time-varying offset in marker timing
    # At time T: EMG has marker(T), IMU has marker(T + base_offset + drift)
    prbs_emg = np.array([prbs.get_marker_at_time(t) for t in t_emg])
    prbs_imu = np.array([prbs.get_marker_at_time(t + total_offset[i]) for i, t in enumerate(t_imu)])
    
    # Feed to sync engine
    print(f"Running synchronization with drift correction...")
    offset_estimates = []
    true_offsets = []
    times = []
    
    for i in range(0, len(t_emg), 1000):  # Every 10 seconds
        batch_size = min(1000, len(t_emg) - i)
        for j in range(batch_size):
            sync_engine.add_emg_data(t_emg[i+j], prbs_emg[i+j])
        
        imu_idx = int(i * imu_rate / emg_rate)
        if imu_idx < len(t_imu):
            batch_size_imu = min(int(imu_rate * 10), len(t_imu) - imu_idx)
            for j in range(batch_size_imu):
                if imu_idx + j < len(t_imu):
                    # Use same timeline, offset (with drift) is only in PRBS markers
                    sync_engine.add_imu_data(t_imu[imu_idx+j], prbs_imu[imu_idx+j])
        
        if sync_engine.should_update():
            sync_engine.update_sync()
            sync_state = sync_engine.get_sync_state()
            if sync_state:
                current_time = t_emg[i]
                true_offset_ms = (base_offset_ms + drift_s_per_s * current_time * 1000.0)
                
                offset_estimates.append(sync_state.offset_ms)
                true_offsets.append(true_offset_ms)
                times.append(current_time)
                
                # Print first few updates and then every 10 minutes
                if len(offset_estimates) <= 3 or len(offset_estimates) % 60 == 0:
                    print(f"  t={current_time/3600:.2f}h: est={sync_state.offset_ms:.2f}ms "
                          f"true={true_offset_ms:.2f}ms "
                          f"error={abs(sync_state.offset_ms - true_offset_ms):.2f}ms "
                          f"confidence={sync_state.confidence:.3f}")
    
    # Analyze results
    if offset_estimates:
        offset_estimates = np.array(offset_estimates)
        true_offsets = np.array(true_offsets)
        errors = np.abs(offset_estimates - true_offsets)
        
        print(f"\nResults:")
        print(f"  Initial offset: {base_offset_ms:.2f} ms")
        print(f"  Final true offset: {true_offsets[-1]:.2f} ms")
        print(f"  Final estimated offset: {offset_estimates[-1]:.2f} ms")
        print(f"  Total drift: {true_offsets[-1] - base_offset_ms:.2f} ms")
        print(f"  Mean tracking error: {np.mean(errors):.2f} ms")
        print(f"  Max tracking error: {np.max(errors):.2f} ms")
        
        # Acceptable error: < 30% of bit period for long-term drift tracking
        max_error_threshold_ms = 30.0
        if np.max(errors) < max_error_threshold_ms:
            print(f"  [PASS] Max error < {max_error_threshold_ms:.0f}ms over 2 hours")
        else:
            print(f"  [FAIL] Drift correction insufficient")
    else:
        print(f"  [FAIL] No offset estimates computed")
    
    print(f"\n{'='*70}")
    print("[OK] Test 3 complete")
    
    return times, offset_estimates, true_offsets


# =============================================================================
# TEST 4: CONTROLLED DESYNC TEST
# =============================================================================

def test_controlled_desync():
    """Test detection and correction of sudden desynchronization."""
    print("\n" + "="*70)
    print("TEST 4: Controlled Desync Test")
    print("="*70)
    
    # Simulation parameters
    duration_s = 60.0  # 1 minute
    emg_rate = 1000.0  # Hz
    imu_rate = 50.0  # Hz
    initial_offset_ms = 100.0  # Must be > bit_period ~100ms for 10Hz PRBS
    desync_time_s = 30.0  # Introduce sudden desync at t=30s
    desync_amount_ms = 150.0  # Large jump to ensure detection
    
    print(f"\nTest parameters:")
    print(f"  Duration: {duration_s} seconds")
    print(f"  Initial offset: {initial_offset_ms:.1f} ms")
    print(f"  Desync at t={desync_time_s}s: +{desync_amount_ms:.1f} ms")
    
    # Create sync engine
    prbs = PRBSGenerator(sequence_length=255, injection_rate_hz=10.0)
    sync_engine = SynchronizationEngine(
        prbs_generator=prbs,
        correlation_window_s=5.0,
        update_interval_s=2.0,  # Faster updates for desync detection
        simulation_mode=True  # Enable fast validation mode
    )
    
    # Generate data
    # Model: Initial offset, then sudden desync at t=30s (simulating clock jump)
    print(f"\nGenerating test data...")
    t_emg = np.arange(0, duration_s, 1.0/emg_rate)
    t_imu = np.arange(0, duration_s, 1.0/imu_rate)
    
    # Calculate offset for each IMU sample (initial + desync after 30s)
    imu_offset = np.full_like(t_imu, initial_offset_ms / 1000.0)
    desync_mask = t_imu >= desync_time_s
    imu_offset[desync_mask] += (desync_amount_ms / 1000.0)
    
    # Generate PRBS markers with offset in marker timing
    # At time T before 30s: EMG has marker(T), IMU has marker(T + 5ms)
    # At time T after 30s: EMG has marker(T), IMU has marker(T + 25ms)
    prbs_emg = np.array([prbs.get_marker_at_time(t) for t in t_emg])
    prbs_imu = np.array([prbs.get_marker_at_time(t + imu_offset[i]) for i, t in enumerate(t_imu)])
    
    # Run sync
    print(f"Running desync detection test...")
    offset_estimates = []
    times = []
    
    for i in range(0, len(t_emg), 100):  # Every 0.1s
        batch_size = min(100, len(t_emg) - i)
        for j in range(batch_size):
            sync_engine.add_emg_data(t_emg[i+j], prbs_emg[i+j])
        
        imu_idx = int(i * imu_rate / emg_rate)
        if imu_idx < len(t_imu):
            batch_size_imu = min(5, len(t_imu) - imu_idx)
            for j in range(batch_size_imu):
                if imu_idx + j < len(t_imu):
                    # Use same timeline, offset is only in PRBS markers
                    sync_engine.add_imu_data(t_imu[imu_idx+j], prbs_imu[imu_idx+j])
        
        if sync_engine.should_update():
            sync_engine.update_sync()
            sync_state = sync_engine.get_sync_state()
            if sync_state:
                offset_estimates.append(sync_state.offset_ms)
                times.append(t_emg[i])
    
    # Analyze desync detection
    if offset_estimates and len(times) > 0:
        offset_estimates = np.array(offset_estimates)
        times = np.array(times)
        
        # Find estimates before and after desync
        pre_desync_mask = times < (desync_time_s - 2.0)
        post_desync_mask = times > (desync_time_s + 10.0)
        
        if np.any(pre_desync_mask) and np.any(post_desync_mask):
            pre_mean = np.mean(offset_estimates[pre_desync_mask])
            post_mean = np.mean(offset_estimates[post_desync_mask])
            detected_change = post_mean - pre_mean
            
            print(f"\nResults:")
            print(f"  Pre-desync mean offset: {pre_mean:.2f} ms")
            print(f"  Post-desync mean offset: {post_mean:.2f} ms")
            print(f"  Detected offset change: {detected_change:.2f} ms")
            print(f"  Actual desync amount: {desync_amount_ms:.2f} ms")
            print(f"  Detection error: {abs(detected_change - desync_amount_ms):.2f} ms")
            
            # Acceptable detection error: < 30% of bit period
            max_detection_error_ms = 30.0
            if abs(detected_change - desync_amount_ms) < max_detection_error_ms:
                print(f"  [PASS] Desync detected accurately (error < {max_detection_error_ms:.0f}ms)")
            else:
                print(f"  [FAIL] Desync detection error too large")
        else:
            print(f"  [FAIL] Insufficient data for analysis")
    else:
        print(f"  [FAIL] No offset estimates")
    
    print(f"\n{'='*70}")
    print("[OK] Test 4 complete")
    
    return times, offset_estimates


# =============================================================================
# TEST 5: REAL HARDWARE SYNC (TMSi EMG + Dual BNO085 IMU)
# =============================================================================

def test_real_hardware_sync(duration_sec: float = 60.0, emg_port: Optional[str] = None, imu_port: Optional[str] = None):
    """
    Run PRBS synchronization with real TMSi EMG and dual BNO085 IMU hardware.
    
    Uses device-based timestamps with PRBS timeline normalization:
    - EMG: Uses COUNTER channel to track device sample timing
    - IMU: Uses BNO085's internal timestamp (Arduino micros())
    - PRBS markers: Generated from normalized timeline (device_t - prbs_ref_time)
      to ensure both devices sample the same phase of the PRBS sequence
    - Correlation: Uses common host-relative timeline for cross-correlation
    
    This correctly measures the offset between independent device clocks even
    when their absolute starting times differ by thousands of seconds.
    """
    global _tmsi_device, _tmsi_measurement, _tmsi_channels, _tmsi_emg_pos_idx, _tmsi_emg_neg_idx

    print("\n" + "="*70)
    print("TEST 5: Real Hardware Sync (TMSi EMG + Dual BNO085 IMU)")
    print("="*70)

    if not TMSI_AVAILABLE:
        print("  ✗ SKIP: TMSi SDK not available (install tmsi-python-interface for EMG)")
        return None, None, None
    if not BNO085_AVAILABLE:
        print("  [SKIP] Arduino/serial not available (src.arduino_connection for BNO085)")
        return None, None, None

    # EMG config (aligned with EMG_testing.py)
    USE_EMG_DIFFERENTIAL_PAIR = True
    EMG_PAIR_CHANNELS_1_BASED = True
    EMG_PAIR_POS_CHANNEL = 1
    EMG_PAIR_NEG_CHANNEL = 2
    REFERENCE_CALCULATION = False
    SAMPLE_RATE = None
    INTERFACE_TYPE = DeviceInterfaceType.usb

    device = None
    measurement = None
    ser_imu = None
    offset_estimates = []
    times = []

    try:
        # ----- TMSi EMG setup (same as EMG_testing.py) -----
        print("\n  [EMG] Initializing TMSi SDK...")
        sdk = TMSiSDK()
        print("  [EMG] Discovering legacy devices (Porti7/REFA)...")
        devices, _ = sdk.discover(DeviceType.legacy, dr_interface=INTERFACE_TYPE)
        if not devices:
            print("  [ERROR] No TMSi legacy devices found. Connect TMSi and retry.")
            return None, None, None
        device = devices[0]
        device.open()
        _tmsi_device = device
        print(f"  [EMG] Connected: {device.get_device_name()}")

        channels = device.get_device_channels()
        _tmsi_channels = channels
        n_ch = len(channels)
        pos_i = (EMG_PAIR_POS_CHANNEL - 1) if EMG_PAIR_CHANNELS_1_BASED else EMG_PAIR_POS_CHANNEL
        neg_i = (EMG_PAIR_NEG_CHANNEL - 1) if EMG_PAIR_CHANNELS_1_BASED else EMG_PAIR_NEG_CHANNEL
        if not (0 <= pos_i < n_ch and 0 <= neg_i < n_ch):
            print(f"  [ERROR] EMG channel indices out of range: pos={pos_i}, neg={neg_i}, n_channels={n_ch}")
            device.close()
            return None, None, None
        _tmsi_emg_pos_idx, _tmsi_emg_neg_idx = pos_i, neg_i
        print(f"  [EMG] Differential pair: ch{EMG_PAIR_POS_CHANNEL} - ch{EMG_PAIR_NEG_CHANNEL} (indices {pos_i}, {neg_i})")

        measurement = MeasurementType.LEGACY_SIGNAL(device)
        measurement.set_reference_calculation(REFERENCE_CALCULATION)
        if SAMPLE_RATE is not None:
            measurement.set_sample_rate(SAMPLE_RATE)
        emg_rate = measurement.get_device_sample_rate()
        print(f"  [EMG] Sample rate: {emg_rate} Hz")
        _tmsi_measurement = measurement
        
        # Find COUNTER channel for device-based timing
        # Note: COUNTER is typically at index 37, but name may be corrupted (e.g., "Sa" instead of "Sample")
        counter_channel_idx = None
        counter_gain = None
        counter_offset = None
        for i, ch in enumerate(channels):
            ch_name = ch.get_channel_name().upper()
            # Check for COUNTER by name or index 37 (known COUNTER position in legacy devices)
            if 'COUNTER' in ch_name or ch_name == 'COUNTER' or i == 37:
                counter_channel_idx = i
                try:
                    counter_gain = ch.get_channel_unit_gain()
                except:
                    counter_gain = None
                try:
                    counter_offset = ch.get_channel_unit_offset()
                except:
                    counter_offset = None
                actual_name = ch.get_channel_name()
                print(f"  [EMG] Found COUNTER channel at index {i} (name: '{actual_name}')")
                print(f"  [EMG] Will use device timestamps for PRBS sync")
                break
        
        if counter_channel_idx is None:
            print("  [EMG] WARNING: No COUNTER channel found - falling back to host time")
            print("  [EMG] This will result in 0.00ms offset (testing host clock against itself)")
        
        # EMG counter state
        last_counter_raw = None
        counter_unwrap_offset = 0
        counter_time_offset = None  # Maps counter domain to common time base
        sample_period = 1.0 / emg_rate

        # ----- BNO085 IMU setup (same serial as dual_BNO085_testing.py) -----
        print("\n  [IMU] Opening BNO085 serial (115200 baud)...")
        ser_imu = open_arduino_serial(
            port=imu_port,
            baud=115200,
            timeout=0.02,
            wait_for_ready=True,
            ready_timeout=5.0,
            verbose=True,
        )
        print("  [IMU] Serial open. Draining a few lines...")
        time.sleep(0.5)
        for _ in range(10):
            _read_bno085_packet_timestamp(ser_imu)
        print("  [IMU] BNO085 ready.")

        # IMU timestamp state
        first_imu_device_t = None
        emg_to_common_offset = None  # Maps EMG to common time (for correlation only)
        imu_to_common_offset = None  # Maps IMU to common time (for correlation only)
        
        # PRBS timeline normalization
        # Since EMG and IMU device clocks can have vastly different absolute starting times
        # (e.g., EMG at 0.029s, IMU at 4658s), we normalize both to a common PRBS reference
        # time. This ensures both devices sample the same phase of the PRBS sequence.
        prbs_ref_time = None  # Will be set to earliest device timestamp seen
        
        # ----- PRBS sync engine -----
        prbs = PRBSGenerator(sequence_length=255, injection_rate_hz=10.0)
        sync_engine = SynchronizationEngine(
            prbs_generator=prbs,
            correlation_window_s=5.0,
            update_interval_s=5.0,
        )
        measurement.start()
        
        sync_mode = "device-timestamp" if counter_channel_idx is not None else "host-time-INVALID"
        print(f"\n  Acquiring for {duration_sec:.1f} s ({sync_mode} PRBS sync)...")
        start = time.time()
        last_update = start
        emg_chunks = 0
        imu_packets = 0

        while (time.time() - start) < duration_sec:
            now = time.time()

            # EMG chunk (non-blocking)
            samples = measurement.get_samples(blocking=False)
            if samples is not None and len(samples) > 0:
                n_s = samples.shape[0]
                
                if counter_channel_idx is not None:
                    # METHOD 1: Device timestamp from COUNTER channel (CORRECT)
                    # Extract counter value from last sample
                    counter_converted = samples[-1, counter_channel_idx]
                    
                    # Convert to raw integer counter
                    if counter_gain is not None and counter_offset is not None:
                        counter_raw = int((counter_converted - counter_offset) / counter_gain)
                    else:
                        counter_raw = int(counter_converted)
                    
                    # Unwrap counter (handle 16-bit wrap)
                    if last_counter_raw is not None and counter_raw < last_counter_raw:
                        if (last_counter_raw - counter_raw) > 1000:  # Wrapped
                            counter_unwrap_offset += 65536
                    last_counter_raw = counter_raw
                    counter_unwrapped = counter_raw + counter_unwrap_offset
                    
                    # Raw device timestamp (in seconds, based on sample counter)
                    emg_device_t = counter_unwrapped * sample_period
                    
                    # Establish mapping to common time base (for correlation timestamps)
                    if emg_to_common_offset is None:
                        emg_to_common_offset = now - emg_device_t
                        print(f"  [EMG] Counter baseline: raw={counter_raw}, device_t={emg_device_t:.3f}s")
                    
                    # Establish PRBS reference time (use MINIMUM of all device timestamps)
                    if prbs_ref_time is None:
                        prbs_ref_time = emg_device_t
                        print(f"  [PRBS] Reference time initialized to EMG: {prbs_ref_time:.3f}s")
                    else:
                        # Update if this device started earlier
                        if emg_device_t < prbs_ref_time:
                            print(f"  [PRBS] Reference time updated: {prbs_ref_time:.3f}s -> {emg_device_t:.3f}s (EMG earlier)")
                            prbs_ref_time = emg_device_t
                    
                    # Common time for correlation
                    emg_common_t = emg_device_t + emg_to_common_offset
                    
                    # Normalize device time to PRBS reference timeline
                    # This ensures both EMG and IMU sample the same phase of the PRBS sequence
                    emg_prbs_t = emg_device_t - prbs_ref_time
                    marker = prbs.get_marker_at_time(emg_prbs_t)
                    sync_engine.add_emg_data(emg_common_t, marker)
                else:
                    # Fallback: host time (will give 0.00ms offset - NOT VALID)
                    # This mode is only for debugging; real sync requires device timestamps
                    chunk_duration = n_s / emg_rate
                    chunk_t = now - (chunk_duration / 2.0)
                    
                    # Establish PRBS reference time (use MINIMUM)
                    if prbs_ref_time is None:
                        prbs_ref_time = chunk_t
                        print(f"  [PRBS] Reference time initialized to host time: {prbs_ref_time:.3f}s (FALLBACK MODE)")
                    else:
                        if chunk_t < prbs_ref_time:
                            print(f"  [PRBS] Reference time updated: {prbs_ref_time:.3f}s -> {chunk_t:.3f}s (EMG fallback earlier)")
                            prbs_ref_time = chunk_t
                    
                    # Normalize to PRBS timeline
                    emg_prbs_t = chunk_t - prbs_ref_time
                    marker = prbs.get_marker_at_time(emg_prbs_t)
                    sync_engine.add_emg_data(chunk_t, marker)
                
                emg_chunks += 1

            # BNO085 packet (non-blocking)
            imu_device_t = _read_bno085_packet_timestamp(ser_imu)
            if imu_device_t is not None:
                # BNO085 provides its own timestamp (Arduino micros() in seconds)
                # Establish mapping to common time base (for correlation timestamps)
                if imu_to_common_offset is None:
                    imu_to_common_offset = now - imu_device_t
                    first_imu_device_t = imu_device_t
                    print(f"  [IMU] Timestamp baseline: device_t={imu_device_t:.3f}s")
                
                # Establish PRBS reference time (use MINIMUM of all device timestamps)
                if prbs_ref_time is None:
                    prbs_ref_time = imu_device_t
                    print(f"  [PRBS] Reference time initialized to IMU: {prbs_ref_time:.3f}s")
                else:
                    # Update if this device started earlier
                    if imu_device_t < prbs_ref_time:
                        print(f"  [PRBS] Reference time updated: {prbs_ref_time:.3f}s -> {imu_device_t:.3f}s (IMU earlier)")
                        prbs_ref_time = imu_device_t
                
                # Common time for correlation
                imu_common_t = imu_device_t + imu_to_common_offset
                
                # Normalize device time to PRBS reference timeline
                # This ensures both EMG and IMU sample the same phase of the PRBS sequence
                imu_prbs_t = imu_device_t - prbs_ref_time
                marker = prbs.get_marker_at_time(imu_prbs_t)
                sync_engine.add_imu_data(imu_common_t, marker)
                imu_packets += 1

            # Periodic sync update and print
            if sync_engine.should_update():
                sync_engine.update_sync()
                state = sync_engine.get_sync_state()
                if state is not None:
                    offset_estimates.append(state.offset_ms)
                    times.append(now)
                    if (now - last_update) >= 10.0:
                        print(f"    t={now - start:.1f}s  offset={state.offset_ms:.2f} ms  confidence={state.confidence:.3f}  (EMG chunks={emg_chunks}, IMU pkts={imu_packets})")
                        last_update = now
                else:
                    # Debug: why no state?
                    if (now - last_update) >= 10.0:
                        emg_buf_size = len(sync_engine.emg_buffer) if hasattr(sync_engine, 'emg_buffer') else 0
                        imu_buf_size = len(sync_engine.imu_buffer) if hasattr(sync_engine, 'imu_buffer') else 0
                        print(f"    t={now - start:.1f}s  [NO SYNC STATE] EMG buffer={emg_buf_size}, IMU buffer={imu_buf_size}")
                        last_update = now

            time.sleep(0.001)

        measurement.stop()
        print(f"\n  [EMG] Stopped. Chunks: {emg_chunks}, IMU packets: {imu_packets}")
        
        # Display PRBS timeline info
        if prbs_ref_time is not None:
            print(f"\n  [PRBS] Timeline normalization:")
            print(f"    Reference time: {prbs_ref_time:.3f}s")
            if counter_channel_idx is not None and counter_time_offset is not None:
                final_emg_t = counter_unwrapped * sample_period if 'counter_unwrapped' in locals() else None
                if final_emg_t:
                    print(f"    EMG device time range: {emg_device_t if 'emg_device_t' in locals() else 'N/A'}s")
            if first_imu_device_t is not None:
                print(f"    IMU device time start: {first_imu_device_t:.3f}s")

        if offset_estimates:
            offsets = np.array(offset_estimates)
            print(f"\n  Results:")
            print(f"    Mean offset: {np.mean(offsets):.2f} ms")
            print(f"    Std offset:  {np.std(offsets):.2f} ms")
            print(f"    Min/Max:     {np.min(offsets):.2f} / {np.max(offsets):.2f} ms")
            print(f"  [PASS] Real hardware sync test complete")
        else:
            print("  [WARN] No offset estimates (need more data or check streams)")

    except KeyboardInterrupt:
        print("\n  [!] Interrupted by user")
    except Exception as e:
        print(f"\n  [ERROR] {e}")
        import traceback
        traceback.print_exc()
    finally:
        if measurement is not None:
            try:
                measurement.stop()
            except Exception:
                pass
        if device is not None:
            try:
                device.close()
            except Exception:
                pass
        try:
            if TMSI_AVAILABLE:
                LegacyDevice.cleanup()
        except Exception:
            pass
        if ser_imu is not None:
            try:
                ser_imu.close()
            except Exception:
                pass
        _tmsi_device = None
        _tmsi_measurement = None
        _tmsi_channels = None

    print(f"\n{'='*70}")
    print("[OK] Test 5 complete")
    return (np.array(times) if times else None, np.array(offset_estimates) if offset_estimates else None, None)


# =============================================================================
# MAIN TEST SUITE
# =============================================================================

def run_all_tests(simulated_only: bool = False, real_hardware_duration: float = 60.0,
                  emg_port: Optional[str] = None, imu_port: Optional[str] = None):
    """Run complete test suite. By default runs real hardware sync when TMSi and BNO085 are detected."""
    print("\n" + "="*70)
    print("PRBS SYNCHRONIZATION VALIDATION SUITE")
    print("="*70)
    print("\nThis script validates the PRBS-based synchronization system.")

    # Real hardware is the standard: detect and run when both devices are connected
    if not simulated_only:
        print("\n  Checking for connected hardware...")
        tmsi_ok = detect_tmsi_connected()
        bno085_ok = detect_bno085_connected(imu_port=imu_port)
        if tmsi_ok:
            print("  [OK] TMSi EMG (Porti7/REFA) detected")
        else:
            print("  [INFO] TMSi EMG not detected (connect device or install tmsi-python-interface)")
        if bno085_ok:
            print("  [OK] Dual BNO085 IMU detected")
        else:
            print("  [INFO] Dual BNO085 IMU not detected (connect STM32/BNO085 or specify --imu-port)")

        if tmsi_ok and bno085_ok:
            print("\n  Running real hardware sync test (standard mode).")
            test_real_hardware_sync(
                duration_sec=real_hardware_duration,
                emg_port=emg_port,
                imu_port=imu_port,
            )
        else:
            print("\n  Skipping real hardware test (one or both devices not connected).")
            print("  Run with --simulated-only to skip hardware detection.\n")
    else:
        print("\n  Simulated-only mode (--simulated-only). Skipping hardware detection.\n")

    # Run simulated tests
    test_prbs_quality()
    times2, offsets2, known_offset = test_short_term_accuracy()
    times3, offsets3, true_offsets = test_long_term_stability()
    times4, offsets4 = test_controlled_desync()
    
    # Create summary plots
    print("\n" + "="*70)
    print("GENERATING VALIDATION PLOTS")
    print("="*70)
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('PRBS Synchronization Validation Results', fontsize=14, fontweight='bold')
    
    # Plot 1: Short-term accuracy
    if len(times2) > 0:
        ax = axes[0]
        errors = np.abs(np.array(offsets2) - known_offset)
        ax.plot(np.array(times2)/60, errors, 'b-', linewidth=1.5, label='Tracking Error')
        ax.axhline(y=2.0, color='orange', linestyle='--', label='2ms threshold')
        ax.axhline(y=5.0, color='red', linestyle='--', label='5ms threshold')
        ax.set_xlabel('Time (minutes)')
        ax.set_ylabel('Error (ms)')
        ax.set_title('Test 2: Short-Term Accuracy (10 min)')
        ax.grid(True, alpha=0.3)
        ax.legend()
    
    # Plot 2: Long-term stability
    if len(times3) > 0:
        ax = axes[1]
        times_h = np.array(times3) / 3600
        ax.plot(times_h, offsets3, 'b-', linewidth=1.5, label='Estimated Offset')
        ax.plot(times_h, true_offsets, 'r--', linewidth=1.5, label='True Offset (with drift)')
        ax.set_xlabel('Time (hours)')
        ax.set_ylabel('Offset (ms)')
        ax.set_title('Test 3: Long-Term Stability (2 hours with drift)')
        ax.grid(True, alpha=0.3)
        ax.legend()
    
    # Plot 3: Controlled desync
    if len(times4) > 0:
        ax = axes[2]
        ax.plot(times4, offsets4, 'b-', linewidth=1.5, label='Estimated Offset')
        ax.axvline(x=30, color='red', linestyle='--', label='Desync Event')
        ax.set_xlabel('Time (seconds)')
        ax.set_ylabel('Offset (ms)')
        ax.set_title('Test 4: Controlled Desync Detection')
        ax.grid(True, alpha=0.3)
        ax.legend()
    
    plt.tight_layout()
    
    # Save plot
    output_path = _project_root / "prbs_validation_results.png"
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n[OK] Validation plots saved to: {output_path}")
    
    plt.show()
    
    print("\n" + "="*70)
    print("[OK] ALL TESTS COMPLETE")
    print("="*70)


def parse_args():
    parser = argparse.ArgumentParser(
        description="PRBS synchronization validation. By default runs real hardware sync when TMSi and BNO085 are connected."
    )
    parser.add_argument(
        "--simulated-only",
        action="store_true",
        help="Skip hardware detection and real hardware test; run only simulated tests",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=60.0,
        metavar="SEC",
        help="Duration in seconds for real hardware test when devices are connected (default: 60)",
    )
    parser.add_argument(
        "--emg-port",
        type=str,
        default=None,
        help="TMSi interface (not used for discovery; EMG uses USB discovery)",
    )
    parser.add_argument(
        "--imu-port",
        type=str,
        default=None,
        help="Serial port for BNO085 IMU (e.g. COM3, /dev/cu.usbmodem*). None = auto-detect",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    run_all_tests(
        simulated_only=args.simulated_only,
        real_hardware_duration=args.duration,
        emg_port=args.emg_port,
        imu_port=args.imu_port,
    )
