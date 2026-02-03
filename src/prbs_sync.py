"""
PRBS-Based Synchronization Module
==================================

Pseudorandom Binary Sequence (PRBS) generation and cross-correlation-based
synchronization for multi-device real-time data acquisition.

This module provides:
- m-sequence PRBS generation (maximal-length sequences)
- Time-indexed PRBS marker injection
- Cross-correlation-based drift detection
- Real-time timestamp correction with Kalman filtering

Theory:
-------
PRBS sequences have ideal autocorrelation properties:
- Sharp peak at zero lag (perfect self-alignment)
- Low sidelobes at non-zero lags (minimal false positives)
- Deterministic and reproducible

By injecting identical PRBS markers into independent data streams,
cross-correlation can detect time offsets with sub-millisecond precision,
enabling continuous drift correction for extended recordings.

Author: Generated for Master Thesis
Date: January 2026
"""

import time
import numpy as np
from typing import Tuple, Optional, List, Deque
from dataclasses import dataclass
from collections import deque
import threading


# =============================================================================
# PRBS GENERATOR
# =============================================================================

class PRBSGenerator:
    """
    Maximum-length Pseudorandom Binary Sequence (m-sequence) generator.
    
    Generates deterministic binary sequences with ideal autocorrelation
    properties for synchronization applications.
    
    Properties:
    - Period: 2^n - 1 (where n is the polynomial degree)
    - Amplitude: ±1 (bipolar representation)
    - Reproducible: Same seed produces identical sequence
    - Time-indexed: Get PRBS value at any timestamp
    
    Example:
        >>> prbs = PRBSGenerator(sequence_length=255, injection_rate_hz=10.0)
        >>> marker = prbs.get_marker_at_time(1.234)  # Get PRBS value at t=1.234s
        >>> marker
        1.0
    """
    
    def __init__(self, 
                 sequence_length: int = 255,
                 injection_rate_hz: float = 10.0,
                 polynomial: int = 0b111010001,  # x^8 + x^6 + x^5 + x^4 + 1
                 seed: Optional[int] = None):
        """
        Initialize PRBS generator.
        
        Args:
            sequence_length: PRBS period (must be 2^n - 1, e.g., 7, 15, 31, 63, 127, 255, 511)
            injection_rate_hz: Rate at which PRBS bits are injected (Hz)
            polynomial: Generator polynomial in binary format (for LFSR)
            seed: Random seed for sequence generation (None = deterministic default)
        """
        self.sequence_length = sequence_length
        self.injection_rate_hz = injection_rate_hz
        self.polynomial = polynomial
        self.seed = seed if seed is not None else 0xAB  # Default seed
        
        # Validate sequence length
        valid_lengths = [7, 15, 31, 63, 127, 255, 511, 1023, 2047, 4095]
        if sequence_length not in valid_lengths:
            raise ValueError(f"Sequence length must be one of {valid_lengths}, got {sequence_length}")
        
        # Generate the PRBS sequence
        self.sequence = self._generate_sequence()
        
        # Time base
        self.bit_period = 1.0 / injection_rate_hz  # seconds per bit
        self.start_time = time.time()
        
        # Thread safety
        self.lock = threading.Lock()
    
    def _generate_sequence(self) -> np.ndarray:
        """
        Generate one period of m-sequence using Linear Feedback Shift Register (LFSR).
        
        Returns:
            Binary sequence of length sequence_length with values ±1
        """
        # Determine number of bits needed
        n_bits = int(np.ceil(np.log2(self.sequence_length + 1)))
        
        # Initialize LFSR with seed
        lfsr = self.seed & ((1 << n_bits) - 1)  # Mask to n_bits
        if lfsr == 0:
            lfsr = 1  # Prevent all-zero state
        
        sequence = []
        
        # Generate sequence_length bits
        for _ in range(self.sequence_length):
            # Output bit is LSB
            bit = lfsr & 1
            sequence.append(1.0 if bit else -1.0)  # Convert to ±1
            
            # Compute feedback bit using polynomial
            feedback = 0
            temp = lfsr & self.polynomial
            while temp:
                feedback ^= (temp & 1)
                temp >>= 1
            
            # Shift and insert feedback
            lfsr = (lfsr >> 1) | (feedback << (n_bits - 1))
        
        return np.array(sequence, dtype=float)
    
    def get_marker_at_time(self, timestamp: float) -> float:
        """
        Get PRBS marker value at given timestamp.
        
        Args:
            timestamp: Time in seconds (from arbitrary reference)
        
        Returns:
            PRBS value (±1) at given time
        """
        with self.lock:
            # Calculate bit index based on injection rate
            # Time wraps around sequence period
            time_offset = timestamp * self.injection_rate_hz
            bit_index = int(time_offset) % self.sequence_length
            
            return self.sequence[bit_index]
    
    def get_sequence(self) -> np.ndarray:
        """
        Get complete PRBS sequence.
        
        Returns:
            Full PRBS sequence array
        """
        return self.sequence.copy()
    
    def reset_time(self, start_time: Optional[float] = None):
        """
        Reset time reference.
        
        Args:
            start_time: New start time (None = use current time)
        """
        with self.lock:
            self.start_time = start_time if start_time is not None else time.time()
    
    def __repr__(self):
        return (f"PRBSGenerator(length={self.sequence_length}, "
                f"rate={self.injection_rate_hz:.1f}Hz, "
                f"bit_period={self.bit_period*1000:.1f}ms)")


# =============================================================================
# SYNCHRONIZATION ENGINE
# =============================================================================

@dataclass
class SyncState:
    """Synchronization state snapshot."""
    timestamp: float  # Time of measurement
    offset_ms: float  # Time offset between streams (ms)
    drift_rate_ppm: float  # Drift rate (parts per million)
    correlation_peak: float  # Cross-correlation peak value (0-1)
    confidence: float  # Confidence metric (0-1)
    samples_analyzed: int  # Number of samples in correlation window


class SynchronizationEngine:
    """
    Real-time drift detection and correction using PRBS cross-correlation.
    
    Maintains synchronized timestamps between two independent data streams
    by continuously monitoring PRBS markers and applying drift corrections.
    
    Features:
    - Cross-correlation-based offset detection
    - Kalman filtering for smooth corrections
    - Drift rate estimation
    - Quality metrics (correlation peak, confidence)
    - Thread-safe operation
    
    Example:
        >>> prbs_gen = PRBSGenerator()
        >>> sync_engine = SynchronizationEngine(prbs_gen)
        >>> 
        >>> # Add data from both streams
        >>> sync_engine.add_emg_data(emg_timestamps, emg_prbs_markers)
        >>> sync_engine.add_imu_data(imu_timestamps, imu_prbs_markers)
        >>> 
        >>> # Get corrected timestamp
        >>> corrected_time = sync_engine.get_corrected_timestamp(raw_imu_time)
    """
    
    def __init__(self,
                 prbs_generator: PRBSGenerator,
                 correlation_window_s: float = 5.0,
                 update_interval_s: float = 5.0,
                 max_drift_ms: float = 50.0,
                 drift_warning_ms: float = 10.0,
                 simulation_mode: bool = False):
        """
        Initialize synchronization engine.
        
        Args:
            prbs_generator: PRBS generator instance
            correlation_window_s: Window size for cross-correlation (seconds)
            update_interval_s: How often to recompute offset (seconds)
            max_drift_ms: Maximum allowed drift before error (milliseconds)
            drift_warning_ms: Drift threshold for warnings (milliseconds)
            simulation_mode: Use data timestamps instead of wall clock for timing (for fast validation)
        """
        self.prbs_gen = prbs_generator
        self.correlation_window_s = correlation_window_s
        self.update_interval_s = update_interval_s
        self.max_drift_ms = max_drift_ms
        self.drift_warning_ms = drift_warning_ms
        self.simulation_mode = simulation_mode
        
        # Buffers for PRBS markers (time, marker)
        self.emg_buffer: Deque[Tuple[float, float]] = deque(maxlen=10000)
        self.imu_buffer: Deque[Tuple[float, float]] = deque(maxlen=10000)
        
        # Synchronization state
        self.current_offset_ms = 0.0  # Current time offset (IMU - EMG)
        self.drift_rate_ppm = 0.0  # Drift rate in parts per million
        self.last_update_time = 0.0  # Wall clock time of last update (real-time mode)
        self.last_data_time = 0.0  # Data timestamp of last update (simulation mode)
        self.sync_state: Optional[SyncState] = None
        
        # Kalman filter state (for smooth offset tracking)
        self.kf_offset = 0.0  # Filtered offset
        self.kf_variance = 100.0  # Initial variance (high uncertainty)
        self.kf_process_noise = 0.01  # Process noise (drift over time)
        self.kf_measurement_noise = 1.0  # Measurement noise (correlation uncertainty)
        
        # Statistics
        self.update_count = 0
        self.drift_warnings = 0
        self.drift_errors = 0
        
        # Thread safety
        self.lock = threading.Lock()
        
        # Start time
        self.start_time = time.time()
    
    def add_emg_data(self, timestamp: float, prbs_marker: float):
        """
        Add EMG data point with PRBS marker.
        
        Args:
            timestamp: Data timestamp (seconds)
            prbs_marker: PRBS marker value (±1)
        """
        with self.lock:
            self.emg_buffer.append((timestamp, prbs_marker))
    
    def add_imu_data(self, timestamp: float, prbs_marker: float):
        """
        Add IMU data point with PRBS marker.
        
        Args:
            timestamp: Data timestamp (seconds)
            prbs_marker: PRBS marker value (±1)
        """
        with self.lock:
            self.imu_buffer.append((timestamp, prbs_marker))
    
    def should_update(self) -> bool:
        """
        Check if it's time to recompute synchronization.
        
        In real-time mode: Uses wall clock time.
        In simulation mode: Uses data timestamps (enables fast validation).
        
        Returns:
            True if update should be performed
        """
        if self.simulation_mode:
            # Simulation mode: Use most recent data timestamp instead of wall clock
            if len(self.emg_buffer) == 0 and len(self.imu_buffer) == 0:
                return False
            
            current_data_time = max(
                self.emg_buffer[-1][0] if self.emg_buffer else 0,
                self.imu_buffer[-1][0] if self.imu_buffer else 0
            )
            
            if self.last_data_time == 0.0:
                self.last_data_time = current_data_time
                return True  # First update
            
            return (current_data_time - self.last_data_time) >= self.update_interval_s
        else:
            # Real-time mode: Original wall clock logic
            current_time = time.time()
            if self.last_update_time == 0.0:
                return True  # First update
            
            return (current_time - self.last_update_time) >= self.update_interval_s
    
    def compute_time_offset(self) -> Optional[Tuple[float, float]]:
        """
        Compute time offset between streams using cross-correlation.
        
        Returns:
            (offset_ms, correlation_confidence) or None if insufficient data
        """
        with self.lock:
            # Check if we have enough data for reliable correlation
            # Need at least 2-3 PRBS cycles worth of samples
            # With 10 Hz injection rate and 5s window, we expect ~50 samples
            min_samples = max(50, int(2.5 * self.correlation_window_s * self.prbs_gen.injection_rate_hz))
            
            if len(self.emg_buffer) < min_samples or len(self.imu_buffer) < min_samples:
                return None
            
            # Extract data from buffers
            emg_times = np.array([t for t, _ in self.emg_buffer])
            emg_markers = np.array([m for _, m in self.emg_buffer])
            imu_times = np.array([t for t, _ in self.imu_buffer])
            imu_markers = np.array([m for _, m in self.imu_buffer])
            
            # Get most recent window
            current_time = max(emg_times[-1], imu_times[-1])
            window_start = current_time - self.correlation_window_s
            
            # Filter to window
            emg_mask = emg_times >= window_start
            imu_mask = imu_times >= window_start
            
            emg_times_win = emg_times[emg_mask]
            emg_markers_win = emg_markers[emg_mask]
            imu_times_win = imu_times[imu_mask]
            imu_markers_win = imu_markers[imu_mask]
            
            if len(emg_times_win) < 10 or len(imu_times_win) < 10:
                return None
            
            # Resample to common time base (use finer resolution)
            # Choose sample rate based on PRBS injection rate
            resample_dt = 0.5 / self.prbs_gen.injection_rate_hz  # 2x Nyquist
            
            time_min = max(emg_times_win[0], imu_times_win[0])
            time_max = min(emg_times_win[-1], imu_times_win[-1])
            
            if time_max <= time_min:
                return None
            
            common_times = np.arange(time_min, time_max, resample_dt)
            
            if len(common_times) < 20:
                return None
            
            # Interpolate both signals to common time base
            emg_resampled = np.interp(common_times, emg_times_win, emg_markers_win)
            imu_resampled = np.interp(common_times, imu_times_win, imu_markers_win)
            
            # Compute cross-correlation
            # np.correlate(a, b, mode='full'):
            #   - Slides signal b across signal a
            #   - Result[k] corresponds to lag of b relative to a
            #   - Positive lag: b is delayed (behind) relative to a
            #   - Negative lag: b is ahead of a
            # In our case: a=EMG, b=IMU
            correlation = np.correlate(emg_resampled, imu_resampled, mode='full')
            correlation = correlation / (np.std(emg_resampled) * np.std(imu_resampled) * len(emg_resampled))
            
            # Find peak
            peak_idx = np.argmax(np.abs(correlation))
            peak_value = correlation[peak_idx]
            
            # Convert peak index to time offset
            # Positive offset_ms: IMU is delayed (behind) EMG → IMU timestamps need to be shifted back
            # Negative offset_ms: IMU is ahead of EMG → IMU timestamps need to be shifted forward
            # Usage: corrected_imu_time = imu_time - offset_ms/1000
            center_idx = len(correlation) // 2
            lag_samples = peak_idx - center_idx
            offset_seconds = lag_samples * resample_dt
            offset_ms = offset_seconds * 1000.0
            
            # Confidence based on peak sharpness and magnitude
            peak_magnitude = abs(peak_value)
            # Measure sharpness: ratio of peak to mean of surrounding values
            window_size = min(5, len(correlation) // 10)
            surrounding = np.concatenate([
                correlation[max(0, peak_idx - window_size):peak_idx],
                correlation[peak_idx + 1:min(len(correlation), peak_idx + window_size + 1)]
            ])
            if len(surrounding) > 0:
                sharpness = peak_magnitude / (np.mean(np.abs(surrounding)) + 1e-6)
            else:
                sharpness = 1.0
            
            confidence = min(1.0, peak_magnitude * 0.5 + min(1.0, sharpness / 10.0) * 0.5)
            
            return (offset_ms, confidence)
    
    def update_sync(self):
        """
        Update synchronization state by computing new offset.
        
        Updates internal offset estimate using Kalman filtering.
        """
        result = self.compute_time_offset()
        
        if result is None:
            return
        
        offset_ms, confidence = result
        
        with self.lock:
            # Kalman filter update
            # Prediction step (assume constant offset with slow drift)
            self.kf_variance += self.kf_process_noise
            
            # Update step
            kalman_gain = self.kf_variance / (self.kf_variance + self.kf_measurement_noise / confidence)
            self.kf_offset += kalman_gain * (offset_ms - self.kf_offset)
            self.kf_variance *= (1.0 - kalman_gain)
            
            # Update current offset (use filtered value)
            old_offset = self.current_offset_ms
            self.current_offset_ms = self.kf_offset
            
            # Estimate drift rate
            if self.simulation_mode:
                # Simulation mode: use data timestamps
                current_data_time = max(
                    self.emg_buffer[-1][0] if self.emg_buffer else 0,
                    self.imu_buffer[-1][0] if self.imu_buffer else 0
                )
                if self.last_data_time > 0:
                    dt_hours = (current_data_time - self.last_data_time) / 3600.0
                    if dt_hours > 0:
                        drift_ms = self.current_offset_ms - old_offset
                        # Convert to ppm (parts per million)
                        self.drift_rate_ppm = (drift_ms / 1000.0) / dt_hours * 1e6
                self.last_data_time = current_data_time
            else:
                # Real-time mode: use wall clock
                current_time = time.time()
                if self.last_update_time > 0:
                    dt_hours = (current_time - self.last_update_time) / 3600.0
                    if dt_hours > 0:
                        drift_ms = self.current_offset_ms - old_offset
                        # Convert to ppm (parts per million)
                        self.drift_rate_ppm = (drift_ms / 1000.0) / dt_hours * 1e6
                self.last_update_time = current_time
            
            self.update_count += 1
            
            # Store sync state
            state_timestamp = self.last_data_time if self.simulation_mode else self.last_update_time
            self.sync_state = SyncState(
                timestamp=state_timestamp,
                offset_ms=self.current_offset_ms,
                drift_rate_ppm=self.drift_rate_ppm,
                correlation_peak=abs(confidence),
                confidence=confidence,
                samples_analyzed=min(len(self.emg_buffer), len(self.imu_buffer))
            )
            
            # Check thresholds
            abs_offset = abs(self.current_offset_ms)
            if abs_offset > self.max_drift_ms:
                self.drift_errors += 1
                print(f"\n[SYNC] ERROR: Drift exceeds maximum ({abs_offset:.1f} ms > {self.max_drift_ms:.1f} ms)")
            elif abs_offset > self.drift_warning_ms:
                self.drift_warnings += 1
                if self.drift_warnings % 10 == 1:  # Print every 10th warning
                    print(f"\n[SYNC] WARNING: Drift detected ({abs_offset:.1f} ms)")
    
    def get_corrected_timestamp(self, imu_timestamp: float) -> float:
        """
        Apply drift correction to IMU timestamp.
        
        Args:
            imu_timestamp: Raw IMU timestamp (seconds)
        
        Returns:
            Corrected timestamp aligned with EMG time base (seconds)
        """
        with self.lock:
            # Apply offset correction (convert ms to seconds)
            corrected = imu_timestamp - (self.current_offset_ms / 1000.0)
            return corrected
    
    def get_sync_state(self) -> Optional[SyncState]:
        """
        Get current synchronization state.
        
        Returns:
            SyncState snapshot or None if not yet computed
        """
        with self.lock:
            return self.sync_state
    
    def get_statistics(self) -> dict:
        """
        Get synchronization statistics.
        
        Returns:
            Dictionary with sync stats
        """
        with self.lock:
            runtime_s = time.time() - self.start_time
            
            return {
                'runtime_seconds': runtime_s,
                'runtime_hours': runtime_s / 3600.0,
                'update_count': self.update_count,
                'current_offset_ms': self.current_offset_ms,
                'drift_rate_ppm': self.drift_rate_ppm,
                'drift_warnings': self.drift_warnings,
                'drift_errors': self.drift_errors,
                'emg_buffer_size': len(self.emg_buffer),
                'imu_buffer_size': len(self.imu_buffer),
                'kalman_variance': self.kf_variance,
                'last_update_age_s': time.time() - self.last_update_time if self.last_update_time > 0 else 0
            }
    
    def __repr__(self):
        return (f"SynchronizationEngine(offset={self.current_offset_ms:.2f}ms, "
                f"drift={self.drift_rate_ppm:.1f}ppm, updates={self.update_count})")


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def compute_autocorrelation(prbs_sequence: np.ndarray) -> np.ndarray:
    """
    Compute autocorrelation of PRBS sequence.
    
    Useful for validation - ideal PRBS has sharp peak at zero lag.
    
    Args:
        prbs_sequence: PRBS sequence array
    
    Returns:
        Autocorrelation array
    """
    n = len(prbs_sequence)
    autocorr = np.correlate(prbs_sequence, prbs_sequence, mode='full')
    autocorr = autocorr / (n * np.var(prbs_sequence))
    return autocorr


def validate_prbs_quality(prbs_sequence: np.ndarray) -> dict:
    """
    Validate PRBS sequence quality.
    
    Args:
        prbs_sequence: PRBS sequence array
    
    Returns:
        Dictionary with quality metrics
    """
    autocorr = compute_autocorrelation(prbs_sequence)
    center_idx = len(autocorr) // 2
    
    # Peak value at zero lag
    peak_value = autocorr[center_idx]
    
    # Mean of sidelobes (excluding peak)
    sidelobes = np.concatenate([autocorr[:center_idx-1], autocorr[center_idx+2:]])
    sidelobe_mean = np.mean(np.abs(sidelobes))
    sidelobe_max = np.max(np.abs(sidelobes))
    
    # Peak to sidelobe ratio
    psr = peak_value / (sidelobe_mean + 1e-10)
    
    return {
        'peak_value': float(peak_value),
        'sidelobe_mean': float(sidelobe_mean),
        'sidelobe_max': float(sidelobe_max),
        'peak_to_sidelobe_ratio': float(psr),
        'sequence_length': len(prbs_sequence),
        'quality': 'excellent' if psr > 100 else 'good' if psr > 50 else 'poor'
    }


if __name__ == "__main__":
    # Demo and validation
    print("PRBS Synchronization Module Demo")
    print("=" * 60)
    
    # Create PRBS generator
    prbs = PRBSGenerator(sequence_length=255, injection_rate_hz=10.0)
    print(f"\n{prbs}")
    
    # Validate sequence quality
    quality = validate_prbs_quality(prbs.get_sequence())
    print(f"\nPRBS Quality Metrics:")
    print(f"  Peak value: {quality['peak_value']:.3f}")
    print(f"  Sidelobe mean: {quality['sidelobe_mean']:.3f}")
    print(f"  Sidelobe max: {quality['sidelobe_max']:.3f}")
    print(f"  Peak-to-sidelobe ratio: {quality['peak_to_sidelobe_ratio']:.1f}")
    print(f"  Quality: {quality['quality']}")
    
    # Test time-indexed access
    print(f"\nTime-indexed access test:")
    for t in [0.0, 0.05, 0.1, 0.15, 0.2]:
        marker = prbs.get_marker_at_time(t)
        print(f"  t={t:.2f}s → marker={marker:+.0f}")
    
    # Create synchronization engine
    print(f"\n{'='*60}")
    sync_engine = SynchronizationEngine(prbs, correlation_window_s=2.0, update_interval_s=1.0)
    print(f"{sync_engine}")
    
    # Simulate synchronized data streams with known offset
    print(f"\nSimulating synchronized streams with 10ms offset...")
    known_offset_ms = 10.0  # Ground truth
    
    for i in range(100):
        t_emg = i * 0.01  # 100 Hz
        t_imu = t_emg + (known_offset_ms / 1000.0)  # IMU is 10ms ahead
        
        marker_emg = prbs.get_marker_at_time(t_emg)
        marker_imu = prbs.get_marker_at_time(t_imu)
        
        sync_engine.add_emg_data(t_emg, marker_emg)
        sync_engine.add_imu_data(t_imu, marker_imu)
    
    # Compute offset
    result = sync_engine.compute_time_offset()
    if result:
        measured_offset_ms, confidence = result
        error_ms = abs(measured_offset_ms - known_offset_ms)
        print(f"  Known offset: {known_offset_ms:.2f} ms")
        print(f"  Measured offset: {measured_offset_ms:.2f} ms")
        print(f"  Error: {error_ms:.2f} ms")
        print(f"  Confidence: {confidence:.3f}")
        print(f"  [{'PASS' if error_ms < 1.0 else 'FAIL'}] Test {'PASSED' if error_ms < 1.0 else 'FAILED'}")
    else:
        print(f"  [FAIL] Test FAILED: Could not compute offset")
    
    print(f"\n{'='*60}")
    print("[OK] Demo complete")
