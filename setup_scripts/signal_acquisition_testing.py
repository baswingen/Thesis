"""
Synchronized IMU-EMG Signal Acquisition
========================================

Real-time synchronized acquisition of dual BMI160 IMU data (~200Hz) and 
TMSi EMG data (~2000Hz+) with hardware-timestamp-based alignment and live visualization.

This script combines:
- Dual IMU acquisition via Arduino/serial (src.IMUDevice with auto-detection)
- Multi-channel EMG acquisition via TMSi USB (src.EMGDevice)
- Thread-safe data buffering with timestamp synchronization
- Real-time matplotlib visualization

Hardware Requirements:
- Arduino with dual BMI160 IMUs (auto-detected on Windows, macOS, Linux)
- TMSi Porti7/REFA device (USB connection, Windows only)
- EMG electrodes configured as differential pairs

Synchronization Approach:
- ✅ Hardware timestamp synchronization using IMU's Arduino microsecond counter
- ✅ EMG counter-based timing for precise sample-accurate timestamps
- ✅ Common time base established at acquisition start
- ✅ No smoothing - raw timestamps for downstream signal processing
- ✅ Sample counter tracking for dropped sample detection
- ✅ Fallback to receive-time-based timestamps if counter unavailable

Features:
- ✅ Cross-platform Arduino auto-detection (no manual port configuration)
- ✅ Hardware-synchronized timestamping for IMU and EMG data
- ✅ Real-time visualization with matplotlib
- ✅ Thread-safe data buffering
- ✅ Configurable acquisition duration and plot windows
- ✅ Timing statistics and latency monitoring

Author: Generated for Master Thesis
Date: January 2026
"""

import time
import numpy as np
import threading
from collections import deque
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Any
import sys
from pathlib import Path

# Add project root to path
_project_root = Path(__file__).parent.parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

# Visualization
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for better compatibility
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec

# Import from src package
from src import (
    IMUDevice,
    IMUConfig,
    IMUReading,
    IMUType,
    EMGDevice
)

# =============================================================================
# CONFIGURATION
# =============================================================================

# Use a monotonic clock for all synchronization/timing.
# IMPORTANT: time.time() can jump (NTP / manual clock adjustments), which will
# create apparent drift/offset between devices and even negative "latency".
CLOCK = time.perf_counter

# System Configuration
# --------------------
# Set to False to disable a device (useful for testing one system at a time)
ENABLE_EMG = True   # Enable TMSi EMG acquisition
ENABLE_IMU = True   # Enable dual BMI160 IMU acquisition

# EMG Configuration
# -----------------
# NOTE: TMSi devices require Windows! The TMSiSDK.dll is Windows-only.
# Set ENABLE_EMG = False to test IMU on macOS.
#
# Configure differential pairs: (positive_channel, negative_channel)
# Example: [(0, 1), (2, 3)] means two muscle pairs
EMG_DIFFERENTIAL_PAIRS = [
    (1,2),  # Pair 1: Channel 0 (+) - Channel 1 (-)
    # (2, 3),  # Pair 2: Channel 2 (+) - Channel 3 (-) (uncomment if needed)
]

EMG_CONNECTION_TYPE = 'usb'  # 'usb', 'bluetooth', 'network', 'wifi'
# If you're seeing real-time lag/backlog, try lowering this.
# Common stable values on legacy TMSi devices are 1024 or 2048 Hz (device dependent).
EMG_SAMPLE_RATE = 1024  # None = device default (often 2048 or 4096 Hz)

# IMU Configuration
# -----------------
# Auto-detection works on Windows (COM ports), macOS (/dev/cu.*), and Linux (/dev/tty*)
IMU_PORT = None  # None = auto-detect (recommended), or specify like "COM3" or "/dev/cu.usbmodem..."
IMU_BAUD = 230400
IMU_CALIBRATION_SAMPLES = 200  # Number of samples for gyro bias calibration

# Acquisition Settings
# --------------------
ACQUISITION_DURATION = 60.0  # seconds (None = run indefinitely until Ctrl+C)
PLOT_WINDOW_SECONDS = 5.0    # Time window to display in plots
PLOT_UPDATE_INTERVAL = 33    # milliseconds (~30 FPS)

# Buffer Settings
# ---------------
MAX_BUFFER_SIZE = 100000  # Maximum samples to keep in memory

# Visualization Settings
# ----------------------
SHOW_IMU1 = True   # Show IMU1 orientation
SHOW_IMU2 = True   # Show IMU2 orientation
EMG_PLOT_SCALE = 'auto'  # 'auto' or specific range like (-500, 500)

# Debug Settings
# --------------
VERBOSE = True  # Print detailed acquisition info
SHOW_TIMING_STATS = True  # Display timing statistics

# =============================================================================
# SYNCHRONIZATION METHODOLOGY
# =============================================================================
"""
Cross-Device Timestamp Synchronization Strategy
------------------------------------------------

CHALLENGE:
EMG (TMSi) and IMU (Arduino) are separate hardware devices with independent clocks.
They cannot share a hardware clock signal, so software synchronization is required.

APPROACH:

1. IMU Synchronization (Hardware Timestamps):
   - IMU provides hardware timestamps via Arduino's micros() counter
   - On first sample, establish time offset: Python_time = Arduino_time + offset
   - All subsequent IMU samples use hardware timestamp + offset
   - Advantages: Immune to serial communication jitter, precise timing

2. EMG Synchronization (Counter-Based or Time-Based):
   - **Counter-based (preferred)**: Use COUNTER channel to track exact sample count
     * Timestamp = start_time + (counter_value / sample_rate)
     * Immune to USB jitter and buffering delays
     * Precise timing based on device's internal counter
   - **Time-based (fallback)**: If no counter channel available
     * Timestamp = receive_time - chunk_duration
     * Subject to USB latency variability (10-50ms typical)
   - No smoothing - raw timestamps for downstream processing

3. Common Time Base:
   - Both devices establish their time offset at acquisition start
   - All timestamps reference Python's time.time() as common base
   - Latency compensation: timestamp immediately upon data reception

4. Jitter Reduction:
   - IMU: Hardware timestamps eliminate serial jitter
   - EMG: Moving average smoothing reduces USB polling jitter
   - Both: Timestamp at earliest possible moment

ACCURACY:
- IMU: ~1µs precision (limited by Arduino clock)
- EMG (counter-based): ~0.5ms precision (1/sample_rate)
- EMG (time-based fallback): ~10-50ms precision (limited by USB latency)
- Relative timing: <1ms error with counter-based, <50ms with time-based

LIMITATIONS:
- No hardware clock sync (impossible with these devices)
- Assumes stable sample rates (devices don't drift significantly)
- Initial offset assumes negligible delay between device starts
- Clock drift over long acquisitions (minutes to hours) not compensated
"""

# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class IMUData:
    """Container for IMU data with timestamp."""
    timestamp: float  # Python time.time() - synchronized
    reading: IMUReading  # IMUReading object from src.imu_acquisition
    hardware_time: float  # Arduino microseconds converted to seconds

@dataclass
class EMGData:
    """Container for EMG differential pair data with timestamp."""
    timestamp: float  # Python time.time() - synchronized
    pairs: np.ndarray  # Differential pair values [pair1, pair2, ...]
    sample_count: int  # Number of samples in this chunk
    counter: Optional[int] = None  # Sample counter from COUNTER channel (if available)


# =============================================================================
# THREAD-SAFE BUFFER
# =============================================================================

class TimestampedBuffer:
    """Thread-safe buffer for timestamped data with efficient windowed queries."""
    
    def __init__(self, maxlen: int = 10000):
        """
        Initialize buffer.
        
        Args:
            maxlen: Maximum number of items to store
        """
        self.buffer = deque(maxlen=maxlen)
        self.lock = threading.Lock()
        self.total_count = 0
    
    def append(self, timestamp: float, data: Any):
        """
        Append timestamped data.
        
        Args:
            timestamp: Time of data acquisition
            data: Data to store
        """
        with self.lock:
            self.buffer.append((timestamp, data))
            self.total_count += 1
    
    def get_recent(self, window_seconds: float) -> List[Tuple[float, Any]]:
        """
        Get data from the last N seconds.
        
        Args:
            window_seconds: Time window in seconds
            
        Returns:
            List of (timestamp, data) tuples
        """
        with self.lock:
            if not self.buffer:
                return []
            
            t_now = CLOCK()
            t_cutoff = t_now - window_seconds
            
            # Efficient: start from end and work backwards
            result = []
            for t, d in reversed(self.buffer):
                if t >= t_cutoff:
                    result.append((t, d))
                else:
                    break  # Since deque is ordered, we can stop
            
            result.reverse()
            return result
    
    def get_all(self) -> List[Tuple[float, Any]]:
        """Get all data in buffer."""
        with self.lock:
            return list(self.buffer)
    
    def clear(self):
        """Clear all data."""
        with self.lock:
            self.buffer.clear()
    
    def __len__(self) -> int:
        """Get current buffer size."""
        with self.lock:
            return len(self.buffer)


# =============================================================================
# IMU ACQUISITION THREAD
# =============================================================================

class IMUAcquisitionThread(threading.Thread):
    """Thread for continuous IMU data acquisition with hardware timestamp synchronization."""
    
    def __init__(self, buffer: TimestampedBuffer, config: IMUConfig, start_event: threading.Event):
        """
        Initialize IMU acquisition thread.
        
        Args:
            buffer: Shared buffer for storing data
            config: IMU configuration
            start_event: Event used to start acquisition simultaneously across devices
        """
        super().__init__(name="IMU-Thread", daemon=True)
        self.buffer = buffer
        self.config = config
        self.start_event = start_event
        self.ready_event = threading.Event()
        self.device: Optional[IMUDevice] = None
        self.running = False
        self.error = None
        self.sample_count = 0
        self.start_time = None  # Python time when acquisition starts
        self.start_time_hardware = None  # Arduino t_us when acquisition starts
        self.time_offset = None  # Offset to convert Arduino time to Python time
        self._last_t_us = None  # Track Arduino time to detect resets/reconnects
    
    def run(self):
        """Main thread loop."""
        try:
            # Initialize device
            print("\n[IMU] Initializing device...")
            self.device = IMUDevice(config=self.config)
            self.device.connect()
            
            print("[IMU] Connected successfully")
            print(f"[IMU] Configuration: {self.config.baud} baud")
            
            # Calibration
            print("\n[IMU] Starting calibration...")
            print("[IMU] ⚠️  Place IMUs FLAT and STILL on a surface!")
            time.sleep(3)
            
            self.device.calibrate(samples=IMU_CALIBRATION_SAMPLES)
            print("[IMU] ✓ Calibration complete")

            self.ready_event.set()
            print("[IMU] Ready. Waiting for synchronized start...")

            # Wait for synchronized start
            self.start_event.wait()
            if not self.running:
                self.running = True

            # Start acquisition (synchronized start moment)
            self.start_time = CLOCK()
            print("[IMU] Starting data acquisition (synchronized)...")

            # IMPORTANT: Flush any serial backlog so our first sample is "fresh".
            # If the Arduino kept streaming while we were calibrating / waiting for start,
            # the OS serial buffer can contain old packets. Using the oldest buffered packet
            # to establish the time offset can make subsequent timestamps appear "in the future",
            # producing negative latency and making EMG look seconds behind.
            try:
                if self.device and getattr(self.device, "ser", None) is not None:
                    self.device.ser.reset_input_buffer()
                    # Give the UART/USB stack a moment to settle
                    time.sleep(0.01)
            except Exception:
                pass
            
            # Establish time synchronization baseline
            first_sample = True
            
            # Continuous reading loop
            for reading in self.device.read_stream():
                if not self.running:
                    break
                
                # Timestamp IMMEDIATELY upon receiving data to minimize latency
                receive_time = CLOCK()

                # Detect Arduino reset/reconnect (micros() jumps backwards)
                # This can happen after a USB disconnect/reconnect or Arduino auto-reset.
                if self._last_t_us is not None:
                    if reading.t_us < self._last_t_us and (self._last_t_us - reading.t_us) > 10000:
                        print("\n[IMU] ⚠️ Detected Arduino time reset (t_us jumped backwards). Re-syncing...")
                        # Flush any backlog that may have accumulated during reconnect
                        try:
                            if self.device and getattr(self.device, "ser", None) is not None:
                                self.device.ser.reset_input_buffer()
                                time.sleep(0.01)
                        except Exception:
                            pass
                        first_sample = True  # Re-establish offset on the next sample

                self._last_t_us = reading.t_us
                
                # Establish time base on first sample
                if first_sample:
                    self.start_time_hardware = reading.t_us
                    self.time_offset = receive_time - (reading.t_us / 1_000_000.0)
                    first_sample = False
                    print(f"[IMU] Time base established at first sample")
                    print(f"      Hardware time: {self.start_time_hardware} µs")
                    print(f"      Receive time: {receive_time:.6f} s")
                    print(f"      Time offset: {self.time_offset:.6f} s")
                
                # Calculate synchronized timestamp using hardware time
                # This accounts for jitter in serial communication
                hardware_time_sec = reading.t_us / 1_000_000.0  # Convert µs to seconds
                synchronized_timestamp = hardware_time_sec + self.time_offset
                
                # Create IMU data with synchronized timestamp
                imu_data = IMUData(
                    timestamp=synchronized_timestamp,
                    reading=reading,
                    hardware_time=hardware_time_sec
                )
                
                self.buffer.append(synchronized_timestamp, imu_data)
                self.sample_count += 1
                
                if VERBOSE and self.sample_count % 100 == 0:
                    latency = receive_time - synchronized_timestamp
                    print(f"[IMU] Samples: {self.sample_count}, Latency: {latency*1000:.1f}ms")
        
        except Exception as e:
            self.error = e
            print(f"\n[IMU] ❌ Error: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            print("[IMU] Shutting down...")
            if self.device:
                self.device.close()
    
    def stop(self):
        """Stop acquisition."""
        self.running = False
    
    def get_stats(self) -> Dict[str, Any]:
        """Get acquisition statistics."""
        if self.start_time:
            elapsed = CLOCK() - self.start_time
            rate = self.sample_count / elapsed if elapsed > 0 else 0
        else:
            elapsed = 0
            rate = 0
        
        return {
            'samples': self.sample_count,
            'elapsed': elapsed,
            'rate': rate,
            'buffer_size': len(self.buffer),
            'error': self.error
        }


# =============================================================================
# EMG ACQUISITION THREAD
# =============================================================================

class EMGAcquisitionThread(threading.Thread):
    """Thread for continuous EMG data acquisition with timestamp synchronization."""
    
    def __init__(self, buffer: TimestampedBuffer, start_event: threading.Event, connection_type: str = 'usb'):
        """
        Initialize EMG acquisition thread.
        
        Args:
            buffer: Shared buffer for storing data
            start_event: Event used to start acquisition simultaneously across devices
            connection_type: Connection type for EMG device
        """
        super().__init__(name="EMG-Thread", daemon=True)
        self.buffer = buffer
        self.start_event = start_event
        self.ready_event = threading.Event()
        self.connection_type = connection_type
        self.device: Optional[EMGDevice] = None
        self.running = False
        self.error = None
        self.sample_count = 0
        self.chunk_count = 0
        self.start_time = None  # Python time when acquisition starts
        self.counter_channel_idx = None  # Index of COUNTER channel (if available)
        self.last_counter_raw = None  # Last raw counter value (int)
        self.last_counter_unwrapped = None  # Last unwrapped counter value (int)
        self.counter_unwrap_offset = 0  # Offset added after wraps
        self.counter_modulus = None  # Detected wrap modulus (e.g., 65536) or None
        self.first_counter_unwrapped = None  # First unwrapped counter value baseline
        self.use_counter_timing = False  # Whether to use counter-based timing
        self.latency_est = None  # Estimated receive latency (seconds) for monitoring
        self.counter_time_offset = None  # Offset to map counter->time.time() domain
        self.counter_gain = None
        self.counter_offset = None
    
    def _counter_to_int(self, x_converted: float) -> Optional[int]:
        """
        Convert COUNTER channel value to integer sample counter.

        IMPORTANT: The legacy SDK converts raw uint32 channel values into physical units via:
            converted = (raw + unit_offset) * unit_gain
        For COUNTER, we want the original integer "raw" counter value, so we invert this
        conversion using the channel's unit gain/offset if available.
        """
        try:
            if x_converted is None or not np.isfinite(x_converted):
                return None

            x = float(x_converted)
            if self.counter_gain is not None and self.counter_offset is not None:
                g = float(self.counter_gain)
                o = float(self.counter_offset)
                if np.isfinite(g) and abs(g) > 0:
                    x = (x / g) - o

            # Counter should be integer-like
            return int(round(x))
        except Exception:
            return None
    
    def _unwrap_counter(self, raw_counter: int) -> int:
        """
        Convert possibly-wrapping raw counter into a monotonic counter.

        Notes:
        - The legacy SDK delivers channel samples as uint32 values converted to float64.
        - Many TMSi legacy devices expose a COUNTER that behaves like a 16-bit counter
          (wrap at 65536), but not all variants are documented consistently.
        - We detect wrap behavior dynamically from observed values.
        """
        if self.last_counter_raw is None:
            self.last_counter_raw = raw_counter
            self.last_counter_unwrapped = raw_counter
            return raw_counter

        # Detect counter width / wrap modulus
        if self.counter_modulus is None:
            # If we ever observe values > 16-bit, assume 32-bit counter (no wrap in practice)
            if raw_counter > 65535 or self.last_counter_raw > 65535:
                self.counter_modulus = 2**32
            else:
                # Detect a classic 16-bit wrap (high -> low jump)
                if raw_counter < self.last_counter_raw and (self.last_counter_raw - raw_counter) > 1000:
                    self.counter_modulus = 65536

        # Apply unwrap if using a modulus that can wrap
        if self.counter_modulus == 65536:
            if raw_counter < self.last_counter_raw:
                self.counter_unwrap_offset += 65536
        elif self.counter_modulus == 2**32:
            # 32-bit wrap is extremely unlikely in typical recordings; handle anyway
            if raw_counter < self.last_counter_raw and (self.last_counter_raw - raw_counter) > (2**31):
                self.counter_unwrap_offset += 2**32
        else:
            # If modulus is still unknown and counter decreases slightly, treat as discontinuity
            # without unwrapping.
            pass

        unwrapped = raw_counter + self.counter_unwrap_offset
        self.last_counter_raw = raw_counter
        self.last_counter_unwrapped = unwrapped
        return unwrapped
    
    def run(self):
        """Main thread loop."""
        try:
            # Initialize device
            print("\n[EMG] Initializing device...")
            self.device = EMGDevice(connection_type=self.connection_type)
            
            # Connect
            info = self.device.connect()
            print(f"[EMG] Connected to: {info['name']}")
            print(f"[EMG] Channels: {info['num_channels']}")
            print(f"[EMG] Sample rate: {info['sample_rate']} Hz")
            
            # Configure
            if EMG_SAMPLE_RATE is not None:
                self.device.configure_acquisition(sample_rate=EMG_SAMPLE_RATE)
                print(f"[EMG] Configured sample rate: {EMG_SAMPLE_RATE} Hz")
            
            # Verify differential pair configuration
            num_channels = info['num_channels']
            for pos_ch, neg_ch in EMG_DIFFERENTIAL_PAIRS:
                if pos_ch >= num_channels or neg_ch >= num_channels:
                    raise ValueError(
                        f"Differential pair ({pos_ch}, {neg_ch}) exceeds "
                        f"available channels (0-{num_channels-1})"
                    )
            
            print(f"[EMG] Differential pairs configured: {EMG_DIFFERENTIAL_PAIRS}")
            
            # Try to find COUNTER channel for precise timing
            for i, ch in enumerate(self.device.channels):
                ch_name = ch.get_channel_name().upper()
                if 'COUNTER' in ch_name or ch_name == 'COUNTER':
                    self.counter_channel_idx = i
                    self.use_counter_timing = True
                    print(f"[EMG] ✓ Found COUNTER channel at index {i}")
                    print(f"[EMG] ✓ Using counter-based timing for precise synchronization")
                    # Cache unit conversion parameters so we can recover raw integer counter values.
                    try:
                        self.counter_gain = ch.get_channel_unit_gain()
                    except Exception:
                        self.counter_gain = None
                    try:
                        self.counter_offset = ch.get_channel_unit_offset()
                    except Exception:
                        self.counter_offset = None

                    if self.counter_gain is not None or self.counter_offset is not None:
                        print(f"[EMG] COUNTER unit gain/offset: gain={self.counter_gain}, offset={self.counter_offset}")
                    break
            
            if self.counter_channel_idx is None:
                print("[EMG] ⚠️  COUNTER channel not found")
                print("[EMG] ⚠️  Using receive-time-based timestamps (less accurate)")

            self.ready_event.set()
            print("[EMG] Ready. Waiting for synchronized start...")

            # Wait for synchronized start
            self.start_event.wait()
            if not self.running:
                self.running = True

            # Start acquisition as close as possible to synchronized start moment
            self.device.start_acquisition()
            self.start_time = CLOCK()
            print(f"[EMG] Starting data acquisition (synchronized)...")

            # Continuous reading loop (after synchronized start)
            while self.running:
                samples = self.device.get_samples(blocking=False)
                
                if samples is not None and len(samples) > 0:
                    # Timestamp IMMEDIATELY upon receiving data
                    receive_time = CLOCK()
                    n_samples = samples.shape[0]
                    sample_period = 1.0 / self.device.sample_rate
                    
                    # Extract counter value if available
                    counter_raw = None
                    counter_unwrapped = None
                    if self.counter_channel_idx is not None:
                        counter_raw = self._counter_to_int(samples[-1, self.counter_channel_idx])
                        if counter_raw is not None:
                            counter_unwrapped = self._unwrap_counter(counter_raw)

                            # Establish baseline on first valid counter after start
                            if self.first_counter_unwrapped is None:
                                self.first_counter_unwrapped = counter_unwrapped
                                # Establish counter->time.time() mapping at first chunk using receive time.
                                # We anchor the LAST sample of the chunk close to receive_time.
                                # This avoids seconds-scale offsets caused by any device/driver startup delay.
                                self.counter_time_offset = receive_time - (counter_unwrapped * sample_period)

                                # Initialize latency estimate conservatively for monitoring only
                                chunk_duration = n_samples * sample_period
                                self.latency_est = min(0.02, chunk_duration / 2.0)
                                print(f"[EMG] Counter timing baseline established")
                                print(f"      Counter (raw): {counter_raw}")
                                print(f"      Counter (unwrapped): {counter_unwrapped}")
                                print(f"      Start time (python): {self.start_time:.6f} s")
                                print(f"      Counter time offset: {self.counter_time_offset:.6f} s")
                                print(f"      Initial latency est: {self.latency_est*1000:.1f} ms")

                    # Check for counter discontinuity (dropped samples) when counter is available
                    if self.use_counter_timing and counter_unwrapped is not None and self.last_counter_unwrapped is not None:
                        expected = self.last_counter_unwrapped + n_samples
                        if counter_unwrapped != expected:
                            gap = counter_unwrapped - expected
                            print(f"[EMG] ⚠️  Counter discontinuity: expected {expected}, got {counter_unwrapped} (Δ={gap} samples)")
                    
                    # =================================================================
                    # TIMESTAMP CALCULATION - Counter-based (precise) or Time-based (fallback)
                    # =================================================================
                    
                    if self.use_counter_timing and counter_unwrapped is not None and self.counter_time_offset is not None:
                        # METHOD 1: Counter-based timing (preferred)
                        # Map counter ticks to the Python time.time() domain via a learned offset.
                        # Counter corresponds to the LAST sample in this chunk.
                        last_sample_timestamp = (counter_unwrapped * sample_period) + self.counter_time_offset
                        mid_offset = ((n_samples - 1) / 2.0) * sample_period
                        chunk_timestamp = last_sample_timestamp - mid_offset

                        # Update latency estimate for monitoring (not used to compute timestamp)
                        latency_meas = receive_time - last_sample_timestamp
                        if np.isfinite(latency_meas):
                            if self.latency_est is None:
                                self.latency_est = float(latency_meas)
                            else:
                                self.latency_est = 0.95 * self.latency_est + 0.05 * float(latency_meas)

                        # Slowly adapt counter_time_offset to track small clock drift and DLL buffering jitter.
                        # (Keeps timestamps stable but prevents multi-minute drift.)
                        new_off = receive_time - (counter_unwrapped * sample_period)
                        if np.isfinite(new_off):
                            self.counter_time_offset = (0.995 * self.counter_time_offset) + (0.005 * float(new_off))
                        
                    else:
                        # METHOD 2: Receive-time-based (FALLBACK)
                        # Timestamp the CHUNK MIDPOINT because we store the CHUNK MEAN.
                        chunk_duration = n_samples * sample_period
                        chunk_timestamp = receive_time - (chunk_duration / 2.0)
                    
                    self.sample_count += n_samples
                    self.chunk_count += 1
                    
                    # Calculate differential pairs
                    # Average across all samples in this chunk for smoother display
                    pair_values = []
                    for pos_ch, neg_ch in EMG_DIFFERENTIAL_PAIRS:
                        diff = samples[:, pos_ch] - samples[:, neg_ch]
                        # Store mean value of chunk
                        pair_values.append(np.nanmean(diff))
                    
                    # Store in buffer with synchronized timestamp
                    emg_data = EMGData(
                        timestamp=chunk_timestamp,
                        pairs=np.array(pair_values),
                        sample_count=n_samples,
                        counter=counter_raw
                    )
                    self.buffer.append(chunk_timestamp, emg_data)
                    
                    if VERBOSE and self.chunk_count % 50 == 0:
                        latency = receive_time - chunk_timestamp
                        method = "counter" if self.use_counter_timing else "time"
                        lat_est_ms = (self.latency_est * 1000.0) if self.latency_est is not None else float("nan")
                        print(f"[EMG] Chunks: {self.chunk_count}, Samples: {self.sample_count}, "
                              f"Method: {method}, Latency: {latency*1000:.1f}ms, LatEst: {lat_est_ms:.1f}ms")
                
                time.sleep(0.01)  # Small delay to avoid busy-waiting
        
        except Exception as e:
            self.error = e
            print(f"\n[EMG] ❌ Error: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            print("[EMG] Shutting down...")
            if self.device:
                try:
                    self.device.stop_acquisition()
                    self.device.disconnect()
                except:
                    pass
    
    def stop(self):
        """Stop acquisition."""
        self.running = False
    
    def get_stats(self) -> Dict[str, Any]:
        """Get acquisition statistics."""
        if self.start_time:
            elapsed = CLOCK() - self.start_time
            rate = self.sample_count / elapsed if elapsed > 0 else 0
        else:
            elapsed = 0
            rate = 0
        
        return {
            'samples': self.sample_count,
            'chunks': self.chunk_count,
            'elapsed': elapsed,
            'rate': rate,
            'buffer_size': len(self.buffer),
            'error': self.error
        }


# =============================================================================
# REAL-TIME VISUALIZATION
# =============================================================================

class RealtimePlotter:
    """Real-time matplotlib visualization of synchronized IMU and EMG data."""
    
    def __init__(self, imu_buffer: TimestampedBuffer, emg_buffer: TimestampedBuffer):
        """
        Initialize plotter.
        
        Args:
            imu_buffer: Buffer containing IMU data
            emg_buffer: Buffer containing EMG data
        """
        self.imu_buffer = imu_buffer
        self.emg_buffer = emg_buffer
        self.start_time = None  # Will be set from first data timestamp
        
        # Create figure and subplots
        self._setup_figure()
        
        # Animation
        self.anim = None
        self.frame_count = 0
    
    def _setup_figure(self):
        """Setup matplotlib figure and subplots."""
        # Determine number of rows
        num_rows = 0
        if ENABLE_EMG:
            num_rows += 1
        if ENABLE_IMU and SHOW_IMU1:
            num_rows += 1
        if ENABLE_IMU and SHOW_IMU2:
            num_rows += 1
        
        if num_rows == 0:
            raise ValueError("No plots to display - enable at least one data source")
        
        # Create figure
        self.fig = plt.figure(figsize=(14, 4 * num_rows))
        self.fig.suptitle('Synchronized IMU-EMG Acquisition', 
                         fontsize=14, fontweight='bold')
        
        gs = GridSpec(num_rows, 1, figure=self.fig, hspace=0.3)
        
        # Create subplots
        row = 0
        
        # EMG subplot
        if ENABLE_EMG:
            self.ax_emg = self.fig.add_subplot(gs[row, 0])
            self.ax_emg.set_title('EMG Differential Pairs', fontweight='bold')
            self.ax_emg.set_xlabel('Time (s)')
            self.ax_emg.set_ylabel('Amplitude (µV)')
            self.ax_emg.grid(True, alpha=0.3)
            
            # Create line for each EMG pair
            self.emg_lines = []
            colors = plt.cm.tab10(np.linspace(0, 1, len(EMG_DIFFERENTIAL_PAIRS)))
            for i, (pos, neg) in enumerate(EMG_DIFFERENTIAL_PAIRS):
                line, = self.ax_emg.plot([], [], '-', linewidth=1.5, 
                                         color=colors[i],
                                         label=f'Pair {i+1} (Ch{pos}-Ch{neg})')
                self.emg_lines.append(line)
            self.ax_emg.legend(loc='upper right')
            row += 1
        else:
            self.ax_emg = None
            self.emg_lines = []
        
        # IMU1 subplot
        if ENABLE_IMU and SHOW_IMU1:
            self.ax_imu1 = self.fig.add_subplot(gs[row, 0])
            self.ax_imu1.set_title('IMU1 Orientation', fontweight='bold')
            self.ax_imu1.set_xlabel('Time (s)')
            self.ax_imu1.set_ylabel('Angle (degrees)')
            self.ax_imu1.grid(True, alpha=0.3)
            
            self.imu1_roll_line, = self.ax_imu1.plot([], [], 'r-', linewidth=1.5, label='Roll')
            self.imu1_pitch_line, = self.ax_imu1.plot([], [], 'g-', linewidth=1.5, label='Pitch')
            self.imu1_yaw_line, = self.ax_imu1.plot([], [], 'b-', linewidth=1.5, label='Yaw')
            self.ax_imu1.legend(loc='upper right')
            row += 1
        else:
            self.ax_imu1 = None
        
        # IMU2 subplot
        if ENABLE_IMU and SHOW_IMU2:
            self.ax_imu2 = self.fig.add_subplot(gs[row, 0])
            self.ax_imu2.set_title('IMU2 Orientation', fontweight='bold')
            self.ax_imu2.set_xlabel('Time (s)')
            self.ax_imu2.set_ylabel('Angle (degrees)')
            self.ax_imu2.grid(True, alpha=0.3)
            
            self.imu2_roll_line, = self.ax_imu2.plot([], [], 'r-', linewidth=1.5, label='Roll')
            self.imu2_pitch_line, = self.ax_imu2.plot([], [], 'g-', linewidth=1.5, label='Pitch')
            self.imu2_yaw_line, = self.ax_imu2.plot([], [], 'b-', linewidth=1.5, label='Yaw')
            self.ax_imu2.legend(loc='upper right')
            row += 1
        else:
            self.ax_imu2 = None
        
        plt.tight_layout()
    
    def update(self, frame):
        """Update plot with new data using synchronized time axis."""
        self.frame_count += 1
        
        # Get recent data
        imu_data = self.imu_buffer.get_recent(PLOT_WINDOW_SECONDS)
        emg_data = self.emg_buffer.get_recent(PLOT_WINDOW_SECONDS)
        
        # Establish common start time from first available data
        if self.start_time is None:
            start_times = []
            if imu_data:
                start_times.append(imu_data[0][1].timestamp)
            if emg_data:
                start_times.append(emg_data[0][1].timestamp)
            if start_times:
                self.start_time = min(start_times)
                print(f"[VIZ] Time axis reference established: t0 = {self.start_time:.6f} s")
            else:
                return []  # No data yet
        
        # Calculate common time axis limits for synchronized display
        # Use the latest timestamp from either device to set the time window
        latest_time = 0.0
        
        # Update EMG plot
        emg_times_rel = None
        if ENABLE_EMG and emg_data and self.ax_emg:
            # Extract synchronized timestamps
            emg_times = np.array([d.timestamp for _, d in emg_data])
            # Relative time (start from 0)
            emg_times_rel = emg_times - self.start_time
            
            # Track latest time
            if len(emg_times_rel) > 0:
                latest_time = max(latest_time, emg_times_rel[-1])
            
            # Extract pair values
            for pair_idx in range(len(EMG_DIFFERENTIAL_PAIRS)):
                pair_values = np.array([d.pairs[pair_idx] for _, d in emg_data])
                self.emg_lines[pair_idx].set_data(emg_times_rel, pair_values)
            
            if EMG_PLOT_SCALE == 'auto':
                all_values = np.concatenate([d.pairs for _, d in emg_data])
                y_min, y_max = np.nanmin(all_values), np.nanmax(all_values)
                y_range = y_max - y_min
                margin = y_range * 0.1 if y_range > 0 else 10
                self.ax_emg.set_ylim(y_min - margin, y_max + margin)
            else:
                self.ax_emg.set_ylim(EMG_PLOT_SCALE)
        
        # Update IMU plots
        imu_times_rel = None
        if ENABLE_IMU and imu_data:
            # Extract synchronized timestamps
            imu_times = np.array([d.timestamp for _, d in imu_data])
            imu_times_rel = imu_times - self.start_time
            
            # Track latest time
            if len(imu_times_rel) > 0:
                latest_time = max(latest_time, imu_times_rel[-1])
            
            if self.ax_imu1:
                rolls = np.array([d.reading.euler1[0] if d.reading.euler1 else np.nan for _, d in imu_data])
                pitches = np.array([d.reading.euler1[1] if d.reading.euler1 else np.nan for _, d in imu_data])
                yaws = np.array([d.reading.euler1[2] if d.reading.euler1 else np.nan for _, d in imu_data])
                
                self.imu1_roll_line.set_data(imu_times_rel, rolls)
                self.imu1_pitch_line.set_data(imu_times_rel, pitches)
                self.imu1_yaw_line.set_data(imu_times_rel, yaws)
                
                self.ax_imu1.relim()
                self.ax_imu1.autoscale_view(scalex=False)
            
            if self.ax_imu2:
                rolls = np.array([d.reading.euler2[0] if d.reading.euler2 else np.nan for _, d in imu_data])
                pitches = np.array([d.reading.euler2[1] if d.reading.euler2 else np.nan for _, d in imu_data])
                yaws = np.array([d.reading.euler2[2] if d.reading.euler2 else np.nan for _, d in imu_data])
                
                self.imu2_roll_line.set_data(imu_times_rel, rolls)
                self.imu2_pitch_line.set_data(imu_times_rel, pitches)
                self.imu2_yaw_line.set_data(imu_times_rel, yaws)
                
                self.ax_imu2.relim()
                self.ax_imu2.autoscale_view(scalex=False)
        
        # Apply synchronized X-axis limits to ALL plots
        # This ensures all time axes are perfectly aligned
        if latest_time > 0:
            x_min = max(0, latest_time - PLOT_WINDOW_SECONDS)
            x_max = latest_time + 0.1
            
            if self.ax_emg:
                self.ax_emg.set_xlim(x_min, x_max)
            if self.ax_imu1:
                self.ax_imu1.set_xlim(x_min, x_max)
            if self.ax_imu2:
                self.ax_imu2.set_xlim(x_min, x_max)
        
        # Update title with stats
        if SHOW_TIMING_STATS and self.frame_count % 10 == 0:
            if self.start_time is not None:
                elapsed = CLOCK() - self.start_time
                imu_count = len(self.imu_buffer)
                emg_count = len(self.emg_buffer)
                title = (f'Synchronized IMU-EMG Acquisition  |  '
                        f'Time: {elapsed:.1f}s  |  '
                        f'IMU: {imu_count} samples  |  '
                        f'EMG: {emg_count} chunks')
                self.fig.suptitle(title, fontsize=14, fontweight='bold')
        
        return []
    
    def start(self):
        """Start animation."""
        self.anim = FuncAnimation(
            self.fig, 
            self.update,
            interval=PLOT_UPDATE_INTERVAL,
            blit=False,
            cache_frame_data=False
        )
        plt.show()


# =============================================================================
# MAIN EXECUTION
# =============================================================================

def print_header():
    """Print script header."""
    print("\n" + "="*70)
    print("SYNCHRONIZED IMU-EMG SIGNAL ACQUISITION")
    print("="*70)
    print("\nConfiguration:")
    print(f"  IMU Enabled: {ENABLE_IMU}")
    print(f"  EMG Enabled: {ENABLE_EMG}")
    if ENABLE_EMG:
        print(f"  EMG Differential Pairs: {EMG_DIFFERENTIAL_PAIRS}")
    if ENABLE_IMU:
        print(f"  IMU Port: {IMU_PORT or 'Auto-detect'}")
    print(f"  Acquisition Duration: {ACQUISITION_DURATION if ACQUISITION_DURATION else 'Continuous'}")
    print(f"  Plot Window: {PLOT_WINDOW_SECONDS} seconds")
    
    if not ENABLE_EMG and not ENABLE_IMU:
        print("\n⚠️  WARNING: Both EMG and IMU are disabled!")
    
    print("="*70)


def main():
    """Main execution function."""
    print_header()
    
    # Validate configuration
    if not ENABLE_EMG and not ENABLE_IMU:
        print("\n❌ ERROR: Both EMG and IMU are disabled!")
        print("Please enable at least one device in the configuration.")
        return
    
    # Create buffers
    imu_buffer = TimestampedBuffer(maxlen=MAX_BUFFER_SIZE)
    emg_buffer = TimestampedBuffer(maxlen=MAX_BUFFER_SIZE)

    # Synchronized start event (ensures both devices begin acquisition together)
    start_event = threading.Event()
    
    # Create acquisition threads
    imu_thread = None
    emg_thread = None
    
    if ENABLE_IMU:
        imu_config = IMUConfig(
            imu_type=IMUType.BMI160_DUAL,
            port=IMU_PORT,
            baud=IMU_BAUD
        )
        imu_thread = IMUAcquisitionThread(imu_buffer, imu_config, start_event)
    
    if ENABLE_EMG:
        emg_thread = EMGAcquisitionThread(emg_buffer, start_event, EMG_CONNECTION_TYPE)
    
    # Start threads
    print("\n" + "="*70)
    print("STARTING ACQUISITION THREADS")
    print("="*70)
    
    try:
        # Start threads (they initialize, then wait for synchronized start_event)
        if emg_thread:
            emg_thread.start()
        if imu_thread:
            imu_thread.start()
        
        # Wait for threads to be ready (IMU includes calibration)
        print("\nWaiting for devices to be ready (IMU calibration happens here)...")
        t_ready0 = CLOCK()
        while True:
            if imu_thread and imu_thread.error:
                raise RuntimeError(f"IMU thread error: {imu_thread.error}")
            if emg_thread and emg_thread.error:
                raise RuntimeError(f"EMG thread error: {emg_thread.error}")

            imu_ready = (not ENABLE_IMU) or (imu_thread and imu_thread.ready_event.is_set())
            emg_ready = (not ENABLE_EMG) or (emg_thread and emg_thread.ready_event.is_set())
            if imu_ready and emg_ready:
                break
            if CLOCK() - t_ready0 > 120:
                raise TimeoutError("Timed out waiting for devices to be ready")
            time.sleep(0.1)
        
        # Check for errors
        if imu_thread and imu_thread.error:
            raise RuntimeError(f"IMU thread error: {imu_thread.error}")
        if emg_thread and emg_thread.error:
            raise RuntimeError(f"EMG thread error: {emg_thread.error}")
        
        # Clear buffers and trigger synchronized acquisition start
        imu_buffer.clear()
        emg_buffer.clear()
        start_event.set()

        print("\n" + "="*70)
        print("✓ ACQUISITION STARTED")
        print("="*70)
        print("\nPress Ctrl+C to stop acquisition")
        print("\nStarting visualization...")
        time.sleep(1)
        
        # Create and start visualization
        plotter = RealtimePlotter(imu_buffer, emg_buffer)
        
        # Start animation (blocking)
        plotter.start()
    
    except KeyboardInterrupt:
        print("\n\n[MAIN] Interrupted by user (Ctrl+C)")
    
    except Exception as e:
        print(f"\n\n[MAIN] ❌ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Stop threads
        print("\n" + "="*70)
        print("SHUTTING DOWN")
        print("="*70)
        
        print("\n[MAIN] Stopping acquisition threads...")
        if imu_thread:
            imu_thread.stop()
        if emg_thread:
            emg_thread.stop()
        
        # Wait for threads to finish
        print("[MAIN] Waiting for threads to finish...")
        if imu_thread:
            imu_thread.join(timeout=5)
        if emg_thread:
            emg_thread.join(timeout=5)
        
        # Print final statistics
        print("\n" + "="*70)
        print("ACQUISITION STATISTICS")
        print("="*70)
        
        if imu_thread:
            imu_stats = imu_thread.get_stats()
            print("\nIMU:")
            print(f"  Total samples: {imu_stats['samples']}")
            print(f"  Duration: {imu_stats['elapsed']:.2f} seconds")
            print(f"  Average rate: {imu_stats['rate']:.1f} Hz")
            print(f"  Buffer size: {imu_stats['buffer_size']}")
            if imu_thread.time_offset is not None:
                print(f"  Time offset: {imu_thread.time_offset:.6f} seconds")
                print(f"  Hardware start: {imu_thread.start_time_hardware} µs")
        
        if emg_thread:
            emg_stats = emg_thread.get_stats()
            print("\nEMG:")
            print(f"  Total samples: {emg_stats['samples']}")
            print(f"  Total chunks: {emg_stats['chunks']}")
            print(f"  Duration: {emg_stats['elapsed']:.2f} seconds")
            print(f"  Average rate: {emg_stats['rate']:.1f} Hz")
            print(f"  Buffer size: {emg_stats['buffer_size']}")
            if emg_thread.counter_channel_idx is not None:
                print(f"  Counter channel: {emg_thread.counter_channel_idx}")
                if emg_thread.last_counter_raw is not None:
                    print(f"  Last counter (raw): {emg_thread.last_counter_raw}")
                if emg_thread.last_counter_unwrapped is not None:
                    print(f"  Last counter (unwrapped): {emg_thread.last_counter_unwrapped}")
                if emg_thread.counter_modulus is not None:
                    print(f"  Counter modulus: {emg_thread.counter_modulus}")
                if emg_thread.latency_est is not None:
                    print(f"  Latency estimate: {emg_thread.latency_est*1000:.1f} ms")
        
        # Print synchronization info
        if imu_thread and emg_thread and imu_thread.start_time and emg_thread.start_time:
            print("\nSYNCHRONIZATION:")
            time_diff = abs(imu_thread.start_time - emg_thread.start_time)
            print(f"  Device start time difference: {time_diff*1000:.1f} ms")
            if time_diff < 0.1:
                print(f"  ✓ Devices well synchronized (< 100ms difference)")
            elif time_diff < 1.0:
                print(f"  ⚠️  Moderate sync offset (< 1s, acceptable for most applications)")
            else:
                print(f"  ⚠️  Large sync offset (> 1s, consider simultaneous start)")
            
            # Calculate timing statistics from buffers
            if len(imu_buffer) > 0 and len(emg_buffer) > 0:
                imu_data_all = imu_buffer.get_all()
                emg_data_all = emg_buffer.get_all()
                
                imu_times = [t for t, _ in imu_data_all]
                emg_times = [t for t, _ in emg_data_all]
                
                print(f"  IMU time range: {min(imu_times):.3f} - {max(imu_times):.3f} s")
                print(f"  EMG time range: {min(emg_times):.3f} - {max(emg_times):.3f} s")
                
                # Calculate overlap
                overlap_start = max(min(imu_times), min(emg_times))
                overlap_end = min(max(imu_times), max(emg_times))
                overlap_duration = max(0, overlap_end - overlap_start)
                print(f"  Synchronized data overlap: {overlap_duration:.2f} seconds")
        
        print("\n" + "="*70)
        print("✓ SHUTDOWN COMPLETE")
        print("="*70)


if __name__ == "__main__":
    main()
