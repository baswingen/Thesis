"""
Real-time Synchronized EMG + IMU Visualization
==============================================

Real-time plotting of synchronized EMG and IMU data streams using the
hardware-synchronized acquisition protocol from signal_acquisition_testing.py.

Features:
- Hardware-timestamp synchronized data streams
- Counter-based EMG timing for precision
- Live EMG differential pair display
- Live IMU orientation (roll, pitch, yaw)
- Synchronized time axis across all plots
- Signal quality and timing statistics

Usage:
    python -m src.synchronized_visualization [--dummy] [--duration SECONDS]
    
Example:
    # Test with dummy signals
    python -m src.synchronized_visualization --dummy
    
    # Real hardware for 60 seconds
    python -m src.synchronized_visualization --duration 60
"""

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec
import time
import threading
from collections import deque
from typing import Optional, List, Tuple, Any, Dict
from dataclasses import dataclass
import argparse
import sys
from pathlib import Path

# Add project root to path if needed
_project_root = Path(__file__).parent.parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.imu_acquisition import IMUDevice, IMUConfig, IMUReading
from src.emg_acquisition import EMGDevice

# Use monotonic clock for all timing
CLOCK = time.perf_counter

# =============================================================================
# CONFIGURATION
# =============================================================================

# EMG Configuration - Differential Pairs
EMG_DIFFERENTIAL_PAIRS = [
    (0, 1),  # Pair 1
    (2, 3),  # Pair 2
]

# Visualization Settings
PLOT_WINDOW_SECONDS = 5.0
PLOT_UPDATE_INTERVAL = 33  # ms (~30 FPS)
EMG_PLOT_SCALE = 'auto'
MAX_BUFFER_SIZE = 100000

# Hardware Settings
IMU_PORT = None  # None = auto-detect
IMU_BAUD = 230400
IMU_CALIBRATION_SAMPLES = 200
EMG_CONNECTION_TYPE = 'usb'
EMG_SAMPLE_RATE = 2048

VERBOSE = False

# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class IMUData:
    """Container for IMU data with synchronized timestamp."""
    timestamp: float
    reading: IMUReading
    hardware_time: float

@dataclass
class EMGData:
    """Container for EMG differential pair data with synchronized timestamp."""
    timestamp: float
    pairs: np.ndarray
    sample_count: int
    counter: Optional[int] = None

# =============================================================================
# THREAD-SAFE BUFFER
# =============================================================================

class TimestampedBuffer:
    """Thread-safe buffer for timestamped data."""
    
    def __init__(self, maxlen: int = 10000):
        self.buffer = deque(maxlen=maxlen)
        self.lock = threading.Lock()
        self.total_count = 0
    
    def append(self, timestamp: float, data: Any):
        with self.lock:
            self.buffer.append((timestamp, data))
            self.total_count += 1
    
    def get_recent(self, window_seconds: float) -> List[Tuple[float, Any]]:
        with self.lock:
            if not self.buffer:
                return []
            t_now = CLOCK()
            t_cutoff = t_now - window_seconds
            result = []
            for t, d in reversed(self.buffer):
                if t >= t_cutoff:
                    result.append((t, d))
                else:
                    break
            result.reverse()
            return result
    
    def get_all(self) -> List[Tuple[float, Any]]:
        with self.lock:
            return list(self.buffer)
    
    def clear(self):
        with self.lock:
            self.buffer.clear()
    
    def __len__(self) -> int:
        with self.lock:
            return len(self.buffer)

# =============================================================================
# ACQUISITION THREADS (from signal_acquisition_testing.py protocol)
# =============================================================================

class IMUAcquisitionThread(threading.Thread):
    """IMU acquisition with hardware timestamp synchronization."""
    
    def __init__(self, buffer: TimestampedBuffer, config: IMUConfig, start_event: threading.Event):
        super().__init__(name="IMU-Thread", daemon=True)
        self.buffer = buffer
        self.config = config
        self.start_event = start_event
        self.ready_event = threading.Event()
        self.device: Optional[IMUDevice] = None
        self.running = False
        self.error = None
        self.sample_count = 0
        self.start_time = None
        self.time_offset = None
        self._last_t_us = None
    
    def run(self):
        try:
            print("\n[IMU] Initializing...")
            self.device = IMUDevice(config=self.config)
            self.device.connect()
            
            print("[IMU] Calibrating (place flat and still)...")
            time.sleep(2)
            self.device.calibrate(samples=IMU_CALIBRATION_SAMPLES)
            print("[IMU] ✓ Calibration complete")
            
            self.ready_event.set()
            self.start_event.wait()
            self.running = True
            self.start_time = CLOCK()
            
            # Flush serial buffer
            try:
                if self.device and getattr(self.device, "ser", None):
                    self.device.ser.reset_input_buffer()
                    time.sleep(0.01)
            except:
                pass
            
            first_sample = True
            
            for reading in self.device.read_stream():
                if not self.running:
                    break
                
                receive_time = CLOCK()
                
                # Detect Arduino reset
                if self._last_t_us and reading.t_us < self._last_t_us and \
                   (self._last_t_us - reading.t_us) > 10000:
                    print("[IMU] ⚠ Time reset detected, re-syncing...")
                    first_sample = True
                
                self._last_t_us = reading.t_us
                
                if first_sample:
                    self.time_offset = receive_time - (reading.t_us / 1_000_000.0)
                    first_sample = False
                
                hw_time = reading.t_us / 1_000_000.0
                sync_timestamp = hw_time + self.time_offset
                
                data = IMUData(timestamp=sync_timestamp, reading=reading, hardware_time=hw_time)
                self.buffer.append(sync_timestamp, data)
                self.sample_count += 1
                
        except Exception as e:
            self.error = e
            print(f"[IMU] Error: {e}")
        finally:
            if self.device:
                self.device.close()
    
    def stop(self):
        self.running = False


class EMGAcquisitionThread(threading.Thread):
    """EMG acquisition with counter-based timing."""
    
    def __init__(self, buffer: TimestampedBuffer, start_event: threading.Event):
        super().__init__(name="EMG-Thread", daemon=True)
        self.buffer = buffer
        self.start_event = start_event
        self.ready_event = threading.Event()
        self.device: Optional[EMGDevice] = None
        self.running = False
        self.error = None
        self.sample_count = 0
        self.start_time = None
        self.counter_channel_idx = None
        self.use_counter_timing = False
        self.counter_time_offset = None
        self.counter_gain = None
        self.counter_offset = None
        self.last_counter_raw = None
        self.counter_unwrap_offset = 0
        self.counter_modulus = None
    
    def _counter_to_int(self, x: float) -> Optional[int]:
        try:
            if not np.isfinite(x):
                return None
            if self.counter_gain and self.counter_offset:
                g, o = float(self.counter_gain), float(self.counter_offset)
                if abs(g) > 0:
                    x = (x / g) - o
            return int(round(x))
        except:
            return None
    
    def _unwrap_counter(self, raw: int) -> int:
        if self.last_counter_raw is None:
            self.last_counter_raw = raw
            return raw
        
        if self.counter_modulus is None:
            if raw > 65535 or self.last_counter_raw > 65535:
                self.counter_modulus = 2**32
            elif raw < self.last_counter_raw and (self.last_counter_raw - raw) > 1000:
                self.counter_modulus = 65536
        
        if self.counter_modulus == 65536 and raw < self.last_counter_raw:
            self.counter_unwrap_offset += 65536
        
        self.last_counter_raw = raw
        return raw + self.counter_unwrap_offset
    
    def run(self):
        try:
            print("\n[EMG] Initializing...")
            self.device = EMGDevice(connection_type=EMG_CONNECTION_TYPE)
            info = self.device.connect()
            print(f"[EMG] Connected: {info['name']}, {info['sample_rate']} Hz")
            
            if EMG_SAMPLE_RATE:
                self.device.configure_acquisition(sample_rate=EMG_SAMPLE_RATE)
            
            # Find COUNTER channel
            for i, ch in enumerate(self.device.channels):
                if 'COUNTER' in ch.get_channel_name().upper():
                    self.counter_channel_idx = i
                    self.use_counter_timing = True
                    print(f"[EMG] ✓ Counter channel found at index {i}")
                    try:
                        self.counter_gain = ch.get_channel_unit_gain()
                        self.counter_offset = ch.get_channel_unit_offset()
                    except:
                        pass
                    break
            
            self.ready_event.set()
            self.start_event.wait()
            self.running = True
            
            self.device.start_acquisition()
            self.start_time = CLOCK()
            
            while self.running:
                samples = self.device.get_samples(blocking=False)
                
                if samples is not None and len(samples) > 0:
                    receive_time = CLOCK()
                    n_samples = samples.shape[0]
                    sample_period = 1.0 / self.device.sample_rate
                    
                    counter_raw = None
                    counter_unwrapped = None
                    
                    if self.counter_channel_idx is not None:
                        counter_raw = self._counter_to_int(samples[-1, self.counter_channel_idx])
                        if counter_raw is not None:
                            counter_unwrapped = self._unwrap_counter(counter_raw)
                            
                            if self.counter_time_offset is None:
                                self.counter_time_offset = receive_time - (counter_unwrapped * sample_period)
                    
                    # Calculate timestamp
                    if self.use_counter_timing and counter_unwrapped is not None and self.counter_time_offset:
                        last_sample_time = (counter_unwrapped * sample_period) + self.counter_time_offset
                        mid_offset = ((n_samples - 1) / 2.0) * sample_period
                        chunk_timestamp = last_sample_time - mid_offset
                        
                        # Adapt offset slowly
                        new_off = receive_time - (counter_unwrapped * sample_period)
                        if np.isfinite(new_off):
                            self.counter_time_offset = 0.995 * self.counter_time_offset + 0.005 * new_off
                    else:
                        chunk_duration = n_samples * sample_period
                        chunk_timestamp = receive_time - (chunk_duration / 2.0)
                    
                    self.sample_count += n_samples
                    
                    # Calculate differential pairs
                    pair_values = []
                    for pos_ch, neg_ch in EMG_DIFFERENTIAL_PAIRS:
                        diff = samples[:, pos_ch] - samples[:, neg_ch]
                        pair_values.append(np.nanmean(diff))
                    
                    data = EMGData(
                        timestamp=chunk_timestamp,
                        pairs=np.array(pair_values),
                        sample_count=n_samples,
                        counter=counter_raw
                    )
                    self.buffer.append(chunk_timestamp, data)
                
                time.sleep(0.01)
        
        except Exception as e:
            self.error = e
            print(f"[EMG] Error: {e}")
        finally:
            if self.device:
                try:
                    self.device.stop_acquisition()
                    self.device.disconnect()
                except:
                    pass
    
    def stop(self):
        self.running = False

# =============================================================================
# REAL-TIME VISUALIZATION
# =============================================================================

class RealtimePlotter:
    """Real-time visualization of synchronized EMG + IMU data."""
    
    def __init__(self, imu_buffer: TimestampedBuffer, emg_buffer: TimestampedBuffer):
        self.imu_buffer = imu_buffer
        self.emg_buffer = emg_buffer
        self.start_time = None
        self.frame_count = 0
        self._setup_figure()
        self.anim = None
    
    def _setup_figure(self):
        self.fig = plt.figure(figsize=(14, 10))
        self.fig.suptitle('Synchronized IMU-EMG Visualization', fontsize=14, fontweight='bold')
        
        gs = GridSpec(3, 1, figure=self.fig, hspace=0.3)
        
        # EMG subplot
        self.ax_emg = self.fig.add_subplot(gs[0, 0])
        self.ax_emg.set_title('EMG Differential Pairs', fontweight='bold')
        self.ax_emg.set_xlabel('Time (s)')
        self.ax_emg.set_ylabel('Amplitude (µV)')
        self.ax_emg.grid(True, alpha=0.3)
        
        self.emg_lines = []
        colors = plt.cm.tab10(np.linspace(0, 1, len(EMG_DIFFERENTIAL_PAIRS)))
        for i, (pos, neg) in enumerate(EMG_DIFFERENTIAL_PAIRS):
            line, = self.ax_emg.plot([], [], '-', linewidth=1.5, color=colors[i],
                                     label=f'Pair {i+1} (Ch{pos}-Ch{neg})')
            self.emg_lines.append(line)
        self.ax_emg.legend(loc='upper right')
        
        # IMU1 subplot
        self.ax_imu1 = self.fig.add_subplot(gs[1, 0])
        self.ax_imu1.set_title('IMU1 Orientation', fontweight='bold')
        self.ax_imu1.set_xlabel('Time (s)')
        self.ax_imu1.set_ylabel('Angle (degrees)')
        self.ax_imu1.grid(True, alpha=0.3)
        
        self.imu1_roll, = self.ax_imu1.plot([], [], 'r-', linewidth=1.5, label='Roll')
        self.imu1_pitch, = self.ax_imu1.plot([], [], 'g-', linewidth=1.5, label='Pitch')
        self.imu1_yaw, = self.ax_imu1.plot([], [], 'b-', linewidth=1.5, label='Yaw')
        self.ax_imu1.legend(loc='upper right')
        
        # IMU2 subplot
        self.ax_imu2 = self.fig.add_subplot(gs[2, 0])
        self.ax_imu2.set_title('IMU2 Orientation', fontweight='bold')
        self.ax_imu2.set_xlabel('Time (s)')
        self.ax_imu2.set_ylabel('Angle (degrees)')
        self.ax_imu2.grid(True, alpha=0.3)
        
        self.imu2_roll, = self.ax_imu2.plot([], [], 'r-', linewidth=1.5, label='Roll')
        self.imu2_pitch, = self.ax_imu2.plot([], [], 'g-', linewidth=1.5, label='Pitch')
        self.imu2_yaw, = self.ax_imu2.plot([], [], 'b-', linewidth=1.5, label='Yaw')
        self.ax_imu2.legend(loc='upper right')
        
        plt.tight_layout()
    
    def update(self, frame):
        self.frame_count += 1
        
        imu_data = self.imu_buffer.get_recent(PLOT_WINDOW_SECONDS)
        emg_data = self.emg_buffer.get_recent(PLOT_WINDOW_SECONDS)
        
        if self.start_time is None:
            times = []
            if imu_data:
                times.append(imu_data[0][1].timestamp)
            if emg_data:
                times.append(emg_data[0][1].timestamp)
            if times:
                self.start_time = min(times)
        
        latest_time = 0.0
        
        # Update EMG
        if emg_data:
            emg_times = np.array([d.timestamp for _, d in emg_data])
            emg_times_rel = emg_times - self.start_time if self.start_time else emg_times
            latest_time = max(latest_time, emg_times_rel[-1] if len(emg_times_rel) > 0 else 0)
            
            for i in range(len(EMG_DIFFERENTIAL_PAIRS)):
                vals = np.array([d.pairs[i] for _, d in emg_data])
                self.emg_lines[i].set_data(emg_times_rel, vals)
            
            if EMG_PLOT_SCALE == 'auto':
                all_vals = np.concatenate([d.pairs for _, d in emg_data])
                y_min, y_max = np.nanmin(all_vals), np.nanmax(all_vals)
                margin = (y_max - y_min) * 0.1 if y_max > y_min else 10
                self.ax_emg.set_ylim(y_min - margin, y_max + margin)
        
        # Update IMU
        if imu_data:
            imu_times = np.array([d.timestamp for _, d in imu_data])
            imu_times_rel = imu_times - self.start_time if self.start_time else imu_times
            latest_time = max(latest_time, imu_times_rel[-1] if len(imu_times_rel) > 0 else 0)
            
            # IMU1
            rolls1 = np.array([d.reading.euler1[0] if d.reading.euler1 else np.nan for _, d in imu_data])
            pitches1 = np.array([d.reading.euler1[1] if d.reading.euler1 else np.nan for _, d in imu_data])
            yaws1 = np.array([d.reading.euler1[2] if d.reading.euler1 else np.nan for _, d in imu_data])
            
            self.imu1_roll.set_data(imu_times_rel, rolls1)
            self.imu1_pitch.set_data(imu_times_rel, pitches1)
            self.imu1_yaw.set_data(imu_times_rel, yaws1)
            
            self.ax_imu1.relim()
            self.ax_imu1.autoscale_view(scalex=False)
            
            # IMU2
            rolls2 = np.array([d.reading.euler2[0] if d.reading.euler2 else np.nan for _, d in imu_data])
            pitches2 = np.array([d.reading.euler2[1] if d.reading.euler2 else np.nan for _, d in imu_data])
            yaws2 = np.array([d.reading.euler2[2] if d.reading.euler2 else np.nan for _, d in imu_data])
            
            self.imu2_roll.set_data(imu_times_rel, rolls2)
            self.imu2_pitch.set_data(imu_times_rel, pitches2)
            self.imu2_yaw.set_data(imu_times_rel, yaws2)
            
            self.ax_imu2.relim()
            self.ax_imu2.autoscale_view(scalex=False)
        
        # Synchronized X-axis
        if latest_time > 0:
            x_min = max(0, latest_time - PLOT_WINDOW_SECONDS)
            x_max = latest_time + 0.1
            self.ax_emg.set_xlim(x_min, x_max)
            self.ax_imu1.set_xlim(x_min, x_max)
            self.ax_imu2.set_xlim(x_min, x_max)
        
        return []
    
    def start(self):
        self.anim = FuncAnimation(self.fig, self.update, interval=PLOT_UPDATE_INTERVAL,
                                 blit=False, cache_frame_data=False)
        plt.show()

# =============================================================================
# MAIN RUNNER
# =============================================================================

def run_visualization(use_dummy: bool = False, duration: Optional[float] = None):
    """Run synchronized EMG + IMU visualization."""
    
    print("\n" + "="*70)
    print("SYNCHRONIZED EMG + IMU VISUALIZATION")
    print("="*70)
    print(f"Mode: {'DUMMY (simulated)' if use_dummy else 'REAL HARDWARE'}")
    print(f"Duration: {duration if duration else 'Continuous'} seconds")
    print(f"EMG Pairs: {EMG_DIFFERENTIAL_PAIRS}")
    print("="*70)
    
    if use_dummy:
        from .dummy_acquisition import DummyAcquisition
        print("\n⚠ Dummy mode not yet integrated with new protocol")
        print("Please use real hardware or wait for dummy integration")
        return
    
    imu_buffer = TimestampedBuffer(maxlen=MAX_BUFFER_SIZE)
    emg_buffer = TimestampedBuffer(maxlen=MAX_BUFFER_SIZE)
    start_event = threading.Event()
    
    imu_config = IMUConfig(port=IMU_PORT, baud=IMU_BAUD)
    imu_thread = IMUAcquisitionThread(imu_buffer, imu_config, start_event)
    emg_thread = EMGAcquisitionThread(emg_buffer, start_event)
    
    try:
        imu_thread.start()
        emg_thread.start()
        
        print("\nWaiting for devices...")
        t0 = CLOCK()
        while not (imu_thread.ready_event.is_set() and emg_thread.ready_event.is_set()):
            if CLOCK() - t0 > 60:
                raise TimeoutError("Device initialization timeout")
            time.sleep(0.1)
        
        if imu_thread.error:
            raise RuntimeError(f"IMU error: {imu_thread.error}")
        if emg_thread.error:
            raise RuntimeError(f"EMG error: {emg_thread.error}")
        
        imu_buffer.clear()
        emg_buffer.clear()
        start_event.set()
        
        print("\n✓ Acquisition started")
        print("Starting visualization...\n")
        time.sleep(1)
        
        plotter = RealtimePlotter(imu_buffer, emg_buffer)
        plotter.start()
    
    except KeyboardInterrupt:
        print("\n⚠ Interrupted by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nStopping...")
        imu_thread.stop()
        emg_thread.stop()
        imu_thread.join(timeout=5)
        emg_thread.join(timeout=5)
        
        print("\n" + "="*70)
        print("STATISTICS")
        print("="*70)
        print(f"IMU samples: {imu_thread.sample_count}")
        print(f"EMG samples: {emg_thread.sample_count}")
        print("="*70)

def main():
    parser = argparse.ArgumentParser(description='Synchronized EMG+IMU visualization')
    parser.add_argument('--dummy', action='store_true', help='Use dummy signals')
    parser.add_argument('--duration', type=float, help='Duration in seconds')
    
    args = parser.parse_args()
    run_visualization(use_dummy=args.dummy, duration=args.duration)

if __name__ == '__main__':
    main()
