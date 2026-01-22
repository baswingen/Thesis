"""
Synchronized IMU-EMG Signal Acquisition
========================================

Real-time synchronized acquisition of dual BMI160 IMU data (~200Hz) and 
TMSi EMG data (~2000Hz+) with timestamp-based alignment and live visualization.

This script combines:
- Dual IMU acquisition via Arduino/serial (src.IMUDevice)
- Multi-channel EMG acquisition via TMSi USB (src.EMGDevice)
- Thread-safe data buffering with timestamp synchronization
- Real-time matplotlib visualization

Hardware Requirements:
- Arduino with dual BMI160 IMUs (serial connection)
- TMSi Porti7/REFA device (USB connection)
- EMG electrodes configured as differential pairs

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
    EMGDevice
)

# =============================================================================
# CONFIGURATION
# =============================================================================

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
    (0, 1),  # Pair 1: Channel 0 (+) - Channel 1 (-)
    # (2, 3),  # Pair 2: Channel 2 (+) - Channel 3 (-) (uncomment if needed)
]

EMG_CONNECTION_TYPE = 'usb'  # 'usb', 'bluetooth', 'network', 'wifi'
EMG_SAMPLE_RATE = None  # None = use device default (usually 2048 or 4096 Hz)

# IMU Configuration
# -----------------
IMU_PORT = None  # None = auto-detect, or specify like "/dev/cu.usbmodem..." or "COM3"
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
VERBOSE = False  # Print detailed acquisition info
SHOW_TIMING_STATS = True  # Display timing statistics

# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class IMUData:
    """Container for IMU data with timestamp."""
    timestamp: float  # Python time.time()
    reading: IMUReading  # IMUReading object from src.imu_acquisition

@dataclass
class EMGData:
    """Container for EMG differential pair data with timestamp."""
    timestamp: float  # Python time.time()
    pairs: np.ndarray  # Differential pair values [pair1, pair2, ...]
    sample_count: int  # Number of samples in this chunk


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
            
            t_now = time.time()
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
    """Thread for continuous IMU data acquisition."""
    
    def __init__(self, buffer: TimestampedBuffer, config: IMUConfig):
        """
        Initialize IMU acquisition thread.
        
        Args:
            buffer: Shared buffer for storing data
            config: IMU configuration
        """
        super().__init__(name="IMU-Thread", daemon=True)
        self.buffer = buffer
        self.config = config
        self.device: Optional[IMUDevice] = None
        self.running = False
        self.error = None
        self.sample_count = 0
        self.start_time = None
    
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
            
            # Start acquisition
            self.running = True
            self.start_time = time.time()
            print("[IMU] Starting data acquisition...")
            
            # Continuous reading loop
            for reading in self.device.read_stream():
                if not self.running:
                    break
                
                # Store with Python timestamp
                timestamp = time.time()
                self.buffer.append(timestamp, reading)
                self.sample_count += 1
                
                if VERBOSE and self.sample_count % 100 == 0:
                    print(f"[IMU] Samples: {self.sample_count}")
        
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
            elapsed = time.time() - self.start_time
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
    """Thread for continuous EMG data acquisition with differential pair calculation."""
    
    def __init__(self, buffer: TimestampedBuffer, connection_type: str = 'usb'):
        """
        Initialize EMG acquisition thread.
        
        Args:
            buffer: Shared buffer for storing data
            connection_type: Connection type for EMG device
        """
        super().__init__(name="EMG-Thread", daemon=True)
        self.buffer = buffer
        self.connection_type = connection_type
        self.device: Optional[EMGDevice] = None
        self.running = False
        self.error = None
        self.sample_count = 0
        self.chunk_count = 0
        self.start_time = None
    
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
            
            # Start acquisition
            self.device.start_acquisition()
            self.running = True
            self.start_time = time.time()
            print("[EMG] Starting data acquisition...")
            
            # Continuous reading loop
            while self.running:
                samples = self.device.get_samples(blocking=False)
                
                if samples is not None and len(samples) > 0:
                    timestamp = time.time()
                    n_samples = samples.shape[0]
                    self.sample_count += n_samples
                    self.chunk_count += 1
                    
                    # Calculate differential pairs
                    # Average across all samples in this chunk for smoother display
                    pair_values = []
                    for pos_ch, neg_ch in EMG_DIFFERENTIAL_PAIRS:
                        diff = samples[:, pos_ch] - samples[:, neg_ch]
                        # Store mean value of chunk
                        pair_values.append(np.nanmean(diff))
                    
                    # Store in buffer
                    emg_data = EMGData(
                        timestamp=timestamp,
                        pairs=np.array(pair_values),
                        sample_count=n_samples
                    )
                    self.buffer.append(timestamp, emg_data)
                    
                    if VERBOSE and self.chunk_count % 50 == 0:
                        print(f"[EMG] Chunks: {self.chunk_count}, Samples: {self.sample_count}")
                
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
            elapsed = time.time() - self.start_time
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
        self.start_time = time.time()
        
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
        """Update plot with new data."""
        self.frame_count += 1
        
        # Get recent data
        imu_data = self.imu_buffer.get_recent(PLOT_WINDOW_SECONDS)
        emg_data = self.emg_buffer.get_recent(PLOT_WINDOW_SECONDS)
        
        # Update EMG plot
        if ENABLE_EMG and emg_data and self.ax_emg:
            emg_times = np.array([t for t, _ in emg_data])
            # Relative time (start from 0)
            emg_times_rel = emg_times - self.start_time
            
            # Extract pair values
            for pair_idx in range(len(EMG_DIFFERENTIAL_PAIRS)):
                pair_values = np.array([d.pairs[pair_idx] for _, d in emg_data])
                self.emg_lines[pair_idx].set_data(emg_times_rel, pair_values)
            
            # Update axis limits
            self.ax_emg.set_xlim(max(0, emg_times_rel[-1] - PLOT_WINDOW_SECONDS), 
                                 emg_times_rel[-1] + 0.1)
            
            if EMG_PLOT_SCALE == 'auto':
                all_values = np.concatenate([d.pairs for _, d in emg_data])
                y_min, y_max = np.nanmin(all_values), np.nanmax(all_values)
                y_range = y_max - y_min
                margin = y_range * 0.1 if y_range > 0 else 10
                self.ax_emg.set_ylim(y_min - margin, y_max + margin)
            else:
                self.ax_emg.set_ylim(EMG_PLOT_SCALE)
        
        # Update IMU plots
        if ENABLE_IMU and imu_data:
            imu_times = np.array([t for t, _ in imu_data])
            imu_times_rel = imu_times - self.start_time
            
            if self.ax_imu1:
                rolls = np.array([d.euler1[0] if d.euler1 else np.nan for _, d in imu_data])
                pitches = np.array([d.euler1[1] if d.euler1 else np.nan for _, d in imu_data])
                yaws = np.array([d.euler1[2] if d.euler1 else np.nan for _, d in imu_data])
                
                self.imu1_roll_line.set_data(imu_times_rel, rolls)
                self.imu1_pitch_line.set_data(imu_times_rel, pitches)
                self.imu1_yaw_line.set_data(imu_times_rel, yaws)
                
                self.ax_imu1.set_xlim(max(0, imu_times_rel[-1] - PLOT_WINDOW_SECONDS), 
                                     imu_times_rel[-1] + 0.1)
                self.ax_imu1.relim()
                self.ax_imu1.autoscale_view(scalex=False)
            
            if self.ax_imu2:
                rolls = np.array([d.euler2[0] if d.euler2 else np.nan for _, d in imu_data])
                pitches = np.array([d.euler2[1] if d.euler2 else np.nan for _, d in imu_data])
                yaws = np.array([d.euler2[2] if d.euler2 else np.nan for _, d in imu_data])
                
                self.imu2_roll_line.set_data(imu_times_rel, rolls)
                self.imu2_pitch_line.set_data(imu_times_rel, pitches)
                self.imu2_yaw_line.set_data(imu_times_rel, yaws)
                
                self.ax_imu2.set_xlim(max(0, imu_times_rel[-1] - PLOT_WINDOW_SECONDS), 
                                     imu_times_rel[-1] + 0.1)
                self.ax_imu2.relim()
                self.ax_imu2.autoscale_view(scalex=False)
        
        # Update title with stats
        if SHOW_TIMING_STATS and self.frame_count % 10 == 0:
            elapsed = time.time() - self.start_time
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
    
    # Create acquisition threads
    imu_thread = None
    emg_thread = None
    
    if ENABLE_IMU:
        imu_config = IMUConfig(
            port=IMU_PORT,
            baud=IMU_BAUD
        )
        imu_thread = IMUAcquisitionThread(imu_buffer, imu_config)
    
    if ENABLE_EMG:
        emg_thread = EMGAcquisitionThread(emg_buffer, EMG_CONNECTION_TYPE)
    
    # Start threads
    print("\n" + "="*70)
    print("STARTING ACQUISITION THREADS")
    print("="*70)
    
    try:
        # Start EMG first (faster startup)
        if emg_thread:
            emg_thread.start()
            time.sleep(1)  # Give EMG time to initialize
        
        # Start IMU (includes calibration)
        if imu_thread:
            imu_thread.start()
        
        # Wait for threads to initialize
        print("\nWaiting for threads to initialize...")
        time.sleep(2)
        
        # Check for errors
        if imu_thread and imu_thread.error:
            raise RuntimeError(f"IMU thread error: {imu_thread.error}")
        if emg_thread and emg_thread.error:
            raise RuntimeError(f"EMG thread error: {emg_thread.error}")
        
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
        
        if emg_thread:
            emg_stats = emg_thread.get_stats()
            print("\nEMG:")
            print(f"  Total samples: {emg_stats['samples']}")
            print(f"  Total chunks: {emg_stats['chunks']}")
            print(f"  Duration: {emg_stats['elapsed']:.2f} seconds")
            print(f"  Average rate: {emg_stats['rate']:.1f} Hz")
            print(f"  Buffer size: {emg_stats['buffer_size']}")
        
        print("\n" + "="*70)
        print("✓ SHUTDOWN COMPLETE")
        print("="*70)


if __name__ == "__main__":
    main()
