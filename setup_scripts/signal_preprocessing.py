"""
Real-Time Signal Preprocessing for EMG and IMU
===============================================

Preprocesses synchronized EMG and IMU signals for machine learning:
- EMG: Bandpass filtering, notch filtering, rectification, smoothing, MVC normalization
- IMU: Pass-through (already clean from acquisition module)

Features:
- Real-time preprocessing using stateful filters
- MVC (Maximum Voluntary Contraction) calibration and normalization
- Time-domain visualization comparing raw vs preprocessed signals
- Integration with SynchronizedAcquisition for hardware-synchronized data

Author: Generated for Master Thesis
Date: January 2026
"""

import time
import numpy as np
import threading
from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple, Any
import sys
from pathlib import Path

# Add project root to path
_project_root = Path(__file__).parent.parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

# Visualization
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec

# Import acquisition and processing modules
from src.synchronized_acquisition import (
    SynchronizedAcquisition,
    SyncConfig,
    EMGSyncConfig,
    IMUSyncConfig,
    EMGSelection,
    IMUSample,
    EMGChunk
)
from src.emg_processing import (
    EMGPreprocessor,
    calibrate_mvc_from_data
)
from src.imu_acquisition import IMUConfig, IMUType

# =============================================================================
# CONFIGURATION
# =============================================================================

# EMG Configuration
EMG_SAMPLE_RATE = 1024  # Hz
EMG_DIFFERENTIAL_PAIRS = [(1, 2)]  # Channel pairs to acquire

# EMG Preprocessing Parameters
BANDPASS_LOW = 20.0      # Hz - Remove DC offset and motion artifacts
BANDPASS_HIGH = 450.0    # Hz - Remove high-frequency noise
NOTCH_FREQ = 50.0        # Hz - Powerline interference (50 Hz EU, 60 Hz US)
ENVELOPE_CUTOFF = 10.0   # Hz - Smooth muscle activation envelope

# MVC Calibration
MVC_CALIBRATION_DURATION = 5.0  # seconds - Time for user to perform max contraction
MVC_PERCENTILE = 95.0           # Use 95th percentile to avoid outliers

# Visualization
PLOT_WINDOW_SECONDS = 10.0   # Time window to display
PLOT_UPDATE_INTERVAL = 50    # milliseconds (~20 FPS)

# Acquisition Settings
ACQUISITION_DURATION = 60.0  # seconds (None = run indefinitely)

# =============================================================================
# MVC CALIBRATION ROUTINE (using native module)
# =============================================================================
# Note: EMGPreprocessor is now imported from src.emg_processing

def calibrate_mvc(sync_acq: SynchronizedAcquisition, 
                  duration: float = 5.0,
                  percentile: float = 95.0) -> float:
    """
    Perform MVC (Maximum Voluntary Contraction) calibration.
    
    Prompts user to perform maximum contraction and records peak envelope value.
    
    Args:
        sync_acq: SynchronizedAcquisition instance (must be started)
        duration: Calibration duration in seconds
        percentile: Percentile to use for MVC (95th avoids outliers)
        
    Returns:
        MVC value (peak envelope)
    """
    print("\n" + "="*70)
    print("MVC CALIBRATION")
    print("="*70)
    print(f"\nInstructions:")
    print(f"  1. Get ready to perform MAXIMUM muscle contraction")
    print(f"  2. Press ENTER to start calibration")
    print(f"  3. Contract as hard as possible for {duration} seconds")
    print(f"  4. Relax after the beep")
    
    input("\nPress ENTER when ready...")
    
    # Create temporary preprocessor without MVC normalization
    temp_preprocessor = EMGPreprocessor(
        fs=EMG_SAMPLE_RATE,
        mvc_value=None,  # No normalization during calibration
        bandpass_low=BANDPASS_LOW,
        bandpass_high=BANDPASS_HIGH,
        notch_freq=NOTCH_FREQ,
        envelope_cutoff=ENVELOPE_CUTOFF
    )
    
    print(f"\nCalibrating... CONTRACT NOW! ({duration} seconds)")
    
    envelope_values = []
    start_time = time.perf_counter()
    
    # Collect data for calibration duration
    while time.perf_counter() - start_time < duration:
        # Get recent EMG chunks
        emg_chunks = sync_acq.emg_buffer.get_recent(
            now_t=time.perf_counter(),
            window_s=0.5  # Look at last 500ms
        )
        
        if emg_chunks:
            for t, chunk in emg_chunks:
                # Process first differential pair
                if 'pairs' in chunk.data:
                    raw_emg = chunk.data['pairs'][:, 0]  # First pair
                    result = temp_preprocessor.process(raw_emg, return_all_stages=True)
                    envelope_values.append(result['envelope'])
        
        time.sleep(0.05)  # 50ms polling
    
    print("\nCalibration complete! You can relax now.")
    print("\a")  # Beep
    
    # Calculate MVC from collected envelopes
    if envelope_values:
        all_envelopes = np.concatenate(envelope_values)
        mvc_value = np.percentile(all_envelopes, percentile)
        print(f"\nMVC Results:")
        print(f"  - Samples collected: {len(all_envelopes)}")
        print(f"  - MVC value ({percentile}th percentile): {mvc_value:.4f}")
        print(f"  - Peak value: {np.max(all_envelopes):.4f}")
        print(f"  - Mean value: {np.mean(all_envelopes):.4f}")
        return mvc_value
    else:
        print("\nWARNING: No data collected during calibration!")
        print("Using default MVC value of 1.0")
        return 1.0


# =============================================================================
# REAL-TIME VISUALIZATION
# =============================================================================

class PreprocessingVisualizer:
    """
    Real-time visualization comparing raw vs preprocessed signals.
    """
    
    def __init__(self, 
                 sync_acq: SynchronizedAcquisition,
                 preprocessor: EMGPreprocessor,
                 window_seconds: float = 10.0):
        """
        Initialize visualizer.
        
        Args:
            sync_acq: SynchronizedAcquisition instance
            preprocessor: EMGPreprocessor instance
            window_seconds: Time window to display (seconds)
        """
        self.sync_acq = sync_acq
        self.preprocessor = preprocessor
        self.window_seconds = window_seconds
        
        # Data buffers for plotting
        self.raw_times = []
        self.raw_values = []
        self.normalized_times = []
        self.normalized_values = []
        self.imu_times = []
        self.imu_rolls = []
        self.imu_pitches = []
        self.imu_yaws = []
        
        # Setup figure
        self.fig = plt.figure(figsize=(14, 10))
        self.gs = GridSpec(3, 1, figure=self.fig, hspace=0.3)
        
        # Create subplots
        self.ax_raw = self.fig.add_subplot(self.gs[0])
        self.ax_preprocessed = self.fig.add_subplot(self.gs[1])
        self.ax_imu = self.fig.add_subplot(self.gs[2])
        
        # Initialize lines
        self.line_raw, = self.ax_raw.plot([], [], 'b-', linewidth=0.5, label='Raw EMG')
        self.line_normalized, = self.ax_preprocessed.plot([], [], 'r-', linewidth=1.0, label='Normalized EMG')
        self.line_imu_roll, = self.ax_imu.plot([], [], 'r-', linewidth=1.5, label='Roll')
        self.line_imu_pitch, = self.ax_imu.plot([], [], 'g-', linewidth=1.5, label='Pitch')
        self.line_imu_yaw, = self.ax_imu.plot([], [], 'b-', linewidth=1.5, label='Yaw')
        
        # Configure axes
        self.ax_raw.set_ylabel('Raw EMG (µV)')
        self.ax_raw.set_title('Raw EMG Signal', fontsize=12, fontweight='bold')
        self.ax_raw.grid(True, alpha=0.3)
        self.ax_raw.legend(loc='upper right')
        
        self.ax_preprocessed.set_ylabel('Normalized EMG')
        self.ax_preprocessed.set_title('Preprocessed EMG (Filtered → Rectified → Smoothed → Normalized)', 
                                      fontsize=12, fontweight='bold')
        self.ax_preprocessed.grid(True, alpha=0.3)
        self.ax_preprocessed.legend(loc='upper right')
        self.ax_preprocessed.axhline(y=1.0, color='gray', linestyle='--', linewidth=0.5, label='MVC')
        
        self.ax_imu.set_xlabel('Time (s)')
        self.ax_imu.set_ylabel('Angle (degrees)')
        self.ax_imu.set_title('IMU1 Orientation (Pass-through)', fontsize=12, fontweight='bold')
        self.ax_imu.grid(True, alpha=0.3)
        self.ax_imu.legend(loc='upper right')
        
        self.fig.suptitle('Real-Time Signal Preprocessing', fontsize=14, fontweight='bold')
        
    def update(self, frame):
        """Update plot with new data."""
        now = time.perf_counter()
        
        # Get recent EMG chunks
        emg_chunks = self.sync_acq.emg_buffer.get_recent(now, self.window_seconds)
        
        if emg_chunks:
            # Clear old data outside window
            cutoff_time = now - self.window_seconds
            
            # Process EMG chunks
            for t, chunk in emg_chunks[-10:]:  # Process last 10 chunks
                if 'pairs' in chunk.data:
                    raw_emg = chunk.data['pairs'][:, 0]  # First pair
                    
                    # Process through pipeline
                    result = self.preprocessor.process(raw_emg, return_all_stages=True)
                    
                    # Create timestamps for samples
                    if len(chunk.sample_t) == len(raw_emg):
                        times = chunk.sample_t
                    else:
                        times = np.linspace(t - 0.05, t, len(raw_emg))
                    
                    # Add to buffers
                    self.raw_times.extend(times)
                    self.raw_values.extend(result['raw'])
                    self.normalized_times.extend(times)
                    self.normalized_values.extend(result['normalized'])
            
            # Trim old data
            while self.raw_times and self.raw_times[0] < cutoff_time:
                self.raw_times.pop(0)
                self.raw_values.pop(0)
            while self.normalized_times and self.normalized_times[0] < cutoff_time:
                self.normalized_times.pop(0)
                self.normalized_values.pop(0)
        
        # Get recent IMU samples
        imu_samples = self.sync_acq.imu_buffer.get_recent(now, self.window_seconds)
        
        if imu_samples:
            cutoff_time = now - self.window_seconds
            
            for t, sample in imu_samples[-100:]:  # Last 100 samples
                self.imu_times.append(t)
                # Extract Euler angles (roll, pitch, yaw) in degrees
                if sample.reading.euler1:
                    self.imu_rolls.append(sample.reading.euler1[0])
                    self.imu_pitches.append(sample.reading.euler1[1])
                    self.imu_yaws.append(sample.reading.euler1[2])
                else:
                    self.imu_rolls.append(np.nan)
                    self.imu_pitches.append(np.nan)
                    self.imu_yaws.append(np.nan)
            
            # Trim old data
            while self.imu_times and self.imu_times[0] < cutoff_time:
                self.imu_times.pop(0)
                self.imu_rolls.pop(0)
                self.imu_pitches.pop(0)
                self.imu_yaws.pop(0)
        
        # Update plots
        if self.raw_times:
            self.line_raw.set_data(self.raw_times, self.raw_values)
            self.ax_raw.relim()
            self.ax_raw.autoscale_view()
        
        if self.normalized_times:
            self.line_normalized.set_data(self.normalized_times, self.normalized_values)
            self.ax_preprocessed.relim()
            self.ax_preprocessed.autoscale_view()
            self.ax_preprocessed.set_ylim(-0.1, max(1.5, np.max(self.normalized_values[-1000:]) * 1.1))
        
        if self.imu_times:
            self.line_imu_roll.set_data(self.imu_times, self.imu_rolls)
            self.line_imu_pitch.set_data(self.imu_times, self.imu_pitches)
            self.line_imu_yaw.set_data(self.imu_times, self.imu_yaws)
            self.ax_imu.relim()
            self.ax_imu.autoscale_view()
        
        return self.line_raw, self.line_normalized, self.line_imu_roll, self.line_imu_pitch, self.line_imu_yaw
    
    def start(self):
        """Start animation."""
        self.anim = FuncAnimation(
            self.fig,
            self.update,
            interval=PLOT_UPDATE_INTERVAL,
            blit=True,
            cache_frame_data=False
        )
        plt.show()


# =============================================================================
# MAIN EXECUTION
# =============================================================================

def main():
    """Main preprocessing script."""
    print("="*70)
    print("REAL-TIME SIGNAL PREPROCESSING")
    print("="*70)
    print("\nThis script will:")
    print("  1. Calibrate MVC (Maximum Voluntary Contraction)")
    print("  2. Acquire synchronized EMG + IMU signals")
    print("  3. Preprocess EMG in real-time")
    print("  4. Visualize raw vs preprocessed signals")
    print("\nEMG Preprocessing Pipeline:")
    print("  Raw → Bandpass (20-450 Hz) → Notch (50 Hz) →")
    print("  → Rectify → Smooth (10 Hz) → MVC Normalize")
    print("\nIMU Processing:")
    print("  Pass-through (already clean from acquisition)")
    
    # Configuration
    sync_config = SyncConfig(
        emg=EMGSyncConfig(
            sample_rate=EMG_SAMPLE_RATE,
            selection=EMGSelection(
                differential_pairs=EMG_DIFFERENTIAL_PAIRS
            ),
            connection_type='usb'
        ),
        imu=IMUSyncConfig(
            imu_config=IMUConfig(
                imu_type=IMUType.BMI160_DUAL,
                port=None,  # Auto-detect
                baud=230400
            ),
            calibration_samples=200
        )
    )
    
    try:
        # Initialize synchronized acquisition
        print("\n" + "="*70)
        print("INITIALIZING ACQUISITION")
        print("="*70)
        sync_acq = SynchronizedAcquisition(sync_config)
        sync_acq.start()
        
        print("\nWaiting for data...")
        time.sleep(2.0)
        
        # MVC Calibration
        mvc_value = calibrate_mvc(
            sync_acq,
            duration=MVC_CALIBRATION_DURATION,
            percentile=MVC_PERCENTILE
        )
        
        # Initialize preprocessor with MVC value
        preprocessor = EMGPreprocessor(
            fs=EMG_SAMPLE_RATE,
            mvc_value=mvc_value,
            bandpass_low=BANDPASS_LOW,
            bandpass_high=BANDPASS_HIGH,
            notch_freq=NOTCH_FREQ,
            envelope_cutoff=ENVELOPE_CUTOFF
        )
        
        # Start visualization
        print("\n" + "="*70)
        print("STARTING REAL-TIME VISUALIZATION")
        print("="*70)
        print("\nClose the plot window to stop acquisition.")
        
        visualizer = PreprocessingVisualizer(
            sync_acq,
            preprocessor,
            window_seconds=PLOT_WINDOW_SECONDS
        )
        
        # This blocks until window is closed
        visualizer.start()
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        print("\n" + "="*70)
        print("SHUTTING DOWN")
        print("="*70)
        if 'sync_acq' in locals():
            sync_acq.stop()
        print("Done!")


if __name__ == '__main__':
    main()
