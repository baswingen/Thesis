"""
Dummy Acquisition System
========================
Simulates EMG and IMU signals for testing without hardware.

Generates realistic-looking signals with noise and patterns.
"""

import time
import threading
import numpy as np
from dataclasses import dataclass
from typing import Callable, Optional, Dict
from .imu_acquisition import IMUReading


@dataclass
class DummyIMUSample:
    """Dummy IMU sample matching real IMUSample interface."""
    t: float
    reading: IMUReading
    quat1: Optional[np.ndarray] = None
    quat2: Optional[np.ndarray] = None
    euler1: Optional[np.ndarray] = None
    euler2: Optional[np.ndarray] = None
    t_hardware_s: float = 0.0


@dataclass
class DummyEMGChunk:
    """Dummy EMG chunk matching real EMGChunk interface."""
    t: float
    sample_t: np.ndarray
    data: Dict[str, np.ndarray]
    counter_raw_last: Optional[int] = None


class DummyAcquisition:
    """
    Dummy acquisition system that generates simulated signals.
    
    Mimics the SynchronizedAcquisition interface for testing.
    """
    
    def __init__(self, 
                 emg_sample_rate: int = 2048,
                 imu_sample_rate: int = 200,
                 emg_channels: int = 4,
                 emg_amplitude: float = 50.0,
                 emg_noise_level: float = 5.0,
                 imu_motion: bool = True,
                 on_emg: Optional[Callable] = None,
                 on_imu: Optional[Callable] = None):
        """
        Initialize dummy acquisition.
        
        Args:
            emg_sample_rate: EMG sampling rate (Hz)
            imu_sample_rate: IMU sampling rate (Hz)
            emg_channels: Number of EMG channels
            emg_amplitude: EMG signal amplitude (microvolts)
            emg_noise_level: EMG noise level (microvolts)
            imu_motion: Add simulated motion to IMU
            on_emg: Callback for EMG chunks
            on_imu: Callback for IMU samples
        """
        self.emg_sample_rate = emg_sample_rate
        self.imu_sample_rate = imu_sample_rate
        self.emg_channels = emg_channels
        self.emg_amplitude = emg_amplitude
        self.emg_noise_level = emg_noise_level
        self.imu_motion = imu_motion
        
        self.on_emg = on_emg
        self.on_imu = on_imu
        
        self.running = False
        self.emg_thread = None
        self.imu_thread = None
        
        self.start_time = None
        self.emg_sample_count = 0
        self.imu_sample_count = 0
        
        # For signal generation
        self.emg_phase = 0.0
        self.imu_phase = 0.0
        
        self.errors = []
        
        print("✓ Dummy acquisition initialized (simulated signals)")
    
    def start(self):
        """Start generating dummy signals."""
        if self.running:
            return
        
        self.running = True
        self.start_time = time.perf_counter()
        self.emg_sample_count = 0
        self.imu_sample_count = 0
        
        # Start EMG thread
        if self.on_emg:
            self.emg_thread = threading.Thread(target=self._emg_loop, daemon=True)
            self.emg_thread.start()
        
        # Start IMU thread
        if self.on_imu:
            self.imu_thread = threading.Thread(target=self._imu_loop, daemon=True)
            self.imu_thread.start()
        
        print("✓ Dummy acquisition started")
    
    def stop(self):
        """Stop generating signals."""
        self.running = False
        
        if self.emg_thread:
            self.emg_thread.join(timeout=1.0)
        if self.imu_thread:
            self.imu_thread.join(timeout=1.0)
        
        print("✓ Dummy acquisition stopped")
    
    def _emg_loop(self):
        """Generate EMG chunks."""
        chunk_size = 16  # samples per chunk
        chunk_interval = chunk_size / self.emg_sample_rate
        
        while self.running:
            try:
                t_now = time.perf_counter()
                
                # Generate chunk of EMG data
                sample_times = []
                emg_data = []
                
                for i in range(chunk_size):
                    sample_t = t_now + (i * (1.0 / self.emg_sample_rate))
                    sample_times.append(sample_t)
                    
                    # Generate multi-channel EMG signal
                    # Mix of: baseline + sine wave + noise
                    sample = np.zeros(self.emg_channels)
                    
                    for ch in range(self.emg_channels):
                        # Different frequency for each channel
                        freq = 20 + ch * 5  # 20, 25, 30, 35 Hz
                        
                        # Sine wave (muscle activation)
                        sine_component = self.emg_amplitude * np.sin(2 * np.pi * freq * self.emg_phase)
                        
                        # Noise
                        noise_component = self.emg_noise_level * np.random.randn()
                        
                        sample[ch] = sine_component + noise_component
                    
                    emg_data.append(sample)
                    self.emg_phase += 1.0 / self.emg_sample_rate
                
                # Create chunk
                chunk = DummyEMGChunk(
                    t=np.mean(sample_times),
                    sample_t=np.array(sample_times),
                    data={'raw': np.array(emg_data)},
                    counter_raw_last=None
                )
                
                # Call callback
                if self.on_emg:
                    self.on_emg(chunk)
                
                self.emg_sample_count += chunk_size
                
                # Sleep until next chunk
                time.sleep(chunk_interval)
                
            except Exception as e:
                self.errors.append(f"EMG generation error: {e}")
                print(f"⚠ Dummy EMG error: {e}")
                time.sleep(0.1)
    
    def _imu_loop(self):
        """Generate IMU samples."""
        sample_interval = 1.0 / self.imu_sample_rate
        
        while self.running:
            try:
                t_now = time.perf_counter()
                
                # Generate IMU data
                if self.imu_motion:
                    # Simulated motion
                    accel1 = np.array([
                        np.sin(2 * np.pi * 0.5 * self.imu_phase),
                        np.cos(2 * np.pi * 0.3 * self.imu_phase),
                        1.0 + 0.1 * np.sin(2 * np.pi * 0.2 * self.imu_phase)
                    ])
                    gyro1 = np.array([
                        10 * np.sin(2 * np.pi * 0.4 * self.imu_phase),
                        10 * np.cos(2 * np.pi * 0.3 * self.imu_phase),
                        5 * np.sin(2 * np.pi * 0.5 * self.imu_phase)
                    ])
                    
                    accel2 = np.array([
                        np.sin(2 * np.pi * 0.4 * self.imu_phase + 0.5),
                        np.cos(2 * np.pi * 0.35 * self.imu_phase + 0.5),
                        1.0 + 0.1 * np.sin(2 * np.pi * 0.25 * self.imu_phase + 0.5)
                    ])
                    gyro2 = np.array([
                        10 * np.sin(2 * np.pi * 0.45 * self.imu_phase + 0.5),
                        10 * np.cos(2 * np.pi * 0.35 * self.imu_phase + 0.5),
                        5 * np.sin(2 * np.pi * 0.55 * self.imu_phase + 0.5)
                    ])
                else:
                    # Static (just gravity)
                    accel1 = np.array([0.0, 0.0, 1.0])
                    gyro1 = np.array([0.0, 0.0, 0.0])
                    accel2 = np.array([0.0, 0.0, 1.0])
                    gyro2 = np.array([0.0, 0.0, 0.0])
                
                # Add small noise
                accel1 += 0.01 * np.random.randn(3)
                gyro1 += 0.5 * np.random.randn(3)
                accel2 += 0.01 * np.random.randn(3)
                gyro2 += 0.5 * np.random.randn(3)
                
                # Create IMU reading
                reading = IMUReading(
                    timestamp=t_now,
                    t_us=int(self.imu_sample_count * 1e6 / self.imu_sample_rate),
                    seq=self.imu_sample_count,
                    accel1=accel1,
                    gyro1=gyro1,
                    accel2=accel2,
                    gyro2=gyro2
                )
                
                # Create sample
                sample = DummyIMUSample(
                    t=t_now,
                    reading=reading,
                    quat1=None,
                    quat2=None,
                    euler1=None,
                    euler2=None,
                    t_hardware_s=self.imu_sample_count / self.imu_sample_rate
                )
                
                # Call callback
                if self.on_imu:
                    self.on_imu(sample)
                
                self.imu_sample_count += 1
                self.imu_phase += sample_interval
                
                # Sleep until next sample
                time.sleep(sample_interval)
                
            except Exception as e:
                self.errors.append(f"IMU generation error: {e}")
                print(f"⚠ Dummy IMU error: {e}")
                time.sleep(0.1)
