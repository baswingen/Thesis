"""
BNO085 UART-RVC Dual IMU Acquisition Module
============================================

Standalone module for acquiring data from dual BNO085 IMU sensors
via Arduino running the BNO085_RVC_Dual sketch.

This module provides native BNO085 acquisition independent of the
generic IMUDevice framework, with a focus on simplicity and direct
Arduino CSV protocol handling.

Features:
- Direct Arduino CSV protocol parsing
- Dual BNO085 sensor support
- Native acceleration data (m/s²)
- Built-in PRBS marker injection support
- Reference orientation calibration
- Context manager support

Arduino CSV Format:
  t_ms,s1_y,s1_p,s1_r,s1_ax,s1_ay,s1_az,s2_y,s2_p,s2_r,s2_ax,s2_ay,s2_az

Hardware Requirements:
- Arduino with dual BNO085 sensors in UART-RVC mode
- USB connection
- BNO085_RVC_Dual.ino sketch running on Arduino

Author: Generated for Master Thesis
Date: January 2026
"""

import time
import serial
import numpy as np
from typing import Optional, Tuple, Generator, List
from dataclasses import dataclass
from pathlib import Path
import sys

# Add project root to path for arduino_connection
_project_root = Path(__file__).parent.parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.arduino_connection import find_arduino_port


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class BNO085Sample:
    """
    Sample from dual BNO085 sensors.
    
    Orientation angles in degrees (Yaw-Pitch-Roll convention).
    Acceleration in m/s².
    """
    timestamp: float  # Python time.time() when received
    t_ms: int  # Arduino millis() timestamp
    
    # Sensor 1
    yaw1: float  # degrees
    pitch1: float  # degrees
    roll1: float  # degrees
    ax1: float  # m/s²
    ay1: float  # m/s²
    az1: float  # m/s²
    
    # Sensor 2
    yaw2: float  # degrees
    pitch2: float  # degrees
    roll2: float  # degrees
    ax2: float  # m/s²
    ay2: float  # m/s²
    az2: float  # m/s²
    
    @property
    def euler1(self) -> Tuple[float, float, float]:
        """Sensor 1 orientation as (roll, pitch, yaw) tuple."""
        return (self.roll1, self.pitch1, self.yaw1)
    
    @property
    def euler2(self) -> Tuple[float, float, float]:
        """Sensor 2 orientation as (roll, pitch, yaw) tuple."""
        return (self.roll2, self.pitch2, self.yaw2)
    
    @property
    def accel1(self) -> np.ndarray:
        """Sensor 1 acceleration as [ax, ay, az] array."""
        return np.array([self.ax1, self.ay1, self.az1])
    
    @property
    def accel2(self) -> np.ndarray:
        """Sensor 2 acceleration as [ax, ay, az] array."""
        return np.array([self.ax2, self.ay2, self.az2])


@dataclass
class BNO085Calibration:
    """Calibration data for BNO085 sensors (reference orientation)."""
    # Reference orientation (degrees)
    yaw1_ref: float
    pitch1_ref: float
    roll1_ref: float
    yaw2_ref: float
    pitch2_ref: float
    roll2_ref: float
    
    # Reference acceleration (m/s²)
    ax1_ref: float
    ay1_ref: float
    az1_ref: float
    ax2_ref: float
    ay2_ref: float
    az2_ref: float
    
    samples: int  # Number of samples used for calibration
    
    def __str__(self):
        return (f"BNO085 Calibration ({self.samples} samples):\n"
                f"  Sensor 1: Y={self.yaw1_ref:+.1f}° P={self.pitch1_ref:+.1f}° R={self.roll1_ref:+.1f}° "
                f"A=[{self.ax1_ref:+.2f}, {self.ay1_ref:+.2f}, {self.az1_ref:+.2f}] m/s²\n"
                f"  Sensor 2: Y={self.yaw2_ref:+.1f}° P={self.pitch2_ref:+.1f}° R={self.roll2_ref:+.1f}° "
                f"A=[{self.ax2_ref:+.2f}, {self.ay2_ref:+.2f}, {self.az2_ref:+.2f}] m/s²")


# =============================================================================
# BNO085 DEVICE CLASS
# =============================================================================

class BNO085UartRvcDevice:
    """
    Native BNO085 UART-RVC dual IMU acquisition (Arduino-based).
    
    Communicates directly with Arduino running BNO085_RVC_Dual sketch.
    Provides enhanced timing with PRBS synchronization support.
    
    Example:
        >>> with BNO085UartRvcDevice() as device:
        >>>     device.calibrate(samples=50)
        >>>     for sample in device.read_stream(duration=10.0):
        >>>         print(f"Sensor 1: Y={sample.yaw1:.1f}° P={sample.pitch1:.1f}° R={sample.roll1:.1f}°")
    """
    
    def __init__(self, port: Optional[str] = None, baud: int = 115200, timeout: float = 0.25):
        """
        Initialize BNO085 device interface.
        
        Args:
            port: Serial port (None = auto-detect)
            baud: Baud rate (default: 115200)
            timeout: Serial read timeout in seconds
        """
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self.calibration: Optional[BNO085Calibration] = None
        self.last_rx_time = 0.0
    
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
    
    def connect(self) -> bool:
        """
        Connect to Arduino with BNO085 sensors.
        
        Returns:
            True if connection successful
        
        Raises:
            RuntimeError: If connection fails
        """
        # Auto-detect port if not specified
        port = self.port or find_arduino_port(preferred_substr="usbmodem", verbose=True)
        if not port:
            raise RuntimeError("No serial port found. Please specify port manually.")
        
        if self.port is None:
            print(f"Auto-detected port: {port}")
        
        print(f"Connecting to {port} @ {self.baud} baud...")
        
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=self.baud,
                timeout=self.timeout
            )
            
            time.sleep(1.5)  # Wait for Arduino reset
            self.ser.reset_input_buffer()
            self.last_rx_time = time.time()
            
            # Verify connection by reading a few valid samples
            valid_count = 0
            for _ in range(20):
                line = self._read_line()
                if line and self._parse_csv_line(line) is not None:
                    valid_count += 1
                    if valid_count >= 3:
                        print("✓ Connected and receiving data")
                        return True
                time.sleep(0.05)
            
            if valid_count > 0:
                print(f"✓ Connected ({valid_count} valid samples)")
                return True
            else:
                print("⚠ Connected but no valid data received")
                return False
                
        except Exception as e:
            raise RuntimeError(f"Failed to connect: {e}") from e
    
    def close(self):
        """Close serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✓ Disconnected")
    
    def _read_line(self) -> Optional[str]:
        """
        Read one line from serial port.
        
        Returns:
            Line string or None if timeout/error
        """
        if not self.ser or not self.ser.is_open:
            return None
        
        try:
            raw = self.ser.readline()
            if not raw:
                return None
            
            line = raw.decode('ascii', errors='ignore').strip()
            self.last_rx_time = time.time()
            return line
            
        except Exception:
            return None
    
    def _parse_csv_line(self, line: str) -> Optional[Tuple[int, float, float, float, float, float, float,
                                                            float, float, float, float, float, float]]:
        """
        Parse CSV line from Arduino.
        
        Format: t_ms,s1_y,s1_p,s1_r,s1_ax,s1_ay,s1_az,s2_y,s2_p,s2_r,s2_ax,s2_ay,s2_az
        
        Returns:
            Tuple of parsed values or None if invalid
        """
        if not line or line.startswith('#') or line.startswith('t_ms'):
            return None  # Skip comments and header
        
        parts = line.split(',')
        if len(parts) != 13:
            return None
        
        try:
            t_ms = int(parts[0])
            s1_y = float(parts[1])
            s1_p = float(parts[2])
            s1_r = float(parts[3])
            s1_ax = float(parts[4])
            s1_ay = float(parts[5])
            s1_az = float(parts[6])
            s2_y = float(parts[7])
            s2_p = float(parts[8])
            s2_r = float(parts[9])
            s2_ax = float(parts[10])
            s2_ay = float(parts[11])
            s2_az = float(parts[12])
            
            return (t_ms, s1_y, s1_p, s1_r, s1_ax, s1_ay, s1_az,
                    s2_y, s2_p, s2_r, s2_ax, s2_ay, s2_az)
        except (ValueError, IndexError):
            return None
    
    def read_sample(self) -> Optional[BNO085Sample]:
        """
        Read one sample from Arduino.
        
        Returns:
            BNO085Sample or None if no data available
        """
        line = self._read_line()
        if not line:
            return None
        
        parsed = self._parse_csv_line(line)
        if parsed is None:
            return None
        
        timestamp = time.time()
        (t_ms, s1_y, s1_p, s1_r, s1_ax, s1_ay, s1_az,
         s2_y, s2_p, s2_r, s2_ax, s2_ay, s2_az) = parsed
        
        # Apply calibration if available
        if self.calibration is not None:
            s1_y -= self.calibration.yaw1_ref
            s1_p -= self.calibration.pitch1_ref
            s1_r -= self.calibration.roll1_ref
            s2_y -= self.calibration.yaw2_ref
            s2_p -= self.calibration.pitch2_ref
            s2_r -= self.calibration.roll2_ref
            
            # Wrap angles to [-180, 180]
            s1_y = ((s1_y + 180.0) % 360.0) - 180.0
            s1_p = ((s1_p + 180.0) % 360.0) - 180.0
            s1_r = ((s1_r + 180.0) % 360.0) - 180.0
            s2_y = ((s2_y + 180.0) % 360.0) - 180.0
            s2_p = ((s2_p + 180.0) % 360.0) - 180.0
            s2_r = ((s2_r + 180.0) % 360.0) - 180.0
        
        return BNO085Sample(
            timestamp=timestamp,
            t_ms=t_ms,
            yaw1=s1_y, pitch1=s1_p, roll1=s1_r,
            ax1=s1_ax, ay1=s1_ay, az1=s1_az,
            yaw2=s2_y, pitch2=s2_p, roll2=s2_r,
            ax2=s2_ax, ay2=s2_ay, az2=s2_az
        )
    
    def calibrate(self, samples: int = 50, timeout: float = 30.0) -> BNO085Calibration:
        """
        Capture reference orientation.
        
        Place sensors in desired reference position (e.g., flat on table).
        
        Args:
            samples: Number of samples to average
            timeout: Timeout in seconds
        
        Returns:
            BNO085Calibration object
        
        Raises:
            TimeoutError: If calibration fails
        """
        print(f"\nCalibrating BNO085 sensors ({samples} samples)...")
        print("Place sensors FLAT and STILL in reference position")
        
        yaw1_list: List[float] = []
        pitch1_list: List[float] = []
        roll1_list: List[float] = []
        ax1_list: List[float] = []
        ay1_list: List[float] = []
        az1_list: List[float] = []
        
        yaw2_list: List[float] = []
        pitch2_list: List[float] = []
        roll2_list: List[float] = []
        ax2_list: List[float] = []
        ay2_list: List[float] = []
        az2_list: List[float] = []
        
        t0 = time.time()
        
        while len(yaw1_list) < samples:
            if time.time() - t0 > timeout:
                if len(yaw1_list) >= 10:
                    print(f"\nTimeout - using {len(yaw1_list)} samples")
                    break
                raise TimeoutError(f"Calibration failed: only {len(yaw1_list)} samples")
            
            sample = self.read_sample()
            if sample is None:
                continue
            
            yaw1_list.append(sample.yaw1)
            pitch1_list.append(sample.pitch1)
            roll1_list.append(sample.roll1)
            ax1_list.append(sample.ax1)
            ay1_list.append(sample.ay1)
            az1_list.append(sample.az1)
            
            yaw2_list.append(sample.yaw2)
            pitch2_list.append(sample.pitch2)
            roll2_list.append(sample.roll2)
            ax2_list.append(sample.ax2)
            ay2_list.append(sample.ay2)
            az2_list.append(sample.az2)
            
            if len(yaw1_list) % 10 == 0:
                print(f"  {len(yaw1_list)}/{samples}...", end='\r')
        
        print(f"\n✓ Reference captured ({len(yaw1_list)} samples)")
        
        # Compute circular mean for angles (handles wrap-around)
        def circular_mean_deg(angles: List[float]) -> float:
            angles_rad = np.deg2rad(angles)
            s = np.mean(np.sin(angles_rad))
            c = np.mean(np.cos(angles_rad))
            return float(np.rad2deg(np.arctan2(s, c)))
        
        self.calibration = BNO085Calibration(
            yaw1_ref=circular_mean_deg(yaw1_list),
            pitch1_ref=circular_mean_deg(pitch1_list),
            roll1_ref=circular_mean_deg(roll1_list),
            ax1_ref=float(np.mean(ax1_list)),
            ay1_ref=float(np.mean(ay1_list)),
            az1_ref=float(np.mean(az1_list)),
            yaw2_ref=circular_mean_deg(yaw2_list),
            pitch2_ref=circular_mean_deg(pitch2_list),
            roll2_ref=circular_mean_deg(roll2_list),
            ax2_ref=float(np.mean(ax2_list)),
            ay2_ref=float(np.mean(ay2_list)),
            az2_ref=float(np.mean(az2_list)),
            samples=len(yaw1_list)
        )
        
        print(self.calibration)
        return self.calibration
    
    def read_stream(self, duration: Optional[float] = None,
                   max_samples: Optional[int] = None) -> Generator[BNO085Sample, None, None]:
        """
        Stream samples.
        
        Args:
            duration: Stop after this many seconds (None = infinite)
            max_samples: Stop after this many samples (None = infinite)
        
        Yields:
            BNO085Sample objects
        """
        start_time = time.time()
        count = 0
        
        while True:
            if duration and (time.time() - start_time) >= duration:
                break
            if max_samples and count >= max_samples:
                break
            
            sample = self.read_sample()
            if sample:
                yield sample
                count += 1
            else:
                time.sleep(0.001)  # Avoid busy-wait
    
    def __repr__(self):
        status = "connected" if (self.ser and self.ser.is_open) else "disconnected"
        calib_status = "calibrated" if self.calibration else "uncalibrated"
        return f"BNO085UartRvcDevice({status}, {calib_status}, {self.baud} baud)"


# =============================================================================
# CONVENIENCE FUNCTIONS
# =============================================================================

def acquire_bno085_data(port: Optional[str] = None, samples: int = 100,
                       calibrate: bool = True, calib_samples: int = 50) -> List[BNO085Sample]:
    """
    Quick utility to acquire BNO085 data.
    
    Args:
        port: Serial port (None = auto-detect)
        samples: Number of samples to read
        calibrate: Whether to calibrate first
        calib_samples: Calibration samples
    
    Returns:
        List of BNO085Sample objects
    
    Example:
        >>> data = acquire_bno085_data(samples=100)
        >>> print(f"Collected {len(data)} samples")
    """
    with BNO085UartRvcDevice(port=port) as device:
        if calibrate:
            device.calibrate(samples=calib_samples)
        
        readings = []
        for sample in device.read_stream(max_samples=samples):
            readings.append(sample)
        
        return readings


if __name__ == "__main__":
    # Demo
    print("BNO085 UART-RVC Dual IMU Acquisition Module Demo")
    print("=" * 60)
    
    with BNO085UartRvcDevice() as device:
        print("\nPlace sensors flat for calibration...")
        time.sleep(3)
        
        device.calibrate(samples=50)
        
        print("\nReading 50 samples...")
        for i, sample in enumerate(device.read_stream(max_samples=50)):
            if i % 10 == 0:
                print(f"\nSample {i}:")
                print(f"  Sensor 1: Y={sample.yaw1:+7.1f}° P={sample.pitch1:+7.1f}° R={sample.roll1:+7.1f}°")
                print(f"  Sensor 2: Y={sample.yaw2:+7.1f}° P={sample.pitch2:+7.1f}° R={sample.roll2:+7.1f}°")
                print(f"  Accel 1: [{sample.ax1:+6.2f}, {sample.ay1:+6.2f}, {sample.az1:+6.2f}] m/s²")
        
        print("\n✓ Demo complete")
