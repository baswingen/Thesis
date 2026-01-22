"""
Dual BMI160 IMU Acquisition Module (ASCII Protocol)
===================================================

High-performance module for acquiring data from dual BMI160 IMU sensors
via Arduino with ASCII serial protocol.

Features:
- ASCII protocol with XOR checksum validation
- Automatic port detection and reconnection
- Mahony filter for orientation estimation
- Gyro bias calibration
- Context manager support
- Easy-to-use interface

Hardware Requirements:
- Arduino with dual BMI160 IMUs (I2C addresses 0x68 and 0x69)
- ASCII protocol sketch (230400 baud)

Example Usage:
    ```python
    from src.bmi160_dual import DualIMU, IMUConfig
    
    # Basic usage
    with DualIMU() as imu:
        imu.calibrate(samples=200)
        
        for reading in imu.read_stream(duration=10.0):
            print(f"IMU1 orientation: {reading.euler1}")
            print(f"IMU2 orientation: {reading.euler2}")
    
    # Advanced configuration
    config = IMUConfig(
        port='/dev/ttyUSB0',
        baud=230400,
        accel_scale=1.0/16384.0,
        gyro_scale=1.0/131.2
    )
    
    with DualIMU(config) as imu:
        imu.calibrate()
        while True:
            reading = imu.read()
            if reading:
                # Process data
                pass
    ```

Author: Generated from IMU_testing.py
Date: January 2026
"""

import time
import numpy as np
import serial
from serial.tools import list_ports
from typing import Optional, Tuple, Generator, Callable
from dataclasses import dataclass, field
from enum import Enum


# =============================================================================
# CONFIGURATION
# =============================================================================

@dataclass
class IMUConfig:
    """Configuration for dual IMU acquisition."""
    
    # Serial settings
    port: Optional[str] = None  # None = auto-detect
    baud: int = 230400
    timeout: float = 0.25
    
    # Scaling factors (depends on Arduino BMI160 configuration)
    accel_scale: float = 1.0 / 16384.0  # ±2g range
    gyro_scale: float = 1.0 / 131.2     # ±250°/s range
    
    # Filter settings
    use_mahony: bool = True
    kp: float = 2.0  # Mahony proportional gain
    ki: float = 0.01  # Mahony integral gain
    adaptive_gains: bool = True
    
    # Calibration
    gyro_still_threshold: float = 0.5  # rad/s
    accel_gate_min: float = 0.92  # g
    accel_gate_max: float = 1.08  # g
    
    # Axis remapping (if needed)
    imu1_axis_map: Tuple[int, int, int] = (1, 2, 3)
    imu2_axis_map: Tuple[int, int, int] = (1, 2, 3)
    invert_accel: bool = False
    
    # Reconnection
    auto_reconnect: bool = True
    stall_timeout: float = 3.0
    reconnect_backoff: float = 1.0


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class RawSample:
    """Raw sample from Arduino (integer values)."""
    seq: int
    t_us: int
    ax1: int
    ay1: int
    az1: int
    gx1: int
    gy1: int
    gz1: int
    ok1: int
    ax2: int
    ay2: int
    az2: int
    gx2: int
    gy2: int
    gz2: int
    ok2: int


@dataclass
class IMUReading:
    """Processed reading from dual IMU sensors."""
    
    # Timing
    timestamp: float  # Python time.time()
    t_us: int  # Arduino microseconds
    seq: int  # Sequence number
    
    # IMU 1 raw data
    gyro1: np.ndarray  # [gx, gy, gz] in rad/s
    accel1: np.ndarray  # [ax, ay, az] in g
    
    # IMU 2 raw data
    gyro2: np.ndarray  # [gx, gy, gz] in rad/s
    accel2: np.ndarray  # [ax, ay, az] in g
    
    # IMU 1 orientation (if filtering enabled)
    quat1: Optional[np.ndarray] = None  # [w, x, y, z]
    euler1: Optional[Tuple[float, float, float]] = None  # (roll, pitch, yaw) in degrees
    
    # IMU 2 orientation (if filtering enabled)
    quat2: Optional[np.ndarray] = None  # [w, x, y, z]
    euler2: Optional[Tuple[float, float, float]] = None  # (roll, pitch, yaw) in degrees
    
    # Status
    ok1: bool = True
    ok2: bool = True
    
    @property
    def timestamp_ms(self) -> float:
        """Arduino timestamp in milliseconds."""
        return self.t_us / 1000.0


@dataclass
class CalibrationData:
    """Gyroscope bias calibration data."""
    bias1: np.ndarray  # IMU1 bias in rad/s
    bias2: np.ndarray  # IMU2 bias in rad/s
    samples: int
    
    def __str__(self):
        b1_deg = np.rad2deg(self.bias1)
        b2_deg = np.rad2deg(self.bias2)
        return (f"Calibration ({self.samples} samples):\n"
                f"  IMU1: [{b1_deg[0]:.2f}, {b1_deg[1]:.2f}, {b1_deg[2]:.2f}] deg/s\n"
                f"  IMU2: [{b2_deg[0]:.2f}, {b2_deg[1]:.2f}, {b2_deg[2]:.2f}] deg/s")


# =============================================================================
# QUATERNION MATH
# =============================================================================

def quat_mul(q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Quaternion multiplication."""
    w0, x0, y0, z0 = q
    w1, x1, y1, z1 = r
    return np.array([
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1
    ], dtype=float)


def quat_conj(q: np.ndarray) -> np.ndarray:
    """Quaternion conjugate."""
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quat_norm(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion."""
    n = np.linalg.norm(q)
    if n < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n


def quat_to_euler(q: np.ndarray) -> Tuple[float, float, float]:
    """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees."""
    w, x, y, z = q
    
    # Roll (x-axis)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


# =============================================================================
# MAHONY FILTER
# =============================================================================

class MahonyFilter:
    """Mahony AHRS filter for IMU orientation estimation."""
    
    def __init__(self, kp: float = 2.0, ki: float = 0.01, adaptive: bool = True):
        self.kp = kp
        self.ki = ki
        self.adaptive = adaptive
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.integral_error = np.zeros(3, dtype=float)
    
    def update(self, gyro: np.ndarray, accel: np.ndarray, dt: float) -> np.ndarray:
        """
        Update filter with new IMU data.
        
        Args:
            gyro: [gx, gy, gz] in rad/s
            accel: [ax, ay, az] in g
            dt: Time step in seconds
        
        Returns:
            Quaternion [w, x, y, z]
        """
        if dt <= 0:
            return self.q
        
        gx, gy, gz = float(gyro[0]), float(gyro[1]), float(gyro[2])
        ax, ay, az = float(accel[0]), float(accel[1]), float(accel[2])
        
        # Adaptive gain
        gyro_mag = float(np.linalg.norm(gyro))
        accel_mag = float(np.linalg.norm(accel))
        
        if self.adaptive:
            if gyro_mag < 0.5 and 0.92 <= accel_mag <= 1.08:
                kp = self.kp * 3.0  # High gain when still
            elif accel_mag < 0.92 or accel_mag > 1.08:
                kp = self.kp * 0.1  # Low gain during motion
            else:
                kp = self.kp
        else:
            kp = self.kp
        
        q0, q1, q2, q3 = self.q
        
        # Normalize accel
        if accel_mag > 1e-6:
            ax /= accel_mag
            ay /= accel_mag
            az /= accel_mag
            
            # Estimated gravity direction
            vx = 2.0 * (q1*q3 - q0*q2)
            vy = 2.0 * (q0*q1 + q2*q3)
            vz = q0*q0 - q1*q1 - q2*q2 + q3*q3
            
            # Error (cross product)
            ex = ay*vz - az*vy
            ey = az*vx - ax*vz
            ez = ax*vy - ay*vx
            
            # Integral feedback
            if self.ki > 0:
                self.integral_error += np.array([ex, ey, ez]) * dt * self.ki
                gx += self.integral_error[0]
                gy += self.integral_error[1]
                gz += self.integral_error[2]
            
            # Proportional feedback
            gx += kp * ex
            gy += kp * ey
            gz += kp * ez
        
        # Integrate quaternion
        half_dt = 0.5 * dt
        qa, qb, qc = q0, q1, q2
        q0 += (-qb*gx - qc*gy - q3*gz) * half_dt
        q1 += (qa*gx + qc*gz - q3*gy) * half_dt
        q2 += (qa*gy - qb*gz + q3*gx) * half_dt
        q3 += (qa*gz + qb*gy - qc*gx) * half_dt
        
        self.q = quat_norm(np.array([q0, q1, q2, q3], dtype=float))
        return self.q
    
    def reset(self):
        """Reset to identity orientation."""
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.integral_error = np.zeros(3, dtype=float)


# =============================================================================
# SERIAL PROTOCOL
# =============================================================================

def xor_checksum(payload: str) -> int:
    """Calculate XOR checksum."""
    c = 0
    for ch in payload:
        c ^= ord(ch) & 0xFF
    return c


def parse_line(line: str) -> Optional[RawSample]:
    """
    Parse ASCII line: $seq,us,ax1,ay1,az1,gx1,gy1,gz1,ok1,ax2,ay2,az2,gx2,gy2,gz2,ok2*HH
    
    Returns:
        RawSample if valid, else None
    """
    line = line.strip()
    if not line or not line.startswith("$"):
        return None
    
    star = line.rfind("*")
    if star == -1 or star < 2:
        return None
    
    payload = line[1:star]
    chk_hex = line[star + 1:]
    if len(chk_hex) != 2:
        return None
    
    try:
        expected = int(chk_hex, 16)
    except ValueError:
        return None
    
    actual = xor_checksum(payload)
    if actual != expected:
        return None
    
    parts = payload.split(",")
    # Arduino format has 16 or 17 parts (bug in some sketches)
    if len(parts) not in (16, 17):
        return None
    
    try:
        vals = list(map(int, parts))
    except ValueError:
        return None
    
    return RawSample(
        seq=vals[0],
        t_us=vals[1],
        ax1=vals[2], ay1=vals[3], az1=vals[4],
        gx1=vals[5], gy1=vals[6], gz1=vals[7],
        ok1=vals[8],
        ax2=vals[9], ay2=vals[10], az2=vals[11],
        gx2=vals[12], gy2=vals[13], gz2=vals[14],
        ok2=vals[15]
    )


def remap_vec(v: np.ndarray, mapping: Tuple[int, int, int]) -> np.ndarray:
    """Remap 3-vector with signed axis mapping."""
    out = np.empty(3, dtype=float)
    for i, a in enumerate(mapping):
        s = 1.0 if a > 0 else -1.0
        idx = abs(int(a)) - 1
        out[i] = s * v[idx]
    return out


# =============================================================================
# SERIAL CONNECTION
# =============================================================================

def guess_port(hint: Optional[str] = None) -> Optional[str]:
    """Auto-detect Arduino serial port."""
    ports = list(list_ports.comports())
    if not ports:
        return None
    
    if hint:
        hint = hint.lower()
        for p in ports:
            if hint in (p.device or "").lower() or hint in (p.description or "").lower():
                return p.device
    
    # Heuristic: Arduino-like ports
    for p in ports:
        desc = (p.description or "").lower()
        if "arduino" in desc or "usb" in desc or "serial" in desc:
            return p.device
    
    return ports[0].device


# =============================================================================
# MAIN DUAL IMU CLASS
# =============================================================================

class DualIMU:
    """
    Main interface for dual BMI160 IMU acquisition.
    
    Handles serial communication, data parsing, calibration, and orientation
    estimation for two BMI160 IMUs connected via Arduino.
    
    Example:
        ```python
        with DualIMU() as imu:
            imu.calibrate(samples=200)
            
            for i in range(1000):
                reading = imu.read()
                if reading:
                    print(f"Euler1: {reading.euler1}")
        ```
    """
    
    def __init__(self, config: Optional[IMUConfig] = None):
        """
        Initialize dual IMU interface.
        
        Args:
            config: IMUConfig object (uses defaults if None)
        """
        self.config = config or IMUConfig()
        self.ser: Optional[serial.Serial] = None
        self.calibration: Optional[CalibrationData] = None
        self.last_rx_time = 0.0
        self.t_prev = None
        
        # Filters
        if self.config.use_mahony:
            self.filter1 = MahonyFilter(
                kp=self.config.kp,
                ki=self.config.ki,
                adaptive=self.config.adaptive_gains
            )
            self.filter2 = MahonyFilter(
                kp=self.config.kp,
                ki=self.config.ki,
                adaptive=self.config.adaptive_gains
            )
        else:
            self.filter1 = None
            self.filter2 = None
    
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
    
    def connect(self) -> bool:
        """
        Connect to Arduino serial port.
        
        Returns:
            True if connection successful
        """
        port = self.config.port or guess_port("usbmodem")
        if not port:
            raise RuntimeError("No serial port found")
        
        print(f"Connecting to {port} @ {self.config.baud} baud...")
        
        self.ser = serial.Serial(
            port=port,
            baudrate=self.config.baud,
            timeout=self.config.timeout
        )
        
        time.sleep(1.5)  # Wait for Arduino reset
        self.ser.reset_input_buffer()
        self.last_rx_time = time.time()
        
        # Verify connection
        valid = 0
        for _ in range(20):
            raw = self.ser.readline()
            if raw:
                try:
                    line = raw.decode("ascii", errors="ignore").strip()
                    if line.startswith("#"):
                        continue
                    sample = parse_line(line)
                    if sample:
                        valid += 1
                        if valid >= 3:
                            print("✓ Connected and receiving data")
                            return True
                except Exception:
                    pass
            time.sleep(0.05)
        
        if valid > 0:
            print(f"✓ Connected ({valid} valid packets)")
            return True
        else:
            print("⚠ Connected but no valid data received")
            return False
    
    def close(self):
        """Close serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✓ Disconnected")
    
    def reconnect(self) -> bool:
        """Reconnect to serial port."""
        try:
            if self.ser:
                self.ser.close()
        except:
            pass
        time.sleep(self.config.reconnect_backoff)
        return self.connect()
    
    def calibrate(self, samples: int = 200, timeout: float = 30.0,
                  callback: Optional[Callable[[int, int], None]] = None) -> CalibrationData:
        """
        Calibrate gyroscope biases (IMUs must be stationary).
        
        Args:
            samples: Number of samples to collect
            timeout: Timeout in seconds
            callback: Optional callback(current, total) for progress
        
        Returns:
            CalibrationData object
        """
        print(f"\nCalibrating ({samples} samples)...")
        print("Keep IMUs FLAT and STILL!")
        
        g1_samples = []
        g2_samples = []
        t0 = time.time()
        
        while len(g1_samples) < samples:
            if time.time() - t0 > timeout:
                if len(g1_samples) >= 50:
                    print(f"\nTimeout - using {len(g1_samples)} samples")
                    break
                raise TimeoutError(f"Calibration failed: only {len(g1_samples)} samples")
            
            raw_reading = self._read_raw()
            if raw_reading is None:
                continue
            
            sample, timestamp = raw_reading
            
            # Convert to rad/s
            g1 = np.array([sample.gx1, sample.gy1, sample.gz1]) * self.config.gyro_scale
            g2 = np.array([sample.gx2, sample.gy2, sample.gz2]) * self.config.gyro_scale
            g1 = np.deg2rad(g1)
            g2 = np.deg2rad(g2)
            
            g1_samples.append(g1)
            g2_samples.append(g2)
            
            if callback:
                callback(len(g1_samples), samples)
            elif len(g1_samples) % 20 == 0:
                print(f"  {len(g1_samples)}/{samples}...", end='\r')
        
        print(f"\n✓ Calibration complete ({len(g1_samples)} samples)")
        
        bias1 = np.mean(g1_samples, axis=0)
        bias2 = np.mean(g2_samples, axis=0)
        
        self.calibration = CalibrationData(
            bias1=bias1,
            bias2=bias2,
            samples=len(g1_samples)
        )
        
        # Reset filters
        if self.filter1:
            self.filter1.reset()
        if self.filter2:
            self.filter2.reset()
        
        self.t_prev = None
        
        print(self.calibration)
        return self.calibration
    
    def _read_raw(self) -> Optional[Tuple[RawSample, float]]:
        """
        Read one raw sample from serial.
        
        Returns:
            (RawSample, timestamp) or None
        """
        if not self.ser or not self.ser.is_open:
            return None
        
        try:
            raw = self.ser.readline()
            if not raw:
                return None
            
            line = raw.decode("ascii", errors="ignore").strip()
            if not line or line.startswith("#"):
                return None
            
            sample = parse_line(line)
            if sample is None:
                return None
            
            if sample.ok1 == 0 or sample.ok2 == 0:
                return None
            
            self.last_rx_time = time.time()
            return sample, time.time()
        
        except Exception:
            return None
    
    def read(self, timeout: Optional[float] = None) -> Optional[IMUReading]:
        """
        Read one processed IMU reading.
        
        Args:
            timeout: Optional timeout in seconds (None = use config timeout)
        
        Returns:
            IMUReading object or None
        """
        # Check for stale connection
        if self.config.auto_reconnect:
            if time.time() - self.last_rx_time > self.config.stall_timeout:
                print("\n⚠ Connection stalled, reconnecting...")
                if not self.reconnect():
                    return None
                self.t_prev = None
        
        raw_result = self._read_raw()
        if raw_result is None:
            return None
        
        sample, timestamp = raw_result
        
        # Convert to physical units
        gyro1 = np.array([sample.gx1, sample.gy1, sample.gz1]) * self.config.gyro_scale
        accel1 = np.array([sample.ax1, sample.ay1, sample.az1]) * self.config.accel_scale
        gyro2 = np.array([sample.gx2, sample.gy2, sample.gz2]) * self.config.gyro_scale
        accel2 = np.array([sample.ax2, sample.ay2, sample.az2]) * self.config.accel_scale
        
        # Apply axis remapping
        gyro1 = remap_vec(gyro1, self.config.imu1_axis_map)
        accel1 = remap_vec(accel1, self.config.imu1_axis_map)
        gyro2 = remap_vec(gyro2, self.config.imu2_axis_map)
        accel2 = remap_vec(accel2, self.config.imu2_axis_map)
        
        if self.config.invert_accel:
            accel1 = -accel1
            accel2 = -accel2
        
        # Convert gyro to rad/s and subtract bias
        gyro1_rad = np.deg2rad(gyro1)
        gyro2_rad = np.deg2rad(gyro2)
        
        if self.calibration:
            gyro1_rad -= remap_vec(self.calibration.bias1, self.config.imu1_axis_map)
            gyro2_rad -= remap_vec(self.calibration.bias2, self.config.imu2_axis_map)
        
        # Update orientation filters
        quat1 = None
        quat2 = None
        euler1 = None
        euler2 = None
        
        if self.filter1 and self.filter2:
            t_s = sample.t_us / 1_000_000.0
            
            if self.t_prev is not None:
                dt = t_s - self.t_prev
                if 0 < dt < 0.2:  # Sanity check
                    quat1 = self.filter1.update(gyro1_rad, accel1, dt)
                    quat2 = self.filter2.update(gyro2_rad, accel2, dt)
                    euler1 = quat_to_euler(quat1)
                    euler2 = quat_to_euler(quat2)
            
            self.t_prev = t_s
        
        return IMUReading(
            timestamp=timestamp,
            t_us=sample.t_us,
            seq=sample.seq,
            gyro1=gyro1_rad,
            accel1=accel1,
            gyro2=gyro2_rad,
            accel2=accel2,
            quat1=quat1,
            euler1=euler1,
            quat2=quat2,
            euler2=euler2,
            ok1=bool(sample.ok1),
            ok2=bool(sample.ok2)
        )
    
    def read_stream(self, duration: Optional[float] = None,
                   max_samples: Optional[int] = None) -> Generator[IMUReading, None, None]:
        """
        Stream IMU readings.
        
        Args:
            duration: Stop after this many seconds (None = infinite)
            max_samples: Stop after this many samples (None = infinite)
        
        Yields:
            IMUReading objects
        """
        start_time = time.time()
        count = 0
        
        while True:
            if duration and (time.time() - start_time) >= duration:
                break
            if max_samples and count >= max_samples:
                break
            
            reading = self.read()
            if reading:
                yield reading
                count += 1


# =============================================================================
# CONVENIENCE FUNCTIONS
# =============================================================================

def quick_read(port: Optional[str] = None, samples: int = 100,
               calibrate: bool = True, calib_samples: int = 200) -> list[IMUReading]:
    """
    Quick utility to read IMU data.
    
    Args:
        port: Serial port (None = auto-detect)
        samples: Number of samples to read
        calibrate: Whether to calibrate first
        calib_samples: Calibration samples
    
    Returns:
        List of IMUReading objects
    """
    config = IMUConfig(port=port)
    
    with DualIMU(config) as imu:
        if calibrate:
            imu.calibrate(samples=calib_samples)
        
        readings = []
        for reading in imu.read_stream(max_samples=samples):
            readings.append(reading)
        
        return readings


if __name__ == "__main__":
    # Demo
    print("Dual BMI160 IMU Module Demo")
    print("="*60)
    
    config = IMUConfig()
    
    with DualIMU(config) as imu:
        print("\nPlace IMUs flat and still for calibration...")
        time.sleep(3)
        
        imu.calibrate(samples=200)
        
        print("\nReading 100 samples...")
        for i, reading in enumerate(imu.read_stream(max_samples=100)):
            if i % 10 == 0:
                print(f"\nSample {reading.seq}:")
                print(f"  IMU1 Accel: {reading.accel1}")
                print(f"  IMU1 Euler: {reading.euler1}")
                print(f"  IMU2 Euler: {reading.euler2}")
        
        print("\n✓ Demo complete")
