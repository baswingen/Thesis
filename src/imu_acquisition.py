"""
IMU Signal Acquisition Module
==============================

High-performance module for acquiring data from dual BMI160 IMU sensors
via Arduino with ASCII serial protocol.

This module provides:
- Robust ASCII protocol with XOR checksum validation
- Dual BMI160 IMU support (I2C addresses 0x68 and 0x69)
- Mahony AHRS filter for orientation estimation
- Automatic gyroscope bias calibration
- Auto-reconnection and error handling
- Context manager support

Hardware Requirements:
- Arduino with dual BMI160 IMUs
- ASCII protocol sketch (230400 baud)
- USB connection

Example Usage:
    ```python
    from src.imu_acquisition import IMUDevice, IMUConfig
    
    # Basic usage
    with IMUDevice() as imu:
        imu.calibrate(samples=200)
        
        for reading in imu.read_stream(duration=10.0):
            print(f"IMU1: {reading.euler1}")
            print(f"IMU2: {reading.euler2}")
    ```

Author: Generated from IMU_testing.py
Date: January 2026
"""

import time
import numpy as np
import serial
from typing import Optional, Tuple, Generator, Callable, List
from dataclasses import dataclass
from enum import Enum

# Import Arduino connection utilities
from .arduino_connection import find_arduino_port


# =============================================================================
# CONFIGURATION
# =============================================================================

class IMUType(Enum):
    """Type of IMU sensor."""
    BMI160_DUAL = "bmi160_dual"  # Dual BMI160 (requires Mahony filter)
    BNO085_SINGLE = "bno085_single"  # Single BNO085 (built-in sensor fusion)


@dataclass
class IMUConfig:
    """Configuration for IMU acquisition (supports BMI160 and BNO085)."""
    
    # IMU type
    imu_type: IMUType = IMUType.BMI160_DUAL
    
    # Serial settings
    port: Optional[str] = None  # None = auto-detect
    baud: int = 230400  # 230400 for BMI160, 115200 for BNO085
    timeout: float = 0.25
    
    # Scaling factors (BMI160 only)
    accel_scale: float = 1.0 / 16384.0  # ±2g range
    gyro_scale: float = 1.0 / 131.2     # ±250°/s range
    
    # Filter settings (BMI160 only - BNO085 has built-in fusion)
    use_mahony: bool = True
    kp: float = 2.0  # Mahony proportional gain
    ki: float = 0.01  # Mahony integral gain
    adaptive_gains: bool = True
    
    # Calibration (BMI160 only)
    gyro_still_threshold: float = 0.5  # rad/s
    accel_gate_min: float = 0.92  # g
    accel_gate_max: float = 1.08  # g
    
    # BNO085 calibration settings
    bno085_calib_samples: int = 50  # Samples for reference orientation
    
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
    """Raw sample from Arduino (integer values for BMI160)."""
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
class QuaternionSample:
    """Raw quaternion sample from BNO085."""
    t_ms: int
    qw: float
    qx: float
    qy: float
    qz: float


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
class IMUCalibration:
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


def quat_inv(q: np.ndarray) -> np.ndarray:
    """Quaternion inverse (for unit quaternions)."""
    return quat_conj(q)


def rotate_vec_by_quat(v: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Rotate 3D vector by quaternion."""
    vq = np.array([0.0, v[0], v[1], v[2]], dtype=float)
    return quat_mul(quat_mul(q, vq), quat_conj(q))[1:]


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

class MahonyIMU:
    """
    Mahony AHRS filter for IMU orientation estimation.
    
    Superior to Madgwick for this application:
    - Separate proportional/integral gains
    - Faster computation
    - Better gravity constraint handling
    - Adaptive gain scheduling
    """
    
    def __init__(self, kp: float = 2.0, ki: float = 0.01, adaptive: bool = True):
        """
        Initialize Mahony filter.
        
        Args:
            kp: Proportional gain (accel correction strength)
            ki: Integral gain (gyro bias learning)
            adaptive: Enable adaptive gain scheduling
        """
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
    
    def get_quaternion(self) -> np.ndarray:
        """Get current orientation as quaternion [w, x, y, z]."""
        return self.q.copy()
    
    def get_euler(self) -> Tuple[float, float, float]:
        """Get current orientation as Euler angles (roll, pitch, yaw) in degrees."""
        return quat_to_euler(self.q)
    
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


def parse_bno085_line(line: str) -> Optional[QuaternionSample]:
    """
    Parse BNO085 CSV line: t_ms,qw,qx,qy,qz
    
    Returns:
        QuaternionSample if valid, else None
    """
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    
    parts = line.split(",")
    if len(parts) != 5:
        return None
    
    try:
        t_ms = int(parts[0])
        qw = float(parts[1])
        qx = float(parts[2])
        qy = float(parts[3])
        qz = float(parts[4])
        
        # Validate quaternion
        q_norm = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if q_norm < 0.1 or q_norm > 2.0:
            return None
        
        return QuaternionSample(t_ms=t_ms, qw=qw, qx=qx, qy=qy, qz=qz)
    except (ValueError, IndexError):
        return None


# =============================================================================
# SERIAL CONNECTION
# =============================================================================

# =============================================================================
# MAIN IMU DEVICE CLASS
# =============================================================================

class IMUDevice:
    """
    Main interface for IMU acquisition (supports BMI160 and BNO085).
    
    Handles serial communication, data parsing, calibration, and orientation
    estimation for:
    - Dual BMI160 IMUs (with Mahony filter)
    - Single BNO085 IMU (built-in sensor fusion)
    
    Example:
        ```python
        # BMI160 (dual IMU)
        config = IMUConfig(imu_type=IMUType.BMI160_DUAL)
        with IMUDevice(config) as imu:
            imu.calibrate(samples=200)
            for reading in imu.read_stream(max_samples=1000):
                print(f"Euler1: {reading.euler1}")
                print(f"Euler2: {reading.euler2}")
        
        # BNO085 (single IMU)
        config = IMUConfig(imu_type=IMUType.BNO085_SINGLE, baud=115200)
        with IMUDevice(config) as imu:
            imu.calibrate()  # Captures reference orientation
            for reading in imu.read_stream(max_samples=1000):
                print(f"Euler1: {reading.euler1}")
        ```
    """
    
    def __init__(self, config: Optional[IMUConfig] = None):
        """
        Initialize IMU device interface.
        
        Args:
            config: IMUConfig object (uses defaults if None)
        """
        self.config = config or IMUConfig()
        self.ser: Optional[serial.Serial] = None
        self.calibration: Optional[IMUCalibration] = None
        self.last_rx_time = 0.0
        self.t_prev = None
        
        # BNO085-specific: reference quaternion for relative orientation
        self.q_reference_inv: Optional[np.ndarray] = None
        
        # Filters (BMI160 only)
        if self.config.imu_type == IMUType.BMI160_DUAL and self.config.use_mahony:
            self.filter1 = MahonyIMU(
                kp=self.config.kp,
                ki=self.config.ki,
                adaptive=self.config.adaptive_gains
            )
            self.filter2 = MahonyIMU(
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
        Connect to Arduino serial port using arduino_connection module.
        
        Returns:
            True if connection successful
        """
        port = self.config.port or find_arduino_port(preferred_substr="usbmodem", verbose=True)
        if not port:
            raise RuntimeError("No serial port found. Please specify port in config.")
        
        if self.config.port is None:
            print(f"Auto-detected port: {port}")
        
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
                  callback: Optional[Callable[[int, int], None]] = None) -> IMUCalibration:
        """
        Calibrate IMU sensors.
        
        For BMI160: Calibrates gyroscope biases (IMUs must be stationary and flat)
        For BNO085: Captures reference orientation (device in known position)
        
        Args:
            samples: Number of samples to collect (BMI160: 200, BNO085: 50)
            timeout: Timeout in seconds
            callback: Optional callback(current, total) for progress
        
        Returns:
            IMUCalibration object
        """
        if self.config.imu_type == IMUType.BNO085_SINGLE:
            return self._calibrate_bno085(samples=self.config.bno085_calib_samples, 
                                         timeout=timeout, callback=callback)
        else:
            return self._calibrate_bmi160(samples=samples, timeout=timeout, callback=callback)
    
    def _calibrate_bmi160(self, samples: int, timeout: float,
                         callback: Optional[Callable[[int, int], None]]) -> IMUCalibration:
        """Calibrate BMI160 gyroscope biases."""
        print(f"\nCalibrating BMI160 ({samples} samples)...")
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
        
        self.calibration = IMUCalibration(
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
    
    def _calibrate_bno085(self, samples: int, timeout: float,
                         callback: Optional[Callable[[int, int], None]]) -> IMUCalibration:
        """Capture BNO085 reference orientation."""
        print(f"\nCalibrating BNO085 ({samples} samples)...")
        print("Place device in reference position (e.g., flat on table)")
        
        quaternions = []
        t0 = time.time()
        
        while len(quaternions) < samples:
            if time.time() - t0 > timeout:
                if len(quaternions) >= 10:
                    print(f"\nTimeout - using {len(quaternions)} samples")
                    break
                raise TimeoutError(f"Calibration failed: only {len(quaternions)} samples")
            
            raw_reading = self._read_raw()
            if raw_reading is None:
                continue
            
            # For BNO085, _read_raw returns quaternion sample
            q_sample, timestamp = raw_reading
            q = np.array([q_sample.qw, q_sample.qx, q_sample.qy, q_sample.qz])
            q = quat_norm(q)
            quaternions.append(q)
            
            if callback:
                callback(len(quaternions), samples)
            elif len(quaternions) % 10 == 0:
                print(f"  {len(quaternions)}/{samples}...", end='\r')
        
        print(f"\n✓ Reference captured ({len(quaternions)} samples)")
        
        # Average quaternions
        q_avg = np.mean(quaternions, axis=0)
        q_avg = quat_norm(q_avg)
        self.q_reference_inv = quat_inv(q_avg)
        
        print(f"  Reference: [{q_avg[0]:.3f}, {q_avg[1]:.3f}, {q_avg[2]:.3f}, {q_avg[3]:.3f}]")
        
        # Create dummy calibration object
        self.calibration = IMUCalibration(
            bias1=np.zeros(3),
            bias2=np.zeros(3),
            samples=len(quaternions)
        )
        
        return self.calibration
    
    def _read_raw(self):
        """
        Read one raw sample from serial.
        
        Returns:
            For BMI160: (RawSample, timestamp) or None
            For BNO085: (QuaternionSample, timestamp) or None
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
            
            if self.config.imu_type == IMUType.BNO085_SINGLE:
                sample = parse_bno085_line(line)
            else:
                sample = parse_line(line)
                if sample and (sample.ok1 == 0 or sample.ok2 == 0):
                    return None
            
            if sample is None:
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
                try:
                    ok = self.reconnect()
                except Exception as e:
                    print(f"⚠ Reconnect attempt failed: {e}")
                    time.sleep(self.config.reconnect_backoff)
                    return None

                if not ok:
                    time.sleep(self.config.reconnect_backoff)
                    return None

                self.t_prev = None
        
        raw_result = self._read_raw()
        if raw_result is None:
            return None
        
        sample, timestamp = raw_result
        
        # Handle BNO085 (quaternion-based)
        if self.config.imu_type == IMUType.BNO085_SINGLE:
            return self._process_bno085(sample, timestamp)
        
        # Handle BMI160 (gyro/accel-based)
        return self._process_bmi160(sample, timestamp)
    
    def _process_bno085(self, sample: QuaternionSample, timestamp: float) -> IMUReading:
        """Process BNO085 quaternion sample."""
        # Normalize quaternion
        q_raw = np.array([sample.qw, sample.qx, sample.qy, sample.qz])
        q_raw = quat_norm(q_raw)
        
        # Apply reference calibration if available
        if self.q_reference_inv is not None:
            quat1 = quat_mul(self.q_reference_inv, q_raw)
        else:
            quat1 = q_raw
        
        euler1 = quat_to_euler(quat1)
        
        # BNO085 only has one IMU
        return IMUReading(
            timestamp=timestamp,
            t_us=sample.t_ms * 1000,  # Convert ms to us
            seq=0,  # BNO085 doesn't provide sequence number
            gyro1=np.zeros(3),  # BNO085 doesn't output raw gyro
            accel1=np.zeros(3),  # BNO085 doesn't output raw accel
            gyro2=np.zeros(3),
            accel2=np.zeros(3),
            quat1=quat1,
            euler1=euler1,
            quat2=None,
            euler2=None,
            ok1=True,
            ok2=False
        )
    
    def _process_bmi160(self, sample: RawSample, timestamp: float) -> IMUReading:
        """Process BMI160 raw sample."""
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
            else:
                # Avoid tight busy-loop when no data is available (e.g., during reconnects)
                time.sleep(0.001)


# =============================================================================
# CONVENIENCE FUNCTIONS
# =============================================================================

def acquire_imu_data(port: Optional[str] = None, samples: int = 100,
                     calibrate: bool = True, calib_samples: int = 200) -> List[IMUReading]:
    """
    Quick utility to acquire IMU data.
    
    Args:
        port: Serial port (None = auto-detect)
        samples: Number of samples to read
        calibrate: Whether to calibrate first
        calib_samples: Calibration samples
    
    Returns:
        List of IMUReading objects
    """
    config = IMUConfig(port=port)
    
    with IMUDevice(config) as imu:
        if calibrate:
            imu.calibrate(samples=calib_samples)
        
        readings = []
        for reading in imu.read_stream(max_samples=samples):
            readings.append(reading)
        
        return readings


if __name__ == "__main__":
    # Demo
    print("Dual BMI160 IMU Acquisition Module Demo")
    print("="*60)
    
    config = IMUConfig()
    
    with IMUDevice(config) as imu:
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
