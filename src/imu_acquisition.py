"""
IMU Signal Acquisition Module

This module provides high-level classes and functions for acquiring data from
BMI160 dual IMU sensors via serial communication. It includes:
- Serial communication and data parsing
- Madgwick filter for orientation estimation
- Gyro bias calibration
- Convenient data acquisition interface

Author: Auto-generated from IMU_testing.py
Date: January 2026
"""

import time
import numpy as np
import serial
from typing import Optional, Tuple, List, Dict, Any
from dataclasses import dataclass


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class IMUReading:
    """Single reading from dual IMU sensors."""
    timestamp_ms: int
    imu1_gyro: np.ndarray  # [gx, gy, gz] in deg/s
    imu1_accel: np.ndarray  # [ax, ay, az] in g
    imu2_gyro: np.ndarray  # [gx, gy, gz] in deg/s
    imu2_accel: np.ndarray  # [ax, ay, az] in g
    
    @property
    def timestamp_s(self) -> float:
        """Timestamp in seconds."""
        return self.timestamp_ms / 1000.0


@dataclass
class IMUCalibration:
    """Calibration data for IMU sensors."""
    gyro_bias_imu1: np.ndarray  # [gx, gy, gz] bias in deg/s
    gyro_bias_imu2: np.ndarray  # [gx, gy, gz] bias in deg/s
    num_samples: int
    
    def __str__(self):
        return (f"IMU Calibration:\n"
                f"  IMU1 Gyro Bias: [{self.gyro_bias_imu1[0]:.2f}, "
                f"{self.gyro_bias_imu1[1]:.2f}, {self.gyro_bias_imu1[2]:.2f}] deg/s\n"
                f"  IMU2 Gyro Bias: [{self.gyro_bias_imu2[0]:.2f}, "
                f"{self.gyro_bias_imu2[1]:.2f}, {self.gyro_bias_imu2[2]:.2f}] deg/s\n"
                f"  Samples: {self.num_samples}")


# =============================================================================
# QUATERNION UTILITIES
# =============================================================================

def quat_mul(q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Multiply two quaternions (Hamilton product)."""
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
    """Normalize quaternion to unit length."""
    n = np.linalg.norm(q)
    if n == 0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n


def quat_inv(q: np.ndarray) -> np.ndarray:
    """Quaternion inverse (for unit quaternions, equals conjugate)."""
    return quat_conj(q)


def rotate_vec_by_quat(v: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Rotate a 3D vector by a quaternion."""
    vq = np.array([0.0, v[0], v[1], v[2]], dtype=float)
    return quat_mul(quat_mul(q, vq), quat_conj(q))[1:]


def quat_to_euler(q: np.ndarray) -> Tuple[float, float, float]:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw) in radians.
    
    Returns:
        (roll, pitch, yaw) in radians
    """
    w, x, y, z = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


# =============================================================================
# MADGWICK FILTER
# =============================================================================

class MadgwickIMU:
    """
    Madgwick AHRS algorithm for IMU orientation estimation.
    
    Fuses gyroscope and accelerometer data to estimate orientation as a quaternion.
    """
    
    def __init__(self, beta: float = 0.1):
        """
        Initialize Madgwick filter.
        
        Args:
            beta: Filter gain (typically 0.01 to 0.3). Higher values = faster
                  convergence but more noise. Lower values = smoother but slower.
        """
        self.beta = beta
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)  # Identity quaternion
    
    def update(self, gx: float, gy: float, gz: float,
               ax: float, ay: float, az: float,
               dt: float, use_accel: bool = True) -> np.ndarray:
        """
        Update orientation estimate with new sensor data.
        
        Args:
            gx, gy, gz: Gyroscope readings in rad/s
            ax, ay, az: Accelerometer readings in g
            dt: Time delta in seconds
            use_accel: Whether to use accelerometer for correction (disable during high acceleration)
        
        Returns:
            Updated quaternion [w, x, y, z]
        """
        if dt <= 0:
            return self.q
        
        # Always integrate gyro
        q_dot_omega = 0.5 * quat_mul(self.q, np.array([0.0, gx, gy, gz], dtype=float))
        
        if not use_accel:
            self.q = quat_norm(self.q + q_dot_omega * dt)
            return self.q
        
        # Normalize accelerometer
        a = np.array([ax, ay, az], dtype=float)
        an = np.linalg.norm(a)
        if an == 0:
            self.q = quat_norm(self.q + q_dot_omega * dt)
            return self.q
        a /= an
        ax, ay, az = a
        
        q1, q2, q3, q4 = self.q
        
        # Objective function and Jacobian
        f1 = 2.0*(q2*q4 - q1*q3) - ax
        f2 = 2.0*(q1*q2 + q3*q4) - ay
        f3 = 2.0*(0.5 - q2*q2 - q3*q3) - az
        
        J_11 = -2.0*q3
        J_12 =  2.0*q4
        J_13 = -2.0*q1
        J_14 =  2.0*q2
        
        J_21 =  2.0*q2
        J_22 =  2.0*q1
        J_23 =  2.0*q4
        J_24 =  2.0*q3
        
        J_31 =  0.0
        J_32 = -4.0*q2
        J_33 = -4.0*q3
        J_34 =  0.0
        
        grad = np.array([
            J_11*f1 + J_21*f2 + J_31*f3,
            J_12*f1 + J_22*f2 + J_32*f3,
            J_13*f1 + J_23*f2 + J_33*f3,
            J_14*f1 + J_24*f2 + J_34*f3
        ], dtype=float)
        
        grad_n = np.linalg.norm(grad)
        if grad_n != 0:
            grad /= grad_n
        
        q_dot = q_dot_omega - self.beta * grad
        self.q = quat_norm(self.q + q_dot * dt)
        return self.q
    
    def get_quaternion(self) -> np.ndarray:
        """Get current orientation as quaternion [w, x, y, z]."""
        return self.q.copy()
    
    def get_euler(self) -> Tuple[float, float, float]:
        """Get current orientation as Euler angles (roll, pitch, yaw) in radians."""
        return quat_to_euler(self.q)
    
    def reset(self):
        """Reset to identity orientation."""
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)


# =============================================================================
# SERIAL PARSING
# =============================================================================

def parse_imu_line(line: str, debug: bool = False) -> Optional[IMUReading]:
    """
    Parse a line of IMU data from serial.
    
    Expected format: "t_ms | gx1 gy1 gz1 ax1 ay1 az1 | gx2 gy2 gz2 ax2 ay2 az2"
    
    Args:
        line: Raw line from serial
        debug: Print debug messages on parse failures
    
    Returns:
        IMUReading object or None if parsing fails
    """
    if not line or line.startswith("t_ms"):
        return None
    
    try:
        parts = line.split("|")
        if len(parts) != 3:
            if debug:
                print(f"  Parse fail: Expected 3 parts, got {len(parts)}: '{line[:100]}'")
            return None
        
        t_ms = int(parts[0].strip())
        imu1 = [float(x) for x in parts[1].split()]
        imu2 = [float(x) for x in parts[2].split()]
        
        if len(imu1) != 6 or len(imu2) != 6:
            if debug:
                print(f"  Parse fail: IMU1 has {len(imu1)} values, IMU2 has {len(imu2)} values (expected 6 each)")
            return None
        
        return IMUReading(
            timestamp_ms=t_ms,
            imu1_gyro=np.array(imu1[:3], dtype=float),
            imu1_accel=np.array(imu1[3:], dtype=float),
            imu2_gyro=np.array(imu2[:3], dtype=float),
            imu2_accel=np.array(imu2[3:], dtype=float)
        )
    
    except (ValueError, IndexError) as e:
        if debug:
            print(f"  Parse exception: {e} on line: '{line[:100]}'")
        return None


# =============================================================================
# IMU DEVICE CLASS
# =============================================================================

class IMUDevice:
    """
    High-level interface for dual BMI160 IMU sensors via serial.
    
    This class handles:
    - Serial connection management
    - Data parsing
    - Gyro bias calibration
    - Optional Madgwick orientation filtering
    """
    
    def __init__(self, port: str, baud: int = 115200,
                 auto_reconnect: bool = True,
                 stall_timeout_s: float = 3.0):
        """
        Initialize IMU device.
        
        Args:
            port: Serial port (e.g., "COM3" or "/dev/ttyUSB0")
            baud: Baud rate
            auto_reconnect: Automatically reconnect on serial errors
            stall_timeout_s: Reconnect if no data received for this duration
        """
        self.port = port
        self.baud = baud
        self.auto_reconnect = auto_reconnect
        self.stall_timeout_s = stall_timeout_s
        
        self._ser: Optional[serial.Serial] = None
        self._last_rx_time: float = 0
        self._calibration: Optional[IMUCalibration] = None
        
        # Optional Madgwick filters
        self._filter1: Optional[MadgwickIMU] = None
        self._filter2: Optional[MadgwickIMU] = None
        self._last_timestamp_s: Optional[float] = None
    
    def connect(self):
        """Open serial connection to IMU device."""
        print(f"Opening serial port {self.port} @ {self.baud}...")
        self._ser = serial.Serial(self.port, self.baud, timeout=1)
        
        # Arduino often resets on open; give it a moment
        time.sleep(2.0)
        
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass
        
        self._last_rx_time = time.time()
        print(f"[OK] Connected to {self.port}")
    
    def disconnect(self):
        """Close serial connection."""
        if self._ser is not None:
            try:
                self._ser.close()
                print(f"[OK] Disconnected from {self.port}")
            except Exception as e:
                print(f"[WARN] Error closing serial: {e}")
            finally:
                self._ser = None
    
    def is_connected(self) -> bool:
        """Check if serial connection is open."""
        return self._ser is not None and self._ser.is_open
    
    def _reconnect(self):
        """Internal method to reconnect serial port."""
        print(f"\n[WARN] Reconnecting to {self.port}...")
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
        
        time.sleep(1.0)
        self.connect()
        self._last_timestamp_s = None  # Reset timestamp tracking
    
    def read_sample(self, debug: bool = False) -> Optional[IMUReading]:
        """
        Read a single sample from the IMU device.
        
        Args:
            debug: Print debug messages on parse failures
        
        Returns:
            IMUReading object or None if no valid data available
        """
        if not self.is_connected():
            raise RuntimeError("Not connected. Call connect() first.")
        
        # Check for serial stall
        if self.auto_reconnect and (time.time() - self._last_rx_time) > self.stall_timeout_s:
            print(f"\n[WARN] Serial stalled for > {self.stall_timeout_s:.1f}s")
            self._reconnect()
            return None
        
        try:
            raw = self._ser.readline().decode("utf-8", errors="ignore").strip()
            
            if raw:
                self._last_rx_time = time.time()
            
            reading = parse_imu_line(raw, debug=debug)
            return reading
        
        except serial.SerialException as e:
            print(f"\n[WARN] SerialException: {e}")
            if self.auto_reconnect:
                self._reconnect()
            else:
                raise
            return None
        
        except Exception as e:
            print(f"\n[WARN] Serial read error: {e}")
            if self.auto_reconnect:
                self._reconnect()
            else:
                raise
            return None
    
    def calibrate_gyro(self, num_samples: int = 150, timeout_s: float = 60.0) -> IMUCalibration:
        """
        Calibrate gyroscope bias.
        
        IMPORTANT: IMUs must be laying FLAT and STILL during calibration.
        
        Args:
            num_samples: Number of samples to collect
            timeout_s: Maximum time to wait for calibration
        
        Returns:
            IMUCalibration object with bias values
        """
        if not self.is_connected():
            raise RuntimeError("Not connected. Call connect() first.")
        
        print("\n" + "="*60)
        print("GYRO CALIBRATION")
        print("="*60)
        print("IMPORTANT: Lay BOTH IMUs FLAT on a surface and keep them STILL")
        print(f"Collecting {num_samples} samples...")
        print()
        
        g1_samples = []
        g2_samples = []
        t0 = time.time()
        
        while len(g1_samples) < num_samples:
            if time.time() - t0 > timeout_s:
                if len(g1_samples) >= 50:
                    print(f"\n[WARN] Timeout - using {len(g1_samples)} samples")
                    break
                else:
                    raise RuntimeError(f"Calibration failed: only {len(g1_samples)} samples collected")
            
            reading = self.read_sample(debug=False)
            if reading is None:
                continue
            
            g1_samples.append(reading.imu1_gyro)
            g2_samples.append(reading.imu2_gyro)
            
            if len(g1_samples) % 10 == 0:
                progress = len(g1_samples) / num_samples
                print(f"  Progress: {len(g1_samples)}/{num_samples} ({progress*100:.1f}%)", end='\r')
        
        print()
        print(f"[OK] Calibration complete: {len(g1_samples)} samples collected")
        
        # Calculate gyro bias
        g1_arr = np.array(g1_samples, dtype=float)
        g2_arr = np.array(g2_samples, dtype=float)
        
        bias1 = np.mean(g1_arr, axis=0)
        bias2 = np.mean(g2_arr, axis=0)
        
        self._calibration = IMUCalibration(
            gyro_bias_imu1=bias1,
            gyro_bias_imu2=bias2,
            num_samples=len(g1_samples)
        )
        
        print(self._calibration)
        print("="*60)
        
        return self._calibration
    
    def get_calibration(self) -> Optional[IMUCalibration]:
        """Get current calibration data."""
        return self._calibration
    
    def set_calibration(self, calibration: IMUCalibration):
        """Set calibration data (e.g., from a saved file)."""
        self._calibration = calibration
    
    def enable_orientation_filter(self, beta: float = 0.08):
        """
        Enable Madgwick orientation filtering.
        
        Args:
            beta: Filter gain (0.01-0.3). Higher = faster convergence, more noise.
        """
        self._filter1 = MadgwickIMU(beta=beta)
        self._filter2 = MadgwickIMU(beta=beta)
        print(f"[OK] Orientation filters enabled (beta={beta})")
    
    def disable_orientation_filter(self):
        """Disable orientation filtering."""
        self._filter1 = None
        self._filter2 = None
        self._last_timestamp_s = None
        print("[OK] Orientation filters disabled")
    
    def get_orientation(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Get current orientation estimates as quaternions.
        
        Returns:
            (imu1_quaternion, imu2_quaternion) or None if filters not enabled
        """
        if self._filter1 is None or self._filter2 is None:
            return None
        return self._filter1.get_quaternion(), self._filter2.get_quaternion()
    
    def get_euler_angles(self) -> Optional[Tuple[Tuple[float, float, float], 
                                                   Tuple[float, float, float]]]:
        """
        Get current orientation estimates as Euler angles.
        
        Returns:
            ((imu1_roll, imu1_pitch, imu1_yaw), (imu2_roll, imu2_pitch, imu2_yaw))
            or None if filters not enabled. All angles in radians.
        """
        if self._filter1 is None or self._filter2 is None:
            return None
        return self._filter1.get_euler(), self._filter2.get_euler()
    
    def read_sample_with_orientation(self, 
                                      apply_calibration: bool = True,
                                      accel_gate_min_g: float = 0.85,
                                      accel_gate_max_g: float = 1.15) -> Optional[Dict[str, Any]]:
        """
        Read a sample and update orientation filters (if enabled).
        
        Args:
            apply_calibration: Subtract gyro bias from readings
            accel_gate_min_g: Min accelerometer magnitude (in g) to trust for correction
            accel_gate_max_g: Max accelerometer magnitude (in g) to trust for correction
        
        Returns:
            Dictionary with keys:
                - 'reading': IMUReading object
                - 'imu1_quat': IMU1 quaternion (if filters enabled)
                - 'imu2_quat': IMU2 quaternion (if filters enabled)
                - 'imu1_euler': IMU1 Euler angles in radians (if filters enabled)
                - 'imu2_euler': IMU2 Euler angles in radians (if filters enabled)
        """
        reading = self.read_sample()
        if reading is None:
            return None
        
        result = {'reading': reading}
        
        # Update orientation filters if enabled
        if self._filter1 is not None and self._filter2 is not None:
            # Calculate dt
            t_s = reading.timestamp_s
            if self._last_timestamp_s is None:
                self._last_timestamp_s = t_s
                return result
            
            dt = t_s - self._last_timestamp_s
            if dt <= 0 or dt > 0.2:
                self._last_timestamp_s = t_s
                return result
            self._last_timestamp_s = t_s
            
            # Apply calibration
            gyro1 = reading.imu1_gyro.copy()
            gyro2 = reading.imu2_gyro.copy()
            
            if apply_calibration and self._calibration is not None:
                gyro1 -= self._calibration.gyro_bias_imu1
                gyro2 -= self._calibration.gyro_bias_imu2
            
            # Convert gyro from deg/s to rad/s
            gyro1_rad = np.deg2rad(gyro1)
            gyro2_rad = np.deg2rad(gyro2)
            
            # Accelerometer gating (only use accel correction if magnitude near 1g)
            a1_mag = np.linalg.norm(reading.imu1_accel)
            a2_mag = np.linalg.norm(reading.imu2_accel)
            use_accel1 = accel_gate_min_g <= a1_mag <= accel_gate_max_g
            use_accel2 = accel_gate_min_g <= a2_mag <= accel_gate_max_g
            
            # Update filters
            q1 = self._filter1.update(
                gyro1_rad[0], gyro1_rad[1], gyro1_rad[2],
                reading.imu1_accel[0], reading.imu1_accel[1], reading.imu1_accel[2],
                dt, use_accel=use_accel1
            )
            q2 = self._filter2.update(
                gyro2_rad[0], gyro2_rad[1], gyro2_rad[2],
                reading.imu2_accel[0], reading.imu2_accel[1], reading.imu2_accel[2],
                dt, use_accel=use_accel2
            )
            
            result['imu1_quat'] = q1
            result['imu2_quat'] = q2
            result['imu1_euler'] = quat_to_euler(q1)
            result['imu2_euler'] = quat_to_euler(q2)
        
        return result


# =============================================================================
# CONVENIENCE FUNCTIONS
# =============================================================================

def acquire_imu_data(port: str,
                     duration: float = 10.0,
                     baud: int = 115200,
                     calibrate: bool = True,
                     enable_filter: bool = False,
                     filter_beta: float = 0.08) -> Tuple[List[IMUReading], Optional[IMUCalibration]]:
    """
    Convenience function to acquire IMU data for a specified duration.
    
    Args:
        port: Serial port
        duration: Acquisition duration in seconds
        baud: Baud rate
        calibrate: Perform gyro calibration before acquisition
        enable_filter: Enable Madgwick orientation filtering
        filter_beta: Madgwick filter gain
    
    Returns:
        (list_of_readings, calibration_data)
    """
    device = None
    readings = []
    calibration = None
    
    try:
        # Connect
        device = IMUDevice(port, baud=baud)
        device.connect()
        
        # Flush initial data
        print("\n[OK] Flushing initial serial data...")
        time.sleep(0.5)
        
        # Calibrate if requested
        if calibrate:
            calibration = device.calibrate_gyro(num_samples=150)
        
        # Enable filter if requested
        if enable_filter:
            device.enable_orientation_filter(beta=filter_beta)
        
        # Acquire data
        print(f"\n[OK] Starting {duration}s data acquisition...")
        print("="*60)
        
        start_time = time.time()
        sample_count = 0
        
        while (time.time() - start_time) < duration:
            if enable_filter:
                sample_data = device.read_sample_with_orientation(apply_calibration=calibrate)
                if sample_data is not None:
                    readings.append(sample_data)
                    sample_count += 1
            else:
                reading = device.read_sample()
                if reading is not None:
                    readings.append(reading)
                    sample_count += 1
            
            if sample_count % 50 == 0:
                elapsed = time.time() - start_time
                rate = sample_count / elapsed if elapsed > 0 else 0
                print(f"  [{elapsed:6.2f}s] Samples: {sample_count} ({rate:.1f} Hz)", end='\r')
        
        elapsed = time.time() - start_time
        rate = sample_count / elapsed if elapsed > 0 else 0
        print(f"\n[OK] Acquisition complete!")
        print(f"  Duration: {elapsed:.2f}s")
        print(f"  Samples: {sample_count}")
        print(f"  Rate: {rate:.1f} Hz")
        print("="*60)
        
        return readings, calibration
    
    except KeyboardInterrupt:
        print("\n\n[WARN] Acquisition interrupted by user")
        return readings, calibration
    
    except Exception as e:
        print(f"\n[ERROR] Acquisition failed: {e}")
        raise
    
    finally:
        if device is not None:
            device.disconnect()


# =============================================================================
# EXAMPLE USAGE
# =============================================================================

if __name__ == "__main__":
    # Example: Simple data acquisition
    print("="*60)
    print("IMU Data Acquisition Example")
    print("="*60)
    
    # IMPORTANT: Update this to your serial port
    PORT = "/dev/cu.usbmodem9888E00A0CC02"  # macOS/Linux
    # PORT = "COM3"  # Windows
    
    print("\nIMPORTANT: Before starting:")
    print("  1. Connect your dual IMU device via USB")
    print("  2. Place both IMUs FLAT on a surface for calibration")
    print("  3. Keep them STILL during the calibration phase")
    print("\nStarting in 3 seconds...\n")
    time.sleep(3)
    
    # Acquire 10 seconds of data with calibration and orientation filtering
    readings, calibration = acquire_imu_data(
        port=PORT,
        duration=10.0,
        calibrate=True,
        enable_filter=True,
        filter_beta=0.08
    )
    
    if readings:
        print(f"\n[OK] Acquired {len(readings)} samples")
        
        # Show first sample
        print("\nFirst sample:")
        first = readings[0]
        if isinstance(first, dict):
            reading = first['reading']
            print(f"  Timestamp: {reading.timestamp_ms} ms")
            print(f"  IMU1 Gyro: {reading.imu1_gyro}")
            print(f"  IMU1 Accel: {reading.imu1_accel}")
            print(f"  IMU2 Gyro: {reading.imu2_gyro}")
            print(f"  IMU2 Accel: {reading.imu2_accel}")
            if 'imu1_euler' in first:
                euler1 = np.rad2deg(first['imu1_euler'])
                euler2 = np.rad2deg(first['imu2_euler'])
                print(f"  IMU1 Euler (deg): Roll={euler1[0]:.1f}, Pitch={euler1[1]:.1f}, Yaw={euler1[2]:.1f}")
                print(f"  IMU2 Euler (deg): Roll={euler2[0]:.1f}, Pitch={euler2[1]:.1f}, Yaw={euler2[2]:.1f}")
        else:
            print(f"  Timestamp: {first.timestamp_ms} ms")
            print(f"  IMU1 Gyro: {first.imu1_gyro}")
            print(f"  IMU1 Accel: {first.imu1_accel}")
    else:
        print("\n[WARN] No data acquired")
