"""
Optimized Dual BMI160 IMU Tracking - ASCII Protocol
====================================================

High-performance IMU orientation tracking with:
- ASCII serial protocol (230400 baud, 200Hz)
- Robust checksum validation and auto-reconnect
- Mahony filter with gravity-aligned correction
- Adaptive gain scheduling for drift reduction
- Real-time performance monitoring
- Verified coordinate frame mapping

Hardware Requirements:
- Arduino Uno R4 with dual BMI160 IMUs
- USB connection
- Upload arduino/IMU_sketch.ino to Arduino first

Coordinate Frame Reference:
┌─────────────┐
│   BMI160    │  ← Component side UP (when calibrating)
│   [●]       │  ← Dot marks +X direction (FORWARD)
│             │
│      Y↑     │
│      │      │
│      └─→X   │  Z points up (out of screen)
└─────────────┘

When IMU is flat on table (component side up):
  - Accel reads: ~(0, 0, -1)g (gravity points down in sensor -Z)
  - Tilt forward (+pitch) → Dice tilts forward
  - Roll right (+roll) → Dice rolls right
  - Rotate CW (yaw) → Dice rotates CW (but will drift without magnetometer)
"""

import time
import numpy as np
import serial
import sys
import csv
import argparse
from pathlib import Path
from datetime import datetime
from collections import deque
from dataclasses import dataclass
from typing import Optional
from vpython import canvas, box, vector, color, rate, arrow, label, sphere, compound

# Allow running this file directly from anywhere (e.g. via absolute path) while
# still being able to import the project's top-level `src/` package.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# Import Arduino connection utilities
from src.arduino_connection import find_arduino_port, open_arduino_serial, list_all_ports

# ============================================================================
# COMMAND LINE ARGUMENTS
# ============================================================================
def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description='Dual BMI160 IMU Tracker with real-time visualization and data logging'
    )
    parser.add_argument('--port', type=str, default="auto",
                        help='Serial port (or "auto" for auto-detect, default: auto)')
    parser.add_argument('--baud', type=int, default=230400,
                        help='Serial baud rate (default: 230400)')
    parser.add_argument('--csv', type=str, default=None,
                        help='CSV file path for data logging (optional)')
    parser.add_argument('--csv-raw', action='store_true',
                        help='Log raw integer values instead of scaled values')
    parser.add_argument('--no-viz', action='store_true',
                        help='Disable visualization (CSV logging only)')
    parser.add_argument('--accel-scale', type=float, default=1.0/16384.0,
                        help='Accelerometer scale factor (default: 1/16384 for ±2g)')
    parser.add_argument('--gyro-scale', type=float, default=1.0/131.2,
                        help='Gyroscope scale factor (default: 1/131.2 for ±250°/s)')
    return parser.parse_args()

args = parse_args()

# ============================================================================
# USER SETTINGS
# ============================================================================
PORT = None if args.port.lower() == "auto" else args.port
BAUD = args.baud

# Serial robustness
AUTO_RECONNECT = True
STALL_TIMEOUT_S = 3.0
RECONNECT_BACKOFF_S = 1.0

# BMI160 scaling factors (adjust if your Arduino uses different ranges)
ACCEL_SCALE = args.accel_scale  # Convert raw int to g
GYRO_SCALE = args.gyro_scale     # Convert raw int to deg/s

# Data logging
CSV_FILE = args.csv
LOG_RAW_VALUES = args.csv_raw
ENABLE_VISUALIZATION = not args.no_viz

# Mahony filter tuning
KP_BASE = 2.0           # Base proportional gain (accel correction strength)
KI = 0.01               # Integral gain (gyro bias learning)
ADAPTIVE_GAINS = True   # Enable adaptive gain scheduling

# Calibration
CALIB_SAMPLES = 200            # Samples for initial gyro bias calibration
GYRO_STILL_THRESHOLD = 0.5     # rad/s - below this is "still"
ACCEL_GATE_MIN_G = 0.92        # Only trust accel if magnitude in this range
ACCEL_GATE_MAX_G = 1.08

# Visualization
SHOW_RELATIVE_IMU2_TO_IMU1 = False  # False = absolute, True = relative
SHOW_AXES = True                     # Show RGB axes on dice
SHOW_PERFORMANCE_STATS = True        # Show FPS, latency, drift metrics

# Axis remapping (CRITICAL for “dice matches real IMU”)
# -----------------------------------------------------
# Many BMI160 breakout boards have axes that do not match your assumed frame.
# These maps are applied to BOTH gyro and accel before filtering/visualization.
#
# Format: (out_x, out_y, out_z) where each element is ±1..±3 selecting
# (±in_x, ±in_y, ±in_z). Example:
#   (1, 2, 3)   -> identity
#   (1, -2, -3) -> flip Y and Z
#   (2, -1, 3)  -> swap X/Y and flip new Y
IMU1_AXIS_MAP = (1, 2, 3)
IMU2_AXIS_MAP = (1, 2, 3)

# If your “flat on table” accel Z is negative (common if sensor is mounted
# component-side-down), set this to True to invert accel vector.
INVERT_ACCEL = False

# Read/compute: Arduino sends 200Hz but VPython renders ~60-120Hz.
# We must drain multiple packets per frame to avoid lag and desync.
MAX_PACKETS_PER_FRAME = 8

# ============================================================================
# ASCII PROTOCOL - Must match Arduino
#
# Format: $seq,us,ax1,ay1,az1,gx1,gy1,gz1,ok1,ax2,ay2,az2,gx2,gy2,gz2,ok2*HH
# - Syncs on '$'
# - Verifies XOR checksum
# ============================================================================

@dataclass
class Sample:
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

def xor_checksum(payload: str) -> int:
    """Calculate XOR checksum for ASCII payload"""
    c = 0
    for ch in payload:
        c ^= ord(ch) & 0xFF
    return c

def parse_line(line: str) -> Optional[Sample]:
    """
    Parse ASCII line: $seq,us,ax1,ay1,az1,gx1,gy1,gz1,ok1,ax2,ay2,az2,gx2,gy2,gz2,ok2*HH
    Returns Sample if valid, else None.
    """
    line = line.strip()
    if not line or not line.startswith("$"):
        return None

    star = line.rfind("*")
    if star == -1 or star < 2:
        return None

    payload = line[1:star]          # without '$'
    chk_hex = line[star + 1:]       # after '*'
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
    # Arduino format string has 17 specifiers but only 16 args (bug in Arduino code)
    # Accept either 16 or 17 parts to handle both cases
    if len(parts) not in (16, 17):
        return None

    try:
        vals = list(map(int, parts))
    except ValueError:
        return None

    return Sample(
        seq=vals[0],
        t_us=vals[1],
        ax1=vals[2], ay1=vals[3], az1=vals[4],
        gx1=vals[5], gy1=vals[6], gz1=vals[7],
        ok1=vals[8],
        ax2=vals[9], ay2=vals[10], az2=vals[11],
        gx2=vals[12], gy2=vals[13], gz2=vals[14],
        ok2=vals[15],
        # vals[16] is ignored if present (garbage from Arduino format bug)
    )

def remap_vec(v, mapping):
    """Remap 3-vector with signed axis mapping tuple."""
    v = np.asarray(v, dtype=float)
    out = np.empty(3, dtype=float)
    for i, a in enumerate(mapping):
        s = 1.0 if a > 0 else -1.0
        idx = abs(int(a)) - 1
        out[i] = s * v[idx]
    return out

def read_imu_packet(ser, debug=False, return_sample=False):
    """
    Read and parse ASCII IMU packet.
    
    Args:
        ser: Serial port object
        debug: Print debug messages
        return_sample: If True, return (data_tuple, sample_obj), else just data_tuple
    
    Returns:
        If return_sample=False: (timestamp_ms, imu1_gyro_deg, imu1_accel_g, imu2_gyro_deg, imu2_accel_g) or None
        If return_sample=True: ((timestamp_ms, ...), sample) or None
    """
    try:
        raw = ser.readline()
        if not raw:
            return None
        
        try:
            line = raw.decode("ascii", errors="ignore")
        except Exception:
            return None
        
        # Ignore comment/status lines from Arduino
        if line.startswith("#"):
            if debug:
                print(f"  Status: {line.strip()}")
            return None
        
        sample = parse_line(line)
        if sample is None:
            if debug and line.strip():
                print(f"  Parse failed: {line.strip()[:80]}")
            return None
        
        # Check IMU status flags
        if sample.ok1 == 0 or sample.ok2 == 0:
            # IMU not ready or error
            return None
        
        # Convert raw integers to physical units
        timestamp_ms = sample.t_us / 1000.0  # Convert microseconds to milliseconds
        
        imu1_accel = np.array([sample.ax1, sample.ay1, sample.az1], dtype=float) * ACCEL_SCALE
        imu1_gyro = np.array([sample.gx1, sample.gy1, sample.gz1], dtype=float) * GYRO_SCALE
        
        imu2_accel = np.array([sample.ax2, sample.ay2, sample.az2], dtype=float) * ACCEL_SCALE
        imu2_gyro = np.array([sample.gx2, sample.gy2, sample.gz2], dtype=float) * GYRO_SCALE
        
        data = (timestamp_ms, imu1_gyro, imu1_accel, imu2_gyro, imu2_accel)
        
        if return_sample:
            return data, sample
        else:
            return data
        
    except Exception as e:
        return None

# ============================================================================
# QUATERNION MATH
# ============================================================================
def quat_mul(q, r):
    """Quaternion multiplication"""
    w0, x0, y0, z0 = q
    w1, x1, y1, z1 = r
    return np.array([
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1
    ], dtype=float)

def quat_conj(q):
    """Quaternion conjugate"""
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)

def quat_norm(q):
    """Normalize quaternion"""
    n = np.linalg.norm(q)
    if n < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n

def quat_inv(q):
    """Quaternion inverse (for unit quaternions)"""
    return quat_conj(q)

def rotate_vec_by_quat(v, q):
    """Rotate 3D vector by quaternion"""
    vq = np.array([0.0, v[0], v[1], v[2]], dtype=float)
    return quat_mul(quat_mul(q, vq), quat_conj(q))[1:]

def quat_to_euler(q):
    """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees"""
    w, x, y, z = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

# ============================================================================
# MAHONY FILTER - Gravity-aligned orientation estimation
# ============================================================================
class MahonyIMU:
    """
    Mahony filter with adaptive gains for gravity-aligned tracking.
    
    Superior to Madgwick for this application:
    - Separate proportional/integral gains
    - Faster computation (~30% less math)
    - Better gravity constraint handling
    - Adaptive gain scheduling
    """
    
    def __init__(self, kp=2.0, ki=0.01):
        self.kp = kp  # Proportional gain (accel correction)
        self.ki = ki  # Integral gain (bias learning)
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        # Integral term for bias compensation (in rad/s)
        self.integral_error = np.zeros(3, dtype=float)
        
        # Statistics for adaptive tuning
        self.gyro_magnitude_avg = 0.0
        self.accel_magnitude_avg = 1.0
    
    def set_bias(self, bias):
        """Deprecated (bias is handled outside filter)."""
        return
    
    def compute_adaptive_kp(self, gyro_mag, accel_mag):
        """
        Adaptive gain scheduling:
        - High kp when still (trust accelerometer more)
        - Low kp during motion (trust gyro more)
        """
        if not ADAPTIVE_GAINS:
            return self.kp
        
        is_still = (gyro_mag < GYRO_STILL_THRESHOLD)
        accel_valid = (ACCEL_GATE_MIN_G <= accel_mag <= ACCEL_GATE_MAX_G)
        
        if is_still and accel_valid:
            # Stationary with valid accel → strong correction
            return self.kp * 3.0
        elif not accel_valid:
            # High accel (motion artifact) → minimal correction
            return self.kp * 0.1
        else:
            # Normal motion → base correction
            return self.kp
    
    def update(self, gx, gy, gz, ax, ay, az, dt):
        """
        Update orientation estimate.
        
        Args:
            gx, gy, gz: Gyro rates (rad/s)
            ax, ay, az: Accelerometer (g)
            dt: Time step (seconds)
        
        Returns:
            Updated quaternion
        """
        if dt <= 0:
            return self.q

        # Gyro vector (already bias-corrected before calling)
        gx, gy, gz = float(gx), float(gy), float(gz)
        gyro_mag = float(np.sqrt(gx*gx + gy*gy + gz*gz))
        self.gyro_magnitude_avg = 0.95 * self.gyro_magnitude_avg + 0.05 * gyro_mag

        # Accel normalization
        ax, ay, az = float(ax), float(ay), float(az)
        accel_mag = float(np.sqrt(ax*ax + ay*ay + az*az))
        self.accel_magnitude_avg = 0.95 * self.accel_magnitude_avg + 0.05 * accel_mag

        kp_adaptive = self.compute_adaptive_kp(gyro_mag, accel_mag)

        q0, q1, q2, q3 = self.q

        if accel_mag > 1e-6:
            ax /= accel_mag
            ay /= accel_mag
            az /= accel_mag

            # Estimated "up" direction (same as classic Mahony reference)
            vx = 2.0*(q1*q3 - q0*q2)
            vy = 2.0*(q0*q1 + q2*q3)
            vz = q0*q0 - q1*q1 - q2*q2 + q3*q3

            # Error is cross(accel, v)
            ex = (ay*vz - az*vy)
            ey = (az*vx - ax*vz)
            ez = (ax*vy - ay*vx)

            # Integral feedback (acts like slow bias compensation)
            if self.ki > 0.0:
                self.integral_error[0] += ex * dt * self.ki
                self.integral_error[1] += ey * dt * self.ki
                self.integral_error[2] += ez * dt * self.ki

                gx += self.integral_error[0]
                gy += self.integral_error[1]
                gz += self.integral_error[2]

            # Proportional feedback
            gx += kp_adaptive * ex
            gy += kp_adaptive * ey
            gz += kp_adaptive * ez

        # Integrate quaternion rate (reference implementation)
        half_dt = 0.5 * dt
        qa, qb, qc = q0, q1, q2
        q0 += (-qb*gx - qc*gy - q3*gz) * half_dt
        q1 += (qa*gx + qc*gz - q3*gy) * half_dt
        q2 += (qa*gy - qb*gz + q3*gx) * half_dt
        q3 += (qa*gz + qb*gy - qc*gx) * half_dt

        self.q = quat_norm(np.array([q0, q1, q2, q3], dtype=float))
        return self.q

# ============================================================================
# CALIBRATION
# ============================================================================
def estimate_gyro_bias(ser, n_samples=200, timeout_s=30.0, status_label=None):
    """
    Collect gyro bias while IMUs are stationary and flat.
    
    Returns: (bias1_gyro, bias2_gyro) in rad/s
    """
    g1_samples = []
    g2_samples = []
    t0 = time.time()
    last_update = 0
    
    print(f"\nCalibrating gyro bias ({n_samples} samples)...")
    print("Keep IMUs FLAT and STILL on the ground!")
    
    while len(g1_samples) < n_samples:
        if time.time() - t0 > timeout_s:
            if len(g1_samples) >= 50:
                print(f"\nTimeout - using {len(g1_samples)} samples")
                break
            else:
                print("\n" + "="*60)
                print("❌ CALIBRATION FAILED: No valid packets received!")
                print("="*60)
                print("\nTroubleshooting:")
                print("1. Make sure you uploaded the correct Arduino sketch")
                print("2. Check Arduino Serial Monitor (Tools → Serial Monitor)")
                print(f"   Should show ASCII lines: $seq,us,ax1,...*HH")
                print(f"3. Verify baud rate is {BAUD}")
                print("4. Try unplugging and replugging Arduino")
                port_msg = PORT if PORT else "auto-detect"
                print(f"5. Check PORT setting in script (currently: {port_msg})")
                print("="*60)
                raise RuntimeError(f"Calibration timeout: only {len(g1_samples)} samples collected")
        
        try:
            packet = read_imu_packet(ser)
            if packet is None:
                continue
            
            _, g1, _, g2, _ = packet
            g1_samples.append(g1)
            g2_samples.append(g2)
            
            # Progress update
            if status_label and (time.time() - last_update) >= 0.2:
                progress = len(g1_samples) / n_samples
                bar_length = 30
                filled = int(bar_length * progress)
                bar = '█' * filled + '░' * (bar_length - filled)
                status_label.text = f"Calibrating: [{bar}] {len(g1_samples)}/{n_samples}\nKeep IMUs FLAT and STILL!"
                last_update = time.time()
                print(f"  {len(g1_samples)}/{n_samples}...", end='\r')
        
        except serial.SerialException as e:
            print(f"\n⚠ Serial error during calibration: {e}")
            raise
    
    print(f"\n✓ Calibration complete: {len(g1_samples)} samples")
    
    # Calculate mean bias (deg/s)
    bias1_deg = np.mean(g1_samples, axis=0)
    bias2_deg = np.mean(g2_samples, axis=0)
    
    # Convert to rad/s
    bias1_rad = np.deg2rad(bias1_deg)
    bias2_rad = np.deg2rad(bias2_deg)
    
    print(f"  IMU1 bias: [{bias1_deg[0]:.2f}, {bias1_deg[1]:.2f}, {bias1_deg[2]:.2f}] deg/s")
    print(f"  IMU2 bias: [{bias2_deg[0]:.2f}, {bias2_deg[1]:.2f}, {bias2_deg[2]:.2f}] deg/s")
    
    return bias1_rad, bias2_rad

# ============================================================================
# CSV DATA LOGGER
# ============================================================================
class CSVLogger:
    """Handles CSV data logging with automatic file creation and flushing"""
    
    def __init__(self, filepath, log_raw=False):
        self.filepath = Path(filepath)
        self.log_raw = log_raw
        self.file_handle = None
        self.csv_writer = None
        self.sample_count = 0
        
        # Create directory if needed
        self.filepath.parent.mkdir(parents=True, exist_ok=True)
        
        # Open file and write header
        self.file_handle = open(self.filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.file_handle)
        
        if log_raw:
            # Raw integer values
            self.csv_writer.writerow([
                'timestamp', 'seq', 't_us',
                'ax1_raw', 'ay1_raw', 'az1_raw', 'gx1_raw', 'gy1_raw', 'gz1_raw',
                'ax2_raw', 'ay2_raw', 'az2_raw', 'gx2_raw', 'gy2_raw', 'gz2_raw',
                'ok1', 'ok2'
            ])
        else:
            # Scaled values + quaternions + euler angles
            self.csv_writer.writerow([
                'timestamp', 'seq', 't_us',
                'ax1_g', 'ay1_g', 'az1_g', 'gx1_dps', 'gy1_dps', 'gz1_dps',
                'ax2_g', 'ay2_g', 'az2_g', 'gx2_dps', 'gy2_dps', 'gz2_dps',
                'q1_w', 'q1_x', 'q1_y', 'q1_z',
                'q2_w', 'q2_x', 'q2_y', 'q2_z',
                'roll1_deg', 'pitch1_deg', 'yaw1_deg',
                'roll2_deg', 'pitch2_deg', 'yaw2_deg',
                'ok1', 'ok2'
            ])
        
        print(f"✓ CSV logging to: {self.filepath}")
    
    def log_raw_sample(self, sample, timestamp=None):
        """Log raw Sample dataclass"""
        if timestamp is None:
            timestamp = time.time()
        
        self.csv_writer.writerow([
            timestamp, sample.seq, sample.t_us,
            sample.ax1, sample.ay1, sample.az1,
            sample.gx1, sample.gy1, sample.gz1,
            sample.ax2, sample.ay2, sample.az2,
            sample.gx2, sample.gy2, sample.gz2,
            sample.ok1, sample.ok2
        ])
        self.sample_count += 1
        
        # Flush every 100 samples
        if self.sample_count % 100 == 0:
            self.file_handle.flush()
    
    def log_processed(self, timestamp, sample, g1_dps, a1_g, g2_dps, a2_g, 
                     q1, q2, euler1, euler2):
        """Log processed data with quaternions and Euler angles"""
        self.csv_writer.writerow([
            timestamp, sample.seq, sample.t_us,
            a1_g[0], a1_g[1], a1_g[2],
            g1_dps[0], g1_dps[1], g1_dps[2],
            a2_g[0], a2_g[1], a2_g[2],
            g2_dps[0], g2_dps[1], g2_dps[2],
            q1[0], q1[1], q1[2], q1[3],
            q2[0], q2[1], q2[2], q2[3],
            euler1[0], euler1[1], euler1[2],
            euler2[0], euler2[1], euler2[2],
            sample.ok1, sample.ok2
        ])
        self.sample_count += 1
        
        # Flush every 100 samples
        if self.sample_count % 100 == 0:
            self.file_handle.flush()
    
    def close(self):
        """Close CSV file"""
        if self.file_handle:
            self.file_handle.flush()
            self.file_handle.close()
            print(f"\n✓ CSV saved: {self.filepath} ({self.sample_count} samples)")
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

# ============================================================================
# COORDINATE FRAME MAPPING
# ============================================================================
def imu_to_scene(imu_vec):
    """
    Transform IMU coordinate frame to VPython scene frame.
    
    IMU frame (component side up):
      X → forward (dot marking)
      Y → left
      Z → up (out of chip)
    
    Scene frame:
      X → right
      Y → up
      Z → toward viewer
    
    Mapping:
      scene_x = imu_x
      scene_y = imu_z (up)
      scene_z = -imu_y (right-handed)
    """
    return vector(float(imu_vec[0]), float(imu_vec[2]), float(-imu_vec[1]))

# ============================================================================
# VPYTHON VISUALIZATION SETUP
# ============================================================================
print("\n" + "="*60)
print("Dual BMI160 IMU Tracker - ASCII Protocol")
print("="*60)

if CSV_FILE:
    print(f"Data logging: ENABLED → {CSV_FILE}")
    print(f"Log format: {'RAW integers' if LOG_RAW_VALUES else 'Scaled + quaternions'}")
else:
    print("Data logging: DISABLED")

print(f"Visualization: {'ENABLED' if ENABLE_VISUALIZATION else 'DISABLED'}")

# Initialize visualization if enabled
scene = None
if ENABLE_VISUALIZATION:
    scene = canvas(title="Optimized Dual IMU Tracker (200Hz ASCII)", 
                   width=1600, height=900,
                   center=vector(0, 0, 0),
                   background=color.gray(0.1))
    
    scene.forward = vector(-0.3, -0.4, -1)
    scene.range = 2.5
else:
    print("(Headless mode - CSV logging only)")
    # Create dummy objects for compatibility
    class DummyObj:
        def __init__(self):
            self.pos = None
            self.axis = None
            self.up = None
            self.text = ""

if ENABLE_VISUALIZATION:
    # Ground plane
    ground = box(pos=vector(0, -0.6, 0), 
                 length=6, height=0.02, width=4, 
                 color=color.gray(0.3), opacity=0.3)
    
    # Grid lines
    for i in range(-3, 4):
        box(pos=vector(0, -0.59, i*0.5), length=6, height=0.01, width=0.02, color=color.gray(0.4))
        box(pos=vector(i*0.5, -0.59, 0), length=0.02, height=0.01, width=4, color=color.gray(0.4))
    
    # Dice creation
    spacing = 1.8
    dice_size = 0.9
else:
    spacing = 1.8
    dice_size = 0.9

def _pip_uv(n, s):
    """2D pip layouts for dice faces"""
    c, a = 0.0, s
    if n == 1: return [(c, c)]
    if n == 2: return [(-a, -a), (a, a)]
    if n == 3: return [(-a, -a), (c, c), (a, a)]
    if n == 4: return [(-a, -a), (-a, a), (a, -a), (a, a)]
    if n == 5: return [(-a, -a), (-a, a), (c, c), (a, -a), (a, a)]
    if n == 6: return [(-a, -a), (-a, c), (-a, a), (a, -a), (a, c), (a, a)]
    return []

def make_die(pos, size=0.9, body_col=color.white):
    """Create dice as compound with pips"""
    half = size / 2.0
    pip_r = size * 0.085
    pip_spread = size * 0.22
    pip_inset = size * 0.03
    
    origin = vector(0, 0, 0)
    parts = [box(pos=origin, length=size, height=size, width=size, 
                 color=body_col, opacity=1.0)]
    
    # Faces: (normal, u_axis, v_axis, number)
    faces = [
        (vector(1,0,0), vector(0,1,0), vector(0,0,1), 1),
        (vector(-1,0,0), vector(0,1,0), vector(0,0,1), 6),
        (vector(0,1,0), vector(1,0,0), vector(0,0,1), 2),
        (vector(0,-1,0), vector(1,0,0), vector(0,0,1), 5),
        (vector(0,0,1), vector(1,0,0), vector(0,1,0), 3),
        (vector(0,0,-1), vector(1,0,0), vector(0,1,0), 4),
    ]
    
    for nrm, u_axis, v_axis, num in faces:
        face_center = origin + nrm * (half - pip_inset)
        for u, v in _pip_uv(num, pip_spread):
            parts.append(sphere(
                pos=face_center + u_axis * u + v_axis * v,
                radius=pip_r, color=color.black, opacity=1.0
            ))
    
    return compound(parts, pos=pos)

if ENABLE_VISUALIZATION:
    # Create dice
    body1 = make_die(vector(-spacing, 0, 0), size=dice_size, body_col=color.white)
    body2 = make_die(vector(spacing, 0, 0), size=dice_size, body_col=color.white)
    
    # Labels
    label1 = label(pos=vector(-spacing, -0.75, 0), text="IMU 1",
                   height=12, color=color.cyan, box=False, opacity=0)
    label2 = label(pos=vector(spacing, -0.75, 0), text="IMU 2",
                   height=12, color=color.orange, box=False, opacity=0)
    
    # Axes
    def make_axes(origin, scale=1.0):
        ax = arrow(pos=origin, axis=vector(scale, 0, 0), 
                   color=color.red, shaftwidth=0.04, headwidth=0.08, headlength=0.12)
        ay = arrow(pos=origin, axis=vector(0, scale, 0), 
                   color=color.green, shaftwidth=0.04, headwidth=0.08, headlength=0.12)
        az = arrow(pos=origin, axis=vector(0, 0, scale), 
                   color=color.blue, shaftwidth=0.04, headwidth=0.08, headlength=0.12)
        return ax, ay, az
    
    if SHOW_AXES:
        a1x, a1y, a1z = make_axes(body1.pos, scale=1.0)
        a2x, a2y, a2z = make_axes(body2.pos, scale=1.0)
    else:
        a1x = a1y = a1z = a2x = a2y = a2z = None
    
    # Status labels
    status_lab = label(pos=vector(0, 1.8, 0), text="Initializing...", 
                       box=False, height=16, color=color.white, opacity=0)
    
    legend = label(pos=vector(0, -1.5, 0), 
                   text="IMU axes: X=Red(forward) | Y=Green(left) | Z=Blue(up)", 
                   box=False, height=10, color=color.gray(0.7), opacity=0)
    
    if SHOW_PERFORMANCE_STATS:
        perf_lab = label(pos=vector(0, 1.4, 0), text="", 
                         box=False, height=12, color=color.yellow, opacity=0)
    else:
        perf_lab = None
else:
    # Dummy objects for headless mode
    body1 = body2 = label1 = label2 = status_lab = legend = perf_lab = DummyObj()
    a1x = a1y = a1z = a2x = a2y = a2z = None

# ============================================================================
# SERIAL CONNECTION
# ============================================================================
def open_serial():
    """Open serial connection with robust initialization (using arduino_connection module)"""
    # Use the new arduino_connection module for auto-detection and connection
    if PORT is None:
        # Auto-detect mode
        s = open_arduino_serial(
            port=None,
            baud=BAUD,
            timeout=0.25,
            wait_for_ready=True,
            ready_timeout=5.0,
            verbose=True
        )
    else:
        # Manual port specified
        s = open_arduino_serial(
            port=PORT,
            baud=BAUD,
            timeout=0.25,
            wait_for_ready=True,
            ready_timeout=5.0,
            verbose=True
        )
    
    return s

def reopen_serial(old_ser):
    """Close and reopen serial connection"""
    try:
        old_ser.close()
    except:
        pass
    time.sleep(RECONNECT_BACKOFF_S)
    return open_serial()

ser = open_serial()
last_rx_time = time.time()

# Verify ASCII packets
print("\nVerifying ASCII data stream...")

# Give Arduino a moment to start sending if it just reset
time.sleep(0.5)

valid_packets = 0
bad_packets = 0

print("  Listening for packets (debug mode enabled)...")
for i in range(20):  # Try multiple attempts to sync
    packet = read_imu_packet(ser, debug=True)
    if packet:
        valid_packets += 1
        t_ms, g1, a1, g2, a2 = packet
        print(f"  ✓ Packet {valid_packets}: Valid ASCII packet received")
        print(f"    Timestamp: {t_ms:.1f}ms, IMU1 accel: [{a1[0]:.2f}, {a1[1]:.2f}, {a1[2]:.2f}]g")
        last_rx_time = time.time()
        
        if valid_packets >= 3:
            print(f"\n✓ ASCII mode confirmed! Received {valid_packets} valid packets")
            break
    else:
        bad_packets += 1
    
    time.sleep(0.05)

if valid_packets == 0:
    print("\n" + "="*60)
    print("❌ ERROR: No valid ASCII packets received!")
    print("="*60)
    print("\nPossible issues:")
    print("1. Wrong sketch uploaded (check Arduino Serial Monitor)")
    print("2. IMU hardware not connected or not detected")
    print("3. I2C connection issue")
    print("4. Wrong baud rate (should be 230400)")
    print("\nTo debug:")
    print(f"  - Open Arduino IDE Serial Monitor at {BAUD} baud")
    print("  - Should see lines like: $seq,us,ax1,ay1,az1,...*HH")
    print("  - If you see binary data or nothing, wrong sketch is running")
    print("="*60)
    sys.exit(1)

print(f"  ({bad_packets} invalid/incomplete packets skipped)")

# ============================================================================
# INITIALIZE CSV LOGGER
# ============================================================================
csv_logger = None
if CSV_FILE:
    csv_logger = CSVLogger(CSV_FILE, log_raw=LOG_RAW_VALUES)

# ============================================================================
# CALIBRATION SEQUENCE
# ============================================================================
# Show reference orientation
print("\n" + "="*60)
print("CALIBRATION SETUP")
print("="*60)
print("Place BOTH IMUs FLAT on the ground:")
print("  - Component side facing UP (visible)")
print("  - Lying completely flat and horizontal")
print("  - Dot/marking pointing forward (any direction is fine)")
print("  - Keep them STILL during calibration")
print("="*60)

ex = np.array([1.0, 0.0, 0.0])
ey = np.array([0.0, 1.0, 0.0])
ez = np.array([0.0, 0.0, 1.0])

if ENABLE_VISUALIZATION:
    # Show reference orientation
    body1.axis = imu_to_scene(ex)
    body1.up = imu_to_scene(ez)
    body2.axis = imu_to_scene(ex)
    body2.up = imu_to_scene(ez)
    
    if SHOW_AXES:
        a1x.pos = body1.pos; a1x.axis = imu_to_scene(ex)
        a1y.pos = body1.pos; a1y.axis = imu_to_scene(ey)
        a1z.pos = body1.pos; a1z.axis = imu_to_scene(ez)
        a2x.pos = body2.pos; a2x.axis = imu_to_scene(ex)
        a2y.pos = body2.pos; a2y.axis = imu_to_scene(ey)
        a2z.pos = body2.pos; a2z.axis = imu_to_scene(ez)
    
    status_lab.text = "REFERENCE ORIENTATION SHOWN\n\nPlace IMUs FLAT on ground\n(component side up)\n\nStarting in 5 seconds..."
    
    # Countdown
    for i in range(5, 0, -1):
        print(f"  Starting calibration in {i}...", flush=True)
        status_lab.text = f"REFERENCE ORIENTATION\n\nHold position!\n\nStarting in {i}..."
        time.sleep(1)
else:
    # Headless countdown
    for i in range(5, 0, -1):
        print(f"  Starting calibration in {i}...", flush=True)
        time.sleep(1)

print("  Calibrating NOW!")

# Calibrate
bias1_rad, bias2_rad = estimate_gyro_bias(ser, n_samples=CALIB_SAMPLES, status_label=status_lab)

# Initialize filters
f1 = MahonyIMU(kp=KP_BASE, ki=KI)
f2 = MahonyIMU(kp=KP_BASE, ki=KI)
f1.q = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
f2.q = np.array([1.0, 0.0, 0.0, 0.0])

status_lab.text = "✓ Calibration complete!\n\nStarting tracking..."
print("\n✓ Filters initialized")
print("  Reference: IMUs flat = identity orientation")
print("\nStarting real-time tracking...\n")
time.sleep(2)

# ============================================================================
# PERFORMANCE MONITORING
# ============================================================================
class PerformanceMonitor:
    def __init__(self, window_size=100):
        self.frame_times = deque(maxlen=window_size)
        self.latencies = deque(maxlen=window_size)
        self.packet_count = 0
        self.error_count = 0
        self.start_time = time.time()
    
    def update(self, frame_time, latency_ms=None):
        self.frame_times.append(frame_time)
        if latency_ms is not None:
            self.latencies.append(latency_ms)
        self.packet_count += 1
    
    def get_fps(self):
        if len(self.frame_times) < 2:
            return 0.0
        return len(self.frame_times) / sum(self.frame_times)
    
    def get_avg_latency(self):
        if not self.latencies:
            return 0.0
        return np.mean(self.latencies)
    
    def get_packet_loss_rate(self):
        total_expected = (time.time() - self.start_time) * 200  # 200Hz
        if total_expected < 100:
            return 0.0
        return 100.0 * (1.0 - self.packet_count / total_expected)

perf_mon = PerformanceMonitor()

# ============================================================================
# MAIN TRACKING LOOP
# ============================================================================
t_prev = None
python_start_time = time.time()
frame_start = time.time()

print("="*60)
print("TRACKING ACTIVE - Verify coordinate frame:")
print("  - Tilt IMU forward → Dice tilts forward")
print("  - Roll IMU right → Dice rolls right")
print("  - Check raw accel display for validation")
print("="*60)

try:
    while True:
        loop_start = time.time()
        if ENABLE_VISUALIZATION:
            rate(120)  # VPython refresh rate cap
        
        # Watchdog: reconnect if stalled
        if AUTO_RECONNECT and (time.time() - last_rx_time) > STALL_TIMEOUT_S:
            print(f"\n⚠ Serial stalled > {STALL_TIMEOUT_S}s. Reconnecting...")
            ser = reopen_serial(ser)
            t_prev = None
            last_rx_time = time.time()
            continue
        
        # Drain multiple packets per frame so we can keep up with 200Hz source
        latest = None
        latest_sample = None
        got_any = False
        try:
            for _ in range(MAX_PACKETS_PER_FRAME):
                # Get packet with raw sample if CSV logging is enabled
                result = read_imu_packet(ser, return_sample=(csv_logger is not None))
                if result is None:
                    break
                
                if csv_logger is not None:
                    packet, sample = result
                    latest_sample = sample
                    
                    # Log raw data immediately if in raw mode
                    if LOG_RAW_VALUES:
                        csv_logger.log_raw_sample(sample, timestamp=time.time())
                else:
                    packet = result
                
                latest = packet
                got_any = True
                
                last_rx_time = time.time()
        
        except serial.SerialException as e:
            print(f"\n⚠ Serial error: {e}")
            if AUTO_RECONNECT:
                ser = reopen_serial(ser)
                t_prev = None
                last_rx_time = time.time()
                continue
            raise
        except Exception as e:
            print(f"\n⚠ Unexpected error: {e}")
            if AUTO_RECONNECT:
                ser = reopen_serial(ser)
                t_prev = None
                last_rx_time = time.time()
                continue
            raise
        
        if not got_any:
            perf_mon.error_count += 1
            continue

        # Use the latest packet for processing
        t_ms, g1_deg, a1, g2_deg, a2 = latest
        
        # Axis map + optional accel invert
        g1_deg = remap_vec(g1_deg, IMU1_AXIS_MAP)
        g2_deg = remap_vec(g2_deg, IMU2_AXIS_MAP)
        a1 = remap_vec(a1, IMU1_AXIS_MAP)
        a2 = remap_vec(a2, IMU2_AXIS_MAP)
        if INVERT_ACCEL:
            a1 = -a1
            a2 = -a2
        
        # Calculate latency
        python_time_ms = (time.time() - python_start_time) * 1000
        latency_ms = python_time_ms - t_ms
        
        # Convert gyro to rad/s and subtract bias (bias is in sensor frame; mapping already applied)
        g1_rad = np.deg2rad(g1_deg) - remap_vec(bias1_rad, IMU1_AXIS_MAP)
        g2_rad = np.deg2rad(g2_deg) - remap_vec(bias2_rad, IMU2_AXIS_MAP)
        
        # Time step
        t_s = t_ms / 1000.0
        if t_prev is None:
            t_prev = t_s
            continue
        
        dt = t_s - t_prev
        if dt <= 0 or dt > 0.2:
            t_prev = t_s
            continue
        t_prev = t_s
        
        # Update filters (note: we only integrate once per frame using latest packet timestamp)
        q1 = f1.update(g1_rad[0], g1_rad[1], g1_rad[2], a1[0], a1[1], a1[2], dt)
        q2 = f2.update(g2_rad[0], g2_rad[1], g2_rad[2], a2[0], a2[1], a2[2], dt)
        
        # Calculate Euler angles
        roll1, pitch1, yaw1 = quat_to_euler(q1)
        roll2, pitch2, yaw2 = quat_to_euler(q2)
        
        # Accel magnitudes
        a1_mag = np.linalg.norm(a1)
        a2_mag = np.linalg.norm(a2)
        
        # Log processed data if CSV enabled and not in raw mode
        if csv_logger and not LOG_RAW_VALUES and latest_sample:
            csv_logger.log_processed(
                timestamp=time.time(),
                sample=latest_sample,
                g1_dps=g1_deg, a1_g=a1,
                g2_dps=g2_deg, a2_g=a2,
                q1=q1, q2=q2,
                euler1=(roll1, pitch1, yaw1),
                euler2=(roll2, pitch2, yaw2)
            )
        
        # Update performance monitor
        frame_time = time.time() - frame_start
        perf_mon.update(frame_time, latency_ms)
        frame_start = time.time()
        
        # Visualization updates (only if enabled)
        if ENABLE_VISUALIZATION:
            # Relative visualization option
            if SHOW_RELATIVE_IMU2_TO_IMU1:
                q1_vis = np.array([1.0, 0.0, 0.0, 0.0])
                q2_vis = quat_mul(quat_inv(q1), q2)
            else:
                q1_vis = q1
                q2_vis = q2
            
            # Rotate basis vectors
            r1x = rotate_vec_by_quat(ex, q1_vis)
            r1y = rotate_vec_by_quat(ey, q1_vis)
            r1z = rotate_vec_by_quat(ez, q1_vis)
            
            r2x = rotate_vec_by_quat(ex, q2_vis)
            r2y = rotate_vec_by_quat(ey, q2_vis)
            r2z = rotate_vec_by_quat(ez, q2_vis)
            
            # Update visualization
            body1.axis = imu_to_scene(r1x)
            body1.up = imu_to_scene(r1z)
            body2.axis = imu_to_scene(r2x)
            body2.up = imu_to_scene(r2z)
            
            if SHOW_AXES:
                a1x.pos = body1.pos; a1x.axis = imu_to_scene(r1x)
                a1y.pos = body1.pos; a1y.axis = imu_to_scene(r1y)
                a1z.pos = body1.pos; a1z.axis = imu_to_scene(r1z)
                a2x.pos = body2.pos; a2x.axis = imu_to_scene(r2x)
                a2y.pos = body2.pos; a2y.axis = imu_to_scene(r2y)
                a2z.pos = body2.pos; a2z.axis = imu_to_scene(r2z)
            
            # Status display
            status_lab.text = f"IMU1: R={roll1:+6.1f}° P={pitch1:+6.1f}° Y={yaw1:+6.1f}°  |a|={a1_mag:.2f}g\n" + \
                              f"IMU2: R={roll2:+6.1f}° P={pitch2:+6.1f}° Y={yaw2:+6.1f}°  |a|={a2_mag:.2f}g\n" + \
                              f"Raw accel IMU1: [{a1[0]:+.2f}, {a1[1]:+.2f}, {a1[2]:+.2f}]g"
            
            # Performance stats
            if SHOW_PERFORMANCE_STATS and perf_lab:
                fps = perf_mon.get_fps()
                avg_lat = perf_mon.get_avg_latency()
                loss = perf_mon.get_packet_loss_rate()
                perf_lab.text = f"FPS: {fps:.1f} | Latency: {avg_lat:.1f}ms | Loss: {loss:.2f}% | dt: {dt*1000:.1f}ms"
        else:
            # Headless mode: print periodic status
            if perf_mon.packet_count % 100 == 0:
                fps = perf_mon.get_fps() if ENABLE_VISUALIZATION else (perf_mon.packet_count / (time.time() - perf_mon.start_time))
                print(f"  [{perf_mon.packet_count:6d}] IMU1: R={roll1:+6.1f}° P={pitch1:+6.1f}° Y={yaw1:+6.1f}°  |  "
                      f"IMU2: R={roll2:+6.1f}° P={pitch2:+6.1f}° Y={yaw2:+6.1f}°  |  {fps:.1f} Hz", end='\r')

except KeyboardInterrupt:
    print("\n\n✓ Stopped by user")
finally:
    # Clean up CSV logger
    if csv_logger:
        csv_logger.close()
