"""
Dual BNO085 UART-RVC IMU Tracking
==================================

Real-time orientation tracking with BNO085 9-DOF IMUs:
- UART-RVC CSV protocol (115200 baud, 500Hz)
- On-device sensor fusion (no calibration needed)
- Real-time VPython visualization with dice representation
- CSV data logging
- Robust auto-reconnect

Hardware Requirements:
- STM32F401 with dual BNO085 IMUs in UART-RVC mode
- 115200 baud (BNO085 RVC standard)
- 500 Hz output rate
- BNO085 configured for UART-RVC (P0 high/bridged)

CSV Format from STM32:
  t_ms,imu1_ok,imu2_ok,yaw1,pitch1,roll1,ax1,ay1,az1,yaw2,pitch2,roll2,ax2,ay2,az2

Coordinate Frame:
- BNO085 provides absolute orientation (yaw, pitch, roll)
- 9-DOF fusion combines accel, gyro, and magnetometer
- No drift, no calibration needed

Usage:
  python dual_BNO085_testing.py [--port PORT] [--baud BAUD] [--csv FILE] [--no-viz]
  
Examples:
  python dual_BNO085_testing.py                        # Auto-detect port, with visualization
  python dual_BNO085_testing.py --port /dev/ttyACM0    # Specific port
  python dual_BNO085_testing.py --csv data.csv         # Log to CSV file
  python dual_BNO085_testing.py --no-viz --csv data.csv # CSV only, no visualization
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
        description='Dual BNO085 IMU Tracker with real-time visualization and data logging'
    )
    parser.add_argument('--port', type=str, default="auto",
                        help='Serial port (or "auto" for auto-detect, default: auto)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Serial baud rate (default: 115200)')
    parser.add_argument('--csv', type=str, default=None,
                        help='CSV file path for data logging (optional)')
    parser.add_argument('--no-viz', action='store_true',
                        help='Disable visualization (CSV logging only)')
    return parser.parse_args()

args = parse_args()

# ============================================================================
# USER SETTINGSß
# ============================================================================
PORT = None if args.port.lower() == "auto" else args.port
BAUD = args.baud

# Serial robustness
AUTO_RECONNECT = True
STALL_TIMEOUT_S = 3.0
RECONNECT_BACKOFF_S = 1.0

# Data logging
CSV_FILE = args.csv
ENABLE_VISUALIZATION = not args.no_viz

# Visualization
SHOW_RELATIVE_IMU2_TO_IMU1 = False  # False = absolute, True = relative
SHOW_AXES = True                     # Show RGB axes on dice
SHOW_PERFORMANCE_STATS = True        # Show FPS, latency, drift metrics

# Axis remapping (CRITICAL for "dice matches real IMU")
# -----------------------------------------------------
# Format: (out_x, out_y, out_z) where each element is ±1..±3 selecting
# (±in_x, ±in_y, ±in_z). Example:
#   (1, 2, 3)   -> identity
#   (1, -2, -3) -> flip Y and Z
#   (2, -1, 3)  -> swap X/Y and flip new Y
IMU1_AXIS_MAP = (1, 2, 3)
IMU2_AXIS_MAP = (1, 2, 3)

# If your "flat on table" accel Z is negative (common if sensor is mounted
# component-side-down), set this to True to invert accel vector.
INVERT_ACCEL = False

# Read/compute: STM32 sends 500Hz but VPython renders ~60-120Hz.
# We must drain multiple packets per frame to avoid lag and desync.
MAX_PACKETS_PER_FRAME = 16  # 500Hz / ~60fps ≈ 8+ packets per frame

# ============================================================================
# BNO085 UART-RVC CSV PROTOCOL
# Format: t_ms,imu1_ok,imu2_ok,yaw1,pitch1,roll1,ax1,ay1,az1,yaw2,pitch2,roll2,ax2,ay2,az2
# ============================================================================

@dataclass
class SampleBNO085:
    """Parsed BNO085 UART-RVC CSV line (Euler degrees + accel in g)."""
    t_ms: float
    ok1: int
    ok2: int
    yaw1: float
    pitch1: float
    roll1: float
    ax1: float
    ay1: float
    az1: float
    yaw2: float
    pitch2: float
    roll2: float
    ax2: float
    ay2: float
    az2: float

def parse_line_bno085(line: str) -> Optional[SampleBNO085]:
    """
    Parse BNO085 UART-RVC CSV: t_ms,imu1_ok,imu2_ok,yaw1,pitch1,roll1,ax1,ay1,az1,yaw2,pitch2,roll2,ax2,ay2,az2
    Returns SampleBNO085 if valid, else None.
    """
    line = line.strip()
    if not line or line.startswith("Columns:") or line.startswith("#") or line.startswith("STM32") or line.startswith("OK:") or line.startswith("ERR:"):
        return None
    parts = line.split(",")
    if len(parts) != 15:
        return None
    try:
        vals = [float(x) for x in parts]
    except ValueError:
        return None
    return SampleBNO085(
        t_ms=vals[0],
        ok1=int(vals[1]),
        ok2=int(vals[2]),
        yaw1=vals[3], pitch1=vals[4], roll1=vals[5],
        ax1=vals[6], ay1=vals[7], az1=vals[8],
        yaw2=vals[9], pitch2=vals[10], roll2=vals[11],
        ax2=vals[12], ay2=vals[13], az2=vals[14],
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
    Read and parse BNO085 UART-RVC CSV packet.
    
    Args:
        ser: Serial port object
        debug: Print debug messages
        return_sample: If True, return (data_tuple, sample_obj), else just data_tuple
    
    Returns:
        (timestamp_ms, q1, imu1_accel_g, q2, imu2_accel_g) — quaternions from Euler angles
        or ((data_tuple), sample_obj) if return_sample=True
    """
    try:
        raw = ser.readline()
        if not raw:
            return None
        
        try:
            line = raw.decode("ascii", errors="ignore")
        except Exception:
            return None
        
        s = parse_line_bno085(line)
        if s is None:
            if debug and line.strip() and not line.startswith("OK:") and not line.startswith("ERR:") and not line.startswith("STM32"):
                print(f"  Parse failed: {line.strip()[:80]}")
            return None
        if s.ok1 == 0 or s.ok2 == 0:
            return None
        
        timestamp_ms = s.t_ms
        a1 = np.array([s.ax1, s.ay1, s.az1], dtype=float)
        a2 = np.array([s.ax2, s.ay2, s.az2], dtype=float)
        q1 = euler_to_quat(s.roll1, s.pitch1, s.yaw1)
        q2 = euler_to_quat(s.roll2, s.pitch2, s.yaw2)
        data = (timestamp_ms, q1, a1, q2, a2)
        
        return (data, s) if return_sample else data
        
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

def euler_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Convert Euler angles (roll, pitch, yaw) in degrees to unit quaternion [w, x, y, z].
    Same convention as quat_to_euler (intrinsic ZYX / yaw-pitch-roll)."""
    r = np.deg2rad(roll_deg)
    p = np.deg2rad(pitch_deg)
    y = np.deg2rad(yaw_deg)
    cr, sr = np.cos(r / 2), np.sin(r / 2)
    cp, sp = np.cos(p / 2), np.sin(p / 2)
    cy, sy = np.cos(y / 2), np.sin(y / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    yq = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, yq, z], dtype=float)

# ============================================================================
# CSV DATA LOGGER
# ============================================================================
class CSVLogger:
    """Handles CSV data logging for BNO085 data"""
    
    def __init__(self, filepath):
        self.filepath = Path(filepath)
        self.file_handle = None
        self.csv_writer = None
        self.sample_count = 0
        
        # Create directory if needed
        self.filepath.parent.mkdir(parents=True, exist_ok=True)
        
        # Open file and write header
        self.file_handle = open(self.filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.file_handle)
        
        # BNO085 CSV format: timestamp + sensor data + quaternions + euler
        self.csv_writer.writerow([
            'timestamp', 't_ms',
            'ax1_g', 'ay1_g', 'az1_g',
            'ax2_g', 'ay2_g', 'az2_g',
            'q1_w', 'q1_x', 'q1_y', 'q1_z',
            'q2_w', 'q2_x', 'q2_y', 'q2_z',
            'roll1_deg', 'pitch1_deg', 'yaw1_deg',
            'roll2_deg', 'pitch2_deg', 'yaw2_deg',
            'ok1', 'ok2'
        ])
        
        print(f"✓ CSV logging to: {self.filepath}")
    
    def log_sample(self, timestamp, sample, a1_g, a2_g, q1, q2, euler1, euler2):
        """Log BNO085 sample with quaternions and Euler angles"""
        self.csv_writer.writerow([
            timestamp, sample.t_ms,
            a1_g[0], a1_g[1], a1_g[2],
            a2_g[0], a2_g[1], a2_g[2],
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
print("Dual BNO085 UART-RVC IMU Tracker")
print("="*60)

if CSV_FILE:
    print(f"Data logging: ENABLED → {CSV_FILE}")
else:
    print("Data logging: DISABLED")

print(f"Visualization: {'ENABLED' if ENABLE_VISUALIZATION else 'DISABLED'}")

# Initialize visualization if enabled
scene = None
if ENABLE_VISUALIZATION:
    scene = canvas(title="Dual BNO085 IMU Tracker (500Hz UART-RVC)", 
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
    """Open serial connection with robust initialization"""
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

# Verify data stream
print("\nVerifying BNO085 data stream...")

# Give STM32 a moment to start sending if it just reset
time.sleep(0.5)

valid_packets = 0
bad_packets = 0

print("  Listening for packets (debug mode enabled)...")
for i in range(20):  # Try multiple attempts to sync
    packet = read_imu_packet(ser, debug=True)
    if packet:
        valid_packets += 1
        t_ms, q1, a1, q2, a2 = packet
        print(f"  ✓ Packet {valid_packets}: Valid BNO085 CSV packet received")
        print(f"    Timestamp: {t_ms:.1f}ms, IMU1 accel: [{a1[0]:.2f}, {a1[1]:.2f}, {a1[2]:.2f}]g")
        last_rx_time = time.time()
        
        if valid_packets >= 3:
            print(f"\n✓ BNO085 CSV mode confirmed! Received {valid_packets} valid packets")
            break
    else:
        bad_packets += 1
    
    time.sleep(0.05)

if valid_packets == 0:
    print("\n" + "="*60)
    print("❌ ERROR: No valid packets received!")
    print("="*60)
    print("\nPossible issues:")
    print("1. Wrong sketch uploaded (check Arduino Serial Monitor)")
    print("2. BNO085 hardware not connected or not detected")
    print("3. Wrong baud rate (should be 115200 for BNO085 UART-RVC)")
    print("\nTo debug:")
    print(f"  - Open Arduino IDE Serial Monitor at {BAUD} baud")
    print("  - Should see CSV: t_ms,imu1_ok,imu2_ok,yaw1,pitch1,roll1,ax1,ay1,az1,...")
    print("  - If you see binary data or nothing, wrong sketch is running")
    print("="*60)
    sys.exit(1)

print(f"  ({bad_packets} invalid/incomplete packets skipped)")

# ============================================================================
# INITIALIZE CSV LOGGER
# ============================================================================
csv_logger = None
if CSV_FILE:
    csv_logger = CSVLogger(CSV_FILE)

# ============================================================================
# INITIALIZATION - BNO085 (No calibration needed)
# ============================================================================
ex = np.array([1.0, 0.0, 0.0])
ey = np.array([0.0, 1.0, 0.0])
ez = np.array([0.0, 0.0, 1.0])

print("\n" + "="*60)
print("BNO085 MODE - Using raw sensor orientation")
print("="*60)
print("  BNO085 has on-device 9-DOF sensor fusion")
print("  Orientation is absolute (no calibration needed)")
print("\nStarting tracking in 2 seconds...")
print("="*60)

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
    
    status_lab.text = "BNO085 MODE\n\nStarting in 2 seconds..."

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
        total_expected = (time.time() - self.start_time) * 500  # 500Hz
        if total_expected < 100:
            return 0.0
        return 100.0 * (1.0 - self.packet_count / total_expected)

perf_mon = PerformanceMonitor()

# ============================================================================
# MAIN TRACKING LOOP
# ============================================================================
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
            last_rx_time = time.time()
            continue
        
        # Drain multiple packets per frame so we can keep up with 500Hz source
        latest = None
        latest_sample = None
        got_any = False
        try:
            for _ in range(MAX_PACKETS_PER_FRAME):
                # Get packet with sample if CSV logging is enabled
                result = read_imu_packet(ser, return_sample=(csv_logger is not None))
                if result is None:
                    break
                
                if csv_logger is not None:
                    packet, sample = result
                    latest_sample = sample
                else:
                    packet = result
                
                latest = packet
                got_any = True
                
                last_rx_time = time.time()
        
        except serial.SerialException as e:
            print(f"\n⚠ Serial error: {e}")
            if AUTO_RECONNECT:
                ser = reopen_serial(ser)
                last_rx_time = time.time()
                continue
            raise
        except Exception as e:
            print(f"\n⚠ Unexpected error: {e}")
            if AUTO_RECONNECT:
                ser = reopen_serial(ser)
                last_rx_time = time.time()
                continue
            raise
        
        if not got_any:
            perf_mon.error_count += 1
            continue

        # Use the latest packet for processing (BNO085 provides quaternions directly)
        t_ms, q1, a1, q2, a2 = latest
        
        # Axis map + optional accel invert (orientation from sensor used as-is)
        a1 = remap_vec(a1, IMU1_AXIS_MAP)
        a2 = remap_vec(a2, IMU2_AXIS_MAP)
        if INVERT_ACCEL:
            a1 = -a1
            a2 = -a2
        
        # Calculate Euler angles from quaternions
        roll1, pitch1, yaw1 = quat_to_euler(q1)
        roll2, pitch2, yaw2 = quat_to_euler(q2)
        
        # Calculate latency
        python_time_ms = (time.time() - python_start_time) * 1000
        latency_ms = python_time_ms - t_ms
        dt = 0.002  # 500 Hz nominal
        
        # Accel magnitudes
        a1_mag = np.linalg.norm(a1)
        a2_mag = np.linalg.norm(a2)
        
        # Log data if CSV enabled
        if csv_logger and latest_sample:
            csv_logger.log_sample(
                timestamp=time.time(),
                sample=latest_sample,
                a1_g=a1, a2_g=a2,
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
                              f"Raw accel IMU1: [{a1[0]:+.2f}, {a1[1]:+.2f}, {a1[2]:+.2f}]g\n" + \
                              f"Raw accel IMU2: [{a2[0]:+.2f}, {a2[1]:+.2f}, {a2[2]:+.2f}]g"
            
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
