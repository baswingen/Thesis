"""
BNO085 IMU Tracker - Quaternion Visualization
=============================================

Real-time visualization of BNO085 IMU orientation using built-in sensor fusion.
The BNO085 provides quaternions directly, so no filtering is needed.

Hardware Requirements:
- Arduino with BNO085 IMU (I2C address 0x4A)
- USB connection
- Upload BNO085 sketch to Arduino first

Protocol:
- CSV format: t_ms,qw,qx,qy,qz
- Comment lines start with '#'
- 50Hz update rate (20ms intervals)
- 115200 baud

Coordinate Frame:
The visualization shows BNO085 axes directly as printed on the breakout board:
  Red arrow = X axis
  Green arrow = Y axis
  Blue arrow = Z axis

Use this to verify tracking matches the physical reference frame on your board.
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

# Add parent directory to path to allow importing from src
sys.path.insert(0, str(Path(__file__).parent.parent))

# Import Arduino connection utilities
from src.arduino_connection import find_arduino_port, open_arduino_serial, list_all_ports

# ============================================================================
# COMMAND LINE ARGUMENTS
# ============================================================================
def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description='BNO085 IMU Tracker with real-time quaternion visualization'
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
# USER SETTINGS
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
SHOW_AXES = True                     # Show RGB axes on dice
SHOW_PERFORMANCE_STATS = True        # Show FPS, latency, packet stats

# BNO085 Coordinate Frame Mapping
# ===============================
# BNO085 standard coordinate frame (sensor frame):
#   X → forward (toward screen on MacBook)
#   Y → left (left side of MacBook)
#   Z → up (away from keyboard)
#
# MacBook physical orientation:
#   X → length (forward = screen direction)
#   Y → width (right = right side)
#   Z → height (up = away from keyboard)
#
# We need to map BNO085 (X=forward, Y=left, Z=up) to MacBook (X=forward, Y=right, Z=up)
# This requires a 180° rotation around Z axis to flip Y: [0, 0, 0, 1] → [0, 0, 0, -1]
# Actually, to flip Y we need: [cos(90°), 0, 0, sin(90°)] = [0, 0, 0, 1] for 180° around Z
# But simpler: just negate the Y component of the quaternion rotation
#
# Better approach: Apply coordinate frame transformation quaternion
# 180° rotation around Z axis: q = [0, 0, 0, 1] (w=0, x=0, y=0, z=1) for 180° around Z
# This flips X and Y: X→-X, Y→-Y, Z→Z
# But we only want to flip Y, so we need a different transformation
#
# Actually, the simplest fix: when rotating vectors, swap Y sign or apply proper remapping
# Let's use a quaternion that represents rotating the coordinate frame to match MacBook
BNO085_TO_MACBOOK_QUAT = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)  # 180° around Z (flips X and Y)
# But we only need to flip Y, so let's use a different approach - remap axes in visualization

# Read/compute: Arduino sends 50Hz but VPython renders ~60-120Hz.
# We drain multiple packets per frame to avoid lag.
MAX_PACKETS_PER_FRAME = 3

# ============================================================================
# PROTOCOL PARSING
# ============================================================================

@dataclass
class QuaternionSample:
    t_ms: int
    qw: float
    qx: float
    qy: float
    qz: float

def parse_line(line: str) -> Optional[QuaternionSample]:
    """
    Parse CSV line: t_ms,qw,qx,qy,qz
    Returns QuaternionSample if valid, else None.
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
        
        # Validate quaternion (should be approximately unit length)
        q_norm = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if q_norm < 0.1 or q_norm > 2.0:
            return None  # Invalid quaternion
        
        return QuaternionSample(t_ms=t_ms, qw=qw, qx=qx, qy=qy, qz=qz)
    except (ValueError, IndexError):
        return None

def read_quaternion_packet(ser, debug=False):
    """
    Read and parse quaternion packet from BNO085.
    
    Args:
        ser: Serial port object
        debug: Print debug messages
    
    Returns:
        (timestamp_ms, quaternion_array) or None
        quaternion_array is [qw, qx, qy, qz]
    """
    try:
        raw = ser.readline()
        if not raw:
            return None
        
        try:
            line = raw.decode("ascii", errors="ignore")
        except Exception:
            return None
        
        # Handle comment/status lines
        if line.startswith("#"):
            if debug:
                print(f"  Status: {line.strip()}")
            return None
        
        sample = parse_line(line)
        if sample is None:
            if debug and line.strip():
                print(f"  Parse failed: {line.strip()[:80]}")
            return None
        
        # Normalize quaternion
        q = np.array([sample.qw, sample.qx, sample.qy, sample.qz], dtype=float)
        q_norm = np.linalg.norm(q)
        if q_norm > 1e-6:
            q = q / q_norm
        
        # BNO085 coordinate frame is already correct - we handle remapping in visualization
        # The quaternion from BNO085 represents rotation in its native frame
        # We'll remap the axes when converting to scene coordinates
        
        return (sample.t_ms, q)
        
    except Exception as e:
        if debug:
            print(f"  Error: {e}")
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
# CSV DATA LOGGER
# ============================================================================
class CSVLogger:
    """Handles CSV data logging with automatic file creation and flushing"""
    
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
        
        # Header: timestamp, t_ms, qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg
        self.csv_writer.writerow([
            'timestamp', 't_ms',
            'qw', 'qx', 'qy', 'qz',
            'roll_deg', 'pitch_deg', 'yaw_deg'
        ])
        
        print(f"✓ CSV logging to: {self.filepath}")
    
    def log_sample(self, timestamp, t_ms, q, euler):
        """Log quaternion and Euler angles"""
        self.csv_writer.writerow([
            timestamp, t_ms,
            q[0], q[1], q[2], q[3],
            euler[0], euler[1], euler[2]
        ])
        self.sample_count += 1
        
        # Flush every 50 samples
        if self.sample_count % 50 == 0:
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
def quat_to_scene(quat_vec):
    """
    Direct mapping of BNO085 axes to scene for visualization verification.
    
    This shows the BNO085 reference frame exactly as printed on the board:
    - Red arrow (scene X) = BNO085 X axis
    - Green arrow (scene Y) = BNO085 Y axis  
    - Blue arrow (scene Z) = BNO085 Z axis
    
    Use this to verify the tracking matches the physical board markings.
    """
    # Direct 1:1 mapping - no transformation
    return vector(float(quat_vec[0]), float(quat_vec[1]), float(quat_vec[2]))

# ============================================================================
# VPYTHON VISUALIZATION SETUP
# ============================================================================
print("\n" + "="*60)
print("BNO085 IMU Tracker - Quaternion Visualization")
print("="*60)

if CSV_FILE:
    print(f"Data logging: ENABLED → {CSV_FILE}")
else:
    print("Data logging: DISABLED")

print(f"Visualization: {'ENABLED' if ENABLE_VISUALIZATION else 'DISABLED'}")

# Initialize visualization if enabled
scene = None
if ENABLE_VISUALIZATION:
    scene = canvas(title="BNO085 IMU Tracker (50Hz Quaternion)", 
                   width=1200, height=800,
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
    dice_size = 1.2
else:
    dice_size = 1.2

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

def make_die(pos, size=1.2, body_col=color.white):
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
    body = make_die(vector(0, 0, 0), size=dice_size, body_col=color.white)
    
    # Label
    imu_label = label(pos=vector(0, -0.9, 0), text="BNO085 IMU",
                      height=14, color=color.cyan, box=False, opacity=0)
    
    # Axes
    def make_axes(origin, scale=1.0):
        ax = arrow(pos=origin, axis=vector(scale, 0, 0), 
                   color=color.red, shaftwidth=0.05, headwidth=0.1, headlength=0.15)
        ay = arrow(pos=origin, axis=vector(0, scale, 0), 
                   color=color.green, shaftwidth=0.05, headwidth=0.1, headlength=0.15)
        az = arrow(pos=origin, axis=vector(0, 0, scale), 
                   color=color.blue, shaftwidth=0.05, headwidth=0.1, headlength=0.15)
        return ax, ay, az
    
    if SHOW_AXES:
        ax, ay, az = make_axes(body.pos, scale=1.2)
    else:
        ax = ay = az = None
    
    # Status labels
    status_lab = label(pos=vector(0, 1.5, 0), text="Initializing...", 
                       box=False, height=16, color=color.white, opacity=0)
    
    legend = label(pos=vector(0, -1.3, 0), 
                   text="BNO085 axes (as printed on board): X=Red | Y=Green | Z=Blue", 
                   box=False, height=10, color=color.gray(0.7), opacity=0)
    
    if SHOW_PERFORMANCE_STATS:
        perf_lab = label(pos=vector(0, 1.1, 0), text="", 
                         box=False, height=12, color=color.yellow, opacity=0)
    else:
        perf_lab = None
else:
    # Dummy objects for headless mode
    body = imu_label = status_lab = legend = perf_lab = DummyObj()
    ax = ay = az = None

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

# Verify quaternion packets
print("\nVerifying quaternion data stream...")

# Give Arduino a moment to start sending
time.sleep(0.5)

valid_packets = 0
bad_packets = 0

print("  Listening for packets (debug mode enabled)...")
for i in range(20):  # Try multiple attempts
    packet = read_quaternion_packet(ser, debug=True)
    if packet:
        valid_packets += 1
        t_ms, q = packet
        print(f"  ✓ Packet {valid_packets}: Valid quaternion received")
        print(f"    Timestamp: {t_ms}ms, Quaternion: [{q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f}]")
        last_rx_time = time.time()
        
        if valid_packets >= 3:
            print(f"\n✓ Quaternion mode confirmed! Received {valid_packets} valid packets")
            break
    else:
        bad_packets += 1
    
    time.sleep(0.05)

if valid_packets == 0:
    print("\n" + "="*60)
    print("❌ ERROR: No valid quaternion packets received!")
    print("="*60)
    print("\nPossible issues:")
    print("1. Wrong sketch uploaded (check Arduino Serial Monitor)")
    print("2. BNO085 hardware not connected or not detected")
    print("3. I2C connection issue")
    print("4. Wrong baud rate (should be 115200)")
    print("\nTo debug:")
    print(f"  - Open Arduino IDE Serial Monitor at {BAUD} baud")
    print("  - Should see lines like: t_ms,qw,qx,qy,qz")
    print("  - Or status messages starting with #")
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
# CALIBRATION - CAPTURE REFERENCE ORIENTATION
# ============================================================================
print("\n" + "="*60)
print("CALIBRATION - Reference Orientation")
print("="*60)
print("Place MacBook FLAT on table with screen facing you")
print("This will be the reference (zero) orientation")
print("="*60)

# Reference orientation vectors
ex = np.array([1.0, 0.0, 0.0])
ey = np.array([0.0, 1.0, 0.0])
ez = np.array([0.0, 0.0, 1.0])

def capture_reference_quaternion(ser, n_samples=50):
    """Capture average quaternion for reference orientation"""
    quaternions = []
    print(f"\nCapturing reference orientation ({n_samples} samples)...")
    
    for i in range(n_samples):
        packet = read_quaternion_packet(ser)
        if packet:
            _, q = packet
            quaternions.append(q)
            if (i+1) % 10 == 0:
                print(f"  {i+1}/{n_samples}...", end='\r')
        time.sleep(0.02)
    
    if len(quaternions) < 10:
        print("\n⚠ Warning: Only captured {len(quaternions)} samples")
        return np.array([1.0, 0.0, 0.0, 0.0])
    
    # Average quaternions (simple average, should normalize)
    q_avg = np.mean(quaternions, axis=0)
    q_avg = quat_norm(q_avg)
    
    print(f"\n✓ Reference captured: [{q_avg[0]:.3f}, {q_avg[1]:.3f}, {q_avg[2]:.3f}, {q_avg[3]:.3f}]")
    return q_avg

if ENABLE_VISUALIZATION:
    status_lab.text = "CALIBRATION\n\nPlace MacBook FLAT on table\nScreen facing you\n\nStarting in 5 seconds..."
    
    # Countdown before calibration
    for i in range(5, 0, -1):
        print(f"  Calibration starting in {i}...", flush=True)
        status_lab.text = f"CALIBRATION\n\nPlace MacBook FLAT\nScreen facing you\n\nStarting in {i}..."
        time.sleep(1)
else:
    # Headless countdown
    for i in range(5, 0, -1):
        print(f"  Calibration starting in {i}...", flush=True)
        time.sleep(1)

# Capture reference orientation
q_reference = capture_reference_quaternion(ser, n_samples=50)

# Store inverse of reference for relative orientation
q_reference_inv = quat_inv(q_reference)

if ENABLE_VISUALIZATION:
    status_lab.text = "✓ Calibration complete!\n\nStarting tracking..."
    time.sleep(2)

print("  Starting tracking NOW!")
status_lab.text = "✓ Tracking active!"
print("\nStarting real-time tracking...\n")

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
    
    def get_packet_rate(self):
        elapsed = time.time() - self.start_time
        if elapsed < 0.1:
            return 0.0
        return self.packet_count / elapsed

perf_mon = PerformanceMonitor()

# ============================================================================
# MAIN TRACKING LOOP
# ============================================================================
python_start_time = time.time()
frame_start = time.time()

print("="*60)
print("TRACKING ACTIVE")
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
        
        # Drain multiple packets per frame
        latest = None
        got_any = False
        try:
            for _ in range(MAX_PACKETS_PER_FRAME):
                packet = read_quaternion_packet(ser)
                if packet is None:
                    break
                
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
        
        # Use the latest packet for processing
        t_ms, q_raw = latest
        
        # Apply reference calibration: q_relative = q_reference_inv * q_raw
        # This makes the reference orientation become identity
        q = quat_mul(q_reference_inv, q_raw)
        
        # Calculate latency
        python_time_ms = (time.time() - python_start_time) * 1000
        latency_ms = python_time_ms - t_ms
        
        # Calculate Euler angles
        roll, pitch, yaw = quat_to_euler(q)
        
        # Log data if CSV enabled
        if csv_logger:
            csv_logger.log_sample(
                timestamp=time.time(),
                t_ms=t_ms,
                q=q,
                euler=(roll, pitch, yaw)
            )
        
        # Update performance monitor
        frame_time = time.time() - frame_start
        perf_mon.update(frame_time, latency_ms)
        frame_start = time.time()
        
        # Visualization updates (only if enabled)
        if ENABLE_VISUALIZATION:
            # Rotate basis vectors by quaternion
            r1x = rotate_vec_by_quat(ex, q)  # BNO085 X-axis in world frame
            r1y = rotate_vec_by_quat(ey, q)  # BNO085 Y-axis in world frame
            r1z = rotate_vec_by_quat(ez, q)  # BNO085 Z-axis in world frame
            
            # Map to scene coordinates
            body.axis = quat_to_scene(r1x)
            body.up = quat_to_scene(r1z)
            
            if SHOW_AXES:
                ax.pos = body.pos; ax.axis = quat_to_scene(r1x)
                ay.pos = body.pos; ay.axis = quat_to_scene(r1y)
                az.pos = body.pos; az.axis = quat_to_scene(r1z)
            
            # Status display
            status_lab.text = f"R={roll:+6.1f}° P={pitch:+6.1f}° Y={yaw:+6.1f}°\n" + \
                              f"Quaternion: [{q[0]:+.3f}, {q[1]:+.3f}, {q[2]:+.3f}, {q[3]:+.3f}]"
            
            # Performance stats
            if SHOW_PERFORMANCE_STATS and perf_lab:
                fps = perf_mon.get_fps()
                avg_lat = perf_mon.get_avg_latency()
                rate_hz = perf_mon.get_packet_rate()
                perf_lab.text = f"FPS: {fps:.1f} | Latency: {avg_lat:.1f}ms | Rate: {rate_hz:.1f} Hz | Packets: {perf_mon.packet_count}"
        else:
            # Headless mode: print periodic status
            if perf_mon.packet_count % 50 == 0:
                rate_hz = perf_mon.get_packet_rate()
                print(f"  [{perf_mon.packet_count:6d}] R={roll:+6.1f}° P={pitch:+6.1f}° Y={yaw:+6.1f}°  |  {rate_hz:.1f} Hz", end='\r')

except KeyboardInterrupt:
    print("\n\n✓ Stopped by user")
finally:
    # Clean up CSV logger
    if csv_logger:
        csv_logger.close()
