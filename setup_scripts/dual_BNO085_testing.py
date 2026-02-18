"""
Dual BNO085 UART-RVC IMU Tracking (Binary Protocol)
===================================================

Real-time orientation tracking with BNO085 9-DOF IMUs:
- Binary protocol (921600 baud, 500Hz) via STM32Reader
- On-device sensor fusion (no calibration needed)
- Real-time VPython visualization with dice representation
- CSV data logging
- Robust auto-reconnect

Hardware Requirements:
- STM32F401 with dual BNO085 IMUs in UART-RVC mode
- 921600 baud (High-speed binary)
- 500 Hz output rate
- BNO085 configured for UART-RVC (P0 high/bridged)

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
import sys
import csv
import argparse
from pathlib import Path
from collections import deque
from typing import Optional
from vpython import canvas, box, vector, color, rate, arrow, label, sphere, compound

# Allow running this file directly from anywhere (e.g. via absolute path) while
# still being able to import the project's top-level `src/` package.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# Import STM32 acquisition module
try:
    from setup_scripts.all_STM32_acquisition import STM32Reader, SampleSTM32
except ImportError:
    # Fallback if running from root
    from all_STM32_acquisition import STM32Reader, SampleSTM32

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
    parser.add_argument('--baud', type=int, default=921600,
                        help='Serial baud rate (default: 921600)')
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

def remap_vec(v, mapping):
    """Remap 3-vector with signed axis mapping tuple."""
    v = np.asarray(v, dtype=float)
    out = np.empty(3, dtype=float)
    for i, a in enumerate(mapping):
        s = 1.0 if a > 0 else -1.0
        idx = abs(int(a)) - 1
        out[i] = s * v[idx]
    return out

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
    
    def log_sample(self, timestamp, sample: SampleSTM32, a1_g, a2_g, q1, q2, euler1, euler2):
        """Log BNO085 sample with quaternions and Euler angles"""
        self.csv_writer.writerow([
            timestamp, sample.t_ms,
            a1_g[0], a1_g[1], a1_g[2],
            a2_g[0], a2_g[1], a2_g[2],
            q1[0], q1[1], q1[2], q1[3],
            q2[0], q2[1], q2[2], q2[3],
            euler1[0], euler1[1], euler1[2],
            euler2[0], euler2[1], euler2[2],
            sample.imu1_ok, sample.imu2_ok
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
            print(f"\\n✓ CSV saved: {self.filepath} ({self.sample_count} samples)")
    
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
print("\\n" + "="*60)
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
            self.color = color.white

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

print("\\n" + "="*60)
print("BNO085 MODE - Using raw sensor orientation")
print("="*60)
print("  BNO085 has on-device 9-DOF sensor fusion")
print("  Orientation is absolute (no calibration needed)")
print("\\nStarting tracking in 2 seconds...")
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
    
    status_lab.text = "BNO085 MODE\\n\\nStarting in 2 seconds..."

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

# Initialize STM32Reader
reader = STM32Reader(
    port=PORT,
    baud=BAUD,
    verbose=True,
    binary_mode=True  # Ensure binary mode is active
)
reader.start()

try:
    while True:
        loop_start = time.time()
        if ENABLE_VISUALIZATION:
            rate(120)  # VPython refresh rate cap
        
        # Get latest sample from STM32Reader
        latest_sample: Optional[SampleSTM32] = reader.latest
        
        if latest_sample is None:
            # Wait a bit if no data yet
            time.sleep(0.01)
            continue
        
        # Extract data from SampleSTM32
        # Note: STM32Reader provides Euler angles directly
        t_ms = latest_sample.t_ms
        
        # Accelerometer
        a1 = np.array([latest_sample.ax1, latest_sample.ay1, latest_sample.az1], dtype=float)
        a2 = np.array([latest_sample.ax2, latest_sample.ay2, latest_sample.az2], dtype=float)
        
        # Convert Euler (deg) to Quaternion
        q1 = euler_to_quat(latest_sample.roll1, latest_sample.pitch1, latest_sample.yaw1)
        q2 = euler_to_quat(latest_sample.roll2, latest_sample.pitch2, latest_sample.yaw2)
        
        # Axis map + optional accel invert (orientation from sensor used as-is)
        a1 = remap_vec(a1, IMU1_AXIS_MAP)
        a2 = remap_vec(a2, IMU2_AXIS_MAP)
        if INVERT_ACCEL:
            a1 = -a1
            a2 = -a2
        
        # Euler angles from quaternion (for consistency with prev pipeline, though we started with euler)
        # We can just use the source euler angles directly, but quat_to_euler ensures consistency 
        # with our coordinate frame transformations if any applied.
        roll1, pitch1, yaw1 = quat_to_euler(q1)
        roll2, pitch2, yaw2 = quat_to_euler(q2)
        
        # Calculate latency
        python_time_ms = (time.time() - python_start_time) * 1000
        latency_ms = python_time_ms - t_ms
        
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
            
            # Update dice orientation
            body1.axis = imu_to_scene(r1x)
            body1.up = imu_to_scene(r1z)
            body2.axis = imu_to_scene(r2x)
            body2.up = imu_to_scene(r2z)
            
            # Update separate axes arrows
            if SHOW_AXES and a1x:
                a1x.axis = imu_to_scene(r1x)
                a1y.axis = imu_to_scene(r1y)
                a1z.axis = imu_to_scene(r1z)
                
                a2x.axis = imu_to_scene(r2x)
                a2y.axis = imu_to_scene(r2y)
                a2z.axis = imu_to_scene(r2z)
            
            # Update labels
            if status_lab:
                status_lab.text = (f"FPS: {perf_mon.get_fps():.1f} | Latency: {perf_mon.get_avg_latency():.1f}ms | Loss: {perf_mon.get_packet_loss_rate():.1f}%\\n"
                                   f"IMU1: {latest_sample.imu1_ok} | IMU2: {latest_sample.imu2_ok}")
            
            if perf_mon.get_packet_loss_rate() > 5.0 and status_lab:
                status_lab.color = color.red
            elif status_lab:
                status_lab.color = color.white

except KeyboardInterrupt:
    print("\\nStopping...")
finally:
    reader.stop()
    if csv_logger:
        csv_logger.close()
