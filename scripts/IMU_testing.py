import time
import numpy as np
import serial
from vpython import canvas, box, vector, color, rate, arrow, label, sphere, compound

# -----------------------
# User settings
# -----------------------
PORT = "/dev/cu.usbmodem9888E00A0CC02"
BAUD = 115200

# Serial robustness
AUTO_RECONNECT = True
STALL_TIMEOUT_S = 3.0         # if no bytes received for this long, reconnect
RECONNECT_BACKOFF_S = 1.0     # wait before reopening the port

# Madgwick tuning
BETA = 0.08
GYRO_IS_DEG_PER_S = True

# NEW: drift reduction controls
BIAS_CALIB_SAMPLES = 150          # keep IMUs still while collecting these (reduced for faster calibration)
ACC_GATE_MIN_G = 0.85             # only use accel correction if |a| in this range
ACC_GATE_MAX_G = 1.15
SHOW_RELATIVE_IMU2_TO_IMU1 = False # best for stable visualization

# -----------------------
# Madgwick IMU (gyro+accel) filter
# -----------------------
def quat_mul(q, r):
    w0, x0, y0, z0 = q
    w1, x1, y1, z1 = r
    return np.array([
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1
    ], dtype=float)

def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)

def quat_norm(q):
    n = np.linalg.norm(q)
    if n == 0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n

def quat_inv(q):
    return quat_conj(q)  # unit quaternion inverse

def rotate_vec_by_quat(v, q):
    vq = np.array([0.0, v[0], v[1], v[2]], dtype=float)
    return quat_mul(quat_mul(q, vq), quat_conj(q))[1:]

class MadgwickIMU:
    def __init__(self, beta=0.1):
        self.beta = beta
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    # NEW: allow skipping accel correction when accel isn't reliable
    def update(self, gx, gy, gz, ax, ay, az, dt, use_accel=True):
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

# -----------------------
# Serial parsing (same as yours)
# -----------------------
def parse_line(line: str, debug=False):
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
        return t_ms, imu1, imu2
    except (ValueError, IndexError) as e:
        if debug:
            print(f"  Parse exception: {e} on line: '{line[:100]}'")
        return None

# SIMPLIFIED: Calibration - just collect gyro bias while IMUs are flat
def estimate_gyro_bias(ser, n_samples=150, timeout_s=60.0, label=None):
    """
    Collect gyro bias while IMUs are flat on ground.
    Simple and straightforward - just average the gyro readings.
    
    Returns: (bias1_gyro, bias2_gyro)
    """
    g1 = []
    g2 = []
    t0 = time.time()
    last_update = 0
    update_interval = 0.2  # Update progress every 200ms
    
    print("\nCollecting calibration samples (IMUs must be FLAT and STILL)...")
    
    while len(g1) < n_samples:
        if time.time() - t0 > timeout_s:
            if len(g1) >= 50:  # At least 50 samples is enough
                print(f"\nTimeout - using {len(g1)} samples")
                break
            else:
                raise RuntimeError(f"Calibration failed: only {len(g1)} samples collected")
        
        raw = ser.readline().decode("utf-8", errors="ignore").strip()
        if not raw:
            continue
        
        p = parse_line(raw, debug=False)
        if p is None:
            continue
        
        _, imu1, imu2 = p
        g1.append(imu1[:3])  # Just gyro data
        g2.append(imu2[:3])
        
        # Update progress bar
        if label is not None and (time.time() - last_update) >= update_interval:
            progress = len(g1) / n_samples
            bar_length = 30
            filled = int(bar_length * progress)
            bar = '█' * filled + '░' * (bar_length - filled)
            label.text = f"Calibrating: [{bar}] {len(g1)}/{n_samples} ({progress*100:.1f}%)\nKeep IMUs FLAT and STILL!"
            last_update = time.time()
            print(f"  {len(g1)}/{n_samples} samples...", end='\r')
    
    print(f"\nCalibration complete: {len(g1)} samples collected")
    
    # Calculate gyro bias (simple average)
    g1_arr = np.array(g1, dtype=float)
    g2_arr = np.array(g2, dtype=float)
    
    bias1 = np.mean(g1_arr, axis=0)
    bias2 = np.mean(g2_arr, axis=0)
    
    return bias1, bias2

# NEW: initialize quaternion from accelerometer (gravity alignment)
# When IMU is face up, gravity points down, so we align sensor frame with world frame
def quat_from_accel(ax, ay, az):
    """
    Initialize quaternion so that when IMU is face up:
    - Gravity (pointing down) aligns with world -Z axis
    - Sensor Z-axis (pointing up) aligns with world +Z axis
    - This ensures both IMUs start in the same reference frame when held face up
    """
    # Normalize accelerometer vector (should point down due to gravity when face up)
    a = np.array([ax, ay, az], dtype=float)
    a_norm = np.linalg.norm(a)
    if a_norm < 0.1:
        # No valid gravity reading, return identity
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    a = a / a_norm
    
    # World frame: gravity points down in -Z direction
    # When face up, sensor's gravity vector should point down
    world_down = np.array([0.0, 0.0, -1.0])
    
    # Find rotation from sensor frame to world frame
    # We want to rotate 'a' (sensor's gravity vector) to align with world_down
    v = np.cross(a, world_down)
    s = np.linalg.norm(v)
    c = np.dot(a, world_down)
    
    if s < 1e-6:
        # Vectors are parallel (or anti-parallel)
        if c > 0:
            # Already aligned
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        else:
            # 180 degree rotation needed - choose arbitrary perpendicular axis
            # Use Y-axis as rotation axis (preserves X-Y plane)
            return np.array([0.0, 0.0, 1.0, 0.0], dtype=float)
    
    # Use Rodrigues' rotation formula to get quaternion
    v = v / s
    angle = np.arccos(np.clip(c, -1.0, 1.0))
    half_angle = angle / 2.0
    sin_half = np.sin(half_angle)
    
    q = np.array([
        np.cos(half_angle),
        v[0] * sin_half,
        v[1] * sin_half,
        v[2] * sin_half
    ], dtype=float)
    
    return quat_norm(q)

# -----------------------
# VPython scene setup - Dice visualization
# -----------------------
scene = canvas(title="BMI160 Dual IMU Orientation Tracker", 
               width=1400, height=800, 
               center=vector(0, 0, 0),
               background=color.gray(0.1))

# Better camera angle - view from slightly above and in front
scene.forward = vector(-0.3, -0.4, -1)
scene.range = 2.5

# Add a ground plane for reference
ground = box(pos=vector(0, -0.6, 0), 
             length=5, height=0.02, width=3, 
             color=color.gray(0.3), opacity=0.3)

# Grid lines on ground for better depth perception
for i in range(-2, 3):
    # Lines along X
    box(pos=vector(0, -0.59, i*0.5), length=5, height=0.01, width=0.02, color=color.gray(0.4))
    # Lines along Z
    box(pos=vector(i*0.5, -0.59, 0), length=0.02, height=0.01, width=3, color=color.gray(0.4))

# Dice (proper): build each IMU as a single compound (cube + pips),
# so the dots rotate WITH the die.
spacing = 1.8
dice_size = 0.9

def _pip_uv(n: int, s: float):
    """2D pip layouts in face coordinates (u, v)."""
    c = 0.0
    a = s
    if n == 1:
        return [(c, c)]
    if n == 2:
        return [(-a, -a), (a, a)]
    if n == 3:
        return [(-a, -a), (c, c), (a, a)]
    if n == 4:
        return [(-a, -a), (-a, a), (a, -a), (a, a)]
    if n == 5:
        return [(-a, -a), (-a, a), (c, c), (a, -a), (a, a)]
    if n == 6:
        return [(-a, -a), (-a, c), (-a, a), (a, -a), (a, c), (a, a)]
    raise ValueError("pip number must be 1..6")

def make_die(pos, size=0.9, body_col=color.white):
    """
    Create a standard die as a compound.
    Face mapping (standard-ish):
      +X: 1  opposite -X: 6
      +Y: 2  opposite -Y: 5
      +Z: 3  opposite -Z: 4
    """
    # IMPORTANT: Build parts in LOCAL coordinates (around origin),
    # then pass `pos` directly to compound(). Parts positions are relative to compound's pos.
    half = size / 2.0
    pip_r = size * 0.085
    pip_spread = size * 0.22
    pip_inset = size * 0.03  # sink slightly into face

    origin = vector(0, 0, 0)
    parts = [box(pos=origin, length=size, height=size, width=size, color=body_col, opacity=1.0)]

    faces = [
        # (normal, u_axis, v_axis, number)
        (vector(1, 0, 0),  vector(0, 1, 0), vector(0, 0, 1), 1),
        (vector(-1, 0, 0), vector(0, 1, 0), vector(0, 0, 1), 6),
        (vector(0, 1, 0),  vector(1, 0, 0), vector(0, 0, 1), 2),
        (vector(0, -1, 0), vector(1, 0, 0), vector(0, 0, 1), 5),
        (vector(0, 0, 1),  vector(1, 0, 0), vector(0, 1, 0), 3),
        (vector(0, 0, -1), vector(1, 0, 0), vector(0, 1, 0), 4),
    ]

    for nrm, u_axis, v_axis, num in faces:
        face_center = origin + nrm * (half - pip_inset)
        for u, v in _pip_uv(num, pip_spread):
            parts.append(
                sphere(
                    pos=face_center + u_axis * u + v_axis * v,
                    radius=pip_r,
                    color=color.black,
                    opacity=1.0,
                )
            )

    # Create compound with pos parameter directly - parts are already in local coordinates
    die = compound(parts, pos=pos)
    return die

# Dice 1 / Dice 2
body1 = make_die(vector(-spacing, 0, 0), size=dice_size, body_col=color.white)
body2 = make_die(vector(spacing, 0, 0), size=dice_size, body_col=color.white)

# Labels for each IMU
label1 = label(pos=vector(-spacing, -0.75, 0), text="IMU 1",
               height=12, color=color.cyan, box=False, opacity=0)
label2 = label(pos=vector(spacing, -0.75, 0), text="IMU 2",
               height=12, color=color.orange, box=False, opacity=0)

def make_axes(origin, scale=1.0):
    """Create RGB axes (X=Red, Y=Green, Z=Blue)"""
    ax = arrow(pos=origin, axis=vector(scale, 0, 0), 
               color=color.red, shaftwidth=0.04, headwidth=0.08, headlength=0.12)
    ay = arrow(pos=origin, axis=vector(0, scale, 0), 
               color=color.green, shaftwidth=0.04, headwidth=0.08, headlength=0.12)
    az = arrow(pos=origin, axis=vector(0, 0, scale), 
               color=color.blue, shaftwidth=0.04, headwidth=0.08, headlength=0.12)
    return ax, ay, az

a1x, a1y, a1z = make_axes(body1.pos, scale=1.0)
a2x, a2y, a2z = make_axes(body2.pos, scale=1.0)

# Main status label at the top
lab = label(pos=vector(0, 1.8, 0), text="Waiting for serial...", 
            box=False, height=16, color=color.white, opacity=0)

# Legend for axes
legend = label(pos=vector(0, -1.5, 0), 
               text="IMU axes: X=Red | Y=Green | Z=Blue   (scene up = IMU Z)", 
               box=False, height=10, color=color.gray(0.7), opacity=0)

# -----------------------
# Run
# -----------------------
def open_serial():
    print(f"Opening serial port {PORT} @ {BAUD}...")
    s = serial.Serial(PORT, BAUD, timeout=1)
    # Arduino often resets on open; give it a moment.
    time.sleep(2.0)
    try:
        s.reset_input_buffer()
    except Exception:
        pass
    return s

def reopen_serial(old_ser):
    try:
        old_ser.close()
    except Exception:
        pass
    time.sleep(RECONNECT_BACKOFF_S)
    return open_serial()

ser = open_serial()
last_rx_time = time.time()

# Flush any stale data from the serial buffer
print("Flushing serial buffer...")
try:
    ser.reset_input_buffer()
except Exception:
    pass
time.sleep(0.5)

# Read a few lines to verify data is coming in
print("Verifying serial data...")
for i in range(5):
    try:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
    except Exception as e:
        print(f"  Serial read error during verify: {e}")
        if AUTO_RECONNECT:
            ser = reopen_serial(ser)
            continue
        else:
            raise
    if line:
        print(f"  Sample {i+1}: {line[:80]}")  # Print first 80 chars
        last_rx_time = time.time()

# Coordinate mapping: IMU frame -> VPython scene frame
# VPython uses +Y as "up" (ground plane normal). For a typical IMU board,
# "up" is +Z (out of the chip). Map:
#   scene_x = imu_x
#   scene_y = imu_z   (up)
#   scene_z = -imu_y  (to keep right-handed)
def imu_to_scene(v):
    return vector(float(v[0]), float(v[2]), float(-v[1]))

# Show reference orientation for calibration: LAYING FLAT ON THE GROUND
print("\nDisplaying reference orientation for calibration...")
ex = np.array([1.0, 0.0, 0.0])
ey = np.array([0.0, 1.0, 0.0])
ez = np.array([0.0, 0.0, 1.0])

# Reference: IMUs laying FLAT on the ground, chip facing UP (identity orientation)
# This is the ZERO/REFERENCE position for all measurements
# X forward, Y left, Z up (perpendicular to ground)
body1.axis = imu_to_scene(ex)  # IMU +X
body1.up   = imu_to_scene(ez)  # IMU +Z (up in scene)
body2.axis = imu_to_scene(ex)
body2.up   = imu_to_scene(ez)

# Set axes to identity (flat face up)
a1x.pos = body1.pos; a1x.axis = imu_to_scene(ex)
a1y.pos = body1.pos; a1y.axis = imu_to_scene(ey)
a1z.pos = body1.pos; a1z.axis = imu_to_scene(ez)
a2x.pos = body2.pos; a2x.axis = imu_to_scene(ex)
a2y.pos = body2.pos; a2y.axis = imu_to_scene(ey)
a2z.pos = body2.pos; a2z.axis = imu_to_scene(ez)

lab.text = "CALIBRATION REFERENCE SHOWN ABOVE\n\nLay BOTH IMUs FLAT ON THE GROUND:\n- Chip facing UP (visible)\n- Lying flat on a surface\n- Keep them STILL\n\nCalibration starting in 5 seconds..."
print("\n" + "="*60)
print("CALIBRATION SETUP:")
print("- The boxes above show the REFERENCE orientation")
print("- Lay BOTH physical IMUs FLAT ON THE GROUND:")
print("  • Place them on a flat surface (table, floor, etc.)")
print("  • Chip/components facing UP and visible")
print("  • Lying completely flat and horizontal")
print("  • Any rotation around vertical axis is fine")
print("  • This is the ZERO/REFERENCE position")
print("- Keep them STILL and in the SAME orientation")
print("="*60)
print("\nCalibration countdown:")

# Countdown timer
for i in range(5, 0, -1):
    print(f"  Starting in {i}...", flush=True)
    lab.text = f"CALIBRATION REFERENCE SHOWN ABOVE\n\nHold IMUs in this position!\n\nStarting in {i} seconds..."
    time.sleep(1)

print("  Calibrating NOW!\n")

# CALIBRATION: Collect gyro bias while IMUs are flat
print(f"\nStarting calibration...")
lab.text = f"CALIBRATING\nKeep IMUs FLAT and STILL!"

bias1_deg, bias2_deg = estimate_gyro_bias(ser, n_samples=BIAS_CALIB_SAMPLES, label=lab)

print(f"✓ Calibration complete!")
print(f"  Gyro bias IMU1: [{bias1_deg[0]:.2f}, {bias1_deg[1]:.2f}, {bias1_deg[2]:.2f}] deg/s")
print(f"  Gyro bias IMU2: [{bias2_deg[0]:.2f}, {bias2_deg[1]:.2f}, {bias2_deg[2]:.2f}] deg/s")

# Initialize Madgwick filters with IDENTITY quaternion
# Identity = flat on ground is the zero/reference orientation
f1 = MadgwickIMU(beta=BETA)
f2 = MadgwickIMU(beta=BETA)
f1.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)  # Identity
f2.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)  # Identity

lab.text = f"✓ Calibration complete!\n\nIMUs initialized.\nStarting tracking..."
print("\n✓ Filters initialized (identity quaternion)")
print("  Reference: IMUs flat on ground = zero orientation")
print("\nStarting real-time tracking in 2 seconds...")
time.sleep(2.0)

t_prev = None

def gyro_to_rads(g):
    return np.deg2rad(g) if GYRO_IS_DEG_PER_S else g

while True:
    rate(120)

    # Watchdog: if serial stalls, reconnect so you don't need to reset Arduino.
    if AUTO_RECONNECT and (time.time() - last_rx_time) > STALL_TIMEOUT_S:
        print(f"\n⚠ Serial stalled for > {STALL_TIMEOUT_S:.1f}s. Reconnecting...")
        ser = reopen_serial(ser)
        t_prev = None
        last_rx_time = time.time()
        continue

    try:
        raw = ser.readline().decode("utf-8", errors="ignore").strip()
    except serial.SerialException as e:
        print(f"\n⚠ SerialException: {e}")
        if AUTO_RECONNECT:
            ser = reopen_serial(ser)
            t_prev = None
            last_rx_time = time.time()
            continue
        raise
    except Exception as e:
        print(f"\n⚠ Serial read error: {e}")
        if AUTO_RECONNECT:
            ser = reopen_serial(ser)
            t_prev = None
            last_rx_time = time.time()
            continue
        raise

    if raw:
        last_rx_time = time.time()
    parsed = parse_line(raw)
    if parsed is None:
        continue

    t_ms, imu1, imu2 = parsed
    t_s = t_ms / 1000.0

    if t_prev is None:
        t_prev = t_s
        continue

    dt = t_s - t_prev
    if dt <= 0 or dt > 0.2:
        t_prev = t_s
        continue
    t_prev = t_s

    g1x, g1y, g1z, a1x_g, a1y_g, a1z_g = imu1
    g2x, g2y, g2z, a2x_g, a2y_g, a2z_g = imu2

    # NEW: subtract gyro bias (deg/s)
    g1_deg = np.array([g1x, g1y, g1z]) - bias1_deg
    g2_deg = np.array([g2x, g2y, g2z]) - bias2_deg

    gx1, gy1, gz1 = gyro_to_rads(g1_deg)
    gx2, gy2, gz2 = gyro_to_rads(g2_deg)

    # NEW: accel gating (trust accel only if magnitude near 1g)
    a1m = float(np.linalg.norm([a1x_g, a1y_g, a1z_g]))
    a2m = float(np.linalg.norm([a2x_g, a2y_g, a2z_g]))
    use_acc1 = (ACC_GATE_MIN_G <= a1m <= ACC_GATE_MAX_G)
    use_acc2 = (ACC_GATE_MIN_G <= a2m <= ACC_GATE_MAX_G)

    q1 = f1.update(gx1, gy1, gz1, a1x_g, a1y_g, a1z_g, dt, use_accel=use_acc1)
    q2 = f2.update(gx2, gy2, gz2, a2x_g, a2y_g, a2z_g, dt, use_accel=use_acc2)

    # NEW: relative visualization to reduce yaw drift in the view
    if SHOW_RELATIVE_IMU2_TO_IMU1:
        q1_vis = np.array([1.0, 0.0, 0.0, 0.0])
        q2_vis = quat_mul(quat_inv(q1), q2)
    else:
        q1_vis = q1
        q2_vis = q2

    r1x = rotate_vec_by_quat(ex, q1_vis)
    r1y = rotate_vec_by_quat(ey, q1_vis)
    r1z = rotate_vec_by_quat(ez, q1_vis)

    r2x = rotate_vec_by_quat(ex, q2_vis)
    r2y = rotate_vec_by_quat(ey, q2_vis)
    r2z = rotate_vec_by_quat(ez, q2_vis)

    body1.axis = imu_to_scene(r1x)
    body1.up   = imu_to_scene(r1z)

    body2.axis = imu_to_scene(r2x)
    body2.up   = imu_to_scene(r2z)

    a1x.pos = body1.pos; a1x.axis = imu_to_scene(r1x)
    a1y.pos = body1.pos; a1y.axis = imu_to_scene(r1y)
    a1z.pos = body1.pos; a1z.axis = imu_to_scene(r1z)

    a2x.pos = body2.pos; a2x.axis = imu_to_scene(r2x)
    a2y.pos = body2.pos; a2y.axis = imu_to_scene(r2y)
    a2z.pos = body2.pos; a2z.axis = imu_to_scene(r2z)

    lab.text = f"dt={dt*1000:.1f}ms beta={BETA} a1={a1m:.2f}g a2={a2m:.2f}g rel={int(SHOW_RELATIVE_IMU2_TO_IMU1)}"
