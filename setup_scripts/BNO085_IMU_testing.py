"""
Minimal BNO085 Dual IMU (RVC) VPython Visualizer
================================================

Arduino CSV format (your sketch):
  t_ms,s1_y,s1_p,s1_r,s2_y,s2_p,s2_r

This script keeps things intentionally simple:
- Parse the CSV stream
- Apply the two corrections you asked for:
  1) Flip rotation direction around Z  -> yaw = -yaw
  2) Swap rotations around X and Y     -> (pitch, roll) = (roll, pitch)
- Convert yaw/pitch/roll -> quaternion using the standard ZYX formula
- Convert quaternion -> rotation matrix -> body axes in world
- Drive VPython objects directly from those axes

Notes:
- VPython box/compound orientation uses:
  - `.axis` as the object's local +X direction
  - `.up`   as the object's local +Y direction
  We set axis = IMU X-axis, up = IMU Y-axis (right-hand rule).
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Optional

import serial  # noqa: F401  (needed by open_arduino_serial typing)
from vpython import arrow, box, canvas, color, label, rate, sphere, vector, compound

# Add parent directory to path so we can import src.arduino_connection
sys.path.insert(0, str(Path(__file__).parent.parent))
from src.arduino_connection import open_arduino_serial


# =========================
# USER TUNABLE CONSTANTS
# =========================

# Serial
BAUD = 115200
PORT = "auto"  # "auto" or explicit port string

# Fixes requested by you
FLIP_YAW = True           # Flip rotation direction around Z
SWAP_PITCH_ROLL = True    # X and Y rotations swapped (roll<->pitch)

# VPython settings
FPS = 120
AXIS_LEN = 1.0


# =========================
# MATH HELPERS (minimal)
# =========================

def _quat_normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    inv = 1.0 / n
    return (w * inv, x * inv, y * inv, z * inv)


def euler_zyx_deg_to_quat(yaw_deg: float, pitch_deg: float, roll_deg: float) -> tuple[float, float, float, float]:
    """
    Yaw (Z), Pitch (Y), Roll (X) in degrees -> quaternion (w,x,y,z).
    This matches the standard formula you pasted.
    """
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return _quat_normalize((w, x, y, z))


def quat_to_rotm(q: tuple[float, float, float, float]) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    """Quaternion (w,x,y,z) -> 3x3 rotation matrix."""
    w, x, y, z = q

    ww = w * w
    xx = x * x
    yy = y * y
    zz = z * z

    wx = w * x
    wy = w * y
    wz = w * z
    xy = x * y
    xz = x * z
    yz = y * z

    r00 = ww + xx - yy - zz
    r01 = 2.0 * (xy - wz)
    r02 = 2.0 * (xz + wy)

    r10 = 2.0 * (xy + wz)
    r11 = ww - xx + yy - zz
    r12 = 2.0 * (yz - wx)

    r20 = 2.0 * (xz - wy)
    r21 = 2.0 * (yz + wx)
    r22 = ww - xx - yy + zz

    return ((r00, r01, r02), (r10, r11, r12), (r20, r21, r22))


def rotm_axes(rotm) -> tuple[vector, vector, vector]:
    """
    Return the body axes (x_axis, y_axis, z_axis) expressed in world coordinates.
    Columns of R are the images of the body basis vectors.
    """
    (r00, r01, r02), (r10, r11, r12), (r20, r21, r22) = rotm
    x_axis = vector(r00, r10, r20)
    y_axis = vector(r01, r11, r21)
    z_axis = vector(r02, r12, r22)
    return x_axis, y_axis, z_axis


def apply_requested_fixes(yaw: float, pitch: float, roll: float) -> tuple[float, float, float]:
    if FLIP_YAW:
        yaw = -yaw
    if SWAP_PITCH_ROLL:
        pitch, roll = roll, pitch
    return yaw, pitch, roll


# =========================
# SERIAL PARSING
# =========================

def parse_rvc_line(line: str) -> Optional[tuple[int, float, float, float, float, float, float]]:
    line = line.strip()
    if not line or line.startswith("#") or line.startswith("t_ms"):
        return None
    parts = line.split(",")
    if len(parts) != 7:
        return None
    try:
        t_ms = int(parts[0])
        s1_y, s1_p, s1_r = float(parts[1]), float(parts[2]), float(parts[3])
        s2_y, s2_p, s2_r = float(parts[4]), float(parts[5]), float(parts[6])
        return (t_ms, s1_y, s1_p, s1_r, s2_y, s2_p, s2_r)
    except ValueError:
        return None


def read_sample(ser) -> Optional[tuple[int, float, float, float, float, float, float]]:
    raw = ser.readline()
    if not raw:
        return None
    return parse_rvc_line(raw.decode("ascii", errors="ignore"))


# =========================
# VPYTHON SCENE
# =========================

def create_die(pos, size=1.2, body_color=color.white):
    half = size / 2.0
    pip_radius = size * 0.085
    pip_spread = size * 0.22
    pip_depth = size * 0.03

    parts = [box(pos=vector(0, 0, 0), length=size, height=size, width=size, color=body_color)]

    faces = [
        (vector(half - pip_depth, 0, 0), 1),
        (vector(-half + pip_depth, 0, 0), 6),
        (vector(0, half - pip_depth, 0), 2),
        (vector(0, -half + pip_depth, 0), 5),
        (vector(0, 0, half - pip_depth), 3),
        (vector(0, 0, -half + pip_depth), 4),
    ]

    pip_patterns = {
        1: [(0, 0)],
        2: [(-pip_spread, -pip_spread), (pip_spread, pip_spread)],
        3: [(-pip_spread, -pip_spread), (0, 0), (pip_spread, pip_spread)],
        4: [(-pip_spread, -pip_spread), (-pip_spread, pip_spread), (pip_spread, -pip_spread), (pip_spread, pip_spread)],
        5: [(-pip_spread, -pip_spread), (-pip_spread, pip_spread), (0, 0), (pip_spread, -pip_spread), (pip_spread, pip_spread)],
        6: [
            (-pip_spread, -pip_spread),
            (-pip_spread, 0),
            (-pip_spread, pip_spread),
            (pip_spread, -pip_spread),
            (pip_spread, 0),
            (pip_spread, pip_spread),
        ],
    }

    for face_pos, num_pips in faces:
        for u, v in pip_patterns[num_pips]:
            if abs(face_pos.x) > 0.4:
                pip_pos = face_pos + vector(0, u, v)
            elif abs(face_pos.y) > 0.4:
                pip_pos = face_pos + vector(u, 0, v)
            else:
                pip_pos = face_pos + vector(u, v, 0)
            parts.append(sphere(pos=pip_pos, radius=pip_radius, color=color.black))

    return compound(parts, pos=pos)


scene = canvas(title="BNO085 Dual IMU (RVC) - Minimal", width=1400, height=800, background=color.gray(0.1))
scene.forward = vector(-0.3, -0.4, -1)
scene.range = 3.0

dice1 = create_die(vector(-1.5, 0, 0), size=1.2, body_color=color.white)
dice2 = create_die(vector(1.5, 0, 0), size=1.2, body_color=color.orange)

ax1 = arrow(pos=dice1.pos, axis=vector(AXIS_LEN, 0, 0), color=color.red, shaftwidth=0.05)
ay1 = arrow(pos=dice1.pos, axis=vector(0, AXIS_LEN, 0), color=color.green, shaftwidth=0.05)
az1 = arrow(pos=dice1.pos, axis=vector(0, 0, AXIS_LEN), color=color.blue, shaftwidth=0.05)

ax2 = arrow(pos=dice2.pos, axis=vector(AXIS_LEN, 0, 0), color=color.red, shaftwidth=0.05)
ay2 = arrow(pos=dice2.pos, axis=vector(0, AXIS_LEN, 0), color=color.green, shaftwidth=0.05)
az2 = arrow(pos=dice2.pos, axis=vector(0, 0, AXIS_LEN), color=color.blue, shaftwidth=0.05)

status = label(pos=vector(0, 1.5, 0), text="Connecting...", box=False, height=16, color=color.white)
legend = label(
    pos=vector(0, -1.3, 0),
    text=f"Fixes: FLIP_YAW={FLIP_YAW} | SWAP_PITCH_ROLL={SWAP_PITCH_ROLL}  |  Colors: X=Red, Y=Green, Z=Blue",
    box=False,
    height=10,
    color=color.gray(0.7),
)


# =========================
# MAIN LOOP
# =========================

print("Connecting to Arduino...")
ser = open_arduino_serial(
    port=None if PORT == "auto" else PORT,
    baud=BAUD,
    timeout=0.25,
    wait_for_ready=True,
    ready_timeout=5.0,
    verbose=True,
)
time.sleep(0.2)

status.text = "Reading data..."

while True:
    rate(FPS)
    sample = read_sample(ser)
    if sample is None:
        continue

    t_ms, s1_y, s1_p, s1_r, s2_y, s2_p, s2_r = sample

    # Apply your requested fixes before converting to quaternion
    y1, p1, r1 = apply_requested_fixes(s1_y, s1_p, s1_r)
    y2, p2, r2 = apply_requested_fixes(s2_y, s2_p, s2_r)

    q1 = euler_zyx_deg_to_quat(y1, p1, r1)
    q2 = euler_zyx_deg_to_quat(y2, p2, r2)

    R1 = quat_to_rotm(q1)
    R2 = quat_to_rotm(q2)

    x1, y1v, z1 = rotm_axes(R1)
    x2, y2v, z2 = rotm_axes(R2)

    # Orient dice: local X = IMU X, local Y = IMU Y
    dice1.axis = x1
    dice1.up = y1v
    dice2.axis = x2
    dice2.up = y2v

    # Draw axes
    ax1.axis, ay1.axis, az1.axis = x1, y1v, z1
    ax2.axis, ay2.axis, az2.axis = x2, y2v, z2

    status.text = (
        f"t={t_ms} ms\n"
        f"S1 raw: yaw={s1_y:+6.1f} pitch={s1_p:+6.1f} roll={s1_r:+6.1f}\n"
        f"S2 raw: yaw={s2_y:+6.1f} pitch={s2_p:+6.1f} roll={s2_r:+6.1f}"
    )

