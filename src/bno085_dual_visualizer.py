"""
Dual BNO085 (RVC) VPython visualizer (native module)
====================================================

This is the "native" version of the former setup script:
`setup_scripts/dual_BNO085_testing.py`.

It uses `src.imu_acquisition.IMUDevice` with `IMUType.BNO085_DUAL_RVC` and
renders two dice (sensor 1 + sensor 2) plus RGB axes.

Arduino CSV format (Adafruit BNO08x_RVC standard):
  t_ms,s1_y,s1_p,s1_r,s2_y,s2_p,s2_r
  where y=yaw, p=pitch, r=roll (degrees)
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .imu_acquisition import IMUConfig, IMUDevice, IMUType, rotate_vec_by_quat


@dataclass
class VizConfig:
    # Serial
    port: Optional[str] = None  # None = auto
    baud: int = 115200

    # VPython
    fps: int = 120
    axis_len: float = 1.0
    dice_size: float = 1.2
    die_separation: float = 2.0

    # Optional corrections (pass-through by default; see IMUConfig)
    flip_yaw: bool = False
    swap_pitch_roll: bool = False
    disable_yaw: bool = False

    # Calibration (reference pose)
    calibrate_on_start: bool = True
    calib_samples: int = 50


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Dual BNO085 (RVC) VPython visualizer")
    p.add_argument("--port", default="auto", help='Serial port device (or "auto")')
    p.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    p.add_argument("--fps", type=int, default=120, help="Render FPS cap (default: 120)")
    p.add_argument("--flip-yaw", action="store_true", help="Flip yaw sign (mounting correction)")
    p.add_argument("--swap-pitch-roll", action="store_true", help="Swap pitch/roll (mounting correction)")
    p.add_argument("--disable-yaw", action="store_true", help="Zero yaw (show only pitch/roll)")
    p.add_argument("--no-calibrate", action="store_true", help="Disable reference calibration on start")
    p.add_argument("--calib-samples", type=int, default=50, help="Calibration samples (default: 50)")
    return p.parse_args()


def main(cfg: Optional[VizConfig] = None) -> None:
    # Import VPython lazily so this module can be imported headless if needed.
    from vpython import (  # type: ignore
        arrow,
        box,
        canvas,
        color,
        compound,
        label,
        rate,
        sphere,
        vector,
    )

    if cfg is None:
        args = _parse_args()
        cfg = VizConfig(
            port=None if str(args.port).lower() == "auto" else str(args.port),
            baud=int(args.baud),
            fps=int(args.fps),
            flip_yaw=bool(args.flip_yaw),
            swap_pitch_roll=bool(args.swap_pitch_roll),
            disable_yaw=bool(args.disable_yaw),
            calibrate_on_start=not bool(args.no_calibrate),
            calib_samples=int(args.calib_samples),
        )

    def create_die(pos, size=1.2, body_color=color.white):
        """Create a dice with pips."""
        half = size / 2.0
        pip_radius = size * 0.085
        pip_spread = size * 0.22
        pip_depth = size * 0.03

        parts = [box(pos=vector(0, 0, 0), length=size, height=size, width=size, color=body_color)]

        faces = [
            (vector(half - pip_depth, 0, 0), 1),      # Right face
            (vector(-half + pip_depth, 0, 0), 6),     # Left face
            (vector(0, half - pip_depth, 0), 2),      # Top face
            (vector(0, -half + pip_depth, 0), 5),     # Bottom face
            (vector(0, 0, half - pip_depth), 3),      # Front face
            (vector(0, 0, -half + pip_depth), 4),     # Back face
        ]

        pip_patterns = {
            1: [(0, 0)],
            2: [(-pip_spread, -pip_spread), (pip_spread, pip_spread)],
            3: [(-pip_spread, -pip_spread), (0, 0), (pip_spread, pip_spread)],
            4: [(-pip_spread, -pip_spread), (-pip_spread, pip_spread),
                (pip_spread, -pip_spread), (pip_spread, pip_spread)],
            5: [(-pip_spread, -pip_spread), (-pip_spread, pip_spread), (0, 0),
                (pip_spread, -pip_spread), (pip_spread, pip_spread)],
            6: [(-pip_spread, -pip_spread), (-pip_spread, 0), (-pip_spread, pip_spread),
                (pip_spread, -pip_spread), (pip_spread, 0), (pip_spread, pip_spread)],
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

    def create_floor(size=10, y_pos=-2):
        """Create a ground plane/floor."""
        floor = box(
            pos=vector(0, y_pos, 0),
            length=size,
            width=size,
            height=0.1,
            color=color.gray(0.3),
        )
        grid_parts = [floor]
        grid_spacing = 1.0
        line_thickness = 0.02

        for i in range(-int(size / 2), int(size / 2) + 1):
            grid_parts.append(box(
                pos=vector(0, y_pos + 0.06, i * grid_spacing),
                length=size,
                width=line_thickness,
                height=0.01,
                color=color.gray(0.4),
            ))
            grid_parts.append(box(
                pos=vector(i * grid_spacing, y_pos + 0.06, 0),
                length=line_thickness,
                width=size,
                height=0.01,
                color=color.gray(0.4),
            ))
        return compound(grid_parts)

    # Scene
    scene = canvas(
        title="Dual BNO085 IMU (RVC) - Dice Visualization",
        width=1400,
        height=800,
        background=color.gray(0.15),
    )
    scene.forward = vector(-0.3, -0.5, -1)
    scene.range = 4.0

    create_floor(size=12, y_pos=-2.0)

    dice1 = create_die(vector(-cfg.die_separation / 2, 0, 0), size=cfg.dice_size, body_color=color.white)
    dice2 = create_die(vector(cfg.die_separation / 2, 0, 0), size=cfg.dice_size, body_color=color.orange)

    ax1 = arrow(pos=dice1.pos, axis=vector(cfg.axis_len, 0, 0), color=color.red, shaftwidth=0.05)
    ay1 = arrow(pos=dice1.pos, axis=vector(0, cfg.axis_len, 0), color=color.green, shaftwidth=0.05)
    az1 = arrow(pos=dice1.pos, axis=vector(0, 0, cfg.axis_len), color=color.blue, shaftwidth=0.05)

    ax2 = arrow(pos=dice2.pos, axis=vector(cfg.axis_len, 0, 0), color=color.red, shaftwidth=0.05)
    ay2 = arrow(pos=dice2.pos, axis=vector(0, cfg.axis_len, 0), color=color.green, shaftwidth=0.05)
    az2 = arrow(pos=dice2.pos, axis=vector(0, 0, cfg.axis_len), color=color.blue, shaftwidth=0.05)

    status = label(
        pos=vector(0, 2.5, 0),
        text="Connecting...",
        box=False,
        height=16,
        color=color.white,
    )
    legend = label(
        pos=vector(0, -2.5, 0),
        text="White=Sensor1 | Orange=Sensor2 | Axes: X=Red Y=Green Z=Blue",
        box=False,
        height=10,
        color=color.gray(0.7),
    )

    # Device
    imu_cfg = IMUConfig(
        imu_type=IMUType.BNO085_DUAL_RVC,
        port=cfg.port,
        baud=cfg.baud,
        bno085_rvc_flip_yaw=cfg.flip_yaw,
        bno085_rvc_swap_pitch_roll=cfg.swap_pitch_roll,
        bno085_rvc_disable_yaw=cfg.disable_yaw,
        bno085_calib_samples=cfg.calib_samples,
    )

    with IMUDevice(imu_cfg) as imu:
        status.text = "Connected. Waiting for data..."
        if cfg.calibrate_on_start:
            status.text = f"Calibrating ({cfg.calib_samples} samples)..."
            imu.calibrate(samples=cfg.calib_samples)

        status.text = "Reading data..."

        ex = np.array([1.0, 0.0, 0.0])
        ey = np.array([0.0, 1.0, 0.0])
        ez = np.array([0.0, 0.0, 1.0])

        while True:
            rate(cfg.fps)
            reading = imu.read()
            if reading is None or reading.quat1 is None or reading.quat2 is None:
                continue

            # Rotate basis vectors with quaternions
            r1x = rotate_vec_by_quat(ex, reading.quat1)
            r1y = rotate_vec_by_quat(ey, reading.quat1)
            r1z = rotate_vec_by_quat(ez, reading.quat1)

            r2x = rotate_vec_by_quat(ex, reading.quat2)
            r2y = rotate_vec_by_quat(ey, reading.quat2)
            r2z = rotate_vec_by_quat(ez, reading.quat2)

            x1 = vector(float(r1x[0]), float(r1x[1]), float(r1x[2]))
            y1 = vector(float(r1y[0]), float(r1y[1]), float(r1y[2]))
            z1 = vector(float(r1z[0]), float(r1z[1]), float(r1z[2]))

            x2 = vector(float(r2x[0]), float(r2x[1]), float(r2x[2]))
            y2 = vector(float(r2y[0]), float(r2y[1]), float(r2y[2]))
            z2 = vector(float(r2z[0]), float(r2z[1]), float(r2z[2]))

            # Dice orientation: axis is local +X, up is local +Y
            dice1.axis = x1
            dice1.up = y1
            dice2.axis = x2
            dice2.up = y2

            ax1.axis = x1 * cfg.axis_len
            ay1.axis = y1 * cfg.axis_len
            az1.axis = z1 * cfg.axis_len

            ax2.axis = x2 * cfg.axis_len
            ay2.axis = y2 * cfg.axis_len
            az2.axis = z2 * cfg.axis_len

            if reading.euler1 is not None and reading.euler2 is not None:
                r1, p1, y1d = reading.euler1
                r2d, p2d, y2d = reading.euler2
                status.text = (
                    f"t={reading.t_us/1000.0:.0f} ms\n"
                    f"Sensor 1: Y={y1d:+6.1f}° P={p1:+6.1f}° R={r1:+6.1f}°\n"
                    f"Sensor 2: Y={y2d:+6.1f}° P={p2d:+6.1f}° R={r2d:+6.1f}°"
                )


if __name__ == "__main__":
    main()

