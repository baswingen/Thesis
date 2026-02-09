"""
STM32 signal verification – real-time visualization
====================================================

Connects to the same STM32 serial stream as all_STM32_acquisition.py and
visualizes in real time:
- Incoming IMU signals (dual BNO085: yaw, pitch, roll vs time)
- Button matrix state (keys_mask vs time)
- PRBS (prbs_tick and in_mark from CSV)

Protocol: 115200 baud, 500 Hz CSV, 20 columns (same as the sketch).
Use this script to verify that IMU, buttons, and PRBS are streaming correctly.

Usage:
  python stm32_visualization_test.py [--port PORT] [--baud BAUD] [--window SEC] [--duration SEC]
"""

import time
import sys
import argparse
import math
from pathlib import Path
from dataclasses import dataclass
from collections import deque
from typing import Optional

# Project root for src imports
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import serial
from src.arduino_connection import open_arduino_serial

try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    import numpy as np
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    plt = None
    np = None

# ============================================================================
# CLI
# ============================================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="Real-time STM32 signal verification: IMU, buttons, PRBS"
    )
    parser.add_argument("--port", type=str, default="auto",
                        help="Serial port (default: auto)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--window", type=float, default=10.0,
                        help="Plot time window in seconds (default: 10)")
    parser.add_argument("--duration", type=float, default=None,
                        help="Run for N seconds then exit (optional)")
    return parser.parse_args()

args = parse_args()
PORT = None if (args.port or "auto").lower() == "auto" else args.port
BAUD = args.baud
WINDOW_SEC = args.window
DURATION_S = args.duration

STALL_TIMEOUT_S = 3.0
RECONNECT_BACKOFF_S = 1.0
MAX_SAMPLES = max(100, int(WINDOW_SEC * 500))  # rolling buffer size

# ============================================================================
# Parser and data (in-file copy, same protocol as all_STM32_acquisition)
# ============================================================================

@dataclass
class SampleSTM32:
    """One parsed STM32 CSV row (20 columns). Missing IMU fields are float('nan')."""
    t_ms: float
    imu1_ok: int
    imu2_ok: int
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
    keys_mask: int
    keys_rise: int
    keys_fall: int
    prbs_tick: int
    in_mark: int


def _parse_float(s: str) -> float:
    s = s.strip()
    if s == "":
        return math.nan
    try:
        return float(s)
    except ValueError:
        return math.nan


def parse_line_stm32(line: str) -> Optional[SampleSTM32]:
    """Parse one STM32 CSV line (20 columns). Returns SampleSTM32 or None."""
    line = line.strip()
    if not line:
        return None
    if line.startswith("OK:") or line.startswith("ERR:") or line.startswith("STM32") or line.startswith("#"):
        return None
    if line.startswith("t_ms,") or line.startswith("Columns:"):
        return None
    parts = line.split(",")
    if len(parts) != 20:
        return None
    try:
        t_ms = _parse_float(parts[0])
        imu1_ok = int(parts[1].strip() or "0")
        imu2_ok = int(parts[2].strip() or "0")
        yaw1 = _parse_float(parts[3])
        pitch1 = _parse_float(parts[4])
        roll1 = _parse_float(parts[5])
        ax1 = _parse_float(parts[6])
        ay1 = _parse_float(parts[7])
        az1 = _parse_float(parts[8])
        yaw2 = _parse_float(parts[9])
        pitch2 = _parse_float(parts[10])
        roll2 = _parse_float(parts[11])
        ax2 = _parse_float(parts[12])
        ay2 = _parse_float(parts[13])
        az2 = _parse_float(parts[14])
        keys_mask = int(parts[15].strip() or "0")
        keys_rise = int(parts[16].strip() or "0")
        keys_fall = int(parts[17].strip() or "0")
        prbs_tick = int(parts[18].strip() or "0")
        in_mark = int(parts[19].strip() or "0")
    except (ValueError, IndexError):
        return None
    return SampleSTM32(
        t_ms=t_ms, imu1_ok=imu1_ok, imu2_ok=imu2_ok,
        yaw1=yaw1, pitch1=pitch1, roll1=roll1, ax1=ax1, ay1=ay1, az1=az1,
        yaw2=yaw2, pitch2=pitch2, roll2=roll2, ax2=ax2, ay2=ay2, az2=az2,
        keys_mask=keys_mask, keys_rise=keys_rise, keys_fall=keys_fall,
        prbs_tick=prbs_tick, in_mark=in_mark,
    )


# ============================================================================
# Serial
# ============================================================================

def open_serial():
    """Open serial connection (auto or specified port)."""
    if PORT is None:
        return open_arduino_serial(
            port=None,
            baud=BAUD,
            timeout=0.25,
            wait_for_ready=True,
            ready_timeout=5.0,
            verbose=True,
        )
    return open_arduino_serial(
        port=PORT,
        baud=BAUD,
        timeout=0.25,
        wait_for_ready=True,
        ready_timeout=5.0,
        verbose=True,
    )


def reopen_serial(old_ser):
    """Close and reopen serial connection."""
    try:
        old_ser.close()
    except Exception:
        pass
    time.sleep(RECONNECT_BACKOFF_S)
    return open_serial()


# ============================================================================
# Buffers and figure
# ============================================================================

def create_buffers():
    """Create deques for rolling data (maxlen = MAX_SAMPLES)."""
    return {
        "t_sec": deque(maxlen=MAX_SAMPLES),
        "yaw1": deque(maxlen=MAX_SAMPLES),
        "pitch1": deque(maxlen=MAX_SAMPLES),
        "roll1": deque(maxlen=MAX_SAMPLES),
        "yaw2": deque(maxlen=MAX_SAMPLES),
        "pitch2": deque(maxlen=MAX_SAMPLES),
        "roll2": deque(maxlen=MAX_SAMPLES),
        "keys_mask": deque(maxlen=MAX_SAMPLES),
        "prbs_tick": deque(maxlen=MAX_SAMPLES),
        "in_mark": deque(maxlen=MAX_SAMPLES),
    }


def append_sample(buffers: dict, s: SampleSTM32):
    """Append one sample to buffers. NaN is appended for missing IMU fields."""
    t_sec = s.t_ms / 1000.0
    buffers["t_sec"].append(t_sec)
    buffers["yaw1"].append(s.yaw1)
    buffers["pitch1"].append(s.pitch1)
    buffers["roll1"].append(s.roll1)
    buffers["yaw2"].append(s.yaw2)
    buffers["pitch2"].append(s.pitch2)
    buffers["roll2"].append(s.roll2)
    buffers["keys_mask"].append(float(s.keys_mask))
    buffers["prbs_tick"].append(float(s.prbs_tick))
    buffers["in_mark"].append(float(s.in_mark))


def setup_figure():
    """Create figure with 4 subplots: IMU1, IMU2, Buttons, PRBS."""
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("STM32 signal verification – IMU, buttons, PRBS", fontsize=12, fontweight="bold")

    # IMU1
    ax1 = axes[0]
    ax1.set_ylabel("deg")
    ax1.set_title("IMU1 (yaw, pitch, roll)")
    ax1.grid(True, alpha=0.3)
    line_yaw1, = ax1.plot([], [], "r-", linewidth=1, label="yaw1")
    line_pitch1, = ax1.plot([], [], "g-", linewidth=1, label="pitch1")
    line_roll1, = ax1.plot([], [], "b-", linewidth=1, label="roll1")
    ax1.legend(loc="upper right", fontsize=8)

    # IMU2
    ax2 = axes[1]
    ax2.set_ylabel("deg")
    ax2.set_title("IMU2 (yaw, pitch, roll)")
    ax2.grid(True, alpha=0.3)
    line_yaw2, = ax2.plot([], [], "r-", linewidth=1, label="yaw2")
    line_pitch2, = ax2.plot([], [], "g-", linewidth=1, label="pitch2")
    line_roll2, = ax2.plot([], [], "b-", linewidth=1, label="roll2")
    ax2.legend(loc="upper right", fontsize=8)

    # Buttons
    ax3 = axes[2]
    ax3.set_ylabel("keys_mask")
    ax3.set_title("Button matrix (keys_mask 0..511)")
    ax3.grid(True, alpha=0.3)
    line_keys, = ax3.plot([], [], "k-", linewidth=1)
    ax3.set_ylim(-0.5, 512)

    # PRBS: prbs_tick (left) and in_mark (right twin)
    ax4 = axes[3]
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("prbs_tick")
    ax4.set_title("PRBS (tick + in_mark)")
    ax4.grid(True, alpha=0.3)
    line_prbs, = ax4.plot([], [], "b-", linewidth=0.8, label="prbs_tick")
    ax4_twin = ax4.twinx()
    ax4_twin.set_ylabel("in_mark")
    line_mark, = ax4_twin.plot([], [], "orange", linewidth=1, label="in_mark")
    ax4.legend(loc="upper left", fontsize=8)
    ax4_twin.legend(loc="upper right", fontsize=8)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.ion()
    plt.show(block=False)

    return {
        "fig": fig,
        "axes": axes,
        "lines": {
            "yaw1": line_yaw1, "pitch1": line_pitch1, "roll1": line_roll1,
            "yaw2": line_yaw2, "pitch2": line_pitch2, "roll2": line_roll2,
            "keys_mask": line_keys,
            "prbs_tick": line_prbs, "in_mark": line_mark,
        },
        "ax4_twin": ax4_twin,
    }


def update_plots(plot_ctx: dict, buffers: dict):
    """Update line data from buffers (lists for matplotlib)."""
    t = list(buffers["t_sec"])
    if not t:
        return
    lines = plot_ctx["lines"]
    lines["yaw1"].set_data(t, list(buffers["yaw1"]))
    lines["pitch1"].set_data(t, list(buffers["pitch1"]))
    lines["roll1"].set_data(t, list(buffers["roll1"]))
    lines["yaw2"].set_data(t, list(buffers["yaw2"]))
    lines["pitch2"].set_data(t, list(buffers["pitch2"]))
    lines["roll2"].set_data(t, list(buffers["roll2"]))
    lines["keys_mask"].set_data(t, list(buffers["keys_mask"]))
    lines["prbs_tick"].set_data(t, list(buffers["prbs_tick"]))
    lines["in_mark"].set_data(t, list(buffers["in_mark"]))

    # Auto-scale time axis to last WINDOW_SEC
    ax0 = plot_ctx["axes"][0]
    t_min, t_max = min(t), max(t)
    if t_max > t_min:
        ax0.set_xlim(max(t_min, t_max - WINDOW_SEC), t_max)
    plot_ctx["fig"].canvas.draw_idle()
    plt.pause(0.001)


# ============================================================================
# Main
# ============================================================================

def main():
    if not MATPLOTLIB_AVAILABLE:
        print("Warning: matplotlib not available – visualization disabled")
        print("Install with: pip install matplotlib numpy")
        sys.exit(1)

    buffers = create_buffers()
    plot_ctx = setup_figure()
    ser = open_serial()
    last_rx_time = time.time()
    start_time = time.time()

    print("Waiting for first valid data line...")
    time.sleep(0.5)
    ser.reset_input_buffer()

    try:
        while True:
            if DURATION_S is not None and (time.time() - start_time) >= DURATION_S:
                print(f"\nDuration {DURATION_S}s reached. Exiting.")
                break

            if (time.time() - last_rx_time) > STALL_TIMEOUT_S:
                print(f"\nStall > {STALL_TIMEOUT_S}s. Reconnecting...")
                ser = reopen_serial(ser)
                last_rx_time = time.time()
                continue

            try:
                raw = ser.readline()
            except serial.SerialException as e:
                print(f"\nSerial error: {e}")
                ser = reopen_serial(ser)
                last_rx_time = time.time()
                continue

            if not raw:
                plt.pause(0.001)
                continue
            try:
                line = raw.decode("ascii", errors="ignore")
            except Exception:
                continue

            sample = parse_line_stm32(line)
            if sample is None:
                plt.pause(0.001)
                continue

            last_rx_time = time.time()
            append_sample(buffers, sample)
            update_plots(plot_ctx, buffers)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        print("Close the plot window to exit.")
        plt.show(block=True)


if __name__ == "__main__":
    main()
