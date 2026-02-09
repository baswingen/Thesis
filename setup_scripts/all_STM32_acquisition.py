"""
General STM32 acquisition
==========================

Acquires all signals from the STM32F401 sketch (STM32_all_in_python.ino):
- Dual BNO085 UART-RVC IMU (yaw, pitch, roll, accel x/y/z per sensor)
- 3x3 button matrix (keys_mask, keys_rise, keys_fall)
- PRBS-15 trigger state (prbs_tick, in_mark)

Protocol: 115200 baud, 500 Hz CSV lines.
Header: t_ms,imu1_ok,imu2_ok,yaw1,pitch1,roll1,ax1,ay1,az1,yaw2,pitch2,roll2,ax2,ay2,az2,keys_mask,keys_rise,keys_fall,prbs_tick,in_mark

When an IMU has no data yet, the sketch sends empty fields for that IMU's 6 columns.
Startup lines (OK:, ERR:, STM32F401:...) and the header line are skipped.

Usage:
  python all_STM32_acquisition.py [--port PORT] [--baud BAUD] [--csv FILE] [--duration SEC] [--quiet]
"""

import time
import sys
import csv
import argparse
import math
from pathlib import Path
from dataclasses import dataclass
from typing import Optional

# Allow running from anywhere while importing project src
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import serial
from src.arduino_connection import find_arduino_port, open_arduino_serial, list_all_ports

# ============================================================================
# CLI
# ============================================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="General STM32 acquisition: dual IMU, matrix, PRBS (500 Hz CSV)"
    )
    parser.add_argument("--port", type=str, default="auto",
                        help="Serial port (default: auto)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--csv", type=str, default=None,
                        help="CSV file path for logging (optional)")
    parser.add_argument("--duration", type=float, default=None,
                        help="Run for N seconds then exit (optional)")
    parser.add_argument("--quiet", action="store_true",
                        help="Reduce console output")
    return parser.parse_args()

args = parse_args()
PORT = None if (args.port or "auto").lower() == "auto" else args.port
BAUD = args.baud
CSV_FILE = args.csv
DURATION_S = args.duration
QUIET = args.quiet

STALL_TIMEOUT_S = 3.0
RECONNECT_BACKOFF_S = 1.0
STATS_INTERVAL_S = 2.0
CSV_FLUSH_EVERY = 100

# ============================================================================
# Data model and parser
# ============================================================================

HEADER_NAMES = [
    "t_ms", "imu1_ok", "imu2_ok",
    "yaw1", "pitch1", "roll1", "ax1", "ay1", "az1",
    "yaw2", "pitch2", "roll2", "ax2", "ay2", "az2",
    "keys_mask", "keys_rise", "keys_fall", "prbs_tick", "in_mark",
]

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
    """
    Parse one STM32 CSV line (20 columns). Returns SampleSTM32 or None.
    Skips header, empty lines, and status lines (OK:, ERR:, STM32, #).
    Empty IMU fields become float('nan').
    """
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
# Serial and CSV
# ============================================================================

def open_serial():
    """Open serial connection (auto or specified port)."""
    if PORT is None:
        s = open_arduino_serial(
            port=None,
            baud=BAUD,
            timeout=0.25,
            wait_for_ready=True,
            ready_timeout=5.0,
            verbose=not QUIET,
        )
    else:
        s = open_arduino_serial(
            port=PORT,
            baud=BAUD,
            timeout=0.25,
            wait_for_ready=True,
            ready_timeout=5.0,
            verbose=not QUIET,
        )
    return s


def reopen_serial(old_ser):
    """Close and reopen serial connection."""
    try:
        old_ser.close()
    except Exception:
        pass
    time.sleep(RECONNECT_BACKOFF_S)
    return open_serial()


class CSVLogger:
    """Write parsed STM32 samples to CSV with same 20 columns as sketch."""
    def __init__(self, filepath: Path):
        self.filepath = Path(filepath)
        self.filepath.parent.mkdir(parents=True, exist_ok=True)
        self.file_handle = open(self.filepath, "w", newline="")
        self.writer = csv.writer(self.file_handle)
        self.writer.writerow(HEADER_NAMES)
        self.count = 0
        if not QUIET:
            print(f"CSV logging to: {self.filepath}")

    def log(self, s: SampleSTM32):
        self.writer.writerow([
            s.t_ms, s.imu1_ok, s.imu2_ok,
            s.yaw1, s.pitch1, s.roll1, s.ax1, s.ay1, s.az1,
            s.yaw2, s.pitch2, s.roll2, s.ax2, s.ay2, s.az2,
            s.keys_mask, s.keys_rise, s.keys_fall, s.prbs_tick, s.in_mark,
        ])
        self.count += 1
        if self.count % CSV_FLUSH_EVERY == 0:
            self.file_handle.flush()

    def close(self):
        if self.file_handle:
            self.file_handle.flush()
            self.file_handle.close()
            if not QUIET:
                print(f"CSV saved: {self.filepath} ({self.count} samples)")
            self.file_handle = None


# ============================================================================
# Main
# ============================================================================

def main():
    csv_logger = CSVLogger(CSV_FILE) if CSV_FILE else None
    ser = open_serial()
    last_rx_time = time.time()
    start_time = time.time()
    last_stats_time = start_time
    sample_count = 0

    if not QUIET:
        print("Waiting for first valid data line...")
    time.sleep(0.5)
    ser.reset_input_buffer()

    try:
        while True:
            # Duration limit
            if DURATION_S is not None and (time.time() - start_time) >= DURATION_S:
                if not QUIET:
                    print(f"\nDuration {DURATION_S}s reached. Exiting.")
                break

            # Stall reconnect
            if (time.time() - last_rx_time) > STALL_TIMEOUT_S:
                if not QUIET:
                    print(f"\nStall > {STALL_TIMEOUT_S}s. Reconnecting...")
                ser = reopen_serial(ser)
                last_rx_time = time.time()
                continue

            try:
                raw = ser.readline()
            except serial.SerialException as e:
                if not QUIET:
                    print(f"\nSerial error: {e}")
                ser = reopen_serial(ser)
                last_rx_time = time.time()
                continue

            if not raw:
                continue
            try:
                line = raw.decode("ascii", errors="ignore")
            except Exception:
                continue

            sample = parse_line_stm32(line)
            if sample is None:
                continue

            last_rx_time = time.time()
            sample_count += 1

            if csv_logger:
                csv_logger.log(sample)

            # Periodic stats
            if not QUIET and (time.time() - last_stats_time) >= STATS_INTERVAL_S:
                elapsed = time.time() - start_time
                rate = sample_count / elapsed if elapsed > 0 else 0
                print(f"  samples={sample_count}  rate={rate:.1f} Hz  t_ms={sample.t_ms:.0f}  prbs_tick={sample.prbs_tick}  in_mark={sample.in_mark}")
                last_stats_time = time.time()

    except KeyboardInterrupt:
        if not QUIET:
            print("\nInterrupted.")
    finally:
        if csv_logger:
            csv_logger.close()
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
