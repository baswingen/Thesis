"""
General STM32 acquisition
==========================

High-performance, thread-safe acquisition for the STM32F401 sketch
(STM32_all_in_python.ino).  Captures:

- Dual BNO085 UART-RVC IMU (yaw, pitch, roll, accel x/y/z per sensor)
- 3x3 button matrix (keys_mask, keys_rise, keys_fall)
- PRBS-15 trigger state (prbs_tick, prbs_level)

Architecture
------------
``STM32Reader`` runs a **daemon thread** that continuously drains the serial
port and pushes parsed samples into pre-allocated **NumPy ring buffers**.
The main thread (or any other thread) can snapshot the latest N samples in
O(1) via ``get_snapshot()``.

PRBS data is stored as a ±1 float stream (``prbs_signal``) ready for
cross-correlation with the ``SynchronizationEngine`` in ``src.prbs_sync``.

Protocol: 115200 baud, 500 Hz CSV, 21 columns.
Header: t_ms,imu1_ok,imu2_ok,yaw1,pitch1,roll1,ax1,ay1,az1,
        yaw2,pitch2,roll2,ax2,ay2,az2,
        keys_mask,keys_rise,keys_fall,prbs_tick,prbs_level

Usage (standalone):
  python all_STM32_acquisition.py [--port PORT] [--baud BAUD] [--csv FILE]
                                  [--duration SEC] [--quiet]

Usage (importable):
  from setup_scripts.all_STM32_acquisition import STM32Reader
  reader = STM32Reader(port=None, baud=115200)
  reader.start()
  ...
  snap = reader.get_snapshot(n=500)  # last 500 samples as dict of np arrays
  reader.stop()
"""

from __future__ import annotations

import csv
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional

import numpy as np

# Project root for src imports
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import serial  # noqa: E402
from src.arduino_connection import open_arduino_serial  # noqa: E402


# ============================================================================
# Constants
# ============================================================================

HEADER_NAMES: List[str] = [
    "t_ms", "imu1_ok", "imu2_ok",
    "yaw1", "pitch1", "roll1", "ax1", "ay1", "az1",
    "yaw2", "pitch2", "roll2", "ax2", "ay2", "az2",
    "keys_mask", "keys_rise", "keys_fall",
    "prbs_tick", "prbs_level",
]

NUM_COLUMNS = len(HEADER_NAMES)  # 21

# Column indices (avoid magic numbers everywhere)
_COL = {name: idx for idx, name in enumerate(HEADER_NAMES)}

# Columns that are integer-typed
_INT_COLS = {"imu1_ok", "imu2_ok", "keys_mask", "keys_rise", "keys_fall",
             "prbs_tick", "prbs_level"}

# Default ring-buffer capacity (seconds * Hz)
DEFAULT_CAPACITY = 60 * 500  # 60 s at 500 Hz


# ============================================================================
# Data model
# ============================================================================

@dataclass(slots=True)
class SampleSTM32:
    """One parsed STM32 CSV row (21 columns). Missing IMU fields are NaN."""
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
    prbs_lvl: int


# ============================================================================
# Fast parser
# ============================================================================

_NAN = float("nan")


def _fast_float(s: str) -> float:
    """Convert stripped string to float; empty string -> NaN."""
    if not s:
        return _NAN
    try:
        return float(s)
    except ValueError:
        return _NAN


def _fast_int(s: str) -> int:
    """Convert stripped string to int; empty string -> 0."""
    if not s:
        return 0
    try:
        return int(s)
    except ValueError:
        return 0


def parse_line_stm32(line: str) -> Optional[SampleSTM32]:
    """Parse one CSV line (20 columns) -> SampleSTM32 or None."""
    line = line.strip()
    if not line:
        return None
    c = line[0]
    if c in ("O", "E", "S", "#", "t", "C"):
        # Skip OK:, ERR:, STM32..., #, t_ms,..., Columns:
        if (line.startswith("OK:") or line.startswith("ERR:")
                or line.startswith("STM32") or c == "#"
                or line.startswith("t_ms,") or line.startswith("Columns:")):
            return None
    parts = line.split(",")
    if len(parts) != NUM_COLUMNS:
        return None
    try:
        return SampleSTM32(
            t_ms=_fast_float(parts[0]),
            imu1_ok=_fast_int(parts[1]),
            imu2_ok=_fast_int(parts[2]),
            yaw1=_fast_float(parts[3]),
            pitch1=_fast_float(parts[4]),
            roll1=_fast_float(parts[5]),
            ax1=_fast_float(parts[6]),
            ay1=_fast_float(parts[7]),
            az1=_fast_float(parts[8]),
            yaw2=_fast_float(parts[9]),
            pitch2=_fast_float(parts[10]),
            roll2=_fast_float(parts[11]),
            ax2=_fast_float(parts[12]),
            ay2=_fast_float(parts[13]),
            az2=_fast_float(parts[14]),
            keys_mask=_fast_int(parts[15]),
            keys_rise=_fast_int(parts[16]),
            keys_fall=_fast_int(parts[17]),
            prbs_tick=_fast_int(parts[18]),
            prbs_lvl=_fast_int(parts[19]),
        )
    except (ValueError, IndexError):
        return None


# ============================================================================
# NumPy ring buffer
# ============================================================================

class RingBuffer:
    """
    Fixed-capacity, pre-allocated NumPy ring buffer.

    - ``push(value)`` is O(1) – no allocation.
    - ``get_last(n)`` returns the most-recent *n* values as a contiguous
      NumPy array (one copy, no Python-level loop).
    """

    __slots__ = ("_buf", "_cap", "_cursor", "_count")

    def __init__(self, capacity: int, dtype=np.float64):
        self._buf = np.full(capacity, np.nan, dtype=dtype)
        self._cap = capacity
        self._cursor = 0
        self._count = 0

    def push(self, value: float) -> None:
        self._buf[self._cursor] = value
        self._cursor = (self._cursor + 1) % self._cap
        if self._count < self._cap:
            self._count += 1

    @property
    def count(self) -> int:
        return self._count

    def get_last(self, n: int) -> np.ndarray:
        """Return the last *n* values (oldest-first). Clips to available count."""
        n = min(n, self._count)
        if n == 0:
            return np.empty(0, dtype=self._buf.dtype)
        start = (self._cursor - n) % self._cap
        if start + n <= self._cap:
            return self._buf[start:start + n].copy()
        # Wrap-around: two slices
        tail = self._buf[start:]
        head = self._buf[:n - len(tail)]
        return np.concatenate((tail, head))


# ============================================================================
# STM32Reader – threaded serial reader
# ============================================================================

class STM32Reader:
    """
    High-performance threaded STM32 serial reader.

    - Daemon thread reads serial at full speed (short timeout).
    - Parsed samples are pushed into per-column ``RingBuffer`` instances.
    - ``prbs_signal`` stores ``prbs_lvl`` as ±1 float for PRBS sync.
    - ``get_snapshot(n)`` returns last *n* samples as dict of np arrays.
    - Optional ``on_sample`` callback fired for every parsed sample.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        baud: int = 115200,
        capacity: int = DEFAULT_CAPACITY,
        on_sample: Optional[Callable[[SampleSTM32], None]] = None,
        verbose: bool = True,
    ):
        self.port = port
        self.baud = baud
        self.capacity = capacity
        self.on_sample = on_sample
        self.verbose = verbose

        # Ring buffers – one per column + prbs_signal (±1)
        self.buffers: Dict[str, RingBuffer] = {
            name: RingBuffer(capacity) for name in HEADER_NAMES
        }
        self.buffers["prbs_signal"] = RingBuffer(capacity)  # ±1 for sync engine
        self.buffers["t_sec"] = RingBuffer(capacity)          # t_ms / 1000
        self.buffers["pc_time"] = RingBuffer(capacity)        # time.perf_counter()

        # Latest sample (atomic via GIL for simple reads)
        self.latest: Optional[SampleSTM32] = None

        # Stats
        self.sample_count = 0
        self.parse_fail_count = 0
        self._start_pc: float = 0.0

        # Thread plumbing
        self._ser: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

    # ---- public API ----

    def start(self) -> None:
        """Open serial and start the reader thread."""
        self._stop_event.clear()
        self._ser = self._open_serial()
        self._start_pc = time.perf_counter()
        time.sleep(0.3)
        self._ser.reset_input_buffer()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True, name="STM32Reader")
        self._thread.start()
        if self.verbose:
            print("[STM32Reader] started")

    def stop(self) -> None:
        """Signal the reader thread to stop and close serial."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        if self.verbose:
            print(f"[STM32Reader] stopped  ({self.sample_count} samples, "
                  f"{self.parse_fail_count} parse fails)")

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def get_snapshot(self, n: int = 5000) -> Dict[str, np.ndarray]:
        """
        Return the last *n* samples across all ring buffers.
        Keys match ``HEADER_NAMES`` plus ``prbs_signal``, ``t_sec``, ``pc_time``.
        """
        with self._lock:
            return {name: rb.get_last(n) for name, rb in self.buffers.items()}

    def get_sample_rate(self) -> float:
        """Measured sample rate (Hz) since start."""
        elapsed = time.perf_counter() - self._start_pc
        return self.sample_count / elapsed if elapsed > 0.5 else 0.0

    # ---- internal ----

    def _open_serial(self) -> serial.Serial:
        return open_arduino_serial(
            port=self.port,
            baud=self.baud,
            timeout=0.02,       # 20 ms – short so the loop stays responsive
            wait_for_ready=True,
            ready_timeout=5.0,
            verbose=self.verbose,
        )

    def _reopen_serial(self) -> serial.Serial:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        time.sleep(1.0)
        return self._open_serial()

    def _reader_loop(self) -> None:
        """Background thread: read serial as fast as possible."""
        ser = self._ser
        assert ser is not None
        last_rx = time.perf_counter()

        while not self._stop_event.is_set():
            # Stall detection
            if time.perf_counter() - last_rx > 3.0:
                if self.verbose:
                    print("[STM32Reader] stall detected, reconnecting...")
                try:
                    ser = self._reopen_serial()
                    self._ser = ser
                except Exception as e:
                    if self.verbose:
                        print(f"[STM32Reader] reconnect failed: {e}")
                    time.sleep(1.0)
                    continue
                last_rx = time.perf_counter()
                continue

            try:
                raw = ser.readline()
            except serial.SerialException as e:
                if self.verbose:
                    print(f"[STM32Reader] serial error: {e}")
                try:
                    ser = self._reopen_serial()
                    self._ser = ser
                except Exception:
                    time.sleep(1.0)
                continue

            if not raw:
                continue

            try:
                line = raw.decode("ascii", errors="ignore")
            except Exception:
                continue

            sample = parse_line_stm32(line)
            if sample is None:
                self.parse_fail_count += 1
                continue

            pc_now = time.perf_counter()
            last_rx = pc_now
            self.sample_count += 1
            self.latest = sample

            # Push into ring buffers (lock-free for single-writer is safe,
            # but we lock briefly so get_snapshot sees a consistent cursor)
            with self._lock:
                b = self.buffers
                b["t_ms"].push(sample.t_ms)
                b["t_sec"].push(sample.t_ms / 1000.0)
                b["pc_time"].push(pc_now)
                b["imu1_ok"].push(float(sample.imu1_ok))
                b["imu2_ok"].push(float(sample.imu2_ok))
                b["yaw1"].push(sample.yaw1)
                b["pitch1"].push(sample.pitch1)
                b["roll1"].push(sample.roll1)
                b["ax1"].push(sample.ax1)
                b["ay1"].push(sample.ay1)
                b["az1"].push(sample.az1)
                b["yaw2"].push(sample.yaw2)
                b["pitch2"].push(sample.pitch2)
                b["roll2"].push(sample.roll2)
                b["ax2"].push(sample.ax2)
                b["ay2"].push(sample.ay2)
                b["az2"].push(sample.az2)
                b["keys_mask"].push(float(sample.keys_mask))
                b["keys_rise"].push(float(sample.keys_rise))
                b["keys_fall"].push(float(sample.keys_fall))
                b["prbs_tick"].push(float(sample.prbs_tick))
                b["prbs_level"].push(float(sample.prbs_lvl))
                # ±1 for PRBS synchronization engine
                b["prbs_signal"].push(1.0 if sample.prbs_lvl else -1.0)

            if self.on_sample is not None:
                try:
                    self.on_sample(sample)
                except Exception:
                    pass


# ============================================================================
# CSV logger
# ============================================================================

class CSVLogger:
    """Write parsed STM32 samples to CSV (20 columns, same as sketch)."""

    def __init__(self, filepath: str | Path, quiet: bool = False):
        self.filepath = Path(filepath)
        self.filepath.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.filepath, "w", newline="")
        self._writer = csv.writer(self._fh)
        self._writer.writerow(HEADER_NAMES)
        self.count = 0
        if not quiet:
            print(f"[CSVLogger] logging to {self.filepath}")

    def log(self, s: SampleSTM32) -> None:
        self._writer.writerow([
            s.t_ms, s.imu1_ok, s.imu2_ok,
            s.yaw1, s.pitch1, s.roll1, s.ax1, s.ay1, s.az1,
            s.yaw2, s.pitch2, s.roll2, s.ax2, s.ay2, s.az2,
            s.keys_mask, s.keys_rise, s.keys_fall,
            s.prbs_tick, s.prbs_lvl,
        ])
        self.count += 1
        if self.count % 100 == 0:
            self._fh.flush()

    def close(self) -> None:
        if self._fh:
            self._fh.flush()
            self._fh.close()
            self._fh = None
            print(f"[CSVLogger] saved {self.filepath} ({self.count} samples)")


# ============================================================================
# CLI main (only runs when executed directly)
# ============================================================================

def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(
        description="STM32 acquisition: dual IMU + matrix + PRBS (500 Hz)"
    )
    parser.add_argument("--port", type=str, default="auto")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--csv", type=str, default=None)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    port = None if (args.port or "auto").lower() == "auto" else args.port
    quiet = args.quiet

    csv_logger = CSVLogger(args.csv, quiet=quiet) if args.csv else None

    def _on_sample(s: SampleSTM32) -> None:
        if csv_logger:
            csv_logger.log(s)

    reader = STM32Reader(
        port=port, baud=args.baud,
        on_sample=_on_sample,
        verbose=not quiet,
    )
    reader.start()
    start = time.time()
    last_stats = start

    try:
        while True:
            time.sleep(0.5)
            now = time.time()
            if args.duration and (now - start) >= args.duration:
                if not quiet:
                    print(f"\nDuration {args.duration}s reached.")
                break
            if not quiet and (now - last_stats) >= 2.0:
                last_stats = now
                s = reader.latest
                rate = reader.get_sample_rate()
                if s:
                    print(f"  samples={reader.sample_count}  rate={rate:.0f} Hz  "
                          f"t_ms={s.t_ms:.0f}  keys={s.keys_mask}  "
                          f"prbs_tick={s.prbs_tick}  prbs_lvl={s.prbs_lvl}")
    except KeyboardInterrupt:
        if not quiet:
            print("\nInterrupted.")
    finally:
        reader.stop()
        if csv_logger:
            csv_logger.close()


if __name__ == "__main__":
    main()
