"""
STM32 serial reader (native module)
====================================

High-performance, thread-safe acquisition for the STM32F401 sketch
(STM32_all_in_python.ino). Captures:

- Dual BNO085 UART-RVC IMU (yaw, pitch, roll, accel x/y/z per sensor)
- 3x4 button matrix (keys_mask, keys_rise, keys_fall)
- PRBS-15 trigger state (prbs_tick, prbs_level)

Protocol: 921600 baud, 500 Hz Binary (67-byte packets) or CSV.
Header: t_ms,imu1_ok,imu2_ok,yaw1,pitch1,roll1,ax1,ay1,az1,
        yaw2,pitch2,roll2,ax2,ay2,az2,
        keys_mask,keys_rise,keys_fall,prbs_tick,prbs_level,in_mark

PRBS-15 runs continuously at 500 Hz without frame markers for optimal correlation.
Matches STM32 sample rate (1:1 ratio) for most reliable reconstruction.

Usage:
  from src.stm32_reader import STM32Reader
  reader = STM32Reader(port=None, baud=115200)
  reader.start()
  snap = reader.get_snapshot(n=500)
  reader.stop()
"""

from __future__ import annotations

import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional

import numpy as np

try:
    import serial
except ImportError:
    serial = None  # type: ignore

# Ensure parent and current directory are in path for direct execution
_SRC_ROOT = Path(__file__).resolve().parent
if str(_SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(_SRC_ROOT))

try:
    from .arduino_connection import open_arduino_serial
except (ImportError, ValueError):
    from arduino_connection import open_arduino_serial


# ============================================================================
# Constants
# ============================================================================

HEADER_NAMES: List[str] = [
    "t_ms", "imu1_ok", "imu2_ok",
    "yaw1", "pitch1", "roll1", "ax1", "ay1", "az1",
    "yaw2", "pitch2", "roll2", "ax2", "ay2", "az2",
    "keys_mask", "keys_rise", "keys_fall",
    "prbs_tick", "prbs_level", "in_mark",
]

NUM_COLUMNS = len(HEADER_NAMES)
DEFAULT_CAPACITY = 60 * 500  # 60 s at 500 Hz

# Binary Protocol Constants
BIN_SYNC1 = 0xAA
BIN_SYNC2 = 0x55
BIN_PACKET_SIZE = 67
# Format: sync1(B), sync2(B), t_ms(I), imu_ok(B), imu1(6f), imu2(6f), keys(3H), prbs_tick(I), prbs_bits(B), cs(B)
BIN_FORMAT = "<BB I B 6f 6f 3H I B B"


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
    in_mark: int


# ============================================================================
# Parser and ring buffer
# ============================================================================

_NAN = float("nan")


def _fast_float(s: str) -> float:
    if not s:
        return _NAN
    try:
        return float(s)
    except ValueError:
        return _NAN


def _fast_int(s: str) -> int:
    if not s:
        return 0
    try:
        return int(s)
    except ValueError:
        return 0


def parse_line_stm32(line: str) -> Optional[SampleSTM32]:
    """Parse one CSV line (21 columns) -> SampleSTM32 or None."""
    line = line.strip()
    if not line:
        return None
    c = line[0]
    if c in ("O", "E", "S", "#", "t", "C"):
        if (line.startswith("OK:") or line.startswith("ERR:")
                or line.startswith("STM32") or c == "#"
                or line.startswith("t_ms,") or line.startswith("Columns:")):
            return None
    parts = line.split(",")
    if len(parts) not in (NUM_COLUMNS, NUM_COLUMNS - 1):
        return None
    # Allow 20 columns (legacy firmware): in_mark defaults to 0
    in_mark_val = _fast_int(parts[20]) if len(parts) >= NUM_COLUMNS else 0
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
            in_mark=in_mark_val,
        )
    except (ValueError, IndexError):
        return None


class RingBuffer:
    """Fixed-capacity NumPy ring buffer. push() O(1), get_last(n) returns last n values."""

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
        n = min(n, self._count)
        if n == 0:
            return np.empty(0, dtype=self._buf.dtype)
        start = (self._cursor - n) % self._cap
        if start + n <= self._cap:
            return self._buf[start:start + n].copy()
        tail = self._buf[start:]
        head = self._buf[:n - len(tail)]
        return np.concatenate((tail, head))


# ============================================================================
# STM32Reader
# ============================================================================

class STM32Reader:
    """
    High-performance threaded STM32 serial reader.
    Daemon thread fills ring buffers; get_snapshot(n) returns last n samples.
    Optional on_sample callback for PRBS sync etc.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        baud: int = 921600,
        capacity: int = DEFAULT_CAPACITY,
        on_sample: Optional[Callable[[SampleSTM32], None]] = None,
        verbose: bool = True,
        binary_mode: bool = True,
    ):
        if serial is None:
            raise RuntimeError("pyserial is required for STM32Reader. Install with: pip install pyserial")
        self.port = port
        self.baud = baud
        self.capacity = capacity
        self.on_sample = on_sample
        self.verbose = verbose
        self.binary_mode = binary_mode

        self.buffers: Dict[str, RingBuffer] = {
            name: RingBuffer(capacity) for name in HEADER_NAMES
        }
        self.buffers["prbs_signal"] = RingBuffer(capacity)
        self.buffers["t_sec"] = RingBuffer(capacity)
        self.buffers["pc_time"] = RingBuffer(capacity)

        self.latest: Optional[SampleSTM32] = None
        self.sample_count = 0
        self.parse_fail_count = 0
        self._time_offset: Optional[float] = None
        self._last_raw_t_sec = 0.0
        self._start_pc: float = 0.0

        self._ser: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

    def start(self) -> None:
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
        with self._lock:
            return {name: rb.get_last(n) for name, rb in self.buffers.items()}

    def get_sample_rate(self) -> float:
        elapsed = time.perf_counter() - self._start_pc
        return self.sample_count / elapsed if elapsed > 0.5 else 0.0

    def _open_serial(self) -> serial.Serial:
        return open_arduino_serial(
            port=self.port,
            baud=self.baud,
            timeout=0.02,
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
        ser = self._ser
        assert ser is not None
        last_rx = time.perf_counter()

        # Buffer for binary mode
        bin_buf = bytearray()

        while not self._stop_event.is_set():
            if time.perf_counter() - last_rx > 3.0:
                if self.verbose:
                    print("[STM32Reader] stall detected, reconnecting...")
                try:
                    ser = self._reopen_serial()
                    self._ser = ser
                    bin_buf.clear()
                except Exception as e:
                    if self.verbose:
                        print(f"[STM32Reader] reconnect failed: {e}")
                    time.sleep(1.0)
                    continue
                last_rx = time.perf_counter()
                continue

            try:
                if self.binary_mode:
                    chunk = None
                    # Binary mode: read chunks and look for sync
                    if ser.in_waiting > 0:
                        chunk = ser.read(ser.in_waiting)
                        if chunk:
                            bin_buf.extend(chunk)
                            last_rx = time.perf_counter()
                    
                    # Process as many packets as possible
                    while len(bin_buf) >= BIN_PACKET_SIZE:
                        # Find sync
                        sync_pos = bin_buf.find(bytes([BIN_SYNC1, BIN_SYNC2]))
                        if sync_pos == -1:
                            # No sync found, clear buffer except possible start of sync
                            if bin_buf[-1] == BIN_SYNC1:
                                del bin_buf[:-1]
                            else:
                                bin_buf.clear()
                            break
                        
                        if sync_pos > 0:
                            # Skip garbage
                            del bin_buf[:sync_pos]
                        
                        if len(bin_buf) < BIN_PACKET_SIZE:
                            break
                        
                        # Peek at packet
                        packet_data = bin_buf[:BIN_PACKET_SIZE]
                        
                        # Validate checksum (XOR of bytes 2 to size-2)
                        cs = 0
                        for b in packet_data[2:-1]:
                            cs ^= b
                        
                        if cs != packet_data[-1]:
                            # Checksum failed, skip the sync and continue
                            del bin_buf[:2]
                            self.parse_fail_count += 1
                            continue
                        
                        # Valid packet! Parse it.
                        try:
                            sample = self._parse_binary_packet(packet_data)
                            if sample:
                                self._push_sample(sample)
                        except Exception:
                            self.parse_fail_count += 1
                        
                        # Remove packet from buffer
                        del bin_buf[:BIN_PACKET_SIZE]
                    
                    if not chunk: # if no data read, sleep a bit
                        time.sleep(0.001)
                    continue

                else:
                    # Legacy CSV mode
                    raw = ser.readline()
                    if not raw:
                        continue
                    try:
                        line = raw.decode("ascii", errors="ignore")
                        sample = parse_line_stm32(line)
                        if sample:
                            self._push_sample(sample)
                        else:
                            self.parse_fail_count += 1
                    except Exception:
                        self.parse_fail_count += 1
                        continue
                    last_rx = time.perf_counter()

            except serial.SerialException as e:
                if self.verbose:
                    print(f"[STM32Reader] serial error: {e}")
                try:
                    ser = self._reopen_serial()
                    self._ser = ser
                    bin_buf.clear()
                except Exception:
                    time.sleep(1.0)
                continue

    def _parse_binary_packet(self, data: bytes) -> Optional[SampleSTM32]:
        """Unpack 67-byte binary packet into SampleSTM32."""
        try:
            # Format: <BB I B 6f 6f 3H I B B
            vals = struct.unpack(BIN_FORMAT, data)
            # 0:s1, 1:s2, 2:t_ms, 3:imu_ok, 4-9:imu1, 10-15:imu2, 16:mask, 17:rise, 18:fall, 19:tick, 20:bits, 21:cs
            
            return SampleSTM32(
                t_ms=float(vals[2]),
                imu1_ok=(vals[3] & 1),
                imu2_ok=((vals[3] >> 1) & 1),
                yaw1=vals[4], pitch1=vals[5], roll1=vals[6], ax1=vals[7], ay1=vals[8], az1=vals[9],
                yaw2=vals[10], pitch2=vals[11], roll2=vals[12], ax2=vals[13], ay2=vals[14], az2=vals[15],
                keys_mask=int(vals[16]),
                keys_rise=int(vals[17]),
                keys_fall=int(vals[18]),
                prbs_tick=int(vals[19]),
                prbs_lvl=(vals[20] & 1),
                in_mark=((vals[20] >> 1) & 1)
            )
        except Exception:
            return None

    def _push_sample(self, sample: SampleSTM32) -> None:
        """Helper to push a parsed sample into buffers."""
        pc_now = time.perf_counter()
        t_sec = sample.t_ms / 1000.0
        
        if self._time_offset is None:
            self._time_offset = pc_now - t_sec

        pc_time = t_sec + self._time_offset
        
        self.sample_count += 1
        self.latest = sample

        with self._lock:
            b = self.buffers
            b["t_ms"].push(sample.t_ms)
            b["t_sec"].push(t_sec)
            b["pc_time"].push(pc_time)
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
            b["in_mark"].push(float(sample.in_mark))
            b["prbs_signal"].push(1.0 if sample.prbs_lvl else -1.0)

        if self.on_sample is not None:
            try:
                self.on_sample(sample)
            except Exception:
                pass
