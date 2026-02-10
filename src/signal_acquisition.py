"""
Synchronized STM32 + EMG signal acquisition (native module)
==========================================================

Single, program-wide signal acquisition for:
- STM32F401: dual BNO085 IMU + button matrix + PRBS TRIG output (500 Hz)
- TMSi Porti7: multi-channel EMG + TRIG input (~2000 Hz)
- PRBS-15 hardware sync: STM32 PA8 → Porti7 TRIG; cross-correlation for clock offset.

Use SignalAcquisition for one-shot start/stop and access to STM32 snapshots,
EMG buffer, and sync state. All components are also importable for testing
(STM32 only, EMG only, PRBS test).

Usage:
  from src.signal_acquisition import SignalAcquisition, SignalAcquisitionConfig

  config = SignalAcquisitionConfig()
  acq = SignalAcquisition(config)
  acq.start()
  snap = acq.get_stm32_snapshot(500)
  emg_list = acq.get_emg_recent(5.0)
  sync = acq.get_sync_state()
  acq.stop()

STM32 sketch: arduino/STM32_all_copy_20260209152029/STM32_all_in_python/STM32_all_in_python.ino
TMSi SDK: tmsi-python-interface/
"""

from __future__ import annotations

import argparse
import queue
import sys
import time
import threading
import traceback
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# Project root (parent of src/) for TMSi SDK path
_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_TMSI_PATH = _PROJECT_ROOT / "tmsi-python-interface"
if _TMSI_PATH.exists() and str(_TMSI_PATH) not in sys.path:
    sys.path.insert(0, str(_TMSI_PATH))

try:
    from TMSiSDK.tmsi_sdk import TMSiSDK
    from TMSiSDK.device.tmsi_device_enums import (
        DeviceType, DeviceInterfaceType, MeasurementType,
    )
    from TMSiSDK.device.devices.legacy.legacy_device import LegacyDevice
    _TMSI_AVAILABLE = True
except ImportError as exc:
    _TMSI_AVAILABLE = False
    LegacyDevice = None  # type: ignore
    print(f"[WARN] TMSi SDK not available ({exc}) — EMG acquisition disabled")

try:
    from scipy.signal import correlate as sp_correlate
    _USE_SCIPY = True
except Exception:
    _USE_SCIPY = False

from .stm32_reader import STM32Reader, SampleSTM32

CLOCK = time.perf_counter


# =============================================================================
# PRBS-15 RECONSTRUCTION (must match Arduino sketch)
# =============================================================================

# STM32 sketch: CHIP_RATE_HZ=2000, FRAME_HZ=1, MARK_MS=30, PRBS15 seed 0x7ACE
PRBS_CHIP_RATE_HZ = 2000
PRBS_CHIPS_PER_FRAME = 2000
PRBS_MARK_CHIPS = 61  # Arduino ISR outputs 61 LOW ticks per mark (not 60):
                       # mark_ticks=60 on entry, counts down to 0, and the tick
                       # where mark_ticks==0 still outputs LOW before clearing
                       # in_mark.  Total mark duration = 61 chips = 30.5 ms.
PRBS15_SEED = 0x7ACE
PRBS15_PERIOD = (1 << 15) - 1  # 32_767


def _build_prbs15_lookup() -> np.ndarray:
    """Precompute full PRBS-15 period for O(1) lookup (avoids stall on large backfills)."""
    lfsr = PRBS15_SEED & 0x7FFF
    seq = np.empty(PRBS15_PERIOD, dtype=np.uint8)
    for i in range(PRBS15_PERIOD):
        b14 = (lfsr >> 14) & 1
        b13 = (lfsr >> 13) & 1
        newbit = b14 ^ b13
        seq[i] = newbit
        lfsr = ((lfsr << 1) | newbit) & 0x7FFF
    return seq


_PRBS15_LOOKUP = _build_prbs15_lookup()


def _prbs15_bit_at_index(lfsr_idx: int) -> int:
    """Return PRBS-15 bit at position lfsr_idx (O(1) via precomputed lookup)."""
    return int(_PRBS15_LOOKUP[lfsr_idx % PRBS15_PERIOD])


def reconstruct_prbs_chip_value(chip_idx: int) -> float:
    """
    Reconstruct PRBS output at global chip index (0-based).
    Returns ±1.0 (bipolar) for correlation with EMG TRIG.
    """
    frame = chip_idx // PRBS_CHIPS_PER_FRAME
    pos_in_frame = chip_idx % PRBS_CHIPS_PER_FRAME
    if pos_in_frame < PRBS_MARK_CHIPS:
        return -1.0  # LOW during marker gap (bipolar: -1)
    lfsr_idx = frame * (PRBS_CHIPS_PER_FRAME - PRBS_MARK_CHIPS) + (pos_in_frame - PRBS_MARK_CHIPS)
    bit = _prbs15_bit_at_index(lfsr_idx)
    return 2.0 * bit - 1.0  # 0 -> -1, 1 -> +1


# =============================================================================
# CONFIGURATION
# =============================================================================

@dataclass
class SignalAcquisitionConfig:
    """Configuration for STM32 + EMG + PRBS sync. All fields have defaults."""
    enable_stm32: bool = True
    enable_emg: bool = True
    stm32_port: Optional[str] = None
    stm32_baud: int = 115200
    stm32_capacity: int = 60 * 500
    emg_differential_pairs: List[Tuple[int, int]] = field(
        default_factory=lambda: [(0, 1)]
    )
    emg_connection_type: str = "usb"
    emg_sample_rate: Optional[int] = 2000
    emg_poll_sleep_s: float = 0.002
    enable_prbs_sync: bool = True
    prbs_correlation_window_s: float = 5.0
    prbs_update_interval_s: float = 5.0
    # Resample rate used for PRBS cross-correlation. Using the chip-rate avoids
    # aliasing a 2000 Hz NRZ chipstream down to 1 kHz.
    prbs_resample_rate_hz: float = 2000.0
    prbs_drift_warning_ms: float = 10.0
    prbs_drift_error_ms: float = 50.0
    max_emg_buffer_size: int = 200_000
    max_get_recent_items: int = 2000
    verbose: bool = True


# =============================================================================
# TMSi HELPERS
# =============================================================================

def fix_channel_name(index: int) -> str:
    """Return the correct Porti7 channel name for a given 0-based index."""
    if index < 16:
        return f"UNI{index + 1}"
    elif index < 36:
        return f"BIP{index - 15}"
    elif index == 36:
        return "STATUS"
    elif index == 37:
        return "COUNTER"
    return f"CH{index}"


def fix_all_channel_names(channels) -> int:
    """Fix corrupted UTF-16LE names on TMSi legacy channels. Returns count fixed."""
    fixed = 0
    for i, ch in enumerate(channels):
        try:
            name = ch.get_channel_name()
            if any(ord(c) > 127 for c in name):
                corrected = fix_channel_name(i)
                ch._alt_name = corrected
                ch._def_name = corrected
                fixed += 1
        except Exception:
            pass
    return fixed


def find_trig_channel(channels) -> Tuple[Optional[int], bool]:
    """Locate TRIG/digital input channel. Returns (channel_index, is_status_bit)."""
    for i, ch in enumerate(channels):
        name = ch.get_channel_name().upper()
        if "TRIG" in name:
            return i, False
    for i, ch in enumerate(channels):
        name = ch.get_channel_name().upper()
        if name.startswith("DIG"):
            return i, True
    for i, ch in enumerate(channels):
        name = ch.get_channel_name().upper()
        if "STATUS" in name or "STATU" in name:
            return i, True
    if len(channels) > 36:
        return 36, True
    return None, None


def find_counter_channel(channels):
    """Locate COUNTER channel. Returns (index, gain, offset)."""
    for i, ch in enumerate(channels):
        name = ch.get_channel_name().upper()
        if "COUNTER" in name or i == 37:
            gain, offset = None, None
            try:
                gain = ch.get_channel_unit_gain()
            except Exception:
                pass
            try:
                offset = ch.get_channel_unit_offset()
            except Exception:
                pass
            return i, gain, offset
    return None, None, None


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class EMGData:
    """One processed EMG chunk (per-sample data for waveform display)."""
    timestamp: float
    timestamps: np.ndarray
    pairs: np.ndarray
    sample_count: int
    counter: Optional[int] = None


@dataclass
class SyncState:
    """Snapshot of PRBS synchronisation state."""
    timestamp: float
    offset_ms: float
    drift_rate_ppm: float
    correlation_peak: float
    confidence: float
    samples_analyzed: int


class TimestampedBuffer:
    """Thread-safe deque of (timestamp, data) tuples."""

    def __init__(self, maxlen: int = 10_000, max_get_recent: int = 2000):
        self.buffer: deque = deque(maxlen=maxlen)
        self.lock = threading.Lock()
        self.total_count = 0
        self._max_get_recent = max_get_recent

    def append(self, timestamp: float, data: Any):
        with self.lock:
            self.buffer.append((timestamp, data))
            self.total_count += 1

    def get_recent(self, window_seconds: float, now: Optional[float] = None) -> List[Tuple[float, Any]]:
        with self.lock:
            if not self.buffer:
                return []
            t_cutoff = (now if now is not None else CLOCK()) - window_seconds
            result = []
            for t, d in reversed(self.buffer):
                if t >= t_cutoff:
                    result.append((t, d))
                    if len(result) >= self._max_get_recent:
                        break
                else:
                    break
            result.reverse()
            return result

    def get_all(self) -> List[Tuple[float, Any]]:
        with self.lock:
            return list(self.buffer)

    def clear(self):
        with self.lock:
            self.buffer.clear()

    def __len__(self) -> int:
        with self.lock:
            return len(self.buffer)


# =============================================================================
# EMG ACQUISITION THREAD
# =============================================================================

class EMGAcquisitionThread(threading.Thread):
    """Reads TMSi Porti7 EMG + TRIG in a background thread. Uses TMSi SDK directly."""

    _INTERFACE_MAP = {
        "usb":       DeviceInterfaceType.usb,
        "bluetooth": DeviceInterfaceType.bluetooth,
        "network":   DeviceInterfaceType.network,
        "wifi":      DeviceInterfaceType.wifi,
    } if _TMSI_AVAILABLE else {}

    def __init__(
        self,
        emg_buffer: TimestampedBuffer,
        sync_engine: Optional["HardwarePRBSSync"],
        connection_type: str = "usb",
        emg_differential_pairs: Optional[List[Tuple[int, int]]] = None,
        sample_rate: Optional[int] = 2000,
        poll_sleep_s: float = 0.002,
        verbose: bool = True,
    ):
        super().__init__(name="EMG-Thread", daemon=True)
        self.emg_buffer = emg_buffer
        self.sync_engine = sync_engine
        self.connection_type = connection_type
        self.emg_differential_pairs = emg_differential_pairs or [(0, 1)]
        self.sample_rate_hz = sample_rate
        self.poll_sleep_s = poll_sleep_s
        self.verbose = verbose

        self.ready_event = threading.Event()
        self.running = False
        self.error = None

        self._sdk = None
        self._device = None
        self._measurement = None

        self.sample_count = 0
        self.chunk_count = 0
        self.sample_rate: float = 0.0
        self.start_time: Optional[float] = None

        self.counter_idx: Optional[int] = None
        self.counter_gain = None
        self.counter_offset = None
        self.use_counter_timing = False
        self.last_counter_raw: Optional[int] = None
        self.counter_unwrap_offset = 0
        self.counter_time_offset: Optional[float] = None
        self.first_counter_unwrapped: Optional[int] = None
        self.trig_idx: Optional[int] = None
        self.trig_is_status_bit = False

    def _counter_to_int(self, x: float) -> Optional[int]:
        try:
            if x is None or not np.isfinite(x):
                return None
            val = float(x)
            if self.counter_gain is not None and self.counter_offset is not None:
                g, o = float(self.counter_gain), float(self.counter_offset)
                if np.isfinite(g) and abs(g) > 0:
                    val = (val / g) - o
            return int(round(val))
        except Exception:
            return None

    def _unwrap_counter(self, raw: int) -> int:
        if self.last_counter_raw is None:
            self.last_counter_raw = raw
            return raw
        if raw < self.last_counter_raw and (self.last_counter_raw - raw) > 1000:
            self.counter_unwrap_offset += 65536
        self.last_counter_raw = raw
        return raw + self.counter_unwrap_offset

    def _extract_trig_bipolar(self, raw: np.ndarray) -> np.ndarray:
        raw_int = raw.astype(np.int64)
        candidates = [(raw_int & 1).astype(np.float64)]
        lo, hi = np.nanmin(raw), np.nanmax(raw)
        if hi - lo > 1e-9:
            threshold = (lo + hi) / 2.0
            candidates.append((raw > threshold).astype(np.float64))
        candidates.append((raw_int != 0).astype(np.float64))
        best = candidates[0]
        best_trans = int(np.sum(np.abs(np.diff(best))))
        for c in candidates[1:]:
            t = int(np.sum(np.abs(np.diff(c))))
            if t > best_trans:
                best_trans = t
                best = c
        return 2.0 * best - 1.0

    def run(self):
        if not _TMSI_AVAILABLE:
            self.error = RuntimeError("TMSi SDK not available")
            return
        try:
            if self.verbose:
                print("\n[EMG] Initialising TMSi SDK...")
            self._sdk = TMSiSDK()
            interface = self._INTERFACE_MAP.get(
                self.connection_type.lower(), DeviceInterfaceType.usb)

            if self.verbose:
                print(f"[EMG] Scanning for TMSi legacy devices ({self.connection_type})...")
            devices, _ = self._sdk.discover(DeviceType.legacy, dr_interface=interface)

            if not devices:
                raise RuntimeError("No TMSi legacy devices found. Check power / cable / drivers.")

            self._device = devices[0]
            self._device.open()

            dev_name = self._device.get_device_name()
            base_rate = self._device.get_device_base_sample_rate()
            if self.verbose:
                print(f"[EMG] Connected: {dev_name} (base rate {base_rate} Hz)")

            channels = self._device.get_device_channels()
            n_fixed = fix_all_channel_names(channels)
            if n_fixed and self.verbose:
                print(f"[EMG] Fixed {n_fixed} channel names (encoding workaround)")

            n_ch = len(channels)
            for pos, neg in self.emg_differential_pairs:
                if pos >= n_ch or neg >= n_ch:
                    raise ValueError(f"Pair ({pos},{neg}) out of range (0-{n_ch-1})")

            self.counter_idx, self.counter_gain, self.counter_offset = find_counter_channel(channels)
            if self.counter_idx is not None:
                self.use_counter_timing = True
                if self.verbose:
                    print(f"[EMG] COUNTER channel at index {self.counter_idx}")
            else:
                if self.verbose:
                    print("[EMG] COUNTER not found -- using receive-time timestamps")

            self.trig_idx, self.trig_is_status_bit = find_trig_channel(channels)
            if self.trig_idx is not None and self.verbose:
                mode = "bit-field" if self.trig_is_status_bit else "direct"
                print(f"[EMG] TRIG channel at index {self.trig_idx} ({mode})")
            elif self.trig_idx is None and self.verbose:
                print("[EMG] TRIG channel not found -- PRBS sync disabled")

            self._measurement = MeasurementType.LEGACY_SIGNAL(self._device)
            if self.sample_rate_hz is not None:
                self._measurement.set_sample_rate(self.sample_rate_hz)
            self._measurement.set_reference_calculation(False)
            self.sample_rate = self._measurement.get_device_sample_rate()
            if self.verbose:
                print(f"[EMG] Measurement configured: {self.sample_rate} Hz, ref_calc=off")

            self.ready_event.set()
            if self.verbose:
                print("[EMG] Ready.")

            self._measurement.start()
            self.start_time = CLOCK()
            self.running = True
            sample_period = 1.0 / self.sample_rate
            if self.verbose:
                print(f"[EMG] Acquisition started ({self.sample_rate} Hz)")

            while self.running:
                samples = self._measurement.get_samples(blocking=False)
                if samples is None or len(samples) == 0:
                    time.sleep(self.poll_sleep_s)
                    continue

                receive_time = CLOCK()
                n_s = samples.shape[0]

                counter_unwrapped = None
                if self.use_counter_timing and self.counter_idx is not None:
                    raw_c = self._counter_to_int(samples[-1, self.counter_idx])
                    if raw_c is not None:
                        counter_unwrapped = self._unwrap_counter(raw_c)
                        if self.first_counter_unwrapped is None:
                            self.first_counter_unwrapped = counter_unwrapped
                            self.counter_time_offset = receive_time - (counter_unwrapped * sample_period)
                            if self.verbose:
                                print(f"[EMG] Counter baseline: raw={raw_c}, offset={self.counter_time_offset:.6f} s")

                if (self.use_counter_timing and counter_unwrapped is not None
                        and self.counter_time_offset is not None):
                    last_ts = counter_unwrapped * sample_period + self.counter_time_offset
                    chunk_ts = last_ts - ((n_s - 1) / 2.0) * sample_period
                    new_off = receive_time - counter_unwrapped * sample_period
                    if np.isfinite(new_off):
                        self.counter_time_offset = 0.995 * self.counter_time_offset + 0.005 * new_off
                else:
                    last_ts = receive_time
                    chunk_ts = receive_time - n_s * sample_period / 2.0

                self.sample_count += n_s
                self.chunk_count += 1

                ts_arr = np.linspace(last_ts - (n_s - 1) * sample_period, last_ts, n_s)

                pair_cols = []
                for pos, neg in self.emg_differential_pairs:
                    pair_cols.append(samples[:, pos] - samples[:, neg])
                pair_data = np.column_stack(pair_cols) if pair_cols else np.empty((n_s, 0))

                emg_data = EMGData(
                    timestamp=chunk_ts,
                    timestamps=ts_arr,
                    pairs=pair_data,
                    sample_count=n_s,
                    counter=self.last_counter_raw,
                )
                self.emg_buffer.append(chunk_ts, emg_data)

                if self.trig_idx is not None and self.sync_engine is not None:
                    trig_raw = samples[:, self.trig_idx]
                    if self.trig_is_status_bit:
                        # Provide raw bit-field words; sync engine will pick best bit by PRBS correlation.
                        self.sync_engine.add_emg_trig_word_batch(ts_arr, trig_raw)
                    else:
                        trig_bp = self._extract_trig_bipolar(trig_raw)
                        self.sync_engine.add_emg_trig_batch(ts_arr, trig_bp)

                if self.verbose and self.chunk_count % 100 == 0:
                    lat = receive_time - chunk_ts
                    print(f"[EMG] chunks={self.chunk_count}  samples={self.sample_count}  lat={lat*1000:.1f} ms")

        except Exception as e:
            self.error = e
            print(f"\n[EMG] Error: {e}")
            traceback.print_exc()
        finally:
            if self.verbose:
                print("[EMG] Shutting down...")
            if self._measurement is not None:
                try:
                    self._measurement.stop()
                except Exception:
                    pass
            if self._device is not None:
                try:
                    self._device.close()
                except Exception:
                    pass
            try:
                if LegacyDevice is not None:
                    LegacyDevice.cleanup()
            except Exception:
                pass

    def stop(self):
        self.running = False

    def get_stats(self) -> Dict[str, Any]:
        elapsed = (CLOCK() - self.start_time) if self.start_time else 0
        rate = self.sample_count / elapsed if elapsed > 0 else 0
        return dict(samples=self.sample_count, chunks=self.chunk_count,
                    elapsed=elapsed, rate=rate, error=self.error)


# =============================================================================
# HARDWARE PRBS SYNC
# =============================================================================

class HardwarePRBSSync:
    """
    Cross-correlate reconstructed PRBS-15 (from STM32 prbs_tick/prbs_mark) with Porti7 TRIG.
    Reconstructs the full 2000 Hz PRBS on the host from deterministic (t_ms, prbs_tick, prbs_mark).
    """

    CHIP_PERIOD_S = 1.0 / PRBS_CHIP_RATE_HZ  # 0.0005 s

    def __init__(self, *,
                 correlation_window_s: float = 5.0,
                 update_interval_s: float = 5.0,
                 resample_rate_hz: float = 1000.0,
                 drift_warning_ms: float = 10.0,
                 drift_error_ms: float = 50.0):
        self.correlation_window_s = correlation_window_s
        self.update_interval_s = update_interval_s
        self.resample_dt = 1.0 / resample_rate_hz
        self.drift_warning_ms = drift_warning_ms
        self.drift_error_ms = drift_error_ms

        self.stm32_buf: deque = deque(maxlen=100_000)
        # EMG TRIG can be provided as either a ready-made bipolar stream (preferred for direct TRIG)
        # or as a raw bit-field word (Porti7 "Dig"/STATUS-like channels) for per-bit selection.
        self.emg_buf: deque = deque(maxlen=500_000)        # (t, trig_bipolar)
        self.emg_word_buf: deque = deque(maxlen=500_000)   # (t, raw_word_int)

        # Selected EMG TRIG extraction (learned by correlation against reconstructed PRBS).
        self._trig_mode: Optional[str] = None   # "bit", "threshold", "nonzero"
        self._trig_bit: Optional[int] = None    # 0..15 when mode == "bit"

        self.kf_offset = 0.0
        self.kf_variance = 100.0
        self._kf_Q = 0.01
        self._kf_R = 1.0

        self.current_offset_ms = 0.0
        self.drift_rate_ppm = 0.0
        self.last_update_time = 0.0
        self.update_count = 0
        self.sync_state: Optional[SyncState] = None
        self._update_in_progress = False

        self._lock = threading.Lock()
        self._stm32_queue: queue.Queue = queue.Queue(maxsize=60_000)
        self._drain_stop = threading.Event()
        self._last_prbs_tick: Optional[int] = None
        self._last_stm32_t_ms: Optional[float] = None
        # Map STM32 internal time (t_ms) into the host perf_counter domain.
        # We intentionally set this ONCE (first valid sample) so STM32 clock drift
        # relative to the host/EMG timebase remains measurable by correlation.
        self._stm32_time_offset_s: Optional[float] = None
        self._drain_thread = threading.Thread(
            target=self._drain_stm32_queue,
            name="PRBS-drain-STM32",
            daemon=True,
        )
        self._drain_thread.start()

    def _drain_stm32_queue(self) -> None:
        while not self._drain_stop.is_set():
            try:
                item = self._stm32_queue.get(timeout=0.05)
            except queue.Empty:
                continue
            receive_time, t_ms, prbs_tick, prbs_mark = item
            # Detect STM32 restarts / wraparound in t_ms; reset mapping if so.
            if self._last_stm32_t_ms is not None:
                # millis() wrap is ~49 days; in practice this catches device reset/reconnect.
                if float(t_ms) + 1000.0 < float(self._last_stm32_t_ms):
                    self._last_prbs_tick = None
                    self._stm32_time_offset_s = None
            self._last_stm32_t_ms = float(t_ms)

            # Convert STM32 internal time to host perf_counter domain using a fixed offset.
            # This makes the reconstructed PRBS run at the STM32 clock rate (incl. drift),
            # instead of assuming an ideal 2000 Hz in host time.
            stm32_now_s = float(t_ms) / 1000.0
            if self._stm32_time_offset_s is None:
                self._stm32_time_offset_s = float(receive_time) - stm32_now_s
            mapped_now_s = stm32_now_s + float(self._stm32_time_offset_s)

            last = self._last_prbs_tick
            self._last_prbs_tick = prbs_tick
            chip_start = 0 if last is None else last
            chip_end = prbs_tick  # chips 0..prbs_tick-1 have been output
            # Handle reset/wraparound: prbs_tick decreased
            if chip_end <= chip_start:
                continue
            # Limit backfill on first sample to avoid huge burst
            if last is None and prbs_tick > 50_000:
                chip_start = prbs_tick - 50_000
            with self._lock:
                for c in range(chip_start, chip_end):
                    t_chip = mapped_now_s - (prbs_tick - 1 - c) * self.CHIP_PERIOD_S
                    val = reconstruct_prbs_chip_value(c)
                    self.stm32_buf.append((t_chip, val))

    def add_stm32_prbs_sample(self, receive_time: float, t_ms: float,
                              prbs_tick: int, prbs_mark: int) -> None:
        """Feed (receive_time, t_ms, prbs_tick, prbs_mark) for PRBS reconstruction."""
        try:
            self._stm32_queue.put_nowait((receive_time, t_ms, prbs_tick, prbs_mark))
        except queue.Full:
            pass

    def add_stm32_prbs(self, timestamp: float, value: float) -> None:
        """Legacy: raw (t, v) pairs; prefer add_stm32_prbs_sample for reconstruction."""
        with self._lock:
            self.stm32_buf.append((timestamp, value))

    def add_stm32_prbs_batch(self, timestamps: np.ndarray, values: np.ndarray) -> None:
        with self._lock:
            for t, v in zip(timestamps, values):
                self.stm32_buf.append((float(t), float(v)))

    def add_emg_trig_batch(self, timestamps: np.ndarray, values: np.ndarray):
        with self._lock:
            self.emg_buf.extend(
                (float(t), float(v)) for t, v in zip(timestamps.flat, values.flat)
            )

    def add_emg_trig_word_batch(self, timestamps: np.ndarray, raw_values: np.ndarray) -> None:
        """Feed raw bit-field samples (e.g. Porti7 'Dig' channel). Values are stored as int."""
        # Ensure a 1D view
        ts = timestamps.astype(np.float64).ravel()
        rv = raw_values.ravel()
        with self._lock:
            for t, x in zip(ts, rv):
                try:
                    self.emg_word_buf.append((float(t), int(x)))
                except Exception:
                    # Ignore non-finite / non-castable values
                    pass

    def _extract_emg_trig_bipolar_from_words(self, raw_words: np.ndarray) -> np.ndarray:
        """
        Convert raw bit-field words to a bipolar (-1/+1) TRIG stream using the currently selected mode.
        If no mode has been selected yet, uses a conservative default (threshold).
        """
        if raw_words.size == 0:
            return raw_words.astype(np.float64)
        if self._trig_mode == "bit" and self._trig_bit is not None:
            b = ((raw_words.astype(np.int64) >> int(self._trig_bit)) & 1).astype(np.float64)
            return 2.0 * b - 1.0
        if self._trig_mode == "nonzero":
            b = (raw_words.astype(np.int64) != 0).astype(np.float64)
            return 2.0 * b - 1.0
        # default / "threshold"
        rw = raw_words.astype(np.float64)
        lo, hi = np.nanmin(rw), np.nanmax(rw)
        thr = (lo + hi) / 2.0 if np.isfinite(lo) and np.isfinite(hi) else 0.0
        b = (rw > thr).astype(np.float64)
        return 2.0 * b - 1.0

    def should_update(self) -> bool:
        if self._update_in_progress:
            return False
        if self.last_update_time == 0.0:
            # Reconstructed PRBS: stm32_buf has 2000 chips/s, need ~5 s for correlation
            min_stm32 = int(self.correlation_window_s * PRBS_CHIP_RATE_HZ * 0.5)  # ~5000 chips
            emg_len = len(self.emg_word_buf) if len(self.emg_word_buf) > 0 else len(self.emg_buf)
            return len(self.stm32_buf) > min_stm32 and emg_len > 2000
        return (CLOCK() - self.last_update_time) >= self.update_interval_s

    def update_sync(self):
        with self._lock:
            min_stm32 = int(self.correlation_window_s * PRBS_CHIP_RATE_HZ * 0.2)  # ~2000 chips
            emg_len = len(self.emg_word_buf) if len(self.emg_word_buf) > 0 else len(self.emg_buf)
            if len(self.stm32_buf) < min_stm32 or emg_len < 200:
                return
            self._update_in_progress = True

        try:
            with self._lock:
                s_t = np.array([t for t, _ in self.stm32_buf])
                s_v = np.array([v for _, v in self.stm32_buf])
                if len(self.emg_word_buf) > 0:
                    e_t = np.array([t for t, _ in self.emg_word_buf])
                    e_v = np.array([x for _, x in self.emg_word_buf], dtype=np.int64)
                else:
                    e_t = np.array([t for t, _ in self.emg_buf])
                    e_v = np.array([v for _, v in self.emg_buf], dtype=np.float64)
            self._update_sync_impl(s_t, s_v, e_t, e_v)
        finally:
            with self._lock:
                self._update_in_progress = False

    def _update_sync_impl(self, s_t: np.ndarray, s_v: np.ndarray,
                          e_t: np.ndarray, e_v: np.ndarray):
        min_chips = int(self.correlation_window_s * PRBS_CHIP_RATE_HZ * 0.2)
        if len(s_t) < min_chips or len(e_t) < 200:
            return
        latest = max(s_t[-1], e_t[-1])
        cutoff = latest - self.correlation_window_s
        sm, em = s_t >= cutoff, e_t >= cutoff
        s_t, s_v = s_t[sm], s_v[sm]
        e_t, e_v = e_t[em], e_v[em]
        if len(s_t) < 50 or len(e_t) < 50:
            return
        t_lo, t_hi = max(s_t[0], e_t[0]), min(s_t[-1], e_t[-1])
        if t_hi - t_lo < 0.5:
            return
        common_t = np.arange(t_lo, t_hi, self.resample_dt)
        if len(common_t) < 100:
            return
        s_rs = np.interp(common_t, s_t, s_v)

        def _norm_peak(a: np.ndarray, b: np.ndarray) -> float:
            """Return max abs normalized correlation peak; 0.0 if ill-conditioned."""
            sa = float(np.std(a))
            sb = float(np.std(b))
            if not np.isfinite(sa) or not np.isfinite(sb) or sa < 1e-9 or sb < 1e-9:
                return 0.0
            if _USE_SCIPY:
                cc = sp_correlate(a, b, mode="full")
            else:
                cc = np.correlate(a, b, mode="full")
            n = sa * sb * len(a)
            if not np.isfinite(n) or n <= 0:
                return 0.0
            cc = cc / n
            return float(np.max(np.abs(cc)))

        # If EMG is a raw word stream, choose the best extraction by correlation peak.
        # This is more reliable than "most transitions" when the Dig/STATUS word contains
        # multiple unrelated toggling bits.
        if e_v.dtype.kind in ("i", "u"):
            raw_words = e_v.astype(np.int64)

            # Decide which candidate bits to try: only bits that actually vary in-window.
            candidate_bits: List[int] = []
            for b in range(0, 16):
                vb = (raw_words >> b) & 1
                m = float(np.mean(vb)) if vb.size else 0.0
                if 0.05 < m < 0.95:
                    candidate_bits.append(b)
            # Always keep a small fallback set
            if not candidate_bits:
                candidate_bits = [0, 1, 2, 3]

            # Build candidate e_rs streams on the common time-grid
            # Note: nearest-neighbour like behaviour is fine for digital; np.interp gives linear but values are 0/1.
            rw_float = raw_words.astype(np.float64)
            best_peak = -1.0
            best_mode: Optional[str] = None
            best_bit: Optional[int] = None
            best_e_rs: Optional[np.ndarray] = None

            # Candidate 1: per-bit extraction
            for b in candidate_bits:
                vb = ((raw_words >> b) & 1).astype(np.float64)
                e_bit = 2.0 * vb - 1.0
                e_rs_try = np.interp(common_t, e_t, e_bit)
                peak_try = _norm_peak(e_rs_try, s_rs)
                if peak_try > best_peak:
                    best_peak = peak_try
                    best_mode = "bit"
                    best_bit = int(b)
                    best_e_rs = e_rs_try

            # Candidate 2: threshold on the raw word values (works for two-level encodings like {2,6})
            lo, hi = float(np.min(rw_float)), float(np.max(rw_float))
            if hi - lo > 1e-9:
                thr = (lo + hi) / 2.0
                e_thr = 2.0 * (rw_float > thr).astype(np.float64) - 1.0
                e_rs_try = np.interp(common_t, e_t, e_thr)
                peak_try = _norm_peak(e_rs_try, s_rs)
                if peak_try > best_peak:
                    best_peak = peak_try
                    best_mode = "threshold"
                    best_bit = None
                    best_e_rs = e_rs_try

            # Candidate 3: nonzero
            e_nz = 2.0 * (raw_words != 0).astype(np.float64) - 1.0
            e_rs_try = np.interp(common_t, e_t, e_nz)
            peak_try = _norm_peak(e_rs_try, s_rs)
            if peak_try > best_peak:
                best_peak = peak_try
                best_mode = "nonzero"
                best_bit = None
                best_e_rs = e_rs_try

            # Lock in selection for future updates
            with self._lock:
                self._trig_mode = best_mode
                self._trig_bit = best_bit
            e_rs = best_e_rs if best_e_rs is not None else self._extract_emg_trig_bipolar_from_words(raw_words)
        else:
            e_rs = np.interp(common_t, e_t, e_v)

        if _USE_SCIPY:
            corr = sp_correlate(e_rs, s_rs, mode="full")
        else:
            corr = np.correlate(e_rs, s_rs, mode="full")
        norm = np.std(e_rs) * np.std(s_rs) * len(e_rs)
        if not np.isfinite(norm) or norm <= 0:
            # Ill-conditioned (e.g. constant TRIG after extraction) -> no update.
            return
        corr /= norm
        peak_idx = int(np.argmax(np.abs(corr)))
        peak_val = float(np.abs(corr[peak_idx]))
        center = len(corr) // 2
        lag_samples = peak_idx - center
        offset_ms = lag_samples * self.resample_dt * 1000.0
        confidence = min(1.0, peak_val * 0.7)

        with self._lock:
            self.kf_variance += self._kf_Q
            K = self.kf_variance / (self.kf_variance + self._kf_R / max(confidence, 0.01))
            self.kf_offset += K * (offset_ms - self.kf_offset)
            self.kf_variance *= (1.0 - K)
            old_off = self.current_offset_ms
            self.current_offset_ms = self.kf_offset
            now = CLOCK()
            if self.last_update_time > 0:
                dt_s = now - self.last_update_time
                if dt_s > 0:
                    # ppm = (delta_offset_sec / elapsed_sec) * 1e6
                    self.drift_rate_ppm = (
                        (self.current_offset_ms - old_off) / 1000.0 / dt_s * 1e6
                    )
            self.last_update_time = now
            self.update_count += 1
            self.sync_state = SyncState(
                timestamp=now,
                offset_ms=self.current_offset_ms,
                drift_rate_ppm=self.drift_rate_ppm,
                correlation_peak=peak_val,
                confidence=confidence,
                samples_analyzed=len(common_t),
            )

        a = abs(self.current_offset_ms)
        if a > self.drift_error_ms:
            print(f"\n[SYNC] ERROR: drift {a:.1f} ms > {self.drift_error_ms} ms")
        elif a > self.drift_warning_ms and self.update_count % 5 == 1:
            print(f"\n[SYNC] WARNING: drift {a:.1f} ms")

    def get_sync_state(self) -> Optional[SyncState]:
        with self._lock:
            return self.sync_state

    def get_corrected_time(self, stm32_time: float) -> float:
        with self._lock:
            return stm32_time - self.current_offset_ms / 1000.0


# =============================================================================
# HIGH-LEVEL SIGNAL ACQUISITION
# =============================================================================

class SignalAcquisition:
    """
    Single entry point for STM32 + EMG + PRBS sync.
    start() / stop() manage all threads; get_* methods expose data.
    """

    def __init__(self, config: Optional[SignalAcquisitionConfig] = None):
        self.config = config or SignalAcquisitionConfig()
        self._sync_engine: Optional[HardwarePRBSSync] = None
        self._reader: Optional[STM32Reader] = None
        self._emg_thread: Optional[EMGAcquisitionThread] = None
        self._emg_buffer: Optional[TimestampedBuffer] = None

    def start(self) -> None:
        """Start STM32 (if enabled), sync engine (if PRBS), and EMG (if enabled)."""
        cfg = self.config
        if cfg.enable_emg and not _TMSI_AVAILABLE:
            raise RuntimeError("EMG enabled but TMSi SDK not available. Add tmsi-python-interface to project.")

        if cfg.enable_prbs_sync and cfg.enable_stm32 and cfg.enable_emg:
            self._sync_engine = HardwarePRBSSync(
                correlation_window_s=cfg.prbs_correlation_window_s,
                update_interval_s=cfg.prbs_update_interval_s,
                resample_rate_hz=cfg.prbs_resample_rate_hz,
                drift_warning_ms=cfg.prbs_drift_warning_ms,
                drift_error_ms=cfg.prbs_drift_error_ms,
            )
            if cfg.verbose:
                print("[SYNC] Hardware PRBS-15 synchronisation enabled")

        if cfg.enable_stm32:
            def _on_stm32(sample: SampleSTM32):
                if self._sync_engine is not None:
                    self._sync_engine.add_stm32_prbs_sample(
                        CLOCK(),
                        sample.t_ms,
                        sample.prbs_tick,
                        sample.in_mark,
                    )

            self._reader = STM32Reader(
                port=cfg.stm32_port,
                baud=cfg.stm32_baud,
                capacity=cfg.stm32_capacity,
                on_sample=_on_stm32,
                verbose=cfg.verbose,
            )
            self._reader.start()
            if cfg.verbose:
                print("[STM32] Reader started")

        if cfg.enable_emg:
            self._emg_buffer = TimestampedBuffer(
                maxlen=cfg.max_emg_buffer_size,
                max_get_recent=cfg.max_get_recent_items,
            )
            self._emg_thread = EMGAcquisitionThread(
                self._emg_buffer,
                self._sync_engine,
                connection_type=cfg.emg_connection_type,
                emg_differential_pairs=cfg.emg_differential_pairs,
                sample_rate=cfg.emg_sample_rate,
                poll_sleep_s=cfg.emg_poll_sleep_s,
                verbose=cfg.verbose,
            )
            self._emg_thread.start()
            if cfg.verbose:
                print("[MAIN] Waiting for EMG to be ready...")
            t_wait = CLOCK()
            while not self._emg_thread.ready_event.is_set():
                if self._emg_thread.error:
                    self.stop()
                    raise RuntimeError(f"EMG init error: {self._emg_thread.error}")
                if CLOCK() - t_wait > 60:
                    self.stop()
                    raise TimeoutError("EMG not ready within 60 s")
                time.sleep(0.1)
            if cfg.verbose:
                print("[MAIN] EMG ready")

    def stop(self) -> None:
        if self._emg_thread is not None:
            self._emg_thread.stop()
            self._emg_thread.join(timeout=5)
            self._emg_thread = None
        if self._reader is not None:
            self._reader.stop()
            self._reader = None

    def get_stm32_snapshot(self, n: int = 5000) -> Dict[str, np.ndarray]:
        if self._reader is None:
            return {}
        return self._reader.get_snapshot(n)

    def get_emg_recent(self, window_seconds: float, now: Optional[float] = None) -> List[Tuple[float, EMGData]]:
        if self._emg_buffer is None:
            return []
        return self._emg_buffer.get_recent(window_seconds, now=now)

    def get_sync_state(self) -> Optional[SyncState]:
        if self._sync_engine is None:
            return None
        return self._sync_engine.get_sync_state()

    def get_corrected_time(self, stm32_time: float) -> float:
        if self._sync_engine is None:
            return stm32_time
        return self._sync_engine.get_corrected_time(stm32_time)

    def trigger_prbs_update(self) -> None:
        """Run PRBS sync update in a background thread if due."""
        if self._sync_engine is not None and self._sync_engine.should_update():
            threading.Thread(target=self._sync_engine.update_sync, name="PRBS-Sync", daemon=True).start()

    @property
    def stm32_reader(self) -> Optional[STM32Reader]:
        return self._reader

    @property
    def emg_thread(self) -> Optional[EMGAcquisitionThread]:
        return self._emg_thread

    @property
    def sync_engine(self) -> Optional[HardwarePRBSSync]:
        return self._sync_engine

    @property
    def emg_buffer(self) -> Optional[TimestampedBuffer]:
        return self._emg_buffer


# =============================================================================
# CLI RUNNERS (for setup_scripts/signal_acquisition_testing.py)
# =============================================================================

def run_stm32_only(config: SignalAcquisitionConfig, duration_s: Optional[float]) -> None:
    if config.verbose:
        print("\n" + "=" * 50 + "\nPhase 1: STM32 only\n" + "=" * 50 + "\n  Target: 500 Hz. Press Ctrl+C to stop.\n")
    reader = STM32Reader(
        port=config.stm32_port,
        baud=config.stm32_baud,
        capacity=config.stm32_capacity,
        on_sample=None,
        verbose=config.verbose,
    )
    reader.start()
    start = CLOCK()
    try:
        while True:
            time.sleep(1.0)
            elapsed = CLOCK() - start
            if duration_s is not None and elapsed >= duration_s:
                break
            if config.verbose:
                rate = reader.sample_count / elapsed if elapsed > 0 else 0
                print(f"  [PERF] STM32: {rate:.1f} Hz (samples={reader.sample_count}, elapsed={elapsed:.1f} s)")
    except KeyboardInterrupt:
        if config.verbose:
            print("\n[STM32] Interrupted")
    reader.stop()
    elapsed = CLOCK() - start
    if config.verbose:
        rate = reader.sample_count / elapsed if elapsed > 0.5 else 0
        print(f"\n  [SUMMARY] STM32: {rate:.1f} Hz, {reader.sample_count} samples, {elapsed:.1f} s")


def run_emg_only(config: SignalAcquisitionConfig, duration_s: Optional[float]) -> None:
    if not _TMSI_AVAILABLE:
        print("\nERROR: TMSi SDK not available. Cannot run EMG-only mode.")
        return
    if config.verbose:
        print("\n" + "=" * 50 + "\nPhase 2: EMG only\n" + "=" * 50 + "\n  Target: 2000 Hz. Press Ctrl+C to stop.\n")
    buffer = TimestampedBuffer(maxlen=config.max_emg_buffer_size, max_get_recent=config.max_get_recent_items)
    emg_thread = EMGAcquisitionThread(
        buffer, None,
        connection_type=config.emg_connection_type,
        emg_differential_pairs=config.emg_differential_pairs,
        sample_rate=config.emg_sample_rate,
        poll_sleep_s=config.emg_poll_sleep_s,
        verbose=config.verbose,
    )
    emg_thread.start()
    if config.verbose:
        print("[MAIN] Waiting for EMG ready...")
    t_wait = CLOCK()
    while not emg_thread.ready_event.is_set():
        if emg_thread.error:
            raise RuntimeError(f"EMG init error: {emg_thread.error}")
        if CLOCK() - t_wait > 60:
            raise TimeoutError("EMG not ready within 60 s")
        time.sleep(0.1)
    if config.verbose:
        print("[MAIN] EMG ready.\n")
    start = CLOCK()
    try:
        while True:
            time.sleep(1.0)
            elapsed = CLOCK() - start
            if duration_s is not None and elapsed >= duration_s:
                break
            if config.verbose:
                st = emg_thread.get_stats()
                print(f"  [PERF] EMG: {st['rate']:.0f} Hz (samples={st['samples']}, chunks={st['chunks']}, elapsed={elapsed:.1f} s)")
    except KeyboardInterrupt:
        if config.verbose:
            print("\n[EMG] Interrupted")
    emg_thread.stop()
    emg_thread.join(timeout=5)
    if config.verbose:
        st = emg_thread.get_stats()
        rate = st["samples"] / (CLOCK() - start) if (CLOCK() - start) > 0.5 else 0
        print(f"\n  [SUMMARY] EMG: {rate:.1f} Hz, {st['samples']} samples")


def run_prbs_test(config: SignalAcquisitionConfig, duration_s: Optional[float], chip_rate_hz: int = 2000) -> None:
    if not _TMSI_AVAILABLE:
        print("\nERROR: TMSi SDK not available. Cannot run PRBS test.")
        return
    duration = duration_s if duration_s is not None else 5.0
    if config.verbose:
        print("\n" + "=" * 50 + "\nPhase: PRBS (EMG TRIG)\n" + "=" * 50 +
              f"\n  Target: PRBS on TRIG (~{chip_rate_hz} Hz). Duration: {duration:.0f} s. Press Ctrl+C to stop.\n")
    sync_engine = HardwarePRBSSync(
        correlation_window_s=config.prbs_correlation_window_s,
        update_interval_s=config.prbs_update_interval_s,
        resample_rate_hz=config.prbs_resample_rate_hz,
        drift_warning_ms=config.prbs_drift_warning_ms,
        drift_error_ms=config.prbs_drift_error_ms,
    )

    def _on_stm32(sample: SampleSTM32):
        sync_engine.add_stm32_prbs_sample(
            CLOCK(), sample.t_ms, sample.prbs_tick, sample.in_mark
        )

    reader = STM32Reader(
        port=config.stm32_port,
        baud=config.stm32_baud,
        capacity=config.stm32_capacity,
        on_sample=_on_stm32,
        verbose=config.verbose,
    )
    reader.start()
    emg_buffer = TimestampedBuffer(maxlen=config.max_emg_buffer_size, max_get_recent=config.max_get_recent_items)
    emg_thread = EMGAcquisitionThread(
        emg_buffer, sync_engine,
        connection_type=config.emg_connection_type,
        emg_differential_pairs=config.emg_differential_pairs,
        sample_rate=config.emg_sample_rate,
        poll_sleep_s=config.emg_poll_sleep_s,
        verbose=config.verbose,
    )
    emg_thread.start()
    if config.verbose:
        print("[MAIN] Waiting for EMG ready...")
    t_wait = CLOCK()
    while not emg_thread.ready_event.is_set():
        if emg_thread.error:
            reader.stop()
            raise RuntimeError(f"EMG init error: {emg_thread.error}")
        if CLOCK() - t_wait > 60:
            reader.stop()
            raise TimeoutError("EMG not ready within 60 s")
        time.sleep(0.1)
    if config.verbose:
        print("[MAIN] EMG ready. Collecting TRIG data...\n")
    start = CLOCK()
    try:
        while (CLOCK() - start) < duration:
            time.sleep(0.2)
    except KeyboardInterrupt:
        if config.verbose:
            print("\n[PRBS] Interrupted")
    elapsed = CLOCK() - start
    emg_thread.stop()
    emg_thread.join(timeout=5)
    reader.stop()

    # Compute TRIG transition stats from whichever buffer we populated.
    with sync_engine._lock:
        word_data = list(sync_engine.emg_word_buf)
        bp_data = list(sync_engine.emg_buf)
        mode = getattr(sync_engine, "_trig_mode", None)
        bit = getattr(sync_engine, "_trig_bit", None)

    if len(word_data) >= 2:
        ts = np.array([t for t, _ in word_data], dtype=np.float64)
        raw = np.array([x for _, x in word_data], dtype=np.int64)
        # Use selected extraction if available, else use threshold (matches trig_sync_testing default for {2,6}).
        if mode == "bit" and bit is not None:
            sig = ((raw >> int(bit)) & 1).astype(np.float64)
            extracted = f"bit{int(bit)}"
        else:
            lo, hi = float(np.min(raw)), float(np.max(raw))
            thr = (lo + hi) / 2.0 if hi - lo > 1e-9 else 0.0
            sig = (raw.astype(np.float64) > thr).astype(np.float64)
            extracted = "threshold"
        transition_count = int(np.sum(np.abs(np.diff(sig)) > 0))
        window_duration = float(ts[-1] - ts[0]) if ts[-1] > ts[0] else elapsed
        transition_rate_hz = transition_count / window_duration if window_duration > 0 else 0.0
        if config.verbose:
            print(f"  [PRBS] TRIG extraction used for stats: {extracted}")
    elif len(bp_data) >= 2:
        ts = np.array([t for t, _ in bp_data], dtype=np.float64)
        vs = np.array([v for _, v in bp_data], dtype=np.float64)
        transition_count = int(np.sum(np.diff(vs) != 0))
        window_duration = float(ts[-1] - ts[0]) if ts[-1] > ts[0] else elapsed
        transition_rate_hz = transition_count / window_duration if window_duration > 0 else 0.0
    else:
        transition_rate_hz = 0.0
        transition_count = 0
    if config.verbose:
        sr = getattr(emg_thread, "sample_rate", 0) or 2000.0
        # For NRZ PRBS with ~50/50 bits, expected transitions are ~0.5 * chip_rate_hz.
        expected_transitions_hz = 0.5 * float(chip_rate_hz)
        result = "OK" if transition_rate_hz >= (0.25 * chip_rate_hz) else ("LOW" if transition_rate_hz > 0 else "NONE")
        print(f"  [PRBS] EMG TRIG: transitions={transition_count}, rate={transition_rate_hz:.0f} Hz (expected ~{expected_transitions_hz:.0f})")
        print(f"  [PRBS] PRBS signal on EMG: {result}\n")


def run_combined(config: SignalAcquisitionConfig, duration_s: Optional[float]) -> None:
    if not config.enable_stm32 and not config.enable_emg:
        print("\nERROR: both STM32 and EMG are disabled.")
        return
    if config.enable_emg and not _TMSI_AVAILABLE:
        print("\nERROR: EMG enabled but TMSi SDK not available.")
        return
    if config.verbose:
        print("\n" + "=" * 70 + "\nSYNCHRONIZED STM32 + EMG ACQUISITION\n" + "=" * 70)
        print("  Phase 3: Combined (STM32 + EMG + PRBS sync). Press Ctrl+C to stop.\n")

    acq = SignalAcquisition(config)
    acq.start()
    time.sleep(1.0)  # allow threads to settle
    # Record sample counts at the start of the measurement window so the
    # PERF rate reflects only the active period (not the 1 s settle sleep).
    stm32_count_at_start = acq.stm32_reader.sample_count if acq.stm32_reader else 0
    start_wall = CLOCK()
    last_log = start_wall
    run_duration = duration_s

    try:
        while True:
            now = CLOCK()
            if run_duration is not None and (now - start_wall) >= run_duration:
                if config.verbose:
                    print(f"\nDuration {run_duration} s reached.")
                break
            acq.trigger_prbs_update()
            if config.verbose and (now - last_log) >= 1.0:
                last_log = now
                elapsed = now - start_wall
                if acq.stm32_reader is not None:
                    r = acq.stm32_reader
                    n = r.sample_count - stm32_count_at_start
                    rate = n / elapsed if elapsed > 0 else 0
                    print(f"  [PERF] STM32: {rate:.1f} Hz (samples={n}, elapsed={elapsed:.1f} s)")
                if acq.emg_thread is not None:
                    es = acq.emg_thread.get_stats()
                    print(f"  [PERF] EMG: {es['rate']:.0f} Hz (samples={es['samples']}, chunks={es['chunks']}, elapsed={elapsed:.1f} s)")
                ss = acq.get_sync_state()
                if ss is not None:
                    print(f"  [PERF] PRBS sync: offset={ss.offset_ms:+.2f} ms, confidence={ss.confidence:.3f}, drift={ss.drift_rate_ppm:.1f} ppm")
                else:
                    print("  [PERF] PRBS sync: (waiting for first update)")
            time.sleep(0.1)
    except KeyboardInterrupt:
        if config.verbose:
            print("\n\n[MAIN] Interrupted by user")
    except Exception as e:
        print(f"\n\n[MAIN] Error: {e}")
        traceback.print_exc()

    # Capture stats before stop() clears references
    stm32_reader = acq.stm32_reader
    emg_thread = acq.emg_thread
    sync_state = acq.get_sync_state()
    acq.stop()

    if config.verbose:
        print("\n" + "=" * 70 + "\nACQUISITION STATISTICS\n" + "=" * 70)
        if stm32_reader is not None:
            elapsed_r = CLOCK() - (getattr(stm32_reader, "_start_pc", 0) or CLOCK())
            rate_r = stm32_reader.sample_count / elapsed_r if elapsed_r > 0.5 else 0
            print(f"\n  [SUMMARY] STM32: {rate_r:.1f} Hz, {stm32_reader.sample_count} samples, {elapsed_r:.1f} s")
        if emg_thread is not None:
            es = emg_thread.get_stats()
            print(f"  [SUMMARY] EMG: {es['rate']:.1f} Hz, {es['samples']} samples, {es['elapsed']:.1f} s")
        if sync_state is not None:
            print(f"  [SUMMARY] PRBS sync: offset={sync_state.offset_ms:+.2f} ms, confidence={sync_state.confidence:.3f}")
        print("\n" + "=" * 70 + "\nSHUTDOWN COMPLETE\n" + "=" * 70)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="STM32 + EMG signal acquisition. Use --mode to run phases separately or in sequence.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python -m src.signal_acquisition --mode sequence --duration 10
  python -m src.signal_acquisition --mode stm32 --duration 10
  python -m src.signal_acquisition --mode emg --duration 10
  python -m src.signal_acquisition --mode prbs --duration 5
  python -m src.signal_acquisition --mode all --duration 15
""",
    )
    parser.add_argument("--mode", choices=["stm32", "emg", "all", "prbs", "sequence"], default="sequence",
                        help="stm32=Phase 1 only; emg=Phase 2 only; prbs=PRBS TRIG test; all=combined; sequence=all phases")
    parser.add_argument("--duration", type=float, default=None, metavar="SEC", help="Run each phase for N seconds")
    args = parser.parse_args()

    config = SignalAcquisitionConfig()

    if args.mode == "stm32":
        run_stm32_only(config, args.duration)
    elif args.mode == "emg":
        run_emg_only(config, args.duration)
    elif args.mode == "prbs":
        run_prbs_test(config, args.duration)
    elif args.mode == "all":
        run_combined(config, args.duration)
    else:
        run_stm32_only(config, args.duration)
        run_emg_only(config, args.duration)
        run_prbs_test(config, args.duration)
        run_combined(config, args.duration)


if __name__ == "__main__":
    main()
