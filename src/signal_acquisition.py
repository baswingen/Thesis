"""
Synchronized STM32 + EMG signal acquisition (native module)
==========================================================

Single, program-wide signal acquisition for:
- STM32F401: dual BNO085 IMU + button matrix + PRBS TRIG output (500 Hz)
- TMSi Porti7: multi-channel EMG + TRIG input (~2000 Hz)
- PRBS-15 hardware sync: STM32 PA8 → Porti7 TRIG; cross-correlation for clock offset.

Comprehensive diagnostics are ALWAYS enabled, capturing:
- Raw signal timelines (STM32 PRBS, EMG TRIG)
- Bit extraction candidates and selection
- Cross-correlation results with peak analysis
- Drift compensation evolution over time
- Timestamp jitter and spacing distributions
- Sync state history (offset, confidence, drift rate)

Usage:
  from src.signal_acquisition import SignalAcquisition, SignalAcquisitionConfig

  config = SignalAcquisitionConfig()
  acq = SignalAcquisition(config)
  acq.start()
  snap = acq.get_stm32_snapshot(500)
  emg_list = acq.get_emg_recent(5.0)
  sync = acq.get_sync_state()
  
  # Save diagnostic plots (automatic in run_prbs_test and run_combined)
  acq.save_diagnostic_plot()  # Saves to prbs_diagnostics_YYYYMMDD_HHMMSS.png
  
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
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Literal, Optional, Tuple

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
from .stm32_emg_sync import (
    SyncDelayEstimator,
    SyncDelayResult,
    DelaySignal,
    compute_sync_delay_signal,
    align_emg_to_stm32,
)

CLOCK = time.perf_counter


# =============================================================================
# PRBS-15 RECONSTRUCTION (must match Arduino sketch)
# =============================================================================

# STM32 sketch: CHIP_RATE_HZ=500, PRBS15 seed 0x7ACE
# Mark period REMOVED to preserve PRBS correlation properties
# Scaled to 500 Hz to match STM32 sample rate (1:1 ratio for optimal reconstruction)
PRBS_CHIP_RATE_HZ = 500
PRBS15_SEED = 0x7ACE
PRBS15_PERIOD = (1 << 15) - 1  # 32_767 chips (~65.5 seconds at 500 Hz)


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
    Reconstruct PRBS-15 output at global chip index (0-based).
    Returns ±1.0 (bipolar) for correlation with EMG TRIG.
    
    PRBS-15 period is 32767 chips (~16.4 seconds at 2 kHz).
    No frame markers - continuous PRBS for optimal correlation.
    """
    # Direct mapping: chip_idx -> LFSR sequence index
    lfsr_idx = chip_idx % PRBS15_PERIOD
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
    sync_mode: Literal["realtime", "postprocessing", "none"] = "realtime"
    prbs_correlation_window_s: float = 10.0  # Increased from 5s for better correlation at 500 Hz
    prbs_update_interval_s: float = 2.0  # Kalman smoothing needs infrequent measurements
    prbs_chip_rate_hz: float = 10.0  # PRBS chip rate (Hz); must match STM32 output
    # Resample rate used for PRBS cross-correlation. Match the chip rate for
    # optimal correlation (500 Hz PRBS → 500 Hz resample).
    prbs_resample_rate_hz: float = 500.0
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


@dataclass
class RecordedSession:
    """Recorded session for postprocessing sync. Contains both signals and delay signal."""

    emg_chunks: List[Tuple[float, EMGData]]
    stm32_samples: List[SampleSTM32]
    delay_signal: DelaySignal
    sync_result: SyncDelayResult


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
        sync_delay_estimator: Optional[SyncDelayEstimator] = None,
        trig_collector: Optional[List[Tuple[np.ndarray, np.ndarray]]] = None,
    ):
        super().__init__(name="EMG-Thread", daemon=True)
        self.emg_buffer = emg_buffer
        self.sync_engine = sync_engine
        self.sync_delay_estimator = sync_delay_estimator
        self.trig_collector = trig_collector  # For postprocessing: [(ts, trig_raw), ...]
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
            if self.trig_idx is not None:
                if self.sync_delay_estimator is not None:
                    self.sync_delay_estimator.set_trig_is_status(self.trig_is_status_bit)
                if self.verbose:
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

                if self.trig_idx is not None and self.sync_delay_estimator is not None:
                    trig_raw = samples[:, self.trig_idx].astype(np.float64)
                    self.sync_delay_estimator.add_emg_trig_chunk(ts_arr, trig_raw)

                if self.trig_idx is not None and self.trig_collector is not None:
                    trig_raw = samples[:, self.trig_idx].astype(np.float64)
                    self.trig_collector.append((ts_arr.copy(), trig_raw.copy()))

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
    Cross-correlate reconstructed PRBS-15 (from STM32 prbs_tick) with Porti7 TRIG.
    Reconstructs the full 500 Hz PRBS on the host from deterministic (t_ms, prbs_tick).
    Continuous PRBS-15 generation (no frame markers) for optimal correlation properties.
    Scaled to 500 Hz to match STM32 sample rate (1:1 ratio for optimal reconstruction).
    """

    CHIP_PERIOD_S = 1.0 / PRBS_CHIP_RATE_HZ  # 0.0005 s

    def __init__(self, *,
                 correlation_window_s: float = 10.0,  # Increased for 500 Hz PRBS
                 update_interval_s: float = 5.0,
                 resample_rate_hz: float = 500.0,  # Match PRBS chip rate
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
        self._stm32_queue: queue.Queue = queue.Queue(maxsize=200_000)  # Issue 16: increased from 60k
        self._queue_drops = 0  # Issue 16: track dropped samples
        self._drain_stop = threading.Event()
        self._last_prbs_tick: Optional[int] = None
        self._last_stm32_t_ms: Optional[float] = None
        # Map STM32 internal time (t_ms) into the host perf_counter domain.
        # Set ONCE (first valid sample) so the STM32 timestamps are monotonic
        # (no USB serial jitter) but still roughly aligned with host time.
        self._stm32_time_offset_s: Optional[float] = None
        # Drift estimation: anchor (receive_time, prbs_tick) from the first
        # valid sample, then derive the real chip period in host seconds by
        # cumulative regression.  This drift factor is used in the correlation
        # step to warp s_t from STM32 time to host/EMG time.
        self._anchor_receive_time: Optional[float] = None
        self._anchor_prbs_tick: Optional[int] = None
        self._real_chip_period: float = self.CHIP_PERIOD_S  # start with ideal 1/2000
        # Issue 18: robust drift estimation with regression
        self._drift_history: deque = deque(maxlen=100)
        self._last_chip_period_update = 0.0
        # Diagnostics
        self.diagnostics_enabled = False
        self.diagnostic_snapshots: List[Dict[str, Any]] = []
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
            receive_time, t_ms, prbs_tick = item

            # Detect STM32 restarts / wraparound in t_ms; reset mapping.
            if self._last_stm32_t_ms is not None:
                if float(t_ms) + 1000.0 < float(self._last_stm32_t_ms):
                    self._last_prbs_tick = None
                    self._stm32_time_offset_s = None
                    self._anchor_receive_time = None
                    self._anchor_prbs_tick = None
                    self._real_chip_period = self.CHIP_PERIOD_S
            self._last_stm32_t_ms = float(t_ms)

            # --- Map STM32 internal time to host domain (fixed offset) ---
            # Using t_ms gives perfectly monotonic timestamps (no USB jitter)
            # at the STM32 clock rate.  The drift relative to the EMG crystal
            # clock is corrected in _update_sync_impl before correlation.
            stm32_now_s = float(t_ms) / 1000.0
            if self._stm32_time_offset_s is None:
                self._stm32_time_offset_s = float(receive_time) - stm32_now_s
            mapped_now_s = stm32_now_s + self._stm32_time_offset_s

            # --- Drift estimation (Issue 18: robust regression-based) ---
            # Track (receive_time, prbs_tick) to compute the real chip period
            # in host seconds.  This factor is used by the correlation step to
            # warp STM32 timestamps into the host/EMG time domain.
            if self._anchor_receive_time is None:
                self._anchor_receive_time = float(receive_time)
                self._anchor_prbs_tick = int(prbs_tick)
            
            # Store measurement for regression
            self._drift_history.append((float(receive_time), int(prbs_tick)))
            
            # Update every 5 seconds if we have enough data (robust regression)
            if len(self._drift_history) >= 20 and (receive_time - self._last_chip_period_update) > 5.0:
                # Linear regression: prbs_tick = slope * receive_time + intercept
                # slope = chips/second, so chip_period = 1/slope
                times = np.array([t for t, _ in self._drift_history])
                ticks = np.array([p for _, p in self._drift_history])
                
                # Reject outliers (USB jitter)
                coeffs = np.polyfit(times, ticks, 1)
                predicted = np.polyval(coeffs, times)
                residuals = ticks - predicted
                std_res = np.std(residuals)
                inliers = np.abs(residuals) < 3 * std_res
                
                if np.sum(inliers) >= 10:
                    coeffs = np.polyfit(times[inliers], ticks[inliers], 1)
                    predicted_inliers = np.polyval(coeffs, times[inliers])
                    residuals_inliers = ticks[inliers] - predicted_inliers
                    var_residuals = np.var(residuals_inliers) if len(residuals_inliers) > 1 else 0
                    var_ticks = np.var(ticks[inliers]) if len(ticks[inliers]) > 1 else 1
                    r2 = 1 - (var_residuals / var_ticks) if var_ticks > 0 else 0
                    
                    if r2 > 0.99 and coeffs[0] > 0:  # Only update if fit is excellent
                        measured_chips_per_sec = coeffs[0]
                        self._real_chip_period = 1.0 / measured_chips_per_sec
                        self._last_chip_period_update = receive_time

            last = self._last_prbs_tick
            self._last_prbs_tick = prbs_tick
            if last is None:
                # First sample: set starting point, don't backfill historical
                # chips whose timestamps would be placed with the not-yet-
                # converged drift estimate, contaminating the correlation window.
                continue
            chip_start = last
            chip_end = prbs_tick
            if chip_end <= chip_start:
                continue

            with self._lock:
                for c in range(chip_start, chip_end):
                    # Issue 15: Use measured chip period, not nominal
                    t_chip = mapped_now_s - (prbs_tick - 1 - c) * self._real_chip_period
                    val = reconstruct_prbs_chip_value(c)
                    self.stm32_buf.append((t_chip, val))
                
                # Validate PRBS reconstruction (diagnostic check)
                if self.diagnostics_enabled and len(self.stm32_buf) % 10000 == 0:
                    # Every 10k samples, validate PRBS properties
                    recent_vals = [v for _, v in list(self.stm32_buf)[-1000:]]
                    if len(recent_vals) >= 1000:
                        transitions = sum(1 for i in range(len(recent_vals)-1) 
                                        if recent_vals[i] != recent_vals[i+1])
                        ones = sum(1 for v in recent_vals if v > 0)
                        # PRBS should have ~500 transitions and ~50% ones
                        if transitions < 200 or ones < 200 or ones > 800:
                            print(f"[WARN] PRBS reconstruction anomaly: "
                                  f"transitions={transitions}/1000, ones={ones}/1000")

    def add_stm32_prbs_sample(self, receive_time: float, t_ms: float,
                              prbs_tick: int) -> None:
        """Feed (receive_time, t_ms, prbs_tick) for PRBS reconstruction."""
        try:
            self._stm32_queue.put_nowait((receive_time, t_ms, prbs_tick))
        except queue.Full:
            # Issue 16: Track and warn about dropped samples
            self._queue_drops += 1
            if self._queue_drops % 100 == 1:
                print(f"[WARN] PRBS queue overflow: {self._queue_drops} samples dropped")

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
            # Reconstructed PRBS: stm32_buf has 500 chips/s, need ~10s window for correlation
            min_stm32 = int(self.correlation_window_s * PRBS_CHIP_RATE_HZ * 0.5)  # ~2500 chips
            emg_len = len(self.emg_word_buf) if len(self.emg_word_buf) > 0 else len(self.emg_buf)
            return len(self.stm32_buf) > min_stm32 and emg_len > 2000
        return (CLOCK() - self.last_update_time) >= self.update_interval_s

    def update_sync(self):
        with self._lock:
            min_stm32 = int(self.correlation_window_s * PRBS_CHIP_RATE_HZ * 0.2)  # ~1000 chips (10s * 500 Hz * 0.2)
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
        min_chips = int(self.correlation_window_s * PRBS_CHIP_RATE_HZ * 0.2)  # ~1000 chips
        if len(s_t) < min_chips or len(e_t) < 200:
            return
        latest = max(s_t[-1], e_t[-1])
        cutoff = latest - self.correlation_window_s
        sm, em = s_t >= cutoff, e_t >= cutoff
        s_t, s_v = s_t[sm], s_v[sm]
        e_t, e_v = e_t[em], e_v[em]
        if len(s_t) < 50 or len(e_t) < 50:
            return
        
        # Diagnostics: capture pre-correlation state
        diagnostic_snapshot = None
        if self.diagnostics_enabled:
            diagnostic_snapshot = {
                'timestamp': CLOCK(),
                'stm32_t_raw': s_t.copy(),
                'stm32_v_raw': s_v.copy(),
                'emg_t_raw': e_t.copy(),
                'emg_v_raw': e_v.copy() if e_v.dtype.kind == 'f' else None,
                'emg_words_raw': e_v.copy() if e_v.dtype.kind in ('i', 'u') else None,
                'trig_mode': self._trig_mode,
                'trig_bit': self._trig_bit,
                'real_chip_period': self._real_chip_period,
                'nominal_chip_period': self.CHIP_PERIOD_S,
                'bit_candidates': {},
            }
        # ---- Clock-drift compensation ----
        # s_t comes from the STM32's millis() clock, which may run ~1 % faster
        # than the Porti7's crystal.  Without correction the accumulated phase
        # error over a 5 s window is ~42 ms, smearing the correlation peak into
        # noise.  Warp s_t so that its time scale matches the host/EMG clock.
        rcp = self._real_chip_period          # actual chip period in host-seconds
        if rcp != self.CHIP_PERIOD_S and rcp > 0:
            drift_factor = self.CHIP_PERIOD_S / rcp   # >1 when STM32 runs fast
            s_mid = 0.5 * (s_t[0] + s_t[-1])
            s_t = s_mid + (s_t - s_mid) / drift_factor

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

            # Test ALL 16 bits to find the one carrying PRBS (enhanced diagnostics)
            candidate_bits: List[int] = []
            for b in range(0, 16):
                vb = (raw_words >> b) & 1
                m = float(np.mean(vb)) if vb.size else 0.0
                # Accept any bit that toggles (even if only 1% or 99% high)
                # This catches PRBS that might be inverted or have low duty cycle
                if 0.01 < m < 0.99:
                    candidate_bits.append(b)
            # Always test at least bits 0-7 as fallback
            if not candidate_bits:
                candidate_bits = list(range(8))

            # Build candidate e_rs streams on the common time-grid
            # Note: nearest-neighbour like behaviour is fine for digital; np.interp gives linear but values are 0/1.
            rw_float = raw_words.astype(np.float64)
            best_peak = -1.0
            best_mode: Optional[str] = None
            best_bit: Optional[int] = None
            best_e_rs: Optional[np.ndarray] = None

            # Candidate 1: per-bit extraction (test both polarities)
            for b in candidate_bits:
                vb = ((raw_words >> b) & 1).astype(np.float64)
                
                # Test normal polarity
                e_bit = 2.0 * vb - 1.0
                e_rs_try = np.interp(common_t, e_t, e_bit)
                peak_try = _norm_peak(e_rs_try, s_rs)
                
                # Test inverted polarity (PRBS might be inverted)
                e_bit_inv = -1.0 * e_bit
                e_rs_try_inv = np.interp(common_t, e_t, e_bit_inv)
                peak_try_inv = _norm_peak(e_rs_try_inv, s_rs)
                
                # Use whichever polarity gives better correlation
                if peak_try_inv > peak_try:
                    peak_try = peak_try_inv
                    e_rs_try = e_rs_try_inv
                    polarity = "inverted"
                else:
                    polarity = "normal"
                
                # Diagnostics: record all candidates
                if diagnostic_snapshot is not None:
                    diagnostic_snapshot['bit_candidates'][f'bit{b}'] = {
                        'peak': peak_try,
                        'transitions': int(np.sum(np.abs(np.diff(e_bit)))),
                        'mean': float(np.mean(vb)),
                        'polarity': polarity,
                    }
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
                # Diagnostics: record threshold candidate
                if diagnostic_snapshot is not None:
                    diagnostic_snapshot['bit_candidates']['threshold'] = {
                        'peak': peak_try,
                        'threshold': thr,
                        'range': (lo, hi),
                    }
                if peak_try > best_peak:
                    best_peak = peak_try
                    best_mode = "threshold"
                    best_bit = None
                    best_e_rs = e_rs_try

            # Candidate 3: nonzero
            e_nz = 2.0 * (raw_words != 0).astype(np.float64) - 1.0
            e_rs_try = np.interp(common_t, e_t, e_nz)
            peak_try = _norm_peak(e_rs_try, s_rs)
            # Diagnostics: record nonzero candidate
            if diagnostic_snapshot is not None:
                diagnostic_snapshot['bit_candidates']['nonzero'] = {
                    'peak': peak_try,
                }
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
            
            # Diagnostics: complete snapshot with correlation results
            if diagnostic_snapshot is not None:
                diagnostic_snapshot.update({
                    'common_t': common_t.copy(),
                    'stm32_resampled': s_rs.copy(),
                    'emg_resampled': e_rs.copy(),
                    'correlation': corr.copy(),
                    'peak_idx': peak_idx,
                    'peak_val': peak_val,
                    'lag_samples': lag_samples,
                    'offset_ms': offset_ms,
                    'confidence': confidence,
                    'selected_mode': best_mode if e_v.dtype.kind in ('i', 'u') else 'direct',
                    'selected_bit': best_bit if e_v.dtype.kind in ('i', 'u') else None,
                    'kf_offset': self.kf_offset,
                    'kf_variance': self.kf_variance,
                    'drift_rate_ppm': self.drift_rate_ppm,
                })
                self.diagnostic_snapshots.append(diagnostic_snapshot)

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

    def enable_diagnostics(self) -> None:
        """Enable diagnostic snapshot capture during sync updates (enabled by default)."""
        with self._lock:
            self.diagnostics_enabled = True
            self.diagnostic_snapshots = []
    
    def get_diagnostic_summary(self) -> Dict[str, Any]:
        """Get summary statistics from diagnostic snapshots."""
        with self._lock:
            if not self.diagnostic_snapshots:
                return {}
            
            latest = self.diagnostic_snapshots[-1]
            
            # Get all bit candidates sorted by peak correlation
            bit_candidates = latest.get('bit_candidates', {})
            sorted_bits = sorted(bit_candidates.items(), 
                               key=lambda x: x[1].get('peak', 0), 
                               reverse=True)
            
            return {
                'num_snapshots': len(self.diagnostic_snapshots),
                'latest_offset_ms': latest.get('offset_ms', 0),
                'latest_confidence': latest.get('confidence', 0),
                'latest_peak': latest.get('peak_val', 0),
                'selected_mode': latest.get('selected_mode', 'unknown'),
                'selected_bit': latest.get('selected_bit'),
                'real_chip_period_ms': latest.get('real_chip_period', 0.0005) * 1000,
                'drift_rate_ppm': latest.get('drift_rate_ppm', 0),
                'top_3_bits': [(name, data.get('peak', 0), data.get('polarity', 'N/A')) 
                              for name, data in sorted_bits[:3]],
                'all_bit_peaks': {name: data.get('peak', 0) for name, data in bit_candidates.items()},
            }
    
    def validate_prbs_reconstruction(self) -> Dict[str, Any]:
        """
        Validate that reconstructed PRBS has expected statistical properties.
        
        Returns dict with validation metrics.
        """
        with self._lock:
            if len(self.stm32_buf) < 1000:
                return {'valid': False, 'reason': 'insufficient_data'}
            
            # Get recent 2000 samples (1 second at 2000 Hz)
            recent = list(self.stm32_buf)[-2000:]
            times = np.array([t for t, _ in recent])
            values = np.array([v for _, v in recent])
            
            # Check 1: Temporal spacing should be uniform (~0.5ms)
            spacing = np.diff(times) * 1000  # ms
            mean_spacing = np.mean(spacing)
            std_spacing = np.std(spacing)
            
            # Check 2: Values should be bipolar ±1
            unique_vals = np.unique(values)
            is_bipolar = len(unique_vals) == 2 and -1 in values and 1 in values
            
            # Check 3: ~50% should be +1 (balanced PRBS)
            ones_frac = np.mean(values > 0)
            
            # Check 4: Transition rate should be ~50% (PRBS property)
            transitions = np.sum(np.abs(np.diff(values)) > 0.5)
            transition_rate = transitions / len(values)
            
            # Check 5: No long runs (PRBS-15 max run = 15)
            max_run = 1
            current_run = 1
            for i in range(1, len(values)):
                if values[i] == values[i-1]:
                    current_run += 1
                    max_run = max(max_run, current_run)
                else:
                    current_run = 1
            
            # Identify issues
            issues = []
            if not is_bipolar:
                issues.append(f"Not bipolar: values={unique_vals.tolist()}")
            if not (0.4 < ones_frac < 0.6):
                issues.append(f"Unbalanced: {ones_frac:.1%} ones (expect ~50%)")
            if not (0.3 < transition_rate < 0.7):
                issues.append(f"Low transitions: {transition_rate:.1%} (expect ~50%)")
            if max_run > 20:
                issues.append(f"Long run: {max_run} (PRBS-15 max should be ~15)")
            if std_spacing > 0.1:
                issues.append(f"Jittery spacing: σ={std_spacing:.3f}ms")
            
            return {
                'valid': True,
                'source': 'stm32',
                'num_samples': len(values),
                'mean_spacing_ms': float(mean_spacing),
                'std_spacing_ms': float(std_spacing),
                'is_bipolar': bool(is_bipolar),
                'unique_values': unique_vals.tolist(),
                'ones_fraction': float(ones_frac),
                'transition_rate': float(transition_rate),
                'max_run_length': int(max_run),
                'issues': issues,
                'health': 'good' if len(issues) == 0 else ('warning' if len(issues) <= 2 else 'bad')
            }
    
    def validate_emg_trig(self) -> Dict[str, Any]:
        """
        Validate EMG TRIG signal properties to detect noise, wrong bit, etc.
        
        Returns dict with validation metrics.
        """
        with self._lock:
            # Use whichever buffer has data
            if len(self.emg_word_buf) > 1000:
                recent = list(self.emg_word_buf)[-2000:]
                times = np.array([t for t, _ in recent])
                raw_words = np.array([x for _, x in recent], dtype=np.int64)
                
                # Extract using current mode
                if self._trig_mode == "bit" and self._trig_bit is not None:
                    vb = ((raw_words >> self._trig_bit) & 1).astype(np.float64)
                    values = 2.0 * vb - 1.0
                    extraction = f"bit{self._trig_bit}"
                elif self._trig_mode == "threshold":
                    rw = raw_words.astype(np.float64)
                    lo, hi = float(np.min(rw)), float(np.max(rw))
                    thr = (lo + hi) / 2.0 if hi - lo > 1e-9 else 0.0
                    values = 2.0 * (rw > thr).astype(np.float64) - 1.0
                    extraction = "threshold"
                else:
                    values = 2.0 * (raw_words != 0).astype(np.float64) - 1.0
                    extraction = "nonzero"
            elif len(self.emg_buf) > 1000:
                recent = list(self.emg_buf)[-2000:]
                times = np.array([t for t, _ in recent])
                values = np.array([v for _, v in recent])
                extraction = "direct"
            else:
                return {'valid': False, 'reason': 'insufficient_data'}
            
            # Compute metrics (same as PRBS)
            spacing = np.diff(times) * 1000 if len(times) > 1 else np.array([])
            mean_spacing = np.mean(spacing) if len(spacing) > 0 else 0
            std_spacing = np.std(spacing) if len(spacing) > 0 else 0
            
            unique_vals = np.unique(values)
            is_bipolar = len(unique_vals) == 2 and -1 in values and 1 in values
            ones_frac = np.mean(values > 0)
            
            transitions = np.sum(np.abs(np.diff(values)) > 0.5) if len(values) > 1 else 0
            transition_rate = transitions / (len(values) - 1) if len(values) > 1 else 0
            
            # Expected: PRBS at 500 Hz with ~50% transitions = ~250 Hz
            # At EMG sample rate (usually 2000 Hz), we expect ~12.5% transition rate
            # (4 EMG samples per PRBS chip, so 250 Hz transitions / 2000 Hz samples = 12.5%)
            window_duration = times[-1] - times[0] if len(times) > 1 else 1.0
            transition_rate_hz = transitions / window_duration if window_duration > 0 else 0
            
            # Identify issues
            issues = []
            if not is_bipolar:
                issues.append(f"Not bipolar: values={unique_vals.tolist()[:5]}")
            if not (0.4 < ones_frac < 0.6):
                issues.append(f"Unbalanced: {ones_frac:.1%} ones")
            # Transition rate validation: allow 200-300 Hz for 500Hz PRBS
            if transition_rate_hz < 200:
                issues.append(f"Too few transitions: {transition_rate_hz:.0f} Hz (expect ~250 Hz)")
            elif transition_rate_hz > 300:
                issues.append(f"Too many transitions: {transition_rate_hz:.0f} Hz (expect ~250 Hz) - NOISE or WRONG BIT?")
            
            return {
                'valid': True,
                'source': 'emg_trig',
                'extraction_mode': extraction,
                'num_samples': len(values),
                'mean_spacing_ms': float(mean_spacing),
                'std_spacing_ms': float(std_spacing),
                'is_bipolar': bool(is_bipolar),
                'ones_fraction': float(ones_frac),
                'transition_rate': float(transition_rate),
                'transition_rate_hz': float(transition_rate_hz),
                'issues': issues,
                'health': 'good' if len(issues) == 0 else ('warning' if len(issues) <= 2 else 'bad')
            }


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
        self._sync_delay_estimator: Optional[SyncDelayEstimator] = None
        self._reader: Optional[STM32Reader] = None
        self._emg_thread: Optional[EMGAcquisitionThread] = None
        self._emg_buffer: Optional[TimestampedBuffer] = None
        self._postprocessing_trig_collector: List[Tuple[np.ndarray, np.ndarray]] = []
        self._postprocessing_stm32_samples: List[SampleSTM32] = []

    def start(self) -> None:
        """Start STM32 (if enabled), sync engine (if PRBS), and EMG (if enabled)."""
        cfg = self.config
        if cfg.enable_emg and not _TMSI_AVAILABLE:
            raise RuntimeError("EMG enabled but TMSi SDK not available. Add tmsi-python-interface to project.")

        use_realtime_sync = (
            cfg.enable_prbs_sync
            and cfg.enable_stm32
            and cfg.enable_emg
            and cfg.sync_mode == "realtime"
        )
        use_postprocessing_sync = (
            cfg.enable_prbs_sync
            and cfg.enable_stm32
            and cfg.enable_emg
            and cfg.sync_mode == "postprocessing"
        )

        if use_postprocessing_sync:
            self._postprocessing_trig_collector.clear()
            self._postprocessing_stm32_samples.clear()

        if use_realtime_sync:
            emg_rate = cfg.emg_sample_rate or 2000
            self._sync_delay_estimator = SyncDelayEstimator(
                chip_rate_hz=cfg.prbs_chip_rate_hz,
                emg_sample_rate=float(emg_rate),
                sync_window_s=cfg.prbs_correlation_window_s,
                update_interval_s=cfg.prbs_update_interval_s,
            )
            if cfg.verbose:
                print(f"[SYNC] Real-time PRBS sync enabled (Kalman-smoothed, {cfg.prbs_chip_rate_hz} Hz chip rate)")

        if cfg.enable_stm32:
            # Issue 24: Start STM32 reader WITHOUT callback initially to avoid
            # collecting PRBS data before EMG is ready
            self._reader = STM32Reader(
                port=cfg.stm32_port,
                baud=cfg.stm32_baud,
                capacity=cfg.stm32_capacity,
                on_sample=None,  # No callback yet
                verbose=cfg.verbose,
            )
            self._reader.start()
            if cfg.verbose:
                print("[STM32] Reader started (PRBS callback pending EMG)")

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
                sync_delay_estimator=self._sync_delay_estimator,
                trig_collector=self._postprocessing_trig_collector if use_postprocessing_sync else None,
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
            
            # Activate STM32 callback for sync (real-time or postprocessing)
            if cfg.enable_stm32 and self._reader is not None:
                if self._sync_delay_estimator is not None:
                    def _on_stm32_realtime(sample: SampleSTM32):
                        if self._sync_delay_estimator is not None:
                            self._sync_delay_estimator.add_stm32_samples([sample])
                    self._reader.on_sample = _on_stm32_realtime
                    if cfg.verbose:
                        print("[SYNC] PRBS callback activated (real-time), settling for 2 seconds...")
                    time.sleep(2.0)
                elif use_postprocessing_sync:
                    def _on_stm32_postproc(sample: SampleSTM32):
                        self._postprocessing_stm32_samples.append(sample)
                    self._reader.on_sample = _on_stm32_postproc
                    if cfg.verbose:
                        print("[SYNC] PRBS callback activated (postprocessing mode)")

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
        if self._sync_delay_estimator is not None:
            result = self._sync_delay_estimator.get_result()
            if result is not None:
                return SyncState(
                    timestamp=0.0,
                    offset_ms=self._sync_delay_estimator.get_delay_ms(),
                    drift_rate_ppm=self._sync_delay_estimator.get_drift_rate_ppm(),
                    correlation_peak=result.correlation_peak,
                    confidence=result.confidence,
                    samples_analyzed=len(result.lag_series_chips),
                )
            return None
        if self._sync_engine is not None:
            return self._sync_engine.get_sync_state()
        return None

    def get_sync_delay_ms(self) -> float:
        """Return current Kalman-smoothed sync delay in milliseconds (real-time mode)."""
        if self._sync_delay_estimator is not None:
            return self._sync_delay_estimator.get_delay_ms()
        return 0.0

    def get_corrected_time(self, stm32_time: float) -> float:
        if self._sync_delay_estimator is not None:
            return stm32_time - self._sync_delay_estimator.get_delay_ms() / 1000.0
        if self._sync_engine is not None:
            return self._sync_engine.get_corrected_time(stm32_time)
        return stm32_time

    def trigger_prbs_update(self) -> None:
        """Run PRBS sync update in a background thread if due."""
        if self._sync_delay_estimator is not None and self._sync_delay_estimator.should_update():
            threading.Thread(
                target=self._sync_delay_estimator.update,
                name="PRBS-Sync",
                daemon=True,
            ).start()
        elif self._sync_engine is not None and self._sync_engine.should_update():
            threading.Thread(target=self._sync_engine.update_sync, name="PRBS-Sync", daemon=True).start()
    
    def get_diagnostic_summary(self) -> Dict[str, Any]:
        """Get summary of diagnostic data from PRBS sync engine."""
        if self._sync_delay_estimator is not None:
            result = self._sync_delay_estimator.get_result()
            if result is not None:
                return {
                    "num_snapshots": self._sync_delay_estimator._update_count,
                    "latest_offset_ms": self._sync_delay_estimator.get_delay_ms(),
                    "latest_confidence": result.confidence,
                    "latest_peak": result.correlation_peak,
                    "drift_rate_ppm": self._sync_delay_estimator.get_drift_rate_ppm(),
                }
            return {}
        if self._sync_engine is not None:
            return self._sync_engine.get_diagnostic_summary()
        return {}
    
    def save_diagnostic_plot(self, output_path: Optional[str] = None) -> Optional[str]:
        """
        Save comprehensive diagnostic plot to file.

        Returns the path where the plot was saved, or None if diagnostics unavailable.
        (Only available when using legacy HardwarePRBSSync; real-time mode has no diagnostic snapshots.)
        """
        if self._sync_engine is None or not self._sync_engine.diagnostic_snapshots:
            return None

        if output_path is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = str(Path.cwd() / f"prbs_diagnostics_{timestamp}.png")

        plot_prbs_diagnostics(self._sync_engine, output_path)
        return output_path

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
    def sync_delay_estimator(self) -> Optional[SyncDelayEstimator]:
        return self._sync_delay_estimator

    @property
    def emg_buffer(self) -> Optional[TimestampedBuffer]:
        return self._emg_buffer

    def record_for_postprocessing(
        self, duration_s: float
    ) -> Optional[RecordedSession]:
        """
        Record for given duration, then compute delay signal for postprocessing.

        Must be called after start() with sync_mode="postprocessing".
        Blocks for duration_s, then stops acquisition and returns RecordedSession
        with emg_chunks, stm32_samples, delay_signal, and sync_result.
        """
        if self.config.sync_mode != "postprocessing":
            raise RuntimeError(
                "record_for_postprocessing requires sync_mode='postprocessing'. "
                "Create SignalAcquisition with sync_mode='postprocessing' and start() first."
            )
        if self._emg_thread is None or self._reader is None:
            raise RuntimeError("Acquisition not started. Call start() first.")

        time.sleep(duration_s)
        emg_chunks = self._emg_buffer.get_all() if self._emg_buffer else []
        stm32_samples = list(self._postprocessing_stm32_samples)
        emg_thread = self._emg_thread
        self.stop()

        if not self._postprocessing_trig_collector or not stm32_samples:
            return None

        trig_arrays = [trig for _, trig in self._postprocessing_trig_collector]
        emg_trig_raw = np.concatenate(trig_arrays)
        emg_sample_rate = emg_thread.sample_rate if emg_thread else 2000.0
        is_status = getattr(emg_thread, "trig_is_status_bit", False)

        try:
            delay_signal, sync_result = compute_sync_delay_signal(
                emg_trig_raw=emg_trig_raw,
                stm32_samples=stm32_samples,
                emg_sample_rate=emg_sample_rate,
                is_status=is_status,
            )
        except ValueError:
            return None

        return RecordedSession(
            emg_chunks=emg_chunks,
            stm32_samples=stm32_samples,
            delay_signal=delay_signal,
            sync_result=sync_result,
        )


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


def _run_prbs_test_integrated(config: SignalAcquisitionConfig, duration: float) -> None:
    """PRBS test using integrated SyncDelayEstimator (10 Hz chip rate, Kalman)."""
    cfg = SignalAcquisitionConfig(
        enable_stm32=True,
        enable_emg=True,
        enable_prbs_sync=True,
        sync_mode="realtime",
        verbose=config.verbose,
        stm32_port=config.stm32_port,
        stm32_baud=config.stm32_baud,
        prbs_correlation_window_s=config.prbs_correlation_window_s,
        prbs_update_interval_s=config.prbs_update_interval_s,
    )
    acq = SignalAcquisition(cfg)
    acq.start()
    time.sleep(1.0)
    start = CLOCK()
    last_log = start
    try:
        while (CLOCK() - start) < duration:
            now = CLOCK()
            acq.trigger_prbs_update()
            if config.verbose and (now - last_log) >= 1.0:
                last_log = now
                ss = acq.get_sync_state()
                if ss is not None:
                    print(f"  [PRBS] offset={ss.offset_ms:+.2f} ms, confidence={ss.confidence:.3f}, drift={ss.drift_rate_ppm:.1f} ppm")
                else:
                    print("  [PRBS] waiting for first sync update...")
            time.sleep(0.2)
    except KeyboardInterrupt:
        if config.verbose:
            print("\n[PRBS] Interrupted")
    acq.stop()
    if config.verbose:
        ss = acq.get_sync_state()
        if ss is not None:
            print(f"\n  [SUMMARY] PRBS sync: offset={ss.offset_ms:+.2f} ms, confidence={ss.confidence:.3f}")


def run_prbs_test(config: SignalAcquisitionConfig, duration_s: Optional[float], chip_rate_hz: int = 10) -> None:
    if not _TMSI_AVAILABLE:
        print("\nERROR: TMSi SDK not available. Cannot run PRBS test.")
        return
    duration = duration_s if duration_s is not None else 5.0
    if config.verbose:
        print("\n" + "=" * 50 + "\nPhase: PRBS (EMG TRIG)\n" + "=" * 50 +
              f"\n  Target: PRBS on TRIG (10 Hz chip rate). Duration: {duration:.0f} s. Press Ctrl+C to stop.\n")

    # Use integrated SyncDelayEstimator when sync_mode is realtime (default)
    if config.sync_mode == "realtime":
        _run_prbs_test_integrated(config, duration)
        return

    # Legacy HardwarePRBSSync path (500 Hz) for backward compatibility
    sync_engine = HardwarePRBSSync(
        correlation_window_s=config.prbs_correlation_window_s,
        update_interval_s=config.prbs_update_interval_s,
        resample_rate_hz=config.prbs_resample_rate_hz,
        drift_warning_ms=config.prbs_drift_warning_ms,
        drift_error_ms=config.prbs_drift_error_ms,
    )
    
    # Diagnostics always enabled for comprehensive signal analysis
    sync_engine.enable_diagnostics()
    if config.verbose:
        print("[DIAGNOSTICS] Enabled - will generate plots after test")

    def _on_stm32(sample: SampleSTM32):
        sync_engine.add_stm32_prbs_sample(
            CLOCK(), sample.t_ms, sample.prbs_tick
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
    last_update_check = start
    update_count = 0
    try:
        while (CLOCK() - start) < duration:
            # Trigger sync updates periodically (this was missing!)
            now = CLOCK()
            if now - last_update_check >= 1.0:
                if sync_engine.should_update():
                    update_count += 1
                    if config.verbose:
                        print(f"  [PRBS] Triggering sync update #{update_count} at t={now-start:.1f}s...")
                    sync_engine.update_sync()
                    state = sync_engine.get_sync_state()
                    if state and config.verbose:
                        print(f"  [PRBS] Sync result: offset={state.offset_ms:.2f}ms, "
                              f"confidence={state.confidence:.3f}, peak={state.correlation_peak:.3f}")
                        
                        # Validate signals after first update
                        if update_count == 1:
                            stm32_val = sync_engine.validate_prbs_reconstruction()
                            emg_val = sync_engine.validate_emg_trig()
                            if stm32_val.get('health') != 'good' or emg_val.get('health') != 'good':
                                print(f"  [WARN] Signal validation issues detected!")
                                if stm32_val.get('issues'):
                                    print(f"    STM32: {', '.join(stm32_val['issues'])}")
                                if emg_val.get('issues'):
                                    print(f"    EMG TRIG: {', '.join(emg_val['issues'])}")
                last_update_check = now
            time.sleep(0.2)
    except KeyboardInterrupt:
        if config.verbose:
            print("\n[PRBS] Interrupted")
    elapsed = CLOCK() - start
    
    if config.verbose and update_count == 0:
        print(f"  [WARN] No sync updates occurred during {elapsed:.1f}s test!")
        print(f"         Need more data: STM32={len(sync_engine.stm32_buf)}, EMG={len(sync_engine.emg_word_buf) + len(sync_engine.emg_buf)}")
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
    
    # Generate diagnostic plots (always enabled)
    if len(sync_engine.diagnostic_snapshots) > 0:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = Path.cwd() / f"prbs_diagnostics_{timestamp}.png"
        if config.verbose:
            print(f"[DIAGNOSTICS] Generating comprehensive diagnostic plots...")
        plot_prbs_diagnostics(sync_engine, str(output_path))
        
        # Print enhanced diagnostic summary
        diag_summary = sync_engine.get_diagnostic_summary()
        print(f"\n[DIAGNOSTICS] Top correlation peaks:")
        for name, peak, polarity in diag_summary.get('top_3_bits', []):
            print(f"  {name}: {peak:.4f} ({polarity})")
        
        # Print PRBS validation
        validation = sync_engine.validate_prbs_reconstruction()
        if validation.get('valid'):
            print(f"\n[STM32 PRBS VALIDATION]")
            print(f"  Samples: {validation['num_samples']}, Bipolar: {validation['is_bipolar']}")
            print(f"  Ones fraction: {validation['ones_fraction']:.1%}, Transitions: {validation['transition_rate']:.1%}")
            print(f"  Max run length: {validation['max_run_length']}, Spacing: {validation['mean_spacing_ms']:.3f}ms")
            if validation.get('issues'):
                print(f"  Issues: {', '.join(validation['issues'])}")
        
        # Validate EMG TRIG signal
        emg_validation = sync_engine.validate_emg_trig()
        if emg_validation.get('valid'):
            print(f"\n[EMG TRIG VALIDATION]")
            print(f"  Extraction: {emg_validation.get('extraction_mode', 'unknown')}")
            print(f"  Samples: {emg_validation['num_samples']}, Bipolar: {emg_validation['is_bipolar']}")
            print(f"  Ones fraction: {emg_validation['ones_fraction']:.1%}")
            print(f"  Transition rate: {emg_validation['transition_rate']:.1%} ({emg_validation['transition_rate_hz']:.0f} Hz)")
            if emg_validation.get('issues'):
                print(f"  ⚠️  ISSUES: {', '.join(emg_validation['issues'])}")
    else:
        if config.verbose:
            print("[DIAGNOSTICS] No snapshots captured - sync may not have run")


def plot_prbs_diagnostics(sync_engine: HardwarePRBSSync, output_path: Optional[str] = None) -> None:
    """
    Generate comprehensive 9-panel diagnostic plot for PRBS synchronization.
    
    Shows all signals, timestamps, bit extraction candidates, correlation results,
    and drift compensation to help identify why cross-correlation might fail.
    """
    try:
        import matplotlib.pyplot as plt
        from datetime import datetime
    except ImportError:
        print("[WARN] matplotlib not available - skipping diagnostic plots")
        return
    
    if not sync_engine.diagnostic_snapshots:
        print("[WARN] No diagnostic snapshots available - enable diagnostics before test")
        return
    
    # Use the most recent snapshot
    snap = sync_engine.diagnostic_snapshots[-1]
    
    # Create figure with 9 subplots
    fig = plt.figure(figsize=(16, 22))
    gs = fig.add_gridspec(9, 1, hspace=0.4)
    
    # Panel 1: Raw STM32 PRBS reconstruction timeline
    ax1 = fig.add_subplot(gs[0])
    if 'stm32_t_raw' in snap and 'stm32_v_raw' in snap:
        s_t = snap['stm32_t_raw']
        s_v = snap['stm32_v_raw']
        # Show last 2 seconds for clarity
        if len(s_t) > 4000:
            s_t = s_t[-4000:]
            s_v = s_v[-4000:]
        ax1.plot(s_t - s_t[0], s_v, linewidth=0.5, color='steelblue', alpha=0.7)
        ax1.set_ylabel('PRBS Value')
        ax1.set_title('Panel 1: STM32 Reconstructed PRBS (last 2s)')
        ax1.set_ylim(-1.2, 1.2)
        ax1.grid(True, alpha=0.3)
        ax1.set_xlabel('Time (s, relative)')
    
    # Panel 2: Raw EMG TRIG signal
    ax2 = fig.add_subplot(gs[1])
    if 'emg_t_raw' in snap:
        e_t = snap['emg_t_raw']
        if 'emg_words_raw' in snap and snap['emg_words_raw'] is not None:
            e_raw = snap['emg_words_raw']
            # Show last 2 seconds
            if len(e_t) > 4000:
                e_t = e_t[-4000:]
                e_raw = e_raw[-4000:]
            ax2.plot(e_t - e_t[0], e_raw, linewidth=0.5, color='orange', alpha=0.7)
            ax2.set_ylabel('Raw Word Value')
            ax2.set_title('Panel 2: EMG TRIG Raw Words (last 2s)')
        elif 'emg_v_raw' in snap and snap['emg_v_raw'] is not None:
            e_v = snap['emg_v_raw']
            if len(e_t) > 4000:
                e_t = e_t[-4000:]
                e_v = e_v[-4000:]
            ax2.plot(e_t - e_t[0], e_v, linewidth=0.5, color='orange', alpha=0.7)
            ax2.set_ylabel('Bipolar Value')
            ax2.set_title('Panel 2: EMG TRIG Bipolar (last 2s)')
        ax2.grid(True, alpha=0.3)
        ax2.set_xlabel('Time (s, relative)')
    
    # Panel 3: Overlaid signals on common time axis
    ax3 = fig.add_subplot(gs[2])
    if 'common_t' in snap and 'stm32_resampled' in snap and 'emg_resampled' in snap:
        c_t = snap['common_t']
        s_rs = snap['stm32_resampled']
        e_rs = snap['emg_resampled']
        # Show middle 1 second for clarity
        mid = len(c_t) // 2
        window = min(1000, len(c_t) // 2)
        idx_start = max(0, mid - window)
        idx_end = min(len(c_t), mid + window)
        t_plot = c_t[idx_start:idx_end] - c_t[idx_start]
        ax3.plot(t_plot, s_rs[idx_start:idx_end], linewidth=1, color='steelblue', alpha=0.7, label='STM32 PRBS')
        ax3.plot(t_plot, e_rs[idx_start:idx_end], linewidth=1, color='orange', alpha=0.7, label='EMG TRIG')
        ax3.set_ylabel('Resampled Signal')
        ax3.set_title('Panel 3: Overlaid STM32 PRBS + EMG TRIG (resampled, 1s window)')
        ax3.set_ylim(-1.5, 1.5)
        ax3.legend(loc='upper right')
        ax3.grid(True, alpha=0.3)
        ax3.set_xlabel('Time (s, relative)')
    
    # Panel 4: Bit extraction candidates (ALL bits with polarity info)
    ax4 = fig.add_subplot(gs[3])
    if 'bit_candidates' in snap and snap['bit_candidates']:
        candidates = snap['bit_candidates']
        # Sort by peak correlation for better visualization
        sorted_items = sorted(candidates.items(), key=lambda x: x[1].get('peak', 0), reverse=True)
        names = [item[0] for item in sorted_items]
        peaks = [item[1].get('peak', 0) for item in sorted_items]
        polarities = [item[1].get('polarity', 'N/A') for item in sorted_items]
        
        # Color code: green=selected, blue=inverted polarity, gray=normal but not selected
        colors = []
        selected_name = f"bit{snap.get('selected_bit', -1)}" if snap.get('selected_mode') == 'bit' else None
        for name, pol in zip(names, polarities):
            if name == selected_name:
                colors.append('green')
            elif pol == 'inverted':
                colors.append('steelblue')
            else:
                colors.append('gray')
        
        bars = ax4.barh(names, peaks, color=colors, alpha=0.7)
        ax4.set_xlabel('Normalized Correlation Peak')
        selected_info = f"{snap.get('selected_mode', 'N/A')}"
        if snap.get('selected_bit') is not None:
            selected_info += f" (bit{snap.get('selected_bit')})"
        ax4.set_title(f"Panel 4: ALL Bit Extraction Candidates (selected: {selected_info})")
        ax4.grid(True, alpha=0.3, axis='x')
        
        # Add peak value annotations for top 5
        for i, (name, peak) in enumerate(zip(names[:5], peaks[:5])):
            ax4.text(peak, i, f' {peak:.3f}', va='center', fontsize=8)
        
        # Add legend
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor='green', alpha=0.7, label='Selected'),
            Patch(facecolor='steelblue', alpha=0.7, label='Inverted polarity'),
            Patch(facecolor='gray', alpha=0.7, label='Normal polarity')
        ]
        ax4.legend(handles=legend_elements, loc='lower right', fontsize=8)
    
    # Panel 5: Visual comparison of top 3 bits vs STM32 PRBS (NEW)
    ax5 = fig.add_subplot(gs[4])
    if 'bit_candidates' in snap and snap['bit_candidates'] and 'emg_words_raw' in snap and snap['emg_words_raw'] is not None:
        e_t = snap['emg_t_raw']
        e_words = snap['emg_words_raw'].astype(np.int64)
        s_t = snap['stm32_t_raw']
        s_v = snap['stm32_v_raw']
        
        # Get top 3 bits
        sorted_bits = sorted(snap['bit_candidates'].items(), 
                           key=lambda x: x[1].get('peak', 0), reverse=True)
        top_3_names = [name for name, _ in sorted_bits[:3] if name.startswith('bit')]
        
        # Extract bit numbers
        top_bits = []
        for name in top_3_names:
            if name.startswith('bit'):
                try:
                    top_bits.append(int(name[3:]))
                except:
                    pass
        
        # Show 0.5 second window in the middle
        mid_t = (e_t[0] + e_t[-1]) / 2
        window = 0.5
        mask_e = (e_t >= mid_t - window/2) & (e_t <= mid_t + window/2)
        mask_s = (s_t >= mid_t - window/2) & (s_t <= mid_t + window/2)
        
        e_t_plot = e_t[mask_e] - mid_t
        e_words_plot = e_words[mask_e]
        s_t_plot = s_t[mask_s] - mid_t
        s_v_plot = s_v[mask_s]
        
        # Plot STM32 PRBS reference
        ax5.plot(s_t_plot, s_v_plot + 4, linewidth=1, color='steelblue', alpha=0.8, label='STM32 PRBS (ref)')
        
        # Plot top 3 EMG bits
        colors = ['red', 'orange', 'yellow']
        for i, bit_num in enumerate(top_bits[:3]):
            bit_vals = ((e_words_plot >> bit_num) & 1).astype(np.float64)
            bit_bipolar = 2.0 * bit_vals - 1.0
            ax5.plot(e_t_plot, bit_bipolar + (2 - i), linewidth=0.8, 
                    color=colors[i], alpha=0.7, label=f'EMG bit{bit_num}')
        
        ax5.set_xlabel('Time (s, relative to center)')
        ax5.set_ylabel('Signal + offset')
        ax5.set_title('Panel 5: Visual Comparison - STM32 PRBS vs Top 3 EMG Bits (0.5s window)')
        ax5.legend(loc='upper right')
        ax5.grid(True, alpha=0.3)
        ax5.set_ylim(-0.5, 5.5)
    
    # Panel 6: Cross-correlation result (moved from panel 5)
    ax6 = fig.add_subplot(gs[5])
    if 'correlation' in snap and 'peak_idx' in snap:
        corr = snap['correlation']
        peak_idx = snap['peak_idx']
        lag_samples = snap.get('lag_samples', 0)
        # Show window around peak
        window_half = min(500, len(corr) // 4)
        idx_start = max(0, peak_idx - window_half)
        idx_end = min(len(corr), peak_idx + window_half)
        lags = np.arange(len(corr)) - len(corr) // 2
        ax6.plot(lags[idx_start:idx_end], corr[idx_start:idx_end], linewidth=1, color='purple')
        ax6.axvline(x=lag_samples, color='red', linestyle='--', linewidth=2, label=f'Peak lag={lag_samples}')
        ax6.set_xlabel('Lag (samples)')
        ax6.set_ylabel('Normalized Correlation')
        ax6.set_title(f"Panel 6: Cross-Correlation (peak={snap.get('peak_val', 0):.3f}, offset={snap.get('offset_ms', 0):.2f} ms)")
        ax6.legend()
        ax6.grid(True, alpha=0.3)
    
    # Panel 7: Drift compensation over time
    ax7 = fig.add_subplot(gs[6])
    if len(sync_engine.diagnostic_snapshots) > 1:
        times = [(s['timestamp'] - sync_engine.diagnostic_snapshots[0]['timestamp']) 
                 for s in sync_engine.diagnostic_snapshots]
        chip_periods = [s.get('real_chip_period', snap.get('nominal_chip_period', 0.0005)) * 1000 
                       for s in sync_engine.diagnostic_snapshots]
        nominal = snap.get('nominal_chip_period', 0.0005) * 1000
        ax7.plot(times, chip_periods, marker='o', linewidth=2, color='teal', label='Measured')
        ax7.axhline(y=nominal, color='gray', linestyle='--', linewidth=1, label='Nominal')
        ax7.set_xlabel('Time (s)')
        ax7.set_ylabel('Chip Period (ms)')
        ax7.set_title('Panel 7: Drift Compensation (measured chip period over time)')
        ax7.legend()
        ax7.grid(True, alpha=0.3)
    
    # Panel 8: Timestamp histograms (jitter analysis)
    ax8 = fig.add_subplot(gs[7])
    if 'stm32_t_raw' in snap and 'emg_t_raw' in snap:
        s_t = snap['stm32_t_raw']
        e_t = snap['emg_t_raw']
        if len(s_t) > 2:
            s_dt = np.diff(s_t) * 1000  # ms
            ax8.hist(s_dt, bins=50, alpha=0.5, color='steelblue', label=f'STM32 (σ={np.std(s_dt):.3f}ms)')
        if len(e_t) > 2:
            e_dt = np.diff(e_t) * 1000  # ms
            ax8.hist(e_dt, bins=50, alpha=0.5, color='orange', label=f'EMG (σ={np.std(e_dt):.3f}ms)')
        ax8.set_xlabel('Sample Spacing (ms)')
        ax8.set_ylabel('Count')
        ax8.set_title('Panel 8: Timestamp Spacing Distribution (jitter)')
        ax8.legend()
        ax8.grid(True, alpha=0.3, axis='y')
    
    # Panel 9: Sync state history
    ax9 = fig.add_subplot(gs[8])
    if len(sync_engine.diagnostic_snapshots) > 1:
        times = [(s['timestamp'] - sync_engine.diagnostic_snapshots[0]['timestamp']) 
                 for s in sync_engine.diagnostic_snapshots]
        offsets = [s.get('offset_ms', 0) for s in sync_engine.diagnostic_snapshots]
        confidences = [s.get('confidence', 0) for s in sync_engine.diagnostic_snapshots]
        
        ax9_twin = ax9.twinx()
        line1 = ax9.plot(times, offsets, marker='o', linewidth=2, color='crimson', label='Offset (ms)')
        line2 = ax9_twin.plot(times, confidences, marker='s', linewidth=2, color='green', alpha=0.7, label='Confidence')
        
        ax9.set_xlabel('Time (s)')
        ax9.set_ylabel('Offset (ms)', color='crimson')
        ax9_twin.set_ylabel('Confidence', color='green')
        ax9.set_title('Panel 9: Sync State History (offset & confidence over time)')
        ax9.tick_params(axis='y', labelcolor='crimson')
        ax9_twin.tick_params(axis='y', labelcolor='green')
        ax9.grid(True, alpha=0.3)
        
        # Combined legend
        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax9.legend(lines, labels, loc='upper left')
    
    # Add validation results as text annotation (both STM32 and EMG)
    stm32_val = sync_engine.validate_prbs_reconstruction()
    emg_val = sync_engine.validate_emg_trig()
    
    health_colors = {'good': 'lightgreen', 'warning': 'wheat', 'bad': 'lightcoral', 'unknown': 'lightgray'}
    
    validation_text = ""
    
    # STM32 PRBS validation
    if stm32_val.get('valid'):
        health = stm32_val.get('health', 'unknown')
        validation_text += (
            f"STM32 PRBS ({health.upper()}): {stm32_val['num_samples']} samples\n"
            f"  Bipolar: {stm32_val['is_bipolar']}, Ones: {stm32_val['ones_fraction']:.1%}\n"
            f"  Transitions: {stm32_val['transition_rate']:.1%}, Max run: {stm32_val['max_run_length']}\n"
        )
        if stm32_val.get('issues'):
            validation_text += f"  Issues: {'; '.join(stm32_val['issues'][:2])}\n"
    
    # EMG TRIG validation
    if emg_val.get('valid'):
        health_emg = emg_val.get('health', 'unknown')
        overall_health = health_emg if health_emg == 'bad' else (stm32_val.get('health', 'unknown') if stm32_val.get('valid') else 'unknown')
        
        validation_text += (
            f"\nEMG TRIG ({health_emg.upper()}): {emg_val['num_samples']} samples\n"
            f"  Mode: {emg_val.get('extraction_mode', '?')}, Ones: {emg_val['ones_fraction']:.1%}\n"
            f"  Trans: {emg_val['transition_rate']:.1%} ({emg_val['transition_rate_hz']:.0f} Hz)\n"
        )
        if emg_val.get('issues'):
            validation_text += f"  ⚠️  {'; '.join(emg_val['issues'][:2])}\n"
    else:
        overall_health = stm32_val.get('health', 'unknown') if stm32_val.get('valid') else 'unknown'
    
    if validation_text:
        fig.text(0.02, 0.98, validation_text.strip(), transform=fig.transFigure, 
                fontsize=8, verticalalignment='top', family='monospace',
                bbox=dict(boxstyle='round', facecolor=health_colors.get(overall_health, 'lightgray'), alpha=0.7))
    
    fig.suptitle(f'PRBS Synchronization Diagnostics - {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}', 
                 fontsize=14, fontweight='bold')
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"[OK] Diagnostic plot saved: {output_path}")
    
    plt.show(block=False)


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
    
    # Generate diagnostic plots if PRBS sync was enabled
    if config.enable_prbs_sync and acq.sync_engine is not None:
        diag_path = acq.save_diagnostic_plot()
        if diag_path and config.verbose:
            print(f"\n[DIAGNOSTICS] Plot saved: {diag_path}")
    
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
        
        # Print diagnostic summary with enhanced bit analysis
        diag_summary = acq.get_diagnostic_summary()
        if diag_summary:
            print(f"  [DIAGNOSTICS] Snapshots: {diag_summary.get('num_snapshots', 0)}, "
                  f"Mode: {diag_summary.get('selected_mode', 'N/A')}, "
                  f"Bit: {diag_summary.get('selected_bit', 'N/A')}")
            
            # Show top 3 bit candidates
            top_3 = diag_summary.get('top_3_bits', [])
            if top_3:
                print(f"  [DIAGNOSTICS] Top 3 bits by correlation:")
                for i, (name, peak, polarity) in enumerate(top_3, 1):
                    print(f"    {i}. {name}: peak={peak:.4f} ({polarity})")
        
        # Print validation results
        if acq.sync_engine:
            stm32_val = acq.sync_engine.validate_prbs_reconstruction()
            if stm32_val.get('valid'):
                health = stm32_val.get('health', 'unknown')
                print(f"  [STM32 PRBS] {health.upper()}: "
                      f"Bipolar={stm32_val['is_bipolar']}, "
                      f"Ones={stm32_val['ones_fraction']:.1%}, "
                      f"Trans={stm32_val['transition_rate']:.1%}")
                if stm32_val.get('issues'):
                    print(f"    Issues: {', '.join(stm32_val['issues'])}")
            
            emg_val = acq.sync_engine.validate_emg_trig()
            if emg_val.get('valid'):
                health = emg_val.get('health', 'unknown')
                print(f"  [EMG TRIG] {health.upper()}: "
                      f"Mode={emg_val.get('extraction_mode', '?')}, "
                      f"Ones={emg_val['ones_fraction']:.1%}, "
                      f"TransRate={emg_val['transition_rate_hz']:.0f}Hz")
                if emg_val.get('issues'):
                    print(f"    ⚠️  Issues: {', '.join(emg_val['issues'])}")
        
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
