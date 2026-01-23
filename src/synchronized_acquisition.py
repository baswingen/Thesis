"""
Synchronized IMU + EMG acquisition (native module)
==================================================

Reusable, thread-based acquisition for:
- Dual BMI160 IMU via Arduino (`IMUDevice`, hardware micros() timestamps)
- TMSi legacy EMG via USB (`EMGDevice`, COUNTER-based timing when available)

Design goals:
- Stable synchronization on a monotonic clock (default: time.perf_counter)
- Robustness to serial buffering and Arduino resets/reconnects
- Scalable EMG channel selection (raw channels and/or differential pairs)
- Clean API usable across the codebase (start/stop, callbacks, buffers)
"""

from __future__ import annotations

import time
import threading
from dataclasses import dataclass, field
from collections import deque
from typing import Callable, Deque, Dict, Iterable, List, Optional, Tuple, Any

import numpy as np

from .imu_acquisition import IMUDevice, IMUConfig, IMUReading
from .emg_acquisition import EMGDevice


# =============================================================================
# DATA MODELS
# =============================================================================


ClockFn = Callable[[], float]


@dataclass(frozen=True)
class IMUSample:
    """One IMU sample with synchronized timestamp."""
    t: float  # synchronized timestamp (clock domain)
    reading: IMUReading
    t_hardware_s: float  # Arduino micros() converted to seconds


@dataclass(frozen=True)
class EMGChunk:
    """
    One EMG chunk with synchronized timestamps.

    Notes:
    - `t` is the chunk midpoint (appropriate when plotting chunk means).
    - `sample_t` is per-sample timestamps in the same clock domain.
    """
    t: float
    sample_t: np.ndarray  # shape (n_samples,)
    data: Dict[str, np.ndarray]  # 'raw', 'pairs', etc.
    counter_raw_last: Optional[int] = None  # last sample's counter (raw integer)


# =============================================================================
# THREAD-SAFE BUFFER
# =============================================================================


class TimestampedBuffer:
    """Thread-safe buffer for (timestamp, item)."""

    def __init__(self, maxlen: int = 10000):
        self._buf: Deque[Tuple[float, Any]] = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def append(self, t: float, item: Any) -> None:
        with self._lock:
            self._buf.append((t, item))

    def get_all(self) -> List[Tuple[float, Any]]:
        with self._lock:
            return list(self._buf)

    def get_recent(self, now_t: float, window_s: float) -> List[Tuple[float, Any]]:
        cutoff = now_t - window_s
        with self._lock:
            if not self._buf:
                return []
            out: List[Tuple[float, Any]] = []
            for t, item in reversed(self._buf):
                if t >= cutoff:
                    out.append((t, item))
                else:
                    break
            out.reverse()
            return out

    def clear(self) -> None:
        with self._lock:
            self._buf.clear()

    def __len__(self) -> int:
        with self._lock:
            return len(self._buf)


# =============================================================================
# CONFIG
# =============================================================================


@dataclass
class EMGSelection:
    """
    How to extract EMG signals from the TMSi sample matrix.

    - raw_channels: list of channel indices to keep (0-based)
    - differential_pairs: list of (pos_idx, neg_idx) pairs (0-based)
    """
    raw_channels: Optional[List[int]] = None
    differential_pairs: Optional[List[Tuple[int, int]]] = None


@dataclass
class EMGSyncConfig:
    connection_type: str = "usb"
    sample_rate: Optional[int] = None  # None = device default
    selection: EMGSelection = field(default_factory=EMGSelection)
    prefer_counter_timing: bool = True
    poll_interval_s: float = 0.01
    buffer_max_chunks: int = 10000


@dataclass
class IMUSyncConfig:
    imu_config: IMUConfig = field(default_factory=IMUConfig)
    calibration_samples: int = 200
    flush_serial_on_start: bool = True
    resync_on_time_reset: bool = True
    time_reset_threshold_us: int = 10_000  # detect micros() jump backwards
    buffer_max_samples: int = 200000


@dataclass
class SyncConfig:
    """
    Top-level config. By default uses a monotonic clock (perf_counter).
    """
    clock: ClockFn = time.perf_counter
    enable_imu: bool = True
    enable_emg: bool = True
    imu: IMUSyncConfig = field(default_factory=IMUSyncConfig)
    emg: EMGSyncConfig = field(default_factory=EMGSyncConfig)


# =============================================================================
# EMG COUNTER TIMING UTIL
# =============================================================================


class _EMGCounterSynchronizer:
    """
    Map EMG COUNTER ticks to the acquisition clock domain.

    The legacy SDK converts raw uint32 channel values into "physical units" via:
        converted = (raw + unit_offset) * unit_gain
    For COUNTER we want the raw integer, so we invert using gain/offset if available.
    """

    def __init__(self, sample_rate_hz: float):
        self.sample_rate = float(sample_rate_hz)
        self.sample_period = 1.0 / self.sample_rate

        self.counter_gain: Optional[float] = None
        self.counter_offset: Optional[float] = None

        self.last_raw: Optional[int] = None
        self.unwrap_offset: int = 0
        self.modulus: Optional[int] = None  # 65536 or 2**32

        self.time_offset: Optional[float] = None  # offset for t = counter*period + time_offset

    def set_unit_conversion(self, gain: Optional[float], offset: Optional[float]) -> None:
        self.counter_gain = None if gain is None else float(gain)
        self.counter_offset = None if offset is None else float(offset)

    def _to_raw_int(self, x_converted: float) -> Optional[int]:
        try:
            if x_converted is None or not np.isfinite(x_converted):
                return None
            x = float(x_converted)
            if self.counter_gain is not None and self.counter_offset is not None:
                g = self.counter_gain
                o = self.counter_offset
                if np.isfinite(g) and abs(g) > 0:
                    x = (x / g) - o
            return int(round(x))
        except Exception:
            return None

    def unwrap(self, raw: int) -> int:
        if self.last_raw is None:
            self.last_raw = raw
            return raw

        if self.modulus is None:
            if raw > 65535 or self.last_raw > 65535:
                self.modulus = 2**32
            else:
                if raw < self.last_raw and (self.last_raw - raw) > 1000:
                    self.modulus = 65536

        if self.modulus == 65536:
            if raw < self.last_raw:
                self.unwrap_offset += 65536
        elif self.modulus == 2**32:
            if raw < self.last_raw and (self.last_raw - raw) > (2**31):
                self.unwrap_offset += 2**32

        self.last_raw = raw
        return raw + self.unwrap_offset

    def observe_chunk(self, samples: np.ndarray, counter_idx: int, receive_t: float) -> Tuple[Optional[int], Optional[np.ndarray]]:
        """
        Return (counter_raw_last, sample_timestamps) or (None, None) if counter invalid.
        """
        n = int(samples.shape[0])
        raw_last = self._to_raw_int(samples[-1, counter_idx])
        if raw_last is None:
            return None, None

        c_last = self.unwrap(raw_last)

        # Initialize mapping using the first observed chunk:
        # anchor last sample close to receive_t (USB latency is small compared to seconds-scale errors)
        new_off = receive_t - (c_last * self.sample_period)
        if self.time_offset is None:
            self.time_offset = new_off
        else:
            # small adaptation to track drift/jitter
            self.time_offset = 0.995 * self.time_offset + 0.005 * float(new_off)

        # Per-sample counter values: last counter corresponds to last sample
        c0 = c_last - (n - 1)
        sample_idx = np.arange(n, dtype=np.float64)
        sample_t = (c0 + sample_idx) * self.sample_period + float(self.time_offset)
        return raw_last, sample_t


# =============================================================================
# MAIN MANAGER
# =============================================================================


class SynchronizedAcquisition:
    """
    Threaded synchronized acquisition for IMU + EMG.

    Typical usage:
        acq = SynchronizedAcquisition(SyncConfig(...))
        acq.start()
        ...
        acq.stop()

    Buffers:
        acq.imu_buffer: TimestampedBuffer of IMUSample
        acq.emg_buffer: TimestampedBuffer of EMGChunk
    """

    def __init__(
        self,
        config: Optional[SyncConfig] = None,
        on_imu: Optional[Callable[[IMUSample], None]] = None,
        on_emg: Optional[Callable[[EMGChunk], None]] = None,
    ):
        self.config = config or SyncConfig()
        self.clock: ClockFn = self.config.clock

        self.on_imu = on_imu
        self.on_emg = on_emg

        self.imu_buffer = TimestampedBuffer(maxlen=self.config.imu.buffer_max_samples)
        self.emg_buffer = TimestampedBuffer(maxlen=self.config.emg.buffer_max_chunks)

        self._start_event = threading.Event()
        self._stop_event = threading.Event()

        self._imu_thread: Optional[threading.Thread] = None
        self._emg_thread: Optional[threading.Thread] = None
        self._imu_ready = threading.Event()
        self._emg_ready = threading.Event()

        self._imu_device: Optional[IMUDevice] = None
        self._emg_device: Optional[EMGDevice] = None

        self._errors: List[BaseException] = []

    @property
    def errors(self) -> List[BaseException]:
        return list(self._errors)

    def start(self, ready_timeout_s: float = 120.0) -> None:
        """Initialize devices, then start both streams on a shared start event."""
        self._stop_event.clear()
        self._start_event.clear()
        self.imu_buffer.clear()
        self.emg_buffer.clear()
        self._errors.clear()
        self._imu_ready.clear()
        self._emg_ready.clear()

        if self.config.enable_imu:
            self._imu_thread = threading.Thread(target=self._run_imu, name="Sync-IMU", daemon=True)
            self._imu_thread.start()
        else:
            self._imu_ready.set()

        if self.config.enable_emg:
            self._emg_thread = threading.Thread(target=self._run_emg, name="Sync-EMG", daemon=True)
            self._emg_thread.start()
        else:
            self._emg_ready.set()

        t0 = self.clock()
        while True:
            if self._imu_ready.is_set() and self._emg_ready.is_set():
                break
            if (self.clock() - t0) > ready_timeout_s:
                raise TimeoutError("Timed out waiting for acquisition threads to become ready")
            time.sleep(0.05)

        # Start both streams
        self._start_event.set()

    def stop(self, join_timeout_s: float = 5.0) -> None:
        """Request stop and join threads."""
        self._stop_event.set()

        if self._imu_thread and self._imu_thread.is_alive():
            self._imu_thread.join(timeout=join_timeout_s)
        if self._emg_thread and self._emg_thread.is_alive():
            self._emg_thread.join(timeout=join_timeout_s)

        # Best-effort cleanup
        try:
            if self._emg_device is not None:
                try:
                    self._emg_device.stop_acquisition()
                except Exception:
                    pass
                try:
                    self._emg_device.disconnect()
                except Exception:
                    pass
        finally:
            self._emg_device = None

        try:
            if self._imu_device is not None:
                try:
                    self._imu_device.close()
                except Exception:
                    pass
        finally:
            self._imu_device = None

    def __enter__(self) -> "SynchronizedAcquisition":
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.stop()

    # -------------------------
    # IMU thread
    # -------------------------

    def _run_imu(self) -> None:
        last_t_us: Optional[int] = None
        first_sample = True
        time_offset: Optional[float] = None

        try:
            imu_cfg = self.config.imu
            self._imu_device = IMUDevice(config=imu_cfg.imu_config)
            self._imu_device.connect()

            # Calibration (optional but recommended)
            self._imu_device.calibrate(samples=imu_cfg.calibration_samples)
            self._imu_ready.set()

            # Wait for sync start
            self._start_event.wait()

            # Flush serial backlog so the first sample is fresh
            if imu_cfg.flush_serial_on_start and getattr(self._imu_device, "ser", None) is not None:
                try:
                    self._imu_device.ser.reset_input_buffer()
                    time.sleep(0.01)
                except Exception:
                    pass

            # Stream loop
            for reading in self._imu_device.read_stream():
                if self._stop_event.is_set():
                    break

                recv_t = self.clock()

                # Detect Arduino reset/reconnect (micros() jumps backwards)
                if imu_cfg.resync_on_time_reset and last_t_us is not None:
                    if reading.t_us < last_t_us and (last_t_us - reading.t_us) > imu_cfg.time_reset_threshold_us:
                        # Flush backlog and re-establish offset on next sample
                        if getattr(self._imu_device, "ser", None) is not None:
                            try:
                                self._imu_device.ser.reset_input_buffer()
                                time.sleep(0.01)
                            except Exception:
                                pass
                        first_sample = True
                        time_offset = None

                last_t_us = reading.t_us

                if first_sample:
                    time_offset = recv_t - (reading.t_us / 1_000_000.0)
                    first_sample = False

                if time_offset is None:
                    continue

                t_hw = reading.t_us / 1_000_000.0
                t_sync = t_hw + float(time_offset)

                sample = IMUSample(t=t_sync, reading=reading, t_hardware_s=t_hw)
                self.imu_buffer.append(t_sync, sample)
                if self.on_imu:
                    self.on_imu(sample)

        except BaseException as e:
            self._errors.append(e)
            self._imu_ready.set()
        finally:
            try:
                if self._imu_device is not None:
                    self._imu_device.close()
            except Exception:
                pass

    # -------------------------
    # EMG thread
    # -------------------------

    def _run_emg(self) -> None:
        try:
            emg_cfg = self.config.emg
            self._emg_device = EMGDevice(connection_type=emg_cfg.connection_type)
            info = self._emg_device.connect()

            if emg_cfg.sample_rate is not None:
                self._emg_device.configure_acquisition(sample_rate=emg_cfg.sample_rate)

            sr = float(self._emg_device.sample_rate)
            sample_period = 1.0 / sr

            # Counter channel discovery
            counter_idx: Optional[int] = None
            counter_sync: Optional[_EMGCounterSynchronizer] = None
            if emg_cfg.prefer_counter_timing:
                for i, ch in enumerate(self._emg_device.channels):
                    nm = (ch.get_channel_name() or "").upper()
                    if nm == "COUNTER" or "COUNTER" in nm:
                        counter_idx = i
                        counter_sync = _EMGCounterSynchronizer(sample_rate_hz=sr)
                        # Cache unit conversion, if present
                        try:
                            gain = ch.get_channel_unit_gain()
                        except Exception:
                            gain = None
                        try:
                            off = ch.get_channel_unit_offset()
                        except Exception:
                            off = None
                        counter_sync.set_unit_conversion(gain=gain, offset=off)
                        break

            self._emg_ready.set()

            # Wait for sync start
            self._start_event.wait()

            self._emg_device.start_acquisition()

            while not self._stop_event.is_set():
                samples = self._emg_device.get_samples(blocking=False)
                if samples is None or len(samples) == 0:
                    time.sleep(emg_cfg.poll_interval_s)
                    continue

                recv_t = self.clock()
                n = int(samples.shape[0])

                # Build per-sample timestamps
                counter_raw_last: Optional[int] = None
                sample_t: Optional[np.ndarray] = None
                if counter_idx is not None and counter_sync is not None:
                    counter_raw_last, sample_t = counter_sync.observe_chunk(samples, counter_idx, recv_t)

                if sample_t is None:
                    # Fallback: place samples centered around receive time
                    # (good for real-time plotting, less accurate than counter timing)
                    t0 = recv_t - ((n - 1) * sample_period)
                    sample_t = t0 + np.arange(n, dtype=np.float64) * sample_period

                # Extract signals
                data_out: Dict[str, np.ndarray] = {}
                sel = emg_cfg.selection
                if sel.raw_channels:
                    data_out["raw"] = samples[:, sel.raw_channels]
                if sel.differential_pairs:
                    pairs = []
                    for p, q in sel.differential_pairs:
                        pairs.append(samples[:, p] - samples[:, q])
                    data_out["pairs"] = np.stack(pairs, axis=1) if pairs else np.empty((n, 0))

                # Default if nothing selected: keep all channels
                if not data_out:
                    data_out["raw"] = samples

                t_mid = float(sample_t[(n - 1) // 2])
                chunk = EMGChunk(t=t_mid, sample_t=sample_t, data=data_out, counter_raw_last=counter_raw_last)
                self.emg_buffer.append(t_mid, chunk)
                if self.on_emg:
                    self.on_emg(chunk)

        except BaseException as e:
            self._errors.append(e)
            self._emg_ready.set()
        finally:
            try:
                if self._emg_device is not None:
                    try:
                        self._emg_device.stop_acquisition()
                    except Exception:
                        pass
                    try:
                        self._emg_device.disconnect()
                    except Exception:
                        pass
            except Exception:
                pass

