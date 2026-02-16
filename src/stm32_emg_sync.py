"""
STM32-EMG PRBS synchronization module
======================================

PRBS-based synchronization between STM32 (internal clock) and Porti7 EMG TRIG.
Supports two integration modes:
- Real-time: Kalman-smoothed delay estimation during live acquisition
- Postprocessing: Delay signal (t_s, delay_ms) for offline alignment

Hardware: STM32 PA8 -> Porti7 TRIG (10 Hz chip rate, PRBS-15).
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from collections import deque
from typing import List, Optional, Tuple

import numpy as np

try:
    from scipy import signal as sp_signal

    _SCIPY_AVAILABLE = True
except Exception:
    _SCIPY_AVAILABLE = False

# =============================================================================
# Constants
# =============================================================================

CHIP_RATE_HZ = 10
EMG_SAMPLE_RATE = 2000
SAMPLES_PER_CHIP = EMG_SAMPLE_RATE // CHIP_RATE_HZ  # 200
MIN_OVERLAP_RATIO = 0.9
DEFAULT_SYNC_WINDOW_S = 10.0
DEFAULT_SYNC_STEP_S = 0.1


# =============================================================================
# Data classes
# =============================================================================


@dataclass
class SyncDelayResult:
    """Result of a single sync delay computation."""

    delay_ms: float
    delay_chips: int
    correlation_peak: float
    confidence: float
    lag_series_chips: np.ndarray
    lag_series_t_s: np.ndarray
    polarity: str = "normal"  # or "inverted"


@dataclass
class DelaySignal:
    """Delay signal for postprocessing: (t_s, delay_ms) over time."""

    t_s: np.ndarray
    delay_ms: np.ndarray


# =============================================================================
# Pure functions
# =============================================================================


def extract_trig_bits(
    raw: np.ndarray, is_status: bool = False
) -> Tuple[np.ndarray, str, int, dict]:
    """Extract binary TRIG signal from raw STATUS/TRIG channel data.

    Args:
        raw: Raw channel data (float64 from TMSi SDK).
        is_status: If True, this is a STATUS register channel.

    Returns:
        (signal, method_name, transition_count, all_method_stats)
    """
    raw_int = raw.astype(np.int64)
    bit0 = (raw_int & 1).astype(np.float64)
    lo, hi = np.nanmin(raw), np.nanmax(raw)
    threshold = (raw > (lo + hi) / 2.0).astype(np.float64) if (hi - lo) > 1e-9 else bit0
    nonzero = (raw_int != 0).astype(np.float64)
    candidates = {"bit0": bit0, "threshold": threshold, "nonzero": nonzero}

    method_stats: dict = {}
    for name, sig in candidates.items():
        method_stats[name] = int(np.sum(np.abs(np.diff(sig))))

    MIN_PLAUSIBLE_TRANSITIONS = 100
    if is_status and method_stats["bit0"] >= MIN_PLAUSIBLE_TRANSITIONS:
        return bit0, "bit0", method_stats["bit0"], method_stats

    best_name = "bit0"
    best_sig = bit0
    best_transitions = -1
    for name, sig in candidates.items():
        if method_stats[name] > best_transitions:
            best_name = name
            best_sig = sig
            best_transitions = method_stats[name]
    return best_sig, best_name, best_transitions, method_stats


def downsample_to_chip_rate(
    trig: np.ndarray,
    samples_per_chip: Optional[int] = None,
    chip_rate_hz: float = CHIP_RATE_HZ,
) -> np.ndarray:
    """Downsample TRIG to chip rate by averaging fixed-size blocks.

    Uses samples_per_chip from actual EMG sample rate when provided,
    so block boundaries match real time and avoid drift.
    """
    spc = samples_per_chip if samples_per_chip is not None else SAMPLES_PER_CHIP
    n_chips = len(trig) // spc
    out = np.empty(n_chips, dtype=np.float64)
    for k in range(n_chips):
        block = trig[k * spc : (k + 1) * spc]
        out[k] = 1.0 if np.mean(block) > 0.5 else 0.0
    return out


def stm32_samples_to_chipstream(samples: List) -> np.ndarray:
    """Convert STM32 samples to chip stream using prbs_lvl directly.

    One chip is emitted each time prbs_tick changes.
    Vectorised with numpy to avoid a Python for-loop over 10 000+ samples.
    """
    if not samples:
        return np.array([], dtype=np.float64)

    # Build numpy arrays from the sample list (one pass through Python)
    ticks = np.array([s.prbs_tick for s in samples], dtype=np.int64)
    lvls = np.array([s.prbs_lvl for s in samples], dtype=np.float64)

    # Indices where tick changes (plus the first sample)
    change_mask = np.empty(len(ticks), dtype=np.bool_)
    change_mask[0] = True
    change_mask[1:] = ticks[1:] != ticks[:-1]

    return lvls[change_mask].copy()


def cross_correlate_fullstream(
    signal_a: np.ndarray,
    signal_b: np.ndarray,
    min_overlap_ratio: float = MIN_OVERLAP_RATIO,
) -> Tuple[int, float, np.ndarray, np.ndarray, int]:
    """Cross-correlate two chip streams. Returns (lag, peak, corr, lags, overlap_at_peak)."""
    return _cross_correlate_impl(signal_a, signal_b, min_overlap_ratio)


def _cross_correlate_impl(
    signal_a: np.ndarray,
    signal_b: np.ndarray,
    min_overlap_ratio: float = MIN_OVERLAP_RATIO,
) -> Tuple[int, float, np.ndarray, np.ndarray, int]:
    M, N = len(signal_a), len(signal_b)
    a = 2.0 * signal_a - 1.0
    b = 2.0 * signal_b - 1.0
    if _SCIPY_AVAILABLE:
        corr_raw = sp_signal.correlate(a, b, mode="full")
    else:
        corr_raw = np.correlate(a, b, mode="full")

    lags = np.arange(len(corr_raw)) - (M - 1)
    overlap = np.where(
        lags >= 0,
        np.minimum(M - lags, N),
        np.minimum(M, N + lags),
    )
    overlap = np.maximum(overlap.astype(np.float64), 1.0)
    corr = corr_raw / overlap

    min_overlap = max(1, int(np.ceil(min_overlap_ratio * min(M, N))))
    valid = overlap >= min_overlap
    abs_corr = np.abs(corr)
    abs_corr_valid = np.where(valid, abs_corr, 0.0)
    peak_idx = int(np.argmax(abs_corr_valid))
    lag = int(lags[peak_idx])
    peak = float(corr[peak_idx])
    overlap_at_peak = int(overlap[peak_idx])
    return lag, peak, corr, lags, overlap_at_peak


def sliding_window_lag_series(
    signal_a: np.ndarray,
    signal_b: np.ndarray,
    window_chips: int,
    step_chips: int,
    min_overlap_ratio: float = MIN_OVERLAP_RATIO,
    chip_rate_hz: float = CHIP_RATE_HZ,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Compute lag estimate over sliding windows."""
    n = min(len(signal_a), len(signal_b))
    if n < window_chips or window_chips <= 0:
        return (
            np.array([]),
            np.array([]),
            np.array([]),
            np.array([]),
        )

    starts = list(range(0, max(1, n - window_chips + 1), step_chips))
    if starts[-1] != (n - window_chips):
        starts.append(n - window_chips)

    centers_s = []
    lags = []
    peaks = []
    overlaps = []
    for s in starts:
        a_win = signal_a[s : s + window_chips]
        b_win = signal_b[s : s + window_chips]
        lag, peak, _, _, overlap = _cross_correlate_impl(
            a_win, b_win, min_overlap_ratio=min_overlap_ratio
        )
        centers_s.append((s + 0.5 * window_chips) / chip_rate_hz)
        lags.append(lag)
        peaks.append(peak)
        overlaps.append(overlap)

    return (
        np.asarray(centers_s, dtype=np.float64),
        np.asarray(lags, dtype=np.int64),
        np.asarray(peaks, dtype=np.float64),
        np.asarray(overlaps, dtype=np.int64),
    )


# =============================================================================
# Postprocessing API
# =============================================================================


def compute_sync_delay_signal(
    emg_trig_raw: np.ndarray,
    stm32_samples: List,
    emg_sample_rate: float,
    is_status: bool = False,
    sync_window_s: float = DEFAULT_SYNC_WINDOW_S,
    sync_step_s: float = DEFAULT_SYNC_STEP_S,
    min_overlap_ratio: float = MIN_OVERLAP_RATIO,
    chip_rate_hz: float = CHIP_RATE_HZ,
    return_chips: bool = False,
) -> Tuple[DelaySignal, SyncDelayResult]:
    """Compute delay signal for postprocessing.

    Returns ``(DelaySignal, SyncDelayResult)``.
    If *return_chips* is True the result tuple is extended to
    ``(DelaySignal, SyncDelayResult, emg_chips, stm32_chips)``
    so callers can reuse the chip arrays without reprocessing.
    """
    samples_per_chip = max(1, int(round(emg_sample_rate / chip_rate_hz)))
    trig_bin, _, _, _ = extract_trig_bits(emg_trig_raw, is_status=is_status)
    emg_chips = downsample_to_chip_rate(trig_bin, samples_per_chip, chip_rate_hz)
    stm32_chips = stm32_samples_to_chipstream(stm32_samples)

    n_common = min(len(emg_chips), len(stm32_chips))
    min_chips = max(50, int(round(sync_window_s * chip_rate_hz * 0.6)))
    if n_common < min_chips:
        raise ValueError(
            f"Not enough chip overlap: {n_common} chips (need >= {min_chips})"
        )

    emg_sync = emg_chips[:n_common]
    stm32_sync = stm32_chips[:n_common]
    sync_window_chips = min(n_common, int(round(sync_window_s * chip_rate_hz)))
    sync_step_chips = max(1, int(round(sync_step_s * chip_rate_hz)))

    lag_t_s, lag_series_chips, peak_series, overlap_series = sliding_window_lag_series(
        emg_sync,
        stm32_sync,
        window_chips=sync_window_chips,
        step_chips=sync_step_chips,
        min_overlap_ratio=min_overlap_ratio,
        chip_rate_hz=chip_rate_hz,
    )

    if len(lag_series_chips) == 0:
        raise ValueError("Sliding-window lag estimation failed")

    strong_mask = np.abs(peak_series) >= 0.10
    selected_idx = (
        np.where(strong_mask)[0] if np.any(strong_mask) else np.arange(len(lag_series_chips))
    )
    lag_chips = int(np.round(np.median(lag_series_chips[selected_idx])))
    representative_i = int(
        selected_idx[np.argmin(np.abs(lag_series_chips[selected_idx] - lag_chips))]
    )
    corr_peak = float(peak_series[representative_i])
    lag_ms = lag_chips / chip_rate_hz * 1000.0
    polarity = "inverted" if corr_peak < 0 else "normal"
    confidence = min(1.0, abs(corr_peak) * 1.2)

    delay_signal = DelaySignal(
        t_s=lag_t_s.copy(),
        delay_ms=(lag_series_chips.astype(np.float64) / chip_rate_hz * 1000.0),
    )

    result = SyncDelayResult(
        delay_ms=lag_ms,
        delay_chips=lag_chips,
        correlation_peak=corr_peak,
        confidence=confidence,
        lag_series_chips=lag_series_chips.copy(),
        lag_series_t_s=lag_t_s.copy(),
        polarity=polarity,
    )

    if return_chips:
        return delay_signal, result, emg_chips, stm32_chips
    return delay_signal, result


def align_emg_to_stm32(
    emg_timestamps: np.ndarray, delay_signal: DelaySignal
) -> np.ndarray:
    """Align EMG timestamps to STM32 time using delay signal.

    Returns: emg_t - delay_ms(t) / 1000
    """
    delay_at_t = np.interp(
        emg_timestamps,
        delay_signal.t_s,
        delay_signal.delay_ms,
        left=delay_signal.delay_ms[0],
        right=delay_signal.delay_ms[-1],
    )
    return emg_timestamps - delay_at_t / 1000.0


# =============================================================================
# Real-time API: SyncDelayEstimator with Kalman smoothing
# =============================================================================


class SyncDelayEstimator:
    """Real-time sync delay estimator with Kalman smoothing."""

    def __init__(
        self,
        chip_rate_hz: float = CHIP_RATE_HZ,
        emg_sample_rate: float = EMG_SAMPLE_RATE,
        sync_window_s: float = DEFAULT_SYNC_WINDOW_S,
        update_interval_s: float = 2.0,  # Kalman smoothing – infrequent correlations suffice
        sync_step_s: float = 3.0,        # coarse sliding-window step (few windows)
        min_overlap_ratio: float = MIN_OVERLAP_RATIO,
        kf_process_noise: float = 0.01,
        kf_measurement_noise: float = 1.0,
    ):
        self.chip_rate_hz = chip_rate_hz
        self.emg_sample_rate = emg_sample_rate
        self.samples_per_chip = max(1, int(round(emg_sample_rate / chip_rate_hz)))
        self.sync_window_s = sync_window_s
        self.sync_step_s = sync_step_s
        self.update_interval_s = update_interval_s
        self.min_overlap_ratio = min_overlap_ratio
        self._kf_R = kf_measurement_noise

        self._emg_trig_buf: List[Tuple[np.ndarray, np.ndarray]] = []  # (timestamps, samples)
        self._stm32_samples: List = []
        self._lock = threading.Lock()
        self._update_guard = threading.Lock()  # prevents concurrent update() threads

        # 2-state Kalman filter: [offset_ms, drift_rate_ppm]
        self._kf_state = np.array([0.0, 0.0], dtype=np.float64)
        self._kf_covariance = np.array([
            [100.0, 0.0],
            [0.0, 1000.0]
        ], dtype=np.float64)
        # Process noise covariance
        self._kf_Q = np.array([
            [kf_process_noise, 0.0],
            [0.0, 10.0]  # Drift process noise (ppm²)
        ], dtype=np.float64)
        
        self._current_delay_ms = 0.0
        self._drift_rate_ppm = 0.0
        self._last_update_time = 0.0
        self._update_count = 0
        self._last_result: Optional[SyncDelayResult] = None
        self._trig_is_status: bool = True  # Porti7 TRIG is typically STATUS/bit-field

        # Rolling chip buffers – incrementally updated as data arrives so
        # get_chip_streams_for_display() is always a cheap tail-slice.
        self._rolling_emg_chips = np.array([], dtype=np.float64)
        self._rolling_stm32_chips = np.array([], dtype=np.float64)
        self._emg_remainder = np.array([], dtype=np.float64)  # partial-chip leftovers
        self._last_stm32_tick: Optional[int] = None
        self._max_rolling_chips = int(chip_rate_hz * 60)  # ~60 s history
        self._detected_trig_bit: Optional[int] = None  # auto-detected STATUS bit
        self._lag_history_chips = deque(maxlen=7)
        self._min_confidence_for_update = 0.15
        self._max_untrusted_jump_chips = 3

    def set_trig_is_status(self, is_status: bool) -> None:
        """Set whether TRIG channel is STATUS/bit-field (True) or direct (False)."""
        self._trig_is_status = is_status

    def add_emg_trig_chunk(self, timestamps: np.ndarray, samples: np.ndarray) -> None:
        """Feed EMG TRIG chunk (per-sample timestamps and raw TRIG values)."""
        with self._lock:
            self._emg_trig_buf.append((timestamps.copy(), samples.copy()))
            self._append_emg_chips(samples)

    def add_stm32_samples(self, samples: List) -> None:
        """Feed STM32 samples (list of SampleSTM32)."""
        with self._lock:
            self._stm32_samples.extend(samples)
            self._append_stm32_chips(samples)

    # ------------------------------------------------------------------
    # Incremental chip extraction (called under _lock by add_* methods)
    # ------------------------------------------------------------------

    def _detect_trig_bit(self, raw_samples: np.ndarray) -> Optional[int]:
        """Scan all 32 bits of the STATUS register to find which carries TRIG.

        Picks the bit with the most transitions — same exhaustive scan
        that ``trig_testing.py`` uses.  Returns the bit index, or None
        if no bit shows enough transitions to be the PRBS signal.
        """
        raw_int = raw_samples.astype(np.int64)
        best_bit: Optional[int] = None
        best_trans = 1  # require >= 2 transitions to accept
        for bit in range(32):
            bit_vals = (raw_int >> bit) & 1
            trans = int(np.sum(np.abs(np.diff(bit_vals))))
            if trans > best_trans:
                best_trans = trans
                best_bit = bit
        return best_bit

    def _append_emg_chips(self, raw_samples: np.ndarray) -> None:
        """Extract chips from new EMG trig samples and append to rolling buffer.

        Accumulates **raw** samples across chunk boundaries so that each
        chip is determined from a full ``samples_per_chip`` block.

        For STATUS channels, the specific bit carrying the TRIG signal is
        auto-detected by scanning all 32 bits for the one with the most
        transitions (same approach as ``trig_testing.py``).  The detected
        bit is cached after the first successful detection.

        For analog channels, a midpoint threshold is used.
        """
        if len(raw_samples) == 0:
            return

        # Accumulate RAW samples (not extracted bits) so chip boundaries
        # are always processed from a consistent block of raw data.
        if len(self._emg_remainder) > 0:
            raw = np.concatenate([self._emg_remainder, raw_samples])
        else:
            raw = raw_samples

        spc = self.samples_per_chip
        n_complete = len(raw) // spc

        if n_complete > 0:
            usable_flat = raw[: n_complete * spc]

            # --- Auto-detect TRIG bit (STATUS channels only, runs once) ---
            if self._trig_is_status and self._detected_trig_bit is None:
                self._detected_trig_bit = self._detect_trig_bit(usable_flat)

            # --- Extract binary signal ---
            if self._detected_trig_bit is not None:
                # Use the specific bit that carries TRIG
                bits = (
                    (usable_flat.astype(np.int64) >> self._detected_trig_bit) & 1
                ).astype(np.float64)
            else:
                # Non-STATUS or detection pending: midpoint threshold
                lo, hi = float(np.nanmin(usable_flat)), float(np.nanmax(usable_flat))
                thr = (lo + hi) / 2.0 if (hi - lo) > 1e-9 else 0.5
                bits = (usable_flat > thr).astype(np.float64)

            # Majority-vote per chip block
            new_chips = (
                bits.reshape(n_complete, spc).mean(axis=1) > 0.5
            ).astype(np.float64)

            self._rolling_emg_chips = np.concatenate(
                [self._rolling_emg_chips, new_chips]
            )
            if len(self._rolling_emg_chips) > self._max_rolling_chips:
                self._rolling_emg_chips = self._rolling_emg_chips[
                    -self._max_rolling_chips :
                ]

        # Save RAW remainder for next chunk
        self._emg_remainder = raw[n_complete * spc :]

    def _append_stm32_chips(self, samples: List) -> None:
        """Extract chips from new STM32 samples and append to rolling buffer.

        One chip is emitted per ``prbs_tick`` change, matching the logic
        in :func:`stm32_samples_to_chipstream` but done incrementally so
        only the new batch is processed.
        """
        if not samples:
            return

        ticks = np.array([s.prbs_tick for s in samples], dtype=np.int64)
        lvls = np.array([s.prbs_lvl for s in samples], dtype=np.float64)

        # Detect tick transitions (each transition = new chip)
        change_mask = np.empty(len(ticks), dtype=np.bool_)
        if self._last_stm32_tick is not None:
            change_mask[0] = ticks[0] != self._last_stm32_tick
        else:
            change_mask[0] = True
        if len(ticks) > 1:
            change_mask[1:] = ticks[1:] != ticks[:-1]

        new_chips = lvls[change_mask]
        if len(new_chips) > 0:
            self._rolling_stm32_chips = np.concatenate(
                [self._rolling_stm32_chips, new_chips]
            )
            if len(self._rolling_stm32_chips) > self._max_rolling_chips:
                self._rolling_stm32_chips = self._rolling_stm32_chips[
                    -self._max_rolling_chips :
                ]
        self._last_stm32_tick = int(ticks[-1])

    def _get_buffered_data(
        self,
    ) -> Tuple[Optional[np.ndarray], Optional[List], Optional[int]]:
        with self._lock:
            if not self._emg_trig_buf or not self._stm32_samples:
                return None, None, None

            all_ts = []
            all_trig = []
            for ts, trig in self._emg_trig_buf:
                all_ts.append(ts)
                all_trig.append(trig)
            trig_raw = np.concatenate(all_trig)
            samples_per_chip = self.samples_per_chip
            stm32 = list(self._stm32_samples)

        return trig_raw, stm32, samples_per_chip

    def _is_ready(self, now: Optional[float] = None) -> bool:
        """Check whether enough data / time has elapsed for an update.

        Uses rolling chip-buffer lengths (exact chip counts) instead of
        raw-buffer size estimates.
        """
        import time

        t = now if now is not None else time.perf_counter()
        if self._last_update_time == 0.0:
            min_chips = int(self.sync_window_s * self.chip_rate_hz * 0.5)
            with self._lock:
                return (
                    len(self._rolling_emg_chips) >= min_chips
                    and len(self._rolling_stm32_chips) >= min_chips
                )
        return (t - self._last_update_time) >= self.update_interval_s

    def should_update(self, now: Optional[float] = None) -> bool:
        """Public check: ready AND no other update in progress."""
        if self._update_guard.locked():
            return False
        return self._is_ready(now)

    def update(self, now: Optional[float] = None) -> bool:
        """Run correlation and Kalman update. Returns True if update was performed.

        Uses a non-blocking guard so that only one update thread runs at a
        time – prevents CPU starvation from concurrent background threads.
        """
        # Non-blocking acquire: skip if another update is already running
        if not self._update_guard.acquire(blocking=False):
            return False

        try:
            return self._do_update(now)
        finally:
            self._update_guard.release()

    def _do_update(self, now: Optional[float] = None) -> bool:
        """Correlate the rolling chip buffers and run the Kalman update.

        Uses the pre-built rolling chip arrays (leader) so the Kalman
        filter (follower) never touches raw sample buffers.  Total cost:
        one cross-correlation on ~100 chips + 2x2 matrix math ≈ µs.
        """
        import time

        t = now if now is not None else time.perf_counter()
        if not self._is_ready(t):
            return False

        # ---- snapshot rolling chip buffers (cheap copy under lock) ----
        with self._lock:
            emg_chips = self._rolling_emg_chips.copy()
            stm32_chips = self._rolling_stm32_chips.copy()

        # Use a larger trailing span so we can estimate lag from multiple windows
        # (closer to offline robustness) while still running in real time.
        analysis_span_chips = min(
            int(2.0 * self.sync_window_s * self.chip_rate_hz),
            len(emg_chips),
            len(stm32_chips),
        )
        if analysis_span_chips < 20:
            return False

        emg_recent = emg_chips[-analysis_span_chips:]
        stm32_recent = stm32_chips[-analysis_span_chips:]

        window_chips = min(
            int(self.sync_window_s * self.chip_rate_hz),
            len(emg_recent),
            len(stm32_recent),
        )
        if window_chips < 20:
            return False

        step_chips = max(1, int(round(self.sync_step_s * self.chip_rate_hz)))

        # ---- robust lag measurement from a short sliding-window series ----
        try:
            _, lag_series, peak_series, _ = sliding_window_lag_series(
                emg_recent,
                stm32_recent,
                window_chips=window_chips,
                step_chips=step_chips,
                min_overlap_ratio=self.min_overlap_ratio,
                chip_rate_hz=self.chip_rate_hz,
            )
        except Exception:
            return False

        if len(lag_series) == 0:
            return False

        strong_mask = np.abs(peak_series) >= 0.10
        idx = np.where(strong_mask)[0] if np.any(strong_mask) else np.arange(len(lag_series))
        lag_candidates = lag_series[idx]
        peak_candidates = peak_series[idx]

        measured_lag_chips = int(np.round(np.median(lag_candidates)))
        representative_i = int(idx[np.argmin(np.abs(lag_candidates - measured_lag_chips))])
        measured_peak = float(peak_series[representative_i])
        measured_ms = measured_lag_chips / self.chip_rate_hz * 1000.0

        # Confidence combines representative-window quality and lag consistency.
        lag_std = float(np.std(lag_candidates)) if len(lag_candidates) > 1 else 0.0
        consistency = 1.0 / (1.0 + lag_std)
        confidence = min(1.0, abs(measured_peak) * 1.2 * consistency)
        if confidence < self._min_confidence_for_update:
            return False

        with self._lock:
            # Reject large jumps when confidence is only moderate.
            if self._last_result is not None:
                prev_lag = int(self._last_result.delay_chips)
                jump = abs(measured_lag_chips - prev_lag)
                if jump > self._max_untrusted_jump_chips and confidence < 0.45:
                    return False

            # Compute time step for state transition
            dt_s = t - self._last_update_time if self._last_update_time > 0 else 0.1

            # === 2-State Kalman Filter: [offset_ms, drift_rate_ppm] ===

            # State transition matrix (constant velocity model)
            F = np.array([
                [1.0, dt_s * 1000.0 / 1e6],  # offset influenced by drift
                [0.0, 1.0]                     # drift assumed constant
            ], dtype=np.float64)

            # === PREDICT ===
            x_pred = F @ self._kf_state
            P_pred = F @ self._kf_covariance @ F.T + self._kf_Q

            # === UPDATE ===
            H = np.array([[1.0, 0.0]], dtype=np.float64)
            R = self._kf_R / max(0.01, confidence)
            y_innov = measured_ms - (H @ x_pred)[0]
            S = (H @ P_pred @ H.T)[0, 0] + R
            K = (P_pred @ H.T) / S

            self._kf_state = x_pred + K.flatten() * y_innov
            self._kf_covariance = (np.eye(2, dtype=np.float64) - K @ H) @ P_pred

            # Extract smoothed estimates
            self._current_delay_ms = float(self._kf_state[0])
            self._drift_rate_ppm = float(self._kf_state[1])
            self._lag_history_chips.append(measured_lag_chips)

            self._last_update_time = t
            self._update_count += 1
            self._last_result = SyncDelayResult(
                delay_ms=measured_ms,
                delay_chips=int(measured_lag_chips),
                correlation_peak=float(measured_peak),
                confidence=confidence,
                lag_series_chips=lag_series.astype(np.int64, copy=True),
                lag_series_t_s=np.array([0.0]),
                polarity="inverted" if measured_peak < 0 else "normal",
            )

            # Trim raw buffers to prevent unbounded memory growth
            # (still appended to by add_* for backward compatibility)
            # Increased from 20 to 1000 to support longer visualization windows (~50s)
            max_raw_chunks = 1000  
            if len(self._emg_trig_buf) > max_raw_chunks:
                self._emg_trig_buf = self._emg_trig_buf[-max_raw_chunks:]
            max_stm32 = 5000  # ~10 s at 500 Hz
            if len(self._stm32_samples) > max_stm32:
                self._stm32_samples = self._stm32_samples[-max_stm32:]

        return True

    def get_delay_ms(self) -> float:
        """Return current Kalman-smoothed delay in milliseconds."""
        with self._lock:
            return self._current_delay_ms

    def get_drift_rate_ppm(self) -> float:
        """Return estimated drift rate in parts per million."""
        with self._lock:
            return self._drift_rate_ppm

    def get_result(self) -> Optional[SyncDelayResult]:
        """Return last SyncDelayResult from update, or None."""
        with self._lock:
            return self._last_result

    @property
    def emg_buf_len(self) -> int:
        """Return number of chips in the rolling EMG buffer."""
        with self._lock:
            return len(self._rolling_emg_chips)

    @property
    def stm32_buf_len(self) -> int:
        """Return number of chips in the rolling STM32 buffer."""
        with self._lock:
            return len(self._rolling_stm32_chips)

    def _align_chips_for_display(
        self, emg_chips: np.ndarray, stm32_chips: np.ndarray, max_chips: int = 500
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Time-align chip arrays using the current Kalman-smoothed lag.

        Pure numpy slicing – very cheap (no buffer reprocessing).
        """
        lag_chips = int(round(self._current_delay_ms / 1000.0 * self.chip_rate_hz))
        if len(self._lag_history_chips) > 0:
            lag_chips = int(round(float(np.median(np.asarray(self._lag_history_chips)))))

        if lag_chips < 0:
            stm32_chips = stm32_chips[-lag_chips:]
        elif lag_chips > 0:
            emg_chips = emg_chips[lag_chips:]

        n_common = min(len(stm32_chips), len(emg_chips))
        stm32_chips = stm32_chips[:n_common]
        emg_chips = emg_chips[:n_common]

        n = min(n_common, max_chips)
        if n == 0:
            return None, None
        return stm32_chips[-n:].copy(), emg_chips[-n:].copy()

    def get_chip_streams_for_display(
        self,
        max_chips: int = 500,
        is_status: Optional[bool] = None,
        apply_alignment: bool = True,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Return (stm32_chips, emg_chips) for visualization.

        Both arrays are **time-aligned** using the Kalman-smoothed lag so
        the same PRBS chip is at the same array index.  Each has at most
        *max_chips* elements, or ``None`` if insufficient data.

        Data comes from rolling chip buffers that are built incrementally
        as ``add_emg_trig_chunk`` / ``add_stm32_samples`` are called, so
        this method is always O(max_chips) – no raw-buffer reprocessing.
        """
        with self._lock:
            if len(self._rolling_stm32_chips) == 0 or len(self._rolling_emg_chips) == 0:
                return None, None
            stm32_chips = self._rolling_stm32_chips.copy()
            emg_chips = self._rolling_emg_chips.copy()

        if apply_alignment:
            return self._align_chips_for_display(emg_chips, stm32_chips, max_chips)

        n_common = min(len(stm32_chips), len(emg_chips))
        n = min(n_common, max_chips)
        if n == 0:
            return None, None
        return stm32_chips[-n:].copy(), emg_chips[-n:].copy()
