"""
signal_buffer.py
================

Thread-safe rolling buffer that:
  1. Receives real-time EMG chunks and STM32 samples.
  2. Applies PRBS-Kalman delay alignment (interpolates STM32 onto EMG grid).
  3. Extracts the 26-channel array expected by the CNN-LSTM:
       - 8 differential EMG channels  (matching EMG_CHANNEL_CONFIG)
       - 18 IMU channels              (matching CHANNEL_CONFIG imu_channels)

Channel layout (must exactly match training pipeline in model/data_loader.py):
  Col  0: Anterior Deltoid    (BIP ch1 − ch2)
  Col  1: Lateral Deltoid     (BIP ch3 − ch4)
  Col  2: Posterior Deltoid   (BIP ch5 − ch6)
  Col  3: Triceps Brachii     (BIP ch7 − ch8)
  Col  4: Biceps Brachii      (BIP ch17, single)
  Col  5: Brachioradialis     (BIP ch18, single)
  Col  6: FCU                 (BIP ch19, single)
  Col  7: ECR                 (BIP ch20, single)
  Col  8: ax1
  Col  9: ay1
  Col 10: az1
  Col 11: roll_rad1   (yaw1 from STM32 converted °→ rad; note: STM32 field order yaw/pitch/roll)
  Col 12: pitch_rad1
  Col 13: yaw_rad1
  Col 14: ax2
  Col 15: ay2
  Col 16: az2
  Col 17: roll_rad2
  Col 18: pitch_rad2
  Col 19: yaw_rad2
  Col 20: ax_diff    (ax1 − ax2)
  Col 21: ay_diff
  Col 22: az_diff
  Col 23: roll_rad_diff
  Col 24: pitch_rad_diff
  Col 25: yaw_rad_diff
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import List, Optional, Tuple

import numpy as np

import sys
from pathlib import Path
_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from src.stm32_emg_sync import SyncDelayEstimator

from inference.inference_config import (
    EMG_CHANNEL_MAPPING,
    IMU_CHANNEL_MAPPING,
    ZERO_FILL_MISSING,
)

# ---------------------------------------------------------------------------
# TMSi Porti7: 0-based column indices in the raw sample array
# BIP channels start at col 16 (BIP1 = col16, BIP2 = col17, ..., BIP20=col35)
# UNI channels occupy cols 0-15.
# ch1..ch32 in the config mapping refers to the first 32 non-trig channels.
# Because ch1 = index 0 in the 32-ch list, and BIP1 = col 16 in raw TMSi:
#   ch1   → index 0  → raw col 16
#   ch17  → index 16 → raw col 32
# ---------------------------------------------------------------------------

def _name_to_raw_idx(ch_name: str) -> int:
    """Map 'ch1'..'ch48' to raw TMSi column indices 0..47.
    
    Standard Porti7 mapping:
    - ch1 .. ch16  -> UNI 1 .. UNI 16 (Indices 0-15)
    - ch17 .. ch36 -> BIP 1 .. BIP 20 (Indices 16-35)
    """
    try:
        num = int(ch_name.replace("ch", ""))
        return num - 1
    except (ValueError, AttributeError):
        return -1

# Build the model projection mapping
EMG_MUSCLE_NAMES = list(EMG_CHANNEL_MAPPING.keys())

DEG2RAD = np.pi / 180.0

from src.emg_processing import EMGProcessor

# STM32 raw columns (pc_time + SampleSTM32 fields)
_STM32_YAW1_COL   = 4
_STM32_PITCH1_COL = 5
_STM32_ROLL1_COL  = 6
_STM32_AX1_COL    = 7
_STM32_AY1_COL    = 8
_STM32_AZ1_COL    = 9
_STM32_YAW2_COL   = 10
_STM32_PITCH2_COL = 11
_STM32_ROLL2_COL  = 12
_STM32_AX2_COL    = 13
_STM32_AY2_COL    = 14
_STM32_AZ2_COL    = 15

# Total number of model input channels
N_MODEL_CHANNELS = len(EMG_CHANNEL_MAPPING) + len(IMU_CHANNEL_MAPPING)


class SignalBuffer:
    """
    Thread-safe rolling buffer for real-time multi-modal signal assembly.

    Usage
    -----
    Attach callbacks from the acquisition threads:

        buf = SignalBuffer(estimator, emg_buffer_s=12.0, stm32_buffer_s=12.0)
        tmsi_thread.callback = buf.add_emg_chunk
        stm32_thread.callback = buf.add_stm32_sample

    Then query the latest aligned segment:

        segment = buf.get_segment(window_s=1.2)
        if segment is not None:
            prediction = model.predict(segment)
    """

    def __init__(
        self,
        estimator: SyncDelayEstimator,
        emg_sample_rate: int = 2000,
        emg_buffer_s: float = 12.0,
        stm32_buffer_s: float = 12.0,
        zero_fill_missing: bool = True,
    ):
        self._estimator = estimator
        self._emg_fs = emg_sample_rate
        self._zero_fill = zero_fill_missing
        self._lock = threading.Lock()

        # EMG rolling buffer: list of (t_arr, envelopes_2d) tuples
        # Stores the 8 muscles defined in the mapping
        self._emg_chunks: deque = deque()
        self._emg_max_samples = int(emg_buffer_s * emg_sample_rate)

        # Stateful EMG Processor (BP 10-500Hz, Notch 50Hz, Env 10Hz)
        # Matches training pipeline parameters in model/postprocessing.py
        self._processor = EMGProcessor(
            fs=float(emg_sample_rate),
            bandpass_low=10.0,
            bandpass_high=500.0,
            notch_freq=50.0,
            envelope_cutoff=10.0
        )

        # Causal LPF for IMU (15Hz Butterworth) 
        # Matches model/postprocessing.py (filtfilt replaced with causal lfilter for real-time)
        from scipy.signal import butter, lfilter, lfilter_zi
        nyq = 0.5 * 500.0 # ~500Hz STM32
        self._imu_b, self._imu_a = butter(4, 15.0 / nyq, btype='low')
        # Initialize filter states for causal filtering
        self._imu_stats = {} # per-column filter state (zi)

        # STM32 rolling buffer: list of (pc_time, SampleSTM32) pairs
        self._stm32_samples: deque = deque()
        self._stm32_max_samples = int(stm32_buffer_s * 500)  # ~500 Hz STM32

        # Tracking
        self._emg_total: int = 0
        self._stm32_total: int = 0
        self._emg_last_t: float = 0.0
        self._stm32_last_t: float = 0.0

        # Zero-fill status: list of muscle names that are all-zero
        self.zero_filled_channels: List[str] = []
        self._trig_idx: int = -1  # set by the caller after TMSi init

    # ------------------------------------------------------------------
    # Ingestion callbacks (called from acquisition threads)
    # ------------------------------------------------------------------

    def add_emg_chunk(self, t_arr: np.ndarray, samples_2d: np.ndarray) -> None:
        """Append an EMG chunk to the rolling buffer.

        Parameters
        ----------
        t_arr      : (N,) PC timestamps (perf_counter seconds)
        samples_2d : (N, n_channels) raw TMSi samples
        """
        if t_arr is None or samples_2d is None or len(t_arr) == 0:
            return

        # 1. Project raw TMSi columns to the 8 muscle signals
        n_muscles = len(EMG_MUSCLE_NAMES)
        n_raw = samples_2d.shape[1]
        projected = np.zeros((len(t_arr), n_muscles), dtype=np.float32)
        
        for i, name in enumerate(EMG_MUSCLE_NAMES):
            mapping = EMG_CHANNEL_MAPPING[name]
            if isinstance(mapping, tuple):
                p_idx = _name_to_raw_idx(mapping[0])
                m_idx = _name_to_raw_idx(mapping[1])
                if p_idx < n_raw and m_idx < n_raw:
                    projected[:, i] = samples_2d[:, p_idx] - samples_2d[:, m_idx]
            else:
                idx = _name_to_raw_idx(mapping)
                if idx < n_raw:
                    projected[:, i] = samples_2d[:, idx]

        # 2. Process through stateful EMG pipeline (Filters -> Envelopes)
        try:
            _, envelopes = self._processor.process(projected, return_envelope=True)
        except Exception:
            # Fallback for unexpected math errors (NaNs etc)
            envelopes = np.abs(projected)

        with self._lock:
            self._emg_chunks.append((t_arr.copy(), envelopes.copy()))
            self._emg_total += len(t_arr)
            if len(t_arr) > 0:
                self._emg_last_t = float(t_arr[-1])

            # Trim buffer
            while self._emg_total > self._emg_max_samples and len(self._emg_chunks) > 1:
                old_t, old_s = self._emg_chunks.popleft()
                self._emg_total -= len(old_t)

    def add_stm32_sample(self, pc_time: float, sample) -> None:
        """Append a single STM32 sample.

        Parameters
        ----------
        pc_time : float — PC wall clock (perf_counter)
        sample  : SampleSTM32 object
        """
        with self._lock:
            self._stm32_samples.append((pc_time, sample))
            self._stm32_total += 1
            self._stm32_last_t = pc_time

            while len(self._stm32_samples) > self._stm32_max_samples:
                self._stm32_samples.popleft()

    def set_trig_idx(self, idx: int) -> None:
        """Set the index of the TRIG channel in the raw TMSi sample array."""
        with self._lock:
            self._trig_idx = idx

    # ------------------------------------------------------------------
    # Segment retrieval
    # ------------------------------------------------------------------

    def get_segment(self, window_s: float) -> Optional[np.ndarray]:
        """
        Return the latest aligned (T, 26) segment or None if insufficient data.

        Steps
        -----
        1. Concatenate recent EMG chunks covering at least `window_s` seconds.
        2. Apply PRBS-Kalman delay: shift STM32 timestamps, interpolate onto
           EMG time grid (same approach as trial/setup_trial.py _finalize_trial_data).
        3. Extract 8 differential EMG + 18 IMU channels → (T, 26).
        4. Zero-fill any missing channels and update self.zero_filled_channels.
        """
        with self._lock:
            emg_chunks = list(self._emg_chunks)
            stm32_list = list(self._stm32_samples)
            trig_idx = self._trig_idx

        if not emg_chunks or not stm32_list:
            return None

        # --- Step 1: Concatenate EMG covering the window ---
        all_t = np.concatenate([c[0] for c in emg_chunks])
        all_s = np.concatenate([c[1] for c in emg_chunks])

        t_end = all_t[-1]
        t_start = t_end - window_s
        mask = all_t >= t_start
        if np.sum(mask) < 10:
            return None  # Not enough samples yet

        t_emg = all_t[mask]
        s_emg = all_s[mask]

        # --- Step 2: PRBS-Kalman alignment ---
        delay_ms = self._estimator.get_delay_ms()
        drift_ppm = self._estimator.get_drift_rate_ppm() if hasattr(self._estimator, 'get_drift_rate_ppm') else 0.0
        delay_s = delay_ms / 1000.0
        drift_rate = drift_ppm / 1e6

        # Build STM32 arrays covering the window (with some margin)
        stm32_arr = np.array(
            [(pc, s) for pc, s in stm32_list if pc >= t_start - 0.5],
            dtype=object
        )
        if len(stm32_arr) < 4:
            return None

        pc_times = np.array([float(row[0]) for row in stm32_arr])
        samples_stm32 = [row[1] for row in stm32_arr]

        # Build (N_stm32, 22) raw STM32 matrix (same layout as trial logger)
        stm32_mat = np.zeros((len(samples_stm32), 22), dtype=np.float64)
        for i, s in enumerate(samples_stm32):
            stm32_mat[i] = [
                pc_times[i], s.t_ms, s.imu1_ok, s.imu2_ok,
                s.yaw1, s.pitch1, s.roll1, s.ax1, s.ay1, s.az1,
                s.yaw2, s.pitch2, s.roll2, s.ax2, s.ay2, s.az2,
                s.keys_mask, s.keys_rise, s.keys_fall,
                s.prbs_tick, s.prbs_lvl, s.in_mark,
            ]

        # Apply Kalman delay shift to STM32 timestamps
        t_ref = t_emg[0]
        dt_stm32 = pc_times - t_ref
        t_stm32_aligned = pc_times + delay_s + dt_stm32 * drift_rate

        # Interpolate each STM32 column onto the EMG timestamp grid
        stm32_interp = np.zeros((len(t_emg), 22), dtype=np.float64)
        for col in range(22):
            stm32_interp[:, col] = np.interp(
                t_emg, t_stm32_aligned, stm32_mat[:, col],
                left=stm32_mat[0, col], right=stm32_mat[-1, col],
            )

        # --- Step 3: Extract 26-channel model input ---
        # 1-8: EMG Muscles (Already projected and filtered in add_emg_chunk)
        segment = np.zeros((len(t_emg), N_MODEL_CHANNELS), dtype=np.float32)
        segment[:, :8] = s_emg[:, :8]
        
        # Track zero-filled channels for health monitoring
        self.zero_filled_channels = [EMG_MUSCLE_NAMES[i] for i in range(8) if np.all(s_emg[:, i] == 0)]

        # 9-26: IMU channels from interpolated STM32 data
        # We apply the 15Hz causal LPF to smooth out interpolation/staircase artifacts
        from scipy.signal import lfilter
        
        def _lpf(data, col_name):
            if col_name not in self._imu_stats:
                from scipy.signal import lfilter_zi
                self._imu_stats[col_name] = lfilter_zi(self._imu_b, self._imu_a) * data[0]
            
            y, next_zi = lfilter(self._imu_b, self._imu_a, data, zi=self._imu_stats[col_name])
            self._imu_stats[col_name] = next_zi
            return y.astype(np.float32)

        ax1    = _lpf(stm32_interp[:, _STM32_AX1_COL], "ax1")
        ay1    = _lpf(stm32_interp[:, _STM32_AY1_COL], "ay1")
        az1    = _lpf(stm32_interp[:, _STM32_AZ1_COL], "az1")
        roll1  = _lpf(stm32_interp[:, _STM32_ROLL1_COL]  * DEG2RAD, "roll1")
        pitch1 = _lpf(stm32_interp[:, _STM32_PITCH1_COL] * DEG2RAD, "pitch1")
        yaw1   = _lpf(stm32_interp[:, _STM32_YAW1_COL]   * DEG2RAD, "yaw1")

        ax2    = _lpf(stm32_interp[:, _STM32_AX2_COL], "ax2")
        ay2    = _lpf(stm32_interp[:, _STM32_AY2_COL], "ay2")
        az2    = _lpf(stm32_interp[:, _STM32_AZ2_COL], "az2")
        roll2  = _lpf(stm32_interp[:, _STM32_ROLL2_COL]  * DEG2RAD, "roll2")
        pitch2 = _lpf(stm32_interp[:, _STM32_PITCH2_COL] * DEG2RAD, "pitch2")
        yaw2   = _lpf(stm32_interp[:, _STM32_YAW2_COL]   * DEG2RAD, "yaw2")

        segment[:,  8] = ax1
        segment[:,  9] = ay1
        segment[:, 10] = az1
        segment[:, 11] = roll1
        segment[:, 12] = pitch1
        segment[:, 13] = yaw1
        segment[:, 14] = ax2
        segment[:, 15] = ay2
        segment[:, 16] = az2
        segment[:, 17] = roll2
        segment[:, 18] = pitch2
        segment[:, 19] = yaw2
        
        # Differential IMU (S2 - S1) matching elbow joint relative kinematics
        segment[:, 20] = ax2 - ax1
        segment[:, 21] = ay2 - ay1
        segment[:, 22] = az2 - az1
        segment[:, 23] = roll2  - roll1
        segment[:, 24] = pitch2 - pitch1
        segment[:, 25] = yaw2   - yaw1

        # Replace NaN/Inf with zero (safety)
        np.nan_to_num(segment, copy=False, nan=0.0, posinf=0.0, neginf=0.0)

        return segment

    # ------------------------------------------------------------------
    # Status helpers
    # ------------------------------------------------------------------

    @property
    def emg_rate_hz(self) -> float:
        """Approximate EMG sample rate based on buffer contents."""
        with self._lock:
            if len(self._emg_chunks) < 2:
                return 0.0
            t0 = self._emg_chunks[0][0][0]
            t1 = self._emg_chunks[-1][0][-1]
            dt = t1 - t0
            if dt <= 0:
                return 0.0
            return self._emg_total / dt

    @property
    def stm32_rate_hz(self) -> float:
        """Approximate STM32 sample rate."""
        with self._lock:
            if len(self._stm32_samples) < 2:
                return 0.0
            t0 = self._stm32_samples[0][0]
            t1 = self._stm32_samples[-1][0]
            dt = t1 - t0
            if dt <= 0:
                return 0.0
            return len(self._stm32_samples) / dt

    @property
    def n_emg_samples(self) -> int:
        with self._lock:
            return self._emg_total

    @property
    def n_stm32_samples(self) -> int:
        with self._lock:
            return len(self._stm32_samples)
