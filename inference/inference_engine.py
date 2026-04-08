"""
inference_engine.py
===================

Background thread that periodically:
  1. Checks PRBS sync confidence — skips if below threshold.
  2. Queries SignalBuffer for the latest rolling window.
  3. Runs ModelLoader.predict() on the segment.
  4. Applies an Exponential Moving Average (EMA) for temporal smoothing.
  5. Stores the result in a thread-safe deque for the dashboard to read.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Optional, Tuple, List

import numpy as np

import sys
from pathlib import Path
_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from src.stm32_emg_sync import SyncDelayEstimator
from inference.model_loader import ModelLoader
from inference.signal_buffer import SignalBuffer


class InferenceEngine(threading.Thread):
    """
    Daemon thread that runs the CNN-LSTM inference loop.

    Parameters
    ----------
    model   : ModelLoader  — loaded model wrapper
    buffer  : SignalBuffer — rolling signal buffer
    estimator : SyncDelayEstimator — for sync confidence check
    window_s  : float — rolling segment duration in seconds
    step_s    : float — re-inference interval in seconds
    ema_alpha : float — EMA smoothing factor (0=no update, 1=no history)
    sync_min_confidence : float — skip inference below this confidence
    """

    def __init__(
        self,
        model: ModelLoader,
        buffer: SignalBuffer,
        estimator: SyncDelayEstimator,
        window_s: float = 1.2,
        step_s: float = 2.0,
        ema_alpha: float = 0.4,
        sync_min_confidence: float = 0.4,
    ):
        super().__init__(name="InferenceEngine", daemon=True)
        self._model = model
        self._buffer = buffer
        self._estimator = estimator
        self._window_s = window_s
        self._step_s = step_s
        self._ema_alpha = ema_alpha
        self._sync_min_conf = sync_min_confidence

        self._running = False
        self._lock = threading.Lock()

        # Circular history of (timestamp, raw_pred, smoothed_pred)
        # 300 entries at 2 s step → 10 min of history
        self._history: deque = deque(maxlen=300)

        # Latest values (fast access without iterating deque)
        self._latest_raw: Optional[float] = None
        self._latest_smoothed: Optional[float] = None
        self._latest_t: float = 0.0
        self._latest_error: Optional[str] = None

        # EMA state
        self._ema_value: Optional[float] = None

        # Counters
        self.inference_count: int = 0
        self.skip_count: int = 0

    # ------------------------------------------------------------------
    # Thread lifecycle
    # ------------------------------------------------------------------

    def start_engine(self) -> None:
        """Start the inference loop."""
        self._running = True
        self.start()

    def stop(self) -> None:
        """Signal the thread to stop."""
        self._running = False

    def run(self) -> None:
        print(f"[InferenceEngine] Started | window={self._window_s}s | step={self._step_s}s")
        while self._running:
            t_tick = time.perf_counter()

            self._do_inference()

            # Sleep for the remainder of the step interval
            elapsed = time.perf_counter() - t_tick
            sleep_s = max(0.0, self._step_s - elapsed)
            time.sleep(sleep_s)

        print("[InferenceEngine] Stopped.")

    # ------------------------------------------------------------------
    # Core inference step
    # ------------------------------------------------------------------

    def _do_inference(self) -> None:
        """Single inference step — called once per step interval."""
        now = time.perf_counter()

        # --- 1. Check PRBS sync quality ---
        sync_result = self._estimator.get_result()
        if sync_result is None or sync_result.confidence < self._sync_min_conf:
            # Not yet synced — skip but don't record an error
            self.skip_count += 1
            return

        # --- 2. Fetch rolling segment ---
        try:
            segment = self._buffer.get_segment(self._window_s)
        except Exception as e:
            with self._lock:
                self._latest_error = f"Buffer error: {e}"
            return

        if segment is None:
            self.skip_count += 1
            return

        # --- 3. Run model ---
        try:
            raw_pred = self._model.predict(segment)
        except Exception as e:
            with self._lock:
                self._latest_error = f"Model error: {e}"
                print(f"[InferenceEngine] Predict error: {e}")
            return

        # --- 4. EMA smoothing ---
        if self._ema_value is None:
            self._ema_value = raw_pred
        else:
            self._ema_value = (
                self._ema_alpha * raw_pred
                + (1.0 - self._ema_alpha) * self._ema_value
            )
        smoothed_pred = self._ema_value

        # --- 5. Store result ---
        with self._lock:
            self._latest_raw = raw_pred
            self._latest_smoothed = smoothed_pred
            self._latest_t = now
            self._latest_error = None
            self._history.append((now, raw_pred, smoothed_pred))

        self.inference_count += 1

        print(
            f"[InferenceEngine] #{self.inference_count:04d} | "
            f"raw={raw_pred:.3f} kg | smoothed={smoothed_pred:.3f} kg | "
            f"sync_conf={sync_result.confidence:.2f}"
        )

    # ------------------------------------------------------------------
    # Thread-safe accessors for the dashboard
    # ------------------------------------------------------------------

    def get_prediction(self) -> Tuple[Optional[float], Optional[float], float]:
        """Return (raw_kg, smoothed_kg, timestamp).

        Returns (None, None, 0.0) if no prediction has been made yet.
        """
        with self._lock:
            return self._latest_raw, self._latest_smoothed, self._latest_t

    def get_history(self) -> List[Tuple[float, float, float]]:
        """Return a list of (timestamp, raw_kg, smoothed_kg) tuples."""
        with self._lock:
            return list(self._history)

    def get_latest_error(self) -> Optional[str]:
        """Return the latest error message, or None if no error."""
        with self._lock:
            return self._latest_error

    @property
    def is_synced(self) -> bool:
        """True if PRBS confidence is above threshold."""
        result = self._estimator.get_result()
        return result is not None and result.confidence >= self._sync_min_conf

    @property
    def sync_confidence(self) -> float:
        result = self._estimator.get_result()
        return result.confidence if result else 0.0

    @property
    def sync_delay_ms(self) -> float:
        return self._estimator.get_delay_ms()

    @property
    def ema_alpha(self) -> float:
        return self._ema_alpha

    @ema_alpha.setter
    def ema_alpha(self, value: float) -> None:
        self._ema_alpha = float(np.clip(value, 0.0, 1.0))
