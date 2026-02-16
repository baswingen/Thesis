"""
Synchronized STM32 + EMG Signal Acquisition (single-mode test)
=============================================================

Single mode: run both acquisition devices (STM32 + EMG) and PRBS synchronization
together with PyQtGraph real-time visualization. No CLI; tune via the config block below.

Hardware:
    STM32F401 PA8 --[330R]-- Porti7 TRIG (LEMO centre)
    STM32F401 GND ---------- Porti7 TRIG (LEMO shield)

Run:
    python signal_acquisition_testing.py

Tuning: edit the CONFIG block below (duration, PRBS window, update interval, chip rate).
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

# -----------------------------------------------------------------------------
# CONFIG – edit these to tune duration and PRBS sync (no CLI)
# -----------------------------------------------------------------------------
DURATION_S: float = 30.0
PRBS_CORRELATION_WINDOW_S: float = 10.0
PRBS_UPDATE_INTERVAL_S: float = 2.0
PRBS_CHIP_RATE_HZ: float = 10.0
PRBS_VIZ_WINDOW_S: float = 10.0  # Scrolling time window for PRBS plots
STM32_PORT: Optional[str] = None
STM32_BAUD: int = 115200
EMG_CONNECTION_TYPE: str = "usb"
EMG_SAMPLE_RATE: Optional[int] = 2000
VERBOSE: bool = True
# -----------------------------------------------------------------------------

# PyQtGraph required (no matplotlib fallback)
try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtWidgets, QtCore
    _PYQTGRAPH_AVAILABLE = True
except ImportError:
    _PYQTGRAPH_AVAILABLE = False

_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.signal_acquisition import SignalAcquisition, SignalAcquisitionConfig

try:
    from src.signal_acquisition import _TMSI_AVAILABLE
except ImportError:
    _TMSI_AVAILABLE = False


def _build_config() -> SignalAcquisitionConfig:
    """Build config from module-level CONFIG constants."""
    return SignalAcquisitionConfig(
        enable_stm32=True,
        enable_emg=True,
        enable_prbs_sync=True,
        sync_mode="realtime",
        verbose=VERBOSE,
        stm32_port=STM32_PORT,
        stm32_baud=STM32_BAUD,
        emg_connection_type=EMG_CONNECTION_TYPE,
        emg_sample_rate=EMG_SAMPLE_RATE,
        prbs_correlation_window_s=PRBS_CORRELATION_WINDOW_S,
        prbs_update_interval_s=PRBS_UPDATE_INTERVAL_S,
        prbs_chip_rate_hz=PRBS_CHIP_RATE_HZ,
    )


class PRBSVisualizationWindow(QtWidgets.QWidget):
    """Real-time PRBS visualization with shared timeline (STM32 time)."""

    def __init__(
        self,
        acq: SignalAcquisition,
        duration_s: Optional[float] = None,
    ):
        super().__init__()
        self.acq = acq
        self.duration_s = duration_s
        self.start_time = time.perf_counter()
        self.metrics_plots_enabled = True
        self.prbs_window_s = PRBS_VIZ_WINDOW_S

        self.timer: Optional[QtCore.QTimer] = None
        self.update_timer: Optional[QtCore.QTimer] = None
        self.duration_timer: Optional[QtCore.QTimer] = None

        self.history: Dict[str, List[float]] = {
            "t": [], "offset": [], "drift": [], "confidence": []
        }
        self.max_history = 200

        self._last_stm32_len: int = 0
        self._last_stm32_hash: int = 0
        self._last_emg_len: int = 0
        self._last_emg_hash: int = 0

        self.setWindowTitle("PRBS Sync – STM32 + EMG")
        self.resize(1800, 1100)
        layout = QtWidgets.QVBoxLayout()
        self.setLayout(layout)

        self.status_label = QtWidgets.QLabel("Initializing...")
        self.status_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px;")
        layout.addWidget(self.status_label)

        self.metrics_checkbox = QtWidgets.QCheckBox("Show offset/drift/confidence plots")
        self.metrics_checkbox.setChecked(True)
        self.metrics_checkbox.toggled.connect(self._on_metrics_toggled)
        layout.addWidget(self.metrics_checkbox)

        self.graphics_layout = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphics_layout)

        self.plot_stm32 = self.graphics_layout.addPlot(row=0, col=0, title="STM32 PRBS (STM32 time)")
        self.plot_stm32.setLabel("left", "Level")
        self.plot_stm32.setLabel("bottom", "Time", units="s")
        self.plot_stm32.setYRange(-0.2, 1.2)
        self.plot_stm32.showGrid(x=True, y=True, alpha=0.3)
        self.curve_stm32 = self.plot_stm32.plot(pen=pg.mkPen(color="#2ecc71", width=2))

        self.plot_emg = self.graphics_layout.addPlot(row=1, col=0, title="Porti7 PRBS (same timeline)")
        self.plot_emg.setLabel("left", "Level")
        self.plot_emg.setLabel("bottom", "Time", units="s")
        self.plot_emg.setYRange(-0.2, 1.2)
        self.plot_emg.showGrid(x=True, y=True, alpha=0.3)
        self.curve_emg = self.plot_emg.plot(pen=pg.mkPen(color="#3498db", width=2))
        self.plot_emg.setXLink(self.plot_stm32)

        self.plot_offset = self.graphics_layout.addPlot(row=2, col=0, title="Clock Offset")
        self.plot_offset.setLabel("left", "Offset", units="ms")
        self.plot_offset.setLabel("bottom", "Time", units="s")
        self.plot_offset.showGrid(x=True, y=True, alpha=0.3)
        self.plot_offset.enableAutoRange(axis='y')
        self.curve_offset = self.plot_offset.plot(pen=pg.mkPen(color="#9b59b6", width=2))

        self.plot_drift = self.graphics_layout.addPlot(row=3, col=0, title="Clock Drift Rate")
        self.plot_drift.setLabel("left", "Drift Rate", units="ppm")
        self.plot_drift.setLabel("bottom", "Time", units="s")
        self.plot_drift.showGrid(x=True, y=True, alpha=0.3)
        self.plot_drift.enableAutoRange(axis='y')
        self.curve_drift = self.plot_drift.plot(pen=pg.mkPen(color="#e74c3c", width=2))

        self.plot_confidence = self.graphics_layout.addPlot(row=4, col=0, title="Correlation Confidence")
        self.plot_confidence.setLabel("left", "Confidence")
        self.plot_confidence.setLabel("bottom", "Time", units="s")
        self.plot_confidence.setYRange(0, 1.1)
        self.plot_confidence.showGrid(x=True, y=True, alpha=0.3)
        self.curve_confidence = self.plot_confidence.plot(pen=pg.mkPen(color="#f39c12", width=2))

        for curve in [self.curve_offset, self.curve_drift, self.curve_confidence]:
            curve.setDownsampling(auto=True, method='peak')
            curve.setClipToView(True)
        for curve in [self.curve_stm32, self.curve_emg]:
            curve.setClipToView(True)

        # Single timer for all viz updates (~30 fps)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._on_viz_tick)
        self.timer.start(33)

        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self._trigger_sync_update)
        self.update_timer.start(1000)

        if self.duration_s is not None:
            self.duration_timer = QtCore.QTimer()
            self.duration_timer.timeout.connect(self._check_duration)
            self.duration_timer.start(1000)

    def _trigger_sync_update(self) -> None:
        if self.acq is not None:
            self.acq.trigger_prbs_update()

    def _on_metrics_toggled(self, enabled: bool) -> None:
        self.metrics_plots_enabled = bool(enabled)
        if self.metrics_plots_enabled:
            self.plot_offset.show()
            self.plot_drift.show()
            self.plot_confidence.show()
            if self.update_timer is not None and not self.update_timer.isActive():
                self.update_timer.start(1000)
        else:
            self.plot_offset.hide()
            self.plot_drift.hide()
            self.plot_confidence.hide()
            if self.update_timer is not None and self.update_timer.isActive():
                self.update_timer.stop()

    def _check_duration(self) -> None:
        if self.duration_s is not None:
            elapsed = time.perf_counter() - self.start_time
            if elapsed >= self.duration_s:
                if VERBOSE:
                    print(f"\n[VIZ] Duration {self.duration_s:.0f}s reached, closing...")
                self.close()

    def _on_viz_tick(self) -> None:
        """Single tick: update sync metrics and PRBS plots (shared timeline)."""
        self._update_sync_plots()
        self._update_prbs_plots()

    @staticmethod
    def _make_step_data_from_x(x_base: np.ndarray, y_base: np.ndarray):
        n = len(y_base)
        if n == 0:
            return np.empty(0, dtype=float), np.empty(0, dtype=float)
        x = np.empty(2 * n, dtype=float)
        y = np.empty(2 * n, dtype=float)
        x[0::2] = x_base
        if n > 1:
            x[1::2] = np.concatenate((x_base[1:], [x_base[-1] + (x_base[-1] - x_base[-2])]))
        else:
            x[1::2] = x_base + 1.0 / 500.0
        y[0::2] = y_base
        y[1::2] = y_base
        return x, y

    def _update_sync_plots(self) -> None:
        if self.acq is None or not self.metrics_plots_enabled:
            return
        sync_state = self.acq.get_sync_state()
        if sync_state is not None:
            elapsed = time.perf_counter() - self.start_time
            self.history["t"].append(elapsed)
            self.history["offset"].append(sync_state.offset_ms)
            self.history["drift"].append(sync_state.drift_rate_ppm)
            self.history["confidence"].append(sync_state.confidence)
            for key in ("t", "offset", "drift", "confidence"):
                self.history[key] = self.history[key][-self.max_history:]
            if len(self.history["t"]) > 1:
                t_arr = np.array(self.history["t"])
                self.curve_offset.setData(t_arr, self.history["offset"])
                self.curve_drift.setData(t_arr, self.history["drift"])
                self.curve_confidence.setData(t_arr, self.history["confidence"])
            status_color = self._status_color(sync_state)
            self.status_label.setText(
                f"Offset: {sync_state.offset_ms:+.2f} ms | "
                f"Drift: {sync_state.drift_rate_ppm:+.1f} ppm | "
                f"Confidence: {sync_state.confidence:.3f}"
            )
            self.status_label.setStyleSheet(
                f"font-size: 14px; font-weight: bold; padding: 10px; "
                f"background-color: {status_color}; color: white;"
            )
        else:
            self.status_label.setText("Waiting for sync data...")
            self.status_label.setStyleSheet(
                "font-size: 14px; font-weight: bold; padding: 10px; "
                "background-color: #95a5a6; color: white;"
            )

    def _update_prbs_plots(self) -> None:
        # offset_ms from sync: negative when EMG is delayed; we use delay_emg_s = -offset_ms/1000
        # so newest EMG is at t_ref - delay_emg_s (to the left when Porti7 receives later).
        if self.acq is None:
            return
        try:
            t_end = None
            estimator = getattr(self.acq, "sync_delay_estimator", None)
            chip_rate_hz = float(estimator.chip_rate_hz) if estimator is not None else PRBS_CHIP_RATE_HZ
            chip_period_s = 1.0 / chip_rate_hz

            stm32_snapshot = self.acq.get_stm32_snapshot(n=1200)
            stm32_t = stm32_snapshot.get("t_sec")
            stm32_pc = stm32_snapshot.get("pc_time")
            stm32_tick = stm32_snapshot.get("prbs_tick")
            stm32_prbs = stm32_snapshot.get("prbs_level")
            use_host_time = (
                stm32_t is not None
                and stm32_pc is not None
                and len(stm32_t) == len(stm32_pc)
            )
            if (
                stm32_t is not None
                and len(stm32_t) > 0
                and not use_host_time
                and not getattr(self, "_warned_no_pc_time", False)
            ):
                self._warned_no_pc_time = True
                print("[VIZ] PRBS: pc_time not in snapshot, using STM32 time (alignment may drift)")
            if (
                stm32_t is not None and stm32_tick is not None and stm32_prbs is not None
                and len(stm32_t) > 0 and len(stm32_t) == len(stm32_tick) == len(stm32_prbs)
            ):
                valid = np.isfinite(stm32_t) & np.isfinite(stm32_tick) & np.isfinite(stm32_prbs)
                if use_host_time and stm32_pc is not None:
                    valid = valid & np.isfinite(stm32_pc)
                t_raw = np.asarray(stm32_t[valid], dtype=float)
                tick_raw = np.asarray(stm32_tick[valid], dtype=np.int64)
                prbs_raw = (np.asarray(stm32_prbs[valid], dtype=float) > 0.5).astype(float)
                if use_host_time and stm32_pc is not None:
                    t_axis = np.asarray(stm32_pc[valid], dtype=float)
                else:
                    t_axis = t_raw
                if len(tick_raw) > 0:
                    chip_change = np.empty(len(tick_raw), dtype=bool)
                    chip_change[0] = True
                    chip_change[1:] = tick_raw[1:] != tick_raw[:-1]
                    t_vals = t_axis[chip_change]
                    y_vals = prbs_raw[chip_change]
                else:
                    t_vals = np.empty(0, dtype=float)
                    y_vals = np.empty(0, dtype=float)
                if len(t_vals) > 0:
                    t_end = float(t_vals[-1])
                if len(t_vals) > 0:
                    h = hash(y_vals.data.tobytes())
                else:
                    h = 0
                if h != self._last_stm32_hash or len(y_vals) != self._last_stm32_len:
                    self._last_stm32_hash = h
                    self._last_stm32_len = len(y_vals)
                    sx, sy = self._make_step_data_from_x(t_vals, y_vals)
                    self.curve_stm32.setData(sx, sy)

            if estimator is None:
                if t_end is not None:
                    self.plot_stm32.setXRange(max(0.0, t_end - self.prbs_window_s), t_end + 0.05, padding=0)
                return

            _, emg_chips = estimator.get_chip_streams_for_display(max_chips=200, apply_alignment=False)

            if emg_chips is not None and len(emg_chips) > 0:
                sync_state = self.acq.get_sync_state()
                # EMG delay in seconds (positive when EMG is delayed); offset_ms is negative when delayed
                delay_emg_s = (
                    -sync_state.offset_ms / 1000.0 if sync_state is not None else 0.0
                )
                if t_end is None:
                    t_end = time.perf_counter() - self.start_time
                t_ref = t_end
                n_emg = len(emg_chips)
                t_emg = np.array([
                    t_ref - delay_emg_s - (n_emg - 1 - i) * chip_period_s
                    for i in range(n_emg)
                ], dtype=float)
                h = hash(emg_chips.data.tobytes())
                if h != self._last_emg_hash or n_emg != self._last_emg_len:
                    self._last_emg_hash = h
                    self._last_emg_len = n_emg
                    ex, ey = self._make_step_data_from_x(t_emg, emg_chips.astype(float))
                    self.curve_emg.setData(ex, ey)

            if t_end is None:
                t_end = time.perf_counter() - self.start_time
            self.plot_stm32.setXRange(max(0.0, t_end - self.prbs_window_s), t_end + 0.05, padding=0)
        except Exception as exc:
            if not hasattr(self, "_last_prbs_err") or self._last_prbs_err != str(exc):
                print(f"[VIZ] PRBS plot error: {exc}")
                self._last_prbs_err = str(exc)

    @staticmethod
    def _status_color(sync_state) -> str:
        c = sync_state.confidence
        d = abs(sync_state.drift_rate_ppm)
        if c > 0.6 and d < 1000:
            return "#27ae60"
        if c > 0.3 or d < 3000:
            return "#f39c12"
        return "#e74c3c"

    def closeEvent(self, event) -> None:
        for attr in ("timer", "update_timer", "duration_timer"):
            t = getattr(self, attr, None)
            if t is not None:
                t.stop()
        event.accept()


def run() -> None:
    """Single mode: STM32 + EMG + PRBS sync with PyQtGraph. Runs for DURATION_S then exits."""
    if not _PYQTGRAPH_AVAILABLE:
        print("\nERROR: PyQtGraph is required. Install with: pip install pyqtgraph PyQt6")
        sys.exit(1)
    if not _TMSI_AVAILABLE:
        print("\nERROR: TMSi SDK not available. Cannot run acquisition.")
        sys.exit(1)

    config = _build_config()
    if VERBOSE:
        print("\n" + "=" * 60)
        print("STM32 + EMG + PRBS sync (single-mode test)")
        print("=" * 60)
        print(f"  Duration: {DURATION_S:.0f} s. Window closes automatically.\n")

    acq = SignalAcquisition(config)
    acq.start()
    time.sleep(1.0)

    stm32_count_start = acq.stm32_reader.sample_count if acq.stm32_reader else 0
    start_wall = time.perf_counter()

    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)
    window = PRBSVisualizationWindow(acq, DURATION_S)
    window.show()

    try:
        app.exec() if hasattr(app, "exec") else app.exec_()
    except KeyboardInterrupt:
        if VERBOSE:
            print("\n[MAIN] Interrupted")
    finally:
        acq.stop()

    if VERBOSE:
        elapsed = time.perf_counter() - start_wall
        if acq.stm32_reader is not None:
            n = acq.stm32_reader.sample_count - stm32_count_start
            rate = n / elapsed if elapsed > 0.5 else 0
            print(f"\n  [SUMMARY] STM32: {rate:.1f} Hz, {acq.stm32_reader.sample_count} samples")
        if acq.emg_thread is not None:
            st = acq.emg_thread.get_stats()
            print(f"  [SUMMARY] EMG: {st['rate']:.0f} Hz, {st['samples']} samples")
        ss = acq.get_sync_state()
        if ss is not None:
            print(f"  [SUMMARY] PRBS: offset={ss.offset_ms:+.2f} ms, confidence={ss.confidence:.3f}")


if __name__ == "__main__":
    run()
