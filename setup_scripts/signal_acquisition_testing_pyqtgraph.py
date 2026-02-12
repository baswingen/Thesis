"""
Synchronized STM32 + EMG Signal Acquisition with PyQtGraph Visualization
========================================================================

High-performance real-time visualization using PyQtGraph (10-100x faster than matplotlib).

Hardware-accelerated rendering with OpenGL support for smooth 30+ fps visualization.

Usage:
    python signal_acquisition_testing_pyqtgraph.py --mode all --duration 60

Requirements:
    pip install pyqtgraph PyQt6  # or PyQt5
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

# Ensure project root is on path
_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.signal_acquisition import (
    SignalAcquisition,
    SignalAcquisitionConfig,
    run_stm32_only,
    run_emg_only,
)

try:
    from src.signal_acquisition import _TMSI_AVAILABLE
except ImportError:
    _TMSI_AVAILABLE = False

# Try to import PyQtGraph
try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtWidgets, QtCore, QtGui
    _PYQTGRAPH_AVAILABLE = True
except ImportError:
    _PYQTGRAPH_AVAILABLE = False
    print("[WARN] PyQtGraph not available. Install with: pip install pyqtgraph PyQt6")


class PRBSVisualizationWindow(QtWidgets.QWidget):
    """High-performance real-time PRBS incoming-signal visualization."""
    
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
        self.prbs_window_s = 10.0
        
        # Initialise all timer attributes early (before any Qt timers fire)
        self.timer: Optional[QtCore.QTimer] = None
        self.prbs_timer: Optional[QtCore.QTimer] = None
        self.update_timer: Optional[QtCore.QTimer] = None
        self.duration_timer: Optional[QtCore.QTimer] = None
        
        # History buffers
        self.history: Dict[str, List[float]] = {
            "t": [],
            "offset": [],
            "drift": [],
            "confidence": []
        }
        self.max_history = 200
        
        # PRBS change-detection cache (skip redraws when data hasn't changed)
        self._last_stm32_len: int = 0
        self._last_emg_len: int = 0
        self._last_stm32_hash: int = 0
        self._last_emg_hash: int = 0
        
        # Setup UI
        self.setWindowTitle("PRBS Raw Incoming Signals + Sync Metrics")
        self.resize(1800, 1100)
        
        # Create layout
        layout = QtWidgets.QVBoxLayout()
        self.setLayout(layout)
        
        # Status label
        self.status_label = QtWidgets.QLabel("Initializing...")
        self.status_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px;")
        layout.addWidget(self.status_label)

        # Toggle for non-PRBS metric plots
        self.metrics_checkbox = QtWidgets.QCheckBox("Show offset/drift/confidence plots")
        self.metrics_checkbox.setChecked(True)
        self.metrics_checkbox.toggled.connect(self._on_metrics_toggled)
        layout.addWidget(self.metrics_checkbox)

        # Create plot widgets
        self.graphics_layout = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphics_layout)
        
        # STM32 PRBS plot (no stepMode – we build step data manually to avoid
        # the PlotDataItem / PlotCurveItem length-mismatch bug)
        self.plot_stm32 = self.graphics_layout.addPlot(row=0, col=0, title="STM32 PRBS (incoming)")
        self.plot_stm32.setLabel("left", "Level")
        self.plot_stm32.setLabel("bottom", "Time", units="s")
        self.plot_stm32.setYRange(-0.2, 1.2)
        self.plot_stm32.showGrid(x=True, y=True, alpha=0.3)
        self.curve_stm32 = self.plot_stm32.plot(pen=pg.mkPen(color="#2ecc71", width=2))
        
        # EMG PRBS plot
        self.plot_emg = self.graphics_layout.addPlot(row=1, col=0, title="Porti7 PRBS (incoming)")
        self.plot_emg.setLabel("left", "Level")
        self.plot_emg.setLabel("bottom", "Time", units="s")
        self.plot_emg.setYRange(-0.2, 1.2)
        self.plot_emg.showGrid(x=True, y=True, alpha=0.3)
        self.curve_emg = self.plot_emg.plot(pen=pg.mkPen(color="#3498db", width=2))
        self.plot_emg.setXLink(self.plot_stm32)
        
        # Offset plot
        self.plot_offset = self.graphics_layout.addPlot(row=2, col=0, title="Clock Offset")
        self.plot_offset.setLabel("left", "Offset", units="ms")
        self.plot_offset.setLabel("bottom", "Time", units="s")
        self.plot_offset.showGrid(x=True, y=True, alpha=0.3)
        self.plot_offset.enableAutoRange(axis='y')
        self.curve_offset = self.plot_offset.plot(pen=pg.mkPen(color="#9b59b6", width=2))
        
        # Drift plot
        self.plot_drift = self.graphics_layout.addPlot(row=3, col=0, title="Clock Drift Rate")
        self.plot_drift.setLabel("left", "Drift Rate", units="ppm")
        self.plot_drift.setLabel("bottom", "Time", units="s")
        self.plot_drift.showGrid(x=True, y=True, alpha=0.3)
        self.plot_drift.enableAutoRange(axis='y')
        self.curve_drift = self.plot_drift.plot(pen=pg.mkPen(color="#e74c3c", width=2))
        
        # Confidence plot
        self.plot_confidence = self.graphics_layout.addPlot(row=4, col=0, title="Correlation Confidence")
        self.plot_confidence.setLabel("left", "Confidence")
        self.plot_confidence.setLabel("bottom", "Time", units="s")
        self.plot_confidence.setYRange(0, 1.1)
        self.plot_confidence.showGrid(x=True, y=True, alpha=0.3)
        self.curve_confidence = self.plot_confidence.plot(pen=pg.mkPen(color="#f39c12", width=2))
        
        # Performance optimizations (only on time-series curves; PRBS uses
        # manually-built step data so downsampling would break the shape)
        for curve in [self.curve_offset, self.curve_drift, self.curve_confidence]:
            curve.setDownsampling(auto=True, method='peak')
            curve.setClipToView(True)
        for curve in [self.curve_stm32, self.curve_emg]:
            curve.setClipToView(True)
        
        # Timer for sync-state plots (offset/drift/confidence) – 30 fps (cheap)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_sync_plots)
        self.timer.start(33)  # ~30 fps
        
        # Timer for PRBS chip plots (STM32/Porti7) – 30 fps for smooth scrolling
        self.prbs_timer = QtCore.QTimer()
        self.prbs_timer.timeout.connect(self.update_prbs_plots)
        self.prbs_timer.start(33)  # ~30 fps for smoother real-time x-axis motion
        
        # Timer to trigger PRBS estimator updates
        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self.trigger_sync_update)
        self.update_timer.start(1000)  # Check every 1s – correlation runs every ~2s
        
        # Timer for duration check
        if self.duration_s is not None:
            self.duration_timer = QtCore.QTimer()
            self.duration_timer.timeout.connect(self.check_duration)
            self.duration_timer.start(1000)  # Check every second
    
    def trigger_sync_update(self):
        """Trigger PRBS sync update if needed."""
        if self.acq is not None:
            self.acq.trigger_prbs_update()

    def _on_metrics_toggled(self, enabled: bool):
        """Show/hide offset, drift and confidence plots."""
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
    
    def check_duration(self):
        """Check if duration has elapsed and close window if so."""
        if self.duration_s is not None:
            elapsed = time.perf_counter() - self.start_time
            if elapsed >= self.duration_s:
                print(f"\n[VIZ] Duration {self.duration_s:.0f}s reached, closing...")
                self.close()
    
    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _make_step_data(chips: np.ndarray, x_offset: float = 0.0):
        """Build step-shaped (x, y) arrays from a 1-D chip array.

        Returns arrays of length ``2 * len(chips)`` that look like a staircase
        when plotted with a normal line – no ``stepMode`` needed.
        """
        n = len(chips)
        x = np.empty(2 * n, dtype=float)
        y = np.empty(2 * n, dtype=float)
        idx = np.arange(n, dtype=float) + x_offset
        x[0::2] = idx          # left edges
        x[1::2] = idx + 1.0    # right edges
        y[0::2] = chips
        y[1::2] = chips
        return x, y

    @staticmethod
    def _make_step_data_from_x(x_base: np.ndarray, y_base: np.ndarray):
        """Build step-shaped (x, y) arrays using explicit sample times."""
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

    # ------------------------------------------------------------------
    # fast path – sync-state time series (offset / drift / confidence)
    # Called at ~30 fps.  Very cheap: just append scalars and setData.
    # ------------------------------------------------------------------
    def update_sync_plots(self):
        """Update offset / drift / confidence plots (called at ~30 fps)."""
        if self.acq is None:
            return
        if not self.metrics_plots_enabled:
            return

        sync_state = self.acq.get_sync_state()

        if sync_state is not None:
            elapsed = time.perf_counter() - self.start_time
            self.history["t"].append(elapsed)
            self.history["offset"].append(sync_state.offset_ms)
            self.history["drift"].append(sync_state.drift_rate_ppm)
            self.history["confidence"].append(sync_state.confidence)

            # Keep only recent history
            for key in ("t", "offset", "drift", "confidence"):
                self.history[key] = self.history[key][-self.max_history:]

            # Update metric plots only when enabled
            if self.metrics_plots_enabled and len(self.history["t"]) > 1:
                t_array = np.array(self.history["t"])
                self.curve_offset.setData(t_array, self.history["offset"])
                self.curve_drift.setData(t_array, self.history["drift"])
                self.curve_confidence.setData(t_array, self.history["confidence"])

            # Update status label with color coding
            status_color = self._get_status_color(sync_state)
            status_text = (
                f"Offset: {sync_state.offset_ms:+.2f} ms | "
                f"Drift: {sync_state.drift_rate_ppm:+.1f} ppm | "
                f"Confidence: {sync_state.confidence:.3f}"
            )
            self.status_label.setText(status_text)
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

    # ------------------------------------------------------------------
    # slow path – PRBS chip waveform plots (STM32 / Porti7)
    # Called at ~30 fps.  get_chip_streams_for_display + step-data
    # construction is the expensive part; skipped when data hasn't changed.
    # ------------------------------------------------------------------
    def update_prbs_plots(self):
        """Update STM32 and Porti7 PRBS chip plots (called at ~30 fps)."""
        if self.acq is None:
            return

        try:
            t_end = None

            # --- STM32 (directly from live STM32Reader acquisition) ---
            stm32_changed = False
            stm32_snapshot = self.acq.get_stm32_snapshot(n=1200)
            stm32_t = stm32_snapshot.get("t_sec")
            stm32_tick = stm32_snapshot.get("prbs_tick")
            stm32_prbs = stm32_snapshot.get("prbs_level")
            if (
                stm32_t is not None
                and stm32_tick is not None
                and stm32_prbs is not None
                and len(stm32_t) > 0
                and len(stm32_t) == len(stm32_tick) == len(stm32_prbs)
            ):
                valid = np.isfinite(stm32_t) & np.isfinite(stm32_tick) & np.isfinite(stm32_prbs)
                t_raw = np.asarray(stm32_t[valid], dtype=float)
                tick_raw = np.asarray(stm32_tick[valid], dtype=np.int64)
                prbs_raw = (np.asarray(stm32_prbs[valid], dtype=float) > 0.5).astype(float)

                # Convert 500 Hz sample stream to true chip stream by selecting
                # only samples where the chip tick changes.
                if len(tick_raw) > 0:
                    chip_change = np.empty(len(tick_raw), dtype=bool)
                    chip_change[0] = True
                    chip_change[1:] = tick_raw[1:] != tick_raw[:-1]
                    t_vals = t_raw[chip_change]
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
                    stm32_changed = True
                    sx, sy = self._make_step_data_from_x(t_vals, y_vals)
                    self.curve_stm32.setData(sx, sy)

            estimator = getattr(self.acq, "sync_delay_estimator", None)
            if estimator is None:
                return

            _, emg_chips = estimator.get_chip_streams_for_display(
                max_chips=200,
                apply_alignment=False,
            )

            # --- EMG / Porti7 ---
            emg_changed = False
            if emg_chips is not None and len(emg_chips) > 0:
                h = hash(emg_chips.data.tobytes())
                if h != self._last_emg_hash or len(emg_chips) != self._last_emg_len:
                    self._last_emg_hash = h
                    self._last_emg_len = len(emg_chips)
                    emg_changed = True

                    if t_end is None:
                        t_end = time.perf_counter() - self.start_time
                    chip_period_s = 1.0 / float(estimator.chip_rate_hz)
                    emg_t = np.linspace(
                        t_end - (len(emg_chips) - 1) * chip_period_s,
                        t_end,
                        len(emg_chips),
                        dtype=float,
                    )
                    ex, ey = self._make_step_data_from_x(emg_t, emg_chips.astype(float))
                    self.curve_emg.setData(ex, ey)

            if t_end is None:
                t_end = time.perf_counter() - self.start_time
            self.plot_stm32.setXRange(max(0.0, t_end - self.prbs_window_s), t_end + 0.05, padding=0)

            _ = stm32_changed or emg_changed

        except Exception as exc:
            msg = str(exc)
            if not hasattr(self, '_last_prbs_err') or self._last_prbs_err != msg:
                print(f"[VIZ] PRBS plot error: {msg}")
                self._last_prbs_err = msg
    
    def _get_status_color(self, sync_state) -> str:
        """Get status color based on sync quality."""
        confidence = sync_state.confidence
        drift = abs(sync_state.drift_rate_ppm)
        
        # Green: Good sync
        if confidence > 0.6 and drift < 1000:
            return "#27ae60"  # Green
        # Yellow: Moderate sync
        elif confidence > 0.3 or drift < 3000:
            return "#f39c12"  # Orange
        # Red: Poor sync
        else:
            return "#e74c3c"  # Red
    
    def closeEvent(self, event):
        """Clean up when window is closed."""
        print("\n[VIZ] Window closed, stopping acquisition...")
        for attr in ("timer", "prbs_timer", "update_timer", "duration_timer"):
            t = getattr(self, attr, None)
            if t is not None:
                t.stop()
        event.accept()


def run_with_pyqtgraph_visualization(
    config: SignalAcquisitionConfig,
    duration_s: Optional[float],
    mode: str,
) -> None:
    """Run PRBS acquisition with PyQtGraph real-time visualization."""
    if not _TMSI_AVAILABLE:
        print("\nERROR: TMSi SDK not available. Cannot run visualization.")
        return
    
    if not _PYQTGRAPH_AVAILABLE:
        print("\nERROR: PyQtGraph not available. Install with: pip install pyqtgraph PyQt6")
        return
    
    cfg = SignalAcquisitionConfig(
        enable_stm32=True,
        enable_emg=True,
        enable_prbs_sync=True,
        sync_mode="realtime",
        verbose=config.verbose,
        stm32_port=config.stm32_port,
        stm32_baud=config.stm32_baud,
        emg_connection_type=config.emg_connection_type,
        emg_differential_pairs=config.emg_differential_pairs,
        emg_sample_rate=config.emg_sample_rate,
        prbs_correlation_window_s=config.prbs_correlation_window_s,
        prbs_update_interval_s=config.prbs_update_interval_s,
    )
    
    if config.verbose:
        dur_msg = f"Duration: {duration_s:.0f} s. " if duration_s is not None else ""
        print("\n" + "=" * 70)
        print("SYNCHRONIZED STM32 + EMG (PyQtGraph High-Performance Visualization)")
        print("=" * 70)
        print(f"  Combined STM32 + EMG + PRBS sync. {dur_msg}Close window to stop.\n")
    
    # Start acquisition
    acq = SignalAcquisition(cfg)
    acq.start()
    time.sleep(1.0)  # Let acquisition stabilize
    
    # Create Qt application
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)
    
    # Create and show visualization window
    window = PRBSVisualizationWindow(acq, duration_s)
    window.show()
    
    # Run Qt event loop
    try:
        app.exec() if hasattr(app, 'exec') else app.exec_()
    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted by user")
    finally:
        # Stop acquisition
        acq.stop()
        
        # Print summary
        if config.verbose:
            ss = acq.get_sync_state()
            if ss is not None:
                print(f"\n  [SUMMARY] PRBS: offset={ss.offset_ms:+.2f} ms, confidence={ss.confidence:.3f}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="STM32 + EMG signal acquisition with PyQtGraph visualization.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python signal_acquisition_testing_pyqtgraph.py --mode all --duration 60
  python signal_acquisition_testing_pyqtgraph.py --mode prbs
""",
    )
    parser.add_argument(
        "--mode",
        choices=["stm32", "emg", "prbs", "all", "sequence"],
        default="all",
        help="Run mode",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        metavar="SEC",
        help="Duration (seconds). If not set, runs until window is closed.",
    )
    args = parser.parse_args()
    
    config = SignalAcquisitionConfig()
    
    if args.mode == "stm32":
        run_stm32_only(config, args.duration)
    elif args.mode == "emg":
        run_emg_only(config, args.duration)
    elif args.mode in ("prbs", "all"):
        run_with_pyqtgraph_visualization(config, args.duration, args.mode)
    else:  # sequence
        run_stm32_only(config, args.duration)
        run_emg_only(config, args.duration)
        run_with_pyqtgraph_visualization(config, args.duration, "all")


if __name__ == "__main__":
    main()
