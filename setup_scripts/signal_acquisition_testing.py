"""
Synchronized STM32 + EMG Signal Acquisition (Real-time Visualization)
=====================================================================

Single mode: run both acquisition devices (STM32 + EMG) and PRBS synchronization
together with PyQtGraph real-time visualization.

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
from typing import Optional

import numpy as np

# -----------------------------------------------------------------------------
# CONFIG – edit these to tune duration and PRBS sync (no CLI)
# -----------------------------------------------------------------------------
DURATION_S: Optional[float] = None  # Run indefinitely until window closed
PRBS_CORRELATION_WINDOW_S: float = 10.0
PRBS_UPDATE_INTERVAL_S: float = 2.0
PRBS_CHIP_RATE_HZ: float = 10.0  # Must match STM32 firmware (10 Hz chip rate)
PRBS_VIZ_WINDOW_S: float = 10.0  # Sliding window width in seconds for the plot
STM32_PORT: Optional[str] = None # Auto-detect
STM32_BAUD: int = 921600
EMG_CONNECTION_TYPE: str = "usb"
EMG_SAMPLE_RATE: Optional[int] = 2000
VERBOSE: bool = True
# -----------------------------------------------------------------------------

# PyQtGraph required
try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtWidgets, QtCore
    _PYQTGRAPH_AVAILABLE = True
except ImportError:
    _PYQTGRAPH_AVAILABLE = False

_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

try:
    from src.signal_acquisition import SignalAcquisition, SignalAcquisitionConfig
    from src.stm32_reader import STM32Reader
    from src.stm32_emg_sync import SyncDelayEstimator
    from src.signal_acquisition import _TMSI_AVAILABLE
except ImportError as e:
    print(f"Import Error: {e}")
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
        self.prbs_window_s = PRBS_VIZ_WINDOW_S

        self.timer: Optional[QtCore.QTimer] = None
        self.duration_timer: Optional[QtCore.QTimer] = None

        self._last_stm32_len: int = 0
        self._last_stm32_hash: int = 0
        self._last_emg_len: int = 0
        self._last_emg_hash: int = 0
        self._last_prbs_err: Optional[str] = None

        self.setWindowTitle("PRBS Signal Monitor – STM32 + EMG")
        self.resize(1000, 800)
        layout = QtWidgets.QVBoxLayout()
        self.setLayout(layout)

        self.status_label = QtWidgets.QLabel("Acquiring PRBS signals...")
        self.status_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet(
            "font-size: 14px; font-weight: bold; padding: 10px; "
            "background-color: #2c3e50; color: white;"
        )
        layout.addWidget(self.status_label)

        self.graphics_layout = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphics_layout)

        # Plot 1: STM32 PRBS
        self.plot_stm32 = self.graphics_layout.addPlot(row=0, col=0, title="STM32 PRBS (500 Hz)")
        self.plot_stm32.setLabel("left", "Level")
        self.plot_stm32.setLabel("bottom", "Time", units="s")
        self.plot_stm32.setYRange(-1.2, 1.2)
        self.plot_stm32.showGrid(x=True, y=True, alpha=0.3)
        self.curve_stm32 = self.plot_stm32.plot(pen=pg.mkPen(color="#2ecc71", width=2))

        # Plot 2: EMG TRIG
        self.plot_emg = self.graphics_layout.addPlot(row=1, col=0, title="Porti7 TRIG (EMG)")
        self.plot_emg.setLabel("left", "Level")
        self.plot_emg.setLabel("bottom", "Time", units="s")
        self.plot_emg.setYRange(-1.2, 1.2)
        self.plot_emg.showGrid(x=True, y=True, alpha=0.3)
        self.curve_emg = self.plot_emg.plot(pen=pg.mkPen(color="#3498db", width=2))
        
        # Link X axes
        self.plot_emg.setXLink(self.plot_stm32)

        for curve in [self.curve_stm32, self.curve_emg]:
            curve.setClipToView(True)

        # Single timer for viz updates (~30 fps)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._on_viz_tick)
        self.timer.start(33)

        if self.duration_s is not None:
            self.duration_timer = QtCore.QTimer()
            self.duration_timer.timeout.connect(self._check_duration)
            self.duration_timer.start(1000)

    def _check_duration(self) -> None:
        if self.duration_s is not None:
            elapsed = time.perf_counter() - self.start_time
            if elapsed >= self.duration_s:
                if VERBOSE:
                    print(f"\n[VIZ] Duration {self.duration_s:.0f}s reached, closing...")
                self.close()

    def _on_viz_tick(self) -> None:
        """Update PRBS signal plots."""
        self._update_prbs_plots()
        
        # Update status
        elapsed = time.perf_counter() - self.start_time
        reader = getattr(self.acq, "stm32_reader", None)
        estimator = getattr(self.acq, "sync_delay_estimator", None)
        
        stm32_info = "STM32: --"
        if reader is not None:
            n_samples = reader.sample_count
            rate = n_samples / elapsed if elapsed > 0.5 else 0
            stm32_info = f"STM32: {n_samples} samples ({rate:.0f} Hz)"
            
        emg_info = "EMG: --"
        if estimator is not None:
            # Use SyncDelayEstimator rolling buffer length
            n_bufs = estimator.emg_buf_len
            emg_info = f"EMG Buf: {n_bufs}"
        elif sync_engine is not None:
            # Fallback to legacy sync_engine buffer length
            n_bufs = len(sync_engine.emg_word_buf) + len(sync_engine.emg_buf)
            emg_info = f"EMG Buf: {n_bufs}"

        self.status_label.setText(
            f"{stm32_info} | {emg_info} | Elapsed: {elapsed:.0f}s"
        )
        
        # DEBUG: Print status every ~1s
        if int(elapsed) > getattr(self, "_last_debug_s", -1):
            self._last_debug_s = int(elapsed)
            print(f"[DEBUG] {stm32_info} | {emg_info}")
            if estimator:
                 sync = getattr(self.acq, "sync_engine", None)
                 if sync:
                     wb = len(sync.emg_word_buf)
                     bb = len(sync.emg_buf)
                     print(f"  Buffers: word_buf={wb}, bipolar_buf={bb}")
                     if wb > 0:
                         # Print last few raw words to see if we have valid data
                         last_few = [x for _, x in list(sync.emg_word_buf)[-5:]]
                         print(f"  Last 5 raw words: {last_few}")
                     if bb > 0:
                         last_few = [x for _, x in list(sync.emg_buf)[-5:]]
                         print(f"  Last 5 bipolar: {last_few}")

    def _update_prbs_plots(self) -> None:
        """Fetch latest data and update curves."""
        if self.acq is None:
            return

        try:
            # We want to show the last N seconds of data.
            # We can get this from SignalAcquisition.stm32_reader and SignalAcquisition.emg_buffer (via Thread)
            # OR better, use the ring buffers in SignalAcquisition directly if available.
            # Actually, SignalAcquisition wraps STM32Reader.
            
            elapsed = time.perf_counter() - self.start_time
            now = time.perf_counter()
            
            # --- STM32 Data ---
            # Get last window seconds
            # 500 Hz * window
            n_samples_stm32 = int(self.prbs_window_s * 550) 
            stm32_snap = self.acq.get_stm32_snapshot(n=n_samples_stm32)
            
            if stm32_snap and "t_sec" in stm32_snap and len(stm32_snap["t_sec"]) > 0:
                # Re-align t_sec to be relative to start_time for simple viewing
                # But wait, t_sec from reader is "t_ms / 1000.0". 
                # This is STM32 internal time.
                # Ideally we want to plot against "pc_time" which is host time.
                
                t_host = stm32_snap.get("pc_time", None)
                prbs_lvl = stm32_snap.get("prbs_level", None) # 0, 1
                
                if t_host is not None and prbs_lvl is not None:
                     # Filter to window
                    mask = t_host > (now - self.prbs_window_s)
                    t_plot = t_host[mask]
                    y_plot = prbs_lvl[mask]
                    
                    if len(t_plot) > 0:
                        # Normalize time to relative to viz start
                        t_rel = t_plot - self.start_time
                        self.curve_stm32.setData(t_rel, y_plot)
            
            # --- EMG Data ---
            # Priority 1: SyncDelayEstimator (real-time mode)
            estimator = getattr(self.acq, "sync_delay_estimator", None)
            sync_engine = getattr(self.acq, "sync_engine", None)
            
            if estimator is not None:
                # RAW 2000 Hz signal visualization
                with estimator._lock:
                    chunks = list(estimator._emg_trig_buf)
                
                if chunks:
                    # Concatenate all recent raw chunks
                    t_raw = np.concatenate([c[0] for c in chunks])
                    v_raw = np.concatenate([c[1] for c in chunks])
                    
                    # Convert raw STATUS/TRIG to binary (0/1) for visual clarity
                    is_status = getattr(estimator, "_trig_is_status", True)
                    trig_bit = getattr(estimator, "_detected_trig_bit", None)
                    
                    if is_status and trig_bit is not None:
                        y_plot = ((v_raw.astype(np.int64) >> trig_bit) & 1).astype(np.float64)
                    else:
                        # Fallback bit extraction for visual
                        lo, hi = np.nanmin(v_raw), np.nanmax(v_raw)
                        thr = (lo + hi) / 2.0 if (hi - lo) > 1e-9 else 0.5
                        y_plot = (v_raw > thr).astype(np.float64)
                    
                    # Filter to window
                    mask = t_raw > (now - self.prbs_window_s)
                    t_plot = t_raw[mask]
                    y_plot = y_plot[mask]
                    
                    if len(t_plot) > 0:
                        t_rel = t_plot - self.start_time
                        self.curve_emg.setData(t_rel, y_plot)
            
            # Priority 2: Legacy sync_engine (if estimator not used)
            elif sync_engine:
                # Try emg_buf first (bipolar floats)
                target_buf = sync_engine.emg_buf
                is_word_buf = False
                
                if len(target_buf) == 0:
                    target_buf = sync_engine.emg_word_buf
                    is_word_buf = True
                
                if len(target_buf) > 0:
                    try:
                        # Fetch last 2.5s worth of data approx (2000Hz * 2.5 = 5000)
                        # or based on window. 2000Hz * 10s = 20000.
                        buf_len = len(target_buf)
                        points_to_fetch = min(buf_len, int(self.prbs_window_s * 2500))
                        
                        # Efficient deque slicing using itertools
                        import itertools
                        slice_iter = itertools.islice(target_buf, buf_len - points_to_fetch, buf_len)
                        data_points = list(slice_iter)
                        
                        if data_points:
                            arr = np.array(data_points) # shape (N, 2)
                            t_vals = arr[:, 0]
                            y_vals = arr[:, 1]
                            
                            # Filter to window
                            mask = t_vals > (now - self.prbs_window_s)
                            t_plot = t_vals[mask]
                            y_plot = y_vals[mask]
                            
                            if is_word_buf:
                                # Data is raw integer bitfield.
                                # Check if sync_engine has identified a trigger bit
                                trig_bit = getattr(sync_engine, "_trig_bit", None)
                                
                                if trig_bit is not None:
                                    # Extract that bit
                                    y_plot = ((y_plot.astype(int) >> trig_bit) & 1).astype(float)
                                else:
                                    # Auto-detect: find the bit with the most transitions in this window
                                    # We check bits 0-7 (Porti7 status/dig usually low bits)
                                    best_bit = 0
                                    max_trans = -1
                                    
                                    # Cast once to int to avoid repeated casting
                                    y_int = y_plot.astype(int)
                                    
                                    # Heuristic: check variance/transitions
                                    # Optimization: checking 8 bits for 20k samples is fast enough for 30fps viz
                                    for b in range(8):
                                        bit_sig = (y_int >> b) & 1
                                        # Count transitions (simple sum of absolute differences)
                                        # Use a subset if array is huge to save time
                                        subset = bit_sig[::10] if len(bit_sig) > 2000 else bit_sig
                                        if len(subset) > 1:
                                            trans = np.sum(np.abs(np.diff(subset)))
                                            if trans > max_trans:
                                                max_trans = trans
                                                best_bit = b
                                    
                                    # If no activity found, stay at 0, otherwise use best
                                    if max_trans > 0:
                                        y_plot = ((y_int >> best_bit) & 1).astype(float)
                                    else:
                                        # Fallback to bit 0
                                        y_plot = (y_int & 1).astype(float)

                            if len(t_plot) > 0:
                                t_rel = t_plot - self.start_time
                                self.curve_emg.setData(t_rel, y_plot)
                                
                    except Exception:
                        pass

            # Update X Range to latest time
            # Relative time
            current_rel = now - self.start_time
            self.plot_stm32.setXRange(max(0, current_rel - self.prbs_window_s), current_rel + 0.5)

        except Exception as exc:
            if self._last_prbs_err != str(exc):
                print(f"[VIZ] PRBS update error: {exc}")
                self._last_prbs_err = str(exc)

    def closeEvent(self, event) -> None:
        if self.timer:
            self.timer.stop()
        if self.duration_timer:
            self.duration_timer.stop()
        event.accept()


def run() -> None:
    """Run real-time visualization."""
    if not _PYQTGRAPH_AVAILABLE:
        print("\nERROR: PyQtGraph is required. Install with: pip install pyqtgraph PyQt6")
        sys.exit(1)
    if not _TMSI_AVAILABLE:
        print("\nERROR: TMSi SDK not available. Cannot run acquisition.")
        sys.exit(1)

    config = _build_config()
    print("=" * 60)
    print("STM32 + EMG PRBS Real-time Monitor")
    print("=" * 60)
    print(f"  Duration: {DURATION_S if DURATION_S else 'Indefinite'} s")

    # Initialize Acquisition
    acq = SignalAcquisition(config)
    acq.start()
    
    # Wait a moment for threads to spin up
    time.sleep(1.0)

    try:
        app = QtWidgets.QApplication.instance()
        if app is None:
            app = QtWidgets.QApplication(sys.argv)
            
        # Set dark theme for cool aesthetics (as per instructions 'Use Rich Aesthetics')
        app.setStyle("Fusion")
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.ColorRole.Window, QtGui.QColor(53, 53, 53))
        palette.setColor(QtGui.QPalette.ColorRole.WindowText, QtCore.Qt.GlobalColor.white)
        palette.setColor(QtGui.QPalette.ColorRole.Base, QtGui.QColor(25, 25, 25))
        palette.setColor(QtGui.QPalette.ColorRole.AlternateBase, QtGui.QColor(53, 53, 53))
        palette.setColor(QtGui.QPalette.ColorRole.ToolTipBase, QtCore.Qt.GlobalColor.white)
        palette.setColor(QtGui.QPalette.ColorRole.ToolTipText, QtCore.Qt.GlobalColor.white)
        palette.setColor(QtGui.QPalette.ColorRole.Text, QtCore.Qt.GlobalColor.white)
        palette.setColor(QtGui.QPalette.ColorRole.Button, QtGui.QColor(53, 53, 53))
        palette.setColor(QtGui.QPalette.ColorRole.ButtonText, QtCore.Qt.GlobalColor.white)
        palette.setColor(QtGui.QPalette.ColorRole.BrightText, QtCore.Qt.GlobalColor.red)
        palette.setColor(QtGui.QPalette.ColorRole.Link, QtGui.QColor(42, 130, 218))
        palette.setColor(QtGui.QPalette.ColorRole.Highlight, QtGui.QColor(42, 130, 218))
        palette.setColor(QtGui.QPalette.ColorRole.HighlightedText, QtCore.Qt.GlobalColor.black)
        app.setPalette(palette)

        window = PRBSVisualizationWindow(acq, DURATION_S)
        window.show()

        app.exec() if hasattr(app, "exec") else app.exec_()
        
    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted by user.")
    except Exception as e:
        print(f"\n[MAIN] Error: {e}")
    finally:
        print("\n[MAIN] Stopping acquisition...")
        acq.stop()
        print("[MAIN] Done.")

if __name__ == "__main__":
    # Import QtGui here for the palette code above
    from pyqtgraph.Qt import QtGui
    run()
