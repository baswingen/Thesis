"""
Inference Dashboard
===================

Real-time PyQt6 dashboard for live weight prediction using a trained CNN-LSTM.

Panels
------
- Header           : model info, run directory, elapsed time, LIVE / SYNCING
- Sensor Health    : STM32, EMG, IMU1, IMU2 LEDs + rates + PRBS confidence
- Predicted Weight : large hero readout, confidence ring, history sparkline
- Prediction Chart : 60-second scrolling prediction history (pyqtgraph)
- EMG Channels     : 8 scrolling raw-voltage plots
- IMU 3-D          : dual 3-D orientation cubes (reusing trial/dashboard.py)
- Model Info       : architecture, hyperparameters, per-weight MAE table

Run in demo mode (no hardware):
    python inference/run_inference.py --demo
"""

from __future__ import annotations

import sys
import time
import math
import threading
from collections import deque
from pathlib import Path
from typing import Optional, List, Tuple, TYPE_CHECKING

import numpy as np

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

# ---------------------------------------------------------------------------
# Qt / pyqtgraph
# ---------------------------------------------------------------------------
try:
    from PyQt6 import QtWidgets, QtCore, QtGui
    import pyqtgraph as pg
    _QT_OK = True
except ImportError:
    _QT_OK = False
    print("ERROR: PyQt6 + pyqtgraph are required. Run: pip install PyQt6 pyqtgraph")

# OpenGL is optional (only for 3-D IMU cubes)
try:
    import pyqtgraph.opengl as gl
    pg.setConfigOptions(antialias=True, useOpenGL=True)
    _GL_OK = True
except ImportError:
    _GL_OK = False
    gl = None
    print("WARNING: PyOpenGL not found. 3-D IMU visualization will be disabled. (pip install PyOpenGL)")

# ---------------------------------------------------------------------------
# Design tokens — match trial/dashboard.py exactly
# ---------------------------------------------------------------------------
BG_DARK   = "#2c3e50"
BG_MID    = "#34495e"
BG_LIGHT  = "#3d5166"
TEXT_MAIN = "#ecf0f1"
TEXT_DIM  = "#7f8c8d"
ACCENT    = "#3498db"
GREEN     = "#2ecc71"
RED       = "#e74c3c"
ORANGE    = "#f39c12"
GREEN_D   = "#27ae60"
RED_D     = "#c0392b"
PURPLE    = "#9b59b6"

_COMMON_STYLE = f"""
    QWidget        {{ background-color: {BG_DARK}; color: {TEXT_MAIN}; font-family: 'Segoe UI', sans-serif; }}
    QLabel         {{ color: {TEXT_MAIN}; }}
    QFrame         {{ border: none; }}
    QPushButton    {{ background-color: {BG_MID}; color: {TEXT_MAIN}; border: none;
                      padding: 8px 16px; border-radius: 5px; font-weight: bold; }}
    QPushButton:hover {{ background-color: {BG_LIGHT}; }}
    QScrollArea    {{ border: none; }}
"""

from inference.inference_config import (
    FAST_REFRESH_MS, PLOT_REFRESH_MS, IMU_REFRESH_MS,
    EMG_SCROLL_S, EMG_CHANNEL_MAPPING
)
from src.emg_processing import EMGProcessor

EMG_MUSCLE_NAMES = list(EMG_CHANNEL_MAPPING.keys())

def _name_to_raw_idx(ch_name: str) -> int:
    """Map 'ch1'..'ch32' to raw TMSi column indices 16..47 (BIP start)."""
    try:
        num = int(ch_name.replace("ch", ""))
        return 15 + num
    except:
        return -1

EMG_SCROLL_S_INFERENCE = EMG_SCROLL_S
IMU_SCROLL_S_INFERENCE = 4.0

# Weight colours for the confidence ring / hero readout
def _weight_to_confidence_color(confidence: float) -> str:
    if confidence >= 0.7:
        return GREEN
    elif confidence >= 0.4:
        return ORANGE
    return RED


# ---------------------------------------------------------------------------
# Shared UI helpers (mirror trial/dashboard.py)
# ---------------------------------------------------------------------------

def _section_label(text: str) -> "QtWidgets.QLabel":
    lbl = QtWidgets.QLabel(text.upper())
    lbl.setStyleSheet(
        f"font-weight: bold; font-size: 11px; color: {ACCENT}; "
        f"letter-spacing: 1px; padding: 2px 0px;"
    )
    return lbl


def _divider() -> "QtWidgets.QFrame":
    f = QtWidgets.QFrame()
    f.setFrameShape(QtWidgets.QFrame.Shape.HLine)
    f.setStyleSheet(f"background-color: {BG_MID}; max-height: 1px;")
    return f


def _led(color: str = RED, size: int = 14) -> "QtWidgets.QLabel":
    lbl = QtWidgets.QLabel()
    lbl.setFixedSize(size, size)
    lbl.setStyleSheet(f"background-color: {color}; border-radius: {size // 2}px;")
    return lbl


def _styled_plot(title: str = "", y_label: str = "") -> "pg.PlotWidget":
    p = pg.PlotWidget()
    p.setBackground(BG_DARK)
    p.showGrid(x=True, y=True, alpha=0.15)
    p.getAxis("left").setPen(TEXT_DIM)
    p.getAxis("bottom").setPen(TEXT_DIM)
    p.getAxis("left").setTextPen(TEXT_DIM)
    p.getAxis("bottom").setTextPen(TEXT_DIM)
    p.getAxis("top").setHeight(0)
    p.getAxis("right").setWidth(0)
    if title:
        p.setTitle(title, color=TEXT_MAIN, size="9pt")
    if y_label:
        p.setLabel("left", y_label)
    return p


# ---------------------------------------------------------------------------
# Panel: Header
# ---------------------------------------------------------------------------

class _HeaderPanel(QtWidgets.QFrame):
    stopRequested = QtCore.pyqtSignal()

    def __init__(self, run_dir: str = "", parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {BG_MID}; border-radius: 6px;")
        self._start_time = time.perf_counter()
        self._blink = True

        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(14, 10, 14, 10)

        # Left: model info
        info_lay = QtWidgets.QVBoxLayout()
        self._lbl_title = QtWidgets.QLabel("<b>CNN-LSTM Inference</b>")
        self._lbl_title.setStyleSheet(f"font-size: 15px; color: {TEXT_MAIN};")
        self._lbl_run = QtWidgets.QLabel(run_dir)
        self._lbl_run.setStyleSheet(
            f"font-size: 10px; color: {TEXT_DIM}; font-family: 'Consolas', monospace;"
        )
        info_lay.addWidget(self._lbl_title)
        info_lay.addWidget(self._lbl_run)
        lay.addLayout(info_lay)
        lay.addStretch()

        # Centre: elapsed + status
        centre_lay = QtWidgets.QVBoxLayout()
        self._lbl_time = QtWidgets.QLabel("00:00")
        self._lbl_time.setStyleSheet(
            f"font-size: 22px; font-weight: bold; color: {TEXT_MAIN}; font-family: 'Consolas', monospace;"
        )
        self._lbl_time.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        
        self._lbl_status = QtWidgets.QLabel("⏳ SYNCING")
        self._lbl_status.setFixedWidth(140)  # Reserve space to prevent layout shifting
        self._lbl_status.setStyleSheet(
            f"font-size: 14px; font-weight: bold; color: {ORANGE}; letter-spacing: 2px;"
        )
        self._lbl_status.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        centre_lay.addWidget(self._lbl_time)
        centre_lay.addWidget(self._lbl_status)
        lay.addLayout(centre_lay)
        lay.addStretch()

        # Right: stop button
        self._btn_stop = QtWidgets.QPushButton("⏹  STOP")
        self._btn_stop.setStyleSheet(
            f"background-color: {RED}; color: white; font-weight: bold; "
            f"font-size: 13px; padding: 10px 22px; border-radius: 6px;"
        )
        self._btn_stop.setCursor(QtGui.QCursor(QtCore.Qt.CursorShape.PointingHandCursor))
        self._btn_stop.clicked.connect(self.stopRequested)
        lay.addWidget(self._btn_stop)

    def tick(self, is_live: bool = False):
        elapsed = time.perf_counter() - self._start_time
        m, s = int(elapsed) // 60, int(elapsed) % 60
        self._lbl_time.setText(f"{m:02d}:{s:02d}")

        self._blink = not self._blink
        if is_live:
            self._lbl_status.setText("● LIVE")
            color = GREEN if self._blink else "transparent"
            self._lbl_status.setStyleSheet(
                f"font-size: 14px; font-weight: bold; color: {color}; letter-spacing: 2px;"
            )
        else:
            self._lbl_status.setText("⏳ SYNCING")
            self._lbl_status.setStyleSheet(
                f"font-size: 14px; font-weight: bold; color: {ORANGE}; letter-spacing: 2px;"
            )


# ---------------------------------------------------------------------------
# Panel: Sensor Health
# ---------------------------------------------------------------------------

class _SensorHealthPanel(QtWidgets.QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {BG_MID}; border-radius: 6px;")

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(10, 6, 10, 6)
        lay.setSpacing(4)
        lay.addWidget(_section_label("Sensor Health"))
        lay.addWidget(_divider())

        grid = QtWidgets.QGridLayout()
        grid.setSpacing(4)
        grid.setContentsMargins(0, 0, 0, 0)

        def _cell(label_text):
            w = QtWidgets.QWidget()
            l = QtWidgets.QHBoxLayout(w)
            l.setContentsMargins(0, 0, 0, 0)
            l.setSpacing(4)
            led = _led(RED, size=12)
            lbl = QtWidgets.QLabel(label_text)
            lbl.setStyleSheet(f"font-size: 10px; color: {TEXT_MAIN};")
            txt = QtWidgets.QLabel("—")
            txt.setStyleSheet(f"font-family: 'Consolas'; font-size: 9px; color: {TEXT_DIM};")
            l.addWidget(led); l.addWidget(lbl); l.addWidget(txt); l.addStretch()
            return w, led, txt

        w1, self._stm32_led, self._stm32_txt = _cell("STM32")
        w2, self._tmsi_led,  self._tmsi_txt  = _cell("EMG")
        w3, self._imu1_led,  self._imu1_txt  = _cell("IMU 1")
        w4, self._imu2_led,  self._imu2_txt  = _cell("IMU 2")

        grid.addWidget(w1, 0, 0); grid.addWidget(w2, 0, 1)
        grid.addWidget(w3, 1, 0); grid.addWidget(w4, 1, 1)
        lay.addLayout(grid)

    def _set_led(self, led: QtWidgets.QLabel, ok: bool, color_override: str | None = None):
        color = color_override if color_override else (GREEN if ok else RED)
        led.setStyleSheet(
            f"background-color: {color}; border: 1px solid rgba(255,255,255,0.1); border-radius: 8px;"
        )

    def update_stm32(self, online: bool, rate_hz: float = 0.0):
        is_active = online and rate_hz > 200
        self._set_led(self._stm32_led, is_active)
        self._stm32_txt.setText(f"{rate_hz:.0f} Hz" if is_active else "Offline")

    def update_imu1(self, ok: bool, stuck: bool = False):
        color = GREEN if (ok and not stuck) else (ORANGE if (ok and stuck) else RED)
        self._set_led(self._imu1_led, True, color_override=color)
        self._imu1_txt.setText("OK" if (ok and not stuck) else ("STUCK" if (ok and stuck) else "OFFLINE"))

    def update_imu2(self, ok: bool, stuck: bool = False):
        color = GREEN if (ok and not stuck) else (ORANGE if (ok and stuck) else RED)
        self._set_led(self._imu2_led, True, color_override=color)
        self._imu2_txt.setText("OK" if (ok and not stuck) else ("STUCK" if (ok and stuck) else "OFFLINE"))

    def update_tmsi(self, online: bool, rate_hz: float = 0.0):
        is_active = online and rate_hz > 1000
        self._set_led(self._tmsi_led, is_active)
        self._tmsi_txt.setText(f"{rate_hz:.0f} Hz" if is_active else "Offline")


# ---------------------------------------------------------------------------
# Panel: PRBS Sync Monitor (Migrated from trial/dashboard.py)
# ---------------------------------------------------------------------------

class _SyncPanel(QtWidgets.QFrame):
    """
    Compact PRBS synchronisation status panel.
    Shows Kalman delay, confidence, drift and update count as styled text rows.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {BG_MID}; border-radius: 6px;")

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(10, 6, 10, 6)
        lay.setSpacing(4)
        lay.addWidget(_section_label("PRBS Synchronization"))
        lay.addWidget(_divider())

        grid = QtWidgets.QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(8)

        def _metric_box(label_text, default="—"):
            w = QtWidgets.QWidget()
            l = QtWidgets.QVBoxLayout(w)
            l.setContentsMargins(0, 0, 0, 0)
            l.setSpacing(0)
            lbl = QtWidgets.QLabel(label_text)
            lbl.setStyleSheet(f"font-size: 9px; color: {TEXT_DIM}; text-transform: uppercase;")
            val = QtWidgets.QLabel(default)
            val.setStyleSheet(
                f"font-family: 'Consolas'; font-size: 12px; color: {TEXT_MAIN}; font-weight: bold;"
            )
            l.addWidget(lbl)
            l.addWidget(val)
            return w, val

        w1, self._lbl_delay   = _metric_box("Delay")
        w2, self._lbl_conf    = _metric_box("Conf.")
        w3, self._lbl_drift   = _metric_box("Drift")
        w4, self._lbl_updates = _metric_box("Update")

        grid.addWidget(w1, 0, 0)
        grid.addWidget(w2, 0, 1)
        grid.addWidget(w3, 1, 0)
        grid.addWidget(w4, 1, 1)

        lay.addLayout(grid)

    def update(self, estimator):
        """Feed a SyncDelayEstimator instance and refresh text labels."""
        if estimator is None:
            return
        result = estimator.get_result()
        if result is None:
            return

        delay_ms   = estimator.get_delay_ms()
        confidence = result.confidence
        drift_ppm  = estimator.get_drift_rate_ppm() if hasattr(estimator, "get_drift_rate_ppm") else 0.0
        updates    = getattr(estimator, "_update_count", 0)

        self._lbl_delay.setText(f"{delay_ms:+.1f} ms")
        self._lbl_conf.setText(f"{confidence:.3f}")
        self._lbl_drift.setText(f"{drift_ppm:.1f} ppm")
        self._lbl_updates.setText(str(updates))

        # Colour-code confidence value
        if confidence > 0.4:
            conf_color = GREEN
        elif confidence > 0.15:
            conf_color = ORANGE
        else:
            conf_color = RED
        self._lbl_conf.setStyleSheet(
            f"font-family: 'Consolas', monospace; font-size: 13px; "
            f"color: {conf_color}; font-weight: bold;"
        )


# ---------------------------------------------------------------------------
# Panel: Hero predicted weight
# ---------------------------------------------------------------------------

class _PredictionHeroPanel(QtWidgets.QFrame):
    """Full-width hero panel showing the current predicted weight."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {BG_MID}; border-radius: 8px;")

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(16, 12, 16, 12)
        lay.setSpacing(6)
        lay.addWidget(_section_label("Predicted Weight"))
        lay.addWidget(_divider())

        # Use a stacked layout to prevent shifting between 'syncing' and 'readout' states
        self._readout_stack = QtWidgets.QStackedLayout()
        
        # Syncing overlay (shown until PRBS ready)
        self._lbl_syncing = QtWidgets.QLabel("⏳ Awaiting PRBS sync…")
        self._lbl_syncing.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self._lbl_syncing.setStyleSheet(
            f"font-size: 20px; font-weight: bold; color: {ORANGE}; padding: 20px;"
        )
        self._readout_stack.addWidget(self._lbl_syncing)

        # Main weight readout
        self._weight_widget = QtWidgets.QWidget()
        weight_lay = QtWidgets.QVBoxLayout(self._weight_widget)
        weight_lay.setContentsMargins(0, 0, 0, 0)
        weight_lay.setSpacing(4)

        self._lbl_weight = QtWidgets.QLabel("— kg")
        self._lbl_weight.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self._lbl_weight.setStyleSheet(
            f"font-size: 72px; font-weight: bold; color: {GREEN}; "
            f"font-family: 'Consolas', monospace; letter-spacing: -2px;"
        )
        weight_lay.addWidget(self._lbl_weight)

        # Sub-info row: raw / smoothed / updated
        self._lbl_sub = QtWidgets.QLabel("EMA α=0.40  |  last updated: —")
        self._lbl_sub.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self._lbl_sub.setStyleSheet(f"font-size: 11px; color: {TEXT_DIM}; font-family: 'Consolas';")
        weight_lay.addWidget(self._lbl_sub)
        
        self._readout_stack.addWidget(self._weight_widget)
        lay.addLayout(self._readout_stack)

        # Zero-fill warning banner
        self._lbl_zero_warn = QtWidgets.QLabel("")
        self._lbl_zero_warn.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self._lbl_zero_warn.setWordWrap(True)
        self._lbl_zero_warn.setVisible(False)
        self._lbl_zero_warn.setStyleSheet(
            f"font-size: 10px; color: {ORANGE}; background-color: rgba(243,156,18,0.15); "
            f"border: 1px solid {ORANGE}; border-radius: 4px; padding: 4px 8px;"
        )
        lay.addWidget(self._lbl_zero_warn)

        # Mini sparkline (last 20 predictions)
        self._sparkline = _styled_plot(title="", y_label="kg")
        self._sparkline.setMinimumHeight(60)
        self._sparkline.setMaximumHeight(80)
        self._sparkline.getAxis("bottom").setStyle(showValues=False)
        self._spark_curve = self._sparkline.plot(pen=pg.mkPen(ACCENT, width=2))
        self._spark_x: List[float] = []
        self._spark_y: List[float] = []
        lay.addWidget(self._sparkline)

    def update(
        self,
        smoothed_kg: Optional[float],
        raw_kg: Optional[float],
        confidence: float,
        ema_alpha: float,
        last_t: float,
        zero_filled: List[str],
        is_synced: bool,
    ):
        if not is_synced or smoothed_kg is None:
            self._readout_stack.setCurrentIndex(0)  # Show syncing
            return

        self._readout_stack.setCurrentIndex(1)  # Show readout

        # Colour based on confidence
        color = _weight_to_confidence_color(confidence)
        self._lbl_weight.setText(f"{smoothed_kg:.2f} kg")
        self._lbl_weight.setStyleSheet(
            f"font-size: 72px; font-weight: bold; color: {color}; "
            f"font-family: 'Consolas', monospace; letter-spacing: -2px;"
        )

        # Sub-info
        age = time.perf_counter() - last_t if last_t > 0 else 0.0
        raw_str = f"{raw_kg:.2f}" if raw_kg is not None else "—"
        self._lbl_sub.setText(
            f"raw={raw_str} kg  |  EMA α={ema_alpha:.2f}  |  updated {age:.1f}s ago"
        )

        # Zero-fill warning
        if zero_filled:
            self._lbl_zero_warn.setText(
                f"⚠ {len(zero_filled)} channel(s) zero-filled — accuracy may be reduced:\n"
                + ", ".join(zero_filled)
            )
            self._lbl_zero_warn.setVisible(True)
        else:
            self._lbl_zero_warn.setVisible(False)

        # Sparkline
        now = time.perf_counter()
        self._spark_x.append(now)
        self._spark_y.append(smoothed_kg)
        # Keep last 20 points
        if len(self._spark_x) > 20:
            self._spark_x = self._spark_x[-20:]
            self._spark_y = self._spark_y[-20:]
        if len(self._spark_x) > 1:
            t_rel = np.array(self._spark_x) - self._spark_x[0]
            self._spark_curve.setData(t_rel, self._spark_y)


# ---------------------------------------------------------------------------
# Panel: Prediction history chart
# ---------------------------------------------------------------------------

class _PredictionHistoryPanel(QtWidgets.QFrame):
    def __init__(self, history_s: float = 60.0, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {BG_MID}; border-radius: 6px;")
        self._history_s = history_s

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(4)
        lay.addWidget(_section_label("Prediction History"))
        lay.addWidget(_divider())

        self._plot = _styled_plot(y_label="Weight (kg)")
        self._plot.setLabel("bottom", "Time (s)")
        self._curve_smooth = self._plot.plot(
            pen=pg.mkPen(GREEN, width=2), name="Smoothed"
        )
        self._curve_raw = self._plot.plot(
            pen=pg.mkPen(ACCENT, width=1, style=QtCore.Qt.PenStyle.DashLine), name="Raw"
        )
        self._plot.addLegend(offset=(10, 10))
        lay.addWidget(self._plot)

        self._t0 = time.perf_counter()

    def update_history(self, history: List[Tuple[float, float, float]]):
        """history: list of (timestamp, raw_kg, smoothed_kg)"""
        if len(history) < 2:
            return
        now = time.perf_counter()
        cutoff = now - self._history_s
        history = [(t, r, s) for t, r, s in history if t >= cutoff]
        if len(history) < 2:
            return

        ts   = np.array([h[0] for h in history]) - self._t0
        raws = np.array([h[1] for h in history])
        smoo = np.array([h[2] for h in history])

        self._curve_raw.setData(ts, raws)
        self._curve_smooth.setData(ts, smoo)


# ---------------------------------------------------------------------------
# Panel: EMG channels (scrolling)
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Panel: EMG channels (Migrated from trial/dashboard.py)
# ---------------------------------------------------------------------------

class _EMGPanel(QtWidgets.QFrame):
    """
    8-channel configurable EMG visualization (Processed Envelope).
    Displays 8 separate plots of processed signals.
    """
    def __init__(self, n_channels: int = 8, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {BG_MID}; border-radius: 6px;")
        self.n_channels = n_channels
        
        # Pre-load muscle names from mapping
        self.channel_indices = []
        for name in EMG_MUSCLE_NAMES:
            mapping = EMG_CHANNEL_MAPPING[name]
            if isinstance(mapping, tuple):
                idx = _name_to_raw_idx(mapping[0])
            else:
                idx = _name_to_raw_idx(mapping)
            self.channel_indices.append(idx)
        
        # Ensure we have exactly n_channels
        if len(self.channel_indices) < n_channels:
            self.channel_indices.extend([i for i in range(n_channels - len(self.channel_indices))])
        self.channel_indices = self.channel_indices[:n_channels]
        
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(4)
        
        # Header with Config Toggle
        hdr = QtWidgets.QHBoxLayout()
        hdr.addWidget(_section_label(f"EMG Channels (Readout)"))
        hdr.addStretch()
        self._btn_config = QtWidgets.QPushButton("⚙ CONFIG")
        self._btn_config.setFixedSize(70, 24)
        self._btn_config.setStyleSheet(
            f"font-size: 10px; font-weight: bold; background-color: {BG_DARK}; color: {ACCENT}; "
            f"border: 1px solid {BG_LIGHT}; border-radius: 4px;"
        )
        self._btn_config.setCursor(QtGui.QCursor(QtCore.Qt.CursorShape.PointingHandCursor))
        self._btn_config.clicked.connect(self._toggle_config)
        hdr.addWidget(self._btn_config)
        lay.addLayout(hdr)
        lay.addWidget(_divider())

        # Config Area (Hidden by default)
        self._config_widget = QtWidgets.QWidget()
        self._config_widget.setVisible(False)
        config_lay = QtWidgets.QGridLayout(self._config_widget)
        config_lay.setContentsMargins(0, 5, 0, 5)
        config_lay.setSpacing(6)
        
        self._spinners = []
        for i in range(n_channels):
            lbl = QtWidgets.QLabel(f"Plot {i+1} ← CH:")
            lbl.setStyleSheet(f"font-size: 11px; color: {TEXT_DIM};")
            spin = QtWidgets.QSpinBox()
            spin.setRange(0, 64)
            spin.setValue(self.channel_indices[i])
            spin.setStyleSheet(
                f"background: {BG_DARK}; color: {TEXT_MAIN}; border: 1px solid {BG_LIGHT}; "
                f"border-radius: 3px; padding: 2px; min-width: 40px;"
            )
            spin.valueChanged.connect(self._update_indices)
            config_lay.addWidget(lbl, i // 2, (i % 2) * 2)
            config_lay.addWidget(spin, i // 2, (i % 2) * 2 + 1)
            self._spinners.append(spin)
            
        # Diagnostic Info Label
        self._diagnostics_lbl = QtWidgets.QLabel("Detecting TMSi Channels...")
        self._diagnostics_lbl.setWordWrap(True)
        self._diagnostics_lbl.setStyleSheet(f"font-size: 9px; color: {TEXT_DIM}; padding-top: 5px;")
        lay.addWidget(self._config_widget)
        lay.addWidget(self._diagnostics_lbl)
        self._diagnostics_lbl.setVisible(False)

        # Main Plotting Area
        self.graphics_layout = pg.GraphicsLayoutWidget()
        self.graphics_layout.setBackground(BG_DARK)
        lay.addWidget(self.graphics_layout, stretch=1)
        
        self.plots = []
        self.curves = []
        self.colors = [ACCENT, GREEN, ORANGE, RED, PURPLE, "#1abc9c", "#e67e22", "#ecf0f1"]
        
        for i in range(n_channels):
            p = self.graphics_layout.addPlot(row=i, col=0)
            name = EMG_MUSCLE_NAMES[i] if i < len(EMG_MUSCLE_NAMES) else f"Muscle {i+1}"
            p.setLabel("left", name, size="7pt")
            p.showGrid(x=True, y=True, alpha=0.1)
            p.getAxis("bottom").setHeight(0) if i < n_channels - 1 else p.setLabel("bottom", "s", size="8pt")
            p.getAxis("left").setWidth(45)
            p.getAxis("left").setTextPen(TEXT_DIM)
            p.getAxis("bottom").setTextPen(TEXT_DIM)
            p.setTitle(None)
            
            curve = p.plot(pen=pg.mkPen(self.colors[i % len(self.colors)], width=1.5))
            self.plots.append(p)
            self.curves.append(curve)
            if i > 0:
                p.setXLink(self.plots[0])

        # Buffers
        self.PLOT_WINDOW_SECONDS = 2.0
        self.max_pts = int(self.PLOT_WINDOW_SECONDS * 2000) + 1000
        self.time_buf = deque(maxlen=self.max_pts)
        self.data_bufs = [deque(maxlen=self.max_pts) for _ in range(n_channels)]
        
        self._emg_idx = 0
        self.processor = None
        self.t0 = None
        self.channel_names = []

    def _toggle_config(self):
        visible = self._config_widget.isVisible()
        self._config_widget.setVisible(not visible)
        self._btn_config.setText("⚙ CLOSE" if not visible else "⚙ CONFIG")
        self._diagnostics_lbl.setVisible(not visible)

    def _update_indices(self):
        new_indices = [s.value() for s in self._spinners]
        changed_idx = None
        for i, (old, new) in enumerate(zip(self.channel_indices, new_indices)):
            if old != new:
                changed_idx = i
                break
        
        self.channel_indices = new_indices
        for i, idx in enumerate(self.channel_indices):
            name = self.channel_names[idx] if idx < len(self.channel_names) else f"CH{idx+1}"
            if i < len(EMG_MUSCLE_NAMES):
                name = EMG_MUSCLE_NAMES[i]
            self.plots[i].setLabel("left", name, size="7pt")
            
        if changed_idx is not None:
            self._pending_reset_idx = changed_idx
            current_len = len(self.time_buf)
            self.data_bufs[changed_idx] = deque([0.0] * current_len, maxlen=self.max_pts)

    def update_from_thread(self, tmsi_thread, processor=None):
        if tmsi_thread is None:
            return
            
        if not self.channel_names and hasattr(tmsi_thread, 'channels') and tmsi_thread.channels:
            self.channel_names = tmsi_thread.channels
            diag_text = "Available: " + ", ".join([f"[{i}]{n}" for i, n in enumerate(self.channel_names)])
            self._diagnostics_lbl.setText(diag_text)
            self._update_indices()
            
        if self.processor is None:
            self.processor = processor

        with tmsi_thread._lock:
            new_chunks = tmsi_thread._history[self._emg_idx:]
            self._emg_idx = len(tmsi_thread._history)

        if not new_chunks:
            return

        chunks_t = []
        chunks_v = []
        
        for t_arr, samp in new_chunks:
            if samp.ndim != 2:
                continue
            n_samples = samp.shape[0]
            n_cols = samp.shape[1]
            selected_samp = np.zeros((n_samples, self.n_channels), dtype=np.float32)
            for i, idx in enumerate(self.channel_indices):
                if 0 <= idx < n_cols:
                    selected_samp[:, i] = samp[:, idx]
            chunks_t.append(t_arr)
            chunks_v.append(selected_samp)

        if not chunks_t:
            return

        t_batch = np.concatenate(chunks_t)
        v_batch = np.concatenate(chunks_v)
        
        if self.t0 is None:
            self.t0 = t_batch[0]
            
        if hasattr(self, '_pending_reset_idx') and self._pending_reset_idx is not None:
            idx = self._pending_reset_idx
            self._pending_reset_idx = None
            if self.processor is not None:
                self.processor.reset(channel_idx=idx, warm_value=float(v_batch[0, idx]))
        
        self.time_buf.extend(t_batch)
        
        if self.processor is not None:
            import warnings
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", RuntimeWarning)
                _, envelopes = self.processor.process(v_batch, return_envelope=True)
            for i in range(self.n_channels):
                self.data_bufs[i].extend(envelopes[:, i])
        else:
            rectified = np.abs(v_batch)
            for i in range(self.n_channels):
                self.data_bufs[i].extend(rectified[:, i])
            
        if len(self.time_buf) > 0:
            t_arr = np.array(self.time_buf)
            t_rel = t_arr - self.t0
            x_max = t_rel[-1]
            x_min = max(0, x_max - self.PLOT_WINDOW_SECONDS)
            for i in range(self.n_channels):
                self.curves[i].setData(t_rel, np.array(self.data_bufs[i]))
            self.plots[0].setXRange(x_min, x_max)


# ---------------------------------------------------------------------------
# Panel: IMU 3-D (reuse helpers from trial/dashboard.py inline)
# ---------------------------------------------------------------------------

def _euler_to_quat(roll_deg, pitch_deg, yaw_deg) -> np.ndarray:
    r, p, y = np.radians(roll_deg), np.radians(pitch_deg), np.radians(yaw_deg)
    cr, sr = np.cos(r/2), np.sin(r/2)
    cp, sp = np.cos(p/2), np.sin(p/2)
    cy, sy = np.cos(y/2), np.sin(y/2)
    return np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ])


def _make_cube_mesh():
    colors_pairs = [
        (np.array([231,76,60,255])/255,  np.array([192,57,43,255])/255),
        (np.array([46,204,113,255])/255, np.array([39,174,96,255])/255),
        (np.array([52,152,219,255])/255, np.array([41,128,185,255])/255),
    ]
    normals = [
        (np.array([1,0,0]),  colors_pairs[0][0]),
        (np.array([-1,0,0]), colors_pairs[0][1]),
        (np.array([0,1,0]),  colors_pairs[1][0]),
        (np.array([0,-1,0]), colors_pairs[1][1]),
        (np.array([0,0,1]),  colors_pairs[2][0]),
        (np.array([0,0,-1]), colors_pairs[2][1]),
    ]
    s = 0.5
    verts, cols = [], []
    for normal, color in normals:
        if abs(normal[0]) > 0.9: u, v = np.array([0,1,0]), np.array([0,0,1])
        elif abs(normal[1]) > 0.9: u, v = np.array([1,0,0]), np.array([0,0,1])
        else: u, v = np.array([1,0,0]), np.array([0,1,0])
        c = normal * s
        p1,p2,p3,p4 = c-u*s-v*s, c+u*s-v*s, c+u*s+v*s, c-u*s+v*s
        for tri in ([p1,p2,p3],[p1,p3,p4]):
            verts.append(tri)
            cols.append([color,color,color])
    return np.array(verts,np.float32), np.array(cols,np.float32)


class _ImuCube:
    _VERTS = _COLS = None

    def __init__(self, view, position=(0,0,0)):
        if _ImuCube._VERTS is None:
            _ImuCube._VERTS, _ImuCube._COLS = _make_cube_mesh()
        self._view = view
        self._base_pos = np.array(position, float)
        self._container = gl.GLAxisItem(size=QtGui.QVector3D(0.01,0.01,0.01), glOptions='opaque')
        self._container.translate(*position)
        view.addItem(self._container)
        self._mesh = gl.GLMeshItem(
            vertexes=self._VERTS, vertexColors=self._COLS,
            smooth=False, shader='balloon', drawEdges=True, edgeColor=(0,0,0,0.5),
        )
        self._mesh.setParentItem(self._container)
        self._axes = gl.GLAxisItem(size=QtGui.QVector3D(1.5,1.5,1.5))
        self._axes.setParentItem(self._container)

    def update(self, q: np.ndarray):
        w, x, y, z = q
        angle = np.degrees(2 * np.arccos(np.clip(w,-1,1)))
        s = np.sqrt(max(0.0, 1-w*w))
        ax, ay, az = (x/s,y/s,z/s) if s > 1e-3 else (1,0,0)
        self._container.resetTransform()
        self._container.translate(*self._base_pos)
        self._container.rotate(angle, ax, ay, az)


class _IMUPanel(QtWidgets.QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {BG_MID}; border-radius: 6px;")

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(6,6,6,6)
        lay.setSpacing(4)

        hdr = QtWidgets.QHBoxLayout()
        hdr.addWidget(_section_label("IMU Orientation (3-D)"))
        hdr.addStretch()
        self._led1, self._led2 = _led(RED,12), _led(RED,12)
        for led, txt in [(self._led1,"IMU 1"),(self._led2,"IMU 2")]:
            w = QtWidgets.QWidget()
            rl = QtWidgets.QHBoxLayout(w); rl.setContentsMargins(0,0,0,0); rl.setSpacing(4)
            rl.addWidget(led)
            l = QtWidgets.QLabel(txt); l.setStyleSheet(f"font-size:10px;color:{TEXT_DIM};")
            rl.addWidget(l); hdr.addWidget(w)
        lay.addLayout(hdr)
        lay.addWidget(_divider())
        # State tracking for debouncing
        self._is_live_stable = False
        self._sync_loss_counter = 0
        self._SYNC_DEBOUNCE_TICKS = 15  # ~500ms at 30fps

        if gl is not None:
            self._gl = gl.GLViewWidget()
            self._gl.setBackgroundColor(BG_DARK)
            self._gl.setCameraPosition(distance=8, elevation=30, azimuth=45)
            grid_item = gl.GLGridItem(); grid_item.setSize(x=20,y=20); grid_item.setSpacing(x=1,y=1,z=1)
            self._gl.addItem(grid_item)
            self._cube1 = _ImuCube(self._gl, position=(-2,0,0))
            self._cube2 = _ImuCube(self._gl, position=( 2,0,0))
            lay.addWidget(self._gl, stretch=1)
            self._ok = True
        else:
            lay.addWidget(QtWidgets.QLabel("OpenGL not available"))
            self._ok = False

    def update_imu(self, euler1, euler2, ok1: bool, ok2: bool):
        color1 = GREEN if ok1 else RED
        color2 = GREEN if ok2 else RED
        self._led1.setStyleSheet(f"background-color:{color1};border-radius:6px;")
        self._led2.setStyleSheet(f"background-color:{color2};border-radius:6px;")
        if not self._ok:
            return
        if euler1 is not None:
            q1 = _euler_to_quat(*euler1)
            self._cube1.update(q1)
        if euler2 is not None:
            q2 = _euler_to_quat(*euler2)
            self._cube2.update(q2)


# ---------------------------------------------------------------------------
# Panel: Model Info sidebar
# ---------------------------------------------------------------------------

class _ModelInfoPanel(QtWidgets.QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {BG_MID}; border-radius: 6px;")

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(4)
        lay.addWidget(_section_label("Model Info"))
        lay.addWidget(_divider())

        self._lbl_arch = self._row(lay, "Architecture", "—")
        self._lbl_channels = self._row(lay, "Channels", "—")
        self._lbl_filters = self._row(lay, "CNN Filters", "—")
        self._lbl_lstm = self._row(lay, "LSTM", "—")
        self._lbl_window = self._row(lay, "Window", "—")
        self._lbl_step = self._row(lay, "Step", "—")
        self._lbl_mae = self._row(lay, "MAE", "—")
        self._lbl_r2 = self._row(lay, "R²", "—")

        lay.addWidget(_divider())
        lay.addWidget(_section_label("Per-Weight MAE (CV)"))
        self._weight_table = QtWidgets.QLabel("—")
        self._weight_table.setStyleSheet(
            f"font-family: 'Consolas'; font-size: 9px; color: {TEXT_DIM};"
        )
        self._weight_table.setWordWrap(True)
        lay.addWidget(self._weight_table)

        lay.addStretch()

    @staticmethod
    def _row(parent_lay, label: str, default: str) -> "QtWidgets.QLabel":
        row = QtWidgets.QHBoxLayout()
        lbl = QtWidgets.QLabel(label + ":")
        lbl.setStyleSheet(f"font-size: 10px; color: {TEXT_DIM}; font-weight: bold;")
        val = QtWidgets.QLabel(default)
        val.setStyleSheet(f"font-family: 'Consolas'; font-size: 10px; color: {TEXT_MAIN};")
        row.addWidget(lbl); row.addWidget(val); row.addStretch()
        parent_lay.addLayout(row)
        return val

    def populate(self, metadata, window_s: float, step_s: float):
        """Fill in values from ModelMetadata."""
        self._lbl_arch.setText("CNN-LSTM")
        self._lbl_channels.setText(str(metadata.n_channels))
        self._lbl_filters.setText(str(metadata.cnn_filters))
        self._lbl_lstm.setText(
            f"h={metadata.lstm_hidden_size} ×{metadata.lstm_num_layers}"
        )
        self._lbl_window.setText(f"{window_s:.2f} s")
        self._lbl_step.setText(f"{step_s:.1f} s")

        if metadata.mae is not None:
            self._lbl_mae.setText(f"{metadata.mae:.3f} ±{metadata.mae_std:.3f} kg")
        if metadata.r2 is not None:
            self._lbl_r2.setText(f"{metadata.r2:.3f} ±{metadata.r2_std:.3f}")

        if metadata.per_weight:
            lines = []
            for w, mae in sorted(metadata.per_weight.items()):
                lines.append(f"{w:.2f} kg → MAE {mae:.3f} kg")
            self._weight_table.setText("\n".join(lines))


# ---------------------------------------------------------------------------
# Demo data generator
# ---------------------------------------------------------------------------

class _DemoDataGenerator:
    """Generates synthetic EMG + IMU data for testing without hardware."""

    _WEIGHTS = [0.0, 0.899, 0.979, 1.966, 2.238, 2.945, 4.142, 5.922]

    def __init__(self, emg_fs: int = 2000):
        self._fs = emg_fs
        self._t0 = time.perf_counter()
        self._weight_idx = 0
        self._weight_change_t = self._t0

    def get_imu(self):
        t = time.perf_counter() - self._t0
        roll1  = 20 * math.sin(t * 0.3)
        pitch1 = 15 * math.cos(t * 0.2)
        yaw1   = t * 10 % 360
        roll2  = 10 * math.sin(t * 0.4 + 1)
        pitch2 = 8  * math.cos(t * 0.3 + 0.5)
        yaw2   = (t * 8 + 30) % 360
        ok1, ok2 = True, True
        return (roll1, pitch1, yaw1), (roll2, pitch2, yaw2), ok1, ok2

    def get_prediction(self):
        """Cycle through known weights every 5 s."""
        t = time.perf_counter()
        if t - self._weight_change_t > 5.0:
            self._weight_idx = (self._weight_idx + 1) % len(self._WEIGHTS)
            self._weight_change_t = t
        w = self._WEIGHTS[self._weight_idx]
        noise = np.random.normal(0, 0.05)
        raw = max(0.0, w + noise)
        return raw

    def get_emg_chunk(self, n: int = 64):
        n_ch = 38
        return np.random.randn(n, n_ch).astype(np.float32) * 1e-5

    def get_stm32_rate(self):
        return 500.0

    def get_emg_rate(self):
        return 2000.0


# ---------------------------------------------------------------------------
# Main Dashboard window
# ---------------------------------------------------------------------------

class InferenceDashboard(QtWidgets.QMainWindow):
    """
    Main window for the real-time inference dashboard.

    Parameters
    ----------
    engine       : InferenceEngine (or None in demo mode)
    buffer       : SignalBuffer (or None in demo mode)
    estimator    : SyncDelayEstimator (or None in demo mode)
    stm32_thread : RawSTM32Thread (or None in demo mode)
    tmsi_thread  : RawTMSiThread  (or None in demo mode)
    model        : ModelLoader (required — provides metadata)
    window_s     : float — inference window duration
    step_s       : float — re-inference interval
    demo         : bool  — run with synthetic data
    """

    def __init__(
        self,
        engine=None,
        buffer=None,
        estimator=None,
        stm32_thread=None,
        tmsi_thread=None,
        model=None,
        window_s: float = 1.2,
        step_s: float = 2.0,
        demo: bool = False,
        parent=None,
    ):
        super().__init__(parent)
        self._engine = engine
        self._buffer = buffer
        self._estimator = estimator
        self._stm32 = stm32_thread
        self._tmsi = tmsi_thread
        self._model = model
        self._window_s = window_s
        self._step_s = step_s
        self._demo = demo

        if demo:
            self._demo_gen = _DemoDataGenerator()
            self._demo_ema = None
            self._demo_alpha = 0.4

        self.setWindowTitle("CNN-LSTM — Real-Time Weight Inference")
        self.resize(1600, 950)
        self.setStyleSheet(_COMMON_STYLE)

        # ── Central widget ──────────────────────────────────────────────
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # ── Header ──────────────────────────────────────────────────────
        run_dir_str = str(model.run_dir) if model else "demo mode"
        self._header = _HeaderPanel(run_dir=run_dir_str)
        self._header.stopRequested.connect(self.close)
        root.addWidget(self._header)

        # ── Main content (horizontal) ───────────────────────────────────
        main_row = QtWidgets.QHBoxLayout()
        main_row.setSpacing(6)
        root.addLayout(main_row, stretch=1)

        # Left column
        left_col = QtWidgets.QVBoxLayout()
        left_col.setSpacing(6)

        self._health = _SensorHealthPanel()
        left_col.addWidget(self._health)

        self._sync_panel = _SyncPanel()
        self._sync_panel.setFixedHeight(90)
        left_col.addWidget(self._sync_panel)

        self._hero = _PredictionHeroPanel()
        left_col.addWidget(self._hero, stretch=1)

        self._model_info = _ModelInfoPanel()
        left_col.addWidget(self._model_info)
        main_row.addLayout(left_col, stretch=3)

        # Centre column: IMU & Prediction History
        centre_col = QtWidgets.QVBoxLayout()
        centre_col.setSpacing(6)

        self._history_panel = _PredictionHistoryPanel(history_s=60.0)
        centre_col.addWidget(self._history_panel, stretch=2)

        self._imu_panel = _IMUPanel()
        centre_col.addWidget(self._imu_panel, stretch=3)
        main_row.addLayout(centre_col, stretch=5)

        # Right column: EMG
        self._emg_panel = _EMGPanel(n_channels=8)
        main_row.addWidget(self._emg_panel, stretch=2)

        # ── Populate static model info ──────────────────────────────────
        if model is not None:
            self._model_info.populate(model.metadata, window_s, step_s)

        # ── Timers ──────────────────────────────────────────────────────
        self._fast_timer = QtCore.QTimer(self)
        self._fast_timer.timeout.connect(self._fast_tick)
        self._fast_timer.start(FAST_REFRESH_MS)

        self._plot_timer = QtCore.QTimer(self)
        self._plot_timer.timeout.connect(self._plot_tick)
        self._plot_timer.start(PLOT_REFRESH_MS)

        self._imu_timer = QtCore.QTimer(self)
        self._imu_timer.timeout.connect(self._imu_tick)
        self._imu_timer.start(IMU_REFRESH_MS)

        # Demo prediction deque
        self._demo_history: List[Tuple[float, float, float]] = []

        # State tracking for debouncing
        self._is_live_stable = False
        self._sync_loss_counter = 0
        self._SYNC_DEBOUNCE_TICKS = 15  # ~500ms at 30fps

    # ------------------------------------------------------------------
    # Timer callbacks
    # ------------------------------------------------------------------

    def _fast_tick(self):
        """30 fps — header, sensor health, hero readout."""
        is_live = False

        if self._demo:
            # ── Demo mode ──────────────────────────────────────────────
            raw = self._demo_gen.get_prediction()
            if self._demo_ema is None:
                self._demo_ema = raw
            else:
                self._demo_ema = self._demo_alpha * raw + (1 - self._demo_alpha) * self._demo_ema
            smoothed = self._demo_ema
            now = time.perf_counter()
            self._demo_history.append((now, raw, smoothed))

            self._hero.update(
                smoothed_kg=smoothed, raw_kg=raw,
                confidence=0.85, ema_alpha=self._demo_alpha,
                last_t=now, zero_filled=[], is_synced=True,
            )
            self._health.update_stm32(True, 500.0)
            self._health.update_tmsi(True, 2000.0)
            self._health.update_imu1(True)
            self._health.update_imu2(True)
            self._health.update_sync(0.85, -12.3)
            is_live = True

        else:
            # ── Hardware mode ──────────────────────────────────────────
            # Sensor health — use buffer rate properties for reliability
            stm32_ok = (self._stm32 is not None and self._stm32.running)
            tmsi_ok  = (self._tmsi is not None and self._tmsi.running)

            if self._buffer is not None:
                stm32_rate = self._buffer.stm32_rate_hz
                tmsi_rate  = self._buffer.emg_rate_hz
            else:
                stm32_rate = 0.0
                tmsi_rate  = getattr(self._tmsi, 'estimated_rate_hz', 0.0) if self._tmsi else 0.0

            self._health.update_stm32(stm32_ok, stm32_rate)
            self._health.update_tmsi(tmsi_ok, tmsi_rate)

            # Health lights & rate check
            imu1_ok = imu2_ok = False
            imu1_stuck = imu2_stuck = False
            if self._stm32 is not None and self._stm32.running:
                try:
                    with self._stm32._lock:
                        hist = self._stm32._history
                        last_entry = hist[-1] if hist else None
                    if last_entry is not None:
                        _, last_s = last_entry if isinstance(last_entry, tuple) else (0, last_entry)
                        imu1_ok = bool(getattr(last_s, 'imu1_ok', 0))
                        imu2_ok = bool(getattr(last_s, 'imu2_ok', 0))
                        
                        # Freeze detection (as in trial dashboard)
                        e1 = (getattr(last_s,'roll1',0), getattr(last_s,'pitch1',0), getattr(last_s,'yaw1',0))
                        e2 = (getattr(last_s,'roll2',0), getattr(last_s,'pitch2',0), getattr(last_s,'yaw2',0))
                        # Note: we should have these on self to persist across ticks
                        if not hasattr(self, '_imu1_last_e'): self._imu1_last_e = e1; self._imu1_freeze_t = time.perf_counter()
                        if not hasattr(self, '_imu2_last_e'): self._imu2_last_e = e2; self._imu2_freeze_t = time.perf_counter()
                        
                        now = time.perf_counter()
                        if e1 != self._imu1_last_e: self._imu1_last_e = e1; self._imu1_freeze_t = now
                        if e2 != self._imu2_last_e: self._imu2_last_e = e2; self._imu2_freeze_t = now
                        if now - self._imu1_freeze_t > 1.0: imu1_stuck = True
                        if now - self._imu2_freeze_t > 1.0: imu2_stuck = True
                except (IndexError, RuntimeError):
                    pass
            
            self._health.update_stm32(stm32_ok, stm32_rate)
            self._health.update_tmsi(tmsi_ok, tmsi_rate)
            self._health.update_imu1(imu1_ok, imu1_stuck)
            self._health.update_imu2(imu2_ok, imu2_stuck)

            # PRBS sync
            if self._estimator:
                self._sync_panel.update(self._estimator)
                result = self._estimator.get_result()
                conf = result.confidence if result else 0.0
            else:
                conf = 0.0

            # Prediction
            raw_kg = sm_kg = None
            last_t = 0.0
            if self._engine:
                raw_kg, sm_kg, last_t = self._engine.get_prediction()
                is_live = self._engine.is_synced and sm_kg is not None
            else:
                is_live = False

            zero_filled: List[str] = []
            if self._buffer:
                zero_filled = list(self._buffer.zero_filled_channels)

            self._hero.update(
                smoothed_kg=sm_kg, raw_kg=raw_kg,
                confidence=conf,
                ema_alpha=self._engine.ema_alpha if self._engine else 0.4,
                last_t=last_t, zero_filled=zero_filled, is_synced=is_live,
            )

        self._header.tick(is_live=is_live)

    def _plot_tick(self):
        """10 fps — EMG scrolling plots, prediction history."""
        if self._demo:
            # Feed synthetic EMG
            chunk = self._demo_gen.get_emg_chunk(64)
            t_arr = np.linspace(time.perf_counter() - 0.032, time.perf_counter(), 64)
            self._emg_panel.push_emg_chunk(t_arr, chunk)
            self._emg_panel.refresh()
            self._history_panel.update_history(self._demo_history[-150:])
        else:
            # Feed real EMG directly from tmsi_thread using the robust trial logic
            if self._tmsi:
                # We reuse the board's stateful processor if available
                # or the dashboard's internal one from the panel.
                proc = getattr(self._buffer, '_processor', None) if self._buffer else None
                self._emg_panel.update_from_thread(self._tmsi, processor=proc)

            if self._engine:
                self._history_panel.update_history(self._engine.get_history())

    def _imu_tick(self):
        """15 fps — IMU 3D cubes."""
        if self._demo:
            e1, e2, ok1, ok2 = self._demo_gen.get_imu()
            self._imu_panel.update_imu(e1, e2, ok1, ok2)
        else:
            if self._stm32 and self._stm32._history:
                with self._stm32._lock:
                    last_entry = self._stm32._history[-1]
                _, s = last_entry if isinstance(last_entry, tuple) else (0, last_entry)
                e1 = (getattr(s,'roll1',0), getattr(s,'pitch1',0), getattr(s,'yaw1',0))
                e2 = (getattr(s,'roll2',0), getattr(s,'pitch2',0), getattr(s,'yaw2',0))
                ok1 = bool(getattr(s,'imu1_ok',0))
                ok2 = bool(getattr(s,'imu2_ok',0))
                self._imu_panel.update_imu(e1, e2, ok1, ok2)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def closeEvent(self, event):
        self._fast_timer.stop()
        self._plot_timer.stop()
        self._imu_timer.stop()
        event.accept()


# ---------------------------------------------------------------------------
# Launch helper
# ---------------------------------------------------------------------------

def launch_dashboard(
    engine=None,
    buffer=None,
    estimator=None,
    stm32_thread=None,
    tmsi_thread=None,
    model=None,
    window_s: float = 1.2,
    step_s: float = 2.0,
    demo: bool = False,
):
    """Create and show the dashboard. Blocks until the window is closed."""
    if not _QT_OK:
        print("ERROR: PyQt6 not available.")
        return

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    win = InferenceDashboard(
        engine=engine, buffer=buffer, estimator=estimator,
        stm32_thread=stm32_thread, tmsi_thread=tmsi_thread,
        model=model, window_s=window_s, step_s=step_s, demo=demo,
    )
    win.show()
    app.exec()
