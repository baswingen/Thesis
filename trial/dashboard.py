"""
Trial Dashboard
===============

Comprehensive PyQt6 dashboard for real-time monitoring during an experimental trial.

Panels:
  - Trial Status Header  (participant, trial #, elapsed time, REC indicator)
  - Sensor Health        (STM32 online, IMU1/IMU2 LEDs, TMSi online)
  - Button Matrix 3×4   (based on src/button_matrix.py)
  - Trial Instruction    (large centred instruction text shown above the matrix)
  - IMU Panel            (dual 3-D OpenGL cubes, forward-kinematics arm, accel plots)
  - PRBS Sync Panel      (compact metric strip — delay, confidence, drift, updates)
  - EMG Channels Panel   (small scrolling plot per channel, default first 8)

Integrates with TrialManager from trial/setup_trial.py.

Usage (demo mode — no hardware required):
    python trial/dashboard.py --demo
"""

from __future__ import annotations

import sys
import time
import threading
import warnings
from collections import deque
from pathlib import Path
from typing import Optional, List, Any, TYPE_CHECKING

import numpy as np

try:
    from trial import trial_config
except ImportError:
    trial_config = None

# ──────────────────────────────────────────────────────────────────────────────
# Path setup
# ──────────────────────────────────────────────────────────────────────────────
_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

# ──────────────────────────────────────────────────────────────────────────────
# Qt / pyqtgraph imports
# ──────────────────────────────────────────────────────────────────────────────
try:
    from PyQt6 import QtWidgets, QtCore, QtGui
    import pyqtgraph as pg
    import pyqtgraph.opengl as gl
    pg.setConfigOptions(antialias=True, useOpenGL=True)
    _QT_OK = True
except ImportError:
    _QT_OK = False
    gl = None
    print("ERROR: PyQt6 + pyqtgraph are required.  pip install PyQt6 pyqtgraph PyOpenGL")

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────
BUTTON_ROWS = 3
BUTTON_COLS = 4
EMG_SCROLL_S = 2.0          # seconds of EMG data to display per channel
IMU_SCROLL_S = 4.0          # seconds of IMU data to display
SYNC_HISTORY_MAX = 4000     # max samples in delay / confidence history arrays
FAST_REFRESH_MS  = 33       # header / health / matrix (≈30 fps)
PLOT_REFRESH_MS  = 66       # pyqtgraph setData calls  (≈15 fps) – halved to reduce render pressure

# Design tokens (matching existing scripts)
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

_COMMON_STYLE = f"""
    QWidget        {{ background-color: {BG_DARK}; color: {TEXT_MAIN}; font-family: 'Segoe UI', sans-serif; }}
    QLabel         {{ color: {TEXT_MAIN}; }}
    QFrame         {{ border: none; }}
    QPushButton    {{ background-color: {BG_MID}; color: {TEXT_MAIN}; border: none;
                      padding: 8px 16px; border-radius: 5px; font-weight: bold; }}
    QPushButton:hover {{ background-color: {BG_LIGHT}; }}
    QScrollArea    {{ border: none; }}
"""


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────
def _section_label(text: str) -> QtWidgets.QLabel:
    """Bold, accented section header."""
    lbl = QtWidgets.QLabel(text.upper())
    lbl.setStyleSheet(
        f"font-weight: bold; font-size: 11px; color: {ACCENT}; "
        f"letter-spacing: 1px; padding: 2px 0px;"
    )
    return lbl


def _divider() -> QtWidgets.QFrame:
    f = QtWidgets.QFrame()
    f.setFrameShape(QtWidgets.QFrame.Shape.HLine)
    f.setStyleSheet(f"background-color: {BG_MID}; max-height: 1px;")
    return f


def _styled_plot(title: str = "", y_label: str = "") -> pg.PlotWidget:
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


def _led(color: str = RED, size: int = 14) -> QtWidgets.QLabel:
    lbl = QtWidgets.QLabel()
    lbl.setFixedSize(size, size)
    lbl.setStyleSheet(
        f"background-color: {color}; border-radius: {size // 2}px;"
    )
    return lbl


# ──────────────────────────────────────────────────────────────────────────────
# Panel: Trial Status Header
# ──────────────────────────────────────────────────────────────────────────────
class _HeaderPanel(QtWidgets.QFrame):
    stopRequested = QtCore.pyqtSignal()

    def __init__(self, participant_id: str = "P00", trial_num: int = 0, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {BG_MID}; border-radius: 6px;")
        self._start_time = time.perf_counter()
        self._rec_state = True

        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(14, 10, 14, 10)

        # Left: participant info
        info_lay = QtWidgets.QVBoxLayout()
        self._lbl_id = QtWidgets.QLabel(
            f"<b>Participant {participant_id}</b> — Trial {trial_num}"
        )
        self._lbl_id.setStyleSheet(f"font-size: 14px; color: {TEXT_MAIN};")
        self._lbl_time = QtWidgets.QLabel("00:00")
        self._lbl_time.setStyleSheet(
            f"font-size: 22px; font-weight: bold; color: {TEXT_MAIN}; font-family: 'Consolas', monospace;"
        )
        info_lay.addWidget(self._lbl_id)
        info_lay.addWidget(self._lbl_time)
        lay.addLayout(info_lay)

        lay.addStretch()

        # Centre: REC indicator
        self._lbl_rec = QtWidgets.QLabel("● REC")
        self._lbl_rec.setStyleSheet(
            f"font-size: 16px; font-weight: bold; color: {RED}; letter-spacing: 2px;"
        )
        lay.addWidget(self._lbl_rec)
        lay.addSpacing(20)

        # Right: STOP button
        self._btn_stop = QtWidgets.QPushButton("⏹  STOP TRIAL")
        self._btn_stop.setStyleSheet(
            f"background-color: {RED}; color: white; font-weight: bold; "
            f"font-size: 13px; padding: 10px 22px; border-radius: 6px;"
        )
        self._btn_stop.setCursor(
            QtGui.QCursor(QtCore.Qt.CursorShape.PointingHandCursor)
        )
        self._btn_stop.clicked.connect(self.stopRequested)
        lay.addWidget(self._btn_stop)

    def tick(self):
        elapsed = time.perf_counter() - self._start_time
        m = int(elapsed) // 60
        s = int(elapsed) % 60
        self._lbl_time.setText(f"{m:02d}:{s:02d}")
        # Blink REC every 500 ms
        self._rec_state = not self._rec_state
        self._lbl_rec.setVisible(self._rec_state)

    def set_start_time(self, t: float):
        self._start_time = t


# ──────────────────────────────────────────────────────────────────────────────
# Panel: Sensor Health
# ──────────────────────────────────────────────────────────────────────────────
class _SensorHealthPanel(QtWidgets.QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {BG_MID}; border-radius: 6px;")

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
            l.addWidget(led)
            l.addWidget(lbl)
            l.addWidget(txt)
            l.addStretch()
            return w, led, txt

        w1, self._stm32_led, self._stm32_txt = _cell("STM32")
        w2, self._tmsi_led,  self._tmsi_txt  = _cell("EMG")
        w3, self._imu1_led,  self._imu1_txt  = _cell("IMU 1")
        w4, self._imu2_led,  self._imu2_txt  = _cell("IMU 2")

        grid.addWidget(w1, 0, 0)
        grid.addWidget(w2, 0, 1)
        grid.addWidget(w3, 1, 0)
        grid.addWidget(w4, 1, 1)

        lay.addLayout(grid)

    def _set_led(self, led: QtWidgets.QLabel, ok: bool, color_override: str | None = None):
        color = color_override if color_override else (GREEN if ok else RED)
        # Use a constant radius for better visual consistency (avoids led.width() race/zeros)
        led.setStyleSheet(
            f"background-color: {color}; border: 1px solid rgba(255,255,255,0.1); border-radius: 8px;"
        )

    def update_stm32(self, online: bool, rate_hz: float = 0.0):
        # Activity-based: Online ONLY if thread is running AND rate > 200 Hz
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
        # Activity-based: Online ONLY if thread is running AND rate > 1000 Hz
        is_active = online and rate_hz > 1000
        self._set_led(self._tmsi_led, is_active)
        self._tmsi_txt.setText(f"{rate_hz:.0f} Hz" if is_active else "Offline")


# ──────────────────────────────────────────────────────────────────────────────
# Panel: Button Matrix 3×4 + Instruction
# ──────────────────────────────────────────────────────────────────────────────
class _ButtonMatrixPanel(QtWidgets.QFrame):
    """
    3×4 button matrix display. Reads key state from stm32 sample (keys_mask).
    The instruction label shows trial-level text (e.g. stimulus name).
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {BG_MID}; border-radius: 6px;")

        self._highlight: Optional[tuple] = None  # (row, col) for highlighted button

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 10)
        lay.setSpacing(8)
        lay.addWidget(_section_label("Button Matrix  3×4"))
        lay.addWidget(_divider())

        # Instruction display
        self._lbl_instruction = QtWidgets.QLabel("—")
        self._lbl_instruction.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self._lbl_instruction.setWordWrap(True)
        self._lbl_instruction.setStyleSheet(
            f"font-size: 18px; font-weight: bold; color: {TEXT_MAIN}; "
            f"background-color: {BG_DARK}; border-radius: 5px; padding: 8px;"
        )
        self._lbl_instruction.setMinimumHeight(50)
        lay.addWidget(self._lbl_instruction)

        # Grid
        grid_w = QtWidgets.QWidget()
        self._grid = QtWidgets.QGridLayout(grid_w)
        self._grid.setSpacing(10)
        lay.addWidget(grid_w)

        self._buttons: List[List[QtWidgets.QLabel]] = []
        for r in range(BUTTON_ROWS):
            row = []
            for c in range(BUTTON_COLS):
                row_letter = chr(ord("A") + r)
                col_number = c + 1
                lbl = QtWidgets.QLabel(f"{row_letter}{col_number}")
                lbl.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
                lbl.setFixedSize(72, 72)
                lbl.setStyleSheet(self._style(pressed=False, highlighted=False))
                font = QtGui.QFont("Segoe UI", 13)
                font.setBold(True)
                lbl.setFont(font)
                self._grid.addWidget(lbl, r, c)
                row.append(lbl)
            self._buttons.append(row)

        # Status bar
        self._lbl_status = QtWidgets.QLabel("Waiting…")
        self._lbl_status.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self._lbl_status.setStyleSheet(
            f"font-size: 10px; color: {TEXT_DIM}; "
            f"font-family: 'Consolas', monospace;"
        )
        lay.addWidget(self._lbl_status)

    @staticmethod
    def _style(pressed: bool, highlighted: bool) -> str:
        if highlighted and not pressed:
            return (
                f"background-color: #2980b9; color: white; "
                f"border: 2px solid {ACCENT}; border-radius: 10px;"
            )
        if pressed:
            return (
                f"background-color: {GREEN}; color: white; "
                f"border: 2px solid {GREEN_D}; border-radius: 10px;"
            )
        return (
            f"background-color: {BG_DARK}; color: {TEXT_DIM}; "
            f"border: 2px solid {BG_MID}; border-radius: 10px;"
        )

    def update_mask(self, keys_mask: int, rate_hz: float = 0.0, sample_count: int = 0):
        for r in range(BUTTON_ROWS):
            for c in range(BUTTON_COLS):
                idx = r * BUTTON_COLS + c
                pressed = bool((keys_mask >> idx) & 1)
                hl = self._highlight == (r, c)
                new_style = self._style(pressed, hl)
                # Only call setStyleSheet when state actually changed
                btn = self._buttons[r][c]
                if btn.styleSheet() != new_style:
                    btn.setStyleSheet(new_style)
        new_status = (
            f"Rate: {rate_hz:.0f} Hz  |  Mask: 0x{keys_mask:03X}  |  Samples: {sample_count}"
        )
        if self._lbl_status.text() != new_status:
            self._lbl_status.setText(new_status)

    def set_instruction(self, text: str):
        self._lbl_instruction.setText(text)

    def highlight_button(self, row: int, col: int):
        """Highlight a specific button (e.g. current stimulus)."""
        self._highlight = (row, col)

    def clear_highlight(self):
        self._highlight = None




# ──────────────────────────────────────────────────────────────────────────────
# 3-D IMU helpers  (ported from dual_BNO085_testing.py)
# ──────────────────────────────────────────────────────────────────────────────

def _euler_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Euler angles (degrees) → quaternion [w, x, y, z]."""
    r, p, y = np.radians(roll_deg), np.radians(pitch_deg), np.radians(yaw_deg)
    cr, sr = np.cos(r / 2), np.sin(r / 2)
    cp, sp = np.cos(p / 2), np.sin(p / 2)
    cy, sy = np.cos(y / 2), np.sin(y / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def _rotate_vector(v: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Rotate 3-vector v by quaternion q [w, x, y, z]."""
    w, x, y, z = q
    qv = np.array([x, y, z])
    return v + 2 * w * np.cross(qv, v) + 2 * np.cross(qv, np.cross(qv, v))


def _make_colored_cube_mesh():
    """Return (verts, colors) arrays for a unit cube with Flat-UI face colours."""
    c_red    = np.array([231,  76,  60, 255]) / 255.0
    c_red_d  = np.array([192,  57,  43, 255]) / 255.0
    c_green  = np.array([ 46, 204, 113, 255]) / 255.0
    c_green_d= np.array([ 39, 174,  96, 255]) / 255.0
    c_blue   = np.array([ 52, 152, 219, 255]) / 255.0
    c_blue_d = np.array([ 41, 128, 185, 255]) / 255.0
    faces = [
        (np.array([ 1, 0, 0]), c_red),    (np.array([-1, 0, 0]), c_red_d),
        (np.array([ 0, 1, 0]), c_green),   (np.array([ 0,-1, 0]), c_green_d),
        (np.array([ 0, 0, 1]), c_blue),    (np.array([ 0, 0,-1]), c_blue_d),
    ]
    s = 0.5
    verts, colors = [], []
    for normal, color in faces:
        if abs(normal[0]) > 0.9:   u, v = np.array([0,1,0]), np.array([0,0,1])
        elif abs(normal[1]) > 0.9: u, v = np.array([1,0,0]), np.array([0,0,1])
        else:                       u, v = np.array([1,0,0]), np.array([0,1,0])
        c = normal * s
        p1, p2, p3, p4 = c-u*s-v*s, c+u*s-v*s, c+u*s+v*s, c-u*s+v*s
        for tri in ([p1,p2,p3], [p1,p3,p4]):
            verts.append(tri)
            colors.append([color, color, color])
    return np.array(verts, dtype=np.float32), np.array(colors, dtype=np.float32)


class _ImuVisualizer3D:
    """
    A single IMU represented as a coloured cube in a GLViewWidget.
    Optionally draws a rod along +X to the next joint.
    """
    _CUBE_VERTS, _CUBE_COLORS = None, None  # class-level cache — built once

    def __init__(self, view, position=(0, 0, 0), rod_length: float = 0.0):
        if _ImuVisualizer3D._CUBE_VERTS is None:
            _ImuVisualizer3D._CUBE_VERTS, _ImuVisualizer3D._CUBE_COLORS = _make_colored_cube_mesh()
        self._view = view
        self._base_pos = np.array(position, dtype=float)
        self._q = np.array([1.0, 0.0, 0.0, 0.0])
        # Ghost container axis (invisible, used as transform parent)
        self._container = gl.GLAxisItem(
            size=QtGui.QVector3D(0.01, 0.01, 0.01), glOptions='opaque'
        )
        self._container.translate(*position)
        view.addItem(self._container)
        # Coloured cube mesh
        self._mesh = gl.GLMeshItem(
            vertexes=self._CUBE_VERTS,
            vertexColors=self._CUBE_COLORS,
            smooth=False, shader='balloon',
            drawEdges=True, edgeColor=(0, 0, 0, 0.5),
        )
        self._mesh.setParentItem(self._container)
        # Local axes
        self._axes = gl.GLAxisItem(size=QtGui.QVector3D(1.5, 1.5, 1.5))
        self._axes.setParentItem(self._container)
        # Optional rod
        if rod_length > 0:
            rod_w, rod_h = 0.15, 0.05
            rod = gl.GLBoxItem(
                size=QtGui.QVector3D(rod_length, rod_w, rod_h),
                color=(0.4, 0.4, 0.45, 1.0),
            )
            rod.translate(0, -rod_w / 2, -rod_h / 2)
            rod.setParentItem(self._container)

    def update(self, q: np.ndarray, pos: np.ndarray | None = None) -> None:
        w, x, y, z = q
        angle = np.degrees(2 * np.arccos(np.clip(w, -1.0, 1.0)))
        s = np.sqrt(max(0.0, 1 - w * w))
        if s < 1e-3:
            ax, ay, az = 1.0, 0.0, 0.0
        else:
            ax, ay, az = x / s, y / s, z / s
        self._container.resetTransform()
        target = pos if pos is not None else self._base_pos
        self._container.translate(*target)
        self._container.rotate(angle, ax, ay, az)


# ──────────────────────────────────────────────────────────────────────────────
# Panel: IMU — dual 3-D cubes  (replaces old 2-D scrolling _IMUPanel)
# ──────────────────────────────────────────────────────────────────────────────
class _IMUPanel3D(QtWidgets.QFrame):
    """
    Middle-column panel that mirrors the dual_BNO085_testing.py visualizer:
      - GLViewWidget with two coloured 3-D cubes (forward-kinematics arm)
      - Two compact acceleration plots (X/Y/Z) below the 3-D view
    Falls back gracefully to a placeholder label if PyOpenGL is missing.
    """

    _ARM_LEN = 2.5   # upper-arm / lower-arm segment length (world units)
    _BUF     = 200   # acceleration history depth

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {BG_MID}; border-radius: 6px;")

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(4)

        # Header row with section label + IMU status LEDs
        hdr = QtWidgets.QHBoxLayout()
        hdr.addWidget(_section_label("IMU Orientation  (3-D)"))
        hdr.addStretch()
        self._led1 = _led(RED, 12)
        self._led2 = _led(RED, 12)
        for led, lbl_txt in [(self._led1, "IMU 1"), (self._led2, "IMU 2")]:
            w = QtWidgets.QWidget()
            rl = QtWidgets.QHBoxLayout(w)
            rl.setContentsMargins(0, 0, 0, 0)
            rl.setSpacing(4)
            rl.addWidget(led)
            lbl = QtWidgets.QLabel(lbl_txt)
            lbl.setStyleSheet(f"font-size: 10px; color: {TEXT_DIM};")
            rl.addWidget(lbl)
            hdr.addWidget(w)
        lay.addLayout(hdr)
        lay.addWidget(_divider())

        # Acceleration data buffers (np.roll pattern from dual_BNO085_testing.py)
        self._acc1 = np.zeros((3, self._BUF), dtype=np.float32)
        self._acc2 = np.zeros((3, self._BUF), dtype=np.float32)
        self._x    = np.arange(self._BUF, dtype=np.float32)

        # 3-D view
        if gl is not None:
            self._gl_view = gl.GLViewWidget()
            self._gl_view.setBackgroundColor(BG_DARK)
            self._gl_view.setCameraPosition(distance=8, elevation=30, azimuth=45)
            # World reference
            grid = gl.GLGridItem()
            grid.setSize(x=20, y=20)
            grid.setSpacing(x=1, y=1, z=1)
            self._gl_view.addItem(grid)
            axes = gl.GLAxisItem()
            axes.setSize(x=1, y=1, z=1)
            self._gl_view.addItem(axes)
            # Two IMU cubes with rods
            self._imu1_vis = _ImuVisualizer3D(self._gl_view, position=(0, 0, 0),           rod_length=self._ARM_LEN)
            self._imu2_vis = _ImuVisualizer3D(self._gl_view, position=(self._ARM_LEN,0,0), rod_length=self._ARM_LEN)
            lay.addWidget(self._gl_view, stretch=3)
        else:
            placeholder = QtWidgets.QLabel("PyOpenGL not installed\npip install PyOpenGL")
            placeholder.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            placeholder.setStyleSheet(f"color: {TEXT_DIM}; font-size: 12px;")
            lay.addWidget(placeholder, stretch=3)

        # Acceleration plots — identical style to dual_BNO085_testing.py sidebar
        def _accel_plot(title):
            p = pg.PlotWidget()
            p.setBackground(BG_DARK)
            p.showGrid(x=False, y=True, alpha=0.25)
            p.setYRange(-20, 20)
            p.getAxis('left').setPen(TEXT_DIM)
            p.getAxis('bottom').setPen(TEXT_DIM)
            p.getAxis('left').setTextPen(TEXT_DIM)
            p.getAxis('bottom').setStyle(showValues=False)
            p.setTitle(title, color=TEXT_MAIN, size='9pt')
            p.getAxis('top').setHeight(0)
            p.getAxis('right').setWidth(0)
            p.setMaximumHeight(100)
            return p

        acc_row = QtWidgets.QHBoxLayout()
        self._plot1 = _accel_plot("Accel IMU 1 (g)")
        self._c1x = self._plot1.plot(pen=pg.mkPen(RED,    width=2))
        self._c1y = self._plot1.plot(pen=pg.mkPen(GREEN,  width=2))
        self._c1z = self._plot1.plot(pen=pg.mkPen(ACCENT, width=2))

        self._plot2 = _accel_plot("Accel IMU 2 (g)")
        self._c2x = self._plot2.plot(pen=pg.mkPen(RED,    width=2))
        self._c2y = self._plot2.plot(pen=pg.mkPen(GREEN,  width=2))
        self._c2z = self._plot2.plot(pen=pg.mkPen(ACCENT, width=2))

        acc_row.addWidget(self._plot1)
        acc_row.addWidget(self._plot2)
        lay.addLayout(acc_row)

        # Colour legend (X=red, Y=green, Z=blue)
        leg = QtWidgets.QHBoxLayout()
        for col, txt in [(RED, "■ X"), (GREEN, "■ Y"), (ACCENT, "■ Z")]:
            l = QtWidgets.QLabel(txt)
            l.setStyleSheet(f"color: {col}; font-weight: bold; font-size: 10px;")
            leg.addWidget(l)
        leg.addStretch()
        lay.addLayout(leg)

        # Freeze detection (same as DualIMUWindow)
        self._last_v1 = None
        self._last_v2 = None
        self._freeze_t1 = time.perf_counter()
        self._freeze_t2 = time.perf_counter()
        self._FREEZE_SEC = 1.0

    # ── helpers ───────────────────────────────────────────────────────────────
    @staticmethod
    def _set_led_color(led: QtWidgets.QLabel, color: str) -> None:
        size = led.width() or 12
        led.setStyleSheet(f"background-color: {color}; border-radius: {size // 2}px;")

    # ── public API ────────────────────────────────────────────────────────────
    def update(self, sample) -> None:
        """Feed a SampleSTM32 instance; refreshes 3-D view + accel plots."""
        if sample is None:
            return
        now = time.perf_counter()

        v1 = (sample.roll1, sample.pitch1, sample.yaw1)
        v2 = (sample.roll2, sample.pitch2, sample.yaw2)

        # Freeze detection
        if v1 != self._last_v1:
            self._freeze_t1 = now
            self._last_v1   = v1
        if v2 != self._last_v2:
            self._freeze_t2 = now
            self._last_v2   = v2

        stuck1 = (now - self._freeze_t1) > self._FREEZE_SEC
        stuck2 = (now - self._freeze_t2) > self._FREEZE_SEC

        c1 = GREEN if (sample.imu1_ok and not stuck1) else (ORANGE if stuck1 else RED)
        c2 = GREEN if (sample.imu2_ok and not stuck2) else (ORANGE if stuck2 else RED)
        self._set_led_color(self._led1, c1)
        self._set_led_color(self._led2, c2)

        # Quaternions from Euler
        q1 = _euler_to_quat(sample.roll1, sample.pitch1, sample.yaw1)
        q2 = _euler_to_quat(sample.roll2, sample.pitch2, sample.yaw2)

        # 3-D cubes — forward kinematics: IMU2 sits at the tip of IMU1's arm
        if gl is not None:
            elbow = _rotate_vector(np.array([self._ARM_LEN, 0.0, 0.0]), q1)
            self._imu1_vis.update(q1)
            self._imu2_vis.update(q2, pos=elbow)

        # Acceleration ring-buffer (np.roll — same pattern as dual_BNO085_testing.py)
        self._acc1 = np.roll(self._acc1, -1, axis=1)
        self._acc1[:, -1] = [sample.ax1, sample.ay1, sample.az1]
        self._acc2 = np.roll(self._acc2, -1, axis=1)
        self._acc2[:, -1] = [sample.ax2, sample.ay2, sample.az2]

        self._c1x.setData(self._x, self._acc1[0])
        self._c1y.setData(self._x, self._acc1[1])
        self._c1z.setData(self._x, self._acc1[2])
        self._c2x.setData(self._x, self._acc2[0])
        self._c2y.setData(self._x, self._acc2[1])
        self._c2z.setData(self._x, self._acc2[2])



# ──────────────────────────────────────────────────────────────────────────────
# Panel: PRBS Sync Monitor
# ──────────────────────────────────────────────────────────────────────────────
class _SyncPanel(QtWidgets.QFrame):
    """
    Compact PRBS synchronisation status panel.
    Shows Kalman delay, confidence, drift and update count as styled text rows
    — no pyqtgraph plots.
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



# ──────────────────────────────────────────────────────────────────────────────
# Panel: Trial Configuration (STM32, EMG, Participant, Logic)
# ──────────────────────────────────────────────────────────────────────────────
class _TrialConfigurationPanel(QtWidgets.QFrame):
    """
    Displays comprehensive trial details from trial_config.py:
    1. STM32 Pins
    2. EMG Mapping
    3. Participant Details
    4. Trial Logic
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {BG_MID}; border-radius: 6px;")
        
        main_lay = QtWidgets.QVBoxLayout(self)
        main_lay.setContentsMargins(10, 6, 10, 6)
        main_lay.setSpacing(4)
        main_lay.addWidget(_section_label("Trial Configuration"))
        main_lay.addWidget(_divider())

        # Grid for the 4 sections
        grid = QtWidgets.QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(10)

        def _make_section(title, data_dict=None, list_data=None, label_list=None, color=TEXT_MAIN):
            w = QtWidgets.QWidget()
            lay = QtWidgets.QVBoxLayout(w)
            lay.setContentsMargins(0, 0, 0, 0)
            lay.setSpacing(2)
            lay.addWidget(QtWidgets.QLabel(f"<b>{title.upper()}</b>"))
            
            g = QtWidgets.QGridLayout()
            g.setContentsMargins(0, 0, 0, 0)
            g.setSpacing(1)
            
            if data_dict:
                for i, (k, v) in enumerate(data_dict.items()):
                    k_lbl = QtWidgets.QLabel(k)
                    k_lbl.setStyleSheet(f"font-size: 8px; color: {TEXT_DIM};")
                    v_lbl = QtWidgets.QLabel(str(v))
                    v_lbl.setStyleSheet(f"font-size: 8px; color: {color};")
                    g.addWidget(k_lbl, i, 0)
                    g.addWidget(v_lbl, i, 1)
            elif list_data:
                for i, val in enumerate(list_data):
                    prefix = f"{label_list[i] if label_list and i < len(label_list) else i+1}:"
                    p_lbl = QtWidgets.QLabel(prefix)
                    p_lbl.setStyleSheet(f"font-size: 8px; color: {TEXT_DIM};")
                    m_lbl = QtWidgets.QLabel(val)
                    m_lbl.setStyleSheet(f"font-size: 8px; color: {color};")
                    g.addWidget(p_lbl, i, 0)
                    g.addWidget(m_lbl, i, 1)
            
            lay.addLayout(g)
            lay.addStretch()
            return w

        # Fetch config
        stm32_conns = getattr(trial_config, "STM32_CONNECTIONS", {})
        emg_conns   = getattr(trial_config, "EMG_CONNECTIONS", [])
        emg_labels  = getattr(trial_config, "EMG_PLOT_LABELS", [])
        part_config = getattr(trial_config, "PARTICIPANT_CONFIG", {})
        logic_config = getattr(trial_config, "TRIAL_LOGIC_CONFIG", {})

        # 1. STM32
        grid.addWidget(_make_section("STM32", data_dict=stm32_conns, color=TEXT_MAIN), 0, 0)
        # 2. EMG
        grid.addWidget(_make_section("EMG", list_data=emg_conns, label_list=emg_labels, color=GREEN), 0, 1)
        # 3. Participant
        grid.addWidget(_make_section("Participant", data_dict=part_config, color=ACCENT), 1, 0)
        # 4. Trial Logic
        grid.addWidget(_make_section("Trial Logic", data_dict=logic_config, color=ORANGE), 1, 1)

        main_lay.addLayout(grid)


# ──────────────────────────────────────────────────────────────────────────────
# Panel: EMG Channels
# ──────────────────────────────────────────────────────────────────────────────
class _EMGPanel(QtWidgets.QFrame):
    """
    8-channel configurable EMG visualization (Processed Envelope).
    Displays 8 separate plots of processed signals.
    """
    def __init__(self, n_channels: int = 8, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background-color: {BG_MID}; border-radius: 6px;")
        self.n_channels = n_channels
        
        # Default mapping: channels 1-8 (0-7 index)
        self.channel_indices = list(range(n_channels))
        
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(4)
        
        # Header with Config Toggle
        hdr = QtWidgets.QHBoxLayout()
        hdr.addWidget(_section_label(f"EMG Channels (Processed Envelopes)"))
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
        # Distinct colors for better tracking
        self.colors = ['#3498db', '#e74c3c', '#2ecc71', '#f1c40f', '#9b59b6', '#1abc9c', '#e67e22', '#ecf0f1']
        
        for i in range(n_channels):
            p = self.graphics_layout.addPlot(row=i, col=0)
            p.setLabel("left", f"CH{self.channel_indices[i]+1}", size="7pt")
            p.showGrid(x=True, y=True, alpha=0.1)
            p.getAxis("bottom").setHeight(0) if i < n_channels - 1 else p.setLabel("bottom", "s", size="8pt")
            p.getAxis("left").setWidth(35)
            p.getAxis("left").setTextPen(TEXT_DIM)
            p.getAxis("bottom").setTextPen(TEXT_DIM)
            
            # Remove title to save vertical space
            p.setTitle(None)
            
            curve = p.plot(pen=pg.mkPen(self.colors[i % len(self.colors)], width=1.5))
            self.plots.append(p)
            self.curves.append(curve)
            if i > 0:
                p.setXLink(self.plots[0])

        # Buffers
        self.PLOT_WINDOW_SECONDS = 5.0
        # Assume 2000Hz sampling
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
        
        # Identify which channel changed to avoid global reset spike
        changed_idx = None
        for i, (old, new) in enumerate(zip(self.channel_indices, new_indices)):
            if old != new:
                changed_idx = i
                break
        
        self.channel_indices = new_indices
        # Update y-axis labels and reset filters
        for i, idx in enumerate(self.channel_indices):
            name = self.channel_names[idx] if idx < len(self.channel_names) else f"CH{idx+1}"
            self.plots[i].setLabel("left", name, size="7pt")
            
        if changed_idx is not None:
            self._pending_reset_idx = changed_idx
            # Clear data buffer for the changed channel to avoid visual discontinuity
            # Must preserve length to match self.time_buf to prevent pyqtgraph crash
            current_len = len(self.time_buf)
            self.data_bufs[changed_idx] = deque([0.0] * current_len, maxlen=self.max_pts)

    def update(self, manager):
        if manager is None or not hasattr(manager, 'tmsi_thread'):
            return
            
        tmsi_thread = manager.tmsi_thread
        
        # Capture channel names once available
        if not self.channel_names and hasattr(tmsi_thread, 'channels') and tmsi_thread.channels:
            self.channel_names = tmsi_thread.channels
            diag_text = "Available: " + ", ".join([f"[{i}]{n}" for i, n in enumerate(self.channel_names)])
            self._diagnostics_lbl.setText(diag_text)
            self._update_indices() # Refresh labels with names
            
        if self.processor is None and hasattr(manager, 'emg_processor'):
            self.processor = manager.emg_processor

        with tmsi_thread._lock:
            # history holds (t_arr, samples_2d)
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
            
            # Safe indexing: Extract requested channels or zeros if out of bounds
            # This prevents the entire panel from skipping chunks if one index is invalid.
            selected_samp = np.zeros((n_samples, self.n_channels), dtype=np.float32)
            for i, idx in enumerate(self.channel_indices):
                if 0 <= idx < n_cols:
                    selected_samp[:, i] = samp[:, idx]
                else:
                    selected_samp[:, i] = 0.0
            
            chunks_t.append(t_arr)
            chunks_v.append(selected_samp)

        if not chunks_t:
            return

        # Concatenate batch
        t_batch = np.concatenate(chunks_t)
        v_batch = np.concatenate(chunks_v) # (N, 8)
        
        if self.t0 is None:
            self.t0 = t_batch[0]
            
        if hasattr(self, '_pending_reset_idx') and self._pending_reset_idx is not None:
            idx = self._pending_reset_idx
            self._pending_reset_idx = None
            if self.processor is not None:
                # Perform warm initialization using the first sample of the new sequence
                self.processor.reset(channel_idx=idx, warm_value=float(v_batch[0, idx]))
        
        # Append time
        self.time_buf.extend(t_batch)
        
        # Process envelopes
        if self.processor is not None:
            # EMGProcessor.process handles 2D (samples, channels)
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", RuntimeWarning)
                _, envelopes = self.processor.process(v_batch, return_envelope=True)
            
            for i in range(self.n_channels):
                self.data_bufs[i].extend(envelopes[:, i])
        else:
            # Fallback to simple rectification
            rectified = np.abs(v_batch)
            for i in range(self.n_channels):
                self.data_bufs[i].extend(rectified[:, i])
            
        # Update curves
        if len(self.time_buf) > 0:
            t_arr = np.array(self.time_buf)
            t_rel = t_arr - self.t0
            
            # The window should be relative to the latest data
            x_max = t_rel[-1]
            x_min = max(0, x_max - self.PLOT_WINDOW_SECONDS)
            
            for i in range(self.n_channels):
                self.curves[i].setData(t_rel, np.array(self.data_bufs[i]))
            
            self.plots[0].setXRange(x_min, x_max)



# ──────────────────────────────────────────────────────────────────────────────
# Main Dashboard Window
# ──────────────────────────────────────────────────────────────────────────────
class TrialDashboard(QtWidgets.QMainWindow):
    """
    Comprehensive Trial Dashboard.

    Parameters
    ----------
    manager : TrialManager | None
        The active TrialManager from setup_trial.py.
        Pass None for demo / standalone mode.
    n_emg_channels : int
        How many EMG channel plots to show (default 8).
    """

    def __init__(
        self,
        manager=None,
        n_emg_channels: int = 8,
        parent: Optional[QtWidgets.QWidget] = None,
    ):
        super().__init__(parent)
        self._manager = manager
        # Persistent rate values — must survive between 0.5-s rate windows
        self._stm32_rate: float = 0.0
        self._tmsi_rate:  float = 0.0
        self._stm32_t0 = time.perf_counter()
        self._stm32_last_count = 0
        
        # IMU Freeze detection state (for health lights)
        self._imu1_last_euler = (0.0, 0.0, 0.0)
        self._imu2_last_euler = (0.0, 0.0, 0.0)
        self._imu1_freeze_t   = time.perf_counter()
        self._imu2_freeze_t   = time.perf_counter()

        p_id   = getattr(manager, "participant_id", "DEMO") if manager else "DEMO"
        t_num  = getattr(manager, "trial_num",      0)      if manager else 0

        self.setWindowTitle(f"Trial Dashboard — {p_id} / Trial {t_num}")
        self.resize(1600, 900)
        self.setStyleSheet(_COMMON_STYLE)

        # ── Root layout ───────────────────────────────────────────────────────
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        root_lay = QtWidgets.QVBoxLayout(root)
        root_lay.setContentsMargins(10, 10, 10, 10)
        root_lay.setSpacing(8)

        # ── Header ────────────────────────────────────────────────────────────
        self._header = _HeaderPanel(p_id, t_num)
        if manager and hasattr(manager, "start_time"):
            self._header.set_start_time(manager.start_time)
        self._header.stopRequested.connect(self._on_stop)
        root_lay.addWidget(self._header)

        # ── Body (3 columns) ─────────────────────────────────────────────────
        body_splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        body_splitter.setHandleWidth(4)
        body_splitter.setStyleSheet("QSplitter::handle { background: #1a2634; }")
        root_lay.addWidget(body_splitter, stretch=1)

        # ──── LEFT column: sensor health + 3-D IMU + Sync ─────────────────────
        left_col = QtWidgets.QWidget()
        left_lay = QtWidgets.QVBoxLayout(left_col)
        left_lay.setContentsMargins(0, 0, 0, 0)
        left_lay.setSpacing(8)

        self._health_panel = _SensorHealthPanel()
        self._health_panel.setFixedHeight(75)
        left_lay.addWidget(self._health_panel, stretch=0)

        self._imu_panel3d = _IMUPanel3D()
        left_lay.addWidget(self._imu_panel3d, stretch=5)

        self._sync_panel = _SyncPanel()
        self._sync_panel.setFixedHeight(85)
        left_lay.addWidget(self._sync_panel, stretch=0)

        self._connections_panel = _TrialConfigurationPanel()
        self._connections_panel.setFixedHeight(220)
        left_lay.addWidget(self._connections_panel, stretch=0)

        body_splitter.addWidget(left_col)

        # ──── MIDDLE column: Button Matrix (Big, Centerpiece) ─────────────────
        mid_col = QtWidgets.QWidget()
        mid_lay = QtWidgets.QVBoxLayout(mid_col)
        mid_lay.setContentsMargins(0, 0, 0, 0)
        mid_lay.setSpacing(8)

        self._matrix_panel = _ButtonMatrixPanel()
        mid_lay.addWidget(self._matrix_panel, stretch=1)

        body_splitter.addWidget(mid_col)

        # ──── RIGHT column: EMG channels ─────────────────────────────────────
        self._emg_panel = _EMGPanel(n_channels=n_emg_channels)
        body_splitter.addWidget(self._emg_panel)

        # Column stretch ratios (left : mid : right)
        body_splitter.setStretchFactor(0, 2)   # IMUs
        body_splitter.setStretchFactor(1, 3)   # matrix
        body_splitter.setStretchFactor(2, 2)   # EMG

        # ── Two-speed refresh timers ──────────────────────────────────────────
        # Fast timer: cheap UI (health LEDs, matrix, header blink)
        self._fast_timer = QtCore.QTimer(self)
        self._fast_timer.timeout.connect(self._fast_tick)
        self._fast_timer.start(FAST_REFRESH_MS)

        # Plot timer: expensive pyqtgraph setData calls
        self._plot_timer = QtCore.QTimer(self)
        self._plot_timer.timeout.connect(self._plot_tick)
        self._plot_timer.start(PLOT_REFRESH_MS)

        # Blink timer (500 ms for REC indicator)
        self._blink_timer = QtCore.QTimer(self)
        self._blink_timer.timeout.connect(self._header.tick)
        self._blink_timer.start(500)

    # ── Fast tick (10 Hz) — cheap UI updates ─────────────────────────────────
    def _fast_tick(self):
        """Update health LEDs, button matrix. Fast, cheap, no pyqtgraph."""
        mgr = self._manager

        stm32_online  = False
        latest_sample = None

        if mgr and hasattr(mgr, "stm32_thread"):
            t = mgr.stm32_thread
            stm32_online = getattr(t, "running", False)
            now = time.perf_counter()
            dt  = now - self._stm32_t0
            n   = getattr(t, "sample_count", 0)
            if dt > 0.5:
                # Only update the rate once per 0.5-s window;
                # between windows _stm32_rate keeps its last good value.
                self._stm32_rate       = (n - self._stm32_last_count) / dt
                self._stm32_last_count = n
                self._stm32_t0         = now
            # Grab only the last sample — no full list copy
            with t._lock:
                if t._history:
                    latest_sample = t._history[-1][1]

        self._health_panel.update_stm32(stm32_online, self._stm32_rate)

        imu1_ok = imu2_ok = False
        imu1_stuck = imu2_stuck = False
        
        if latest_sample is not None:
            imu1_ok = bool(latest_sample.imu1_ok)
            imu2_ok = bool(latest_sample.imu2_ok)
            
            # Stuck check: if ok but value identical to last euler for > 1s
            e1 = (latest_sample.roll1, latest_sample.pitch1, latest_sample.yaw1)
            e2 = (latest_sample.roll2, latest_sample.pitch2, latest_sample.yaw2)
            
            now = time.perf_counter()
            if e1 != self._imu1_last_euler:
                self._imu1_freeze_t = now
                self._imu1_last_euler = e1
            if e2 != self._imu2_last_euler:
                self._imu2_freeze_t = now
                self._imu2_last_euler = e2
                
            if now - self._imu1_freeze_t > 1.0: imu1_stuck = True
            if now - self._imu2_freeze_t > 1.0: imu2_stuck = True

        self._health_panel.update_imu1(imu1_ok, imu1_stuck)
        self._health_panel.update_imu2(imu2_ok, imu2_stuck)

        tmsi_online = False
        if mgr and hasattr(mgr, "tmsi_thread"):
            tt = mgr.tmsi_thread
            tmsi_online      = getattr(tt, "running", False)
            self._tmsi_rate  = getattr(tt, "estimated_rate_hz", self._tmsi_rate)
        self._health_panel.update_tmsi(tmsi_online, self._tmsi_rate)

        if latest_sample is not None:
            self._matrix_panel.update_mask(
                int(latest_sample.keys_mask),
                rate_hz=self._stm32_rate,
                sample_count=getattr(mgr.stm32_thread, "sample_count", 0) if mgr else 0,
            )
        # Cache for plot tick
        self._latest_sample = latest_sample

    # ── Plot tick (≈6.7 Hz) — expensive pyqtgraph updates ────────────────────
    def _plot_tick(self):
        """Update all pyqtgraph plots. Separated from fast_tick to avoid jank."""
        mgr = self._manager
        latest_sample = getattr(self, "_latest_sample", None)

        if latest_sample is not None:
            self._imu_panel3d.update(latest_sample)

        if mgr and hasattr(mgr, "estimator"):
            self._sync_panel.update(mgr.estimator)

        if mgr and hasattr(mgr, "tmsi_thread"):
            self._emg_panel.update(mgr)

    # ── Stop ─────────────────────────────────────────────────────────────────
    def _on_stop(self):
        self._fast_timer.stop()
        self._plot_timer.stop()
        self._blink_timer.stop()
        if self._manager:
            self._manager.stop()
        self.close()

    def closeEvent(self, event):
        self._fast_timer.stop()
        self._plot_timer.stop()
        self._blink_timer.stop()
        if self._manager:
            try:
                self._manager.stop()
            except Exception:
                pass
        event.accept()


# ──────────────────────────────────────────────────────────────────────────────
# Demo / Standalone Mode
# ──────────────────────────────────────────────────────────────────────────────
def _make_demo_manager():
    """
    Build a lightweight stand-in for TrialManager so the dashboard
    can be launched without hardware (--demo flag).
    """
    import types, math, random

    class _FakeSample:
        t_ms    = 0.0
        imu1_ok = 1; imu2_ok = 1
        yaw1 = pitch1 = roll1 = 0.0
        ax1 = ay1 = az1 = 0.0
        yaw2 = pitch2 = roll2 = 0.0
        ax2 = ay2 = az2 = 0.0
        keys_mask  = 0
        keys_rise  = 0; keys_fall = 0
        prbs_tick  = 0; prbs_lvl = 0; in_mark = 0

    class _FakeSTM32Thread:
        running      = True
        sample_count = 0
        _lock        = threading.Lock()
        _history: deque = deque(maxlen=5000)

        def _simulate(self):
            t0 = time.perf_counter()
            while True:
                time.sleep(1 / 500)
                t = time.perf_counter()
                s = _FakeSample()
                s.t_ms   = (t - t0) * 1000
                s.imu1_ok = 1; s.imu2_ok = 1
                phase = t * 0.5
                s.yaw1   = math.sin(phase) * 90
                s.pitch1 = math.cos(phase * 1.3) * 45
                s.roll1  = math.sin(phase * 0.7) * 30
                s.ax1    = math.sin(t * 3) * 2
                s.ay1    = math.cos(t * 2) * 1.5
                s.az1    = 9.81 + math.sin(t * 5) * 0.5
                s.yaw2   = math.cos(phase * 1.1) * 70
                s.pitch2 = math.sin(phase * 0.9) * 35
                s.roll2  = math.cos(phase * 1.5) * 25
                s.ax2    = math.cos(t * 3) * 1.8
                s.ay2    = math.sin(t * 2) * 1.2
                s.az2    = 9.81 + math.cos(t * 4) * 0.4
                s.keys_mask = (1 << random.randint(0, 11)) if random.random() < 0.02 else 0
                with self._lock:
                    self._history.append((t, s))
                    self.sample_count += 1

    class _FakeTMSiThread:
        running          = True
        estimated_rate_hz = 2000.0
        trig_idx         = 0
        channels         = [f"ch{i}" for i in range(1, 33)] + ["Dig"]
        _lock            = threading.Lock()
        _history: list   = []

        def _simulate(self, n_ch=8):
            t0 = time.perf_counter()
            while True:
                time.sleep(0.05)
                now = time.perf_counter()
                n = 100  # ~2000 Hz × 0.05 s
                t_arr = np.linspace(now - 0.05, now, n)
                samp  = np.random.randn(n, n_ch + 1) * 50
                with self._lock:
                    self._history.append((t_arr, samp))

    class _FakeEstimator:
        emg_buf_len  = 0
        stm32_buf_len = 0
        _update_count = 0

        class _Res:
            confidence = 0.72
            delay_ms   = 45.3
            polarity   = "normal"

        def get_result(self):
            return self._Res()

        def get_delay_ms(self):
            return 45.3 + math.sin(time.perf_counter() * 0.2) * 2

        def get_drift_rate_ppm(self):
            return 12.5

    class _FakeManager:
        participant_id = "DEMO"
        trial_num      = 0
        start_time     = time.perf_counter()

        def stop(self): pass

    mgr = _FakeManager()
    mgr.stm32_thread = _FakeSTM32Thread()
    mgr.tmsi_thread  = _FakeTMSiThread()
    mgr.estimator    = _FakeEstimator()

    # Kick off background simulation threads
    t1 = threading.Thread(target=mgr.stm32_thread._simulate, daemon=True)
    t1.start()
    t2 = threading.Thread(target=mgr.tmsi_thread._simulate, args=(32,), daemon=True)
    t2.start()

    return mgr


# ──────────────────────────────────────────────────────────────────────────────
# Entry-point
# ──────────────────────────────────────────────────────────────────────────────
def launch_dashboard(manager=None, n_emg_channels: int = 8):
    """
    Convenience launcher — call from setup_trial.py instead of
    displaying SimpleTrialStatusWidget.
    """
    if not _QT_OK:
        print("Dashboard unavailable: PyQt6 / pyqtgraph missing.")
        return None

    app = QtWidgets.QApplication.instance()
    _new_app = False
    if app is None:
        app = QtWidgets.QApplication(sys.argv)
        _new_app = True

    app.setStyle("Fusion")
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.ColorRole.Window,          QtGui.QColor(44, 62, 80))
    palette.setColor(QtGui.QPalette.ColorRole.WindowText,      QtGui.QColor(236, 240, 241))
    palette.setColor(QtGui.QPalette.ColorRole.Base,            QtGui.QColor(30, 45, 60))
    palette.setColor(QtGui.QPalette.ColorRole.AlternateBase,   QtGui.QColor(52, 73, 94))
    palette.setColor(QtGui.QPalette.ColorRole.Text,            QtGui.QColor(236, 240, 241))
    palette.setColor(QtGui.QPalette.ColorRole.Button,          QtGui.QColor(52, 73, 94))
    palette.setColor(QtGui.QPalette.ColorRole.ButtonText,      QtGui.QColor(236, 240, 241))
    palette.setColor(QtGui.QPalette.ColorRole.Highlight,       QtGui.QColor(52, 152, 219))
    palette.setColor(QtGui.QPalette.ColorRole.HighlightedText, QtGui.QColor(236, 240, 241))
    app.setPalette(palette)

    win = TrialDashboard(manager=manager, n_emg_channels=n_emg_channels)
    win.show()

    if _new_app:
        app.exec()

    return win


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Trial Dashboard")
    parser.add_argument(
        "--demo", action="store_true",
        help="Run in demo mode (no hardware required)"
    )
    parser.add_argument(
        "--emg-channels", type=int, default=8,
        help="Number of EMG channel plots to display (default: 8)"
    )
    args = parser.parse_args()

    if not _QT_OK:
        sys.exit(1)

    mgr = _make_demo_manager() if args.demo else None
    if mgr is None and not args.demo:
        print("Pass --demo to run without hardware, or import launch_dashboard() "
              "from setup_trial.py.")
        sys.exit(0)

    launch_dashboard(manager=mgr, n_emg_channels=args.emg_channels)
