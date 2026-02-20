"""
Trial GUI Module
=================
PyQt6-based GUI for trial data collection with real-time signal visualization.

Features:
- Real-time EMG and IMU signal plots using PyQtGraph
- 3D Dual IMU arm visualization using PyOpenGL
- Exercise instructions display
- Trial progress tracking
- Keyboard controls
- State indicators
"""

import sys
import time
import numpy as np
from collections import deque
from typing import Dict, List, Optional, Callable
from enum import Enum
import threading

try:
    from PyQt6 import QtWidgets, QtCore, QtGui
    import pyqtgraph as pg
    import pyqtgraph.opengl as gl
except ImportError:
    print("Error: PyQt6, pyqtgraph, and PyOpenGL are required.")
    print("pip install PyQt6 pyqtgraph PyOpenGL")
    sys.exit(1)

# =============================================================================
# STATE DEFINITIONS
# =============================================================================

class TrialState(Enum):
    """Trial state machine states."""
    IDLE = "idle"
    CALIBRATING = "calibrating"
    READY = "ready"
    COUNTDOWN = "countdown"
    RECORDING = "recording"
    SAVING = "saving"
    COMPLETED = "completed"


# =============================================================================
# 3D VISUALIZATION HELPERS (from dual BNO085 testing)
# =============================================================================

def euler_to_quat(roll_deg, pitch_deg, yaw_deg):
    """Convert Euler (deg) to Quaternion [w, x, y, z]"""
    r = np.radians(roll_deg)
    p = np.radians(pitch_deg)
    y = np.radians(yaw_deg)
    cr, sr = np.cos(r/2), np.sin(r/2)
    cp, sp = np.cos(p/2), np.sin(p/2)
    cy, sy = np.cos(y/2), np.sin(y/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    yq = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*sy
    return np.array([w, x, yq, z])

def rotate_vector(v, q):
    """Rotate vector v by quaternion q [w, x, y, z]."""
    w, x, y, z = q
    qv = np.array([x, y, z])
    cross1 = np.cross(qv, v)
    cross2 = np.cross(qv, cross1)
    return v + 2 * w * cross1 + 2 * cross2

def create_colored_cube():
    """Create mesh data for an RGB colored cube (+X=Red, +Y=Green, +Z=Blue)."""
    verts, colors = [], []
    c_red     = np.array([231, 76, 60, 255]) / 255.0
    c_red_d   = np.array([192, 57, 43, 255]) / 255.0
    c_green   = np.array([46, 204, 113, 255]) / 255.0
    c_green_d = np.array([39, 174, 96, 255]) / 255.0
    c_blue    = np.array([52, 152, 219, 255]) / 255.0
    c_blue_d  = np.array([41, 128, 185, 255]) / 255.0
    
    faces_info = [
        (np.array([ 1, 0, 0]), c_red),    # +X
        (np.array([-1, 0, 0]), c_red_d),  # -X
        (np.array([ 0, 1, 0]), c_green),  # +Y
        (np.array([ 0,-1, 0]), c_green_d),# -Y
        (np.array([ 0, 0, 1]), c_blue),   # +Z
        (np.array([ 0, 0,-1]), c_blue_d), # -Z
    ]
    s = 0.5
    for normal, color in faces_info:
        if abs(normal[0]) > 0.9: u, v = np.array([0, 1, 0]), np.array([0, 0, 1])
        elif abs(normal[1]) > 0.9: u, v = np.array([1, 0, 0]), np.array([0, 0, 1])
        else: u, v = np.array([1, 0, 0]), np.array([0, 1, 0])
            
        c = normal * s
        p1 = c - u*s - v*s; p2 = c + u*s - v*s
        p3 = c + u*s + v*s; p4 = c - u*s + v*s
        
        verts.append(np.array([p1, p2, p3]))
        colors.append(np.array([color, color, color]))
        verts.append(np.array([p1, p3, p4]))
        colors.append(np.array([color, color, color]))
            
    return np.array(verts), np.array(colors)

class ImuVisualizer:
    """Represents a single IMU in 3D space."""
    def __init__(self, view, position=(0,0,0), label_text="IMU", has_rod=False, rod_length=2.0):
        self.view = view
        self.container = gl.GLAxisItem(size=QtGui.QVector3D(0.01, 0.01, 0.01), glOptions='opaque')
        self.container.translate(*position)
        self.view.addItem(self.container)
        self.base_pos = position

        verts, colors = create_colored_cube()
        self.mesh = gl.GLMeshItem(vertexes=verts, vertexColors=colors, smooth=False, shader='balloon', drawEdges=True, edgeColor=(0,0,0,0.5))
        self.mesh.setParentItem(self.container)
        
        self.axes = gl.GLAxisItem(size=QtGui.QVector3D(1.5, 1.5, 1.5))
        self.axes.setParentItem(self.container)

        if has_rod:
            rod_w, rod_h = 0.15, 0.05
            self.rod = gl.GLBoxItem(size=QtGui.QVector3D(rod_length, rod_w, rod_h), color=(0.4, 0.4, 0.45, 1.0))
            self.rod.translate(0, -rod_w/2, -rod_h/2)
            self.rod.setParentItem(self.container)
        
    def update(self, q, pos=None):
        w, x, y, z = q
        angle = 2 * np.arccos(np.clip(w, -1.0, 1.0))
        s = np.sqrt(1 - w*w)
        if s < 0.001: ax, ay, az = 1, 0, 0
        else: ax, ay, az = x/s, y/s, z/s
            
        self.container.resetTransform()
        if pos is not None:
            self.container.translate(*pos)
        else:
            self.container.translate(*self.base_pos)
        self.container.rotate(np.degrees(angle), ax, ay, az)


# =============================================================================
# MAIN WINDOW FRAMEWORK
# =============================================================================

class TrialGUI(QtWidgets.QMainWindow):
    """
    Main GUI window for trial data collection using PyQt6.
    
    Layout:
    - Top: Exercise instruction panel (large, color-coded)
    - Middle: Splitter containing:
        - Left: 3D OpenGL view representing the arm
        - Right: Real-time graphs for EMG and IMU Acceleration
    - Bottom: Status panel with progress and shortcuts
    """
    
    # Custom PyQt signal for thread-safe cross-thread GUI updates if necessary
    request_close = QtCore.pyqtSignal()
    
    def __init__(self, 
                 config: Optional[Dict] = None,
                 on_key_press: Optional[Callable[[str], None]] = None,
                 on_test_hardware: Optional[Callable[[], None]] = None):
        super().__init__()
        self.config = config or self._default_config()
        self.on_key_press = on_key_press
        self.on_test_hardware = on_test_hardware
        
        # State
        self.state = TrialState.IDLE
        self.current_exercise = ""
        self.current_instruction = ""
        self.trial_number = 0
        self.total_trials = 0
        self.trial_progress = 0.0
        
        # Data buffers for 2D plotting (1D array layout for fast PyQtGraph updates)
        # Using fixed-size circular buffers via rolling arrays
        emg_sr = 2048 # Default estimate; gets updated with real timing ideally
        imu_sr = 200
        
        # Determine approx number of samples for the rolling windows
        self.emg_win_len = int(self.config['emg_plot_window_s'] * emg_sr)
        self.imu_win_len = int(self.config['imu_plot_window_s'] * imu_sr)
        
        # Start time tracking for relative plotting
        self.start_time = time.perf_counter()
        
        # Deques are fast for append, but PyQtGraph prefers contiguous NumPy arrays.
        # We'll use deques internally and build a view array for plotting as done in setup_scripts
        self.emg_time = deque(maxlen=self.emg_win_len)
        self.emg_data = [deque(maxlen=self.emg_win_len) for _ in range(self.config['emg_channels_to_plot'])]
        
        self.imu_time = deque(maxlen=self.imu_win_len)
        self.imu_accel = [deque(maxlen=self.imu_win_len) for _ in range(3)]
        
        # For keeping track of the latest Euler angles so the 3D model doesn't need to retain history
        self.latest_q1 = np.array([1.0, 0.0, 0.0, 0.0])
        self.latest_q2 = np.array([1.0, 0.0, 0.0, 0.0])
        
        # PRBS & Sync buffers
        self.prbs_win_len = 5000
        self.prbs_time_stm32 = deque(maxlen=self.prbs_win_len)
        self.prbs_data_stm32 = deque(maxlen=self.prbs_win_len)
        self.prbs_time_emg = deque(maxlen=self.prbs_win_len)
        self.prbs_data_emg = deque(maxlen=self.prbs_win_len)
        self.sync_delay_time = deque(maxlen=self.prbs_win_len)
        self.sync_delay_data = deque(maxlen=self.prbs_win_len)
        self.sync_conf_data = deque(maxlen=self.prbs_win_len)
        
        # Button matrix state
        self.button_mask = 0
        
        self.update_lock = threading.Lock()
        
        self._create_gui()
        self.request_close.connect(self.close)
        
        # Setup continuous update timer for GUI (plots & status)
        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self._on_update_tick)
        self.update_timer.start(self.config['plot_update_interval_ms'])
        
    def _default_config(self) -> Dict:
        return {
            'window_title': 'Trial Data Collection (PyQt Edition)',
            'window_width': 1300,
            'window_height': 850,
            'fullscreen': False,
            'plot_update_interval_ms': 33, # ~30fps
            'status_update_interval_ms': 100,
            'emg_plot_window_s': 5.0,
            'imu_plot_window_s': 5.0,
            'emg_channels_to_plot': 4,
            'color_idle': '#2ecc71',
            'color_ready': '#3498db',
            'color_recording': '#e74c3c',
            'color_saving': '#f1c40f',
            'color_completed': '#9b59b6',
            'font_instruction': ('Arial', 24, 'bold'),
            'font_status': ('Arial', 14),
            'font_info': ('Arial', 12),
        }

    def _parse_font(self, font_tuple) -> QtGui.QFont:
        """Parse Tkinter style font tuple `('Arial', 14, 'bold')` into `QFont`."""
        if not font_tuple or len(font_tuple) == 0:
            return QtGui.QFont()
        font_family = font_tuple[0]
        font_size = font_tuple[1] if len(font_tuple) > 1 else 10
        font_weight = QtGui.QFont.Weight.Bold if (len(font_tuple) > 2 and 'bold' in font_tuple[2]) else QtGui.QFont.Weight.Normal
        font = QtGui.QFont(font_family, font_size, font_weight)
        return font

    def _create_gui(self):
        self.setWindowTitle(self.config['window_title'])
        self.resize(self.config['window_width'], self.config['window_height'])
        self.setStyleSheet("QMainWindow { background-color: #2c3e50; }")

        # Global layout
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        # 1. Top Panel (Instructions)
        self._create_instruction_panel(main_layout)
        
        # 2. Middle Panel (Splitter for 3D View and 2D Plots)
        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        self._create_3d_panel(splitter)
        self._create_2d_panel(splitter)
        # Give plots a bit more width than the 3D view
        splitter.setSizes([400, 800])
        main_layout.addWidget(splitter, stretch=1)
        
        # 3. Bottom Panel (Status & Controls)
        self._create_status_panel(main_layout)
        
        if self.config['fullscreen']:
            self.showFullScreen()
            
    def _create_instruction_panel(self, parent_layout):
        panel = QtWidgets.QFrame()
        panel.setStyleSheet("background-color: #34495e; border-radius: 5px;")
        layout = QtWidgets.QVBoxLayout(panel)
        
        # State label
        self.state_label = QtWidgets.QLabel("IDLE")
        font_state = QtGui.QFont("Arial", 16, QtGui.QFont.Weight.Bold)
        self.state_label.setFont(font_state)
        self.state_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.state_label.setStyleSheet(f"background-color: {self.config['color_idle']}; color: white; padding: 5px; border-radius: 3px;")
        layout.addWidget(self.state_label)
        
        # Exercise Name
        self.exercise_label = QtWidgets.QLabel("Ready to start")
        self.exercise_label.setFont(self._parse_font(self.config['font_instruction']))
        self.exercise_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.exercise_label.setStyleSheet("color: white;")
        layout.addWidget(self.exercise_label)
        
        # Instruction
        self.instruction_label = QtWidgets.QLabel("Press SPACE to begin first trial")
        self.instruction_label.setFont(self._parse_font(self.config['font_status']))
        self.instruction_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.instruction_label.setStyleSheet("color: #bdc3c7;")
        self.instruction_label.setWordWrap(True)
        layout.addWidget(self.instruction_label)
        
        # Progress and Counter
        prog_layout = QtWidgets.QHBoxLayout()
        self.progress_bar = QtWidgets.QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(False)
        self.progress_bar.setFixedHeight(15)
        self.progress_bar.setStyleSheet("QProgressBar { border: 1px solid #7f8c8d; border-radius: 7px; background-color: #2c3e50; } "
                                      "QProgressBar::chunk { background-color: #3498db; border-radius: 7px; }")
        
        self.trial_counter_label = QtWidgets.QLabel("Trial: 0 / 0")
        self.trial_counter_label.setFont(self._parse_font(self.config['font_info']))
        self.trial_counter_label.setStyleSheet("color: #ecf0f1;")
        
        prog_layout.addWidget(self.progress_bar, stretch=1)
        prog_layout.addWidget(self.trial_counter_label)
        layout.addLayout(prog_layout)
        
        parent_layout.addWidget(panel)
        
    def _create_3d_panel(self, splitter):
        # OpenGL View Widget
        self.gl_view = gl.GLViewWidget()
        self.gl_view.setCameraPosition(distance=10, elevation=25, azimuth=45)
        self.gl_view.setBackgroundColor('#2c3e50')
        
        # Grid
        g = gl.GLGridItem()
        g.setSize(x=20, y=20, z=0)
        g.setSpacing(x=1, y=1, z=1)
        self.gl_view.addItem(g)
        
        # Global Axes
        axis = gl.GLAxisItem()
        axis.setSize(x=1, y=1, z=1)
        self.gl_view.addItem(axis)
        
        # Arm Geometry constants
        self.UPPER_ARM_LEN = 2.5
        self.LOWER_ARM_LEN = 2.5
        
        # Create arm segments
        self.imu1_vis = ImuVisualizer(self.gl_view, position=(0,0,0), label_text="IMU 1 (Upper)", has_rod=True, rod_length=self.UPPER_ARM_LEN)
        self.imu2_vis = ImuVisualizer(self.gl_view, position=(self.UPPER_ARM_LEN,0,0), label_text="IMU 2 (Lower)", has_rod=True, rod_length=self.LOWER_ARM_LEN)

        splitter.addWidget(self.gl_view)

    def _create_2d_panel(self, splitter):
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setStyleSheet("QTabWidget::pane { border: 1px solid #34495e; border-radius: 4px; } "
                                "QTabBar::tab { background: #34495e; color: #ecf0f1; padding: 8px 16px; margin-right: 2px; border-top-left-radius: 4px; border-top-right-radius: 4px; } "
                                "QTabBar::tab:selected { background: #2980b9; font-weight: bold; }")
        
        # ==========================================
        # TAB 1: Raw Signals
        # ==========================================
        self.plot_layout = pg.GraphicsLayoutWidget()
        self.plot_layout.setBackground('#2c3e50')
        
        # EMG Plot
        self.plot_emg = self.plot_layout.addPlot(row=0, col=0, title="EMG Signals (µV)")
        self.plot_emg.showGrid(x=False, y=True, alpha=0.3)
        self.plot_emg.setLabel('bottom', 'Time', units='s')
        self.plot_emg.getAxis('left').setPen('#7f8c8d')
        self.plot_emg.getAxis('bottom').setPen('#7f8c8d')
        # Setup multi-channel lines
        self.emg_curves = []
        emg_colors = ['#3498db', '#e67e22', '#2ecc71', '#e74c3c', '#9b59b6', '#1abc9c']
        for i in range(self.config['emg_channels_to_plot']):
            color = emg_colors[i % len(emg_colors)]
            curve = self.plot_emg.plot(pen=pg.mkPen(color, width=1), name=f'CH{i+1}')
            self.emg_curves.append(curve)
        self.plot_emg.addLegend(offset=(10,10))

        # IMU Acceleration Plot
        self.plot_imu = self.plot_layout.addPlot(row=1, col=0, title="IMU Acceleration (g)")
        self.plot_imu.showGrid(x=False, y=True, alpha=0.3)
        self.plot_imu.setLabel('bottom', 'Time', units='s')
        self.plot_imu.setYRange(-5, 5) # Default reasonable range
        self.plot_imu.getAxis('left').setPen('#7f8c8d')
        self.plot_imu.getAxis('bottom').setPen('#7f8c8d')
        
        self.imu_curves = []
        # Showing IMU1 Accel as representative
        imu_colors = ['#e74c3c', '#2ecc71', '#3498db'] # X Y Z
        for i, axis in enumerate(['X', 'Y', 'Z']):
            curve = self.plot_imu.plot(pen=pg.mkPen(imu_colors[i], width=2), name=f'Acc {axis}')
            self.imu_curves.append(curve)
        self.plot_imu.addLegend(offset=(10,10))
        
        # Link X Axes
        self.plot_imu.setXLink(self.plot_emg)
        
        # Optimization for fast rolling updates
        for curve in self.emg_curves + self.imu_curves:
            curve.setClipToView(True)

        self.tabs.addTab(self.plot_layout, "Raw Signals")
        
        # ==========================================
        # TAB 2: PRBS & Sync
        # ==========================================
        self.sync_layout = pg.GraphicsLayoutWidget()
        self.sync_layout.setBackground('#2c3e50')
        
        self.plot_prbs = self.sync_layout.addPlot(row=0, col=0, title="PRBS Sync Overlay")
        self.plot_prbs.showGrid(x=False, y=True, alpha=0.3)
        self.plot_prbs.setLabel('bottom', 'Time', units='s')
        self.plot_prbs.getAxis('left').setPen('#7f8c8d')
        self.plot_prbs.getAxis('bottom').setPen('#7f8c8d')
        
        self.curve_prbs_emg = self.plot_prbs.plot(pen=pg.mkPen('#3498db', width=2), name="EMG PRBS", stepMode="center")
        self.curve_prbs_stm32 = self.plot_prbs.plot(pen=pg.mkPen('#e74c3c', width=2), name="STM32 PRBS", stepMode="center")
        self.plot_prbs.addLegend(offset=(10,10))
        
        self.plot_delay = self.sync_layout.addPlot(row=1, col=0, title="Sync Delay & Confidence")
        self.plot_delay.showGrid(x=False, y=True, alpha=0.3)
        self.plot_delay.setLabel('bottom', 'Time', units='s')
        self.plot_delay.setLabel('left', 'Delay', units='ms')
        self.plot_delay.getAxis('left').setPen('#7f8c8d')
        self.plot_delay.getAxis('bottom').setPen('#7f8c8d')
        
        self.curve_delay = self.plot_delay.plot(pen=pg.mkPen('#2ecc71', width=2), name="Delay (ms)")
        
        # Secondary Y axis for confidence
        self.vb_conf = pg.ViewBox()
        self.plot_delay.showAxis('right')
        self.plot_delay.scene().addItem(self.vb_conf)
        self.plot_delay.getAxis('right').linkToView(self.vb_conf)
        self.vb_conf.setXLink(self.plot_delay)
        self.plot_delay.getAxis('right').setLabel('Confidence (0-1)')
        self.plot_delay.getAxis('right').setPen('#f1c40f')
        self.curve_conf = pg.PlotCurveItem(pen=pg.mkPen('#f1c40f', width=2), name="Confidence")
        self.vb_conf.addItem(self.curve_conf)
        
        def update_vb_conf():
            self.vb_conf.setGeometry(self.plot_delay.vb.sceneBoundingRect())
            self.vb_conf.linkedViewChanged(self.plot_delay.vb, self.vb_conf.XAxis)
        self.plot_delay.vb.sigResized.connect(update_vb_conf)
        
        self.plot_delay.setXLink(self.plot_prbs)
        
        for curve in [self.curve_prbs_emg, self.curve_prbs_stm32, self.curve_delay]:
            curve.setClipToView(True)
            
        self.tabs.addTab(self.sync_layout, "PRBS & Sync")
        
        # ==========================================
        # TAB 3: Button Matrix
        # ==========================================
        self.button_widget = QtWidgets.QWidget()
        self.button_widget.setStyleSheet("background-color: #2c3e50;")
        btn_layout = QtWidgets.QGridLayout(self.button_widget)
        btn_layout.setSpacing(15)
        
        self.matrix_buttons = []
        for r in range(3):
            row_buttons = []
            for c in range(4):
                lbl = QtWidgets.QLabel(f"{chr(ord('A') + r)}{c + 1}")
                lbl.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
                lbl.setFixedSize(80, 80)
                lbl.setStyleSheet("background-color: #34495e; color: #7f8c8d; border: 2px solid #2c3e50; border-radius: 10px;")
                font = QtGui.QFont("Segoe UI", 16, QtGui.QFont.Weight.Bold)
                lbl.setFont(font)
                btn_layout.addWidget(lbl, r, c)
                row_buttons.append(lbl)
            self.matrix_buttons.append(row_buttons)
            
        btn_layout.setRowStretch(3, 1)
        btn_layout.setColumnStretch(4, 1)
        
        self.tabs.addTab(self.button_widget, "Button Matrix")

        splitter.addWidget(self.tabs)

    def _create_status_panel(self, parent_layout):
        panel = QtWidgets.QFrame()
        panel.setStyleSheet("background-color: #34495e; border-radius: 5px;")
        layout = QtWidgets.QVBoxLayout(panel)
        
        # Status text
        self.status_label = QtWidgets.QLabel("Status: Idle | Waiting to start")
        self.status_label.setFont(self._parse_font(self.config['font_info']))
        self.status_label.setStyleSheet("color: white;")
        layout.addWidget(self.status_label)
        
        # Toolbar Layout
        toolbar = QtWidgets.QHBoxLayout()
        
        # Test hardware button
        self.btn_test_hw = QtWidgets.QPushButton("🔍 Test Hardware (T)")
        self.btn_test_hw.setStyleSheet("""
            QPushButton { background-color: #2980b9; color: white; padding: 6px 12px; border-radius: 4px; font-weight: bold; }
            QPushButton:hover { background-color: #3498db; }
        """)
        self.btn_test_hw.clicked.connect(self._on_test_hardware_clicked)
        toolbar.addWidget(self.btn_test_hw)
        
        lbl_shortcuts = QtWidgets.QLabel(
            "   [SPACE] Start/Stop | [N] Next | [R] Repeat | [C] Calibrate | [T] Test HW | [Q] Quit"
        )
        lbl_shortcuts.setStyleSheet("color: #bdc3c7;")
        toolbar.addWidget(lbl_shortcuts)
        toolbar.addStretch()
        
        layout.addLayout(toolbar)
        
        # Indicators
        indicators_layout = QtWidgets.QHBoxLayout()
        
        def add_indicator(label_text, value_color):
            lbl1 = QtWidgets.QLabel(label_text)
            lbl1.setStyleSheet("color: #95a5a6; font-size: 11px;")
            lbl2 = QtWidgets.QLabel("---")
            lbl2.setStyleSheet(f"color: {value_color}; font-size: 12px; font-weight: bold;")
            indicators_layout.addWidget(lbl1)
            indicators_layout.addWidget(lbl2)
            indicators_layout.addSpacing(10)
            return lbl2
            
        self.emg_health_labels = []
        for i in range(8):
            lbl = QtWidgets.QLabel(f"{(i%4)+1}")
            lbl.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            lbl.setFixedSize(16, 16)
            lbl.setStyleSheet("background-color: #7f8c8d; color: white; border-radius: 8px; font-size: 9px; font-weight: bold;") 
            indicators_layout.addWidget(lbl)
            self.emg_health_labels.append(lbl)
            if i == 3:
                indicators_layout.addSpacing(5) # Space between arms
        indicators_layout.addSpacing(15)
            
        self.emg_rate_label = add_indicator("EMG Hz:", "#3498db")
        self.imu_rate_label = add_indicator("IMU Rate:", "#3498db")
        self.emg_rms_label  = add_indicator("EMG RMS:", "#2ecc71")
        
        self.imu1_health_label = add_indicator("IMU1:", "#95a5a6")
        self.imu2_health_label = add_indicator("IMU2:", "#95a5a6")
        
        indicators_layout.addStretch()
        layout.addLayout(indicators_layout)
        
        parent_layout.addWidget(panel)

    def keyPressEvent(self, event: QtGui.QKeyEvent):
        key = ""
        if event.key() == QtCore.Qt.Key.Key_Space:
            key = "space"
        elif event.key() == QtCore.Qt.Key.Key_Escape:
            key = "escape"
        else:
            try:
                # Convert standard letters gracefully
                key = chr(event.key()).lower()
            except ValueError:
                pass

        if key == 't':
            self._on_test_hardware_clicked()
        elif self.on_key_press:
            self.on_key_press(key)

    def _on_test_hardware_clicked(self):
        if self.on_test_hardware:
            self.on_test_hardware()

    def _on_update_tick(self):
        """Update matplotlib plots with buffered data and rotate 3D view."""
        # 1. Update 3D Model
        self.imu1_vis.update(self.latest_q1)
        elbow_pos = rotate_vector(np.array([self.UPPER_ARM_LEN, 0, 0]), self.latest_q1)
        self.imu2_vis.update(self.latest_q2, pos=elbow_pos)
        
        # 2. Update 2D Plots
        with self.update_lock:
            # Snapshot data
            t_emg = np.array(self.emg_time)
            t_imu = np.array(self.imu_time)
            
            d_emg = [np.array(ch) for ch in self.emg_data]
            d_acc = [np.array(ch) for ch in self.imu_accel]
            
        if len(t_emg) > 1:
            for i, curve in enumerate(self.emg_curves):
                if i < len(d_emg) and len(d_emg[i]) == len(t_emg):
                    curve.setData(t_emg, d_emg[i])
            
            w_emg = self.config['emg_plot_window_s']
            self.plot_emg.setXRange(max(0, t_emg[-1] - w_emg), t_emg[-1])
            
        if len(t_imu) > 1:
            for i, curve in enumerate(self.imu_curves):
                if len(d_acc[i]) == len(t_imu):
                    curve.setData(t_imu, d_acc[i])

        # 3. Update Sync & PRBS Plots
        with self.update_lock:
            t_pe = np.array(self.prbs_time_emg)
            d_pe = np.array(self.prbs_data_emg)
            t_ps = np.array(self.prbs_time_stm32)
            d_ps = np.array(self.prbs_data_stm32)
            
            t_sd = np.array(self.sync_delay_time)
            d_sd = np.array(self.sync_delay_data)
            d_sc = np.array(self.sync_conf_data)
            
            mask = self.button_mask
            
        # Update PRBS curves (pad for stepMode)
        if len(t_pe) > 1 and len(d_pe) == len(t_pe):
            dt = t_pe[-1] - t_pe[-2]
            t_padded = np.append(t_pe, t_pe[-1] + dt)
            self.curve_prbs_emg.setData(t_padded, d_pe)
            
            w_prbs = self.config.get('prbs_plot_window_s', 5.0)
            self.plot_prbs.setXRange(max(0, t_pe[-1] - w_prbs), t_pe[-1] + 0.5)
            
        if len(t_ps) > 1 and len(d_ps) == len(t_ps):
            dt = t_ps[-1] - t_ps[-2]
            t_padded = np.append(t_ps, t_ps[-1] + dt)
            self.curve_prbs_stm32.setData(t_padded, d_ps)
            
        # Update Delay curves
        if len(t_sd) > 1:
            if len(d_sd) == len(t_sd):
                self.curve_delay.setData(t_sd, d_sd)
            if len(d_sc) == len(t_sd):
                self.curve_conf.setData(t_sd, d_sc)
            
        # 4. Update Button Matrix Status
        for r in range(3):
            for c in range(4):
                idx = r * 4 + c
                is_pressed = (mask >> idx) & 1
                if is_pressed:
                    self.matrix_buttons[r][c].setStyleSheet("background-color: #2ecc71; color: white; border: 2px solid #27ae60; border-radius: 10px;")
                else:
                    self.matrix_buttons[r][c].setStyleSheet("background-color: #34495e; color: #7f8c8d; border: 2px solid #2c3e50; border-radius: 10px;")

        # Progress bar
        self.progress_bar.setValue(int(self.trial_progress * 100))
        
    # =========================================================================
    # PUBLIC API for Data Streams
    # =========================================================================

    def update_emg_data(self, timestamps: np.ndarray, data: np.ndarray):
        with self.update_lock:
            # We enforce using the internal relative time from boot/start for X axis
            # Assuming timestamps are absolute float times (like time.perf_counter)
            rel_t = timestamps - self.start_time
            n_samples = len(rel_t)
            n_channels = min(data.shape[1], len(self.emg_data))
            
            self.emg_time.extend(rel_t)
            for ch in range(n_channels):
                self.emg_data[ch].extend(data[:, ch])

    def update_imu_data(self, timestamps: np.ndarray, accel: np.ndarray, gyro: Optional[np.ndarray] = None):
        with self.update_lock:
            rel_t = timestamps - self.start_time
            n_samples = len(rel_t)
            
            self.imu_time.extend(rel_t)
            for axis in range(3):
                self.imu_accel[axis].extend(accel[:, axis])

    def update_imu_orientation(self, q1: np.ndarray, q2: np.ndarray):
        """Update orientation state for the 3D visualizer."""
        self.latest_q1 = q1
        self.latest_q2 = q2

    def update_stm32_data(self, timestamps: np.ndarray, prbs_level: np.ndarray, button_mask: int):
        with self.update_lock:
            rel_t = timestamps - self.start_time
            self.prbs_time_stm32.extend(rel_t)
            self.prbs_data_stm32.extend(prbs_level)
            self.button_mask = button_mask

    def update_emg_prbs_data(self, timestamps: np.ndarray, prbs_level: np.ndarray):
        with self.update_lock:
            rel_t = timestamps - self.start_time
            self.prbs_time_emg.extend(rel_t)
            self.prbs_data_emg.extend(prbs_level)
            
    def update_sync_data(self, timestamp: float, delay_ms: float, confidence: float):
        with self.update_lock:
            rel_t = timestamp - self.start_time
            self.sync_delay_time.append(rel_t)
            self.sync_delay_data.append(delay_ms)
            self.sync_conf_data.append(confidence)

    # =========================================================================
    # PUBLIC API for State & Display
    # =========================================================================
    
    def set_state(self, state: TrialState):
        self.state = state
        state_colors = {
            TrialState.IDLE: self.config.get('color_idle', 'gray'),
            TrialState.CALIBRATING: self.config.get('color_saving', 'orange'),
            TrialState.READY: self.config.get('color_ready', 'blue'),
            TrialState.COUNTDOWN: self.config.get('color_saving', 'orange'),
            TrialState.RECORDING: self.config.get('color_recording', 'red'),
            TrialState.SAVING: self.config.get('color_saving', 'orange'),
            TrialState.COMPLETED: self.config.get('color_completed', 'purple'),
        }
        self.state_label.setText(state.value.upper())
        self.state_label.setStyleSheet(f"background-color: {state_colors.get(state, 'gray')}; color: white; padding: 5px; border-radius: 3px;")

    def set_exercise(self, name: str, instruction: str):
        self.current_exercise = name
        self.current_instruction = instruction
        self.exercise_label.setText(name)
        self.instruction_label.setText(instruction)

    def set_trial_info(self, trial_number: int, total_trials: int):
        self.trial_number = trial_number
        self.total_trials = total_trials
        self.trial_counter_label.setText(f"Trial: {trial_number} / {total_trials}")

    def set_progress(self, progress: float):
        self.trial_progress = max(0.0, min(1.0, progress))

    def set_status(self, status: str):
        self.status_label.setText(status)

    def update_signal_quality(self, emg_rate: float, imu_rate: float, emg_rms: float):
        self.emg_rate_label.setText(f"{emg_rate:.0f} Hz")
        self.imu_rate_label.setText(f"{imu_rate:.0f} Hz")
        self.emg_rms_label.setText(f"{emg_rms:.2f}")

    def update_imu_health(self, imu1_online: bool, imu1_zero_data: bool,
                          imu2_online: bool, imu2_zero_data: bool):
        def _fmt(online, zero):
            if not online: return "OFFLINE", "#e74c3c"
            if zero: return "ZERO", "#f39c12"
            return "OK", "#2ecc71"
            
        t1, c1 = _fmt(imu1_online, imu1_zero_data)
        t2, c2 = _fmt(imu2_online, imu2_zero_data)
        
        self.imu1_health_label.setText(t1)
        self.imu1_health_label.setStyleSheet(f"color: {c1}; font-size: 12px; font-weight: bold;")
        self.imu2_health_label.setText(t2)
        self.imu2_health_label.setStyleSheet(f"color: {c2}; font-size: 12px; font-weight: bold;")

    def update_emg_health(self, status: List[bool]):
        for i, is_ok in enumerate(status[:8]):
            if i < len(self.emg_health_labels):
                if is_ok:
                    self.emg_health_labels[i].setStyleSheet("background-color: #2ecc71; color: white; border-radius: 8px; font-size: 9px; font-weight: bold;")
                else:
                    self.emg_health_labels[i].setStyleSheet("background-color: #e74c3c; color: white; border-radius: 8px; font-size: 9px; font-weight: bold;")

    def clear_plots(self):
        with self.update_lock:
            self.emg_time.clear()
            for buf in self.emg_data: buf.clear()
            self.imu_time.clear()
            for buf in self.imu_accel: buf.clear()
            self.prbs_time_stm32.clear()
            self.prbs_data_stm32.clear()
            self.prbs_time_emg.clear()
            self.prbs_data_emg.clear()
            self.sync_delay_time.clear()
            self.sync_delay_data.clear()
            self.sync_conf_data.clear()

    # =========================================================================
    # QT Dialogs & Event Control
    # =========================================================================
    def show_message(self, title: str, message: str):
        QtWidgets.QMessageBox.information(self, title, message)

    def show_error(self, title: str, message: str):
        QtWidgets.QMessageBox.critical(self, title, message)

    def ask_yes_no(self, title: str, message: str) -> bool:
        reply = QtWidgets.QMessageBox.question(self, title, message,
                                               QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No)
        return reply == QtWidgets.QMessageBox.StandardButton.Yes

    def schedule_task(self, delay_ms: int, callback: Callable) -> QtCore.QTimer:
        """Schedules a function to run after a delay. Returns QTimer object which can be cancel/stopped."""
        timer = QtCore.QTimer()
        timer.setSingleShot(True)
        # Store hard reference on the window to prevent GC issues. Wait, actually returning it is enough if the caller stores it.
        # But to be safe, emit immediately.
        timer.timeout.connect(callback)
        timer.start(delay_ms)
        return timer

    def run(self):
        """Deprecated: The caller should initialize QApplication and call app.exec() itself."""
        pass
