"""
Dual BNO085 UART-RVC IMU Tracking - PyQt Version
===================================================

Real-time orientation tracking with BNO085 9-DOF IMUs using PyQtGraph for visualization.
Features:
- Binary protocol (921600 baud, 500Hz) via STM32Reader
- 3D Visualization using OpenGL (Z-up world frame)
- Schematic World Reference Frame (Grid + Axes)
- Colored Cube for orientation (Red=+X, Green=+Y, Blue=+Z)
- Acceleration Vector Visualization
- Real-time 2D Acceleration Plots
- CSV data logging (full state and quaternions)
- Advanced Performance Monitoring

Usage:
  python setup_scripts/dual_BNO085_testing.py [--port PORT] [--baud BAUD] [--csv FILE]
"""

import sys
import argparse
import time
import numpy as np
import csv
from pathlib import Path
from collections import deque
from typing import Optional

# Setup import path
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# Imports
from src.stm32_reader import STM32Reader, SampleSTM32

try:
    from PyQt6 import QtWidgets, QtCore, QtGui
    import pyqtgraph.opengl as gl
    import pyqtgraph as pg
except ImportError:
    print("Error: PyQt6 and pyqtgraph are required.")
    print("pip install PyQt6 pyqtgraph PyOpenGL")
    sys.exit(1)

# ============================================================================
# SETTINGS
# ============================================================================
# Axis remapping for IMU to match visualization frame
IMU1_AXIS_MAP = (1, 2, 3) 
IMU2_AXIS_MAP = (1, 2, 3)
INVERT_ACCEL = False
ACCEL_PLOT_Y_RANGE = (-20, 20) 

# ============================================================================
# QUATERNION MATH
# ============================================================================
def quat_to_euler_deg(q):
    """Convert quaternion [w, x, y, z] to Euler angles (roll, pitch, yaw) in degrees."""
    w, x, y, z = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

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
    """Rotate vector v by quaternion q [w, x, y, z] using standard formula."""
    w, x, y, z = q
    qv = np.array([x, y, z])
    # v' = v + 2*q_w*(q_v x v) + 2*(q_v x (q_v x v))
    cross1 = np.cross(qv, v)
    cross2 = np.cross(qv, cross1)
    return v + 2 * w * cross1 + 2 * cross2

# ============================================================================
# CSV DATA LOGGER
# ============================================================================
class CSVLogger:
    """Handles CSV data logging for BNO085 data"""
    
    def __init__(self, filepath):
        self.filepath = Path(filepath)
        self.file_handle = None
        self.csv_writer = None
        self.sample_count = 0
        
        # Create directory if needed
        self.filepath.parent.mkdir(parents=True, exist_ok=True)
        
        # Open file and write header
        self.file_handle = open(self.filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.file_handle)
        
        # BNO085 CSV format: timestamp + sensor data + quaternions + euler
        self.csv_writer.writerow([
            'timestamp', 't_ms',
            'ax1_g', 'ay1_g', 'az1_g',
            'ax2_g', 'ay2_g', 'az2_g',
            'q1_w', 'q1_x', 'q1_y', 'q1_z',
            'q2_w', 'q2_x', 'q2_y', 'q2_z',
            'roll1_deg', 'pitch1_deg', 'yaw1_deg',
            'roll2_deg', 'pitch2_deg', 'yaw2_deg',
            'ok1', 'ok2'
        ])
        
        print(f"✓ CSV logging to: {self.filepath}")
    
    def log_sample(self, timestamp, sample: SampleSTM32, a1_g, a2_g, q1, q2, euler1, euler2):
        """Log BNO085 sample with quaternions and Euler angles"""
        self.csv_writer.writerow([
            timestamp, sample.t_ms,
            a1_g[0], a1_g[1], a1_g[2],
            a2_g[0], a2_g[1], a2_g[2],
            q1[0], q1[1], q1[2], q1[3],
            q2[0], q2[1], q2[2], q2[3],
            euler1[0], euler1[1], euler1[2],
            euler2[0], euler2[1], euler2[2],
            sample.imu1_ok, sample.imu2_ok
        ])
        self.sample_count += 1
        
        # Flush every 100 samples
        if self.sample_count % 100 == 0:
            self.file_handle.flush()
    
    def close(self):
        """Close CSV file"""
        if self.file_handle:
            self.file_handle.flush()
            self.file_handle.close()
            print(f"\\n✓ CSV saved: {self.filepath} ({self.sample_count} samples)")
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

# ============================================================================
# PERFORMANCE MONITORING
# ============================================================================
class PerformanceMonitor:
    def __init__(self, window_size=100):
        self.frame_times = deque(maxlen=window_size)
        self.latencies = deque(maxlen=window_size)
        self.packet_count = 0
        self.error_count = 0
        self.start_time = time.time()
        self.t_last = time.time()
        self.frames = 0
        self.fps = 0
    
    def update(self, latency_ms=None):
        now = time.time()
        dt = now - self.t_last
        self.frame_times.append(dt)
        self.t_last = now
        self.frames += 1
        
        if latency_ms is not None:
            self.latencies.append(latency_ms)
            self.packet_count += 1
            
        return self.get_fps()
            
    def get_fps(self):
        if len(self.frame_times) < 2:
            return 0.0
        return len(self.frame_times) / sum(self.frame_times)
    
    def get_avg_latency(self):
        if not self.latencies:
            return 0.0
        return np.mean(self.latencies)
    
    def get_packet_loss_rate(self):
        total_expected = (time.time() - self.start_time) * 500  # 500Hz
        if total_expected < 100:
            return 0.0
        return 100.0 * max(0.0, 1.0 - self.packet_count / total_expected)


# ============================================================================
# 3D VISUALIZATION OBJECTS
# ============================================================================
def create_colored_cube():
    """
    Create mesh data for a cube with colored faces.
    Using Flat UI colors for a nicer look.
    """
    verts = []
    colors = []
    
    # Flat UI Palette
    # Red: #e74c3c / #c0392b
    # Green: #2ecc71 / #27ae60
    # Blue: #3498db / #2980b9
    
    c_red    = np.array([231, 76, 60, 255]) / 255.0
    c_red_d  = np.array([192, 57, 43, 255]) / 255.0
    c_green  = np.array([46, 204, 113, 255]) / 255.0
    c_green_d= np.array([39, 174, 96, 255]) / 255.0
    c_blue   = np.array([52, 152, 219, 255]) / 255.0
    c_blue_d = np.array([41, 128, 185, 255]) / 255.0
    
    # Faces: normal, color
    faces_info = [
        (np.array([ 1, 0, 0]), c_red),    # +X
        (np.array([-1, 0, 0]), c_red_d),  # -X
        (np.array([ 0, 1, 0]), c_green),  # +Y
        (np.array([ 0,-1, 0]), c_green_d),# -Y
        (np.array([ 0, 0, 1]), c_blue),   # +Z
        (np.array([ 0, 0,-1]), c_blue_d), # -Z
    ]
    
    s = 0.5 # Half size
    
    for normal, color in faces_info:
        # Construct basis for face
        if abs(normal[0]) > 0.9:
            u, v = np.array([0, 1, 0]), np.array([0, 0, 1])
        elif abs(normal[1]) > 0.9:
            u, v = np.array([1, 0, 0]), np.array([0, 0, 1])
        else:
            u, v = np.array([1, 0, 0]), np.array([0, 1, 0])
            
        c = normal * s
        p1 = c - u*s - v*s
        p2 = c + u*s - v*s
        p3 = c + u*s + v*s
        p4 = c - u*s + v*s
        
        # Triangle 1
        verts.append(np.array([p1, p2, p3]))
        colors.append(np.array([color, color, color]))
        
        # Triangle 2
        verts.append(np.array([p1, p3, p4]))
        colors.append(np.array([color, color, color]))
            
    return np.array(verts), np.array(colors)

class ImuVisualizer:
    """Represents a single IMU in 3D space."""
    def __init__(self, view, position=(0,0,0), label_text="IMU", has_rod=False, rod_length=2.0):
        self.view = view
        
        # Container
        self.container = gl.GLAxisItem(size=QtGui.QVector3D(0.01, 0.01, 0.01), glOptions='opaque')
        self.container.translate(*position)
        self.view.addItem(self.container)
        self.base_pos = position

        # Colored Cube Body
        verts, colors = create_colored_cube()
        self.mesh = gl.GLMeshItem(vertexes=verts, vertexColors=colors, smooth=False, shader='balloon', drawEdges=True, edgeColor=(0,0,0,0.5))
        self.mesh.setParentItem(self.container)
        
        # Local Axes
        self.axes = gl.GLAxisItem(size=QtGui.QVector3D(1.5, 1.5, 1.5))
        self.axes.setParentItem(self.container)

        # Rod (Optional)
        if has_rod:
            # Rod represented by a gray rectangular beam
            # Origin at (0,0,0) in local coords, length along +X (points forward to next joint)
            rod_length_val = rod_length
            rod_w = 0.15 # Width (Y)
            rod_h = 0.05 # Height (Z)
            self.rod = gl.GLBoxItem(size=QtGui.QVector3D(rod_length_val, rod_w, rod_h), color=(0.4, 0.4, 0.45, 1.0))
            # Extend from 0 to rod_length_val, centered on Y and Z
            self.rod.translate(0, -rod_w/2, -rod_h/2)
            self.rod.setParentItem(self.container)
        
    def update(self, q, pos=None):
        """
        Update orientation.
        q: [w, x, y, z] quaternion
        pos: optional [x, y, z] position override
        """
        w, x, y, z = q
        angle = 2 * np.arccos(np.clip(w, -1.0, 1.0))
        s = np.sqrt(1 - w*w)
        if s < 0.001:
            ax, ay, az = 1, 0, 0
        else:
            ax, ay, az = x/s, y/s, z/s
            
        self.container.resetTransform()
        if pos is not None:
            self.container.translate(*pos)
        else:
            self.container.translate(*self.base_pos)
        self.container.rotate(np.degrees(angle), ax, ay, az)


# ============================================================================
# MAIN WINDOW
# ============================================================================
class DualIMUWindow(QtWidgets.QMainWindow):
    def __init__(self, port, baud, csv_file=None):
        super().__init__()
        self.setWindowTitle("STM32 Dual BNO085")
        self.resize(1000, 700)
        
        # Styling
        self.setStyleSheet("""
            QMainWindow { background-color: #2c3e50; }
            QLabel { color: #ecf0f1; font-family: 'Segoe UI', sans-serif; }
            QPushButton { 
                background-color: #34495e; color: white; border: none; padding: 8px; 
                border-radius: 4px; font-weight: bold;
            }
            QPushButton:hover { background-color: #4e6a85; }
        """)
        
        # Layout
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        # 3D View
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=8, elevation=30, azimuth=45)
        self.view.setBackgroundColor('#2c3e50') # Match window bg
        layout.addWidget(self.view, stretch=3)
        
        # Sidebar
        sidebar = QtWidgets.QWidget()
        sidebar.setFixedWidth(280)
        sidebar.setStyleSheet("background-color: #34495e;") # Slightly lighter than bg
        side_layout = QtWidgets.QVBoxLayout(sidebar)
        side_layout.setContentsMargins(15, 15, 15, 15)
        side_layout.setSpacing(10)
        layout.addWidget(sidebar, stretch=0)
        
        # Title
        title = QtWidgets.QLabel("SYSTEM STATUS")
        title.setStyleSheet("font-weight: bold; font-size: 14px; color: #3498db; letter-spacing: 1px;")
        side_layout.addWidget(title)
        
        # Stats Box
        self.stats_frame = QtWidgets.QFrame()
        self.stats_frame.setStyleSheet("background-color: #2c3e50; border-radius: 5px; padding: 10px;")
        sf_layout = QtWidgets.QVBoxLayout(self.stats_frame)
        
        # Status Lights
        lights_layout = QtWidgets.QHBoxLayout()
        def make_light():
            container = QtWidgets.QWidget()
            l_lay = QtWidgets.QVBoxLayout(container)
            l_lay.setContentsMargins(0, 0, 0, 0)
            l_lay.setSpacing(2)
            
            indicator = QtWidgets.QLabel()
            indicator.setFixedSize(14, 14)
            indicator.setStyleSheet("background-color: #e74c3c; border-radius: 7px; border: 1px solid #c0392b;")
            
            l_lay.addWidget(indicator, alignment=QtCore.Qt.AlignmentFlag.AlignCenter)
            return container, indicator
            
        self.light1_w, self.light1 = make_light()
        self.light2_w, self.light2 = make_light()
        
        def add_labeled_light(lay, widget, name):
            item_lay = QtWidgets.QVBoxLayout()
            item_lay.addWidget(widget)
            lbl = QtWidgets.QLabel(name)
            lbl.setStyleSheet("font-size: 9px; color: #95a5a6; font-weight: bold;")
            lbl.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            item_lay.addWidget(lbl)
            lay.addLayout(item_lay)
            
        add_labeled_light(lights_layout, self.light1_w, "IMU 1")
        add_labeled_light(lights_layout, self.light2_w, "IMU 2")
        
        sf_layout.addLayout(lights_layout)
        sf_layout.addSpacing(5)

        self.lbl_stats = QtWidgets.QLabel("Initializing...")
        self.lbl_stats.setStyleSheet("font-family: 'Consolas', monospace; font-size: 11px; color: #bdc3c7;")
        sf_layout.addWidget(self.lbl_stats)
        side_layout.addWidget(self.stats_frame)
        
        # Controls
        self.btn_reset = QtWidgets.QPushButton("RESET CAMERA VIEW")
        self.btn_reset.setCursor(QtCore.Qt.CursorShape.PointingHandCursor)
        self.btn_reset.clicked.connect(self.reset_view)
        side_layout.addWidget(self.btn_reset)
        
        side_layout.addSpacing(10)
        
        # Plots
        lbl_plot = QtWidgets.QLabel("ACCELERATION (g)")
        lbl_plot.setStyleSheet("font-weight: bold; font-size: 12px; color: #95a5a6; margin-top: 10px;")
        side_layout.addWidget(lbl_plot)
        
        def setup_plot(title):
            p = pg.PlotWidget()
            p.setBackground('#2c3e50')
            p.showGrid(x=False, y=True, alpha=0.3)
            p.setYRange(*ACCEL_PLOT_Y_RANGE)
            p.getAxis('left').setPen('#7f8c8d')
            p.getAxis('bottom').setPen('#7f8c8d')
            p.getAxis('left').setTextPen('#7f8c8d')
            p.getAxis('bottom').setStyle(showValues=False)
            p.setTitle(title, color='#ecf0f1', size='10pt')
            return p
            
        self.plot1 = setup_plot("IMU 1")
        self.c1x = self.plot1.plot(pen=pg.mkPen('#e74c3c', width=2))
        self.c1y = self.plot1.plot(pen=pg.mkPen('#2ecc71', width=2))
        self.c1z = self.plot1.plot(pen=pg.mkPen('#3498db', width=2))
        side_layout.addWidget(self.plot1)
        
        self.plot2 = setup_plot("IMU 2")
        self.c2x = self.plot2.plot(pen=pg.mkPen('#e74c3c', width=2))
        self.c2y = self.plot2.plot(pen=pg.mkPen('#2ecc71', width=2))
        self.c2z = self.plot2.plot(pen=pg.mkPen('#3498db', width=2))
        side_layout.addWidget(self.plot2)
        
        # Legend (Manual)
        legend_layout = QtWidgets.QHBoxLayout()
        for col, txt in [('#e74c3c', 'X'), ('#2ecc71', 'Y'), ('#3498db', 'Z')]:
            l = QtWidgets.QLabel(f"■ {txt}")
            l.setStyleSheet(f"color: {col}; font-weight: bold; font-size: 10px;")
            legend_layout.addWidget(l)
        legend_layout.addStretch()
        side_layout.addLayout(legend_layout)
        
        side_layout.addStretch()

        # Data buffers for plotting
        self.buf_size = 200
        self.data_a1 = np.zeros((3, self.buf_size))
        self.data_a2 = np.zeros((3, self.buf_size))

        # Status Tracking
        self.last_update_t = time.time()
        self.last_v1 = None
        self.last_v2 = None
        self.freeze_t1 = time.time()
        self.freeze_t2 = time.time()
        self.TIMEOUT_SEC = 0.5
        self.FREEZE_SEC = 1.0

        # World Reference (Grid + Global Axes)
        self.add_world_reference()
        
        # Arm Geometry
        self.UPPER_ARM_LEN = 2.5
        self.LOWER_ARM_LEN = 2.5

        # IMU Objects
        # IMU 1: Upper Arm (Shoulder -> Elbow)
        self.imu1 = ImuVisualizer(self.view, position=(0, 0, 0), label_text="IMU 1 (Upper)", has_rod=True, rod_length=self.UPPER_ARM_LEN)
        # IMU 2: Lower Arm (Elbow -> Wrist)
        self.imu2 = ImuVisualizer(self.view, position=(self.UPPER_ARM_LEN, 0, 0), label_text="IMU 2 (Lower)", has_rod=True, rod_length=self.LOWER_ARM_LEN)
        
        # Data
        self.reader = STM32Reader(port=port, baud=baud, verbose=True, binary_mode=True)
        self.reader.start()
        
        self.csv_logger = None
        if csv_file:
            self.csv_logger = CSVLogger(csv_file)

        # Update Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(16) # ~60 FPS
        
        self.perf = PerformanceMonitor()
        self.python_start_time = time.time()

    def add_world_reference(self):
        """Add grid and global axes"""
        # Grid
        g = gl.GLGridItem()
        g.setSize(x=20, y=20, z=0)
        g.setSpacing(x=1, y=1, z=1)
        self.view.addItem(g)
        
        # Global Axes
        axis = gl.GLAxisItem()
        axis.setSize(x=1, y=1, z=1)
        self.view.addItem(axis)

    def reset_view(self):
        self.view.setCameraPosition(distance=8, elevation=30, azimuth=45)

    def update_loop(self):
        sample = self.reader.latest
        now = time.time()
        
        # 1. Check for global reader timeout
        if sample is None or (now - self.last_update_t > self.TIMEOUT_SEC and sample is self.reader.latest):
            # No data coming in
            self.light1.setStyleSheet("background-color: #e74c3c; border-radius: 8px;")
            self.light2.setStyleSheet("background-color: #e74c3c; border-radius: 8px;")
            self.perf.update(latency_ms=None) # Update FPS only
            if sample is None: return
        
        # Detect if this is a fresh sample
        is_new = False
        if hasattr(self, '_last_processed_t') and sample.t_ms != self._last_processed_t:
            is_new = True
            self.last_update_t = now
            self._last_processed_t = sample.t_ms
        elif not hasattr(self, '_last_processed_t'):
            self._last_processed_t = sample.t_ms
            is_new = True

        # Data for IMU1 and IMU2
        v1 = (sample.roll1, sample.pitch1, sample.yaw1)
        v2 = (sample.roll2, sample.pitch2, sample.yaw2)
        
        # 2. Check for frozen data (exactly the same)
        if is_new:
            if v1 != self.last_v1:
                self.freeze_t1 = now
                self.last_v1 = v1
            if v2 != self.last_v2:
                self.freeze_t2 = now
                self.last_v2 = v2
        
        imu1_stuck = (now - self.freeze_t1) > self.FREEZE_SEC
        imu2_stuck = (now - self.freeze_t2) > self.FREEZE_SEC
        
        # 3. Update Status Lights
        def set_light(light, active, stuck):
            if not active or stuck:
                light.setStyleSheet("background-color: #e74c3c; border-radius: 8px; border: 1px solid #c0392b;")
            else:
                light.setStyleSheet("background-color: #2ecc71; border-radius: 8px; border: 1px solid #27ae60;")

        set_light(self.light1, sample.imu1_ok, imu1_stuck)
        set_light(self.light2, sample.imu2_ok, imu2_stuck)

        # Orientation
        q1 = euler_to_quat(sample.roll1, sample.pitch1, sample.yaw1)
        q2 = euler_to_quat(sample.roll2, sample.pitch2, sample.yaw2)
        
        # Accel
        a1 = np.array([sample.ax1, sample.ay1, sample.az1])
        a2 = np.array([sample.ax2, sample.ay2, sample.az2])
        
        # Update 3D
        self.imu1.update(q1)

        # Forward Kinematics:
        elbow_pos = rotate_vector(np.array([self.UPPER_ARM_LEN, 0, 0]), q1)
        self.imu2.update(q2, pos=elbow_pos)
        
        # Update latency
        latency_ms = None
        if is_new:
            python_time_ms = (now - self.python_start_time) * 1000
            latency_ms = python_time_ms - sample.t_ms

        # Update FPS, latency, loss
        fps = self.perf.update(latency_ms)
        
        # Update Plots
        if is_new:
            self.data_a1 = np.roll(self.data_a1, -1, axis=1)
            self.data_a1[:, -1] = a1
            self.data_a2 = np.roll(self.data_a2, -1, axis=1)
            self.data_a2[:, -1] = a2
            
            self.c1x.setData(self.data_a1[0])
            self.c1y.setData(self.data_a1[1])
            self.c1z.setData(self.data_a1[2])
            
            self.c2x.setData(self.data_a2[0])
            self.c2y.setData(self.data_a2[1])
            self.c2z.setData(self.data_a2[2])
            
            # Log
            if self.csv_logger:
                self.csv_logger.log_sample(
                    timestamp=now,
                    sample=sample,
                    a1_g=a1, a2_g=a2,
                    q1=q1, q2=q2,
                    euler1=v1, euler2=v2
                )
        
        # Update UI Stats
        rate = self.reader.get_sample_rate()
        txt = (f"FPS: {fps:.1f}\n"
               f"Packet Rate: {rate:.1f} Hz\n"
               f"Latency: {self.perf.get_avg_latency():.1f} ms\n"
               f"Loss: {self.perf.get_packet_loss_rate():.1f} %\n\n"
               f"IMU 1 (Left):\n"
               f"  Status: {'OK' if sample.imu1_ok and not imu1_stuck else 'STUCK/OFF'}\n"
               f"  R:{sample.roll1:5.1f} P:{sample.pitch1:5.1f} Y:{sample.yaw1:5.1f}\n\n"
               f"IMU 2 (Right):\n"
               f"  Status: {'OK' if sample.imu2_ok and not imu2_stuck else 'STUCK/OFF'}\n"
               f"  R:{sample.roll2:5.1f} P:{sample.pitch2:5.1f} Y:{sample.yaw2:5.1f}")
        self.lbl_stats.setText(txt)

    def closeEvent(self, event):
        self.reader.stop()
        if self.csv_logger:
            self.csv_logger.close()
        event.accept()


# ============================================================================
# MAIN
# ============================================================================
def main():
    parser = argparse.ArgumentParser(description="PyQt BNO085 Visualizer")
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--csv", type=str, default=None)
    args = parser.parse_args()
    
    port = None if args.port.lower() == "auto" else args.port
    
    app = QtWidgets.QApplication(sys.argv)
    win = DualIMUWindow(port, args.baud, args.csv)
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
