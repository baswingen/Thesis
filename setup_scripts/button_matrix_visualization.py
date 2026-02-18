"""
Button Matrix Visualization
===========================

Real-time visualization of the 3x4 button matrix connected to the STM32.
Requires the STM32 to be running the `STM32_all_in_python_binary.ino` sketch.

Usage:
    python setup_scripts/button_matrix_visualization.py [--port PORT]

"""

import sys
import argparse
import time
from pathlib import Path

# Project root setup
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.stm32_reader import STM32Reader

try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtWidgets, QtCore, QtGui
except ImportError:
    print("Error: pyqtgraph is required. pip install pyqtgraph")
    sys.exit(1)

# Logic from Arduino sketch:
# ROWS = 3, COLS = 4
# s |= (1u << (r * COLS + c))
BUTTON_ROWS = 3
BUTTON_COLS = 4

class MatrixWindow(QtWidgets.QMainWindow):
    def __init__(self, port=None, baud=921600):
        super().__init__()
        self.setWindowTitle("STM32 Button Matrix")
        self.resize(500, 400)
        
        # Dark theme
        self.setStyleSheet("background-color: #2c3e50; color: white;")

        # Central widget
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        
        # Title
        title = QtWidgets.QLabel("3x4 Button Matrix")
        title.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("font-size: 18px; font-weight: bold; padding: 10px;")
        layout.addWidget(title)

        # Grid
        grid_widget = QtWidgets.QWidget()
        grid_layout = QtWidgets.QGridLayout(grid_widget)
        grid_layout.setSpacing(15)
        layout.addWidget(grid_widget)

        self.buttons = []
        
        # Create 3x4 grid of labels
        for r in range(BUTTON_ROWS):
            row_buttons = []
            for c in range(BUTTON_COLS):
                row_letter = chr(ord('A') + r)
                col_number = c + 1
                lbl = QtWidgets.QLabel(f"{row_letter}{col_number}")
                lbl.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
                lbl.setFixedSize(80, 80)
                # Initial style
                lbl.setStyleSheet(self._get_style(False))
                font = QtGui.QFont("Segoe UI", 12)
                font.setBold(True)
                lbl.setFont(font)
                grid_layout.addWidget(lbl, r, c)
                row_buttons.append(lbl)
            self.buttons.append(row_buttons)

        # Spacer
        layout.addStretch()

        # Status label
        self.status_lbl = QtWidgets.QLabel("Initializing...")
        self.status_lbl.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.status_lbl.setStyleSheet("font-size: 12px; color: #bdc3c7; padding: 5px;")
        layout.addWidget(self.status_lbl)

        # STM32 Reader
        self.reader = STM32Reader(port=port, baud=baud, verbose=True)
        self.reader.start()

        # Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_state)
        self.timer.start(16) # ~60 FPS

    def _get_style(self, pressed):
        if pressed:
            # Bright Green for active
            return (
                "background-color: #2ecc71; color: white; "
                "border: 2px solid #27ae60; border-radius: 10px;"
            )
        else:
            # Dark Gray for idle
            return (
                "background-color: #34495e; color: #7f8c8d; "
                "border: 2px solid #2c3e50; border-radius: 10px;"
            )

    def update_state(self):
        if not self.reader.running:
            self.status_lbl.setText("STM32 Disconnected")
            return

        sample = self.reader.latest
        
        if sample is None:
            self.status_lbl.setText("Waiting for data...")
            return

        mask = sample.keys_mask
        
        # Update buttons
        for r in range(BUTTON_ROWS):
            for c in range(BUTTON_COLS):
                # Map row/col to bit index: r * 4 + c
                idx = r * BUTTON_COLS + c
                is_pressed = (mask >> idx) & 1
                self.buttons[r][c].setStyleSheet(self._get_style(is_pressed))
        
        # Update status
        rate = self.reader.get_sample_rate()
        self.status_lbl.setText(
            f"Connected | Rate: {rate:.0f} Hz | Mask: 0x{mask:03X} | "
            f"Samples: {self.reader.sample_count}"
        )

    def closeEvent(self, event):
        self.reader.stop()
        event.accept()

def main():
    parser = argparse.ArgumentParser(description="STM32 Button Matrix Visualizer")
    parser.add_argument("--port", type=str, default="auto", help="Serial port")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate")
    args = parser.parse_args()

    port = None if (args.port or "auto").lower() == "auto" else args.port
    app = QtWidgets.QApplication(sys.argv)
    win = MatrixWindow(port=port, baud=args.baud)
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
