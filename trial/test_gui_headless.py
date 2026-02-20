"""
Headless capture script to verify PyQt GUI rendering.
"""

import sys
import os
import time

# Add the parent directory (project root) to sys.path so 'trial' and 'src' modules can be found
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Create dummy QApplication
from PyQt6 import QtWidgets, QtCore, QtGui
app = QtWidgets.QApplication(sys.argv)

import trial.trial_manager as tm
manager = tm.TrialManager(mock_mode=True)
manager.initialize()

import argparse

parser = argparse.ArgumentParser(description="Test GUI headless")
parser.add_argument("--screenshot", action="store_true", help="Take a screenshot and exit after 2s")
args, unknown = parser.parse_known_args()

# Setup image grabber
output_dir = os.path.join(os.path.dirname(__file__), "screenshots")
os.makedirs(output_dir, exist_ok=True)

# Let GUI show and paint
manager.gui.show()

# Populate with dummy data so it's not empty
manager.gui.set_state(tm.TrialState.RECORDING)
first_ex = manager.expanded_protocol[0]
manager.gui.set_exercise(first_ex['display_name'], first_ex['instruction'])
manager.gui.set_progress(0.4)

# Run event loop for 2 seconds then screenshot
def take_screenshot():
    print("Taking screenshot...")
    QtCore.QCoreApplication.processEvents()

    # Grab pixel map
    pixmap = manager.gui.grab()
    path = os.path.join(output_dir, "gui_screenshot.png")
    pixmap.save(path)
    print(f"Screenshot saved to {path}")
    manager.gui.request_close.emit()

if args.screenshot:
    # Schedule screenshot
    timer = QtCore.QTimer()
    timer.setSingleShot(True)
    timer.timeout.connect(take_screenshot)
    timer.start(2000) # 2s wait

try:
    app.exec()
except Exception as e:
    print(e)
finally:
    manager.cleanup()
