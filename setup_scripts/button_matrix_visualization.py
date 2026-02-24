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
from pathlib import Path

# Project root setup
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# Import the native visualizer runner
from src.button_matrix import run_matrix_visualization

def main():
    parser = argparse.ArgumentParser(description="STM32 Button Matrix Visualizer")
    parser.add_argument("--port", type=str, default="auto", help="Serial port")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate")
    args = parser.parse_args()

    port = None if (args.port or "auto").lower() == "auto" else args.port
    
    # Run the visualization and keep a reference to the window
    win = run_matrix_visualization(port=port, baud=args.baud)
    
    # If run in an interactive environment where a QApplication already exists
    # (e.g. Spyder or Jupyter), we need to prevent the window from being garbage
    # collected. The win object is returned. We assign it to a global.
    global _matrix_win
    _matrix_win = win

if __name__ == "__main__":
    main()
