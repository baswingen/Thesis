"""
Backward-compatible launcher for the native module.

The primary (native) implementation now lives in:
  `src/bno085_dual_visualizer.py`
"""

from __future__ import annotations

from src.bno085_dual_visualizer import main


if __name__ == "__main__":
    main()
