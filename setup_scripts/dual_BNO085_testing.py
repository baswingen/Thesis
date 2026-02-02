"""
Backward-compatible launcher for the native module.

The primary (native) implementation now lives in:
  `src/bno085_dual_visualizer.py`
"""

from __future__ import annotations

import sys
from pathlib import Path

# Add project root to path so we can import src
project_root = Path(__file__).resolve().parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from src.bno085_dual_visualizer import main


if __name__ == "__main__":
    main()
