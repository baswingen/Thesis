"""
Dashboard Snapshot Generator
============================

Utility script to generate PNG images of the Button Matrix and Instructions
for use in papers and presentations. It leverages the existing PyQt6 
dashboard components.

Usage:
    python trial/generate_dashboard_snapshot.py
"""

import sys
from pathlib import Path
from typing import Dict, Optional, Tuple

# ──────────────────────────────────────────────────────────────────────────────
# PHASE 1 CONFIGURATION (Placement from slots)
# ──────────────────────────────────────────────────────────────────────────────

# Starting slot weights: index (0-7) -> weight_kg
PHASE1_SLOT_WEIGHTS = {
    0: 1.0, 1: 2.0, 
    2: 3.0, 3: 4.0, 
    4: 6.0
}

# The movement instruction
PHASE1_TARGET_SPOT = (1, 1)    # Where the weight is going (B2)
PHASE1_SOURCE_SLOT = 2        # The index of the slot it comes from (S3)
PHASE1_WEIGHT_VALUE = 4.0     # The weight being moved

PHASE1_OUTPUT_NAME = "docs/paper_figures/phase1_placement.png"

# ──────────────────────────────────────────────────────────────────────────────
# PHASE 2 CONFIGURATION (Movement on matrix)
# ──────────────────────────────────────────────────────────────────────────────

# Matrix weights: (row, col) -> weight_kg
# Rows: 0=A, 1=B, 2=C | Cols: 0=1, 1=2, 2=3, 3=4
PHASE2_MATRIX_WEIGHTS = {
    (0, 4): 6.0, 
    (1, 1): 3.0, 
    (2, 0): 1.0,
    (2, 2): 4.0,
    (0, 2): 2.0,
    (0, 3): 6.0
}

# The movement instruction
PHASE2_SOURCE_SPOT = (0, 3)   # Where the weight is coming from (A1)
PHASE2_TARGET_SPOT = (1, 0)   # Where the weight is going (C3)
PHASE2_WEIGHT_VALUE = 6.0     # The weight being moved

PHASE2_PROGRESS_TEXT = "Move 2 / 15"
PHASE2_OUTPUT_NAME   = "docs/paper_figures/phase2_movement.png"

# ──────────────────────────────────────────────────────────────────────────────

# Path setup to ensure imports work
_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

try:
    from PyQt6 import QtWidgets, QtCore, QtGui
    _QT_OK = True
except ImportError:
    _QT_OK = False
    print("ERROR: PyQt6 is required to generate snapshots.")

from trial.dashboard import _ButtonMatrixPanel

class DashboardSnapshotGenerator:
    """
    A utility class to configure and render the button matrix dashboard panel
    to a file.
    """
    def __init__(self):
        if not _QT_OK:
            raise RuntimeError("PyQt6 not installed.")

        # We need a QApplication to use widgets
        self.app = QtWidgets.QApplication.instance()
        if self.app is None:
            self.app = QtWidgets.QApplication(sys.argv)
        
        self.panel = _ButtonMatrixPanel()
        self.panel.setFixedSize(600, 680)
        self.panel.setAutoFillBackground(True)
        
        from trial.dashboard import _COMMON_STYLE
        self.panel.setStyleSheet(_COMMON_STYLE + f" QFrame {{ background-color: #2c3e50; border-radius: 12px; }}")

    def generate(self, 
                 matrix_weights: Dict[Tuple[int, int], float] = None,
                 slot_weights: Dict[int, float] = None,
                 instruction: str = "Instruction",
                 progress: str = "Setup",
                 target: Optional[Tuple[int, int]] = None,
                 source: Optional[Tuple[int, int]] = None,
                 source_slot_idx: Optional[int] = None,
                 keys_mask: int = 0,
                 error_spot: Optional[Tuple[int, int]] = None,
                 output_path: str = "snapshot.png"):
        
        self.panel.update_weights(matrix_weights or {}, slot_weights or {})
        
        source_is_slot = source_slot_idx is not None
        self.panel.set_highlights(
            target=target, 
            source=source, 
            source_is_slot=source_is_slot, 
            source_slot_idx=source_slot_idx
        )
        
        self.panel.set_instruction(instruction, progress)
        self.panel.update_mask(keys_mask, error_spot=error_spot, rate_hz=0.0, sample_count=0)
        
        self.panel.show()
        for _ in range(5):
            QtWidgets.QApplication.processEvents()
            QtCore.QThread.msleep(10)
        
        pixmap = self.panel.grab()
        out_file = Path(output_path)
        out_file.parent.mkdir(parents=True, exist_ok=True)
        
        if pixmap.save(str(out_file)):
            print(f"Successfully generated: {out_file.absolute()}")
        else:
            print(f"Failed to save: {out_file}")
            
        self.panel.hide()

def run_configured_examples():
    gen = DashboardSnapshotGenerator()
    
    # 1. Phase 1 Placement (Using top-level config)
    # We leave the instruction empty as requested for this phase.
    gen.generate(
        matrix_weights={},
        slot_weights=PHASE1_SLOT_WEIGHTS,
        instruction="—",
        progress="Setup",
        target=None,
        source_slot_idx=PHASE1_SOURCE_SLOT,
        output_path=PHASE1_OUTPUT_NAME
    )

    # 2. Phase 2 Movement (Using top-level config)
    sr, sc = PHASE2_SOURCE_SPOT
    tr, tc = PHASE2_TARGET_SPOT
    instr = f"Move {PHASE2_WEIGHT_VALUE}kg from {chr(ord('A')+sr)}{sc+1} to {chr(ord('A')+tr)}{tc+1}"
    
    gen.generate(
        matrix_weights=PHASE2_MATRIX_WEIGHTS,
        instruction=instr,
        progress=PHASE2_PROGRESS_TEXT,
        target=PHASE2_TARGET_SPOT,
        source=PHASE2_SOURCE_SPOT,
        output_path=PHASE2_OUTPUT_NAME
    )

if __name__ == "__main__":
    run_configured_examples()

