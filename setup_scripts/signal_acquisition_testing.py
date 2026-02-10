"""
Synchronized STM32 + EMG Signal Acquisition (performance testing)
================================================================

Thin CLI wrapper around the native signal acquisition module in src.
Runs the same modes (stm32, emg, prbs, all, sequence) by calling
src.signal_acquisition.main().

Performance tests for real-time synchronized acquisition of:
- Dual BNO085 IMU + button matrix + PRBS (via STM32F401, 500 Hz)
- Multi-channel EMG differential pairs (via TMSi Porti7, ~2000 Hz)

PRBS synchronization:
    Uses prbs_tick and prbs_mark from the STM32 to reconstruct the full
    2000 Hz PRBS signal on the host (no need to transmit raw PRBS over USB).
    Cross-correlates reconstructed PRBS with EMG TRIG for clock offset.

Hardware setup:
    STM32F401 PA8 --[330R]-- Porti7 TRIG (LEMO centre)
    STM32F401 GND ---------- Porti7 TRIG (LEMO shield)

Run modes:
    --mode sequence  Run in order: Phase 1 STM32, Phase 2 EMG, Phase PRBS, Phase 3 combined (default).
    --mode stm32     Phase 1 only: STM32 serial/parse rate (target 500 Hz).
    --mode emg       Phase 2 only: EMG rate (target ~2000 Hz).
    --mode prbs      PRBS-on-EMG test only.
    --mode all       Phase 3 only: STM32 + EMG + PRBS sync.

Example:
    python signal_acquisition_testing.py --mode sequence --duration 10
    python signal_acquisition_testing.py --mode all --duration 15

The implementation lives in src.signal_acquisition and src.stm32_reader.
Use SignalAcquisition from src in your code for programmatic access:

    from src import SignalAcquisition, SignalAcquisitionConfig
    config = SignalAcquisitionConfig()
    acq = SignalAcquisition(config)
    acq.start()
    snap = acq.get_stm32_snapshot(500)
    acq.stop()
"""

from __future__ import annotations

import sys
from pathlib import Path

# Ensure project root is on path when running from setup_scripts
_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.signal_acquisition import main

if __name__ == "__main__":
    main()
