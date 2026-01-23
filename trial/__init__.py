"""
Trial Data Collection Module
=============================

Comprehensive trial data collection system for synchronized EMG and IMU signals.

Main components:
- setup_trial: Configuration
- trial_manager: Core orchestration
- trial_gui: GUI interface
- data_storage: HDF5 storage
- trial_protocols: Protocol definitions

Quick Start:
    1. Edit configuration in setup_trial.py
    2. Run: python -m trial.trial_manager
    3. Follow GUI instructions

Mock Mode (no hardware):
    python -m trial.trial_manager --mock
"""

__version__ = '1.0.0'
__author__ = 'EMG+IMU Research Team'

# Lazy imports to avoid requiring matplotlib for basic usage
__all__ = [
    'TrialManager',
    'TrialDataStorage',
    'TrialGUI',
    'TrialState',
    'get_protocol',
    'list_protocols',
    'expand_protocol',
    'create_custom_protocol',
    'PROTOCOLS',
]

def __getattr__(name):
    """Lazy import of module components."""
    if name == 'TrialManager':
        from .trial_manager import TrialManager
        return TrialManager
    elif name == 'TrialDataStorage':
        from .data_storage import TrialDataStorage
        return TrialDataStorage
    elif name in ['TrialGUI', 'TrialState']:
        from .trial_gui import TrialGUI, TrialState
        return TrialGUI if name == 'TrialGUI' else TrialState
    elif name in ['get_protocol', 'list_protocols', 'expand_protocol', 
                  'create_custom_protocol', 'PROTOCOLS']:
        from . import trial_protocols
        return getattr(trial_protocols, name)
    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")
