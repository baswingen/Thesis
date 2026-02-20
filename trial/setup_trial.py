"""
Trial Data Collection - Configuration
======================================
Central configuration file for trial data collection system.

Edit this file to configure:
- Participant and session information
- Device settings (EMG, IMU)
- Trial protocol (exercises)
- Data preprocessing parameters
- Storage options

Usage:
    1. Edit configuration parameters below
    2. Run: python -m trial.trial_manager
"""

import sys
import io
import random
from pathlib import Path
from typing import List, Dict, Optional, Tuple

# Fix Windows console encoding issues
if sys.platform == 'win32':
    try:
        if hasattr(sys.stdout, 'buffer'):
            sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
        if hasattr(sys.stderr, 'buffer'):
            sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')
    except:
        pass

# =============================================================================
# PARTICIPANT AND SESSION INFORMATION
# =============================================================================

# Participant identification
PARTICIPANT_ID = "P001"
PARTICIPANT_INFO = {
    'age': 25,
    'gender': 'M',  # 'M', 'F', 'Other', 'Prefer not to say'
    'handedness': 'right',  # 'right', 'left', 'ambidextrous'
    'medical_notes': '',  # Any relevant medical information
}

# Session identification
SESSION_ID = "S001"
SESSION_INFO = {
    'experimenter': 'Researcher Name',
    'notes': '',  # Session-specific notes
}

# =============================================================================
# EMG DEVICE CONFIGURATION
# =============================================================================

EMG_CONFIG = {
    # Connection
    'connection_type': 'usb',  # 'usb', 'bluetooth', 'network', 'wifi'
    'device_index': 0,  # Device index if multiple devices connected
    
    # Sampling
    'sample_rate': 2048,  # Hz, None = use device default
    'reference_calculation': False,  # Common average reference
    
    # Channel selection
    'raw_channels': [0, 1, 2, 3],  # List of channel indices to record (0-based)
    'differential_pairs': None,  # Optional: [(0,1), (2,3)] for differential recording
    
    # Channel names (optional, for metadata)
    'channel_names': ['EMG_CH1', 'EMG_CH2', 'EMG_CH3', 'EMG_CH4'],
}

# =============================================================================
# IMU DEVICE CONFIGURATION
# =============================================================================

IMU_CONFIG = {
    # Connection
    'port': None,  # Serial port, None = auto-detect
    'baud': 115200,  # Baud rate (BNO085 RVC default)
    'timeout': 2.0,  # Serial timeout in seconds
    'imu_type': 'bno085_dual_rvc',  # IMU mode (for metadata)
    
    # Calibration
    'calibration_samples': 50,  # Reference samples (BNO085) / gyro bias samples (BMI160)
    'auto_calibrate_on_start': True,  # Calibrate automatically when starting
    
    # Filtering
    'use_mahony': False,  # BMI160-only; BNO085 has built-in fusion
    'kp': 2.0,  # Mahony proportional gain
    'ki': 0.01,  # Mahony integral gain
    'adaptive_gains': True,  # Enable adaptive gain scheduling
    
    # Scaling (depends on Arduino configuration)
    'accel_scale': 1.0 / 16384.0,  # ±2g range
    'gyro_scale': 1.0 / 131.2,     # ±250°/s range
}

# =============================================================================
# TRIAL PROTOCOL CONFIGURATION
# =============================================================================

# Button Matrix Configuration
BUTTON_MATRIX_CONFIG = {
    'objects': ['1kg', '2kg', '4kg', '6kg'],
    'iterations': 100,
    # Keys 0-11 map to A1-C4 representing the 3x4 grid
    'grid_rows': ['A', 'B', 'C'],
    'grid_cols': ['1', '2', '3', '4'],
    'timeout_per_movement': 15.0,  # Max time before recording it as a failed or skipped movement
}

def generate_button_matrix_sequence() -> List[Dict]:
    """
    Generate a sequence of randomized instructions for the button matrix protocol.
    Guarantees that multiple objects will not overlap on the same grid coordinate.
    """
    objects = BUTTON_MATRIX_CONFIG['objects']
    grid_rows = BUTTON_MATRIX_CONFIG['grid_rows']
    grid_cols = BUTTON_MATRIX_CONFIG['grid_cols']
    iterations = BUTTON_MATRIX_CONFIG['iterations']
    timeout = BUTTON_MATRIX_CONFIG['timeout_per_movement']
    
    all_positions = [f'{r}{c}' for r in grid_rows for c in grid_cols]
    pos_to_idx = {pos: i for i, pos in enumerate(all_positions)}
    
    exercises = []
    
    # 1. Initial placement
    available_positions = list(all_positions)
    random.shuffle(available_positions)
    
    current_state = {}
    
    for obj in objects:
        pos = available_positions.pop()
        current_state[obj] = pos
        idx = pos_to_idx[pos]
        
        exercises.append({
            'name': f'Init_{obj}_{pos}',
            'target_label': pos,
            'target_button_idx': idx,
            'object_id': obj,
            'display_name': f'Setup: Place {obj}',
            'instruction': f'Place the {obj} object on {pos}.',
            'duration': timeout,
            'rest_after': 1.0,
            'repetitions': 1,
            'advance_on': 'button_press'
        })
        
    # 2. Random movements
    for i in range(iterations):
        # Pick random object
        obj = random.choice(objects)
        
        # Pick random empty position
        occupied = set(current_state.values())
        empty_positions = [p for p in all_positions if p not in occupied]
        target_pos = random.choice(empty_positions)
        
        # Capture origin before updating
        origin_pos = current_state[obj]
        origin_idx = pos_to_idx[origin_pos]
        
        # Update state
        current_state[obj] = target_pos
        idx = pos_to_idx[target_pos]
        
        exercises.append({
            'name': f'Move_{i+1}_{obj}_{target_pos}',
            'target_label': target_pos,
            'target_button_idx': idx,
            'origin_label': origin_pos,
            'origin_button_idx': origin_idx,
            'object_id': obj,
            'display_name': f'Move {obj}',
            'instruction': f'Move the {obj} object from {origin_pos} to {target_pos}.',
            'duration': timeout,
            'rest_after': 0.5,
            'repetitions': 1,
            'advance_on': 'button_press'
        })
        
    return exercises

TRIAL_EXERCISES = generate_button_matrix_sequence()

# =============================================================================
# DATA PREPROCESSING CONFIGURATION
# =============================================================================

PREPROCESSING = {
    # Bandpass filter
    'bandpass_low': 20.0,   # Hz, low cutoff frequency
    'bandpass_high': 450.0, # Hz, high cutoff frequency
    'bandpass_order': 4,    # Filter order
    
    # Notch filter (powerline interference)
    'notch_freq': 50.0,     # Hz, 50 or 60 depending on region, None to disable
    'notch_quality': 30.0,  # Quality factor
    
    # Envelope extraction
    'envelope_cutoff': 10.0,  # Hz, low-pass cutoff for envelope
    'envelope_order': 4,      # Filter order
    
    # Normalization
    'mvc_normalization': False,  # Set to True after MVC calibration
    'mvc_value': None,           # MVC value, None = not calibrated
}

# =============================================================================
# STORAGE CONFIGURATION
# =============================================================================

# Database root directory
DATABASE_ROOT = Path('./database')

# What to save
SAVE_RAW = True          # Save raw EMG and IMU data
SAVE_PREPROCESSED = True  # Save preprocessed EMG data

# File format
DATA_FORMAT = 'hdf5'  # 'hdf5', 'numpy', or 'both'

# Compression (for HDF5)
COMPRESSION = 'gzip'  # 'gzip', 'lzf', None
COMPRESSION_LEVEL = 4  # 0-9 for gzip, higher = more compression but slower

# Backup options
AUTO_BACKUP = False  # Automatically create backup copies
BACKUP_FORMATS = ['numpy']  # Additional formats for backup

# =============================================================================
# GUI CONFIGURATION
# =============================================================================

GUI_CONFIG = {
    # Window
    'window_title': 'EMG+IMU Trial Data Collection',
    'window_width': 1200,
    'window_height': 700,
    'fullscreen': False,
    
    # Update rates
    'plot_update_interval_ms': 50,  # Milliseconds between plot updates
    'status_update_interval_ms': 100,  # Milliseconds between status updates
    
    # Plot settings
    'emg_plot_window_s': 5.0,  # Seconds of EMG data to display
    'imu_plot_window_s': 5.0,  # Seconds of IMU data to display
    'emg_channels_to_plot': 4,  # Number of EMG channels to plot
    
    # Colors
    'color_idle': '#4CAF50',      # Green
    'color_ready': '#2196F3',     # Blue
    'color_recording': '#F44336',  # Red
    'color_saving': '#FF9800',    # Orange
    'color_completed': '#9C27B0', # Purple
    
    # Fonts
    'font_instruction': ('Arial', 24, 'bold'),
    'font_status': ('Arial', 14),
    'font_info': ('Arial', 12),
}

# =============================================================================
# SYNCHRONIZATION CONFIGURATION
# =============================================================================

SYNC_CONFIG = {
    # Enable devices
    'enable_emg': True,
    'enable_imu': True,
    
    # Dummy signal mode (for testing without hardware)
    'use_dummy_signals': False,  # Set to True to use simulated signals
    'dummy_emg_amplitude': 50.0,  # Microvolts, peak amplitude
    'dummy_emg_noise_level': 5.0,  # Microvolts, noise amplitude
    'dummy_imu_motion': True,  # Add simulated motion to IMU
    
    # Buffer sizes
    'emg_buffer_max_chunks': 10000,
    'imu_buffer_max_samples': 200000,
    
    # Timing
    'ready_timeout_s': 30.0,  # Timeout for device initialization
    'poll_interval_s': 0.01,   # EMG polling interval
}

# =============================================================================
# ADVANCED CONFIGURATION (usually don't need to change)
# =============================================================================

# Countdown before trial starts
COUNTDOWN_DURATION = 3  # seconds

# Trial state timeouts
STATE_TIMEOUTS = {
    'calibration': 60.0,  # seconds
    'recording': 3600.0,  # seconds (1 hour max per trial)
    'saving': 60.0,       # seconds
}

# Data quality checks
QUALITY_CHECKS = {
    'check_emg_saturation': True,
    'saturation_threshold': 0.95,  # Fraction of max value
    'check_imu_motion': True,
    'motion_threshold': 10.0,  # degrees/s
}

# Logging
LOGGING_CONFIG = {
    'level': 'INFO',  # 'DEBUG', 'INFO', 'WARNING', 'ERROR'
    'log_to_file': True,
    'log_file': 'trial_log.txt',
}

# =============================================================================
# VALIDATION
# =============================================================================

def validate_config():
    """
    Validate configuration parameters.
    
    Raises:
        ValueError: If configuration is invalid
    """
    # Check participant ID
    if not PARTICIPANT_ID or not isinstance(PARTICIPANT_ID, str):
        raise ValueError("PARTICIPANT_ID must be a non-empty string")
    
    # Check session ID
    if not SESSION_ID or not isinstance(SESSION_ID, str):
        raise ValueError("SESSION_ID must be a non-empty string")
    
    # Check EMG config
    if EMG_CONFIG['sample_rate'] is not None and EMG_CONFIG['sample_rate'] <= 0:
        raise ValueError("EMG sample_rate must be positive or None")
    
    if not EMG_CONFIG['raw_channels'] and not EMG_CONFIG['differential_pairs']:
        raise ValueError("Must specify either raw_channels or differential_pairs")
    
    # Check trial exercises
    if not TRIAL_EXERCISES or len(TRIAL_EXERCISES) == 0:
        raise ValueError("TRIAL_EXERCISES must contain at least one exercise")
    
    for i, ex in enumerate(TRIAL_EXERCISES):
        if 'name' not in ex:
            raise ValueError(f"Exercise {i} missing 'name' field")
        if 'duration' not in ex or ex['duration'] <= 0:
            raise ValueError(f"Exercise {i} must have positive 'duration'")
        if 'instruction' not in ex:
            raise ValueError(f"Exercise {i} missing 'instruction' field")
    
    # Check preprocessing
    if PREPROCESSING['bandpass_low'] >= PREPROCESSING['bandpass_high']:
        raise ValueError("bandpass_low must be less than bandpass_high")
    
    # Check database root
    if not DATABASE_ROOT:
        raise ValueError("DATABASE_ROOT must be specified")
    
    print("[OK] Configuration validated successfully")


def get_session_dir() -> Path:
    """
    Get the directory for the current session.
    
    Returns:
        Path to session directory
    """
    session_dir = DATABASE_ROOT / f"participant_{PARTICIPANT_ID}" / f"session_{SESSION_ID}"
    return session_dir


def print_config_summary():
    """Print a summary of the current configuration."""
    print("\n" + "="*70)
    print("TRIAL CONFIGURATION SUMMARY")
    print("="*70)
    print(f"\nParticipant: {PARTICIPANT_ID}")
    print(f"Session: {SESSION_ID}")
    print(f"\nDevices:")
    
    # Show dummy signal mode
    if SYNC_CONFIG.get('use_dummy_signals', False):
        print(f"  ⚠ USING DUMMY SIGNALS (simulated - no hardware)")
        print(f"     EMG amplitude: {SYNC_CONFIG.get('dummy_emg_amplitude', 50.0)} µV")
        print(f"     EMG noise: {SYNC_CONFIG.get('dummy_emg_noise_level', 5.0)} µV")
        print(f"     IMU motion: {SYNC_CONFIG.get('dummy_imu_motion', True)}")
    
    print(f"  EMG: {'Enabled' if SYNC_CONFIG['enable_emg'] else 'Disabled'}")
    print(f"       Sample rate: {EMG_CONFIG['sample_rate']} Hz")
    print(f"       Channels: {len(EMG_CONFIG['raw_channels'])} raw channels")
    print(f"  IMU: {'Enabled' if SYNC_CONFIG['enable_imu'] else 'Disabled'}")
    print(f"       Auto-calibrate: {IMU_CONFIG['auto_calibrate_on_start']}")
    print(f"\nProtocol: Button Matrix")
    print(f"  Exercises: {len(TRIAL_EXERCISES)}")
    total_duration = sum(ex['duration'] + ex.get('rest_after', 0) for ex in TRIAL_EXERCISES)
    print(f"  Estimated duration: {total_duration:.1f} seconds ({total_duration/60:.1f} minutes)")
    print(f"\nStorage:")
    print(f"  Database: {DATABASE_ROOT}")
    print(f"  Save raw: {SAVE_RAW}")
    print(f"  Save preprocessed: {SAVE_PREPROCESSED}")
    print(f"  Format: {DATA_FORMAT}")
    print("="*70)


# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def get_emg_selection():
    """
    Get EMG channel selection for SynchronizedAcquisition.
    
    Returns:
        EMGSelection object
    """
    from src.synchronized_acquisition import EMGSelection
    
    return EMGSelection(
        raw_channels=EMG_CONFIG['raw_channels'],
        differential_pairs=EMG_CONFIG['differential_pairs']
    )


def get_imu_config():
    """
    Get IMU configuration for SynchronizedAcquisition.
    
    Returns:
        IMUConfig object
    """
    from src.imu_acquisition import IMUConfig, IMUType
    
    return IMUConfig(
        imu_type=IMUType.BNO085_DUAL_RVC,
        port=IMU_CONFIG['port'],
        baud=IMU_CONFIG['baud'],
        timeout=IMU_CONFIG.get('timeout', 2.0),
        accel_scale=IMU_CONFIG['accel_scale'],
        gyro_scale=IMU_CONFIG['gyro_scale'],
        use_mahony=IMU_CONFIG['use_mahony'],
        kp=IMU_CONFIG['kp'],
        ki=IMU_CONFIG['ki'],
        adaptive_gains=IMU_CONFIG['adaptive_gains'],
        bno085_calib_samples=IMU_CONFIG['calibration_samples'],
    )


def get_sync_config():
    """
    Get synchronization configuration.
    
    Returns:
        SyncConfig object
    """
    from src.synchronized_acquisition import SyncConfig, IMUSyncConfig, EMGSyncConfig
    
    imu_sync_cfg = IMUSyncConfig(
        imu_config=get_imu_config(),
        calibration_samples=IMU_CONFIG['calibration_samples'],
        buffer_max_samples=SYNC_CONFIG['imu_buffer_max_samples'],
    )
    
    emg_sync_cfg = EMGSyncConfig(
        connection_type=EMG_CONFIG['connection_type'],
        sample_rate=EMG_CONFIG['sample_rate'],
        selection=get_emg_selection(),
        poll_interval_s=SYNC_CONFIG['poll_interval_s'],
        buffer_max_chunks=SYNC_CONFIG['emg_buffer_max_chunks'],
    )
    
    return SyncConfig(
        enable_imu=SYNC_CONFIG['enable_imu'],
        enable_emg=SYNC_CONFIG['enable_emg'],
        imu=imu_sync_cfg,
        emg=emg_sync_cfg,
    )


# Run validation when module is imported
if __name__ != '__main__':
    try:
        validate_config()
    except Exception as e:
        print(f"[WARNING] Configuration validation failed: {e}")
        print("Please fix the configuration before running trials.")


# If run directly, start trial manager
if __name__ == '__main__':
    import sys
    import os
    import subprocess
    
    validate_config()
    print_config_summary()
    
    # Ask user to confirm before starting
    print("\n" + "="*70)
    response = input("Start trial data collection? [Y/n]: ").strip().lower()
    
    if response in ['', 'y', 'yes']:
        print("\nStarting trial manager...")
        print("="*70 + "\n")
        
        # Run trial manager as a subprocess to avoid circular imports
        try:
            # Try running as module first
            result = subprocess.run(
                [sys.executable, '-m', 'trial.trial_manager'],
                cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            )
            sys.exit(result.returncode)
        except Exception as e:
            print(f"[ERROR] Failed to start trial manager: {e}")
            print("\nTo start manually, run:")
            print("  python -m trial.trial_manager")
            sys.exit(1)
    else:
        print("\nCancelled. To start later, run:")
        print("  python -m trial.trial_manager")
        print("  or")
        print("  python trial/setup_trial.py")
