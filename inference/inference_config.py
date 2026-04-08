"""
inference_config.py
===================

Central configuration for the real-time inference system.

All relative paths are resolved from the project root at runtime.
To switch models, change MODEL_RUN_DIR to any other run folder that
contains `cnn_lstm_model.joblib` and `performance_report.txt`.
"""

from pathlib import Path

# ---------------------------------------------------------------------------
# Project root (resolved at import time)
# ---------------------------------------------------------------------------
PROJECT_ROOT = Path(__file__).resolve().parents[1]

# ---------------------------------------------------------------------------
# Model configuration
# ---------------------------------------------------------------------------

# Relative path from the project root to the run directory that contains:
#   - cnn_lstm_model.joblib
#   - performance_report.txt  (parsed for display metadata)
MODEL_RUN_DIR: str = "model/model_results/run_20260404_191021"

# Architecture key — used for display only; actual hyperparameters are
# extracted directly from the .joblib checkpoint.
ARCHITECTURE: str = "CNN-LSTM"

# ---------------------------------------------------------------------------
# Signal window — informed by seqlen performance plot
# MAE bottoms out at ~1.17 s (w=7).  1.2 s gives a small safety margin.
# ---------------------------------------------------------------------------
WINDOW_S: float = 1.2          # rolling segment duration fed to the model

# ---------------------------------------------------------------------------
# Inference cadence — start slow, tune upward later by changing this value
# ---------------------------------------------------------------------------
INFERENCE_STEP_S: float = 2.0  # seconds between successive predictions

# ---------------------------------------------------------------------------
# Temporal smoothing (Exponential Moving Average)
# alpha=1.0 → no smoothing (raw predictions only)
# alpha=0.4 → moderate smoothing (new data weighted 40%, history 60%)
# ---------------------------------------------------------------------------
PREDICTION_EMA_ALPHA: float = 0.4

# ---------------------------------------------------------------------------
# Hardware — STM32
# ---------------------------------------------------------------------------
STM32_PORT = None       # None = auto-detect via arduino_connection.find_arduino_port
STM32_BAUD: int = 921600

# ---------------------------------------------------------------------------
# Hardware — TMSi Porti7 (EMG)
# ---------------------------------------------------------------------------
EMG_SAMPLE_RATE: int = 2000     # Hz — must match device configuration
EMG_CONNECTION_TYPE: str = "usb"

# ---------------------------------------------------------------------------
# PRBS synchronisation (must match STM32 firmware constants)
# ---------------------------------------------------------------------------
PRBS_CHIP_RATE_HZ: float = 100.0
PRBS_CORRELATION_WINDOW_S: float = 3.0
PRBS_UPDATE_INTERVAL_S: float = 0.5
SYNC_MIN_CONFIDENCE: float = 0.4   # block inference below this threshold

# ---------------------------------------------------------------------------
# Missing channel policy
# ---------------------------------------------------------------------------
ZERO_FILL_MISSING: bool = True  # if False, raises RuntimeError for missing channels

# ---------------------------------------------------------------------------
# Rolling buffer limits
# ---------------------------------------------------------------------------
EMG_BUFFER_S: float = 12.0     # seconds of EMG history to keep
STM32_BUFFER_S: float = 12.0   # seconds of STM32 history to keep

# ---------------------------------------------------------------------------
# Dashboard refresh rates
# ---------------------------------------------------------------------------
FAST_REFRESH_MS: int = 33       # 30 fps — header, health LEDs, hero readout
PLOT_REFRESH_MS: int = 100      # 10 fps — EMG scrolling plots, prediction chart
IMU_REFRESH_MS: int = 66        # 15 fps — 3D IMU cubes

# ---------------------------------------------------------------------------
# EMG display
# ---------------------------------------------------------------------------
EMG_SCROLL_S: float = 2.0       # seconds of EMG history shown per channel plot
N_EMG_DISPLAY: int = 8          # number of EMG channel plots shown

# ---------------------------------------------------------------------------
# Prediction history chart
# ---------------------------------------------------------------------------
PREDICTION_HISTORY_S: float = 60.0  # seconds of prediction history shown

# ---------------------------------------------------------------------------
# Channel Mappings (must match model/config_model.py exactly)
# The order here defines the 26-channel input to the model.
# ---------------------------------------------------------------------------

# 1-8: EMG Channels (8 total)
# Format: { MuscleName: channel_id or (ch_plus, ch_minus) }
EMG_CHANNEL_MAPPING = {
    "Anterior Deltoid":  ("ch1", "ch2"),
    "Lateral Deltoid":   ("ch3", "ch4"),
    "Posterior Deltoid": ("ch5", "ch6"),
    "Triceps Brachii":   ("ch7", "ch8"),
    "Biceps Brachii":    "ch17",
    "Brachioradialis":   "ch18",
    "Flexor Carpi Ulnaris (FCU)":  "ch19",
    "Extensor Carpi Radialis (ECR)": "ch20",
}

# 9-26: IMU Channels (18 total)
# These are grouped by Sensor 1, Sensor 2, and Difference (S2-S1)
IMU_CHANNEL_MAPPING = [
    # Sensor 1
    "ax1", "ay1", "az1", "roll_rad1", "pitch_rad1", "yaw_rad1",
    # Sensor 2
    "ax2", "ay2", "az2", "roll_rad2", "pitch_rad2", "yaw_rad2",
    # Differential (Elbow Kinematics)
    "ax_diff", "ay_diff", "az_diff", "roll_rad_diff", "pitch_rad_diff", "yaw_rad_diff",
]


# ──────────────────────────────────────────────────────────
# PREPROCESSING & NORMALIZATION
# ──────────────────────────────────────────────────────────

# P01 Session 03 Normalization Stats (Median, IQR)
# Extracted from database/participant_P01 using model/postprocessing.py
EMG_NORMALIZATION_STATS = {
    "Anterior Deltoid":  {"median": 35.1, "iqr": 44.2},
    "Lateral Deltoid":   {"median": 44.8, "iqr": 64.3},
    "Posterior Deltoid": {"median": 9.05, "iqr": 7.76},
    "Triceps Brachii":   {"median": 14.7, "iqr": 27.8},
    "Biceps Brachii":    {"median": 15.6, "iqr": 15.2},
    "Brachioradialis":   {"median": 18.9, "iqr": 38.1},
    "Flexor Carpi Ulnaris (FCU)":  {"median": 14.8, "iqr": 19.0},
    "Extensor Carpi Radialis (ECR)": {"median": 21.7, "iqr": 34.4},
}

# Windowing parameters (MUST match training config in model/config_model.py)
# Training used emg_window_size_sec=0.15 and window_step_sec=0.15
INFERENCE_WINDOW_S = 0.15
TICK_RATE_HZ = 1.0 / 0.15  # ~6.67 Hz (updates model every 150ms)

