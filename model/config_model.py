"""
Configuration file for model hyperparameters.
Centralizing these values makes it easier to manage experiments and 
ensures consistency across training and inference scripts.
"""

GLOBAL_RANDOM_STATE = 245
GLOBAL_LOSS_FUNCTION = 'mse'  # Options: 'mse', 'mae', or 'huber'
GLOBAL_BALANCE_WEIGHTS = False

# ──────────────────────────────────────────────────────────
# RUN_MODEL PIPELINE TOGGLES
# ──────────────────────────────────────────────────────────
MODEL_TYPE = "cnn_gru"  # Options: "svr", "rf", "gb", "mlp", "gru", "lstm", "cnn_lstm", "cnn_gru", "cnn_bilstm_attention", "tcn", "transformer"
RUN_GRID_SEARCH = False
USE_PRECOMPUTED_FEATURES = True

# ──────────────────────────────────────────────────────────
# DATABASE & ENVIRONMENT CONFIGURATION
# ──────────────────────────────────────────────────────────
import os
from pathlib import Path

# Detect if we are on the DelftBlue server
ON_DELFTBLUE = os.getenv("SLURM_JOB_ID") is not None

# Define the root of the database depending on the environment.
# Local: project-relative 'database/'
# DelftBlue: high-performance '/scratch/bwingen/thesis/database/'
if ON_DELFTBLUE:
    DATABASE_ROOT = Path("/scratch/bwingen/thesis/database")
else:
    DATABASE_ROOT = Path(__file__).parent.parent / "database"

DATABASE_CONFIG = {
    'root': DATABASE_ROOT,
    'segments_dir': DATABASE_ROOT / "segments",
    'raw_dir': DATABASE_ROOT,
}

###########################################################
# PARTICIPANT CONFIGURATION
###########################################################
# Controls which participants are included in the training/evaluation.
PARTICIPANT_CONFIG = {
    'include': 'all',  # Options: 'all' or a list of IDs (e.g., ['P01', 'P02'])
}

###########################################################
# WEIGHT CALIBRATION CONFIGURATION
###########################################################
# Maps nominal weight labels to precise measured weights in real life.
# If a weight is not found in this list, the fallback is the original label.
TRUE_WEIGHTS = {
    0.75: 0.899,
    1.0: 0.979,
    2.0: 1.966,
    2.25: 2.238,
    3.0: 2.945,
    4.25: 4.142,
    6.0: 5.922,
}

###########################################################
# RAW CHANNEL TOGGLES (apply to ALL architectures)
# Set False to exclude a channel from both raw-segment
# (CNN-LSTM) and feature-extraction (LSTM/GRU/…) pipelines.
###########################################################

CHANNEL_CONFIG = {
    # ── EMG Channels ─────────────────────────────────────────
    'emg_channels': {
        "Anterior Deltoid": True,
        "Lateral Deltoid": True,
        "Posterior Deltoid": True,
        "Triceps Brachii": True,
        "Biceps Brachii": True,
        "Brachioradialis": True,
        "Flexor Carpi Ulnaris (FCU)": True,
        "Extensor Carpi Radialis (ECR)": True,
    },

    # ── IMU Channels ─────────────────────────────────────────
    'imu_channels': {
        # Sensor 1
        'ax1': True, 'ay1': True, 'az1': True,
        'roll_rad1': True, 'pitch_rad1': True, 'yaw_rad1': True,
        # Sensor 2
        'ax2': True, 'ay2': True, 'az2': True,
        'roll_rad2': True, 'pitch_rad2': True, 'yaw_rad2': True,
        # Differential (Elbow Kinematics)
        'ax_diff': True, 'ay_diff': True, 'az_diff': True,
        'roll_rad_diff': True, 'pitch_rad_diff': True, 'yaw_rad_diff': True,
    },
}

###########################################################
# EMG CHANNEL CONFIGURATION
###########################################################
EMG_CHANNEL_CONFIG = {
    # Double-channel muscles (differential pair)
    "Anterior Deltoid":  ("ch1", "ch2"),
    "Lateral Deltoid":   ("ch3", "ch4"),
    "Posterior Deltoid": ("ch5", "ch6"),
    "Biceps Brachii":   ("ch7", "ch8"),
    # Single-channel muscles
    "Triceps Brachii":    "ch17",
    "Brachioradialis":   "ch18",
    "Flexor Carpi Ulnaris (FCU)":  "ch19",
    "Extensor Carpi Radialis (ECR)": "ch20",
}

###########################################################
# FEATURE EXTRACTION CONFIGURATION
###########################################################
FEATURE_CONFIG = {

    # EMG threshold for WAMP & Myopulse features
    'emg_threshold': 1e-5,

    # Sliding window properties (for sequence models)
    # Updated to match LSTM sweep Iteration 68 best params
    'emg_window_size_sec': 0.15,
    'imu_window_size_sec': 0.2,
    'window_step_sec': 0.15,

    # Channel toggles have been moved to the top-level CHANNEL_CONFIG dict.


    # ── EMG Features (set False to disable) ──────────────
    'emg_features': {
        # Time-domain
        'MAV':        True,    # Mean Absolute Value
        'RMS':        True,   # Root Mean Square
        'WL':         True,    # Waveform Length
        'ZC':         True,   # Zero Crossings
        'SSC':        True,    # Slope Sign Changes
        'VAR':        True,    # Variance
        'WAMP':       True,   # Willison Amplitude
        'IEMG':       True,    # Integrated EMG
        'LogDet':     True,   # Log Detector
        'Skew':       True,    # Skewness
        'Kurt':       True,    # Kurtosis
        'HjMob':      True,   # Hjorth Mobility
        'HjComp':     True,   # Hjorth Complexity
        'Myopulse':   True,   # Myopulse Percentage Rate
        # Frequency-domain
        'MNF':        True,    # Mean Frequency
        'MDF':        True,    # Median Frequency
        'Power':      True,   # Total Spectral Power
        'SpecEntropy':True,    # Spectral Entropy
        'PeakFreq':   True,    # Peak Frequency
        'BW':         True,    # Bandwidth (95%)
    },

    # ── IMU Features (set False to disable) ──────────────
    # Trimmed to 13 features to offset cost of 6 new diff channels.
    # Disabled: Min (≈-Max), RMS (≈Std for zero-mean), IQR (≈Std),
    #           ZC (low load-estimation value), Energy (≈Var*N),
    #           SpecEntropy (low discriminability for rigid-body motion),
    #           SVM_Std (SVM_Mean captures the key cross-channel info).
    'imu_features': {
        # Time-domain (kept: 8)
        'Mean':       True,    # Mean value
        'Var':        True,    # Variance
        'Std':        True,    # Standard Deviation
        'Max':        True,    # Maximum
        'Min':        False,   # DISABLED — redundant with Max via P2P
        'RMS':        False,   # DISABLED — ≈Std for near-zero-mean signals
        'SMA':        True,    # Signal Magnitude Area
        'P2P':        True,    # Peak-to-Peak
        'IQR':        False,   # DISABLED — highly correlated with Std
        'Skew':       True,    # Skewness
        'Kurt':       True,    # Kurtosis
        'Jerk':       True,    # Mean Absolute Jerk
        'ZC':         False,   # DISABLED — low value for load estimation
        'Energy':     False,   # DISABLED — ≈Var*N (redundant)
        # Frequency-domain (kept: 3)
        'DomFreq':    True,    # Dominant Frequency
        'SpecEnergy': True,    # Spectral Energy
        'MNF':        True,    # Mean Frequency
        'MDF':        False,   # DISABLED — highly correlated with MNF
        'SpecEntropy':False,   # DISABLED — low discrimination for rigid-body
        # Cross-channel (kept: 1)
        'SVM_Mean':   True,    # Signal Vector Magnitude (mean)
        'SVM_Std':    False,   # DISABLED — SVM_Mean is the key statistic
    },
}

###########################################################
# DATA AUGMENTATION CONFIGURATION (LSTM / GRU only)
###########################################################
AUGMENTATION_CONFIG = {
    # Master toggle — set False to disable all augmentation
    'enabled': True,

    # Probability that any single training sample is augmented (0.0 – 1.0).
    # Each selected sample produces one additional augmented copy alongside
    # the original, so the dataset can grow up to 2× when p=1.0.
    'p': 0.5,

    # Active augmentation methods.  Remove a method name to disable it.
    # Available: 'noise', 'stretch', 'channel_dropout', 'magnitude_scale', 'mixup'
    'methods': ['noise', 'stretch', 'channel_dropout', 'magnitude_scale', 'mixup'],

    # ── Gaussian noise ────────────────────────────────────────────────
    # Standard deviation on the z-score scale (after StandardScaler).
    # 0.05 ≈ 5% of one standard deviation — very conservative.
    'noise_std': 0.05,

    # ── Temporal stretch ─────────────────────────────────────────────
    # Random scale factor applied to sequence length, then resampled back.
    # Increased to ±25% for more robust duration invariance.
    'stretch_factor_range': (0.75, 1.25),

    # ── Channel dropout ───────────────────────────────────────────────
    # Probability that an entire channel (feature) is zeroed for the full window.
    'channel_dropout_p': 0.25,

    # ── Magnitude scaling ─────────────────────────────────────────────
    # Per-feature multiplicative factor drawn uniformly from this range.
    # ±20% range targets inter-participant amplitude differences (EMG/IMU).
    'magnitude_scale_range': (0.5, 2.0),

    # ── MixUp ────────────────────────────────────────────────────────
    # Alpha parameter of the Beta(α, α) distribution for λ.
    # Increased to 0.5 to strongly support interpolating out-of-distribution masses
    # (like the underrepresented 5.92 kg).
    'mixup_alpha': 0.5,
}

# Cross-Validation Configuration
CV_CONFIG = {
    'use_cross_val': True,
    'train_test_split': 0.2,    # Only used if use_cross_val is False
    'n_folds': 5,               # Only used if strategy is 'kfold'
    'strategy': 'participant',  # Options: 'kfold', 'participant'
    'strict_val_split': True,   # Toggles strict cross-participant early stopping
    'strict_val_participants': 2 # Number of participants to hold out for early stopping
}

# SVM Configuration
SVM_CONFIG = {
    'kernel': 'sigmoid', 
    'C': 10.0,           
    'gamma': 0.1,      
    'class_weight': 'balanced',
    'random_state': GLOBAL_RANDOM_STATE
}

# RBF Neural Network Configuration
RBFNN_CONFIG = {
    'n_centers': 300,    
    'gamma': 0.001,      
    'C': 10.0,           
    'random_state': GLOBAL_RANDOM_STATE,
    'class_weight': 'balanced'
}

# SVR (Support Vector Regression) Configuration
# RBF kernel is used instead of linear to prevent unbounded extrapolation
# for out-of-distribution (unseen) participants in LOPO cross-validation.
# A linear kernel defines a global hyperplane that can wildly extrapolate
# when a test participant's features lie outside the training convex hull.
# RBF predictions decay smoothly toward the training mean for OOD inputs.
SVR_CONFIG = {
    'kernel': 'rbf',
    'C': 50.0,
    'epsilon': 0.05,
    'gamma': 'scale',
    'balance_weights': True,
    'random_state': GLOBAL_RANDOM_STATE
}

# Random Forest Regressor Configuration
RF_CONFIG = {
    'n_estimators': 50,
    'max_depth': 10,
    'min_samples_split': 2,
    'random_state': GLOBAL_RANDOM_STATE
}

# Gradient Boosting Regressor Configuration
GB_CONFIG = {
    'n_estimators': 200,
    'learning_rate': 0.05,
    'max_depth': 4,
    'random_state': GLOBAL_RANDOM_STATE
}
# MLP (Multi-Layer Perceptron) Configuration
MLP_CONFIG = {
    'hidden_layers': [256, 128, 64],
    'dropout_rate': 0.1,
    'learning_rate': 0.001,
    'weight_decay': 1e-4,
    'batch_size': 32,
    'epochs': 100,
    'validation_split': 0.2,
    'early_stopping_patience': 10,
    'scheduler_patience': 5,
    'scheduler_factor': 0.5,
    'random_state': GLOBAL_RANDOM_STATE
}

# GRU (Gated Recurrent Unit) Configuration
# Sweep Iteration 15 best params (R²=0.9891, MAE=0.0682)
GRU_CONFIG = {
    'hidden_size': 512,
    'num_layers': 2,
    'dropout_rate': 0.5,
    'learning_rate': 0.0005,
    'weight_decay': 1e-4,
    'batch_size': 32,
    'epochs': 600,
    'validation_split': 0.1,
    'early_stopping_patience': 200,
    'scheduler_patience': 10,
    'scheduler_factor': 0.5,
    'random_state': GLOBAL_RANDOM_STATE
}

# LSTM (Long Short-Term Memory) Configuration
# Sweep Iteration 68 best params (R²=0.9897, MAE=0.0768)
LSTM_CONFIG = {
    'hidden_size': 512,
    'num_layers': 3,
    'dropout_rate': 0.2,
    'learning_rate': 0.0005,
    'weight_decay': 0.001,
    'batch_size': 64,
    'epochs': 600,
    'validation_split': 0.1,
    'early_stopping_patience': 200,
    'scheduler_patience': 10,
    'scheduler_factor': 0.8,
    'random_state': GLOBAL_RANDOM_STATE
}

# CNN-LSTM Configuration (raw-segment end-to-end model)
# Sweep Iteration 60 best params (R²=0.9905, MAE=0.0619)
CNN_LSTM_CONFIG = {
    'cnn_filters': [64, 128, 256, 256],
    'cnn_kernel_sizes': [64, 5, 3, 3],
    'pool_size': 4,
    'lstm_hidden_size': 256,
    'lstm_num_layers': 3,
    'dropout_rate': 0.45,
    'learning_rate': 0.001,
    'weight_decay': 1e-3,
    'batch_size': 64,            # Raw segments padded per-batch; higher values OOM with long lifts
    'epochs': 400,
    'balance_weights': False,
    'balance_participants': False,
    'validation_split': 0.15,
    'early_stopping_patience': 50,
    'scheduler_patience': 10,
    'scheduler_factor': 0.5,
    'random_state': GLOBAL_RANDOM_STATE,
}

# CNN-GRU Configuration (raw-segment end-to-end model)
# Sweep 20260428_133031 winner — lighter model + sqrt target transform
# RMSE 0.662 | R² 0.865 | overfit 3.3× (was 20×)
CNN_GRU_CONFIG = {
    'cnn_filters': [32, 64, 128],
    'cnn_kernel_sizes': [32, 5, 3],
    'pool_size': 4,
    'gru_hidden_size': 256,
    'gru_num_layers': 2,
    'dropout_rate': 0.5,
    'learning_rate': 0.002,
    'weight_decay': 5e-4,
    'batch_size': 128,
    'epochs': 400,
    'target_transform': 'none',    # Options: 'none', 'sqrt', 'log1p'
    'balance_weights': False,
    'balance_participants': False,
    'validation_split': 0.15,
    'early_stopping_patience': 50,
    'scheduler_patience': 20,
    'scheduler_factor': 0.5,
    'random_state': GLOBAL_RANDOM_STATE,
}

# CNN-BiLSTM-Attention Configuration (raw-segment end-to-end model)
# Bidirectional LSTM with learned attention pooling over timesteps.
# Uses the same CNN backbone as CNN-LSTM/GRU but collapses temporal
# information via attention instead of taking the last hidden state.
CNN_BILSTM_ATTENTION_CONFIG = {
    'cnn_filters': [64, 128, 256, 256],
    'cnn_kernel_sizes': [64, 5, 3, 3],
    'pool_size': 4,
    'lstm_hidden_size': 192,        # BiLSTM doubles this → 384-dim context vector
    'lstm_num_layers': 2,
    'fc_units': 256,                # Deep regressor head: 256 → 128 → 1
    'dropout_rate': 0.4,
    'learning_rate': 0.001,
    'weight_decay': 1e-3,
    'batch_size': 64,
    'epochs': 400,
    'balance_weights': True,
    'balance_participants': True,
    'validation_split': 0.15,
    'early_stopping_patience': 200,
    'scheduler_patience': 10,
    'scheduler_factor': 0.5,
    'random_state': GLOBAL_RANDOM_STATE,
}

# Lighter CNN-LSTM Configuration specifically for Sequential Backward Selection Ablation
# Designed to be faster to train but still representative of relative channel importance.
CNN_LSTM_ABLATION_CONFIG = {
    'cnn_filters': [32, 64],
    'cnn_kernel_sizes': [64, 3],
    'pool_size': 4,
    'lstm_hidden_size': 128,
    'lstm_num_layers': 1,
    'dropout_rate': 0.4,
    'learning_rate': 0.0005,
    'weight_decay': 1e-05,
    'batch_size': 256,
    'epochs': 300,
    'validation_split': 0.2,
    'early_stopping_patience': 40,
    'scheduler_T_0': 50,
    'scheduler_T_mult': 2,
    'random_state': GLOBAL_RANDOM_STATE,
}

# TCN Configuration (causal raw-segment convolutional model)
TCN_CONFIG = {
    'cnn_filters': [64, 128, 256, 256],
    'cnn_kernel_sizes': [7, 5, 3, 3],
    'pool_size': 4,
    'tcn_channels': [128, 128, 256],
    'tcn_kernel_size': 3,
    'dropout_rate': 0.45,
    'learning_rate': 0.001,
    'weight_decay': 1e-3,  # Heavily increased regularisation for robust cross-participant performance
    'batch_size': 64,
    'epochs': 1000,
    'balance_weights': False,
    'validation_split': 0.15,
    'early_stopping_patience': 100,
    'scheduler_T_0': 50,       # Initial restart interval for CosineAnnealingWarmRestarts
    'scheduler_T_mult': 2,     # Multiplier for subsequent intervals
    'random_state': GLOBAL_RANDOM_STATE,
}


# Transformer Configuration
TRANSFORMER_CONFIG = {
    'd_model': 64,
    'nhead': 4,
    'num_layers': 3,
    'dim_feedforward': 128,
    'dropout_rate': 0.4,
    'learning_rate': 0.005,
    'weight_decay': 1e-4,
    'batch_size': 128,
    'epochs': 150,
    'validation_split': 0.1,
    'early_stopping_patience': 50,
    'scheduler_patience': 5,
    'scheduler_factor': 0.5,
    'random_state': GLOBAL_RANDOM_STATE
}
