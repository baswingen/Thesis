"""
Configuration file for model hyperparameters.
Centralizing these values makes it easier to manage experiments and 
ensures consistency across training and inference scripts.
"""

GLOBAL_RANDOM_STATE = 18
GLOBAL_LOSS_FUNCTION = 'mse'  # Options: 'mse' or 'mae'

###########################################################
# EMG CHANNEL CONFIGURATION
###########################################################
EMG_CHANNEL_CONFIG = {
    # Double-channel muscles (differential pair)
    "Anterior Deltoid":  ("ch1", "ch2"),
    "Lateral Deltoid":   ("ch3", "ch4"),
    "Posterior Deltoid": ("ch5", "ch6"),
    "Triceps Brachii":   ("ch7", "ch8"),
    # Single-channel muscles
    "Biceps Brachii":    "ch17",
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
    'emg_window_size_sec': 0.15,
    'imu_window_size_sec': 0.3,
    'window_step_sec': 0.1,


    # ── EMG Features (set False to disable) ──────────────
    'emg_features': {
        # Time-domain
        'MAV':        True,    # Mean Absolute Value
        'RMS':        False,   # Root Mean Square
        'WL':         True,    # Waveform Length
        'ZC':         False,   # Zero Crossings
        'SSC':        True,    # Slope Sign Changes
        'VAR':        True,    # Variance
        'WAMP':       False,   # Willison Amplitude
        'IEMG':       True,    # Integrated EMG
        'LogDet':     False,   # Log Detector
        'Skew':       True,    # Skewness
        'Kurt':       True,    # Kurtosis
        'HjMob':      False,   # Hjorth Mobility
        'HjComp':     False,   # Hjorth Complexity
        'Myopulse':   False,   # Myopulse Percentage Rate
        # Frequency-domain
        'MNF':        True,    # Mean Frequency
        'MDF':        True,    # Median Frequency
        'Power':      False,   # Total Spectral Power
        'SpecEntropy':True,    # Spectral Entropy
        'PeakFreq':   True,    # Peak Frequency
        'BW':         True,    # Bandwidth (95%)
    },

    # ── IMU Features (set False to disable) ──────────────
    'imu_features': {
        # Time-domain
        'Mean':       False,   # Mean value
        'Var':        True,    # Variance
        'Std':        True,    # Standard Deviation
        'Max':        True,    # Maximum
        'Min':        False,   # Minimum
        'RMS':        False,   # Root Mean Square
        'SMA':        True,    # Signal Magnitude Area
        'P2P':        True,    # Peak-to-Peak
        'IQR':        False,   # Interquartile Range
        'Skew':       False,   # Skewness
        'Kurt':       True,    # Kurtosis
        'Jerk':       False,   # Mean Absolute Jerk
        'ZC':         False,   # Zero Crossings
        'Energy':     False,   # Energy
        # Frequency-domain
        'DomFreq':    True,    # Dominant Frequency
        'SpecEnergy': True,    # Spectral Energy
        'MNF':        True,    # Mean Frequency
        'MDF':        True,    # Median Frequency
        'SpecEntropy':False,   # Spectral Entropy
        # Cross-channel
        'SVM_Mean':   True,    # Signal Vector Magnitude (mean)
        'SVM_Std':    True,    # Signal Vector Magnitude (std)
    },
}

###########################################################
# DATA AUGMENTATION CONFIGURATION (LSTM / GRU only)
###########################################################
AUGMENTATION_CONFIG = {
    # Master toggle — set False to disable all augmentation
    'enabled': False,

    # Probability that any single training sample is augmented (0.0 – 1.0).
    # Each selected sample produces one additional augmented copy alongside
    # the original, so the dataset can grow up to 2× when p=1.0.
    'p': 0.5,

    # Active augmentation methods.  Remove a method name to disable it.
    # Available: 'noise', 'stretch', 'feature_dropout', 'magnitude_scale', 'mixup'
    'methods': ['noise', 'stretch', 'feature_dropout'],

    # ── Gaussian noise ────────────────────────────────────────────────
    # Standard deviation on the z-score scale (after StandardScaler).
    # 0.05 ≈ 5% of one standard deviation — very conservative.
    'noise_std': 0.05,

    # ── Temporal stretch ─────────────────────────────────────────────
    # Random scale factor applied to sequence length, then resampled back.
    # (0.85, 1.15) = ±15% speed variation.
    'stretch_factor_range': (0.85, 1.15),

    # ── Feature dropout ──────────────────────────────────────────────
    # Probability that a single feature value at a single time step is zeroed.
    'feature_dropout_p': 0.10,

    # ── Magnitude scaling ─────────────────────────────────────────────
    # Per-feature multiplicative factor drawn uniformly from this range.
    'magnitude_scale_range': (0.85, 1.15),

    # ── MixUp ────────────────────────────────────────────────────────
    # Alpha parameter of the Beta(α, α) distribution for λ.
    # Lower α → λ closer to 0 or 1 (less blending).
    'mixup_alpha': 0.2,
}

# Cross-Validation Configuration
CV_CONFIG = {
    'use_cross_val': False,
    'n_folds': 5
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
SVR_CONFIG = {
    'kernel': 'linear',
    'C': 0.1,
    'epsilon': 0.05,
    'gamma': 'scale',
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
GRU_CONFIG = {
    'hidden_size': 256,
    'num_layers': 3,
    'dropout_rate': 0.2,
    'learning_rate': 0.005,
    'weight_decay': 1e-4,
    'batch_size': 64,
    'epochs': 150,
    'validation_split': 0.1,
    'early_stopping_patience': 50,
    'scheduler_patience': 5,
    'scheduler_factor': 0.5,
    'random_state': GLOBAL_RANDOM_STATE
}

# LSTM (Long Short-Term Memory) Configuration
LSTM_CONFIG = {
    'hidden_size': 64,
    'num_layers': 2,
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

# CNN-LSTM Configuration
CNN_LSTM_CONFIG = {
    'cnn_filters': 32,
    'cnn_kernel_size': 5,
    'lstm_hidden_size': 128,
    'lstm_num_layers': 1,
    'dropout_rate': 0.3,
    'learning_rate': 0.005,
    'weight_decay': 1e-4,
    'batch_size': 32,
    'epochs': 100,
    'validation_split': 0.2,
    'early_stopping_patience': 10,
    'random_state': GLOBAL_RANDOM_STATE
}

# Transformer Configuration
TRANSFORMER_CONFIG = {
    'd_model': 64,
    'nhead': 4,
    'num_layers': 2,
    'dim_feedforward': 256,
    'dropout_rate': 0.2,
    'learning_rate': 0.001,
    'weight_decay': 1e-4,
    'batch_size': 64,
    'epochs': 150,
    'validation_split': 0.1,
    'early_stopping_patience': 20,
    'scheduler_patience': 5,
    'scheduler_factor': 0.5,
    'random_state': GLOBAL_RANDOM_STATE
}
