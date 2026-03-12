"""
Configuration file for model hyperparameters.
Centralizing these values makes it easier to manage experiments and 
ensures consistency across training and inference scripts.
"""

GLOBAL_RANDOM_STATE = 11
GLOBAL_LOSS_FUNCTION = 'mae'  # Options: 'mse' or 'mae'

###########################################################
# FEATURE EXTRACTION CONFIGURATION
###########################################################
FEATURE_CONFIG = {
    # Sampling frequencies
    'emg_fs': 2000,
    'imu_fs': 500,

    # EMG threshold for WAMP & Myopulse features
    'emg_threshold': 1e-5,

    # Sliding window properties (for sequence models)
    'window_size_sec': 0.25,
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
        'MNF':        False,   # Mean Frequency
        'MDF':        False,   # Median Frequency
        'Power':      True,    # Total Spectral Power
        'SpecEntropy':False,   # Spectral Entropy
        'PeakFreq':   False,   # Peak Frequency
        'BW':         False,   # Bandwidth (95%)
    },

    # ── IMU Features (set False to disable) ──────────────
    'imu_features': {
        # Time-domain
        'Mean':       True,    # Mean value
        'Var':        False,   # Variance
        'Std':        False,   # Standard Deviation
        'Max':        True,    # Maximum
        'Min':        True,    # Minimum
        'RMS':        False,   # Root Mean Square
        'SMA':        True,    # Signal Magnitude Area
        'P2P':        False,   # Peak-to-Peak
        'IQR':        False,   # Interquartile Range
        'Skew':       False,   # Skewness
        'Kurt':       True,    # Kurtosis
        'Jerk':       False,   # Mean Absolute Jerk
        'ZC':         False,   # Zero Crossings
        'Energy':     False,   # Energy
        # Frequency-domain
        'DomFreq':    False,   # Dominant Frequency
        'SpecEnergy': False,   # Spectral Energy
        'MNF':        False,   # Mean Frequency
        'MDF':        False,   # Median Frequency
        'SpecEntropy':False,   # Spectral Entropy
        # Cross-channel
        'SVM_Mean':   False,   # Signal Vector Magnitude (mean)
        'SVM_Std':    False,   # Signal Vector Magnitude (std)
    },
}

# Cross-Validation Configuration
CV_CONFIG = {
    'use_cross_val': True,
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
    'epochs': 100,
    'validation_split': 0.1,
    'early_stopping_patience': 10,
    'random_state': GLOBAL_RANDOM_STATE
}

# LSTM (Long Short-Term Memory) Configuration
LSTM_CONFIG = {
    'hidden_size': 128,         
    'num_layers': 2,           
    'dropout_rate': 0.4,       
    'learning_rate': 0.005,
    'weight_decay': 1e-4,
    'batch_size': 128,          
    'epochs': 150,             
    'validation_split': 0.1,
    'early_stopping_patience': 50,
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
    'd_model': 32,             
    'nhead': 4,                
    'num_layers': 2,           
    'dim_feedforward': 256,    
    'dropout_rate': 0.3,       
    'learning_rate': 0.005,
    'weight_decay': 1e-4,
    'batch_size': 64,          
    'epochs': 100,             
    'validation_split': 0.2,
    'early_stopping_patience': 10,
    'random_state': GLOBAL_RANDOM_STATE
}
