import os
from pathlib import Path

# Detect if we are on the DelftBlue server
ON_DELFTBLUE = os.getenv("SLURM_JOB_ID") is not None

GLOBAL_RANDOM_STATE = 245
GLOBAL_LOSS_FUNCTION = 'mse'  # Options: 'mse', 'mae', or 'huber'
GLOBAL_BALANCE_WEIGHTS = False

# ──────────────────────────────────────────────────────────
# RUN_MODEL PIPELINE TOGGLES
# ──────────────────────────────────────────────────────────
MODEL_TYPE = "spatio_temporal_transformer5"  # Options: "svr", "rf", "gb", "mlp", "gru", "lstm", "cnn_lstm", "cnn_gru", "cnn_bilstm_attention", "tcn", "transformer", "spatio_temporal_transformer", "spatio_temporal_transformer2", "spatio_temporal_transformer3", "spatio_temporal_transformer4", "spatio_temporal_transformer5", "st_transformer_aksan", "cnn_st_transformer"
RUN_GRID_SEARCH = False
GRID_SEARCH_MODE = "model_arch"  # Options: "model_arch", "generalization", "features" (for spatio_temporal_transformer3)
USE_PRECOMPUTED_FEATURES = True

DEV_MODE = False
DEV_FRACTION = 0.20
DEV_CV_FOLDS = 8
DEV_EPOCHS = 25
DEV_EARLY_STOPPING_PATIENCE = 5

# Enables or disables the computation of feature importance (DeepSHAP & Permutation).
# These computations can be very slow, so disabling them is useful for rapid prototyping.
COMPUTE_FEATURE_IMPORTANCE = True
COMPUTE_PERMUTATION_CHANNEL = True
COMPUTE_PERMUTATION_FEATURE = True
COMPUTE_PERMUTATION_INDIVIDUAL = True
COMPUTE_DEEPSHAP = True
RUN_MODALITY_ABLATION = True


# ──────────────────────────────────────────────────────────
# DATABASE & ENVIRONMENT CONFIGURATION
# ──────────────────────────────────────────────────────────

# Define the root of the database depending on the environment.
# Local: project-relative 'database/'
# DelftBlue: high-performance '/scratch/bwingen/thesis/database/'
if ON_DELFTBLUE:
    DATABASE_ROOT = Path("/scratch/bwingen/thesis/database")
else:
    DATABASE_ROOT = Path("/Volumes/Laurens SSD/BasData")

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
    'include': ['P01', 'P02', 'P03', 'P04', 'P05', 'P06', 'P07', 'P08', 'P09', 'P10', 'P11', 'P12', 'P14', 'P15', 'P16', 'P17', 'P18'],  # Options: 'all' or a list of IDs (e.g., ['P01', 'P02'])
}

###########################################################
# WEIGHT CALIBRATION CONFIGURATION
###########################################################
# Maps nominal weight labels to precise measured weights in real life.
# If a weight is not found in this list, the fallback is the original label.
TRUE_WEIGHTS = {
    0.75: 0.90,
    1.0: 0.98,
    2.0: 1.97,
    2.25: 2.24,
    3.0: 2.95,
    4.25: 4.15,
    6.0: 5.93,
}

# Toggle individual weight classes on/off.
# Set a weight to False to exclude ALL segments with that nominal weight
# from training.  The 0.0 key controls free_movement segments.
WEIGHT_INCLUDE = {
    0.0:  True,   # free_movement (no load)
    0.75: False,
    1.0:  True,
    2.0:  True,
    2.25: False,
    3.0:  True,
    4.25: True,
    6.0:  True,
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
        'ax_diff': False, 'ay_diff': False, 'az_diff': False,
        'roll_rad_diff': False, 'pitch_rad_diff': False, 'yaw_rad_diff': False,
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
    'window_step_sec': 0.10,

    # Channel toggles have been moved to the top-level CHANNEL_CONFIG dict.


    # ── EMG Features (set False to disable) ──────────────
    # Optimized for cross-subject generalization capacity over raw size.
    # Kept: 11 highly generalizable features (e.g. LogDet, WL, RMS, HjMob).
    # Disabled: 9 volatile/redundant features (e.g. Kurtosis, Skewness, SpecEntropy, MAV).
    'emg_features': {
        # Time-domain
        'MAV':        False,   # DISABLED — negative contribution (-0.00371), redundant with RMS/IEMG
        'RMS':        True,    # Root Mean Square (+0.00740)
        'WL':         True,    # Waveform Length (+0.00974)
        'ZC':         True,    # Zero Crossings (+0.00438)
        'SSC':        True,    # Slope Sign Changes (+0.00232)
        'VAR':        False,   # DISABLED — negative contribution (-0.01334), highly redundant with RMS
        'WAMP':       True,    # Willison Amplitude (+0.00179)
        'IEMG':       False,   # DISABLED — negative contribution (-0.00352)
        'LogDet':     True,    # Log Detector (+0.01598) — excellent skin-impedance normalizer
        'Skew':       False,   # DISABLED — negative contribution (-0.01149), volatile 3rd moment
        'Kurt':       False,   # DISABLED — negative contribution (-0.01696), volatile 4th moment
        'HjMob':      True,    # Hjorth Mobility (+0.01274) — clean time-domain frequency proxy
        'HjComp':     False,   # DISABLED — negative contribution (-0.01047), high-variance bandwidth estimator
        'Myopulse':   True,    # Myopulse Percentage Rate (+0.00283)
        # Frequency-domain
        'MNF':        True,    # Mean Frequency (+0.00508)
        'MDF':        False,   # DISABLED — negative contribution (-0.00425)
        'Power':      True,    # Total Spectral Power (+0.00551)
        'SpecEntropy':False,   # DISABLED — negative contribution (-0.01440), high cross-subject variance
        'PeakFreq':   False,   # DISABLED — negative contribution (-0.00326)
        'BW':         True,    # Bandwidth (95%) (+0.00563)
    },

    # ── IMU Features (set False to disable) ──────────────
    # Kept: 10 solid physical kinematics estimators (e.g. Std, Var, Mean, P2P, Jerk).
    # Disabled: 3 noisy frequency/magnitude cross-channel metrics (SpecEnergy, SVM_Mean, MNF).
    'imu_features': {
        # Time-domain (kept: 10)
        'Mean':       True,    # Mean value (+0.01379)
        'Var':        True,    # Variance (+0.01571)
        'Std':        True,    # Standard Deviation (+0.01619) — absolute best generalizer
        'Max':        True,    # Maximum (+0.00262)
        'Min':        False,   # DISABLED — redundant with Max via P2P
        'RMS':        False,   # DISABLED — ≈Std for near-zero-mean signals
        'SMA':        True,    # Signal Magnitude Area (+0.00643)
        'P2P':        True,    # Peak-to-Peak (+0.01239)
        'IQR':        False,   # DISABLED — highly correlated with Std
        'Skew':       True,    # Skewness (+0.00539)
        'Kurt':       True,    # Kurtosis (+0.00634)
        'Jerk':       True,    # Mean Absolute Jerk (+0.01139)
        'ZC':         False,   # DISABLED — low value for load estimation
        'Energy':     False,   # DISABLED — ≈Var*N (redundant)
        # Frequency-domain (kept: 1)
        'DomFreq':    True,    # Dominant Frequency (+0.01096)
        'SpecEnergy': False,   # DISABLED — negative contribution (-0.01337)
        'MNF':        False,   # DISABLED — negative contribution (-0.00363)
        'MDF':        False,   # DISABLED — highly correlated with MNF
        'SpecEntropy':False,   # DISABLED — low discrimination for rigid-body
        # Cross-channel (kept: 0)
        'SVM_Mean':   False,   # DISABLED — negative contribution (-0.00157)
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
    # Optimized to 0.5 based on Spatio-Temporal Transformer 3 Sweep Rank 1 (Iter 92)
    'p': 0.5,

    # Active augmentation methods.  Remove a method name to disable it.
    'methods': ['noise', 'stretch', 'channel_dropout', 'magnitude_scale', 'mixup'],

    # ── Gaussian noise ────────────────────────────────────────────────
    # Standard deviation on the z-score scale (after StandardScaler).
    # Optimized to 0.05 based on Spatio-Temporal Transformer 3 Sweep Rank 1 (Iter 92)
    'noise_std': 0.05,

    # ── Temporal stretch ─────────────────────────────────────────────
    # Random scale factor applied to sequence length, then resampled back.
    # Optimized to (0.75, 1.25) to cover more diverse stretch ranges
    'stretch_factor_range': (0.75, 1.25),

    # ── Channel dropout ───────────────────────────────────────────────
    # Probability that an entire channel (feature) is zeroed for the full window.
    # Optimized to 0.25 as requested
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

    # ── Participant Balancing ────────────────────────────────────────
    # When 'balance_participants' is True, the augmenter will ensure every 
    # participant has exactly 'target_samples_per_participant' in the training set.
    # If they have more, we downsample; if less, we use augmentation to fill.
    'balance_participants': True,
    'target_samples_per_participant': 1500,

    # ── Weight Balancing ─────────────────────────────────────────────
    # When 'balance_weights' is True, the augmenter will ensure every
    # weight class has exactly 'target_samples_per_weight' in the training set.
    # This prevents the majority 0kg class from dominating and oversamples heavier weights.
    'balance_weights': True,
    'target_samples_per_weight': 2000,
    
    # ── Joint Balancing (Participants + Weights) ─────────────────────
    # If BOTH toggles above are True, the augmenter balances every unique 
    # combination of (participant, weight class).
    'target_samples_per_group': 250,  # e.g. 18 participants * 6 weights * 250 = 27,000 total samples
}

# Cross-Validation Configuration
CV_CONFIG = {
    'use_cross_val': True,
    'train_test_split': 0.2,    # Only used if use_cross_val is False
    'n_folds': 17,              # 17-Fold LOPO (Leave-One-Participant-Out) since we have 17 active subjects
    'strategy': 'participant',  # Options: 'kfold', 'participant'
    'strict_val_split': True,   # Toggles strict cross-participant early stopping
    'strict_val_participants': 2 # Hold out 2 participants for early stopping validation to prevent overfitting
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
    'use_anthropometrics': True,    # Feed participant body measurements as conditioning
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
    'use_anthropometrics': True,    # Feed participant body measurements as conditioning
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

# Spatio-Temporal Transformer Configuration
# Validated via 3 HP sweeps: 2× at 10% (sweep_20260505_183441, _185415) + 1× at 20% (_222742)
# 20% winner: R²=0.929, RMSE=0.514, MAE=0.319 (d_model=160 emerged with more data)
SPATIO_TEMPORAL_TRANSFORMER_CONFIG = {
    'd_model': 128,               
    'nhead_spatial': 8,           
    'num_layers_spatial': 3,      
    'nhead_temporal': 2,          
    'num_layers_temporal': 3,     
    'dim_feedforward': 1024,       
    'dropout_rate': 0.5,         
    'learning_rate': 0.0005,      
    'weight_decay': 0.005,        
    'batch_size': 64,            
    'epochs': 100,                
    'validation_split': 0.1,
    'early_stopping_patience': 15, 
    'scheduler_patience': 5,       
    'scheduler_factor': 0.8,
    'use_checkpointing': True,    
    'use_amp': True,              
    'scheduler': {
        'type': 'ReduceLROnPlateau'
    },
    'random_state': GLOBAL_RANDOM_STATE
}

# ST-Transformer (Aksan et al.) Configuration
ST_TRANSFORMER_AKSAN_CONFIG = {
    'embed_dim': 128,
    'num_layers': 4,
    'num_heads': 4,
    'feedforward_dim': 256,
    'dropout': 0.1,
    'learning_rate': 0.0003,
    'weight_decay': 1e-4,
    'batch_size': 128,
    'epochs': 200,                
    'validation_split': 0.1,
    'early_stopping_patience': 15, 
    'scheduler_patience': 7,       
    'scheduler_factor': 0.5,
    'use_amp': True,              
    'random_state': GLOBAL_RANDOM_STATE
}

# Modality-Grouped Spatio-Temporal Transformer (Transformer3) Configuration
# Set to the optimal "Sweet Spot" Generalization baseline (Rank 2, Iteration 61)
# Highly robust LOPO performance with a constrained 1.0M parameter footprint.
SPATIO_TEMPORAL_TRANSFORMER3_CONFIG = {
    'd_model': 96,                # Swept optimal Sweet Spot for cross-subject generalization
    'nhead_spatial': 8,           # 8 spatial heads for detailed muscle activation mapping
    'num_layers': 4,              # Combined spatio-temporal layers (optimized to 4)
    'nhead_temporal': 2,          # 2 temporal heads for clean velocity/jerk tracking
    'dim_feedforward': 512,       # Swept optimal FF dimension (optimized to 512)
    'dropout_rate': 0.25,         # "Sweet spot" dropout rate to prevent overfitting choke
    'learning_rate': 0.0003,      # Optimized stable learning rate
    'weight_decay': 0.005,        # Strong WD for clean regularization without capacity choke
    'batch_size': 64,
    'epochs': 200,                # Full epochs budget
    'validation_split': 0.1,
    'early_stopping_patience': 5, # Generous early stopping patience
    'scheduler_patience': 3,       # Learning rate scheduler patience
    'scheduler_factor': 0.8,       # Smooth learning rate scheduler decay factor
    'use_checkpointing': True,
    'use_amp': True,
    'scheduler': {
        'type': 'ReduceLROnPlateau'
    },
    'random_state': GLOBAL_RANDOM_STATE
}

# Modality-Isolated Spatio-Temporal Transformer (Transformer4) Configuration
# Fully robust LOPO performance with isolated spatial attention and late fusion.
SPATIO_TEMPORAL_TRANSFORMER4_CONFIG = {
    'd_model': 96,                # Swept optimal Sweet Spot for cross-subject generalization
    'nhead_spatial': 8,           # 8 spatial heads for detailed muscle activation mapping
    'num_layers': 4,              # Combined spatio-temporal layers (optimized to 4)
    'nhead_temporal': 2,          # 2 temporal heads for clean velocity/jerk tracking
    'dim_feedforward': 512,       # Swept optimal FF dimension (optimized to 512)
    'dropout_rate': 0.25,         # "Sweet spot" dropout rate to prevent overfitting choke
    'learning_rate': 0.0003,      # Optimized stable learning rate
    'weight_decay': 0.005,        # Strong WD for clean regularization without capacity choke
    'batch_size': 64,
    'epochs': 200,                # Full epochs budget
    'validation_split': 0.1,
    'early_stopping_patience': 15, # Generous early stopping patience
    'scheduler_patience': 5,       # Learning rate scheduler patience
    'scheduler_factor': 0.5,       # Smooth learning rate scheduler decay factor
    'use_checkpointing': True,
    'use_amp': True,
    'scheduler': {
        'type': 'ReduceLROnPlateau'
    },
    'random_state': GLOBAL_RANDOM_STATE
}

# Modality-Isolated SST5 Spatio-Temporal Transformer (SST5) Configuration
# Features isolated spatial attention in deep core and standard spatial cross-modality lookup at the end.
SPATIO_TEMPORAL_TRANSFORMER5_CONFIG = {
    'd_model': 96,                # Swept optimal Sweet Spot for cross-subject generalization
    'nhead_spatial': 8,           # 8 spatial heads for detailed muscle activation mapping
    'num_layers': 4,              # Combined layers (3 isolated blocks + 1 spatial cross-modality block)
    'nhead_temporal': 2,          # 2 temporal heads for clean velocity/jerk tracking
    'dim_feedforward': 512,       # Swept optimal FF dimension (optimized to 512)
    'dropout_rate': 0.25,         # "Sweet spot" dropout rate to prevent overfitting choke
    'learning_rate': 0.0003,      # Optimized stable learning rate
    'weight_decay': 0.005,        # Strong WD for clean regularization without capacity choke
    'batch_size': 64,
    'epochs': 200,                # Full epochs budget
    'validation_split': 0.1,
    'early_stopping_patience': 15, # Generous early stopping patience
    'scheduler_patience': 5,       # Learning rate scheduler patience
    'scheduler_factor': 0.5,       # Smooth learning rate scheduler decay factor
    'use_checkpointing': True,
    'use_amp': True,
    'scheduler': {
        'type': 'ReduceLROnPlateau'
    },
    'random_state': GLOBAL_RANDOM_STATE
}

# CNN-ST-Transformer Configuration
CNN_ST_TRANSFORMER_CONFIG = {
    'cnn_filters': [32, 64, 128],    # MUST end with d_model (128)
    'cnn_kernel_sizes': [8, 5, 3],
    'pool_size': 2,
    'd_model': 128,
    'nhead_spatial': 4,
    'num_layers_spatial': 2,
    'nhead_temporal': 4,
    'num_layers_temporal': 2,
    'dim_feedforward': 256,
    'dropout_rate': 0.1,
    'learning_rate': 0.0003,
    'weight_decay': 1e-4,
    'batch_size': 64,  # Smaller batch size for raw sequences
    'epochs': 200,                
    'validation_split': 0.1,
    'early_stopping_patience': 15, 
    'scheduler_patience': 7,       
    'scheduler_factor': 0.5,
    'use_amp': True,              
    'random_state': GLOBAL_RANDOM_STATE
}

# ──────────────────────────────────────────────────────────
# DEV MODE OVERRIDES
# ──────────────────────────────────────────────────────────
if DEV_MODE:
    print(f"\n[CONFIG] DEV_MODE is ACTIVE! Subsampling {DEV_FRACTION*100}% of data for rapid iteration.")
    CV_CONFIG['strict_val_split'] = True
    
    # Scale augmentation target for rapid iteration
    orig_target_p = AUGMENTATION_CONFIG.get('target_samples_per_participant', 1500)
    AUGMENTATION_CONFIG['target_samples_per_participant'] = max(10, int(orig_target_p * DEV_FRACTION))
    
    orig_target_w = AUGMENTATION_CONFIG.get('target_samples_per_weight', 2000)
    AUGMENTATION_CONFIG['target_samples_per_weight'] = max(10, int(orig_target_w * DEV_FRACTION))
    
    orig_target_g = AUGMENTATION_CONFIG.get('target_samples_per_group', 250)
    AUGMENTATION_CONFIG['target_samples_per_group'] = max(5, int(orig_target_g * DEV_FRACTION))
    print(f"[CONFIG] Scaled augmentation targets - Participant: {AUGMENTATION_CONFIG['target_samples_per_participant']}, Weight: {AUGMENTATION_CONFIG['target_samples_per_weight']}, Group: {AUGMENTATION_CONFIG['target_samples_per_group']}")
    
    # Cap epochs and patience for fast deep learning runs
    for cfg in [MLP_CONFIG, GRU_CONFIG, LSTM_CONFIG, CNN_LSTM_CONFIG, CNN_GRU_CONFIG, CNN_BILSTM_ATTENTION_CONFIG, CNN_LSTM_ABLATION_CONFIG, TCN_CONFIG, TRANSFORMER_CONFIG, SPATIO_TEMPORAL_TRANSFORMER_CONFIG, SPATIO_TEMPORAL_TRANSFORMER3_CONFIG, SPATIO_TEMPORAL_TRANSFORMER4_CONFIG, SPATIO_TEMPORAL_TRANSFORMER5_CONFIG, ST_TRANSFORMER_AKSAN_CONFIG, CNN_ST_TRANSFORMER_CONFIG]:
        if 'epochs' in cfg:
            cfg['epochs'] = min(cfg['epochs'], DEV_EPOCHS)
        if 'early_stopping_patience' in cfg:
            cfg['early_stopping_patience'] = min(cfg['early_stopping_patience'], DEV_EARLY_STOPPING_PATIENCE)
            
    # Downscale architecture dimensions to prevent massive overfitting on the small subset
    LSTM_CONFIG.update({
        'hidden_size': 256,
        'num_layers': 2,
    })
    
    GRU_CONFIG.update({
        'hidden_size': 128,
        'num_layers': 2,
    })
    
    TRANSFORMER_CONFIG.update({
        'd_model': 16,
        'nhead': 2,
        'num_layers': 1,
        'dim_feedforward': 32,
    })
    
    SPATIO_TEMPORAL_TRANSFORMER_CONFIG.update({
        'd_model': 128,               
        'nhead_spatial': 8,
        'num_layers_spatial': 3,
        'nhead_temporal': 2,
        'num_layers_temporal': 3,
        'dim_feedforward': 1024,
        'dropout_rate': 0.5,
        'learning_rate': 0.0005,
        'weight_decay': 0.005,
        'batch_size': 64,
        'use_checkpointing': False,   # Disable for dev mode speed
        'use_amp': True,
    })
    
    SPATIO_TEMPORAL_TRANSFORMER3_CONFIG.update({
        'd_model': 96,
        'nhead_spatial': 8,
        'num_layers': 3,
        'nhead_temporal': 2,
        'dim_feedforward': 256,
        'dropout_rate': 0.5,
        'learning_rate': 0.0005,
        'weight_decay': 0.005,
        'batch_size': 64,
        'use_checkpointing': False,
        'use_amp': True,
    })
    
    SPATIO_TEMPORAL_TRANSFORMER4_CONFIG.update({
        'd_model': 96,
        'nhead_spatial': 8,
        'num_layers': 3,
        'nhead_temporal': 2,
        'dim_feedforward': 256,
        'dropout_rate': 0.5,
        'learning_rate': 0.0005,
        'weight_decay': 0.005,
        'batch_size': 64,
        'use_checkpointing': False,
        'use_amp': True,
    })
    
    SPATIO_TEMPORAL_TRANSFORMER5_CONFIG.update({
        'd_model': 96,
        'nhead_spatial': 8,
        'num_layers': 3,
        'nhead_temporal': 2,
        'dim_feedforward': 256,
        'dropout_rate': 0.5,
        'learning_rate': 0.0005,
        'weight_decay': 0.005,
        'batch_size': 64,
        'use_checkpointing': False,
        'use_amp': True,
    })
    
    ST_TRANSFORMER_AKSAN_CONFIG.update({
        'embed_dim': 48,              # Matched to ~927K params (fair comparison with T2=935K, T3=933K)
        'num_layers': 3,
        'num_heads': 4,               # 48/4 = 12 per head
        'feedforward_dim': 256,
        'dropout': 0.25,
        'learning_rate': 0.0003,
        'weight_decay': 1e-5,
        'batch_size': 128,
    })
    
    CNN_ST_TRANSFORMER_CONFIG.update({
        'cnn_filters': [8, 16],
        'cnn_kernel_sizes': [5, 3],
        'pool_size': 8,           # Large pool size to aggressively shrink the time dimension
        'd_model': 16,
        'nhead_spatial': 2,
        'num_layers_spatial': 1,
        'nhead_temporal': 2,
        'num_layers_temporal': 1,
        'dim_feedforward': 32,
        'batch_size': 32,
    })
    
    CNN_LSTM_CONFIG.update({
        'cnn_filters': [16, 32, 64],
        'cnn_kernel_sizes': [16, 5, 3],
        'lstm_hidden_size': 64,
        'lstm_num_layers': 1,
    })
    
    CNN_GRU_CONFIG.update({
        'cnn_filters': [16, 32, 64],
        'cnn_kernel_sizes': [16, 5, 3],
        'gru_hidden_size': 64,
        'gru_num_layers': 1,
    })
    
    CNN_BILSTM_ATTENTION_CONFIG.update({
        'cnn_filters': [16, 32, 64],
        'cnn_kernel_sizes': [16, 5, 3],
        'lstm_hidden_size': 32,
        'lstm_num_layers': 1,
        'fc_units': 64,
    })
    
    TCN_CONFIG.update({
        'cnn_filters': [16, 32, 64],
        'cnn_kernel_sizes': [7, 5, 3],
        'tcn_channels': [32, 64],
    })
