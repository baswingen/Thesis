"""
Configuration file for model hyperparameters.
Centralizing these values makes it easier to manage experiments and 
ensures consistency across training and inference scripts.
"""

GLOBAL_RANDOM_STATE = 42
# Cross-Validation Configuration
CV_CONFIG = {
    'use_cross_val': False,
    'n_folds': 5
}

# SVM Configuration
SVM_CONFIG = {
    'kernel': 'sigmoid', # Optimized via sweep
    'C': 10.0,           # Optimized via sweep
    'gamma': 0.001,      # Optimized via sweep
    'class_weight': 'balanced',
    'random_state': GLOBAL_RANDOM_STATE
}

# RBF Neural Network Configuration
RBFNN_CONFIG = {
    'n_centers': 300,    # Optimized via sweep
    'gamma': 0.001,      # Optimized via sweep
    'C': 10.0,           # Optimized via sweep
    'random_state': GLOBAL_RANDOM_STATE,
    'class_weight': 'balanced'
}

# SVR (Support Vector Regression) Configuration
SVR_CONFIG = {
    'kernel': 'linear',
    'C': 10.0,
    'epsilon': 0.1,
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
    'batch_size': 64,
    'epochs': 100,
    'validation_split': 0.2,
    'early_stopping_patience': 10,
    'window_size_sec': 0.25,
    'window_step_sec': 0.1,
    'random_state': GLOBAL_RANDOM_STATE
}

# LSTM (Long Short-Term Memory) Configuration
LSTM_CONFIG = {
    'hidden_size': 512,         
    'num_layers': 3,           
    'dropout_rate': 0.3,       
    'learning_rate': 0.0005,    
    'batch_size': 16,          
    'epochs': 150,             
    'validation_split': 0.2,
    'early_stopping_patience': 10,
    'window_size_sec': 0.25,
    'window_step_sec': 0.1,
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
    'batch_size': 32,
    'epochs': 100,
    'validation_split': 0.2,
    'early_stopping_patience': 10,
    'window_size_sec': 0.25,
    'window_step_sec': 0.1,
    'random_state': GLOBAL_RANDOM_STATE
}

# Transformer Configuration
TRANSFORMER_CONFIG = {
    'd_model': 32,             # Optimized via sweep
    'nhead': 4,                # Optimized via sweep
    'num_layers': 2,           # Optimized via sweep
    'dim_feedforward': 256,    # Optimized via sweep
    'dropout_rate': 0.3,       # Optimized via sweep
    'learning_rate': 0.005,    # Optimized via sweep
    'batch_size': 64,          # Optimized via sweep
    'epochs': 100,             # Optimized via sweep
    'validation_split': 0.2,
    'early_stopping_patience': 10,
    'window_size_sec': 0.25,   # Must match extraction window
    'window_step_sec': 0.1,
    'random_state': GLOBAL_RANDOM_STATE
}
