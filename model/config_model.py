"""
Configuration file for model hyperparameters.
Centralizing these values makes it easier to manage experiments and 
ensures consistency across training and inference scripts.
"""

GLOBAL_RANDOM_STATE = 14
# Cross-Validation Configuration
CV_CONFIG = {
    'use_cross_val': True,
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
    'kernel': 'rbf',
    'C': 1000.0,
    'epsilon': 0.2,
    'gamma': 0.001,
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
    'random_state': GLOBAL_RANDOM_STATE
}

# GRU (Gated Recurrent Unit) Configuration
GRU_CONFIG = {
    'hidden_size': 128,
    'num_layers': 2,
    'dropout_rate': 0.2,
    'learning_rate': 0.001,
    'batch_size': 32,
    'epochs': 100,
    'window_size_sec': 0.25,
    'window_step_sec': 0.1,
    'random_state': GLOBAL_RANDOM_STATE
}

# CNN-LSTM Configuration
CNN_LSTM_CONFIG = {
    'cnn_filters': 64,
    'cnn_kernel_size': 3,
    'lstm_hidden_size': 128,
    'lstm_num_layers': 2,
    'dropout_rate': 0.2,
    'learning_rate': 0.001,
    'batch_size': 32,
    'epochs': 100,
    'window_size_sec': 0.25,
    'window_step_sec': 0.1,
    'random_state': GLOBAL_RANDOM_STATE
}
