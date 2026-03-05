"""
Configuration file for model hyperparameters.
Centralizing these values makes it easier to manage experiments and 
ensures consistency across training and inference scripts.
"""

# SVM Configuration
SVM_CONFIG = {
    'kernel': 'sigmoid', # Optimized via sweep
    'C': 10.0,           # Optimized via sweep
    'gamma': 0.001,      # Optimized via sweep
    'class_weight': 'balanced',
    'random_state': 42
}

# RBF Neural Network Configuration
RBFNN_CONFIG = {
    'n_centers': 300,    # Optimized via sweep
    'gamma': 0.001,      # Optimized via sweep
    'C': 10.0,           # Optimized via sweep
    'random_state': 42,
    'class_weight': 'balanced'
}

# SVR (Support Vector Regression) Configuration
SVR_CONFIG = {
    'kernel': 'rbf',
    'C': 1000.0,
    'epsilon': 0.2,
    'gamma': 0.001,
    'random_state': 42
}
