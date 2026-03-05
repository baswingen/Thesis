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

# Random Forest Regressor Configuration
RF_CONFIG = {
    'n_estimators': 50,
    'max_depth': 10,
    'min_samples_split': 2,
    'random_state': 42
}

# Gradient Boosting Regressor Configuration
GB_CONFIG = {
    'n_estimators': 200,
    'learning_rate': 0.05,
    'max_depth': 4,
    'random_state': 42
}
# MLP (Multi-Layer Perceptron) Configuration
MLP_CONFIG = {
    'hidden_layers': [256, 128, 64],
    'dropout_rate': 0.1,
    'learning_rate': 0.001,
    'batch_size': 32,
    'epochs': 100,
    'random_state': 42
}
