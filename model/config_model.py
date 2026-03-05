"""
Configuration file for model hyperparameters.
Centralizing these values makes it easier to manage experiments and 
ensures consistency across training and inference scripts.
"""

# SVM Configuration
SVM_CONFIG = {
    'kernel': 'rbf',
    'C': 1.0,
    'random_state': 42
}

# RBF Neural Network Configuration
RBFNN_CONFIG = {
    'n_centers': 50,
    'gamma': 1.0,
    'C': 1.0,
    'random_state': 42
}
