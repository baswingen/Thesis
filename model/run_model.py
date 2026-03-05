import os
import json
import sys
from pathlib import Path
from datetime import datetime
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, accuracy_score, confusion_matrix

# Add project root to sys.path so 'model' package can be found
sys.path.append(str(Path(__file__).parent.parent))

from model.data_loader import DataLoader
from model.svm import SVMClassifier
from model.rbfnn import RBFNNClassifier
from model.svr import SVRRegressor
from model.config_model import SVM_CONFIG, RBFNN_CONFIG, SVR_CONFIG

from sklearn.metrics import (
    classification_report, accuracy_score, confusion_matrix,
    mean_absolute_error, mean_squared_error, r2_score
)

###########################################################
# CONFIGURATION
###########################################################
# Choose model to train: "svm", "rbfnn", or "svr"
MODEL_TYPE = "svr" 
###########################################################

def main():
    # Define paths
    base_dir = Path(__file__).parent.parent
    segments_dir = base_dir / "database" / "segments"
    results_dir = base_dir / "model_results"
    
    # Create results directory if it doesn't exist
    results_dir.mkdir(parents=True, exist_ok=True)
    
    # Create a unique timestamped folder for this run's results
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = results_dir / f"run_{timestamp}"
    run_dir.mkdir(exist_ok=True)
    
    print(f"Results will be saved to: {run_dir}")
    
    # Get all h5 segments files
    h5_paths = list(segments_dir.glob("*.h5"))
    
    if not h5_paths:
        print(f"No HDF5 segment files found in {segments_dir}.")
        return

    print(f"Found {len(h5_paths)} segment file(s).")
    
    # Load and extract features
    loader = DataLoader()
    print("Extracting features from HDF5 files. This may take a moment...")
    df = loader.load_and_extract_features(h5_paths)
    
    if df.empty:
        print("Data extraction failed or produced an empty DataFrame.")
        return
        
    print(f"Extracted features dataframe shape: {df.shape}")
    
    # Prepare data for ML
    model_type = MODEL_TYPE.lower()
    is_regression = (model_type == "svr")
    target_col = "weight" if is_regression else "label"
    
    X, y = loader.prepare_for_ml(df, target_col=target_col)
    print(f"Feature matrix (X) shape: {X.shape}")
    print(f"Label vector (y) shape: {y.shape} (Target: {target_col})")
    
    if not is_regression and len(y.unique()) < 2:
        print("\nLess than 2 classes found in dataset. Ensure your target labels are correct and varied.")
        return

    # Train/Test Split
    stratify = y if not is_regression else None
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42, stratify=stratify
    )
    
    print(f"\nTraining on {len(X_train)} samples, testing on {len(X_test)} samples.")
    
    # Initialize and train the selected Model
    if model_type == "svm":
        print(f"Training SVM Classifier with config: {SVM_CONFIG}")
        model = SVMClassifier(**SVM_CONFIG)
    elif model_type == "rbfnn":
        print(f"Training RBF Neural Network Classifier with config: {RBFNN_CONFIG}")
        model = RBFNNClassifier(**RBFNN_CONFIG)
    elif model_type == "svr":
        print(f"Training SVR Regressor with config: {SVR_CONFIG}")
        model = SVRRegressor(**SVR_CONFIG)
    else:
        print(f"Unknown model type: {model_type}")
        return

    model.fit(X_train, y_train)
    
    # Evaluate model
    print(f"\nEvaluating {model_type.upper()} Model on unseen test set...")
    
    if is_regression:
        metrics, report_str = model.evaluate(X_test, y_test)
        print("=" * 55)
        print("Regression Metrics:")
        print(report_str)
        print("=" * 55)
    else:
        y_pred = model.predict(X_test)
        accuracy = accuracy_score(y_test, y_pred)
        report_str = classification_report(y_test, y_pred, zero_division=0)
        conf_matrix = confusion_matrix(y_test, y_pred)
        labels = sorted(y.unique())
        
        print("=" * 55)
        print(f"Model Accuracy: {accuracy:.4f}")
        print("Classification Report:")
        print(report_str)
        print("=" * 55)
    
    # Save the Model structure
    model_path = run_dir / f"{model_type}_model.joblib"
    model.save(model_path)
    
    # Save detailed performance report
    report_file = run_dir / "performance_report.txt"
    with open(report_file, "w") as f:
        f.write("=" * 55 + "\n")
        f.write(f"MODEL PERFORMANCE REPORT\n")
        f.write(f"Run Timestamp: {timestamp}\n")
        f.write("=" * 55 + "\n\n")
        
        f.write("--- DATASET INFO ---\n")
        f.write(f"Database segments used: {[p.name for p in h5_paths]}\n")
        f.write(f"Total samples: {len(X)}\n")
        f.write(f"Training samples: {len(X_train)}\n")
        f.write(f"Testing samples: {len(X_test)}\n\n")
        
        f.write("--- HYPERPARAMETERS ---\n")
        f.write(f"Model: {model_type.upper()}\n")
        if model_type == "svm":
            f.write(f"Kernel: {model.kernel}\n")
            f.write(f"C: {model.C}\n")
            f.write(f"Random State: {model.random_state}\n\n")
        elif model_type == "rbfnn":
            f.write(f"Centers: {model.n_centers}\n")
            f.write(f"Gamma: {model.gamma}\n")
            f.write(f"C: {model.C}\n")
            f.write(f"Random State: {model.random_state}\n\n")
        elif model_type == "svr":
            f.write(f"Kernel: {model.kernel}\n")
            f.write(f"C: {model.C}\n")
            f.write(f"Epsilon: {model.epsilon}\n")
            f.write(f"Random State: {model.random_state}\n\n")
        
        f.write("--- EVALUATION METRICS ---\n")
        if is_regression:
            f.write(report_str)
        else:
            f.write(f"Accuracy: {accuracy:.4f}\n\n")
            f.write("Classification Report:\n")
            f.write(report_str)
            f.write("\n\nConfusion Matrix:\n")
            f.write(f"Labels order: {labels}\n")
            f.write(str(conf_matrix))
            f.write("\n")
        
    print(f"\nPerformance report saved to {report_file}")

if __name__ == "__main__":
    main()
