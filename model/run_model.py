import os
import json
import numpy as np
import sys
from pathlib import Path
from datetime import datetime
from sklearn.model_selection import train_test_split, StratifiedKFold
from sklearn.metrics import classification_report, accuracy_score, confusion_matrix

# Add project root to sys.path so 'model' package can be found
sys.path.append(str(Path(__file__).parent.parent))

from model.data_loader import DataLoader
from model.model_archs.svm import SVMClassifier
from model.model_archs.rbfnn import RBFNNClassifier
from model.model_archs.svr import SVRRegressor
from model.model_archs.rf import RFRegressor
from model.model_archs.gb import GBRegressor
from model.model_archs.mlp import MLPRegressor
from model.model_archs.gru import GRURegressor
from model.model_archs.lstm import LSTMRegressor
from model.model_archs.cnn_lstm import CNNLSTMRegressor
from model.model_archs.transformer import TimeSeriesTransformerRegressor
from model.config_model import (
    SVM_CONFIG, RBFNN_CONFIG, SVR_CONFIG, RF_CONFIG, GB_CONFIG, MLP_CONFIG, GRU_CONFIG, LSTM_CONFIG, CNN_LSTM_CONFIG, TRANSFORMER_CONFIG, CV_CONFIG
)

from sklearn.metrics import (
    classification_report, accuracy_score, confusion_matrix,
    mean_absolute_error, mean_squared_error, r2_score
)

###########################################################
# CONFIGURATION
###########################################################
# Choose model to train: "svm", "rbfnn", "svr", "rf", "gb", "mlp", "gru", "lstm", "cnn_lstm", or "transformer"
MODEL_TYPE = "svr"
###########################################################

def initialize_model(model_type: str):
    """Factory function to initialize the correct model based on type."""
    model_type = model_type.lower()
    if model_type == "svm":
        print(f"Initializing SVM Classifier with config: {SVM_CONFIG}")
        from model.model_archs.svm import SVMClassifier
        return SVMClassifier(**SVM_CONFIG)
    elif model_type == "rbfnn":
        print(f"Initializing RBF Neural Network Classifier with config: {RBFNN_CONFIG}")
        from model.model_archs.rbfnn import RBFNNClassifier
        return RBFNNClassifier(**RBFNN_CONFIG)
    elif model_type == "svr":
        print(f"Initializing SVR Regressor with config: {SVR_CONFIG}")
        from model.model_archs.svr import SVRRegressor
        return SVRRegressor(**SVR_CONFIG)
    elif model_type == "rf":
        print(f"Initializing Random Forest Regressor with config: {RF_CONFIG}")
        from model.model_archs.rf import RFRegressor
        return RFRegressor(**RF_CONFIG)
    elif model_type == "gb":
        print(f"Initializing Gradient Boosting Regressor with config: {GB_CONFIG}")
        from model.model_archs.gb import GBRegressor
        return GBRegressor(**GB_CONFIG)
    elif model_type == "mlp":
        print(f"Initializing MLP Regressor with config: {MLP_CONFIG}")
        from model.model_archs.mlp import MLPRegressor
        return MLPRegressor(**MLP_CONFIG)
    elif model_type == "gru":
        print(f"Initializing GRU Regressor with config: {GRU_CONFIG}")
        from model.model_archs.gru import GRURegressor
        return GRURegressor(**GRU_CONFIG)
    elif model_type == "lstm":
        print(f"Initializing LSTM Regressor with config: {LSTM_CONFIG}")
        from model.model_archs.lstm import LSTMRegressor
        return LSTMRegressor(**LSTM_CONFIG)
    elif model_type == "cnn_lstm":
        print(f"Initializing CNN-LSTM Regressor with config: {CNN_LSTM_CONFIG}")
        from model.model_archs.cnn_lstm import CNNLSTMRegressor
        return CNNLSTMRegressor(**CNN_LSTM_CONFIG)
    elif model_type == "transformer":
        print(f"Initializing Transformer Regressor with config: {TRANSFORMER_CONFIG}")
        from model.model_archs.transformer import TimeSeriesTransformerRegressor
        return TimeSeriesTransformerRegressor(**TRANSFORMER_CONFIG)
    else:
        print(f"Unknown model type: {model_type}")
        return None

def calculate_per_weight_metrics(y_true, y_pred):
    """Calculate MAE and RMSE for each unique weight in y_true."""
    unique_weights = np.sort(np.unique(y_true))
    per_weight_results = []
    
    for w in unique_weights:
        mask = (y_true == w)
        if np.any(mask):
            w_true = y_true[mask]
            w_pred = y_pred[mask]
            mae = mean_absolute_error(w_true, w_pred)
            rmse = np.sqrt(mean_squared_error(w_true, w_pred))
            per_weight_results.append({
                'Weight': f"{w:.2f} kg",
                'Count': int(np.sum(mask)),
                'MAE': f"{mae:.4f}",
                'RMSE': f"{rmse:.4f}"
            })
    return per_weight_results

def main():
    # Define paths
    base_dir = Path(__file__).parent.parent
    segments_dir = base_dir / "database" / "segments"
    results_dir = base_dir / "model" / "model_results"
    
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
    
    model_type = MODEL_TYPE.lower()
    if model_type in ["gru", "lstm", "cnn_lstm", "transformer"]:
        # Choose the right config for window parameters (they are identical in default, but good practice)
        if model_type == "gru":
            conf = GRU_CONFIG
        elif model_type == "lstm":
            conf = LSTM_CONFIG
        elif model_type == "transformer":
            conf = TRANSFORMER_CONFIG
        else:
            conf = CNN_LSTM_CONFIG
        df = loader.load_and_extract_features(h5_paths, 
                                              window_size_sec=conf.get('window_size_sec', 0.25),
                                              window_step_sec=conf.get('window_step_sec', 0.1))
    else:
        df = loader.load_and_extract_features(h5_paths)
    
    if df.empty:
        print("Data extraction failed or produced an empty DataFrame.")
        return
        
    print(f"Extracted features dataframe shape: {df.shape}")
    
    # Prepare data for ML
    # Prepare data for ML
    is_regression = (model_type in ["svr", "rf", "gb", "mlp", "gru", "lstm", "cnn_lstm", "transformer"])
    target_col = "weight" if is_regression else "label"
    
    X, y = loader.prepare_for_ml(df, target_col=target_col)
    print(f"Feature matrix (X) shape: {X.shape}")
    print(f"Label vector (y) shape: {y.shape} (Target: {target_col})")
    
    if not is_regression and len(y.unique()) < 2:
        print("\nLess than 2 classes found in dataset. Ensure your target labels are correct and varied.")
        return

    # Determine if we use Cross-Validation or Single Split
    use_cv = CV_CONFIG.get('use_cross_val', False)
    
    if use_cv:
        n_folds = CV_CONFIG.get('n_folds', 5)
        print(f"\nStarting {n_folds}-Fold Stratified Cross-Validation...")
        
        skf = StratifiedKFold(n_splits=n_folds, shuffle=True, random_state=42)
        
        cv_metrics = []
        fold_results = []
        oof_predictions = np.zeros(len(X)) if is_regression else None
        
        # Use df['label'] for stratification even if predicting weight
        strat_labels = df["label"] if "label" in df.columns else None
        
        for fold, (train_idx, test_idx) in enumerate(skf.split(X, strat_labels), 1):
            print(f"--- Fold {fold}/{n_folds} ---")
            
            X_train_fold, X_test_fold = X.iloc[train_idx], X.iloc[test_idx]
            y_train_fold, y_test_fold = y.iloc[train_idx], y.iloc[test_idx]
            
            # Initialize Model
            model = initialize_model(model_type)
            if model is None: return
            
            # Train
            model.fit(X_train_fold, y_train_fold)
            
            # Evaluate
            if is_regression:
                metrics, report_str = model.evaluate(X_test_fold, y_test_fold)
                cv_metrics.append(metrics)
                fold_results.append(report_str)
                oof_predictions[test_idx] = model.predict(X_test_fold)
            else:
                y_pred_fold = model.predict(X_test_fold)
                acc = accuracy_score(y_test_fold, y_pred_fold)
                cv_metrics.append({'accuracy': acc})
                fold_results.append(classification_report(y_test_fold, y_pred_fold, zero_division=0))
        
        # Aggregate Results
        print("\n" + "=" * 55)
        print(f"CROSS-VALIDATION RESULTS ({n_folds} Folds)")
        
        if is_regression:
            avg_metrics = {k: np.mean([m[k] for m in cv_metrics]) for k in cv_metrics[0].keys()}
            std_metrics = {k: np.std([m[k] for m in cv_metrics]) for k in cv_metrics[0].keys()}
            
            report_str = ""
            for k in avg_metrics:
                report_str += f"{k}: {avg_metrics[k]:.4f} (±{std_metrics[k]:.4f})\n"
            print(report_str)
        else:
            avg_acc = np.mean([m['accuracy'] for m in cv_metrics])
            std_acc = np.std([m['accuracy'] for m in cv_metrics])
            report_str = f"Average Accuracy: {avg_acc:.4f} (±{std_acc:.4f})\n"
            print(f"Model Accuracy: {avg_acc:.4f} (±{std_acc:.4f})")
            
        print("=" * 55)
        
        # For saving, train a final model on the full training set (or full dataset)
        print("\nTraining final model on full dataset for saving...")
        model = initialize_model(model_type)
        model.fit(X, y)
        
    else:
        # Train/Test Split
        # Always stratify by labels (classes) if possible, even for regression
        stratify = df["label"] if "label" in df.columns else None
        
        X_train, X_test, y_train, y_test = train_test_split(
            X, y, test_size=0.2, random_state=7, stratify=stratify
        )
        
        print(f"\nTraining on {len(X_train)} samples, testing on {len(X_test)} samples.")
        
        # Initialize and train the selected Model
        model = initialize_model(model_type)
        if model is None: return

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
            
            print("=" * 55)
            print(f"Model Accuracy: {accuracy:.4f}")
            print("Classification Report:")
            print(report_str)
            print("=" * 55)
    
    # Save the Model structure
    model_path = run_dir / f"{model_type}_model.joblib"
    model.save(model_path)
    
    # Save regression plot if applicable (use out-of-fold predictions if CV)
    if is_regression:
        plot_path = run_dir / "regression_plot.png"
        if use_cv:
            model.plot_results(y, oof_predictions, plot_path)
        else:
            y_pred = model.predict(X_test)
            model.plot_results(y_test, y_pred, plot_path)
    
    
    # Calculate per-weight statistics for regression
    per_weight_stats = None
    if is_regression:
        if use_cv:
            per_weight_stats = calculate_per_weight_metrics(y, oof_predictions)
        else:
            y_pred_test = model.predict(X_test)
            per_weight_stats = calculate_per_weight_metrics(y_test, y_pred_test)

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
        if use_cv:
            f.write(f"Evaluation Mode: {n_folds}-Fold Cross-Validation\n")
            f.write(f"Final Model Training samples: {getattr(model, 'train_samples', len(X))}\n")
            if hasattr(model, 'val_samples') and model.val_samples > 0:
                f.write(f"Final Model Validation samples: {model.val_samples}\n")
        else:
            # Determine split percentages
            test_pct = round(len(X_test) / len(X) * 100)
            train_samples = getattr(model, 'train_samples', len(X_train))
            val_samples = getattr(model, 'val_samples', 0)
            
            if val_samples > 0:
                train_pct = round(train_samples / len(X) * 100)
                val_pct = round(val_samples / len(X) * 100)
                f.write(f"Evaluation Mode: Train/Validation/Test Split ({train_pct}/{val_pct}/{test_pct})\n")
            else:
                train_pct = round(train_samples / len(X) * 100)
                f.write(f"Evaluation Mode: Train/Test Split ({train_pct}/{test_pct})\n")
            
            f.write(f"Testing samples: {len(X_test)}\n")
            f.write(f"Training samples: {train_samples}\n")
            if val_samples > 0:
                f.write(f"Validation samples: {val_samples}\n")
        f.write("\n")
        
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
        elif model_type == "rf":
            f.write(f"N Estimators: {model.n_estimators}\n")
            f.write(f"Max Depth: {model.max_depth}\n")
            f.write(f"Min Samples Split: {model.min_samples_split}\n")
            f.write(f"Random State: {model.random_state}\n\n")
        elif model_type == "gb":
            f.write(f"N Estimators: {model.n_estimators}\n")
            f.write(f"Learning Rate: {model.learning_rate}\n")
            f.write(f"Max Depth: {model.max_depth}\n")
            f.write(f"Random State: {model.random_state}\n\n")
        elif model_type == "mlp":
            f.write(f"Hidden Layers: {model.hidden_layers}\n")
            f.write(f"Dropout Rate: {model.dropout_rate}\n")
            f.write(f"Learning Rate: {model.learning_rate}\n")
            f.write(f"Batch Size: {model.batch_size}\n")
            f.write(f"Epochs: {model.epochs}\n")
            f.write(f"Random State: {model.random_state}\n\n")
        elif model_type == "gru":
            f.write(f"Hidden Size: {model.hidden_size}\n")
            f.write(f"Num Layers: {model.num_layers}\n")
            f.write(f"Dropout Rate: {model.dropout_rate}\n")
            f.write(f"Learning Rate: {model.learning_rate}\n")
            f.write(f"Batch Size: {model.batch_size}\n")
            f.write(f"Epochs: {model.epochs}\n")
            f.write(f"Window Size (s): {model.window_size_sec}\n")
            f.write(f"Window Step (s): {model.window_step_sec}\n")
            f.write(f"Random State: {model.random_state}\n\n")
        elif model_type == "lstm":
            f.write(f"Hidden Size: {model.hidden_size}\n")
            f.write(f"Num Layers: {model.num_layers}\n")
            f.write(f"Dropout Rate: {model.dropout_rate}\n")
            f.write(f"Learning Rate: {model.learning_rate}\n")
            f.write(f"Batch Size: {model.batch_size}\n")
            f.write(f"Epochs: {model.epochs}\n")
            f.write(f"Window Size (s): {model.window_size_sec}\n")
            f.write(f"Window Step (s): {model.window_step_sec}\n")
            f.write(f"Random State: {model.random_state}\n\n")
        elif model_type == "cnn_lstm":
            f.write(f"CNN Filters: {model.cnn_filters}\n")
            f.write(f"CNN Kernel Size: {model.cnn_kernel_size}\n")
            f.write(f"LSTM Hidden Size: {model.lstm_hidden_size}\n")
            f.write(f"LSTM Num Layers: {model.lstm_num_layers}\n")
            f.write(f"Dropout Rate: {model.dropout_rate}\n")
            f.write(f"Learning Rate: {model.learning_rate}\n")
            f.write(f"Batch Size: {model.batch_size}\n")
            f.write(f"Epochs: {model.epochs}\n")
            f.write(f"Window Size (s): {model.window_size_sec}\n")
            f.write(f"Window Step (s): {model.window_step_sec}\n")
            f.write(f"Random State: {model.random_state}\n\n")
        elif model_type == "transformer":
            f.write(f"D_Model: {model.d_model}\n")
            f.write(f"N_Heads: {model.nhead}\n")
            f.write(f"Num Layers: {model.num_layers}\n")
            f.write(f"Feedforward Dim: {model.dim_feedforward}\n")
            f.write(f"Dropout Rate: {model.dropout_rate}\n")
            f.write(f"Learning Rate: {model.learning_rate}\n")
            f.write(f"Batch Size: {model.batch_size}\n")
            f.write(f"Epochs: {model.epochs}\n")
            f.write(f"Window Size (s): {model.window_size_sec}\n")
            f.write(f"Window Step (s): {model.window_step_sec}\n")
            f.write(f"Random State: {model.random_state}\n\n")
        
        f.write("--- EVALUATION METRICS ---\n")
        if use_cv:
            f.write(f"(Averaged over {n_folds} folds)\n")
            f.write(report_str)
        else:
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
        
        if per_weight_stats:
            f.write("\n--- PER-WEIGHT METRICS ---\n")
            f.write(f"{'Weight':<12} | {'Count':<8} | {'MAE':<10} | {'RMSE':<10}\n")
            f.write("-" * 50 + "\n")
            for stats in per_weight_stats:
                f.write(f"{stats['Weight']:<12} | {stats['Count']:<8} | {stats['MAE']:<10} | {stats['RMSE']:<10}\n")
            f.write("\n")
            
    print(f"\nPerformance report saved to {report_file}")

if __name__ == "__main__":
    main()
