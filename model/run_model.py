import os
import json
import numpy as np
import sys
from pathlib import Path
from datetime import datetime
import time
import torch
from sklearn.model_selection import train_test_split, StratifiedKFold
from sklearn.metrics import classification_report, accuracy_score, confusion_matrix

# Add project root to sys.path so 'model' package can be found
sys.path.append(str(Path(__file__).parent.parent))

from model import performance_utils, plotting_utils
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
    SVM_CONFIG, RBFNN_CONFIG, SVR_CONFIG, RF_CONFIG, GB_CONFIG, MLP_CONFIG, GRU_CONFIG, LSTM_CONFIG, CNN_LSTM_CONFIG, TRANSFORMER_CONFIG, CV_CONFIG, FEATURE_CONFIG
)

from sklearn.metrics import (
    classification_report, accuracy_score, confusion_matrix,
    mean_absolute_error, mean_squared_error, r2_score
)

###########################################################
# CONFIGURATION
###########################################################
# Choose model to train:
MODEL_TYPE = "gru"  # Options: "svr", "rf", "gb", "mlp", "gru", "lstm", "cnn_lstm", "transformer"
TRAIN_TEST_SPLIT = 0.2
USE_CROSS_VAL = False
RUN_GRID_SEARCH = False

# Use pre-extracted features stored in the segment HDF5 files.
# Run 'python -m model.feature_extraction' to compute and store them.
# Falls back to live extraction automatically if no precomputed features exist.
USE_PRECOMPUTED_FEATURES = True
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

def calculate_per_seqlen_metrics(y_true, y_pred, seq_lengths):
    """Calculate MAE and RMSE grouped by sequence length (number of windows per lift).
    
    Bins lengths into groups of 1 window each up to a threshold, then merges
    the long-tail into a single '>= N' bucket so the table stays readable.
    """
    y_true = np.asarray(y_true)
    y_pred = np.asarray(y_pred)
    seq_lengths = np.asarray(seq_lengths, dtype=int)

    unique_lens = np.sort(np.unique(seq_lengths))
    results = []

    for L in unique_lens:
        mask = (seq_lengths == L)
        mae  = mean_absolute_error(y_true[mask], y_pred[mask])
        rmse = np.sqrt(mean_squared_error(y_true[mask], y_pred[mask]))
        results.append({
            'SeqLen':    L,
            'TimeAtPrediction': None,   # filled below
            'Count':     int(np.sum(mask)),
            'MAE':       f"{mae:.4f}",
            'RMSE':      f"{rmse:.4f}",
        })
    return results


def calculate_per_duration_metrics(y_true, y_pred, durations_sec,
                                   n_bins: int = 60):
    """Calculate MAE and RMSE for the CNN-LSTM binned by raw segment duration.

    Because CNN-LSTM receives the full raw segment (no sliding windows), the
    natural equivalent of 'number of windows seen' is the elapsed time into the
    lift at the moment of prediction, i.e. the segment duration in seconds.

    Parameters
    ----------
    durations_sec : array-like
        Duration of each test segment in seconds.
    n_bins : int
        Number of equal-width duration bins.

    Returns
    -------
    list[dict]
        Same format as ``calculate_per_seqlen_metrics`` so
        ``plot_seqlen_performance`` can be reused directly.
    """
    y_true = np.asarray(y_true)
    y_pred = np.asarray(y_pred)
    durations_sec = np.asarray(durations_sec, dtype=float)

    # Build equal-width bins spanning the full duration range
    bin_edges = np.linspace(durations_sec.min(), durations_sec.max(), n_bins + 1)

    results = []
    for i in range(len(bin_edges) - 1):
        lo, hi = bin_edges[i], bin_edges[i + 1]
        # Include the right edge in the last bin
        if i < len(bin_edges) - 2:
            mask = (durations_sec >= lo) & (durations_sec < hi)
        else:
            mask = (durations_sec >= lo) & (durations_sec <= hi)

        count = int(np.sum(mask))
        if count == 0:
            continue

        mae  = mean_absolute_error(y_true[mask], y_pred[mask])
        rmse = np.sqrt(mean_squared_error(y_true[mask], y_pred[mask]))
        mid  = (lo + hi) / 2  # representative time for this bin

        results.append({
            'SeqLen':          i + 1,           # bin index (used as x-tick label prefix)
            'TimeAtPrediction': f"{mid:.3f}s",  # elapsed time — the real x-axis value
            'Count':           count,
            'MAE':             f"{mae:.4f}",
            'RMSE':            f"{rmse:.4f}",
        })
    return results


def main():
    # Define paths
    base_dir = Path(__file__).parent.parent
    
    if RUN_GRID_SEARCH:
        print(f"\n--- GRID SEARCH ENABLED ---")
        print(f"Launching hyperparameter sweep for {MODEL_TYPE.upper()}...")
        sweep_script = base_dir / "model" / "model_archs" / "model_hp_opti" / f"sweep_{MODEL_TYPE.lower()}.py"
        if sweep_script.exists():
            import subprocess
            subprocess.run([sys.executable, str(sweep_script)])
        else:
            print(f"Error: Grid search script {sweep_script.name} not found.")
        return
        
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
    is_sequence = model_type in ["gru", "lstm", "transformer"]
    is_raw_segment = model_type == "cnn_lstm"
    target_col = "weight"
    
    # Load data — raw segments for CNN-LSTM, features for everything else
    if is_raw_segment:
        df = loader.load_raw_segments(h5_paths)
    else:
        df = loader.load_and_extract_features(
            h5_paths,
            is_sequence=is_sequence,
            use_precomputed=USE_PRECOMPUTED_FEATURES,
        )
    
    if df.empty:
        print("Data extraction failed or produced an empty DataFrame.")
        return
        
    print(f"Extracted features dataframe shape: {df.shape}")
    
    # Prepare data for ML
    
    X, y = loader.prepare_for_ml(df, target_col=target_col)
    
    if is_raw_segment:
        seg_lengths = np.array([seg.shape[0] for seg in X['raw_segment']])
        n_channels = X['raw_segment'].iloc[0].shape[1]
        avg_seg_len = seg_lengths.mean()
        print(f"Raw Segment Data Summary:")
        print(f"  - Total Segments: {len(X)}")
        print(f"  - Channels (EMG + IMU): {n_channels}")
        print(f"  - Avg Segment Length: {avg_seg_len:.0f} samples")
        print(f"  - Min / Max Segment Length: {seg_lengths.min()} / {seg_lengths.max()} samples")
    elif is_sequence:
        avg_seq_len = np.mean([len(s) for s in X.iloc[:, 0]])
        # Access first window of first sequence to get feature count
        first_seq = X.iloc[0, 0]
        num_feats_per_window = len(first_seq[0]) if first_seq else 0
        print(f"Sequence Data Summary:")
        print(f"  - Total Lifts (Sequences): {len(X)}")
        print(f"  - Average Windows per Lift: {avg_seq_len:.1f}")
        print(f"  - Features per Window: {num_feats_per_window}")
    else:
        print(f"Feature matrix (X) shape: {X.shape}")
        
    print(f"Label vector (y) shape: {y.shape} (Target: {target_col})")
    
    # Determine if we use Cross-Validation or Single Split
    use_cv = CV_CONFIG.get('use_cross_val', False)
    
    if use_cv:
        n_folds = CV_CONFIG.get('n_folds', 5)
        print(f"\nStarting {n_folds}-Fold Stratified Cross-Validation...")
        
        skf = StratifiedKFold(n_splits=n_folds, shuffle=True, random_state=42)
        
        cv_metrics = []
        fold_results = []
        oof_predictions = np.zeros(len(X))
        
        # Use df['weight'] (cast to str) for stratification
        strat_labels = df["weight"].astype(str) if "weight" in df.columns else None
        
        for fold, (train_idx, test_idx) in enumerate(skf.split(X, strat_labels), 1):
            print(f"--- Fold {fold}/{n_folds} ---")
            
            X_train_fold, X_test_fold = X.iloc[train_idx], X.iloc[test_idx]
            y_train_fold, y_test_fold = y.iloc[train_idx], y.iloc[test_idx]
            
            # Initialize Model
            model = initialize_model(model_type)
            if model is None: return
            
            # Train
            start_train = time.perf_counter()
            model.fit(X_train_fold, y_train_fold)
            train_time = time.perf_counter() - start_train
            
            # Evaluate
            start_inf = time.perf_counter()
            metrics, report_str = model.evaluate(X_test_fold, y_test_fold)
            inf_time = time.perf_counter() - start_inf
            
            # Store times in metrics for reporting
            metrics['train_time'] = train_time
            metrics['inference_time_total'] = inf_time
            metrics['inference_time_per_sample'] = inf_time / len(X_test_fold)
            
            cv_metrics.append(metrics)
            fold_results.append(report_str)
            oof_predictions[test_idx] = model.predict(X_test_fold)
        
        # Aggregate Results
        print("\n" + "=" * 55)
        print(f"CROSS-VALIDATION RESULTS ({n_folds} Folds)")
        
        avg_metrics = {k: np.mean([m[k] for m in cv_metrics]) for k in cv_metrics[0].keys()}
        std_metrics = {k: np.std([m[k] for m in cv_metrics]) for k in cv_metrics[0].keys()}
        
        report_str = ""
        for k in avg_metrics:
            report_str += f"{k}: {avg_metrics[k]:.4f} (±{std_metrics[k]:.4f})\n"
        print(report_str)
            
        print("=" * 55)
        
        # For saving, train a final model on the full training set (or full dataset)
        print("\nTraining final model on full dataset for saving...")
        model = initialize_model(model_type)
        model.fit(X, y)
        
    else:
        # Train/Test Split
        # Always stratify by weight (cast to str) for regression
        stratify = df["weight"].astype(str) if "weight" in df.columns else None
        
        X_train, X_test, y_train, y_test = train_test_split(
            X, y, test_size=0.2, random_state=42, stratify=stratify
        )
        
        print(f"\nTraining on {len(X_train)} samples, testing on {len(X_test)} samples.")
        
        # Initialize and train the selected Model
        model = initialize_model(model_type)
        if model is None: return

        # Train
        start_train = time.perf_counter()
        model.fit(X_train, y_train)
        train_time = time.perf_counter() - start_train
        
        # Evaluate model
        print(f"\nEvaluating {model_type.upper()} Model on unseen test set...")
        
        start_inf = time.perf_counter()
        metrics, report_str = model.evaluate(X_test, y_test)
        inf_time = time.perf_counter() - start_inf
        
        # Add timing to metrics
        metrics['train_time'] = train_time
        metrics['inference_time_total'] = inf_time
        metrics['inference_time_per_sample'] = inf_time / len(X_test)

        print("=" * 55)
        print("Regression Metrics:")
        print(report_str)
        print(f"Training Time: {train_time:.2f}s")
        print(f"Inference Time (per sample): {metrics['inference_time_per_sample']*1000:.4f}ms")
        print("=" * 55)
    
    # Save the Model structure
    model_path = run_dir / f"{model_type}_model.joblib"
    model.save(model_path)
    
    # Save regression plot if applicable (use out-of-fold predictions if CV)
    plot_path = run_dir / "regression_plot.png"
    if use_cv:
        model.plot_results(y, oof_predictions, plot_path)
    else:
        y_pred = model.predict(X_test)
        model.plot_results(y_test, y_pred, plot_path)
    
    # Save loss plot if applicable (for deep learning models)
    if hasattr(model, 'plot_loss'):
        loss_plot_path = run_dir / "loss_plot.png"
        model.plot_loss(loss_plot_path)
        print(f"Loss plot saved to {loss_plot_path}")
    
    
    # Calculate per-weight statistics for regression
    per_weight_stats = None
    if use_cv:
        per_weight_stats = calculate_per_weight_metrics(y, oof_predictions)
    else:
        # Reuse y_pred already computed for the regression plot above
        per_weight_stats = calculate_per_weight_metrics(y_test, y_pred)

    # Calculate per-sequence-length statistics (sequence models only)
    per_seqlen_stats = None
    if is_sequence and not is_raw_segment:
        if use_cv:
            # For CV we don't easily have X at test time; skip for now
            pass
        else:
            # Extract number of windows for each test sample from the sequence_dicts column
            seq_col = 'sequence_dicts'
            if seq_col in X_test.columns:
                test_seq_lengths = np.array([len(row) for row in X_test[seq_col]])
            else:
                # Fallback: try the first (and only) column
                test_seq_lengths = np.array([len(row) for row in X_test.iloc[:, 0]])

            per_seqlen_stats = calculate_per_seqlen_metrics(
                y_test.values, y_pred, test_seq_lengths
            )
            # Annotate with the real-time elapsed seconds at prediction
            step = FEATURE_CONFIG.get('window_step_sec', 0.1)
            max_win = max(FEATURE_CONFIG.get('emg_window_size_sec', 0.15),
                          FEATURE_CONFIG.get('imu_window_size_sec', 0.3))
            for row in per_seqlen_stats:
                # time elapsed when the L-th window completes: max_window + (L-1)*step
                row['TimeAtPrediction'] = f"{max_win + (row['SeqLen'] - 1) * step:.3f}s"

            # Save the segment-length performance plot
            if per_seqlen_stats:
                seqlen_plot_path = run_dir / "seqlen_performance_plot.png"
                plotting_utils.plot_seqlen_performance(
                    per_seqlen_stats, seqlen_plot_path, model_name=model_type.upper()
                )

    # CNN-LSTM: bin test samples by raw segment duration (elapsed time into lift)
    per_duration_stats = None
    if is_raw_segment and not use_cv:
        # segment_duration_sec was stored in df before prepare_for_ml dropped it
        if 'segment_duration_sec' in df.columns:
            test_durations = df.loc[X_test.index, 'segment_duration_sec'].values
            per_duration_stats = calculate_per_duration_metrics(
                y_test.values, y_pred, test_durations, n_bins=60
            )
            if per_duration_stats:
                seqlen_plot_path = run_dir / "seqlen_performance_plot.png"
                plotting_utils.plot_seqlen_performance(
                    per_duration_stats, seqlen_plot_path, model_name="CNN-LSTM"
                )
        else:
            print("[WARN] segment_duration_sec not found in df — skipping CNN-LSTM seqlen plot.")

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
            f.write(f"Evaluation Mode: {n_folds}-Fold Stratified Cross-Validation (Intra-Subject, Sequences Intact)\n")
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

        f.write("--- FEATURE CONFIGURATION ---\n")
        if is_raw_segment:
            f.write(f"Input: Raw EMG + IMU segments (end-to-end CNN feature extraction)\n")
            emg_ch = [k for k, v in FEATURE_CONFIG.get('emg_channels', {}).items() if v]
            imu_ch = [k for k, v in FEATURE_CONFIG.get('imu_channels', {}).items() if v]
            f.write(f"Enabled EMG Channels ({len(emg_ch)}): {', '.join(emg_ch)}\n")
            f.write(f"Enabled IMU Channels ({len(imu_ch)}): {', '.join(imu_ch)}\n")
        else:
            f.write(f"EMG Window Size: {FEATURE_CONFIG['emg_window_size_sec']}s, IMU Window Size: {FEATURE_CONFIG['imu_window_size_sec']}s, Step: {FEATURE_CONFIG['window_step_sec']}s\n")
            
            emg_enabled = [k for k, v in FEATURE_CONFIG['emg_features'].items() if v]
            f.write(f"Enabled EMG Features: {', '.join(emg_enabled)}\n")
            
            imu_enabled = [k for k, v in FEATURE_CONFIG['imu_features'].items() if v]
            f.write(f"Enabled IMU Features: {', '.join(imu_enabled)}\n")
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
            f.write(f"Loss Function: {getattr(model, 'loss_type', 'mse').upper()}\n")
            f.write(f"Random State: {model.random_state}\n\n")
        elif model_type == "gru":
            f.write(f"Hidden Size: {model.hidden_size}\n")
            f.write(f"Num Layers: {model.num_layers}\n")
            f.write(f"Dropout Rate: {model.dropout_rate}\n")
            f.write(f"Learning Rate: {model.learning_rate}\n")
            f.write(f"Batch Size: {model.batch_size}\n")
            f.write(f"Epochs: {model.epochs}\n")
            f.write(f"Loss Function: {getattr(model, 'loss_type', 'mse').upper()}\n")
            f.write(f"EMG Window Size (s): {model.emg_window_size_sec}\n")
            f.write(f"IMU Window Size (s): {model.imu_window_size_sec}\n")
            f.write(f"Window Step (s): {model.window_step_sec}\n")
            f.write(f"Random State: {model.random_state}\n\n")
        elif model_type == "lstm":
            f.write(f"Hidden Size: {model.hidden_size}\n")
            f.write(f"Num Layers: {model.num_layers}\n")
            f.write(f"Dropout Rate: {model.dropout_rate}\n")
            f.write(f"Learning Rate: {model.learning_rate}\n")
            f.write(f"Batch Size: {model.batch_size}\n")
            f.write(f"Epochs: {model.epochs}\n")
            f.write(f"Loss Function: {getattr(model, 'loss_type', 'mse').upper()}\n")
            f.write(f"EMG Window Size (s): {model.emg_window_size_sec}\n")
            f.write(f"IMU Window Size (s): {model.imu_window_size_sec}\n")
            f.write(f"Window Step (s): {model.window_step_sec}\n")
            f.write(f"Random State: {model.random_state}\n\n")
        elif model_type == "cnn_lstm":
            f.write(f"CNN Filters: {model.cnn_filters}\n")
            f.write(f"CNN Kernel Sizes: {model.cnn_kernel_sizes}\n")
            f.write(f"Pool Size: {model.pool_size}\n")
            f.write(f"LSTM Hidden Size: {model.lstm_hidden_size}\n")
            f.write(f"LSTM Num Layers: {model.lstm_num_layers}\n")
            f.write(f"Dropout Rate: {model.dropout_rate}\n")
            f.write(f"Learning Rate: {model.learning_rate}\n")
            f.write(f"Batch Size: {model.batch_size}\n")
            f.write(f"Epochs: {model.epochs}\n")
            f.write(f"Loss Function: {getattr(model, 'loss_type', 'mse').upper()}\n")
            f.write(f"Input Channels: {model.n_channels}\n")
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
            f.write(f"Loss Function: {getattr(model, 'loss_type', 'mse').upper()}\n")
            f.write(f"EMG Window Size (s): {model.emg_window_size_sec}\n")
            f.write(f"IMU Window Size (s): {model.imu_window_size_sec}\n")
            f.write(f"Window Step (s): {model.window_step_sec}\n")
            f.write(f"Random State: {model.random_state}\n\n")
        
        f.write("--- EVALUATION METRICS ---\n")
        if use_cv:
            f.write(f"(Averaged over {n_folds} folds)\n")
            f.write(report_str)
        else:
            f.write(report_str)
        
        if per_weight_stats:
            f.write("\n--- PER-WEIGHT METRICS ---\n")
            f.write(f"{'Weight':<12} | {'Count':<8} | {'MAE':<10} | {'RMSE':<10}\n")
            f.write("-" * 50 + "\n")
            for stats in per_weight_stats:
                f.write(f"{stats['Weight']:<12} | {stats['Count']:<8} | {stats['MAE']:<10} | {stats['RMSE']:<10}\n")
            f.write("\n")

        if per_seqlen_stats:
            f.write("--- PER-SEQUENCE-LENGTH METRICS ---\n")
            f.write("(How many sliding windows were available when the LSTM made its prediction)\n")
            f.write(f"Window step: {FEATURE_CONFIG.get('window_step_sec', '?')}s  |  "
                    f"First window ready at: {max(FEATURE_CONFIG.get('emg_window_size_sec', 0), FEATURE_CONFIG.get('imu_window_size_sec', 0)):.3f}s\n")
            f.write(f"{'# Windows':<12} | {'Time into lift':<16} | {'Count':<8} | {'MAE':<10} | {'RMSE':<10}\n")
            f.write("-" * 65 + "\n")
            for stats in per_seqlen_stats:
                f.write(f"{stats['SeqLen']:<12} | {stats['TimeAtPrediction']:<16} | "
                        f"{stats['Count']:<8} | {stats['MAE']:<10} | {stats['RMSE']:<10}\n")
            f.write("\n")

        if per_duration_stats:
            f.write("--- PER-SEGMENT-DURATION METRICS (CNN-LSTM) ---\n")
            f.write("(Prediction error vs. elapsed time into lift, binned from raw segment duration)\n")
            f.write(f"{'Bin':<6} | {'Time into lift (mid)':<22} | {'Count':<8} | {'MAE':<10} | {'RMSE':<10}\n")
            f.write("-" * 68 + "\n")
            for stats in per_duration_stats:
                f.write(f"{stats['SeqLen']:<6} | {stats['TimeAtPrediction']:<22} | "
                        f"{stats['Count']:<8} | {stats['MAE']:<10} | {stats['RMSE']:<10}\n")
            f.write("\n")
            
        # Add Compute Metrics
        f.write("--- COMPUTE & TIMING METRICS ---\n")
        device_info = performance_utils.get_device_info()
        f.write(f"Device: {performance_utils.format_device_string(device_info)}\n")
        
        if hasattr(model, 'model') and isinstance(model.model, torch.nn.Module):
            total_params, trainable_params = performance_utils.count_parameters(model.model)
            # Estimate GFLOPs
            # For sequence models, we need seq_len
            seq_len = None
            if is_sequence:
                # Try to get avg seq len from previous calculation
                try:
                    seq_len = int(avg_seq_len)
                except:
                    seq_len = 100 # Fallback
            
            gflops_per_sample = performance_utils.estimate_flops(model.model, seq_len=seq_len)
            f.write(f"Total Parameters: {total_params:,}\n")
            f.write(f"Trainable Parameters: {trainable_params:,}\n")
            f.write(f"Estimated Inference GFLOPs (per sample): {gflops_per_sample:.6f}\n")
            
            # Training total GFLOPs estimate: GFLOPs * samples * epochs
            total_train_gflops = gflops_per_sample * getattr(model, 'train_samples', 0) * getattr(model, 'epochs', 1) * 3 # 3x for backward pass approx
            f.write(f"Estimated Total Training GFLOPs: {total_train_gflops:.4f}\n")
        else:
            f.write("Parameter count: N/A (Classical ML)\n")
            f.write("GFLOPs estimate: N/A (Classical ML)\n")

        if use_cv:
            avg_train_time = np.mean([m['train_time'] for m in cv_metrics])
            avg_inf_time_sample = np.mean([m['inference_time_per_sample'] for m in cv_metrics])
            f.write(f"Avg Training Time: {avg_train_time:.4f}s\n")
            f.write(f"Avg Inference Time (per sample): {avg_inf_time_sample*1000:.4f}ms\n")
        else:
            f.write(f"Training Time: {metrics['train_time']:.4f}s\n")
            f.write(f"Inference Time (per sample): {metrics['inference_time_per_sample']*1000:.4f}ms\n")
        f.write("\n")
            
    print(f"\nPerformance report saved to {report_file}")

if __name__ == "__main__":
    main()
