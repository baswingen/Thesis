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
from model.report_generator import ReportGenerator
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
from model.model_archs.tcn import TCNRegressor
from model.model_archs.transformer import TimeSeriesTransformerRegressor
from model.config_model import (
    SVM_CONFIG, RBFNN_CONFIG, SVR_CONFIG, RF_CONFIG, GB_CONFIG, MLP_CONFIG, GRU_CONFIG, LSTM_CONFIG, CNN_LSTM_CONFIG, TCN_CONFIG, TRANSFORMER_CONFIG, CV_CONFIG, FEATURE_CONFIG, CHANNEL_CONFIG, PARTICIPANT_CONFIG, DATABASE_CONFIG, AUGMENTATION_CONFIG, GLOBAL_RANDOM_STATE
)
from sklearn.metrics import (
    classification_report, accuracy_score, confusion_matrix,
    mean_absolute_error, mean_squared_error, r2_score
)
from sklearn.utils.class_weight import compute_sample_weight, compute_class_weight

###########################################################
# CONFIGURATION
###########################################################
# Choose model to train:
MODEL_TYPE = "tcn"  # Options: "svr", "rf", "gb", "mlp", "gru", "lstm", "cnn_lstm", "tcn", "transformer"
TRAIN_TEST_SPLIT = 0.2
USE_CROSS_VAL = True
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
    elif model_type == "tcn":
        print(f"Initializing TCN Regressor with config: {TCN_CONFIG}")
        from model.model_archs.tcn import TCNRegressor
        return TCNRegressor(**TCN_CONFIG)
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
                                   n_bins: int = None):
    """Calculate MAE and RMSE for the CNN-LSTM binned by raw segment duration.

    Because CNN-LSTM receives the full raw segment (no sliding windows), the
    natural equivalent of 'number of windows seen' is the elapsed time into the
    lift at the moment of prediction, i.e. the segment duration in seconds.

    Parameters
    ----------
    durations_sec : array-like
        Duration of each test segment in seconds.
    n_bins : int or None
        Number of equal-width duration bins. If None, it will be smartly 
        calculated based on the number of available segments and their lengths.

    Returns
    -------
    list[dict]
        Same format as ``calculate_per_seqlen_metrics`` so
        ``plot_seqlen_performance`` can be reused directly.
    """
    y_true = np.asarray(y_true)
    y_pred = np.asarray(y_pred)
    durations_sec = np.asarray(durations_sec, dtype=float)

    if n_bins is None:
        # SMART BINNING strategy
        # Higher resolution (more bins) proportional to lengths (max duration) and available segments (len)
        max_dur = durations_sec.max()
        n_samples = len(durations_sec)
        
        # Base of 4 bins per second, scaled up by the density of the dataset
        density_multiplier = max(1.0, np.sqrt(n_samples / 300.0))
        n_bins = int(np.ceil(max_dur * 4 * density_multiplier))
        # Put a sane limit so we don't create thousands of bins
        n_bins = min(250, max(10, n_bins))

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


def calculate_class_weights(y):
    """Calculate the 'balanced' weight for each unique class in y."""
    from sklearn.utils.class_weight import compute_class_weight
    y_np = np.asarray(y)
    unique_classes = np.sort(np.unique(y_np))
    weights = compute_class_weight(class_weight='balanced', classes=unique_classes, y=y_np)
    
    # Calculate counts for context
    counts = {c: int(np.sum(y_np == c)) for c in unique_classes}
    
    results = []
    for i, c in enumerate(unique_classes):
        results.append({
            'Weight': f"{c:.2f} kg",
            'Multiplier': f"{weights[i]:.4f}",
            'Count': counts[c]
        })
    return results


def setup_run_dir(base_dir):
    """Create a unique timestamped folder for this run's results."""
    results_dir = base_dir / "model" / "model_results"
    results_dir.mkdir(parents=True, exist_ok=True)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = results_dir / f"run_{timestamp}"
    run_dir.mkdir(exist_ok=True)
    return run_dir, timestamp


def load_and_prepare_data(loader, h5_paths, model_type, is_raw_segment, is_sequence, use_precomputed):
    """Load data and prepare X, y arrays."""
    if is_raw_segment:
        df = loader.load_raw_segments(h5_paths)
    else:
        df = loader.load_and_extract_features(
            h5_paths,
            is_sequence=is_sequence,
            use_precomputed=use_precomputed,
        )
    
    if df.empty:
        return None, None, None, None

    groups = df["subject"].astype(str).values if "subject" in df.columns else None
    X, y = loader.prepare_for_ml(df, target_col="weight")
    return X, y, groups, df


def print_data_summary(X, y, is_raw_segment, is_sequence):
    """Print statistics about the loaded dataset."""
    if is_raw_segment:
        seg_lengths = np.array([seg.shape[0] for seg in X['raw_segment']])
        n_channels = X['raw_segment'].iloc[0].shape[1]
        print(f"Raw Segment Data Summary:")
        print(f"  - Total Segments: {len(X)}")
        print(f"  - Channels (EMG + IMU): {n_channels}")
        print(f"  - Avg Segment Length: {seg_lengths.mean():.0f} samples")
        print(f"  - Min / Max Segment Length: {seg_lengths.min()} / {seg_lengths.max()} samples")
    elif is_sequence:
        avg_seq_len = np.mean([len(s) for s in X.iloc[:, 0]])
        first_seq = X.iloc[0, 0]
        num_feats_per_window = len(first_seq[0]) if first_seq else 0
        print(f"Sequence Data Summary:")
        print(f"  - Total Lifts (Sequences): {len(X)}")
        print(f"  - Average Windows per Lift: {avg_seq_len:.1f}")
        print(f"  - Features per Window: {num_feats_per_window}")
    else:
        print(f"Feature matrix (X) shape: {X.shape}")
    print(f"Label vector (y) shape: {y.shape} (Target: weight)")


def execute_cross_validation(X, y, groups, df, model_type, cv_strategy, n_folds):
    """Run cross-validation and return metrics and predictions."""
    if cv_strategy == 'participant':
        from sklearn.model_selection import LeaveOneGroupOut
        logo = LeaveOneGroupOut()
        cv_iterator = list(logo.split(X, y, groups))
        n_folds = len(cv_iterator)
        print(f"\nStarting {n_folds}-Fold Leave-One-Participant-Out Cross-Validation...")
    else:
        from sklearn.model_selection import StratifiedKFold
        skf = StratifiedKFold(n_splits=n_folds, shuffle=True, random_state=GLOBAL_RANDOM_STATE)
        strat_labels = df["weight"].astype(str) if "weight" in df.columns else None
        cv_iterator = list(skf.split(X, strat_labels))
        print(f"\nStarting {n_folds}-Fold Stratified Cross-Validation...")

    cv_metrics = []
    fold_results = []
    oof_predictions = np.zeros(len(X))
    participant_stats = []
    permutation_importances = []

    for fold, (train_idx, test_idx) in enumerate(cv_iterator, 1):
        if cv_strategy == 'participant':
            left_out_participant = groups[test_idx[0]]
            print(f"--- Fold {fold}/{n_folds} (Left out Participant: {left_out_participant}) ---")
        else:
            print(f"--- Fold {fold}/{n_folds} ---")
        
        X_train_fold, X_test_fold = X.iloc[train_idx], X.iloc[test_idx]
        y_train_fold, y_test_fold = y.iloc[train_idx], y.iloc[test_idx]
        
        model = initialize_model(model_type)
        if model is None: continue
        
        start_train = time.perf_counter()
        sample_weight = None
        if model_type == "svr" and SVR_CONFIG.get('balance_weights', False):
            sample_weight = compute_sample_weight(class_weight='balanced', y=y_train_fold)
            
        model.fit(X_train_fold, y_train_fold, sample_weight=sample_weight)
        train_time = time.perf_counter() - start_train
        
        start_inf = time.perf_counter()
        metrics, fold_report = model.evaluate(X_test_fold, y_test_fold)
        inf_time = time.perf_counter() - start_inf
        
        metrics.update({
            'train_time': train_time,
            'inference_time_total': inf_time,
            'inference_time_per_sample': inf_time / len(X_test_fold)
        })
        
        cv_metrics.append(metrics)
        fold_results.append(fold_report)
        preds = model.predict(X_test_fold)
        oof_predictions[test_idx] = preds
        
        if hasattr(model, 'permutation_importance'):
            try:
                print("Computing permutation importance for fold...")
                imp = model.permutation_importance(X_test_fold, y_test_fold)
                permutation_importances.append(imp)
            except Exception as e:
                print(f"[WARN] Permutation importance computation failed: {e}")
                
        if cv_strategy == 'participant':
            participant_stats.append({
                'Participant': left_out_participant,
                'MAE': mean_absolute_error(y_test_fold, preds),
                'RMSE': np.sqrt(mean_squared_error(y_test_fold, preds)),
                'Samples': len(test_idx)
            })

    print("\nTraining final model on full dataset for saving...")
    model = initialize_model(model_type)
    sample_weight = None
    if model_type == "svr" and SVR_CONFIG.get('balance_weights', False):
        sample_weight = compute_sample_weight(class_weight='balanced', y=y)
    model.fit(X, y, sample_weight=sample_weight)

    avg_permutation_importance = None
    if permutation_importances:
        avg_permutation_importance = {}
        for k in permutation_importances[0].keys():
            avg_permutation_importance[k] = np.mean([pi[k] for pi in permutation_importances])

    return model, cv_metrics, oof_predictions, participant_stats, n_folds, avg_permutation_importance


def execute_single_split(X, y, df, model_type, split_val):
    """Run a single train/test split and return metrics and predictions."""
    stratify = df["weight"].astype(str) if "weight" in df.columns else None
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=split_val, random_state=GLOBAL_RANDOM_STATE, stratify=stratify
    )
    
    print(f"\nTraining on {len(X_train)} samples, testing on {len(X_test)} samples.")
    model = initialize_model(model_type)
    
    start_train = time.perf_counter()
    sample_weight = None
    if model_type == "svr" and SVR_CONFIG.get('balance_weights', False):
        sample_weight = compute_sample_weight(class_weight='balanced', y=y_train)
    model.fit(X_train, y_train, sample_weight=sample_weight)
    train_time = time.perf_counter() - start_train
    
    print(f"\nEvaluating {model_type.upper()} Model on unseen test set...")
    start_inf = time.perf_counter()
    metrics, report_str = model.evaluate(X_test, y_test)
    inf_time = time.perf_counter() - start_inf
    
    metrics.update({
        'train_time': train_time,
        'inference_time_total': inf_time,
        'inference_time_per_sample': inf_time / len(X_test)
    })
    
    y_pred = model.predict(X_test)
    
    perm_imp = None
    if hasattr(model, 'permutation_importance'):
        try:
            print("Computing permutation importance for test set...")
            perm_imp = model.permutation_importance(X_test, y_test)
        except Exception as e:
            print(f"[WARN] Permutation importance computation failed: {e}")
            
    return model, metrics, report_str, X_train, X_test, y_train, y_test, y_pred, perm_imp


def save_basic_artifacts(model, run_dir, model_type, use_cv, y, y_test, y_pred, oof_predictions):
    """Save model file, regression plot, and loss plot."""
    model_path = run_dir / f"{model_type}_model.joblib"
    model.save(model_path)
    
    plot_path = run_dir / "regression_plot.png"
    if use_cv:
        model.plot_results(y, oof_predictions, plot_path)
    else:
        model.plot_results(y_test, y_pred, plot_path)
        
    if hasattr(model, 'plot_loss'):
        loss_plot_path = run_dir / "loss_plot.png"
        model.plot_loss(loss_plot_path)


def save_extended_plots(model, run_dir, model_type, use_cv, df, X, X_test, y, y_test, y_pred, oof_predictions,
                        is_raw_segment, is_sequence, groups, participant_stats):
    """Save specialized plots: t-SNE, participant performance, seqlen/duration performance."""
    if is_raw_segment and hasattr(model, 'plot_tsne') and groups is not None:
        print("\nGenerating t-SNE plot of CNN feature space...")
        try:
            model.plot_tsne(X=X, participants=groups, weights=y.values, save_path=run_dir / "tsne_cnn_features.png")
        except Exception as e:
            print(f"[WARN] t-SNE plot failed: {e}")

    if use_cv and participant_stats:
        plotting_utils.plot_participant_performance(participant_stats, run_dir / "participant_performance.png", 
                                                    model_name=model_type.upper())

    per_seqlen_stats = None
    per_duration_stats = None

    if is_sequence and not is_raw_segment:
        target_y = y.values if use_cv else y_test.values
        target_preds = oof_predictions if use_cv else y_pred
        target_X = X if use_cv else X_test
        
        seq_col = 'sequence_dicts'
        if seq_col in df.columns:
            seq_lens = np.array([len(row) for row in df.loc[target_X.index, seq_col]])
        else:
            seq_lens = np.array([len(row) for row in target_X.iloc[:, 0]])
            
        per_seqlen_stats = calculate_per_seqlen_metrics(target_y, target_preds, seq_lens)
        if per_seqlen_stats:
            step = FEATURE_CONFIG.get('window_step_sec', 0.1)
            max_win = max(FEATURE_CONFIG.get('emg_window_size_sec', 0.15), FEATURE_CONFIG.get('imu_window_size_sec', 0.3))
            for row in per_seqlen_stats:
                row['TimeAtPrediction'] = f"{max_win + (row['SeqLen'] - 1) * step:.3f}s"
            
            plotting_utils.plot_seqlen_performance(per_seqlen_stats, run_dir / "seqlen_performance_plot.png", 
                                                    model_name=model_type.upper(), min_count=max(5, int(len(target_y)*0.01)))

    if is_raw_segment and 'segment_duration_sec' in df.columns:
        target_y = y.values if use_cv else y_test.values
        target_preds = oof_predictions if use_cv else y_pred
        durations = df.loc[X.index if use_cv else X_test.index, 'segment_duration_sec'].values
        
        per_duration_stats = calculate_per_duration_metrics(target_y, target_preds, durations)
        if per_duration_stats:
            plotting_utils.plot_seqlen_performance(per_duration_stats, run_dir / "seqlen_performance_plot.png", 
                                                    model_name="CNN-LSTM", min_count=max(5, int(len(target_y)*0.01)))
            
    return per_seqlen_stats, per_duration_stats


    return per_seqlen_stats, per_duration_stats



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
        
    # 1. Setup
    run_dir, timestamp = setup_run_dir(base_dir)
    print(f"Results will be saved to: {run_dir}")
    
    model_type = MODEL_TYPE.lower()
    is_sequence = model_type in ["gru", "lstm", "transformer"]
    is_raw_segment = model_type in ["cnn_lstm", "tcn"]
    
    # 2. Data Loading
    loader = DataLoader()
    h5_paths = list(DATABASE_CONFIG['segments_dir'].glob("*.h5"))
    if not h5_paths:
        print(f"No HDF5 segment files found in {DATABASE_CONFIG['segments_dir']}.")
        return

    X, y, groups, df = load_and_prepare_data(loader, h5_paths, model_type, is_raw_segment, is_sequence, USE_PRECOMPUTED_FEATURES)
    if X is None: 
        print("Data extraction failed or produced an empty DataFrame.")
        return
    
    # 3. Summary
    print_data_summary(X, y, is_raw_segment, is_sequence)
    # 4. Training & Evaluation
    if USE_CROSS_VAL:
        model, cv_metrics, oof_predictions, participant_stats, actual_n_folds, perm_imp = execute_cross_validation(
            X, y, groups, df, model_type, CV_CONFIG.get('strategy', 'kfold'), CV_CONFIG.get('n_folds', 5)
        )
        avg_metrics = {k: np.mean([m[k] for m in cv_metrics]) for k in cv_metrics[0].keys()}
        std_metrics = {k: np.std([m[k] for m in cv_metrics]) for k in cv_metrics[0].keys()}
        metrics, X_train, X_test, y_train, y_test, y_pred = {}, None, None, None, None, None
    else:
        actual_n_folds = 0
        model, metrics, report_str, X_train, X_test, y_train, y_test, y_pred, perm_imp = execute_single_split(
            X, y, df, model_type, TRAIN_TEST_SPLIT
        )
        avg_metrics, std_metrics, oof_predictions, participant_stats = None, None, None, None

    # 5. Artifacts & Specialized Plots
    save_basic_artifacts(model, run_dir, model_type, USE_CROSS_VAL, y, y_test, y_pred, oof_predictions)
    per_seqlen, per_dur = save_extended_plots(model, run_dir, model_type, USE_CROSS_VAL, df, X, X_test, y, y_test, y_pred, 
                                             oof_predictions, is_raw_segment, is_sequence, groups, participant_stats)
    
    if perm_imp:
        plotting_utils.plot_permutation_importance(perm_imp, run_dir / "permutation_importance.png", model_name=model_type.upper())
    
    # 6. Final Report
    generator = ReportGenerator(run_dir, timestamp)
    generator.generate(h5_paths, X, USE_CROSS_VAL, CV_CONFIG.get('strategy', 'kfold'), 
                       actual_n_folds, model, model_type, X_test, X_train, y, y_train, y_test, 
                       is_raw_segment, avg_metrics, std_metrics, metrics, participant_stats, 
                       per_seqlen, per_dur, oof_predictions, y_pred, perm_imp)


if __name__ == "__main__":
    main()
