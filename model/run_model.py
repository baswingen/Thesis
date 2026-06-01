import os
import json
import numpy as np
import sys
from pathlib import Path
from datetime import datetime
import time
import torch
import copy
from sklearn.model_selection import train_test_split, StratifiedKFold
from sklearn.metrics import classification_report, accuracy_score, confusion_matrix

# Add project root to sys.path so 'model' package can be found
sys.path.append(str(Path(__file__).parent.parent))

from model import performance_utils, plotting_utils, deepshap_analysis
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
from model.model_archs.cnn_gru import CNNGRURegressor
from model.model_archs.cnn_bilstm_attention import CNNBiLSTMAttentionRegressor
from model.model_archs.tcn import TCNRegressor
from model.model_archs.transformer import TimeSeriesTransformerRegressor
from model.model_archs.spatio_temporal_transformer import SpatioTemporalTransformerRegressor
from model.model_archs.spatio_temporal_transformer2 import SpatioTemporalTransformerRegressor2
from model.model_archs.spatio_temporal_transformer3 import SpatioTemporalTransformerRegressor3
from model.model_archs.spatio_temporal_transformer4 import SpatioTemporalTransformerRegressor4
from model.model_archs.spatio_temporal_transformer5 import SpatioTemporalTransformerRegressor5
from model.model_archs.spatio_temporal_transformer6 import SpatioTemporalTransformerRegressor6
from model.model_archs.st_transformer_aksan import STTransformerAksanRegressor
from model.config_model import (
    SVM_CONFIG, RBFNN_CONFIG, SVR_CONFIG, RF_CONFIG, GB_CONFIG, MLP_CONFIG, GRU_CONFIG, LSTM_CONFIG, CNN_LSTM_CONFIG, CNN_GRU_CONFIG, CNN_BILSTM_ATTENTION_CONFIG, TCN_CONFIG, TRANSFORMER_CONFIG, SPATIO_TEMPORAL_TRANSFORMER_CONFIG, SPATIO_TEMPORAL_TRANSFORMER3_CONFIG, SPATIO_TEMPORAL_TRANSFORMER4_CONFIG, SPATIO_TEMPORAL_TRANSFORMER5_CONFIG, SPATIO_TEMPORAL_TRANSFORMER6_CONFIG, ST_TRANSFORMER_AKSAN_CONFIG, CNN_ST_TRANSFORMER_CONFIG, CV_CONFIG, FEATURE_CONFIG, CHANNEL_CONFIG, PARTICIPANT_CONFIG, DATABASE_CONFIG, AUGMENTATION_CONFIG, GLOBAL_RANDOM_STATE, MODEL_TYPE, RUN_GRID_SEARCH, USE_PRECOMPUTED_FEATURES, DEV_MODE, DEV_FRACTION, DEV_CV_FOLDS, COMPUTE_FEATURE_IMPORTANCE,
    COMPUTE_PERMUTATION_CHANNEL, COMPUTE_PERMUTATION_FEATURE, COMPUTE_PERMUTATION_INDIVIDUAL, COMPUTE_DEEPSHAP, RUN_MODALITY_ABLATION
)
from sklearn.metrics import (
    classification_report, accuracy_score, confusion_matrix,
    mean_absolute_error, mean_squared_error, r2_score
)
from sklearn.utils.class_weight import compute_sample_weight, compute_class_weight



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
    elif model_type == "cnn_gru":
        print(f"Initializing CNN-GRU Regressor with config: {CNN_GRU_CONFIG}")
        from model.model_archs.cnn_gru import CNNGRURegressor
        return CNNGRURegressor(**CNN_GRU_CONFIG)
    elif model_type == "cnn_bilstm_attention":
        print(f"Initializing CNN-BiLSTM-Attention Regressor with config: {CNN_BILSTM_ATTENTION_CONFIG}")
        from model.model_archs.cnn_bilstm_attention import CNNBiLSTMAttentionRegressor
        return CNNBiLSTMAttentionRegressor(**CNN_BILSTM_ATTENTION_CONFIG)
    elif model_type == "tcn":
        print(f"Initializing TCN Regressor with config: {TCN_CONFIG}")
        from model.model_archs.tcn import TCNRegressor
        return TCNRegressor(**TCN_CONFIG)
    elif model_type == "transformer":
        print(f"Initializing Transformer Regressor with config: {TRANSFORMER_CONFIG}")
        from model.model_archs.transformer import TimeSeriesTransformerRegressor
        return TimeSeriesTransformerRegressor(**TRANSFORMER_CONFIG)
    elif model_type == "spatio_temporal_transformer":
        print(f"Initializing Spatio-Temporal Transformer Regressor with config: {SPATIO_TEMPORAL_TRANSFORMER_CONFIG}")
        from model.model_archs.spatio_temporal_transformer import SpatioTemporalTransformerRegressor
        return SpatioTemporalTransformerRegressor(**SPATIO_TEMPORAL_TRANSFORMER_CONFIG)
    elif model_type == "spatio_temporal_transformer2":
        print(f"Initializing Interleaved Spatio-Temporal Transformer Regressor with config: {SPATIO_TEMPORAL_TRANSFORMER_CONFIG}")
        from model.model_archs.spatio_temporal_transformer2 import SpatioTemporalTransformerRegressor2
        return SpatioTemporalTransformerRegressor2(**SPATIO_TEMPORAL_TRANSFORMER_CONFIG)
    elif model_type == "spatio_temporal_transformer3":
        print(f"Initializing Modality-Grouped Spatio-Temporal Transformer Regressor with config: {SPATIO_TEMPORAL_TRANSFORMER3_CONFIG}")
        from model.model_archs.spatio_temporal_transformer3 import SpatioTemporalTransformerRegressor3
        return SpatioTemporalTransformerRegressor3(**SPATIO_TEMPORAL_TRANSFORMER3_CONFIG)
    elif model_type == "spatio_temporal_transformer4":
        print(f"Initializing Modality-Isolated Spatio-Temporal Transformer Regressor with config: {SPATIO_TEMPORAL_TRANSFORMER4_CONFIG}")
        from model.model_archs.spatio_temporal_transformer4 import SpatioTemporalTransformerRegressor4
        return SpatioTemporalTransformerRegressor4(**SPATIO_TEMPORAL_TRANSFORMER4_CONFIG)
    elif model_type == "spatio_temporal_transformer5":
        print(f"Initializing SST5 Modality-Isolated Spatio-Temporal Transformer Regressor with config: {SPATIO_TEMPORAL_TRANSFORMER5_CONFIG}")
        from model.model_archs.spatio_temporal_transformer5 import SpatioTemporalTransformerRegressor5
        return SpatioTemporalTransformerRegressor5(**SPATIO_TEMPORAL_TRANSFORMER5_CONFIG)
    elif model_type == "spatio_temporal_transformer6":
        print(f"Initializing SST6 Gated Spatio-Temporal Transformer Regressor with config: {SPATIO_TEMPORAL_TRANSFORMER6_CONFIG}")
        from model.model_archs.spatio_temporal_transformer6 import SpatioTemporalTransformerRegressor6
        return SpatioTemporalTransformerRegressor6(**SPATIO_TEMPORAL_TRANSFORMER6_CONFIG)
    elif model_type == "st_transformer_aksan":
        print(f"Initializing ST-Transformer (Aksan et al.) Regressor with config: {ST_TRANSFORMER_AKSAN_CONFIG}")
        from model.model_archs.st_transformer_aksan import STTransformerAksanRegressor
        return STTransformerAksanRegressor(**ST_TRANSFORMER_AKSAN_CONFIG)
    elif model_type == "cnn_st_transformer":
        print(f"Initializing CNN-ST-Transformer Regressor with config: {CNN_ST_TRANSFORMER_CONFIG}")
        from model.model_archs.cnn_st_transformer import CNNSTTransformerRegressor
        return CNNSTTransformerRegressor(**CNN_ST_TRANSFORMER_CONFIG)
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
        
        # Base of 10 bins per second, scaled up by the density of the dataset
        density_multiplier = max(1.0, np.sqrt(n_samples / 300.0))
        n_bins = int(np.ceil(max_dur * 10 * density_multiplier))
        # Put a sane limit so we don't create thousands of bins
        n_bins = min(500, max(10, n_bins))

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

    # --- Weight-class filter (applied before groups/df extraction) ---
    from model.data_loader import _EXCLUDED_TRUE_WEIGHTS
    if _EXCLUDED_TRUE_WEIGHTS and "weight" in df.columns:
        before = len(df)
        mask = ~df["weight"].isin(_EXCLUDED_TRUE_WEIGHTS)
        df = df[mask].reset_index(drop=True)
        n_dropped = before - len(df)
        if n_dropped > 0:
            print(f"[DataLoader] Dropped {n_dropped} segments with excluded weight classes.")

    groups = df["subject"].astype(str).values if "subject" in df.columns else None
    X, y = loader.prepare_for_ml(df, target_col="weight")
    
    # Pass along subject column for neural networks taking raw data for participant balancing
    if is_raw_segment and groups is not None:
        X['subject'] = groups
        
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
        from sklearn.model_selection import GroupKFold
        n_unique_groups = len(np.unique(groups))
        
        if n_unique_groups < 2:
            print(f"\n[WARNING] Only {n_unique_groups} participant found. 'participant' strategy is impossible.")
            print("Falling back to standard Stratified K-Fold (ignoring participant boundaries).")
            cv_strategy = 'kfold'
        else:
            if DEV_MODE:
                n_splits = min(DEV_CV_FOLDS, n_unique_groups)
            else:
                n_splits = min(n_folds, n_unique_groups)
                
            if n_splits < 2:
                print(f"\n[WARNING] n_splits={n_splits} is too low for cross-validation.")
                print("Falling back to standard Stratified K-Fold.")
                cv_strategy = 'kfold'
            else:
                gkf = GroupKFold(n_splits=n_splits)
                cv_iterator = list(gkf.split(X, y, groups))
                n_folds = len(cv_iterator)
                print(f"\nStarting {n_folds}-Fold Cross-Participant Validation...")

    if cv_strategy != 'participant':
        from sklearn.model_selection import StratifiedKFold
        skf = StratifiedKFold(n_splits=n_folds, shuffle=True, random_state=GLOBAL_RANDOM_STATE)
        strat_labels = df["weight"].astype(str) if "weight" in df.columns else None
        cv_iterator = list(skf.split(X, strat_labels))
        print(f"\nStarting {n_folds}-Fold Stratified Cross-Validation...")

    cv_metrics = []
    fold_results = []
    oof_predictions = np.zeros(len(X))
    participant_stats = []
    perm_importances_channel = []
    perm_importances_feature = []
    perm_importances_individual = []
    cv_histories = []
    cv_val_participants = []

    for fold, (train_idx, test_idx) in enumerate(cv_iterator, 1):
        if cv_strategy == 'participant':
            test_participants = np.unique(groups[test_idx])
            left_out_participants_str = ", ".join(test_participants)
            print(f"--- Fold {fold}/{n_folds} (Left out Participants: {left_out_participants_str}) ---")
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
            
        kwargs = {'sample_weight': sample_weight}
        
        # Apply Strict Cross-Participant Early Stopping if enabled
        if cv_strategy == 'participant' and CV_CONFIG.get('strict_val_split', False) and model_type in ['cnn_lstm', 'cnn_gru', 'cnn_bilstm_attention', 'tcn', 'transformer', 'spatio_temporal_transformer', 'spatio_temporal_transformer2', 'spatio_temporal_transformer3', 'spatio_temporal_transformer4', 'spatio_temporal_transformer5', 'spatio_temporal_transformer6', 'st_transformer_aksan', 'cnn_st_transformer', 'lstm', 'gru']:
            from sklearn.model_selection import GroupShuffleSplit
            groups_train = groups[train_idx]
            val_p = CV_CONFIG.get('strict_val_participants', 2)
            
            n_train_groups = len(np.unique(groups_train))
            if val_p >= n_train_groups:
                val_p = max(1, n_train_groups - 1)
                
            gss = GroupShuffleSplit(n_splits=1, test_size=val_p, random_state=GLOBAL_RANDOM_STATE + fold)
            inner_train_idx, inner_val_idx = next(gss.split(X_train_fold, y_train_fold, groups=groups_train))
            
            X_val_fold = X_train_fold.iloc[inner_val_idx]
            y_val_fold = y_train_fold.iloc[inner_val_idx]
            
            val_participants = np.unique(groups_train[inner_val_idx])
            print(f"    [Strict Validation] Holding out {len(val_participants)} participants for early stopping: {', '.join(val_participants)}")
            
            X_train_fold = X_train_fold.iloc[inner_train_idx]
            y_train_fold = y_train_fold.iloc[inner_train_idx]
            
            cv_val_participants.append(list(val_participants))
            
            if sample_weight is not None:
                sample_weight = sample_weight[inner_train_idx]
            kwargs['sample_weight'] = sample_weight
                
            kwargs['X_val'] = X_val_fold
            kwargs['y_val'] = y_val_fold
            
        model.fit(X_train_fold, y_train_fold, **kwargs)
        
        # If we didn't use strict val split, we might still want to track the test participants for labeling
        if not CV_CONFIG.get('strict_val_split', False) and cv_strategy == 'participant':
            cv_val_participants.append(list(test_participants))
            
        train_time = time.perf_counter() - start_train
        
        start_inf = time.perf_counter()
        metrics, fold_report = model.evaluate(X_test_fold, y_test_fold)
        inf_time = time.perf_counter() - start_inf
        
        metrics.update({
            'train_time': train_time,
            'inference_time_total': inf_time,
            'inference_time_per_sample': inf_time / len(X_test_fold)
        })
        
        # Calculate Train metrics for Generalization Gap
        train_preds = model.predict(X_train_fold)
        train_mae = mean_absolute_error(y_train_fold, train_preds)
        train_rmse = np.sqrt(mean_squared_error(y_train_fold, train_preds))
        
        metrics['Train MAE'] = train_mae
        metrics['Train RMSE'] = train_rmse
        metrics['Generalization Gap (MAE)'] = metrics['MAE'] - train_mae
        metrics['Overfit Ratio (RMSE)'] = metrics['RMSE'] / (train_rmse + 1e-8)

        print(f"    Train MAE: {train_mae:.4f} | Test MAE: {metrics['MAE']:.4f} | Gap: {metrics['Generalization Gap (MAE)']:.4f}")
        print(f"    Train RMSE: {train_rmse:.4f} | Test RMSE: {metrics['RMSE']:.4f} | Ratio: {metrics['Overfit Ratio (RMSE)']:.2f}x")
        
        cv_metrics.append(metrics)
        fold_results.append(fold_report)
        cv_histories.append(copy.deepcopy(model.loss_history))
        preds = model.predict(X_test_fold)
        oof_predictions[test_idx] = preds
        
        if COMPUTE_FEATURE_IMPORTANCE and hasattr(model, 'permutation_importance'):
            try:
                import inspect
                sig = inspect.signature(model.permutation_importance)
                if 'importance_type' in sig.parameters:
                    if COMPUTE_PERMUTATION_CHANNEL:
                        print("Computing permutation importance for fold (channel)...")
                        imp_c = model.permutation_importance(X_test_fold, y_test_fold, importance_type='channel')
                        perm_importances_channel.append(imp_c)
                    if COMPUTE_PERMUTATION_FEATURE:
                        print("Computing permutation importance for fold (feature)...")
                        imp_f = model.permutation_importance(X_test_fold, y_test_fold, importance_type='feature')
                        perm_importances_feature.append(imp_f)
                    if COMPUTE_PERMUTATION_INDIVIDUAL:
                        print("Computing permutation importance for fold (individual feature)...")
                        imp_i = model.permutation_importance(X_test_fold, y_test_fold, importance_type='individual_feature')
                        perm_importances_individual.append(imp_i)
                else:
                    if COMPUTE_PERMUTATION_CHANNEL:
                        print("Computing permutation importance for fold...")
                        imp = model.permutation_importance(X_test_fold, y_test_fold)
                        perm_importances_channel.append(imp)
            except Exception as e:
                print(f"[WARN] Permutation importance computation failed: {e}")
                
        if cv_strategy == 'participant':
            fold_groups = groups[test_idx]
            for p in test_participants:
                p_mask = (fold_groups == p)
                p_y_true = y_test_fold[p_mask]
                p_preds = preds[p_mask]
                participant_stats.append({
                    'Participant': p,
                    'MAE': mean_absolute_error(p_y_true, p_preds),
                    'RMSE': np.sqrt(mean_squared_error(p_y_true, p_preds)),
                    'Samples': sum(p_mask)
                })

    if DEV_MODE:
        print("\n[DEV MODE] Skipping final full-dataset training since model parameters are not needed.")
    else:
        print("\nTraining final model on full dataset for saving...")
        model = initialize_model(model_type)
        sample_weight = None
        if model_type == "svr" and SVR_CONFIG.get('balance_weights', False):
            sample_weight = compute_sample_weight(class_weight='balanced', y=y)
            
        kwargs = {'sample_weight': sample_weight}
        
        # Apply Strict Cross-Participant Early Stopping for the final model
        if cv_strategy == 'participant' and CV_CONFIG.get('strict_val_split', False) and model_type in ['cnn_lstm', 'cnn_gru', 'cnn_bilstm_attention', 'tcn', 'transformer', 'spatio_temporal_transformer', 'spatio_temporal_transformer2', 'spatio_temporal_transformer3', 'spatio_temporal_transformer4', 'spatio_temporal_transformer5', 'spatio_temporal_transformer6', 'st_transformer_aksan', 'cnn_st_transformer', 'lstm', 'gru']:
            from sklearn.model_selection import GroupShuffleSplit
            val_p = CV_CONFIG.get('strict_val_participants', 2)
            
            n_train_groups = len(np.unique(groups))
            if val_p >= n_train_groups:
                val_p = max(1, n_train_groups - 1)
                
            gss = GroupShuffleSplit(n_splits=1, test_size=val_p, random_state=GLOBAL_RANDOM_STATE)
            inner_train_idx, inner_val_idx = next(gss.split(X, y, groups=groups))
            
            X_val_final = X.iloc[inner_val_idx]
            y_val_final = y.iloc[inner_val_idx]
            
            val_participants = np.unique(groups[inner_val_idx])
            print(f"    [Strict Validation] Holding out {len(val_participants)} participants from final dataset for early stopping: {', '.join(val_participants)}")
            
            X_train_final = X.iloc[inner_train_idx]
            y_train_final = y.iloc[inner_train_idx]
            
            if sample_weight is not None:
                kwargs['sample_weight'] = sample_weight[inner_train_idx]
                
            kwargs['X_val'] = X_val_final
            kwargs['y_val'] = y_val_final
            
            model.fit(X_train_final, y_train_final, **kwargs)
        else:
            model.fit(X, y, **kwargs)

    avg_permutation_importance = {}
    if perm_importances_channel:
        avg_perm_imp_channel = {}
        for k in perm_importances_channel[0].keys():
            avg_perm_imp_channel[k] = np.mean([pi[k] for pi in perm_importances_channel])
        avg_permutation_importance['channel'] = avg_perm_imp_channel
        
    if perm_importances_feature:
        avg_perm_imp_feature = {}
        for k in perm_importances_feature[0].keys():
            avg_perm_imp_feature[k] = np.mean([pi[k] for pi in perm_importances_feature])
        avg_permutation_importance['feature'] = avg_perm_imp_feature
        
    if perm_importances_individual:
        avg_perm_imp_individual = {}
        for k in perm_importances_individual[0].keys():
            avg_perm_imp_individual[k] = np.mean([pi[k] for pi in perm_importances_individual])
        avg_permutation_importance['individual'] = avg_perm_imp_individual
        
    if not avg_permutation_importance:
        avg_permutation_importance = None

    return model, cv_metrics, oof_predictions, participant_stats, n_folds, avg_permutation_importance, cv_histories, cv_val_participants
    # NOTE: cv_metrics is the raw list of per-fold metric dicts, needed by run_data_exporter


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
    
    perm_imp = {}
    if COMPUTE_FEATURE_IMPORTANCE and hasattr(model, 'permutation_importance'):
        try:
            import inspect
            sig = inspect.signature(model.permutation_importance)
            if 'importance_type' in sig.parameters:
                if COMPUTE_PERMUTATION_CHANNEL:
                    print("Computing permutation importance for test set (channel)...")
                    perm_imp['channel'] = model.permutation_importance(X_test, y_test, importance_type='channel')
                if COMPUTE_PERMUTATION_FEATURE:
                    print("Computing permutation importance for test set (feature)...")
                    perm_imp['feature'] = model.permutation_importance(X_test, y_test, importance_type='feature')
                if COMPUTE_PERMUTATION_INDIVIDUAL:
                    print("Computing permutation importance for test set (individual feature)...")
                    perm_imp['individual'] = model.permutation_importance(X_test, y_test, importance_type='individual_feature')
            else:
                if COMPUTE_PERMUTATION_CHANNEL:
                    print("Computing permutation importance for test set...")
                    perm_imp['channel'] = model.permutation_importance(X_test, y_test)
        except Exception as e:
            print(f"[WARN] Permutation importance computation failed: {e}")
            
    return model, metrics, report_str, X_train, X_test, y_train, y_test, y_pred, perm_imp


def save_basic_artifacts(model, run_dir, model_type, use_cv, y, y_test, y_pred, oof_predictions, 
                         all_histories=None, val_participants=None):
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
        if all_histories and len(all_histories) > 0:
            plotting_utils.plot_training_loss(all_histories, loss_plot_path, 
                                              model_name=model_type.upper().replace("_", " "),
                                              val_participants=val_participants)
        else:
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
                                                    model_name=model_type.upper(), min_count=max(2, int(len(target_y)*0.005)))

    if is_raw_segment and 'segment_duration_sec' in df.columns:
        target_y = y.values if use_cv else y_test.values
        target_preds = oof_predictions if use_cv else y_pred
        durations = df.loc[X.index if use_cv else X_test.index, 'segment_duration_sec'].values
        
        per_duration_stats = calculate_per_duration_metrics(target_y, target_preds, durations)
        if per_duration_stats:
            plotting_utils.plot_seqlen_performance(per_duration_stats, run_dir / "seqlen_performance_plot.png", 
                                                    model_name=model_type.upper().replace("_", "-"), min_count=max(2, int(len(target_y)*0.005)))
            
    return per_seqlen_stats, per_duration_stats


    return per_seqlen_stats, per_duration_stats



def main():
    import argparse
    parser = argparse.ArgumentParser(description="Run the model training and evaluation pipeline.")
    parser.add_argument("--modality", type=str, choices=["all", "emg_only", "imu_only"], default="all",
                        help="Select which sensor modalities to use. Useful for macro-ablation.")
    parser.add_argument("--run_ablation", action="store_true", default=None,
                        help="Run the complete modality ablation loop (all, emg_only, imu_only).")
    args, unknown = parser.parse_known_args()
    
    # Determine whether to run full macro-ablation loop
    run_ablation = RUN_MODALITY_ABLATION if args.run_ablation is None else args.run_ablation
    
    if run_ablation:
        modalities = ["all", "emg_only", "imu_only"]
        print(f"\n[MACRO ABLATION LOOP] Enabled. Will run sequentially for: {modalities}")
    else:
        modalities = [args.modality]

    # Define paths
    base_dir = Path(__file__).parent.parent
    
    if RUN_GRID_SEARCH:
        print(f"\n--- GRID SEARCH ENABLED ---")
        print(f"Launching hyperparameter sweep for {MODEL_TYPE.upper()}...")
        sweep_script = base_dir / "model" / "model_archs" / "model_hp_opti" / f"sweep_{MODEL_TYPE.lower()}.py"
        if sweep_script.exists():
            import subprocess
            # Forward any command line arguments to the sweep script (e.g. --mode architecture --n_iter 150)
            subprocess.run([sys.executable, str(sweep_script)] + sys.argv[1:])
        else:
            print(f"Error: Grid search script {sweep_script.name} not found.")
        return
        
    # 1. Setup (Run directory created once at the root)
    run_dir, timestamp = setup_run_dir(base_dir)
    print(f"Results will be saved to: {run_dir}")
    
    # Keep a deep copy of the original CHANNEL_CONFIG to restore at each iteration
    orig_channel_config = copy.deepcopy(CHANNEL_CONFIG)
    
    for active_modality in modalities:
        print("\n" + "="*80)
        print(f"  RUNNING PIPELINE FOR MODALITY CONFIGURATION: {active_modality.upper()}")
        print("="*80 + "\n")
        
        # Reset to original channels
        CHANNEL_CONFIG['emg_channels'] = copy.deepcopy(orig_channel_config['emg_channels'])
        CHANNEL_CONFIG['imu_channels'] = copy.deepcopy(orig_channel_config['imu_channels'])
        
        # Apply overrides
        if active_modality == "emg_only":
            print("\n[MACRO ABLATION] Overriding config: Disabling all IMU channels.")
            for k in CHANNEL_CONFIG['imu_channels']:
                CHANNEL_CONFIG['imu_channels'][k] = False
        elif active_modality == "imu_only":
            print("\n[MACRO ABLATION] Overriding config: Disabling all EMG channels.")
            for k in CHANNEL_CONFIG['emg_channels']:
                CHANNEL_CONFIG['emg_channels'][k] = False

        if run_ablation:
            run_dir_active = run_dir / active_modality
            run_dir_active.mkdir(parents=True, exist_ok=True)
        else:
            run_dir_active = run_dir

        model_type = MODEL_TYPE.lower()
        is_sequence = model_type in ["gru", "lstm", "transformer", "spatio_temporal_transformer", "spatio_temporal_transformer2", "spatio_temporal_transformer3", "spatio_temporal_transformer4", "spatio_temporal_transformer5", "spatio_temporal_transformer6", "st_transformer_aksan"]
        is_raw_segment = model_type in ["cnn_lstm", "cnn_gru", "cnn_bilstm_attention", "tcn", "cnn_st_transformer"]
        
        # 2. Data Loading
        loader = DataLoader()
        h5_paths = [p for p in DATABASE_CONFIG['segments_dir'].glob("*.h5") if not p.name.startswith("._")]
        if not h5_paths:
            print(f"No HDF5 segment files found in {DATABASE_CONFIG['segments_dir']}.")
            continue

        X, y, groups, df = load_and_prepare_data(loader, h5_paths, model_type, is_raw_segment, is_sequence, USE_PRECOMPUTED_FEATURES)
        if X is None: 
            print("Data extraction failed or produced an empty DataFrame.")
            continue
            
        if DEV_MODE:
            print(f"\n[DEV MODE] Subsampling dataset to {DEV_FRACTION*100}% for rapid testing...")
            stratify_key = None
            if groups is not None and "weight" in df.columns:
                stratify_key = [f"{g}_{w}" for g, w in zip(groups, df["weight"])]
            elif groups is not None:
                stratify_key = groups
            elif "weight" in df.columns:
                stratify_key = df["weight"]
                
            try:
                _, X, _, y, _, groups, _, df = train_test_split(
                    X, y, groups, df, 
                    test_size=DEV_FRACTION, 
                    random_state=GLOBAL_RANDOM_STATE, 
                    stratify=stratify_key
                )
            except ValueError as e:
                print(f"[WARN] Stratified split failed ({e}), falling back to random split.")
                _, X, _, y, _, groups, _, df = train_test_split(
                    X, y, groups, df, 
                    test_size=DEV_FRACTION, 
                    random_state=GLOBAL_RANDOM_STATE
                )
        
        # 3. Summary
        print_data_summary(X, y, is_raw_segment, is_sequence)
        # 4. Training & Evaluation
        if CV_CONFIG.get('use_cross_val', True):
            model, cv_metrics_list, oof_predictions, participant_stats, actual_n_folds, perm_imp, all_histories, cv_val_p = execute_cross_validation(
                X, y, groups, df, model_type, CV_CONFIG.get('strategy', 'kfold'), CV_CONFIG.get('n_folds', 5)
            )
            avg_metrics = {k: np.mean([m[k] for m in cv_metrics_list]) for k in cv_metrics_list[0].keys()}
            std_metrics = {k: np.std([m[k] for m in cv_metrics_list]) for k in cv_metrics_list[0].keys()}
            metrics, X_train, X_test, y_train, y_test, y_pred = {}, None, None, None, None, None
        else:
            actual_n_folds = 0
            cv_metrics_list, cv_val_p = None, None
            model, metrics, report_str, X_train, X_test, y_train, y_test, y_pred, perm_imp = execute_single_split(
                X, y, df, model_type, CV_CONFIG.get('train_test_split', 0.2)
            )
            avg_metrics, std_metrics, oof_predictions, participant_stats, all_histories = None, None, None, None, None

        # 5. Artifacts & Specialized Plots
        save_basic_artifacts(model, run_dir_active, model_type, CV_CONFIG.get('use_cross_val', True), y, y_test, y_pred, oof_predictions, 
                             all_histories=all_histories, val_participants=cv_val_p if CV_CONFIG.get('use_cross_val', True) else None)
        per_seqlen, per_dur = save_extended_plots(model, run_dir_active, model_type, CV_CONFIG.get('use_cross_val', True), df, X, X_test, y, y_test, y_pred, 
                                                 oof_predictions, is_raw_segment, is_sequence, groups, participant_stats)
                                                 
        if DEV_MODE and participant_stats:
            print("\n[DEV MODE] Instant Participant Breakdown:")
            import pandas as pd
            stats_df = pd.DataFrame(participant_stats)
            agg_stats = stats_df.groupby('Participant').agg({'MAE': 'mean', 'RMSE': 'mean', 'Samples': 'sum'})
            print(agg_stats.to_string())
        
        if perm_imp:
            if 'channel' in perm_imp and perm_imp['channel']:
                plotting_utils.plot_permutation_importance(perm_imp['channel'], run_dir_active / "permutation_importance_channel.png", model_name=model_type.upper())
            if 'feature' in perm_imp and perm_imp['feature']:
                plotting_utils.plot_permutation_importance(perm_imp['feature'], run_dir_active / "permutation_importance_feature.png", model_name=model_type.upper())
        
        # 6. Final Report (Moved before DeepSHAP to safeguard against OOM crashes)
        generator = ReportGenerator(run_dir_active, timestamp)
        generator.generate(h5_paths, X, CV_CONFIG.get('use_cross_val', True), CV_CONFIG.get('strategy', 'kfold'), 
                           actual_n_folds, model, model_type, X_test, X_train, y, y_train, y_test, 
                           is_raw_segment, avg_metrics, std_metrics, metrics, participant_stats, 
                           per_seqlen, per_dur, oof_predictions, y_pred, perm_imp,
                           ablation_modality=active_modality, groups=groups)
                           
        # 6b. Save machine-readable run data (JSON)
        from model.run_data_exporter import save_run_data
        save_run_data(
            run_dir=run_dir_active,
            timestamp=timestamp,
            model_type=model_type,
            model=model,
            h5_paths=h5_paths,
            X=X, y=y, groups=groups, df=df,
            use_cv=CV_CONFIG.get('use_cross_val', True),
            cv_strategy=CV_CONFIG.get('strategy', 'kfold'),
            n_folds=actual_n_folds,
            avg_metrics=avg_metrics,
            std_metrics=std_metrics,
            metrics=metrics,
            cv_metrics=cv_metrics_list,
            participant_stats=participant_stats,
            per_seqlen_stats=per_seqlen,
            per_duration_stats=per_dur,
            oof_predictions=oof_predictions,
            y_pred=y_pred,
            y_test=y_test,
            perm_imp=perm_imp,
            all_histories=all_histories,
            cv_val_participants=cv_val_p if CV_CONFIG.get('use_cross_val', True) else None,
            ablation_modality=active_modality,
            is_raw_segment=is_raw_segment,
            is_sequence=is_sequence,
        )

        # 7. Automated DeepSHAP Analysis (High-Fidelity)
        if model_type in ["cnn_lstm", "spatio_temporal_transformer", "spatio_temporal_transformer2", "spatio_temporal_transformer3", "spatio_temporal_transformer4", "spatio_temporal_transformer5", "spatio_temporal_transformer6", "st_transformer_aksan", "cnn_st_transformer"]:
            print("\n" + "-"*50)
            print("LAUNCHING AUTOMATED DEEPSHAP ANALYSIS")
            print("-"*50)
            try:
                # If we used CV, the final model is trained on the whole dataset X.
                # We split it here for the purposes of SHAP analysis.
                if CV_CONFIG.get('use_cross_val', True):
                    strat = df["weight"].astype(str) if "weight" in df.columns else None
                    X_train_shap, X_test_shap = train_test_split(X, test_size=0.2, random_state=42, stratify=strat)
                else:
                    X_train_shap, X_test_shap = X_train, X_test
                    
                if COMPUTE_FEATURE_IMPORTANCE and COMPUTE_DEEPSHAP:
                    n_bg = 20 if DEV_MODE else 100
                    n_exp = 10 if DEV_MODE else 50
                    print(f"[DeepSHAP] Starting attribution with n_bg={n_bg}, n_exp={n_exp} (DEV_MODE={DEV_MODE})...")
                    deepshap_analysis.run_deep_shap_analysis(
                        model, X_train_shap, X_test_shap, run_dir_active, 
                        n_bg=n_bg, n_exp=n_exp
                    )
                    # Re-save JSON to include DeepSHAP importance values
                    save_run_data(
                        run_dir=run_dir_active, timestamp=timestamp, model_type=model_type, model=model,
                        h5_paths=h5_paths, X=X, y=y, groups=groups, df=df,
                        use_cv=CV_CONFIG.get('use_cross_val', True),
                        cv_strategy=CV_CONFIG.get('strategy', 'kfold'),
                        n_folds=actual_n_folds, avg_metrics=avg_metrics, std_metrics=std_metrics,
                        metrics=metrics, cv_metrics=cv_metrics_list, participant_stats=participant_stats,
                        per_seqlen_stats=per_seqlen, per_duration_stats=per_dur,
                        oof_predictions=oof_predictions, y_pred=y_pred, y_test=y_test,
                        perm_imp=perm_imp, all_histories=all_histories,
                        cv_val_participants=cv_val_p if CV_CONFIG.get('use_cross_val', True) else None,
                        ablation_modality=active_modality, is_raw_segment=is_raw_segment, is_sequence=is_sequence,
                    )
                else:
                    print("Skipping DeepSHAP analysis (COMPUTE_FEATURE_IMPORTANCE or COMPUTE_DEEPSHAP is False)...")
            except Exception as e:
                print(f"[WARN] Automated DeepSHAP analysis failed: {e}")

    # Master consolidation of run data if ablation was run
    if run_ablation:
        master_data = {}
        for mod in modalities:
            mod_json_path = run_dir / mod / "run_data.json"
            if mod_json_path.exists():
                try:
                    with open(mod_json_path, "r") as f:
                        master_data[mod] = json.load(f)
                except Exception as e:
                    print(f"[WARN] Failed to load {mod_json_path} for master compile: {e}")
        
        if master_data:
            master_json_path = run_dir / "run_data.json"
            with open(master_json_path, "w") as f:
                json.dump(master_data, f, indent=2)
            print(f"\n[Master Serializer] Consolidated modality ablation data saved to {master_json_path}")


if __name__ == "__main__":
    main()
