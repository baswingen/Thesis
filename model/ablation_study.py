import os
import sys
import copy
import numpy as np
import pandas as pd
from pathlib import Path
from datetime import datetime

# Add project root to sys.path so 'model' package can be found
sys.path.append(str(Path(__file__).parent.parent))

from model.data_loader import DataLoader
from model.run_model import initialize_model, MODEL_TYPE
from model.config_model import CV_CONFIG

from sklearn.model_selection import StratifiedKFold, train_test_split
from sklearn.metrics import accuracy_score

# Configuration for Ablation Study
MODALITIES = ['both', 'emg', 'imu']

def filter_features(df: pd.DataFrame, modality: str, is_sequence: bool) -> pd.DataFrame:
    """
    Filters features in the dataframe based on the modality: 'both', 'emg', or 'imu'.
    """
    df_filtered = df.copy(deep=True)
    
    if modality == 'both':
        return df_filtered
        
    if is_sequence:
        # Filter keys within the sequence dictionaries
        for i in range(len(df_filtered)):
            new_seq = []
            for seq_dict in df_filtered.at[i, 'sequence_dicts']:
                if modality == 'emg':
                    filtered_dict = {k: v for k, v in seq_dict.items() if '_IMU_' not in k}
                elif modality == 'imu':
                    filtered_dict = {k: v for k, v in seq_dict.items() if '_EMG_' not in k}
                new_seq.append(filtered_dict)
            df_filtered.at[i, 'sequence_dicts'] = new_seq
    else:
        # Filter dataframe columns directly
        cols_to_drop = []
        for col in df_filtered.columns:
            if modality == 'emg' and '_IMU_' in col:
                cols_to_drop.append(col)
            elif modality == 'imu' and '_EMG_' in col:
                cols_to_drop.append(col)
        df_filtered.drop(columns=cols_to_drop, inplace=True)
        
    return df_filtered

def run_evaluation(X, y, model_type, is_regression, strat_labels=None, use_cv=False, n_folds=5):
    if use_cv:
        skf = StratifiedKFold(n_splits=n_folds, shuffle=True, random_state=42)
        cv_metrics = []
        
        for fold, (train_idx, test_idx) in enumerate(skf.split(X, strat_labels), 1):
            X_train_fold, X_test_fold = X.iloc[train_idx].copy(), X.iloc[test_idx].copy()
            y_train_fold, y_test_fold = y.iloc[train_idx].copy(), y.iloc[test_idx].copy()
            
            model = initialize_model(model_type)
            if model is None: return None
            
            model.fit(X_train_fold, y_train_fold)
            
            if is_regression:
                metrics, _ = model.evaluate(X_test_fold, y_test_fold)
                cv_metrics.append(metrics)
            else:
                y_pred_fold = model.predict(X_test_fold)
                acc = accuracy_score(y_test_fold, y_pred_fold)
                cv_metrics.append({'accuracy': acc})
                
        if is_regression:
            avg_metrics = {k: np.mean([m[k] for m in cv_metrics]) for k in cv_metrics[0].keys()}
            return avg_metrics
        else:
            avg_acc = np.mean([m['accuracy'] for m in cv_metrics])
            return {'accuracy': avg_acc}
            
    else:
        X_train, X_test, y_train, y_test = train_test_split(
            X, y, test_size=0.2, random_state=7, stratify=strat_labels
        )
        
        model = initialize_model(model_type)
        if model is None: return None
        
        model.fit(X_train, y_train)
        
        if is_regression:
            metrics, _ = model.evaluate(X_test, y_test)
            return metrics
        else:
            y_pred = model.predict(X_test)
            accuracy = accuracy_score(y_test, y_pred)
            return {'accuracy': accuracy}

def main():
    base_dir = Path(__file__).parent.parent
    segments_dir = base_dir / "database" / "segments"
    results_dir = base_dir / "model" / "model_results"
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = results_dir / f"ablation_{MODEL_TYPE}_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Results will be saved to: {run_dir}")
    
    h5_paths = list(segments_dir.glob("*.h5"))
    if not h5_paths:
        print(f"No HDF5 segment files found in {segments_dir}.")
        return

    loader = DataLoader()
    print("Extracting features from HDF5 files...")
    
    model_type = MODEL_TYPE.lower()
    is_sequence = model_type in ["gru", "lstm", "cnn_lstm", "transformer"]
    is_regression = model_type in ["svr", "rf", "gb", "mlp", "gru", "lstm", "cnn_lstm", "transformer"]
    
    if is_sequence:
        if model_type == "gru":
            from model.config_model import GRU_CONFIG as conf
        elif model_type == "lstm":
            from model.config_model import LSTM_CONFIG as conf
        elif model_type == "cnn_lstm":
            from model.config_model import CNN_LSTM_CONFIG as conf
        else:
            from model.config_model import TRANSFORMER_CONFIG as conf
        df_raw = loader.load_and_extract_features(
            h5_paths, 
            window_size_sec=conf.get('window_size_sec', 0.25),
            window_step_sec=conf.get('window_step_sec', 0.1)
        )
    else:
        df_raw = loader.load_and_extract_features(h5_paths)
        
    if df_raw.empty:
        print("Data extraction failed or produced an empty DataFrame.")
        return

    target_col = "weight" if is_regression else "label"
    use_cv = CV_CONFIG.get('use_cross_val', False)
    n_folds = CV_CONFIG.get('n_folds', 5)
    
    results = {}

    for modality in MODALITIES:
        print(f"\n{'='*50}")
        print(f"Running Ablation Study for Modality: {modality.upper()}")
        print(f"{'='*50}")
        
        # Filter features based on modality
        df_filtered = filter_features(df_raw, modality, is_sequence)
        
        X, y = loader.prepare_for_ml(df_filtered, target_col=target_col)
        strat_labels = df_filtered["label"] if "label" in df_filtered.columns else None
        
        if is_sequence:
            # Check length of the first sequence's first element features to show feature count
            num_features = len(X.iloc[0]['sequence_dicts'][0].keys())
            print(f"Feature matrix (X) shape: {X.shape}, Features per timestep: {num_features}")
        else:
            print(f"Feature matrix (X) shape: {X.shape}")
        
        # Run Evaluation
        metrics = run_evaluation(X, y, model_type, is_regression, strat_labels, use_cv, n_folds)
        results[modality] = metrics
        
        print(f"\nResults for {modality.upper()}:")
        if metrics is not None:
            for k, v in metrics.items():
                print(f"  {k}: {v:.4f}")
        else:
            print("  Model failed to initialize.")

    # Save summary report
    report_file = run_dir / "ablation_report.txt"
    with open(report_file, "w") as f:
        f.write("=" * 55 + "\n")
        f.write("ABLATION STUDY REPORT\n")
        f.write(f"Model: {model_type.upper()}\n")
        f.write(f"Run Timestamp: {timestamp}\n")
        f.write("=" * 55 + "\n\n")
        
        f.write(f"Evaluation Mode: {'Cross-Validation' if use_cv else 'Train/Test Split'}\n\n")
        
        for modality in MODALITIES:
            f.write(f"--- Modality: {modality.upper()} ---\n")
            metrics = results[modality]
            if metrics is not None:
                for k, v in metrics.items():
                    f.write(f"{k}: {v:.4f}\n")
            else:
                f.write("Model failed to initialize.\n")
            f.write("\n")
            
    print(f"\nAblation study completed. Report saved to {report_file}")

if __name__ == "__main__":
    main()
