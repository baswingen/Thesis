import os
import sys
import argparse
from pathlib import Path
from datetime import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Add project root to sys.path
sys.path.append(str(Path(__file__).resolve().parent.parent.parent))

from model.data_loader import DataLoader
from model.model_archs.cnn_lstm import CNNLSTMRegressor
from model.config_model import CNN_LSTM_ABLATION_CONFIG, CHANNEL_CONFIG

from sklearn.model_selection import train_test_split, StratifiedKFold, KFold

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def slice_channels(df: pd.DataFrame, all_channels: list, subset_channels: list) -> pd.DataFrame:
    """Subset the 'raw_segment' numpy arrays in the DataFrame to keep only requested channels."""
    indices = [all_channels.index(c) for c in subset_channels if c in all_channels]
    df_sliced = df.copy(deep=True)
    df_sliced['raw_segment'] = df_sliced['raw_segment'].apply(lambda x: x[:, indices])
    return df_sliced


def run_evaluation(df_subset: pd.DataFrame, loader: DataLoader, target_col: str = "weight", n_splits: int = 3, random_state: int = 42):
    """
    Train and evaluate the CNN-LSTM using K-Fold cross-validation.
    Returns averaged metrics dict with MAE, MSE, RMSE, R2.
    """
    X, y = loader.prepare_for_ml(df_subset, target_col=target_col)
    
    # Cross-Validation Split
    strat_labels = df_subset["label"] if "label" in df_subset.columns else None
    
    if strat_labels is not None:
        cv = StratifiedKFold(n_splits=n_splits, shuffle=True, random_state=random_state)
        splits = cv.split(X, strat_labels)
    else:
        cv = KFold(n_splits=n_splits, shuffle=True, random_state=random_state)
        splits = cv.split(X)
        
    all_metrics = []
    
    for train_idx, test_idx in splits:
        X_train = X.iloc[train_idx] if hasattr(X, "iloc") else X[train_idx]
        X_test = X.iloc[test_idx] if hasattr(X, "iloc") else X[test_idx]
        y_train = y.iloc[train_idx] if hasattr(y, "iloc") else y[train_idx]
        y_test = y.iloc[test_idx] if hasattr(y, "iloc") else y[test_idx]
        
        # Note: CNNLSTMRegressor handles the internal validation split automatically
        model = CNNLSTMRegressor(**CNN_LSTM_ABLATION_CONFIG)
        model.fit(X_train, y_train)
        
        metrics, _ = model.evaluate(X_test, y_test)
        all_metrics.append(metrics)
        
    # Average the metrics across all folds
    avg_metrics = {}
    for k in all_metrics[0].keys():
        vals = [m[k] for m in all_metrics if pd.notna(m.get(k, np.nan))]
        avg_metrics[k] = np.mean(vals) if vals else np.nan
        
    return avg_metrics


def plot_sbs_history(sbs_history: list, run_dir: Path, ref_metrics: dict = None):
    """Generates a performance plot showing RMSE and MAE across channel removals."""
    steps = [h['step'] for h in sbs_history]
    rmses = [h['rmse'] for h in sbs_history]
    maes = [h['mae'] for h in sbs_history]
    dropped = [h['dropped_channel'] for h in sbs_history]
    n_remaining = [len(h['remaining_channels']) for h in sbs_history]
    
    fig, ax1 = plt.subplots(figsize=(14, 7))
    
    # Plot RMSE on primary y-axis
    color = '#1f77b4'
    ax1.plot(n_remaining, rmses, marker='o', linestyle='-', linewidth=2, markersize=8, color=color, label='RMSE')
    ax1.set_xlabel("Number of Remaining Channels", fontsize=14, labelpad=10)
    ax1.set_ylabel("Root Mean Squared Error (RMSE)", fontsize=14, labelpad=10, color=color)
    ax1.tick_params(axis='y', labelcolor=color)
    
    # Annotate dropped channels
    for i in range(1, len(sbs_history)):
        ax1.annotate(
            f"-{dropped[i]}",
            (n_remaining[i], rmses[i]),
            textcoords="offset points",
            xytext=(0, 15),
            ha='center',
            fontsize=9,
            rotation=60
        )
        
    # Plot MAE on secondary y-axis
    ax2 = ax1.twinx()
    color2 = '#ff7f0e'
    ax2.plot(n_remaining, maes, marker='s', linestyle='--', linewidth=2, markersize=8, color=color2, label='MAE')
    ax2.set_ylabel("Mean Absolute Error (MAE)", fontsize=14, labelpad=10, color=color2)
    ax2.tick_params(axis='y', labelcolor=color2)
    
    # Invert x-axis so sequence progresses from many channels (left) to few (right)
    ax1.invert_xaxis()
    
    # Setup styling
    plt.title("SBS Profile: CNN-LSTM Performance vs. Remaining Channels", fontsize=16, fontweight='bold', pad=20)
    ax1.grid(True, linestyle='--', alpha=0.6)
    
    if ref_metrics:
        if 'EMG Only' in ref_metrics:
            ax1.axhline(ref_metrics['EMG Only']['RMSE'], color='green', linestyle='-.', alpha=0.7, label='EMG Only RMSE')
            ax2.axhline(ref_metrics['EMG Only']['MAE'], color='#d62728', linestyle='-.', alpha=0.7, label='EMG Only MAE')
        if 'IMU Only' in ref_metrics:
            ax1.axhline(ref_metrics['IMU Only']['RMSE'], color='brown', linestyle=':', alpha=0.7, label='IMU Only RMSE')
            ax2.axhline(ref_metrics['IMU Only']['MAE'], color='#e377c2', linestyle=':', alpha=0.7, label='IMU Only MAE')

    # Highlight the best configuration visually (Lowest RMSE)
    best_step = np.argmin(rmses)
    ax1.scatter([n_remaining[best_step]], [rmses[best_step]], color='red', s=150, zorder=5, label='Best RMSE Subset')
    
    # Add legends
    lines_1, labels_1 = ax1.get_legend_handles_labels()
    lines_2, labels_2 = ax2.get_legend_handles_labels()
    ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc='upper right')
    
    plt.tight_layout()
    plot_path = run_dir / "sbs_performance_plot.png"
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close()
    return plot_path


def write_report(sbs_history: list, run_dir: Path, ref_metrics: dict = None):
    report_file = run_dir / "ablation_cnn_lstm_report.txt"
    with open(report_file, "w") as f:
        f.write("=" * 60 + "\n")
        f.write("CNN-LSTM ABLATION STUDY REPORT\n")
        f.write("Strategy: Sequential Backward Selection (SBS)\n")
        f.write(f"Run Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write("=" * 60 + "\n\n")
        
        f.write("--- TRAINING CONFIGURATION ---\n")
        f.write(f"CNN Filters: {CNN_LSTM_ABLATION_CONFIG['cnn_filters']}\n")
        f.write(f"LSTM Hidden: {CNN_LSTM_ABLATION_CONFIG['lstm_hidden_size']}\n")
        f.write(f"Batch Size : {CNN_LSTM_ABLATION_CONFIG['batch_size']}\n")
        f.write(f"Max Epochs : {CNN_LSTM_ABLATION_CONFIG['epochs']}\n\n")
        
        if ref_metrics:
            f.write("--- REFERENCE BASELINES ---\n")
            f.write(f"{'Configuration':<20} | {'RMSE':<8} | {'MAE':<8} | {'R2':<8}\n")
            f.write("-" * 55 + "\n")
            for name, m in ref_metrics.items():
                r2 = m.get('R2', np.nan)
                f.write(f"{name:<20} | {m['RMSE']:.4f}   | {m['MAE']:.4f}   | {r2:.4f}\n")
            f.write("\n")

        f.write("--- SBS STEP-BY-STEP ---\n")
        f.write(f"{'Step':<5} | {'Dropped Channel':<20} | {'Remaining':<9} | {'RMSE':<8} | {'MAE':<8} | {'R2':<8}\n")
        f.write("-" * 75 + "\n")
        for h in sbs_history:
            r2_val = h.get('r2', np.nan)
            f.write(f"{h['step']:<5} | {h['dropped_channel']:<20} | {len(h['remaining_channels']):<9} | {h['rmse']:.4f}   | {h['mae']:.4f}   | {r2_val:.4f}\n")
            
        f.write("\n" + "=" * 60 + "\n")
        best_h = min(sbs_history, key=lambda x: x['rmse'])
        f.write("RECOMMENDED CHANNEL SUBSET (Lowest RMSE)\n")
        f.write("=" * 60 + "\n")
        f.write(f"Dropped so far : {len(sbs_history) - len(best_h['remaining_channels'])}\n")
        f.write(f"Optimal RMSE   : {best_h['rmse']:.4f}\n")
        f.write(f"Optimal MAE    : {best_h['mae']:.4f}\n")
        f.write(f"Optimal R2     : {best_h.get('r2', np.nan):.4f}\n")
        f.write(f"Ideal Channels : {', '.join(best_h['remaining_channels'])}\n")
        
    return report_file


# ---------------------------------------------------------------------------
# Main Strategy Logic
# ---------------------------------------------------------------------------

def run_sbs(df_raw: pd.DataFrame, loader: DataLoader, target_col: str, run_dir: Path, n_splits: int = 3, random_state: int = 42):
    all_channels = df_raw.attrs.get("channel_names")
    if not all_channels:
        print("[ERROR] No channel names found in DataLoader output.")
        return
        
    current_channels = list(all_channels)
    
    print(f"\n{'='*60}")
    print(f"PHASE — SEQUENTIAL BACKWARD SELECTION ({len(current_channels)} initial channels)")
    print(f"{'='*60}")
    
    sbs_history = []
    
    # --- Step 0: Baseline ---
    print("\n[Step 0] Training Base Model (All Channels) ... ", flush=True)
    base_metrics = run_evaluation(df_raw, loader, target_col, n_splits=n_splits, random_state=random_state)
    
    base_rmse = base_metrics['RMSE']
    base_mae = base_metrics['MAE']
    base_r2 = base_metrics.get('R2', np.nan)
    print(f"=> Base RMSE: {base_rmse:.4f} (MAE: {base_mae:.4f}, R2: {base_r2:.4f})")
    
    sbs_history.append({
        'step': 0,
        'dropped_channel': 'None (Baseline)',
        'remaining_channels': list(current_channels),
        'mae': base_mae,
        'rmse': base_rmse,
        'r2': base_r2
    })
    
    # --- Reference Models ---
    print("\n[Reference] Training EMG-only Model ... ", flush=True)
    emg_keys = list(CHANNEL_CONFIG['emg_channels'].keys())
    imu_keys = list(CHANNEL_CONFIG['imu_channels'].keys())
    actual_emg = [c for c in all_channels if c in emg_keys]
    actual_imu = [c for c in all_channels if c in imu_keys]
    
    ref_metrics = {}
    if actual_emg:
        df_emg = slice_channels(df_raw, all_channels, actual_emg)
        ref_metrics['EMG Only'] = run_evaluation(df_emg, loader, target_col, n_splits=n_splits, random_state=random_state)
        print(f"=> EMG Only RMSE: {ref_metrics['EMG Only']['RMSE']:.4f} (MAE: {ref_metrics['EMG Only']['MAE']:.4f})")
    
    if actual_imu:
        print("\n[Reference] Training IMU-only Model ... ", flush=True)
        df_imu = slice_channels(df_raw, all_channels, actual_imu)
        ref_metrics['IMU Only'] = run_evaluation(df_imu, loader, target_col, n_splits=n_splits, random_state=random_state)
        print(f"=> IMU Only RMSE: {ref_metrics['IMU Only']['RMSE']:.4f} (MAE: {ref_metrics['IMU Only']['MAE']:.4f})")
    
    # --- SBS Loop ---
    step = 1
    while len(current_channels) > 1:
        print(f"\n[Step {step}] Evaluating removals from {len(current_channels)} channels...")
        best_rmse = float('inf')
        best_channel_to_drop = None
        best_metrics = None
        
        for ch in current_channels:
            print(f"  Testing minus '{ch}' ... ", end="", flush=True)
            
            test_subset = [c for c in current_channels if c != ch]
            df_slice = slice_channels(df_raw, all_channels, test_subset)
            
            metrics = run_evaluation(df_slice, loader, target_col, n_splits=n_splits, random_state=random_state)
            rmse = metrics['RMSE']
            mae = metrics['MAE']
            r2 = metrics.get('R2', np.nan)
            
            print(f"RMSE = {rmse:.4f} (MAE: {mae:.4f}, R2: {r2:.4f})")
            
            if rmse < best_rmse:
                best_rmse = rmse
                best_channel_to_drop = ch
                best_metrics = metrics
                
        # Lock in the best drop
        print(f"+++ WINNER Step {step}: Drop '{best_channel_to_drop}' (New Best RMSE: {best_rmse:.4f})")
        current_channels.remove(best_channel_to_drop)
        
        sbs_history.append({
            'step': step,
            'dropped_channel': best_channel_to_drop,
            'remaining_channels': list(current_channels),
            'mae': best_metrics['MAE'],
            'rmse': best_rmse,
            'r2': best_metrics.get('R2', np.nan)
        })
        
        step += 1
        
        # Save checkpoints incrementally
        write_report(sbs_history, run_dir, ref_metrics)
        plot_sbs_history(sbs_history, run_dir, ref_metrics)
        
    return sbs_history, ref_metrics


def main():
    parser = argparse.ArgumentParser(description="CNN-LSTM Channel Ablation Study")
    parser.add_argument("--cv-splits", type=int, default=3, help="Number of cross-validation splits (default: 3)")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for CV shuffling (default: 42)")
    args = parser.parse_args()

    base_dir = Path(__file__).resolve().parent.parent.parent
    segments_dir = base_dir / "database" / "segments"
    
    # Match the existing ablation structure
    results_dir = base_dir / "model" / "ablation" / "ablation_results"
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = results_dir / f"ablation_cnn_lstm_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Results will be saved to:\n  {run_dir}\n")
    
    h5_paths = list(segments_dir.glob("*.h5"))
    if not h5_paths:
        print(f"[ERROR] No HDF5 segment files found in {segments_dir}.")
        return
        
    # Instantiate Data Loader
    loader = DataLoader()
    
    print("Loading raw segments for CNN-LSTM Ablation...")
    df_raw = loader.load_raw_segments(h5_paths)
    
    if df_raw.empty:
        print("[ERROR] Extracted raw segments DataFrame is empty.")
        return
        
    all_channels = df_raw.attrs.get("channel_names")
    print(f"Dataset Loaded: {len(df_raw)} segments found. Channels: {len(all_channels)}")
    
    target_col = "weight"
    
    # Run the SBS strategy
    sbs_history, ref_metrics = run_sbs(df_raw, loader, target_col, run_dir, n_splits=args.cv_splits, random_state=args.seed)
    
    # Finalize outputs
    report_file = write_report(sbs_history, run_dir, ref_metrics)
    plot_file = plot_sbs_history(sbs_history, run_dir, ref_metrics)
    
    print(f"\n{'='*60}")
    print("Ablation study complete.")
    print(f"Report : {report_file}")
    print(f"Plot   : {plot_file}")
    print(f"{'='*60}")

if __name__ == "__main__":
    main()
