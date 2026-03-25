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
from model.config_model import CNN_LSTM_ABLATION_CONFIG

from sklearn.model_selection import train_test_split

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def slice_channels(df: pd.DataFrame, all_channels: list, subset_channels: list) -> pd.DataFrame:
    """Subset the 'raw_segment' numpy arrays in the DataFrame to keep only requested channels."""
    indices = [all_channels.index(c) for c in subset_channels if c in all_channels]
    df_sliced = df.copy(deep=True)
    df_sliced['raw_segment'] = df_sliced['raw_segment'].apply(lambda x: x[:, indices])
    return df_sliced


def run_evaluation(df_subset: pd.DataFrame, loader: DataLoader, target_col: str = "weight"):
    """
    Train and evaluate the CNN-LSTM explicitly mapping to the CNN_LSTM_ABLATION_CONFIG.
    Returns metrics dict with MAE, MSE, RMSE, R2.
    """
    X, y = loader.prepare_for_ml(df_subset, target_col=target_col)
    
    # Train/Test Split
    strat_labels = df_subset["label"] if "label" in df_subset.columns else None
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42, stratify=strat_labels
    )
    
    # Note: CNNLSTMRegressor handles the internal validation split (e.g. 0.2 of X_train) automatically
    model = CNNLSTMRegressor(**CNN_LSTM_ABLATION_CONFIG)
    model.fit(X_train, y_train)
    
    metrics, _ = model.evaluate(X_test, y_test)
    return metrics


def plot_sbs_history(sbs_history: list, run_dir: Path):
    """Generates a performance plot showing MAE dropping across channel removals."""
    steps = [h['step'] for h in sbs_history]
    maes = [h['mae'] for h in sbs_history]
    dropped = [h['dropped_channel'] for h in sbs_history]
    n_remaining = [len(h['remaining_channels']) for h in sbs_history]
    
    plt.figure(figsize=(14, 7))
    plt.plot(n_remaining, maes, marker='o', linestyle='-', linewidth=2, markersize=8, color='#1f77b4')
    
    # Annotate dropped channels
    for i in range(1, len(sbs_history)):
        plt.annotate(
            f"-{dropped[i]}",
            (n_remaining[i], maes[i]),
            textcoords="offset points",
            xytext=(0, 12),
            ha='center',
            fontsize=9,
            rotation=60
        )
        
    # Invert x-axis so sequence progresses from many channels (left) to few (right)
    plt.gca().invert_xaxis()
    
    # Setup styling
    plt.title("SBS Profile: CNN-LSTM Performance vs. Remaining Channels", fontsize=16, fontweight='bold', pad=20)
    plt.xlabel("Number of Remaining Channels", fontsize=14, labelpad=10)
    plt.ylabel("Mean Absolute Error (MAE)", fontsize=14, labelpad=10)
    plt.grid(True, linestyle='--', alpha=0.6)
    
    # Highlight the best configuration visually
    best_step = np.argmin(maes)
    plt.scatter([n_remaining[best_step]], [maes[best_step]], color='red', s=150, zorder=5, label='Best Performance Subset')
    plt.legend()
    
    plt.tight_layout()
    plot_path = run_dir / "sbs_performance_plot.png"
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close()
    return plot_path


def write_report(sbs_history: list, run_dir: Path):
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
        
        f.write("--- SBS STEP-BY-STEP ---\n")
        for h in sbs_history:
            f.write(f"Step {h['step']}: Dropped -> {h['dropped_channel']:<18} | Remaining: {len(h['remaining_channels']):<2} | MAE: {h['mae']:.4f}\n")
            
        f.write("\n" + "=" * 60 + "\n")
        best_h = min(sbs_history, key=lambda x: x['mae'])
        f.write("RECOMMENDED CHANNEL SUBSET (Lowest MAE)\n")
        f.write("=" * 60 + "\n")
        f.write(f"Dropped so far : {len(sbs_history) - len(best_h['remaining_channels'])}\n")
        f.write(f"Optimal MAE    : {best_h['mae']:.4f}\n")
        f.write(f"Ideal Channels : {', '.join(best_h['remaining_channels'])}\n")
        
    return report_file


# ---------------------------------------------------------------------------
# Main Strategy Logic
# ---------------------------------------------------------------------------

def run_sbs(df_raw: pd.DataFrame, loader: DataLoader, target_col: str, run_dir: Path):
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
    base_metrics = run_evaluation(df_raw, loader, target_col)
    
    base_mae = base_metrics['MAE']
    print(f"=> Base MAE: {base_mae:.4f}")
    
    sbs_history.append({
        'step': 0,
        'dropped_channel': 'None (Baseline)',
        'remaining_channels': list(current_channels),
        'mae': base_mae,
        'rmse': base_metrics['RMSE']
    })
    
    # --- SBS Loop ---
    step = 1
    while len(current_channels) > 1:
        print(f"\n[Step {step}] Evaluating removals from {len(current_channels)} channels...")
        best_mae = float('inf')
        best_channel_to_drop = None
        best_metrics = None
        
        for ch in current_channels:
            print(f"  Testing minus '{ch}' ... ", end="", flush=True)
            
            test_subset = [c for c in current_channels if c != ch]
            df_slice = slice_channels(df_raw, all_channels, test_subset)
            
            metrics = run_evaluation(df_slice, loader, target_col)
            mae = metrics['MAE']
            
            print(f"MAE = {mae:.4f}")
            
            if mae < best_mae:
                best_mae = mae
                best_channel_to_drop = ch
                best_metrics = metrics
                
        # Lock in the best drop
        print(f"+++ WINNER Step {step}: Drop '{best_channel_to_drop}' (New Best MAE: {best_mae:.4f})")
        current_channels.remove(best_channel_to_drop)
        
        sbs_history.append({
            'step': step,
            'dropped_channel': best_channel_to_drop,
            'remaining_channels': list(current_channels),
            'mae': best_mae,
            'rmse': best_metrics['RMSE']
        })
        
        step += 1
        
        # Save checkpoints incrementally
        write_report(sbs_history, run_dir)
        plot_sbs_history(sbs_history, run_dir)
        
    return sbs_history


def main():
    parser = argparse.ArgumentParser(description="CNN-LSTM Channel Ablation Study")
    parser.parse_args()

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
    sbs_history = run_sbs(df_raw, loader, target_col, run_dir)
    
    # Finalize outputs
    report_file = write_report(sbs_history, run_dir)
    plot_file = plot_sbs_history(sbs_history, run_dir)
    
    print(f"\n{'='*60}")
    print("Ablation study complete.")
    print(f"Report : {report_file}")
    print(f"Plot   : {plot_file}")
    print(f"{'='*60}")

if __name__ == "__main__":
    main()
