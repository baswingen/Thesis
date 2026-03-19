import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pathlib import Path
from sklearn.metrics import r2_score, mean_absolute_error

def set_style():
    """Sets a professional style for paper-ready plots."""
    try:
        plt.style.use('seaborn-v0_8-paper')
    except:
        plt.style.use('ggplot') # Fallback
    
    # Customize style
    plt.rcParams.update({
        'font.size': 12,
        'axes.labelsize': 14,
        'axes.titlesize': 15,
        'xtick.labelsize': 12,
        'ytick.labelsize': 12,
        'legend.fontsize': 11,
        'axes.spines.top': False,
        'axes.spines.right': False,
        'axes.grid': True,
        'grid.alpha': 0.3,
        'grid.linestyle': '--'
    })

def plot_regression_results(y_true: pd.Series | np.ndarray, y_pred: np.ndarray, save_path: str | Path, model_name: str = "Model"):
    """
    Creates an enhanced 'Predicted vs Actual' box plot with standard dimensions (8x6).
    Ensures points are clean on the line (no jitter), weights are used as axis labels 
    with full decimal precision, and limits are tightly adjusted to the data (max 6kg).
    """
    set_style()
    
    if isinstance(y_true, pd.Series):
        y_true = y_true.values
    
    # Standard dimensions
    fig, ax = plt.subplots(figsize=(8, 6))
    
    # Extract unique actual weights and group predictions
    actual_weights = np.sort(np.unique(y_true))
    pred_groups = [y_pred[y_true == w] for w in actual_weights]
    
    # Define bounds based on data, but respect the user's 6kg limit
    min_val = 0.0
    # Determine a sensible max limit. Use max data point + small buffer, or at least 6.5
    data_max = max(np.max(y_true), np.max(y_pred))
    max_val = min(7.5, max(6.25, data_max + 0.25))
    
    # Unity line (Actual = Predicted)
    ax.plot([min_val, max_val], [min_val, max_val], color='#D62728', linestyle='--', linewidth=1.5, alpha=0.8, label='Perfect Prediction', zorder=1)
    
    # Plot individual points STRICTLY on the actual weight line (clean on the line)
    for i, w in enumerate(actual_weights):
        preds = pred_groups[i]
        ax.scatter(np.full_like(preds, w), preds, 
                    alpha=0.15, s=20, color='#1F77B4', edgecolors='none', zorder=2)

    # Create boxplot
    bp = ax.boxplot(pred_groups, positions=actual_weights, widths=0.4, 
                    patch_artist=True, showfliers=False,
                    medianprops={'color': 'black', 'linewidth': 1.5},
                    boxprops={'alpha': 0.5, 'edgecolor': '#1F77B4', 'linewidth': 1.5},
                    zorder=3)
    
    for box in bp['boxes']:
        box.set_facecolor('#AEC7E8') 
    
    # Metrics annotation
    r2 = r2_score(y_true, y_pred)
    mae = mean_absolute_error(y_true, y_pred)
    
    # Professional metrics box top left
    stats_text = f"$R^2 = {r2:.3f}$\n$MAE = {mae:.3f}$ kg"
    ax.text(0.02, 0.98, stats_text, 
             transform=ax.transAxes, verticalalignment='top',
             bbox=dict(facecolor='white', alpha=0.9, edgecolor='#CCCCCC', boxstyle='round,pad=0.5'))
    
    # Axis labels
    ax.set_xlabel("Weight (kg)", labelpad=12)
    ax.set_ylabel("Weight (kg)", labelpad=12)
    ax.set_title(f"{model_name}: Predicted vs Actual Weight".upper(), pad=20, fontsize=16, fontweight='bold')
    
    # Force axis limits
    ax.set_xlim(min_val - 0.1, max_val)
    ax.set_ylim(min_val - 0.1, max_val)
    
    # Use weights as labels on the axes with full precision
    ticks = actual_weights
    ax.set_xticks(ticks)
    ax.set_yticks(ticks)
    
    def format_label(x):
        return f"{x:.2f}".rstrip('0').rstrip('.')
    
    ax.set_xticklabels([format_label(w) for w in ticks])
    ax.set_yticklabels([format_label(w) for w in ticks])
    
    ax.legend(loc='lower right', frameon=True, facecolor='white', framealpha=0.9)
    plt.tight_layout()
    
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Regression plot saved to {save_path}")

def plot_training_loss(loss_history: dict, save_path: str | Path, model_name: str = "Model"):
    """
    Plots the training and validation loss history in log scale.
    """
    if not loss_history.get("train"):
        return
        
    set_style()
    plt.figure(figsize=(10, 6))
    
    plt.plot(loss_history["train"], label="Training Loss", color='#1F77B4', linewidth=2)
    if loss_history.get("val"):
        plt.plot(loss_history["val"], label="Validation Loss", color='#FF7F0E', linewidth=2)
        
    plt.yscale('log')
    plt.xlabel("Epoch", labelpad=10)
    plt.ylabel("Loss (log scale)", labelpad=10)
    plt.title(f"{model_name}: Training Progress", pad=15)
    
    plt.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.9)
    plt.tight_layout()
    
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Loss plot saved to {save_path}")

def plot_seqlen_performance(per_seqlen_stats: list, save_path: str | Path, model_name: str = "Model"):
    """
    Plots MAE and RMSE as a function of elapsed time into the lift (i.e. how many
    sliding windows were available when the model made its prediction).

    Left y-axis : MAE and RMSE lines.
    Right y-axis: sample count bars (light background bars for context).
    X-axis       : real-time seconds elapsed into the lift at prediction.
    """
    if not per_seqlen_stats:
        return

    set_style()

    times  = [float(row['TimeAtPrediction'].rstrip('s')) for row in per_seqlen_stats]
    maes   = [float(row['MAE'])   for row in per_seqlen_stats]
    rmses  = [float(row['RMSE'])  for row in per_seqlen_stats]
    counts = [row['Count']        for row in per_seqlen_stats]
    n_wins = [row['SeqLen']       for row in per_seqlen_stats]

    fig, ax1 = plt.subplots(figsize=(9, 5))

    # ── Background bars: sample count (right axis) ──────────────────
    ax2 = ax1.twinx()
    bar_width = (times[1] - times[0]) * 0.6 if len(times) > 1 else 0.08
    ax2.bar(times, counts, width=bar_width, color='#AEC7E8', alpha=0.35,
            label='Sample count', zorder=1)
    ax2.set_ylabel("Sample count", color='#7090B0', fontsize=12)
    ax2.tick_params(axis='y', labelcolor='#7090B0')
    ax2.set_ylim(0, max(counts) * 3.5)   # push bars to bottom third
    ax2.spines['top'].set_visible(False)

    # ── Error lines (left axis) ──────────────────────────────────────
    ax1.plot(times, maes,  marker='o', linewidth=2.0, markersize=5,
             color='#1F77B4', label='MAE',  zorder=3)
    ax1.plot(times, rmses, marker='s', linewidth=2.0, markersize=5,
             color='#FF7F0E', label='RMSE', zorder=3)

    # Annotate each point with window count
    for t, mae, n in zip(times, maes, n_wins):
        ax1.annotate(f"w={n}", xy=(t, mae), xytext=(0, 6),
                     textcoords='offset points', ha='center',
                     fontsize=8, color='#1F77B4')

    ax1.set_xlabel("Elapsed time into lift at prediction (s)", labelpad=10)
    ax1.set_ylabel("Error (kg)", labelpad=10)
    ax1.set_title(f"{model_name}: Prediction Error vs. Available Segment Length".upper(),
                  pad=18, fontsize=13, fontweight='bold')
    ax1.set_ylim(bottom=0)
    ax1.set_xticks(times)
    ax1.set_xticklabels([f"{t:.2f}s" for t in times], rotation=30, ha='right')

    # Combined legend
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2,
               loc='upper right', frameon=True, facecolor='white', framealpha=0.9)

    ax1.set_zorder(ax2.get_zorder() + 1)
    ax1.patch.set_visible(False)

    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Sequence-length performance plot saved to {save_path}")
