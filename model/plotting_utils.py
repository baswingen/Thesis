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
