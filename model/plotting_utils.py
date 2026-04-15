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
    
    # Plot individual OUTLIER points STRICTLY on the actual weight line
    first_outlier = True
    for i, w in enumerate(actual_weights):
        preds = pred_groups[i]
        if len(preds) > 0:
            q1 = np.percentile(preds, 25)
            q3 = np.percentile(preds, 75)
            iqr = q3 - q1
            outliers = preds[(preds < q1 - 1.5 * iqr) | (preds > q3 + 1.5 * iqr)]
            
            if len(outliers) > 0:
                label = "Outliers" if first_outlier else ""
                ax.scatter(np.full_like(outliers, w), outliers, 
                            alpha=0.35, s=25, color='#1F77B4', edgecolors='none', 
                            zorder=2, label=label)
                first_outlier = False

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

def plot_seqlen_performance(per_seqlen_stats: list, save_path: str | Path,
                            model_name: str = "Model", min_count: int = 10):
    """
    Plots MAE and RMSE as a function of elapsed time into the lift (i.e. how many
    sliding windows were available when the model made its prediction).

    Left y-axis : MAE and RMSE lines.
    Right y-axis: sample count bars (light background bars for context).
    X-axis       : real-time seconds elapsed into the lift at prediction.
    """
    if not per_seqlen_stats:
        return

    # Drop bins with too few samples for reliable statistics
    filtered = [row for row in per_seqlen_stats if row['Count'] >= min_count]
    n_dropped = len(per_seqlen_stats) - len(filtered)
    if not filtered:
        print(f"[plot_seqlen_performance] All bins dropped (min_count={min_count}). Nothing to plot.")
        return

    set_style()

    times  = [float(row['TimeAtPrediction'].rstrip('s')) for row in filtered]
    maes   = [float(row['MAE'])   for row in filtered]
    rmses  = [float(row['RMSE'])  for row in filtered]
    counts = [row['Count']        for row in filtered]
    n_wins = [row['SeqLen']       for row in filtered]

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
    
    # Tick management: if too many bins, only show a subset of labels to prevent overlap
    ax1.set_xticks(times)
    if len(times) > 20:
        # Show every Nth label
        n_show = 15  # aim for around 15 labels
        step = max(1, len(times) // n_show)
        visible_labels = []
        for i, t in enumerate(times):
            if i % step == 0 or i == len(times) - 1:
                visible_labels.append(f"{t:.2f}s")
            else:
                visible_labels.append("")
        ax1.set_xticklabels(visible_labels, rotation=30, ha='right')
    else:
        ax1.set_xticklabels([f"{t:.2f}s" for t in times], rotation=30, ha='right')

    # Combined legend
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2,
               loc='upper right', frameon=True, facecolor='white', framealpha=0.9)

    ax1.set_zorder(ax2.get_zorder() + 1)
    ax1.patch.set_visible(False)

    # Footnote about omitted bins
    footnote = f"Bins with < {min_count} samples omitted ({n_dropped} bin(s) excluded)."
    fig.text(0.5, -0.02, footnote, ha='center', va='top',
             fontsize=9, color='grey', style='italic')

    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Sequence-length performance plot saved to {save_path}")

def plot_participant_performance(per_participant_stats: list, save_path: str | Path, model_name: str = "Model"):
    """
    Creates a bar chart to visualize the generalization error (MAE and RMSE) for each left-out participant.
    Also plots horizontal lines representing the mean error across all participants.
    """
    if not per_participant_stats:
        return

    set_style()
    
    participants = [stat['Participant'] for stat in per_participant_stats]
    maes = [stat['MAE'] for stat in per_participant_stats]
    rmses = [stat['RMSE'] for stat in per_participant_stats]

    x = np.arange(len(participants))
    width = 0.35

    fig, ax = plt.subplots(figsize=(10, 6))
    
    rects1 = ax.bar(x - width/2, maes, width, label='MAE', color='#1F77B4', alpha=0.85)
    rects2 = ax.bar(x + width/2, rmses, width, label='RMSE', color='#FF7F0E', alpha=0.85)
    
    mean_mae = np.mean(maes)
    mean_rmse = np.mean(rmses)
    
    ax.axhline(mean_mae, color='#1F77B4', linestyle='--', linewidth=1.5, alpha=0.8)
    ax.text(-0.5, mean_mae + max(0.02, mean_mae*0.02), f"Mean MAE: {mean_mae:.3f}", color='#1F77B4', fontsize=10, verticalalignment='bottom')
    
    ax.axhline(mean_rmse, color='#FF7F0E', linestyle='--', linewidth=1.5, alpha=0.8)
    ax.text(-0.5, mean_rmse + max(0.02, mean_rmse*0.02), f"Mean RMSE: {mean_rmse:.3f}", color='#FF7F0E', fontsize=10, verticalalignment='bottom')
    
    ax.set_ylabel('Error (kg)')
    ax.set_xlabel('Unseen Participant Left Out', labelpad=12)
    ax.set_title(f'{model_name}: Generalization Performance per Participant'.upper(), pad=15, fontsize=14, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(participants)
    
    max_val = max(np.max(maes), np.max(rmses))
    ax.set_ylim(0, max_val * 1.25)
    ax.legend(loc='upper right')

    fig.tight_layout()

    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Participant performance plot saved to {save_path}")



def plot_permutation_importance(importances: dict, save_path: str | Path, model_name: str = "Model"):
    """
    Plots a horizontal bar chart of permutation feature/channel importance.
    
    Parameters
    ----------
    importances: dict
        Dictionary mapping channel/feature names to importance scores (MSE drop).
    save_path: str or Path
        Where to save the plot.
    model_name: str
        Name of the model for the title.
    """
    if not importances:
        return

    set_style()
    
    # Sort by importance (highest first)
    sorted_items = sorted(importances.items(), key=lambda x: x[1])
    features = [k for k, v in sorted_items]
    scores = [v for k, v in sorted_items]

    fig, ax = plt.subplots(figsize=(10, max(6, len(features) * 0.3)))
    
    # Choose color based on whether the score is positive or negative
    colors = ['#1F77B4' if s >= 0 else '#D62728' for s in scores]
    
    y = np.arange(len(features))
    ax.barh(y, scores, color=colors, alpha=0.85)
    
    ax.set_yticks(y)
    ax.set_yticklabels(features)
    ax.set_xlabel('Change in MSE (Error Increase on Permutation)', labelpad=12)
    ax.set_title(f'{model_name}: Permutation Channel Importance'.upper(), pad=15, fontsize=14, fontweight='bold')
    
    # Add vertical line at 0
    ax.axvline(0, color='black', linewidth=1, linestyle='-', alpha=0.5)
    
    # Annotate bars with values
    for i, score in enumerate(scores):
        ha = 'left' if score >= 0 else 'right'
        offset = 0.01 * max(np.abs(scores)) * (1 if score >= 0 else -1)
        ax.text(score + offset, i, f"{score:.4f}", va='center', ha=ha, fontsize=9, color='#333333')

    # Add margins to prevent text cutoff
    x_min = min(0, min(scores))
    x_max = max(0, max(scores))
    margin = (x_max - x_min) * 0.15
    ax.set_xlim(x_min - margin if x_min < 0 else 0, x_max + margin)

    fig.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Permutation importance plot saved to {save_path}")

def plot_cnn_tsne(
    features: np.ndarray,
    participants: np.ndarray,
    weights: np.ndarray,
    save_path: "str | Path",
    model_name: str = "CNN-LSTM",
    perplexity: int = 30,
    random_state: int = 42,
):
    """
    t-SNE scatter plot of CNN-extracted features colored by participant and weight.

    Each participant gets a distinct base hue.  Within that hue, weight controls
    lightness: lighter color = lower weight, darker color = heavier weight.
    This lets you visually inspect whether the CNN feature extractor groups
    samples by participant (good clustering) or mixes them up.

    Parameters
    ----------
    features     : (N, D) array of CNN feature vectors.
    participants : (N,)   array-like of participant IDs (str or int).
    weights      : (N,)   array-like of weight labels (float).
    save_path    : Path   where to save the PNG.
    perplexity   : t-SNE perplexity (default 30).
    random_state : reproducibility seed.
    """
    from sklearn.manifold import TSNE
    import colorsys
    import matplotlib.colors as mcolors
    import matplotlib.cm as cm
    import matplotlib.patches as mpatches
    from tqdm import tqdm

    set_style()

    # ── Run t-SNE ──────────────────────────────────────────────────────────
    print(f"[t-SNE] Fitting on {len(features)} samples (perplexity={min(perplexity, len(features) - 1)})…")
    tsne = TSNE(n_components=2, perplexity=min(perplexity, len(features) - 1),
                random_state=random_state, max_iter=1000, init='pca', verbose=1)
    emb = tsne.fit_transform(features)          # (N, 2)

    participants = np.asarray(participants)
    weights = np.asarray(weights, dtype=float)

    unique_participants = np.unique(participants)
    n_participants = len(unique_participants)

    # ── Assign base hues to participants ───────────────────────────────────
    # Use a qualitative colormap spread evenly across the hue wheel
    base_hues = np.linspace(0.0, 1.0, n_participants, endpoint=False)
    participant_to_hue = {p: h for p, h in zip(unique_participants, base_hues)}

    # ── Compute per-sample colours in HLS space ────────────────────────────
    # Lightness range per participant: 0.75 (light, low weight) → 0.25 (dark, heavy)
    w_min, w_max = weights.min(), weights.max()
    w_range = w_max - w_min if w_max > w_min else 1.0

    colors = []
    for p, w in zip(participants, weights):
        hue = participant_to_hue[p]
        # Normalise weight → [0, 1], then map to lightness [0.75, 0.25]
        t = (w - w_min) / w_range          # 0 = lightest, 1 = darkest
        lightness = 0.75 - t * 0.50        # 0.75 → 0.25
        saturation = 0.80
        rgb = colorsys.hls_to_rgb(hue, lightness, saturation)
        colors.append(rgb)

    # ── Plot ───────────────────────────────────────────────────────────────
    # Use a clean white background for this plot regardless of global style
    with plt.rc_context({'axes.facecolor': 'white', 'figure.facecolor': 'white',
                         'axes.grid': True, 'grid.alpha': 0.2, 'grid.linestyle': '--'}):
        fig, ax = plt.subplots(figsize=(11, 9))

    ax.scatter(
        emb[:, 0], emb[:, 1],
        c=colors,
        s=8,
        alpha=0.75,
        linewidths=0.3,
        edgecolors='white',
        zorder=2,
    )

    ax.set_facecolor('white')
    ax.grid(True, linestyle='--', alpha=0.2, zorder=0)
    ax.set_xlabel("t-SNE Component 1", labelpad=10)
    ax.set_ylabel("t-SNE Component 2", labelpad=10)
    ax.set_title(
        f"{model_name}: CNN Feature Space (t-SNE)".upper(),
        pad=18, fontsize=14, fontweight='bold'
    )

    # ── Legend: one patch per participant with a mid-lightness colour ──────
    legend_patches = []
    for p in unique_participants:
        hue = participant_to_hue[p]
        mid_rgb = colorsys.hls_to_rgb(hue, 0.45, 0.80)   # representative dark-ish tone
        patch = mpatches.Patch(color=mid_rgb, label=str(p))
        legend_patches.append(patch)

    participant_legend = ax.legend(
        handles=legend_patches,
        title="Participant",
        loc='best',
        frameon=True,
        facecolor='white',
        edgecolor='#cccccc',
        framealpha=0.95,
        fontsize=9,
        title_fontsize=10,
        markerscale=1.5,
    )
    ax.add_artist(participant_legend)

    # ── Colorbar-style weight gradient legend ─────────────────────────────
    # Use a neutral grey-to-black gradient just for the legend bar
    gradient = np.linspace(0, 1, 256).reshape(1, -1)
    cbar_ax = fig.add_axes([0.92, 0.20, 0.015, 0.60])   # [left, bottom, width, height]

    # Build a custom ListedColormap using the first participant's hue as example
    # (gradient only communicates the lightness concept)
    sample_hue = base_hues[0]
    cmap_colors = [colorsys.hls_to_rgb(sample_hue, 0.75 - t * 0.50, 0.80)
                   for t in np.linspace(0, 1, 256)]
    from matplotlib.colors import ListedColormap
    weight_cmap = ListedColormap(cmap_colors)

    norm = mcolors.Normalize(vmin=w_min, vmax=w_max)
    sm = cm.ScalarMappable(cmap=weight_cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, cax=cbar_ax)
    cbar.set_label("Weight (kg)", rotation=270, labelpad=15, fontsize=10)
    cbar.ax.yaxis.set_ticks_position('right')

    # Footnote
    fig.text(
        0.5, -0.01,
        f"t-SNE perplexity={perplexity}  |  lighter shade = lower weight, darker = heavier",
        ha='center', va='top', fontsize=9, color='grey', style='italic'
    )

    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"t-SNE feature plot saved to {save_path}")
