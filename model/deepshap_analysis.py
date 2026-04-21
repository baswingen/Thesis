"""
deepshap_analysis.py
====================
Modular DeepSHAP channel-importance analysis for the trained CNN-LSTM model.
Can be run standalone or called directly from run_model.py.

Statistically optimal settings for thesis-level fidelity:
- N_BACKGROUND = 200
- N_EXPLAIN = 100
- GPU acceleration enabled where stable (CUDA)
"""

import sys
import numpy as np
import torch
import torch.nn as nn
import matplotlib.pyplot as plt
from pathlib import Path
from sklearn.model_selection import train_test_split
from tqdm import tqdm
import shap

# ---------------------------------------------------------------------------
# Project root on sys.path so 'model' package is importable
# ---------------------------------------------------------------------------
ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from model.data_loader import DataLoader
from model.model_archs.cnn_lstm import CNNLSTMRegressor, CNNLSTMNetwork
from model.config_model import CHANNEL_CONFIG, DATABASE_CONFIG
from model.plotting_utils import MODALITY_COLORS, get_modality_color

# ---------------------------------------------------------------------------
# SHAP-compatible wrapper — removes the 'lengths' argument from forward()
# ---------------------------------------------------------------------------

class CNNLSTMForSHAP(nn.Module):
    """
    Thin wrapper so shap.DeepExplainer can call forward(x) with a single
    tensor. Lengths are reconstructed from the non-zero time-step mask.
    """
    def __init__(self, network: CNNLSTMNetwork):
        super().__init__()
        self.network = network

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (batch, n_channels, time_steps) — may contain zero-padded frames
        nonzero_mask = (x.abs().sum(dim=1) > 1e-8)   # (batch, T)
        lengths = nonzero_mask.sum(dim=1).clamp(min=1)  # (batch,)
        return self.network(x, lengths)

# ---------------------------------------------------------------------------
# Core Analysis Functions
# ---------------------------------------------------------------------------

def pad_to_tensor(scaled_segs: list[np.ndarray], device: torch.device) -> torch.Tensor:
    """Convert scaled segments [(T_i, C)] → padded tensor (N, C, T_max)."""
    n_channels = scaled_segs[0].shape[1]
    max_t = max(s.shape[0] for s in scaled_segs)
    out = np.zeros((len(scaled_segs), n_channels, max_t), dtype=np.float32)
    for i, seg in enumerate(scaled_segs):
        t = seg.shape[0]
        out[i, :, :t] = seg.T          # (T, C) → (C, T)
    return torch.tensor(out, device=device)

def _pad_time(t: torch.Tensor, target_t: int) -> torch.Tensor:
    """Pad last dimension (time) to match target_t."""
    pad = target_t - t.shape[2]
    if pad > 0:
        t = torch.nn.functional.pad(t, (0, pad))
    return t

def run_deep_shap_analysis(regressor, X_train, X_test, out_dir, n_bg=200, n_exp=100, random_state=42):
    """
    Main entry point for DeepSHAP analysis.
    
    Parameters
    ----------
    regressor : CNNLSTMRegressor
        Fitted regressor object.
    X_train, X_test : pd.DataFrame
        Dataframes containing 'raw_segment'.
    out_dir : Path
        Where to save plots and values.
    n_bg : int
        Number of background samples (reference distribution).
    n_exp : int
        Number of test samples to explain.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    
    device = regressor.device
    
    # 1. Select samples
    n_bg = min(n_bg, len(X_train))
    n_exp = min(n_exp, len(X_test))
    
    rng = np.random.default_rng(random_state)
    bg_idx  = rng.choice(len(X_train), size=n_bg,  replace=False)
    exp_idx = rng.choice(len(X_test),  size=n_exp, replace=False)
    
    bg_segs  = regressor._extract_raw_segments(X_train.iloc[bg_idx])
    exp_segs = regressor._extract_raw_segments(X_test.iloc[exp_idx])
    
    bg_scaled  = regressor._scale_segments(bg_segs,  fit=False)
    exp_scaled = regressor._scale_segments(exp_segs, fit=False)
    
    bg_tensor  = pad_to_tensor(bg_scaled,  device)
    exp_tensor = pad_to_tensor(exp_scaled, device)
    
    max_t = max(bg_tensor.shape[2], exp_tensor.shape[2])
    bg_tensor  = _pad_time(bg_tensor,  max_t)
    exp_tensor = _pad_time(exp_tensor, max_t)

    # 2. Setup Device (Statistically Optimal & High Performance)
    # Use CUDA if available, fallback to CPU. 
    # MPS is skipped because of precision issues with SHAP's additivity check.
    if device.type == "cuda":
        shap_device = device
        print(f"[DeepSHAP] Using GPU acceleration (CUDA).")
    else:
        shap_device = torch.device("cpu")
        print(f"[DeepSHAP] Using CPU (MPS/Apple Silicon has precision issues with SHAP).")

    shap_model = CNNLSTMForSHAP(regressor.model).to(shap_device)
    shap_model.eval()
    
    bg_tensor_shap  = bg_tensor.to(shap_device)
    exp_tensor_shap = exp_tensor.to(shap_device)

    # 3. Compute SHAP
    print(f"\nRunning shap.DeepExplainer with {n_bg} background and {n_exp} test samples …")
    explainer = shap.DeepExplainer(shap_model, bg_tensor_shap)
    
    batch_size = 10 if shap_device.type == "cuda" else 5
    all_shap_values = []
    
    for i in tqdm(range(0, len(exp_tensor_shap), batch_size), desc="Computing SHAP", unit="batch"):
        batch_exp = exp_tensor_shap[i : i + batch_size]
        # check_additivity=False to handle potential small float drifts
        batch_shap = explainer.shap_values(batch_exp, check_additivity=False)
        
        if isinstance(batch_shap, list):
            batch_shap = batch_shap[0]
            
        batch_shap = np.array(batch_shap)
        if batch_shap.ndim == 4:
            if batch_shap.shape[-1] == 1: batch_shap = batch_shap[..., 0]
            elif batch_shap.shape[0] == 1: batch_shap = batch_shap[0]
                
        all_shap_values.append(batch_shap)
        
    shap_values = np.concatenate(all_shap_values, axis=0) # (N, C, T)
    
    # 4. Process results
    channel_names = X_test.attrs.get("channel_names", [])
    if not channel_names:
        # Fallback if attributes lost
        emg_cfg = CHANNEL_CONFIG.get('emg_channels', {})
        imu_cfg = CHANNEL_CONFIG.get('imu_channels', {})
        channel_names = [k for k, v in emg_cfg.items() if v] + [k for k, v in imu_cfg.items() if v]
        
    process_and_plot_shap(shap_values, channel_names, out_dir)
    return shap_values

def process_and_plot_shap(shap_values, channel_names, out_dir):
    """Aggregates SHAP values over time/samples and generates importance plots."""
    out_dir = Path(out_dir)
    # Aggregate: mean |SHAP| across time, then mean over test samples → (n_channels,)
    mean_abs_shap = np.abs(shap_values).sum(axis=2).mean(axis=0)
    n_channels = len(mean_abs_shap)
    
    if len(channel_names) != n_channels:
        channel_names = [f"ch{i}" for i in range(n_channels)]

    # IMU grouping (consistent with permutation importance)
    imu_groups = {
        'ax1': '$a_1$', 'ay1': '$a_1$', 'az1': '$a_1$',
        'roll_rad1': '$\\alpha_1$', 'pitch_rad1': '$\\alpha_1$', 'yaw_rad1': '$\\alpha_1$',
        'ax2': '$a_2$', 'ay2': '$a_2$', 'az2': '$a_2$',
        'roll_rad2': '$\\alpha_2$', 'pitch_rad2': '$\\alpha_2$', 'yaw_rad2': '$\\alpha_2$',
        'ax_diff': '$a_{diff}$', 'ay_diff': '$a_{diff}$', 'az_diff': '$a_{diff}$',
        'roll_rad_diff': '$\\alpha_{diff}$', 'pitch_rad_diff': '$\\alpha_{diff}$', 'yaw_rad_diff': '$\\alpha_{diff}$',
    }

    grouped_names = []
    grouped_vals = []
    grouped_is_emg = []
    seen_groups = {}

    n_emg = sum(1 for v in CHANNEL_CONFIG.get('emg_channels', {}).values() if v)
    
    for i, name in enumerate(channel_names):
        is_emg = i < n_emg
        val = mean_abs_shap[i]
        
        if is_emg:
            grouped_names.append(name)
            grouped_vals.append(val)
            grouped_is_emg.append(True)
        else:
            group_label = imu_groups.get(name, name)
            if group_label in seen_groups:
                idx = seen_groups[group_label]
                grouped_vals[idx] += val
            else:
                seen_groups[group_label] = len(grouped_names)
                grouped_names.append(group_label)
                grouped_vals.append(val)
                grouped_is_emg.append(False)

    grouped_names = np.array(grouped_names)
    grouped_vals = np.array(grouped_vals)
    grouped_is_emg = np.array(grouped_is_emg)
    n_grouped = len(grouped_names)

    # ── Plot 1: Bar chart ──────────────────────────────────────────────────
    order = np.argsort(grouped_vals)[::-1]
    sorted_names = grouped_names[order]
    colors = [get_modality_color(name) for name in sorted_names]

    fig, ax = plt.subplots(figsize=(max(10, n_grouped * 0.6), 6))
    ax.bar(range(n_grouped), grouped_vals[order], color=colors, edgecolor="white", linewidth=0.5)
    ax.set_xticks(range(n_grouped))
    ax.set_xticklabels(sorted_names, rotation=45, ha="right", fontsize=10)
    ax.set_ylabel("Mean |SHAP value|", fontsize=11)
    ax.set_title("DeepSHAP Channel Importance (Grouped IMU)", fontsize=13, fontweight="bold")
    ax.spines[["top", "right"]].set_visible(False)

    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor=MODALITY_COLORS["EMG"], label="EMG"),
                       Patch(facecolor=MODALITY_COLORS["IMU"], label="IMU (Grouped)")]
    ax.legend(handles=legend_elements, loc="upper right")

    plt.tight_layout()
    fig.savefig(out_dir / "deepshap_channel_importance.png", dpi=300, bbox_inches="tight")
    plt.close(fig)

    # ── Plot 2: Modality split ─────────────────────────────────────────────
    emg_total = grouped_vals[grouped_is_emg].sum()
    imu_total = grouped_vals[~grouped_is_emg].sum()
    total     = emg_total + imu_total

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].pie(
        [emg_total, imu_total],
        labels=[f"EMG\n{emg_total/total*100:.1f}%", f"IMU\n{imu_total/total*100:.1f}%"],
        colors=[MODALITY_COLORS["EMG"], MODALITY_COLORS["IMU"]],
        startangle=90, autopct="%1.1f%%",
        wedgeprops=dict(edgecolor="white", linewidth=2),
    )
    axes[0].set_title("Modality Share of Total |SHAP|", fontsize=12)

    # Horizontal bars
    all_colors_plot = [MODALITY_COLORS["EMG"]] * sum(grouped_is_emg) + [MODALITY_COLORS["IMU"]] * sum(~grouped_is_emg)
    # Re-sort for within-modality
    emg_idx = np.where(grouped_is_emg)[0]
    imu_idx = np.where(~grouped_is_emg)[0]
    emg_order = emg_idx[np.argsort(grouped_vals[emg_idx])[::-1]]
    imu_order = imu_idx[np.argsort(grouped_vals[imu_idx])[::-1]]
    plot_idx = np.concatenate([emg_order, imu_order])
    
    axes[1].barh(range(n_grouped), grouped_vals[plot_idx][::-1], color=np.array(all_colors_plot)[::-1], edgecolor="white")
    axes[1].set_yticks(range(n_grouped))
    axes[1].set_yticklabels(grouped_names[plot_idx][::-1], fontsize=9)
    axes[1].set_xlabel("Mean |SHAP value|")
    axes[1].set_title("Importance by Modality (Grouped)", fontsize=12)
    axes[1].spines[["top", "right"]].set_visible(False)

    plt.tight_layout()
    fig.savefig(out_dir / "deepshap_modality_split.png", dpi=300, bbox_inches="tight")
    plt.close(fig)

    # ── Save values ────────────────────────────────────────────────────────
    np.savez(out_dir / "deepshap_values.npz",
             shap_values=shap_values,
             channel_names=np.array(channel_names),
             grouped_names=grouped_names,
             grouped_vals=grouped_vals,
             mean_abs_shap=mean_abs_shap)
    print(f"[DeepSHAP] Results saved to {out_dir}")

# ---------------------------------------------------------------------------
# Standalone Execution
# ---------------------------------------------------------------------------

def main():
    # Helper to run standalone-style as before
    print("Running standalone DeepSHAP analysis...")
    # These would normally come from CLI or hardcoded as before
    SAVED_MODEL_PATH = "model/model_results/run_20260418_180005/cnn_lstm_model.joblib"
    
    model_path = ROOT / SAVED_MODEL_PATH
    if not model_path.exists():
        print(f"Error: Model not found at {model_path}"); return

    loader = DataLoader()
    h5_paths = list(DATABASE_CONFIG["segments_dir"].glob("*.h5"))
    df = loader.load_raw_segments(h5_paths)
    X, y = loader.prepare_for_ml(df, target_col="weight")
    
    # Same split as run_model.py
    X_train, X_test, _, _ = train_test_split(X, y, test_size=0.2, random_state=42, stratify=df["weight"])
    regressor = CNNLSTMRegressor.load(model_path)
    
    run_deep_shap_analysis(regressor, X_train, X_test, model_path.parent, n_bg=200, n_exp=100)

if __name__ == "__main__":
    main()
