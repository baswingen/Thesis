"""
deepshap_analysis.py
====================
Modular DeepSHAP channel-importance analysis for trained models.
Supports both CNN-LSTM and Spatio-Temporal Transformer architectures.

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
# SHAP-compatible wrappers
# ---------------------------------------------------------------------------

class CNNLSTMForSHAP(nn.Module):
    """
    Thin wrapper so shap.DeepExplainer can call forward(x) with a single tensor.
    If anthropometrics are enabled, they are extracted from the trailing channels
    of x (where they were tiled across time).
    """
    def __init__(self, network, n_original_channels: int):
        super().__init__()
        self.network = network
        self.n_original_channels = n_original_channels
        self.n_static = getattr(network, 'n_static_features', 0)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (batch, n_channels_total, time_steps)
        if self.n_static > 0:
            time_series_x = x[:, :self.n_original_channels, :]
            # Extract static features from the first time-step of the trailing channels
            static_features = x[:, self.n_original_channels:, 0]
        else:
            time_series_x = x
            static_features = None

        # Reconstruct lengths from the non-zero time-series mask
        nonzero_mask = (time_series_x.abs().sum(dim=1) > 1e-8)
        lengths = nonzero_mask.sum(dim=1).clamp(min=1)
        
        return self.network(time_series_x, lengths, static_features=static_features)

class SpatioTemporalTransformerForSHAP(nn.Module):
    """
    Wrapper for Spatio-Temporal Transformer.
    Input x is (batch, seq_len, total_features).
    """
    def __init__(self, network):
        super().__init__()
        self.network = network

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (batch, seq_len, total_features)
        # Compute lengths based on zero padding (where all features are zero)
        nonzero_mask = (x.abs().sum(dim=2) > 1e-8)
        lengths = nonzero_mask.sum(dim=1).clamp(min=1)
        return self.network(x, lengths)

# ---------------------------------------------------------------------------
# Core Analysis Functions
# ---------------------------------------------------------------------------

def pad_to_tensor(scaled_segs: list[np.ndarray], device: torch.device, 
                  anthropometrics: np.ndarray = None, 
                  format: str = "channels_first") -> torch.Tensor:
    """
    Convert scaled segments [(T_i, C)] → padded tensor.
    format="channels_first" -> (N, C + n_static, T_max) - for CNNs
    format="time_first"     -> (N, T_max, C) - for Transformers
    """
    n_channels = scaled_segs[0].shape[1]
    max_t = max(s.shape[0] for s in scaled_segs)
    n_static = anthropometrics.shape[1] if anthropometrics is not None else 0
    
    if format == "channels_first":
        total_channels = n_channels + n_static
        out = np.zeros((len(scaled_segs), total_channels, max_t), dtype=np.float32)
        for i, seg in enumerate(scaled_segs):
            t = seg.shape[0]
            out[i, :n_channels, :t] = seg.T
            if n_static > 0:
                # Tile static features across time
                out[i, n_channels:, :] = anthropometrics[i].reshape(n_static, 1)
    else: # time_first
        # Transformers currently use (batch, time, features)
        out = np.zeros((len(scaled_segs), max_t, n_channels), dtype=np.float32)
        for i, seg in enumerate(scaled_segs):
            t = seg.shape[0]
            out[i, :t, :] = seg
            
    return torch.tensor(out, device=device)

def _pad_time(t: torch.Tensor, target_t: int, format: str = "channels_first") -> torch.Tensor:
    """Pad time dimension to match target_t."""
    if format == "channels_first":
        pad = target_t - t.shape[2]
        if pad > 0:
            t = torch.nn.functional.pad(t, (0, pad))
    else: # time_first (N, T, C)
        pad = target_t - t.shape[1]
        if pad > 0:
            t = torch.nn.functional.pad(t, (0, 0, 0, pad))
    return t

def run_deep_shap_analysis(regressor, X_train, X_test, out_dir, n_bg=200, n_exp=100, random_state=42):
    """
    Main entry point for DeepSHAP analysis.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    
    device = regressor.device
    model_type = regressor.__class__.__name__.lower()
    is_transformer = "transformer" in model_type
    
    # 1. Select samples
    n_bg = min(n_bg, len(X_train))
    n_exp = min(n_exp, len(X_test))
    
    rng = np.random.default_rng(random_state)
    bg_idx  = rng.choice(len(X_train), size=n_bg,  replace=False)
    exp_idx = rng.choice(len(X_test),  size=n_exp, replace=False)
    
    # Extract and scale data
    if is_transformer:
        bg_sequences = regressor._extract_sequences(X_train.iloc[bg_idx])
        exp_sequences = regressor._extract_sequences(X_test.iloc[exp_idx])
        
        max_l = getattr(regressor, 'max_seq_len', None)
        if max_l:
            bg_sequences = [s[:max_l] for s in bg_sequences]
            exp_sequences = [s[:max_l] for s in exp_sequences]
            
        bg_scaled = []
        for seq in bg_sequences:
            arr = np.array([[w.get(k, 0.0) for k in regressor.feature_names] for w in seq])
            bg_scaled.append(regressor.scaler.transform(regressor._sanitise(arr)).astype(np.float32))
            
        exp_scaled = []
        for seq in exp_sequences:
            arr = np.array([[w.get(k, 0.0) for k in regressor.feature_names] for w in seq])
            exp_scaled.append(regressor.scaler.transform(regressor._sanitise(arr)).astype(np.float32))
            
        format = "time_first"
        n_original_channels = len(regressor.feature_names)
        anthro_bg = None
        anthro_exp = None
    else:
        bg_segs  = regressor._extract_raw_segments(X_train.iloc[bg_idx])
        exp_segs = regressor._extract_raw_segments(X_test.iloc[exp_idx])
        bg_scaled  = regressor._scale_segments(bg_segs,  fit=False)
        exp_scaled = regressor._scale_segments(exp_segs, fit=False)
        
        anthro_bg = None
        anthro_exp = None
        if getattr(regressor, 'use_anthropometrics', False):
            anthro_bg = regressor._extract_anthropometrics(X_train.iloc[bg_idx])
            anthro_exp = regressor._extract_anthropometrics(X_test.iloc[exp_idx])
            if anthro_bg is not None:
                anthro_bg = regressor.anthro_scaler.transform(anthro_bg)
                anthro_exp = regressor.anthro_scaler.transform(anthro_exp)
        
        format = "channels_first"
        n_original_channels = bg_scaled[0].shape[1]

    bg_tensor  = pad_to_tensor(bg_scaled,  device, anthropometrics=anthro_bg, format=format)
    exp_tensor = pad_to_tensor(exp_scaled, device, anthropometrics=anthro_exp, format=format)
    
    # Standardize time dimension
    dim = 2 if format == "channels_first" else 1
    max_t = max(bg_tensor.shape[dim], exp_tensor.shape[dim])
    bg_tensor  = _pad_time(bg_tensor,  max_t, format=format)
    exp_tensor = _pad_time(exp_tensor, max_t, format=format)

    # 2. Setup SHAP device
    if device.type == "cuda":
        shap_device = device
    else:
        shap_device = torch.device("cpu")
        print(f"[DeepSHAP] Using CPU for attribution.")

    if is_transformer:
        shap_model = SpatioTemporalTransformerForSHAP(regressor.model).to(shap_device)
    else:
        shap_model = CNNLSTMForSHAP(regressor.model, n_original_channels).to(shap_device)
        
    shap_model.eval()
    
    bg_tensor_shap  = bg_tensor.to(shap_device)
    exp_tensor_shap = exp_tensor.to(shap_device)

    # 3. Compute SHAP
    print(f"\nRunning DeepExplainer ({n_bg} bg, {n_exp} test) ...")
    explainer = shap.DeepExplainer(shap_model, bg_tensor_shap)
    
    # For CPU (macOS fallbacks), use batch_size=1 to avoid OOM crashes due to autograd graph scaling
    batch_size = 5 if shap_device.type == "cuda" else 1
    all_shap_values = []
    
    for i in tqdm(range(0, len(exp_tensor_shap), batch_size), desc="Computing SHAP"):
        batch_exp = exp_tensor_shap[i : i + batch_size]
        batch_shap = explainer.shap_values(batch_exp, check_additivity=False)
        
        if isinstance(batch_shap, list):
            batch_shap = batch_shap[0]
            
        batch_shap = np.array(batch_shap)
        
        # Remove singleton dimensions (DeepExplainer often adds a dimension for the output)
        if batch_shap.ndim == 4:
            if batch_shap.shape[-1] == 1:
                batch_shap = batch_shap[..., 0]
            elif batch_shap.shape[1] == 1:
                batch_shap = batch_shap[:, 0, :, :]
            elif batch_shap.shape[0] == 1:
                batch_shap = batch_shap[0]

        # Standardize output to (N, C, T) regardless of input format
        if format == "time_first" and batch_shap.ndim == 3:
            # (N, T, C) -> (N, C, T)
            batch_shap = np.transpose(batch_shap, (0, 2, 1))
            
        all_shap_values.append(batch_shap)
        
    shap_values = np.concatenate(all_shap_values, axis=0) # (N, C_total, T)
    
    # 4. Process results
    channel_names = X_test.attrs.get("channel_names", [])
    if not channel_names:
        if is_transformer:
            channel_names = regressor.feature_names
        else:
            emg_cfg = CHANNEL_CONFIG.get('emg_channels', {})
            imu_cfg = CHANNEL_CONFIG.get('imu_channels', {})
            channel_names = [k for k, v in emg_cfg.items() if v] + [k for k, v in imu_cfg.items() if v]
    
    if anthro_exp is not None:
        anthro_names = ["Age", "Height", "Body Weight", "Gender", "Arm Length", "Arm Circ."] 
        channel_names = list(channel_names) + anthro_names[:anthro_exp.shape[1]]
        
    process_and_plot_shap(shap_values, channel_names, out_dir)
    return shap_values

def process_and_plot_shap(shap_values, channel_names, out_dir):
    """Aggregates SHAP values and generates importance plots."""
    out_dir = Path(out_dir)
    mean_abs_shap = np.abs(shap_values).sum(axis=2).mean(axis=0)
    n_channels = len(mean_abs_shap)
    
    if len(channel_names) != n_channels:
        channel_names = [f"ch{i}" for i in range(n_channels)]

    imu_groups = {
        'ax1': '$a_1$', 'ay1': '$a_1$', 'az1': '$a_1$',
        'roll_rad1': '$\\alpha_1$', 'pitch_rad1': '$\\alpha_1$', 'yaw_rad1': '$\\alpha_1$',
        'ax2': '$a_2$', 'ay2': '$a_2$', 'az2': '$a_2$',
        'roll_rad2': '$\\alpha_2$', 'pitch_rad2': '$\\alpha_2$', 'yaw_rad2': '$\\alpha_2$',
        'ax_diff': '$a_{diff}$', 'ay_diff': '$a_{diff}$', 'az_diff': '$a_{diff}$',
        'roll_rad_diff': '$\\alpha_{diff}$', 'pitch_rad_diff': '$\\alpha_{diff}$', 'yaw_rad_diff': '$\\alpha_{diff}$',
    }
    anthro_names = ["Age", "Height", "Body Weight", "Gender", "Arm Length", "Arm Circ."]

    from collections import defaultdict
    channel_vals = defaultdict(float)
    channel_modality = {}
    feat_type_vals = defaultdict(float)
    
    n_emg = sum(1 for v in CHANNEL_CONFIG.get('emg_channels', {}).values() if v)
    n_ts = n_emg + sum(1 for v in CHANNEL_CONFIG.get('imu_channels', {}).values() if v)
    
    for i, name in enumerate(channel_names):
        val = mean_abs_shap[i]
        
        # Identify modality based on index (legacy support) or name
        modality = "Other"
        if name in anthro_names or i >= n_ts:
            modality = "Anthro"
        elif i < n_emg:
            modality = "EMG"
        elif i < n_ts:
            modality = "IMU"

        channel = name
        feat_type = "raw"

        if name in anthro_names:
            channel = "Anthropometrics"
            feat_type = f"Anthro_{name}"
            modality = "Anthro"
        elif name in imu_groups:
            channel = imu_groups[name]
            feat_type = "raw"
        elif '_EMG_' in name:
            parts = name.split('_EMG_')
            channel = parts[0] + "_EMG"
            feat_suffix = parts[1] if len(parts) > 1 else "raw"
            feat_type = f"EMG_{feat_suffix}"
            modality = "EMG"
        elif '_IMU_' in name:
            parts = name.split('_IMU_')
            prefix = parts[0] + "_IMU"
            suffix = parts[1] if len(parts) > 1 else ""
            if any(k in suffix for k in ['ax', 'ay', 'az', 'acc']):
                channel = prefix + "_Accel"
            elif any(k in suffix for k in ['roll', 'pitch', 'yaw', 'gyro', 'rad']):
                channel = prefix + "_Orient"
            else:
                channel = prefix
            sub_parts = suffix.rsplit('_', 1)
            feat_suffix = sub_parts[-1] if len(sub_parts) > 1 else suffix
            if not feat_suffix: feat_suffix = "raw"
            feat_type = f"IMU_{feat_suffix}"
            modality = "IMU"
        elif '_SVM_' in name:
            parts = name.split('_SVM_')
            channel = parts[0] + "_SVM"
            feat_suffix = parts[1] if len(parts) > 1 else "raw"
            feat_type = f"IMU_{feat_suffix}"
            modality = "IMU"
        else:
            channel = "Anthropometrics"
            feat_type = f"Anthro_{name}"
            modality = "Anthro"
            
        channel_vals[channel] += val
        channel_modality[channel] = modality
        feat_type_vals[feat_type] += val

    grouped_names = np.array(list(channel_vals.keys()))
    grouped_vals = np.array(list(channel_vals.values()))
    grouped_modality = np.array([channel_modality[k] for k in grouped_names])
    n_grouped = len(grouped_names)

    # ── Plot 1: Bar chart ──────────────────────────────────────────────────
    # Limit to top 20 to avoid overcrowding if there are still many channels
    order = np.argsort(grouped_vals)[::-1][:20]
    sorted_names = grouped_names[order]
    from model.plotting_utils import get_modality_color
    colors = [get_modality_color(name) for name in sorted_names]

    fig, ax = plt.subplots(figsize=(max(10, len(sorted_names) * 0.6), 6))
    ax.bar(range(len(sorted_names)), grouped_vals[order], color=colors, edgecolor="white", linewidth=0.5)
    ax.set_xticks(range(len(sorted_names)))
    ax.set_xticklabels(sorted_names, rotation=45, ha="right", fontsize=10)
    ax.set_ylabel("Mean |SHAP value|", fontsize=11)
    ax.set_title("DeepSHAP Importance (Top Signal Channels)", fontsize=13, fontweight="bold")
    ax.spines[["top", "right"]].set_visible(False)

    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor=MODALITY_COLORS["EMG"], label="EMG"),
        Patch(facecolor=MODALITY_COLORS["IMU"], label="IMU (Grouped)"),
        Patch(facecolor=MODALITY_COLORS["ANTHRO"], label="Anthropometrics")
    ]
    ax.legend(handles=legend_elements, loc="upper right")

    plt.tight_layout()
    fig.savefig(out_dir / "deepshap_channel_importance.png", dpi=300, bbox_inches="tight")
    plt.close(fig)

    # ── Plot 1b: Feature Type Bar chart ────────────────────────────────────
    if feat_type_vals and list(feat_type_vals.keys()) != ["raw"]:
        f_names = np.array(list(feat_type_vals.keys()))
        f_vals = np.array(list(feat_type_vals.values()))
        f_order = np.argsort(f_vals)[::-1]
        
        from model.plotting_utils import get_modality_color
        f_colors = [get_modality_color(name) for name in f_names[f_order]]
        
        fig, ax = plt.subplots(figsize=(max(8, len(f_names) * 0.6), 6))
        ax.bar(range(len(f_names)), f_vals[f_order], color=f_colors, edgecolor="white", linewidth=0.5)
        ax.set_xticks(range(len(f_names)))
        ax.set_xticklabels(f_names[f_order], rotation=45, ha="right", fontsize=10)
        ax.set_ylabel("Mean |SHAP value|", fontsize=11)
        ax.set_title("DeepSHAP Importance (Feature Types across all signals)", fontsize=13, fontweight="bold")
        ax.spines[["top", "right"]].set_visible(False)
        plt.tight_layout()
        fig.savefig(out_dir / "deepshap_feature_type_importance.png", dpi=300, bbox_inches="tight")
        plt.close(fig)

    # ── Plot 2: Modality split ─────────────────────────────────────────────
    emg_total = grouped_vals[grouped_modality == "EMG"].sum()
    imu_total = grouped_vals[grouped_modality == "IMU"].sum()
    anthro_total = grouped_vals[grouped_modality == "Anthro"].sum()
    total = emg_total + imu_total + anthro_total

    fig, axes = plt.subplots(1, 2, figsize=(13, 6))
    
    shares = [emg_total, imu_total, anthro_total]
    labels = ["EMG", "IMU", "Anthro"]
    colors_pie = [MODALITY_COLORS["EMG"], MODALITY_COLORS["IMU"], MODALITY_COLORS["ANTHRO"]]
    
    shares_p = [s for s in shares if s > 0]
    labels_p = [f"{l}\n{s/total*100:.1f}%" for s, l in zip(shares, labels) if s > 0]
    colors_p = [c for s, c in zip(shares, colors_pie) if s > 0]

    axes[0].pie(
        shares_p,
        labels=labels_p,
        colors=colors_p,
        startangle=90, autopct="%1.1f%%",
        wedgeprops=dict(edgecolor="white", linewidth=2),
    )
    axes[0].set_title("Modality Share of Total |SHAP|", fontsize=12)

    plot_order = []
    for mod in ["EMG", "IMU", "Anthro"]:
        idx = np.where(grouped_modality == mod)[0]
        if len(idx) > 0:
            idx_sorted = idx[np.argsort(grouped_vals[idx])[::-1]]
            plot_order.extend(idx_sorted)
    
    plot_order = np.array(plot_order)
    plot_colors = [get_modality_color(grouped_names[i]) for i in plot_order]
    
    axes[1].barh(range(n_grouped), grouped_vals[plot_order][::-1], color=np.array(plot_colors)[::-1], edgecolor="white")
    axes[1].set_yticks(range(n_grouped))
    axes[1].set_yticklabels(grouped_names[plot_order][::-1], fontsize=9)
    axes[1].set_xlabel("Mean |SHAP value|")
    axes[1].set_title("Importance per Feature (Grouped)", fontsize=12)
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
             feat_types=np.array(list(feat_type_vals.keys())),
             feat_type_vals=np.array(list(feat_type_vals.values())),
             mean_abs_shap=mean_abs_shap)
    print(f"[DeepSHAP] Results saved to {out_dir}")

def main():
    print("Running standalone DeepSHAP analysis...")
    SAVED_MODEL_PATH = "model/model_results/run_20260418_180005/cnn_lstm_model.joblib"
    model_path = ROOT / SAVED_MODEL_PATH
    if not model_path.exists():
        print(f"Error: Model not found at {model_path}"); return

    loader = DataLoader()
    h5_paths = list(DATABASE_CONFIG["segments_dir"].glob("*.h5"))
    df = loader.load_raw_segments(h5_paths)
    X, y = loader.prepare_for_ml(df, target_col="weight")
    
    X_train, X_test, _, _ = train_test_split(X, y, test_size=0.2, random_state=42, stratify=df["weight"])
    regressor = CNNLSTMRegressor.load(model_path)
    run_deep_shap_analysis(regressor, X_train, X_test, model_path.parent)

if __name__ == "__main__":
    main()
