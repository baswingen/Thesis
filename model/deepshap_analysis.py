"""
deepshap_analysis.py
====================
Standalone DeepSHAP channel-importance analysis for the trained CNN-LSTM model.

Usage
-----
Run AFTER training with run_model.py. Point SAVED_MODEL_PATH at the
.joblib file saved in model/model_results/run_<timestamp>/.

    python -m model.deepshap_analysis

The script reuses the exact same data-loading pipeline used in run_model.py
(same DataLoader, same CHANNEL_CONFIG, same PARTICIPANT_CONFIG, same
train/test split with random_state=42 and stratification by weight).

Outputs (saved next to the model file in its run directory):
  - deepshap_channel_importance.png   Bar chart of mean |SHAP| per channel
  - deepshap_modality_split.png       EMG vs IMU importance pie/bar
  - deepshap_values.npz               Raw SHAP arrays for further analysis
"""

import sys
import numpy as np
import torch
import torch.nn as nn
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from pathlib import Path
from sklearn.model_selection import train_test_split

# ---------------------------------------------------------------------------
# Project root on sys.path so 'model' package is importable
# ---------------------------------------------------------------------------
ROOT = Path(__file__).resolve().parent.parent
sys.path.append(str(ROOT))

from model.data_loader import DataLoader
from model.model_archs.cnn_lstm import CNNLSTMRegressor, CNNLSTMNetwork
from model.config_model import CHANNEL_CONFIG, PARTICIPANT_CONFIG, DATABASE_CONFIG

###########################################################
# CONFIGURATION — edit these before running
###########################################################

# Path to a saved CNN-LSTM model (.joblib produced by run_model.py)
SAVED_MODEL_PATH: str | None = "model/model_results/run_20260404_121932/cnn_lstm_model.joblib"  # e.g. "model/model_results/run_20260331_120000/cnn_lstm_model.joblib"
#  → set to None to train a fresh lightweight model on the fly (slow but convenient)

# Number of background samples fed to DeepExplainer (30-100 is plenty)
N_BACKGROUND: int = 50

# Number of test samples to explain
N_EXPLAIN: int = 40

# Output directory (defaults to the same folder as the saved model, or model/model_results/)
OUTPUT_DIR: str | None = None  # e.g. "model/model_results/shap_results"

###########################################################


# ---------------------------------------------------------------------------
# SHAP-compatible wrapper — removes the 'lengths' argument from forward()
# ---------------------------------------------------------------------------

class CNNLSTMForSHAP(nn.Module):
    """
    Thin wrapper so shap.DeepExplainer can call forward(x) with a single
    tensor.  Lengths are reconstructed from the non-zero time-step mask.
    """
    def __init__(self, network: CNNLSTMNetwork):
        super().__init__()
        self.network = network

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (batch, n_channels, time_steps)  — may contain zero-padded frames
        nonzero_mask = (x.abs().sum(dim=1) > 1e-8)   # (batch, T)
        lengths = nonzero_mask.sum(dim=1).clamp(min=1)  # (batch,)
        return self.network(x, lengths)


# ---------------------------------------------------------------------------
# Helper: pad a list of (T, C) arrays to a single (N, C, T_max) tensor
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


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    import shap  # imported here so the rest of the script is importable without shap

    # ── Resolve paths ───────────────────────────────────────────────────────
    segments_dir = DATABASE_CONFIG["segments_dir"]
    results_dir  = ROOT / "model" / "model_results"

    model_path = Path(SAVED_MODEL_PATH) if SAVED_MODEL_PATH else None
    if model_path and not model_path.is_absolute():
        model_path = ROOT / model_path

    if OUTPUT_DIR:
        out_dir = Path(OUTPUT_DIR) if Path(OUTPUT_DIR).is_absolute() else ROOT / OUTPUT_DIR
    elif model_path:
        out_dir = model_path.parent
    else:
        out_dir = results_dir / "shap_results"
    out_dir.mkdir(parents=True, exist_ok=True)

    # ── Load data (mirrors run_model.py exactly) ────────────────────────────
    h5_paths = list(segments_dir.glob("*.h5"))
    if not h5_paths:
        print(f"[ERROR] No HDF5 files found in {segments_dir}"); return

    print(f"Found {len(h5_paths)} HDF5 file(s).")
    loader = DataLoader()
    df = loader.load_raw_segments(h5_paths)

    if df.empty:
        print("[ERROR] Data loading returned an empty DataFrame."); return

    # Recover channel names stored by the loader
    channel_names: list[str] = df.attrs.get("channel_names", [])

    X, y = loader.prepare_for_ml(df, target_col="weight")

    # Same stratified split as run_model.py (random_state=42, test_size=0.2)
    stratify = df["weight"].astype(str) if "weight" in df.columns else None
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42, stratify=stratify
    )
    print(f"Split: {len(X_train)} train / {len(X_test)} test samples.")

    # ── Load or train the model ─────────────────────────────────────────────
    if model_path and model_path.exists():
        print(f"Loading saved model from {model_path} …")
        regressor = CNNLSTMRegressor.load(model_path)
    else:
        if model_path:
            print(f"[WARN] Model not found at {model_path}. Training a fresh model …")
        else:
            print("No model path provided. Training a fresh model …")

        from model.config_model import CNN_LSTM_CONFIG
        regressor = CNNLSTMRegressor(**CNN_LSTM_CONFIG)
        regressor.fit(X_train, y_train)

    regressor.model.eval()
    device = regressor.device
    print(f"Model on device: {device}")

    # ── Prepare background and explanation tensors ──────────────────────────
    n_bg  = min(N_BACKGROUND, len(X_train))
    n_exp = min(N_EXPLAIN,    len(X_test))

    # Background: random sample from train set
    rng = np.random.default_rng(42)
    bg_idx  = rng.choice(len(X_train), size=n_bg,  replace=False)
    exp_idx = rng.choice(len(X_test),  size=n_exp, replace=False)

    bg_segs  = regressor._extract_raw_segments(X_train.iloc[bg_idx])
    exp_segs = regressor._extract_raw_segments(X_test.iloc[exp_idx])

    bg_scaled  = regressor._scale_segments(bg_segs,  fit=False)
    exp_scaled = regressor._scale_segments(exp_segs, fit=False)

    bg_tensor  = pad_to_tensor(bg_scaled,  device)
    exp_tensor = pad_to_tensor(exp_scaled, device)

    # DeepExplainer concatenates bg and exp tensors internally, so they must
    # share the same time dimension. Pad the shorter one to match the longer.
    max_t = max(bg_tensor.shape[2], exp_tensor.shape[2])
    def _pad_time(t: torch.Tensor, target_t: int) -> torch.Tensor:
        pad = target_t - t.shape[2]
        if pad > 0:
            t = torch.nn.functional.pad(t, (0, pad))  # pad last dim (time)
        return t
    bg_tensor  = _pad_time(bg_tensor,  max_t)
    exp_tensor = _pad_time(exp_tensor, max_t)

    print(f"Background tensor : {tuple(bg_tensor.shape)}")
    print(f"Explanation tensor: {tuple(exp_tensor.shape)}")

    # ── Build SHAP-compatible wrapper ───────────────────────────────────────
    # Force CPU for SHAP: MPS (Apple Silicon) uses lower float precision which
    # causes SHAP's additivity check to fail (diff ~4 vs tolerance 0.01).
    shap_device = torch.device("cpu")
    shap_model = CNNLSTMForSHAP(regressor.model).to(shap_device)
    shap_model.eval()

    bg_tensor_cpu  = bg_tensor.to(shap_device)
    exp_tensor_cpu = exp_tensor.to(shap_device)

    # ── DeepExplainer ───────────────────────────────────────────────────────
    from tqdm import tqdm
    
    print(f"\nRunning shap.DeepExplainer with {n_bg} background and {n_exp} test samples …")
    explainer = shap.DeepExplainer(shap_model, bg_tensor_cpu)
    
    # Process in chunks to provide a tqdm progress bar and reduce peak RAM
    batch_size = 5
    all_shap_values = []
    
    for i in tqdm(range(0, len(exp_tensor_cpu), batch_size), desc="Computing SHAP", unit="batch"):
        batch_exp = exp_tensor_cpu[i : i + batch_size]
        batch_shap = explainer.shap_values(batch_exp, check_additivity=False)
        
        if isinstance(batch_shap, list):
            batch_shap = batch_shap[0]
            
        batch_shap = np.array(batch_shap)
        
        # Squeeze rogue dimensions if necessary
        if batch_shap.ndim == 4:
            if batch_shap.shape[-1] == 1:
                batch_shap = batch_shap[..., 0]
            elif batch_shap.shape[0] == 1:
                batch_shap = batch_shap[0]
                
        all_shap_values.append(batch_shap)
        
    shap_values = np.concatenate(all_shap_values, axis=0) # (N, C, T)
        
    print(f"SHAP values shape: {shap_values.shape}")  # (n_exp, n_channels, T)

    # ── Aggregate: sum |SHAP| across time, then mean over test samples → (n_channels,) ─────
    mean_abs_shap = np.abs(shap_values).sum(axis=2).mean(axis=0)  # Total channel importance per sample

    n_channels = mean_abs_shap.shape[0]

    # Fall back to generic names if channel_names wasn't captured
    if not channel_names or len(channel_names) != n_channels:
        emg_cfg = CHANNEL_CONFIG.get('emg_channels', {})
        imu_cfg = CHANNEL_CONFIG.get('imu_channels', {})
        channel_names = (
            [k for k, v in emg_cfg.items() if v]
            + [k for k, v in imu_cfg.items() if v]
        )
        if len(channel_names) != n_channels:
            channel_names = [f"ch{i}" for i in range(n_channels)]

    # Determine EMG / IMU boundary
    n_emg = sum(1 for v in CHANNEL_CONFIG.get('emg_channels', {}).values() if v)
    emg_mask = np.array([i < n_emg for i in range(n_channels)])
    imu_mask = ~emg_mask

    # ── Plot 1: Bar chart sorted by importance ──────────────────────────────
    order = np.argsort(mean_abs_shap)[::-1]
    colors = ["#E05C5C" if emg_mask[i] else "#5C9BE0" for i in order]

    fig, ax = plt.subplots(figsize=(max(12, n_channels * 0.55), 6))
    bars = ax.bar(range(n_channels), mean_abs_shap[order], color=colors, edgecolor="white", linewidth=0.5)
    ax.set_xticks(range(n_channels))
    ax.set_xticklabels(
        [channel_names[i] for i in order],
        rotation=45, ha="right", fontsize=9
    )
    ax.set_ylabel("Mean |SHAP value|", fontsize=11)
    ax.set_title("DeepSHAP Channel Importance — CNN-LSTM", fontsize=13, fontweight="bold")
    ax.set_xlabel("Channel (sorted by importance)", fontsize=11)
    ax.spines[["top", "right"]].set_visible(False)

    # Legend
    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor="#E05C5C", label="EMG"),
                       Patch(facecolor="#5C9BE0", label="IMU")]
    ax.legend(handles=legend_elements, loc="upper right")

    plt.tight_layout()
    p1 = out_dir / "deepshap_channel_importance.png"
    fig.savefig(p1, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {p1}")

    # ── Plot 2: EMG vs IMU modality split ───────────────────────────────────
    emg_total = mean_abs_shap[emg_mask].sum()
    imu_total = mean_abs_shap[imu_mask].sum()
    total     = emg_total + imu_total

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # Pie
    axes[0].pie(
        [emg_total, imu_total],
        labels=[f"EMG\n{emg_total/total*100:.1f}%", f"IMU\n{imu_total/total*100:.1f}%"],
        colors=["#E05C5C", "#5C9BE0"],
        startangle=90, autopct="%1.1f%%",
        wedgeprops=dict(edgecolor="white", linewidth=2),
    )
    axes[0].set_title("Modality Share of Total |SHAP|", fontsize=12)

    # Per-channel bar within each modality
    emg_names  = [channel_names[i] for i in range(n_channels) if emg_mask[i]]
    imu_names  = [channel_names[i] for i in range(n_channels) if imu_mask[i]]
    emg_shap   = mean_abs_shap[emg_mask]
    imu_shap   = mean_abs_shap[imu_mask]

    emg_order  = np.argsort(emg_shap)[::-1]
    imu_order  = np.argsort(imu_shap)[::-1]

    all_names  = [emg_names[i] for i in emg_order] + [imu_names[i] for i in imu_order]
    all_vals   = np.concatenate([emg_shap[emg_order], imu_shap[imu_order]])
    all_colors = ["#E05C5C"] * len(emg_names) + ["#5C9BE0"] * len(imu_names)

    axes[1].barh(range(len(all_names)), all_vals[::-1], color=all_colors[::-1], edgecolor="white")
    axes[1].set_yticks(range(len(all_names)))
    axes[1].set_yticklabels(all_names[::-1], fontsize=8)
    axes[1].set_xlabel("Mean |SHAP value|")
    axes[1].set_title("Per-Channel Importance by Modality", fontsize=12)
    axes[1].spines[["top", "right"]].set_visible(False)

    plt.tight_layout()
    p2 = out_dir / "deepshap_modality_split.png"
    fig.savefig(p2, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {p2}")

    # ── Save raw SHAP arrays ─────────────────────────────────────────────────
    p3 = out_dir / "deepshap_values.npz"
    np.savez(p3,
             shap_values=shap_values,
             channel_names=np.array(channel_names),
             mean_abs_shap=mean_abs_shap,
             n_emg=np.array(n_emg))
    print(f"Saved: {p3}")

    # ── Console summary ──────────────────────────────────────────────────────
    print("\n" + "=" * 55)
    print("CHANNEL IMPORTANCE RANKING (Mean |SHAP|)")
    print("=" * 55)
    for rank, idx in enumerate(order, 1):
        modality = "EMG" if emg_mask[idx] else "IMU"
        print(f"  {rank:>2}. [{modality}] {channel_names[idx]:<40s} {mean_abs_shap[idx]:.6f}")

    print("\n--- Modality Summary ---")
    print(f"  EMG total importance : {emg_total:.6f}  ({emg_total/total*100:.1f}%)")
    print(f"  IMU total importance : {imu_total:.6f}  ({imu_total/total*100:.1f}%)")
    print("=" * 55)
    print(f"\nAll outputs saved to: {out_dir}")


if __name__ == "__main__":
    main()
