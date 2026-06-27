#!/usr/bin/env python3
"""
attention_temporal_window.py
────────────────────────────
Temporal-attention companion to the spatial chord animation. As the **current
feature window** sweeps across the lift, each modality group (EMG / IMU-Acc /
IMU-Ori) shows the attention that window places on the *past* windows it can see
(the causal row of its T×T temporal-attention map). Different groups attend to
different moments → "different variants across channels".

In st3 each modality group has its own temporal MHA (channels within a group
share it); we average the per-channel maps within a group and over the 4 ST
blocks. One forward pass on the segment, no retraining.

Writes SVG frames + an animated GIF to a folder, mirroring attention_chord_temporal.py.

Usage
-----
    python -m visualization.attention_temporal_window --segment segment_0009
    python -m visualization.attention_temporal_window --segment segment_0238 --regime participant
"""
from __future__ import annotations

import argparse
import io
import os
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.colors import to_rgb
from PIL import Image

REGIME = {
    "kfold":       ("model/model_results/final_run_par_spec/all/spatio_temporal_transformer3_model.joblib", "kfold"),
    "participant": ("model/model_results/final_run_lopo_2/all/spatio_temporal_transformer3_model.joblib", "participant"),
}
REGIME_LABEL = {"kfold": "Specialized (k-fold)", "participant": "Generalized (LOPO)"}

_ap = argparse.ArgumentParser(description=__doc__)
_ap.add_argument("--regime", default="kfold", choices=["kfold", "participant"])
_ap.add_argument("--file", default="/Volumes/Laurens SSD/BasData/segments/participant_P01_session_01_segments.h5")
_ap.add_argument("--segment", default="segment_0009")
_ap.add_argument("--imu", type=int, default=1, choices=[1, 2])
_ap.add_argument("--channels", default="signal", choices=["all", "signal"])
_ap.add_argument("--gif", action="store_true", default=True)
_ap.add_argument("--fps", type=float, default=2.0)
ARGS = _ap.parse_args()

os.environ.setdefault("THESIS_CV_STRATEGY", REGIME[ARGS.regime][1])
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import torch
import torch.nn as nn

from model.model_archs.spatio_temporal_transformer3 import SpatioTemporalTransformerRegressor3
from model.data_loader import DataLoader
from model.config_model import USE_PRECOMPUTED_FEATURES
from visualization.plot_segment_signals import load_segment
from visualization.attention_chord import VIVID_MOD, BODY_FONT, TITLE_FONT, node_spec

# model group name → (display label, colour). Lane order mirrors the signal
# figure: EMG, then IMU orientation, then IMU linear acceleration.
GROUPS = [("EMG", "EMG", VIVID_MOD["EMG"]),
          ("IMU_Orient", "IMU Orientation", VIVID_MOD["IMU-Ori"]),
          ("IMU_Accel", "IMU Linear Acceleration", VIVID_MOD["IMU-Acc"])]


def capture_temporal_per_channel(reg, X):
    """Return {channel_name: [T, T]} causal temporal attention (avg over blocks) + pred."""
    caps: dict[str, list] = {}

    def mk(g):
        def hook(_m, _i, out):
            w = out[1]                                  # [B*n_ch, T, T], B=1
            caps.setdefault(g, []).append(w.detach().float().cpu().numpy())
        return hook

    handles = []
    for n, m in reg.model.named_modules():
        if ".temporal_attns." in n and isinstance(m, nn.MultiheadAttention):
            handles.append(m.register_forward_hook(mk(n.split(".")[-1])))
    with torch.no_grad():
        pred = float(np.array(reg.predict(X)).ravel()[0])
    for h in handles:
        h.remove()
        
    channel_attns = {}
    for g, lst in caps.items():
        # lst shape: [num_layers, n_ch, T, T] (since B=1, B*n_ch is n_ch)
        avg_g = np.mean(np.stack(lst), axis=0) # [n_ch, T, T]
        ch_indices = reg.model.modality_groups[g]
        for idx_in_group, ch_idx in enumerate(ch_indices):
            ch_name = reg.model.channel_names[ch_idx]
            channel_attns[ch_name] = avg_g[idx_in_group]
    return channel_attns, pred


def get_channel_style(ch_name, group_key, idx_in_group, total_in_group):
    """Return (label, color) with a gradient within each modality group."""
    from visualization.attention_chord import short_label
    import matplotlib.cm as cm
    
    lbl = short_label(ch_name)
    if total_in_group <= 1:
        frac = 0.5
    else:
        # Avoid the very light/yellow ends of the colormap for readability
        frac = 0.25 + 0.70 * (idx_in_group / (total_in_group - 1))
        
    if group_key == "EMG":
        color = cm.YlOrRd(frac)
    elif group_key == "IMU_Orient":
        color = cm.cool(frac)
    elif group_key == "IMU_Accel":
        color = cm.YlGn(frac)
    else:
        color = "#888888"
        
    return lbl, color


def smooth_curve(x, y, num_points=200):
    """Interpolate (x, y) with a cubic spline for smooth visualization."""
    if len(x) < 3:
        return x, y
    from scipy.interpolate import make_interp_spline
    x_smooth = np.linspace(x[0], x[-1], num_points)
    k = min(3, len(x) - 1)
    spl = make_interp_spline(x, y, k=k)
    y_smooth = spl(x_smooth)
    y_smooth = np.clip(y_smooth, 0.0, None)
    return x_smooth, y_smooth


def grad_area(ax, x, y, color, alpha=0.30, N=60):
    """Filled area under (x, y) with a vertical opacity gradient (opaque base → clear top)
    using pure vector stacked fill_between, smoothed for visual fluidity.
    """
    if len(x) < 2:
        if len(x):
            ax.plot(x, y, "o", color=color, ms=6, zorder=5, markeredgecolor="white", markeredgewidth=0.8)
        return
    
    # Generate smoothed curve for the area and outline
    x_smooth, y_smooth = smooth_curve(x, y)
    
    for i in range(1, N + 1):
        ax.fill_between(x_smooth, 0, y_smooth * (i / N), color=color, alpha=alpha / N, lw=0, zorder=2)
    ax.plot(x_smooth, y_smooth, color=color, lw=2.0, zorder=3, solid_capstyle="round")
    
    # Plot markers at each discrete attention timestep
    ax.plot(x, y, "o", color=color, ms=4.5, zorder=5, markeredgecolor="white", markeredgewidth=0.8)


def main():
    model_path, _ = REGIME[ARGS.regime]
    reg = SpatioTemporalTransformerRegressor3.load(model_path)

    spec = node_spec(reg.model.channel_names, ARGS.channels, ARGS.imu)
    spec_map = {s[0]: (s[1], s[2]) for s in spec}

    loader = DataLoader()
    df = loader.load_and_extract_features([Path(ARGS.file)], is_sequence=True,
                                          use_precomputed=USE_PRECOMPUTED_FEATURES)
    row = df[df["segment_id"] == ARGS.segment]
    if row.empty:
        sys.exit(f"Segment {ARGS.segment} not found (maybe blacklisted). Try another --segment.")
    X, _ = loader.prepare_for_ml(row, target_col="weight")
    channel_attns, pred = capture_temporal_per_channel(reg, X)

    from matplotlib.lines import Line2D

    emg_h = []
    orient_h = []
    accel_h = []
    for gkey, _, _ in GROUPS:
        ch_names = [reg.model.channel_names[idx] for idx in reg.model.modality_groups[gkey]]
        if ARGS.channels == "signal":
            ch_names = [c for c in ch_names if c in spec_map]
        n_ch = len(ch_names)
        for idx_in_group, ch_name in enumerate(ch_names):
            if ARGS.channels == "signal":
                lbl, col = spec_map[ch_name]
            else:
                lbl, col = get_channel_style(ch_name, gkey, idx_in_group, n_ch)
            h = Line2D([0], [0], marker="o", color=col, ms=5.0,
                       markeredgecolor="white", markeredgewidth=0.8,
                       lw=2.0, label=lbl)
            if gkey == "EMG":
                emg_h.append(h)
            elif gkey == "IMU_Orient":
                orient_h.append(h)
            elif gkey == "IMU_Accel":
                accel_h.append(h)

    legend_handles = []
    spacer = Line2D([0], [0], color="none", label="")
    
    if ARGS.channels == "signal":
        # 3 columns, each 3 items
        for r in range(3):
            legend_handles.append(emg_h[r] if r < len(emg_h) else spacer)
        for r in range(3):
            legend_handles.append(orient_h[r] if r < len(orient_h) else spacer)
        for r in range(3):
            legend_handles.append(accel_h[r] if r < len(accel_h) else spacer)
        ncol = 3
    else:
        # 6 columns in total: 2 for EMG, 2 for IMU Orientation, 2 for IMU Acceleration (4 rows total height)
        for r in range(4):
            legend_handles.append(emg_h[r] if r < len(emg_h) else spacer)
        for r in range(4):
            legend_handles.append(emg_h[r + 4] if (r + 4) < len(emg_h) else spacer)
        for r in range(3):
            legend_handles.append(orient_h[r] if r < len(orient_h) else spacer)
        legend_handles.append(spacer)
        for r in range(3):
            legend_handles.append(orient_h[r + 3] if (r + 3) < len(orient_h) else spacer)
        legend_handles.append(spacer)
        for r in range(3):
            legend_handles.append(accel_h[r] if r < len(accel_h) else spacer)
        legend_handles.append(spacer)
        for r in range(3):
            legend_handles.append(accel_h[r + 3] if (r + 3) < len(accel_h) else spacer)
        legend_handles.append(spacer)
        ncol = 6

    D = load_segment(ARGS.file, ARGS.segment, 1)
    dur, meta = float(D["t_emg"][-1]), D["meta"]
    T = next(iter(channel_attns.values())).shape[0]
    win_t = (np.arange(T) + 0.5) / T * dur               # window centre times (s)
    GAMMA = 1.25                                         # mild peak sharpening

    if ARGS.channels == "signal":
        out_dir = (Path(__file__).resolve().parent / "segment_signal_plots"
                   / f"attention_temporal_frames_{ARGS.segment}_imu{ARGS.imu}_{ARGS.regime}")
    else:
        out_dir = (Path(__file__).resolve().parent / "segment_signal_plots"
                   / f"attention_temporal_frames_{ARGS.segment}_all_{ARGS.regime}")

    out_dir.mkdir(parents=True, exist_ok=True)
    for pat in ("frame_*.png", "frame_*.svg", "frame_*.pdf"):
        for old in out_dir.glob(pat):
            old.unlink()

    gif_buffers = []
    for t in range(T):
        fig, axes = plt.subplots(3, 1, figsize=(8.8, 8.2), sharex=True,
                                 gridspec_kw={"hspace": 0.32})
        for ax, (gkey, glabel, gcol) in zip(axes, GROUPS):
            # Find channel indices/names for this group (all available channels)
            ch_names = [reg.model.channel_names[idx] for idx in reg.model.modality_groups[gkey]]
            if ARGS.channels == "signal":
                ch_names = [c for c in ch_names if c in spec_map]
            n_ch = len(ch_names)

            ax.axvspan(win_t[t], dur, color="#F4F4F4", zorder=0)      # unseen future
            ax.axvline(win_t[t], color=gcol, lw=1.3, ls=(0, (4, 3)), alpha=0.8, zorder=4)

            # 1. Pass 1: Fill all gradients first so they sit in the background
            for idx_in_group, ch_name in enumerate(ch_names):
                A = channel_attns[ch_name]
                row_t = A[t, :t + 1].astype(float)
                y = row_t / (row_t.max() + 1e-9)
                y = y ** GAMMA
                x = win_t[:t + 1]

                if ARGS.channels == "signal":
                    lbl, col = spec_map[ch_name]
                else:
                    lbl, col = get_channel_style(ch_name, gkey, idx_in_group, n_ch)
                if len(x) >= 2:
                    x_smooth, y_smooth = smooth_curve(x, y)
                    for i in range(1, 61):
                        ax.fill_between(x_smooth, 0, y_smooth * (i / 60), color=col, alpha=0.30 / 60, lw=0, zorder=2)

            # 2. Pass 2: Plot all lines and markers in front of all gradients
            for idx_in_group, ch_name in enumerate(ch_names):
                A = channel_attns[ch_name]
                row_t = A[t, :t + 1].astype(float)
                y = row_t / (row_t.max() + 1e-9)
                y = y ** GAMMA
                x = win_t[:t + 1]

                if ARGS.channels == "signal":
                    lbl, col = spec_map[ch_name]
                else:
                    lbl, col = get_channel_style(ch_name, gkey, idx_in_group, n_ch)
                if len(x) >= 2:
                    x_smooth, y_smooth = smooth_curve(x, y)
                    ax.plot(x_smooth, y_smooth, color=col, lw=2.0, zorder=3, solid_capstyle="round")
                    ax.plot(x, y, "o", color=col, ms=4.5, zorder=4, markeredgecolor="white", markeredgewidth=0.8)
                elif len(x):
                    ax.plot(x, y, "o", color=col, ms=6, zorder=4, markeredgecolor="white", markeredgewidth=0.8)

            ax.set_xlim(0, dur); ax.set_ylim(0, 1.15)
            ax.set_yticks([])
            ax.set_title(glabel, loc="left", fontfamily=TITLE_FONT, fontweight="bold",
                         color=gcol, fontsize=15, pad=5)
            for sp in ("top", "right", "left"):
                ax.spines[sp].set_visible(False)
            ax.tick_params(axis="x", length=3, color="#999999", labelsize=11)

        axes[-1].set_xlabel("Time into lift (s)", fontsize=12, fontfamily=BODY_FONT)

        # Add a timeline cursor and current time label on the shared x-axis of the bottom subplot
        last_ax = axes[-1]
        last_ax.plot(win_t[t], 0, "^", color="#333333", ms=7, clip_on=False,
                     transform=last_ax.get_xaxis_transform(), zorder=10)
        last_ax.text(win_t[t], -0.18, "now", color="#333333",
                     ha="center", va="top", fontsize=10, fontweight="bold",
                     transform=last_ax.get_xaxis_transform(), zorder=10)

        fig.subplots_adjust(left=0.035, right=0.975, top=0.94, bottom=0.18, hspace=0.42)
        fig.legend(handles=legend_handles, loc="lower center", ncol=ncol, frameon=False,
                   fontsize=9.0, handlelength=1.1, columnspacing=1.8, bbox_to_anchor=(0.5, 0.005))
        fig.savefig(out_dir / f"frame_{t:02d}.svg")
        fig.savefig(out_dir / f"frame_{t:02d}.pdf")
        if ARGS.gif:
            buf = io.BytesIO(); fig.savefig(buf, format="png", dpi=150); buf.seek(0)
            gif_buffers.append(Image.open(buf).convert("RGB"))
        plt.close(fig)

    print(f"Saved {T} SVG/PDF frames → {out_dir}")
    print(f"Segment {ARGS.segment}: true={meta['true_weight']:.2f} kg  pred={pred:.2f} kg | {T} windows")

    if ARGS.gif and gif_buffers:
        if ARGS.channels == "signal":
            gif_path = out_dir / f"attention_temporal_{ARGS.segment}_imu{ARGS.imu}.gif"
        else:
            gif_path = out_dir / f"attention_temporal_{ARGS.segment}.gif"
        gif_buffers[0].save(gif_path, save_all=True, append_images=gif_buffers[1:],
                            duration=int(1000 / ARGS.fps), loop=0)
        print(f"GIF → {gif_path}")


if __name__ == "__main__":
    main()
