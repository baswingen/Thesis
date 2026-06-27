#!/usr/bin/env python3
"""
attention_chord_signal_combined.py
──────────────────────────────────
Combined visualization of raw sensor signals (left) and spatial attention chord (right).
A sliding window sweeps across the signals showing the exact data span used to extract
features for the ST-Transformer3 model at each time step.

Usage
-----
    python -m visualization.attention_chord_signal_combined --segment segment_0009 --imu 1
"""
from __future__ import annotations

import argparse
import io
import os
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import matplotlib.patches as patches
from PIL import Image

# Setup regime models and strategy
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
_ap.add_argument("--gif", action="store_true", default=True, help="assemble an animated GIF")
_ap.add_argument("--fps", type=float, default=2.0, help="GIF frames per second")
ARGS = _ap.parse_args()

# Setup paths and imports
os.environ.setdefault("THESIS_CV_STRATEGY", REGIME[ARGS.regime][1])
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import torch
import torch.nn as nn

from model.model_archs.spatio_temporal_transformer3 import SpatioTemporalTransformerRegressor3
from model.data_loader import DataLoader
from model.config_model import USE_PRECOMPUTED_FEATURES
from visualization.plot_segment_signals import (
    load_segment, EMG_COLORS, AXIS_COLORS, compute_fair_limits,
    DEFAULT_FILE, FALLBACK_FILE
)
from visualization.attention_chord import (
    node_spec, draw_chord, TITLE_FONT, BODY_FONT,
)
from visualization.attention_chord_temporal import capture_spatial_per_window


def main():
    # Resolve file path
    file_path = ARGS.file
    if not Path(file_path).exists():
        if Path(FALLBACK_FILE).exists():
            file_path = FALLBACK_FILE
        else:
            possible_paths = list(Path(__file__).resolve().parents[2].glob("**/participant_P01_session_01_segments.h5"))
            if possible_paths:
                file_path = str(possible_paths[0])
            else:
                raise FileNotFoundError(f"Could not find segment H5 file at {ARGS.file} or fallbacks.")

    model_path, _ = REGIME[ARGS.regime]
    reg = SpatioTemporalTransformerRegressor3.load(model_path)
    chan = list(reg.model.channel_names)
    idx_of = {c: i for i, c in enumerate(chan)}

    spec = node_spec(chan, ARGS.channels, ARGS.imu)
    sel = [idx_of[s[0]] for s in spec]

    # Load data for limits computation and this segment
    from visualization.plot_segment_signals import get_segments_per_weight
    weights_map = get_segments_per_weight(file_path)
    segments_for_limits = list(weights_map.values()) if weights_map else [ARGS.segment]
    emg_lim, orient_lim, accel_lim = compute_fair_limits(file_path, ARGS.imu, segments_for_limits)

    D = load_segment(file_path, ARGS.segment, ARGS.imu)
    dur = float(D["t_emg"][-1])
    meta = D["meta"]

    loader = DataLoader()
    df = loader.load_and_extract_features([Path(file_path)], is_sequence=True,
                                          use_precomputed=USE_PRECOMPUTED_FEATURES)
    row = df[df["segment_id"] == ARGS.segment]
    if row.empty:
        sys.exit(f"Segment {ARGS.segment} not found. Try another --segment.")
    X, _ = loader.prepare_for_ml(row, target_col="weight")

    spatial_TCC, pred = capture_spatial_per_window(reg, X)        # [T, 20, 20]
    spatial_TCC = spatial_TCC[:, sel][:, :, sel]                  # [T, C, C]
    T, C = spatial_TCC.shape[0], spatial_TCC.shape[1]

    # Symmetrize per frame, zero diagonal
    Ssym = (spatial_TCC + spatial_TCC.transpose(0, 2, 1)) / 2.0
    for t in range(T):
        np.fill_diagonal(Ssym[t], 0.0)

    # Dramatised per-edge intensity (matching attention_chord_temporal.py)
    emin = Ssym.min(axis=0)                       # [C, C]
    emax = Ssym.max(axis=0)
    gmax = Ssym.max() or 1.0
    tnorm = (Ssym - emin) / (emax - emin + 1e-9)  # [T,C,C] each edge spans 0..1
    absstr = Ssym / gmax                          # [T,C,C] absolute strength 0..1
    intensity = (0.16 + 0.84 * tnorm) * (0.45 + 0.55 * absstr)   # [T,C,C] in ~[0,1]

    # Output directory setup
    out_dir = (Path(__file__).resolve().parent / "segment_signal_plots"
               / f"attention_chord_combined_{ARGS.segment}_imu{ARGS.imu}_{ARGS.regime}")
    out_dir.mkdir(parents=True, exist_ok=True)
    # Clear stale files
    for pat in ("frame_*.png", "frame_*.svg", "frame_*.pdf"):
        for old in out_dir.glob(pat):
            old.unlink()

    gif_buffers = []
    cross_def = "super" if ARGS.channels == "signal" else "three"
    style = "gradient"
    wide = (ARGS.channels == "signal")

    # Define window properties (matching model configs)
    emg_window_size_sec = 0.15
    imu_window_size_sec = 0.20
    window_step_sec = 0.10

    # Layout styling helper
    def apply_signal_style(ax):
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.grid(True, axis="y", color="#E6E6E6", linewidth=0.8, zorder=0)
        ax.tick_params(length=3, color="#888888")
        ax.margins(x=0.005)

    for t in range(T):
        t_end = 0.20 + t * window_step_sec

        # Create combined figure side-by-side: Signals on left, Chord on right
        fig = plt.figure(figsize=(16, 10))
        gs = fig.add_gridspec(5, 2, width_ratios=[1.0, 1.1], hspace=0.38, wspace=0.25)
        
        # Left side: 5 stacked signal panels
        axes_sig = [fig.add_subplot(gs[i, 0]) for i in range(5)]
        for i in range(1, 5):
            axes_sig[i].sharex(axes_sig[0])

        # Right side: 1 chord diagram spanning all rows
        ax_chord = fig.add_subplot(gs[:, 1])

        # ── 1. Plot Signals on the Left ──
        # EMG panels (1–3)
        for ax, (muscle, color) in zip(axes_sig[:3], EMG_COLORS.items()):
            ax.plot(D["t_emg"], D["emg"][muscle], color=color, lw=0.8, zorder=3)
            ax.axhline(0, color="#BBBBBB", lw=0.6, zorder=1)
            ax.set_ylabel("EMG\n(mV)", fontsize=11, fontfamily=BODY_FONT)
            ax.set_ylim(-emg_lim, emg_lim)
            ax.set_title(muscle, loc="left", fontfamily=TITLE_FONT,
                         fontweight="bold", color=color, fontsize=13.5, pad=4)
            apply_signal_style(ax)

        # IMU orientation panel (4)
        ax_orient = axes_sig[3]
        u = D["imu_unit"]
        orient_labels = {
            "roll": rf"$\phi_{u}$",
            "pitch": rf"$\theta_{u}$",
            "yaw": rf"$\psi_{u}$",
        }
        for (axis, sig), c in zip(D["orient"].items(), AXIS_COLORS):
            ax_orient.plot(D["t_imu"], sig, color=c, lw=1.4, label=orient_labels.get(axis, axis), zorder=3)
        ax_orient.set_ylabel("Orientation\n(°)", fontsize=11, fontfamily=BODY_FONT)
        ax_orient.set_ylim(-orient_lim, orient_lim)
        ax_orient.set_title("IMU Orientation", loc="left",
                            fontfamily=TITLE_FONT, fontweight="bold", color="#2C497F", fontsize=13.5, pad=4)
        apply_signal_style(ax_orient)
        # Framed legend for orientation
        leg_o = ax_orient.legend(ncol=3, loc="upper right", fontsize=10.5, handlelength=1.3,
                                 columnspacing=1.2, borderaxespad=0.3, frameon=True,
                                 framealpha=0.9, edgecolor="#DDDDDD", fancybox=False)
        leg_o.get_frame().set_facecolor("white")
        leg_o.get_frame().set_linewidth(0.8)
        ax_orient.margins(y=0.18)

        # IMU acceleration panel (5)
        ax_accel = axes_sig[4]
        for (axis, sig), c in zip(D["accel"].items(), AXIS_COLORS):
            ax_accel.plot(D["t_imu"], sig, color=c, lw=1.4, label=f"a$_{axis}$", zorder=3)
        ax_accel.set_ylabel("Lin. accel.\n(g)", fontsize=11, fontfamily=BODY_FONT)
        ax_accel.set_ylim(-accel_lim, accel_lim)
        ax_accel.set_title("IMU Linear Acceleration", loc="left",
                            fontfamily=TITLE_FONT, fontweight="bold", color="#10C46B", fontsize=13.5, pad=4)
        apply_signal_style(ax_accel)
        # Framed legend for acceleration
        leg_a = ax_accel.legend(ncol=3, loc="upper right", fontsize=10.5, handlelength=1.3,
                                 columnspacing=1.2, borderaxespad=0.3, frameon=True,
                                 framealpha=0.9, edgecolor="#DDDDDD", fancybox=False)
        leg_a.get_frame().set_facecolor("white")
        leg_a.get_frame().set_linewidth(0.8)
        ax_accel.margins(y=0.18)

        # Bottom signal x-axis
        axes_sig[-1].set_xlabel("Time (s)", fontsize=13, fontfamily=BODY_FONT)
        axes_sig[-1].set_xlim(0.0, dur)

        # ── 2. Draw Sliding Windows on Signals ──
        # EMG sliding window (0.15s)
        t_start_emg = max(0.0, t_end - emg_window_size_sec)
        for ax in axes_sig[:3]:
            ax.axvspan(t_start_emg, t_end, color="#2C497F", alpha=0.18, zorder=2)
            ax.axvline(t_start_emg, color="#2C497F", linestyle="--", linewidth=1.0, alpha=0.5, zorder=2)
            ax.axvline(t_end, color="#2C497F", linestyle="--", linewidth=1.0, alpha=0.5, zorder=2)

        # IMU sliding window (0.20s)
        t_start_imu = max(0.0, t_end - imu_window_size_sec)
        for ax in axes_sig[3:]:
            ax.axvspan(t_start_imu, t_end, color="#2C497F", alpha=0.18, zorder=2)
            ax.axvline(t_start_imu, color="#2C497F", linestyle="--", linewidth=1.0, alpha=0.5, zorder=2)
            ax.axvline(t_end, color="#2C497F", linestyle="--", linewidth=1.0, alpha=0.5, zorder=2)

        # ── 3. Draw Chord Diagram on the Right ──
        draw_chord(ax_chord, Ssym[t], spec, style=style, cross_def=cross_def, wide=wide,
                   intensity=intensity[t], legend=False, lw_max=12.0, lw_power=2.0)

        # ── 4. Draw Load Badge ──
        # Load badge text axes (transparent, overlaying the background patch)
        badge = fig.add_axes([0.42, 0.925, 0.16, 0.045])
        badge.axis("off")

        fig_w, fig_h = fig.get_size_inches()
        badge_x = 0.42 * fig_w
        badge_y = 0.925 * fig_h
        badge_w = 0.16 * fig_w
        badge_h = 0.045 * fig_h
        
        badge_patch = patches.FancyBboxPatch(
            (badge_x, badge_y), badge_w, badge_h,
            transform=fig.dpi_scale_trans,
            boxstyle="round,pad=0,rounding_size=0.08",
            facecolor="#373544", edgecolor="none",
            zorder=100, clip_on=False
        )
        badge.add_patch(badge_patch)

        badge.text(0.12, 0.5, "LOAD", transform=badge.transAxes, va="center", ha="left",
                   fontfamily=TITLE_FONT, fontweight="bold", fontsize=11,
                   color="#C2C0D4", zorder=101)
        badge.text(0.88, 0.5, f"{meta['true_weight']:.2f} kg", transform=badge.transAxes,
                   va="center", ha="right", fontfamily=TITLE_FONT, fontweight="bold",
                   fontsize=18, color="white", zorder=101)

        # Adjust margins
        fig.subplots_adjust(left=0.06, right=0.96, top=0.90, bottom=0.07)

        # Save current frame to disk
        fig.savefig(out_dir / f"frame_{t:02d}.png", dpi=150)
        fig.savefig(out_dir / f"frame_{t:02d}.svg")
        fig.savefig(out_dir / f"frame_{t:02d}.pdf")

        if ARGS.gif:
            buf = io.BytesIO()
            fig.savefig(buf, format="png", dpi=150)
            buf.seek(0)
            gif_buffers.append(Image.open(buf).convert("RGB"))

        plt.close(fig)

    print(f"Saved {T} frames (PNG/SVG/PDF) → {out_dir}")
    if ARGS.gif and gif_buffers:
        gif_path = out_dir / f"attention_chord_combined_{ARGS.segment}_imu{ARGS.imu}.gif"
        gif_buffers[0].save(gif_path, save_all=True, append_images=gif_buffers[1:],
                            duration=int(1000 / ARGS.fps), loop=0)
        print(f"GIF compiled → {gif_path}")


if __name__ == "__main__":
    main()
