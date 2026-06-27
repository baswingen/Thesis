#!/usr/bin/env python3
"""
attention_over_signal.py
────────────────────────
Presentation figure that paints the ST-Transformer3's **spatial** and
**temporal** attention onto the *actual* sensor signals of one lift segment:

  • TEMPORAL attention → a heat band behind the five signal panels + a ribbon
    at the bottom (per-modality), showing *when* in the lift the model focuses.
  • SPATIAL attention  → a per-channel importance strip at the top, colored by
    modality, showing *which* sensors drive the readout.

Both are captured with forward hooks on the trained model running on the exact
segment plotted — no retraining, no averaging across lifts.

Usage
-----
    python -m visualization.attention_over_signal                     # par-spec model, default segment
    python -m visualization.attention_over_signal --regime participant
    python -m visualization.attention_over_signal --segment segment_0057 --imu 1
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.gridspec import GridSpec

# Regime → (model joblib, CV strategy env). Set env BEFORE importing model/config.
REGIME = {
    "kfold":       ("model/model_results/final_run_par_spec/all/spatio_temporal_transformer3_model.joblib", "kfold"),
    "participant": ("model/model_results/final_run_lopo_2/all/spatio_temporal_transformer3_model.joblib", "participant"),
}
REGIME_LABEL = {"kfold": "Specialized (k-fold)", "participant": "Generalized (LOPO)"}

_ap = argparse.ArgumentParser(description=__doc__)
_ap.add_argument("--regime", default="kfold", choices=["kfold", "participant"])
_ap.add_argument("--file", default="/Volumes/Laurens SSD/BasData/segments/participant_P01_session_01_segments.h5")
_ap.add_argument("--segment", default="segment_0238")
_ap.add_argument("--imu", type=int, default=1, choices=[1, 2])
_ap.add_argument("--out", default=None)
ARGS = _ap.parse_args()

# CV strategy must match the model so the feature set matches what it was trained on.
os.environ.setdefault("THESIS_CV_STRATEGY", REGIME[ARGS.regime][1])

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import torch
import torch.nn as nn

from model.model_archs.spatio_temporal_transformer3 import SpatioTemporalTransformerRegressor3
from model.data_loader import DataLoader
from model.config_model import USE_PRECOMPUTED_FEATURES
from visualization.plot_segment_signals import (
    load_segment, EMG_COLORS, AXIS_COLORS, TRUE_WEIGHTS,
)

# ── Thesis fonts ───────────────────────────────────────────────────────────
FONTS_DIR = Path(__file__).resolve().parent / "fonts"
for _f in ("FiraSans-Bold.ttf", "FiraSans-SemiBold.ttf", "FiraSans-Regular.ttf",
           "STIXTwoText-VariableFont_wght.ttf", "STIXTwoMath-Regular.ttf"):
    p = FONTS_DIR / _f
    if p.exists():
        try:
            fm.fontManager.addfont(str(p))
        except Exception:
            pass
HAS_FIRA = any("Fira Sans" in f.name for f in fm.fontManager.ttflist)
HAS_STIX = any("STIX Two Text" in f.name for f in fm.fontManager.ttflist)
BODY_FONT = "STIX Two Text" if HAS_STIX else "serif"
TITLE_FONT = "Fira Sans" if HAS_FIRA else "sans-serif"
plt.rcParams.update({"font.family": BODY_FONT, "figure.facecolor": "white",
                     "savefig.facecolor": "white", "axes.edgecolor": "#444444",
                     "axes.linewidth": 0.9})

MOD_COLORS = {"EMG": "#E4572E", "IMU-Acc": "#137054", "IMU-Ori": "#2C497F"}
BAND_CMAP = LinearSegmentedColormap.from_list("band", ["#FFFFFF", "#2C497F"])
RIBBON = {"EMG": "#E4572E", "IMU_Accel": "#137054", "IMU_Orient": "#2C497F"}
RIBBON_LBL = {"EMG": "EMG", "IMU_Accel": "IMU-Acc", "IMU_Orient": "IMU-Ori"}

_EMG_ABBR = {"Anterior Deltoid": "A.Delt", "Lateral Deltoid": "L.Delt", "Posterior Deltoid": "P.Delt",
             "Triceps Brachii": "Triceps", "Biceps Brachii": "Biceps", "Brachioradialis": "Brachiorad",
             "Flexor Carpi Ulnaris (FCU)": "FCU", "Extensor Carpi Radialis (ECR)": "ECR"}
_IMU_ABBR = {"ax1": "aX1", "ay1": "aY1", "az1": "aZ1", "ax2": "aX2", "ay2": "aY2", "az2": "aZ2",
             "roll_rad1": "roll1", "pitch_rad1": "pitch1", "yaw_rad1": "yaw1",
             "roll_rad2": "roll2", "pitch_rad2": "pitch2", "yaw_rad2": "yaw2"}


def modality_of(ch):
    if "_EMG" in ch:
        return "EMG"
    if "_IMU" in ch and any(k in ch for k in ["ax", "ay", "az"]):
        return "IMU-Acc"
    return "IMU-Ori"


def short_label(ch):
    b = ch.replace("_EMG", "").replace("_IMU", "")
    return _EMG_ABBR.get(b, _IMU_ABBR.get(b, b))


# ── Attention extraction for ONE segment ───────────────────────────────────
def extract_one(reg, X):
    spatial, pooling, temporal = {}, [], {}

    def sphook(_m, _i, out):
        w = out[1] if isinstance(out, tuple) and len(out) > 1 else None
        if w is not None:
            spatial["s"] = w.detach().float().cpu().mean(0).numpy()

    def poolhook(_m, _i, out):
        w = out[1] if isinstance(out, tuple) and len(out) > 1 else None
        if w is not None:
            pooling.append(w.detach().float().cpu().squeeze(-1).mean(0).numpy())

    def thook(group):
        def h(_m, _i, out):
            w = out[1] if isinstance(out, tuple) and len(out) > 1 else None
            if w is not None:
                temporal[group] = w.detach().float().cpu().mean(0).numpy()  # [T, T]
        return h

    handles = []
    for name, mod in reg.model.named_modules():
        if name.endswith("spatial_attn") and isinstance(mod, nn.MultiheadAttention):
            handles.append(mod.register_forward_hook(sphook))
        elif name.endswith("attention_pooling"):
            handles.append(mod.register_forward_hook(poolhook))
        elif ".temporal_attns." in name and isinstance(mod, nn.MultiheadAttention):
            handles.append(mod.register_forward_hook(thook(name.split(".")[-1])))
    with torch.no_grad():
        pred = float(np.array(reg.predict(X)).ravel()[0])
    for h in handles:
        h.remove()
    pool = pooling[0] if pooling else None
    return spatial.get("s"), pool, temporal, pred


def received_profile(A):
    """Causal received-attention per key time-step: mean over queries i>=j of A[i,j]."""
    T = A.shape[0]
    prof = np.array([A[j:, j].mean() for j in range(T)])
    return prof


def main():
    model_path, _ = REGIME[ARGS.regime]
    reg = SpatioTemporalTransformerRegressor3.load(model_path)
    chan = list(reg.model.channel_names)

    # Raw signals for the plotted segment (same file/segment we run the model on)
    D = load_segment(ARGS.file, ARGS.segment, ARGS.imu)
    meta = D["meta"]

    # Feature sequence for that exact segment → forward pass with hooks
    loader = DataLoader()
    df = loader.load_and_extract_features([Path(ARGS.file)], is_sequence=True,
                                          use_precomputed=USE_PRECOMPUTED_FEATURES)
    row = df[df["segment_id"] == ARGS.segment]
    if row.empty:
        sys.exit(f"Segment {ARGS.segment} not found among loaded features "
                 f"(it may be blacklisted). Try another --segment.")
    X, _ = loader.prepare_for_ml(row, target_col="weight")
    n_windows_total = len(X.iloc[0]["sequence_dicts"])
    spatial, pooling, temporal, pred = extract_one(reg, X)

    n_used = next(iter(temporal.values())).shape[0]  # = max_seq_len cap
    dur = float(D["t_emg"][-1])
    dur_used = dur * min(1.0, n_used / max(n_windows_total, 1))

    # Temporal profiles (per group + combined), normalised to [0,1]
    profs = {g: received_profile(A) for g, A in temporal.items()}
    t_steps = (np.arange(n_used) + 0.5) / n_used * dur_used
    combined = np.mean(np.stack(list(profs.values())), axis=0)
    cmin, cmax = combined.min(), combined.max()
    band = (combined - cmin) / (cmax - cmin + 1e-9)

    # ── Figure ────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(8.8, 13.4))
    gs = GridSpec(7, 1, figure=fig,
                  height_ratios=[1.15, 0.8, 0.8, 0.8, 1.0, 1.0, 0.95],
                  hspace=0.42, left=0.115, right=0.93, top=0.905, bottom=0.055)

    ax_pool = fig.add_subplot(gs[0])
    ax_sig = [fig.add_subplot(gs[i]) for i in range(1, 6)]
    ax_rib = fig.add_subplot(gs[6], sharex=ax_sig[0])
    for a in ax_sig:
        a.sharex(ax_sig[0])

    def style(ax):
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.tick_params(length=3, color="#888888")
        ax.margins(x=0.005)

    def paint_band(ax):
        y0, y1 = ax.get_ylim()
        ax.imshow(band[None, :], extent=[0, dur_used, y0, y1], aspect="auto",
                  cmap=BAND_CMAP, alpha=0.45, vmin=0, vmax=1, zorder=0,
                  interpolation="bilinear")
        ax.set_ylim(y0, y1)

    # (0) SPATIAL: per-channel importance bars
    order = sorted(range(len(chan)), key=lambda i: (list(MOD_COLORS).index(modality_of(chan[i])), chan[i]))
    p = np.array(pooling)[order]
    cols = [MOD_COLORS[modality_of(chan[i])] for i in order]
    labs = [short_label(chan[i]) for i in order]
    ax_pool.bar(range(len(p)), p, color=cols, width=0.78, zorder=3)
    ax_pool.set_xticks(range(len(p)))
    ax_pool.set_xticklabels(labs, rotation=90, fontsize=8.5)
    ax_pool.set_ylabel("Readout\nweight", fontsize=11, fontfamily=BODY_FONT)
    ax_pool.set_title("Spatial attention — which sensors drive the prediction",
                      loc="left", fontfamily=TITLE_FONT, fontweight="bold",
                      color="#1A1A1A", fontsize=13.5, pad=6)
    ax_pool.grid(True, axis="y", color="#ECECEC", lw=0.8, zorder=0)
    style(ax_pool)
    ax_pool.legend([plt.Rectangle((0, 0), 1, 1, color=c) for c in MOD_COLORS.values()],
                   MOD_COLORS.keys(), fontsize=9.5, ncol=3, frameon=False,
                   loc="upper right", handlelength=1.1)

    # (1–3) EMG panels  (4) orientation  (5) acceleration — all with temporal band
    for ax, (muscle, color) in zip(ax_sig[:3], EMG_COLORS.items()):
        ax.plot(D["t_emg"], D["emg"][muscle], color=color, lw=0.8, zorder=3)
        ax.axhline(0, color="#BBBBBB", lw=0.6, zorder=1)
        ax.set_ylim(-3.0, 3.0)
        ax.set_ylabel("EMG\n(mV)", fontsize=11, fontfamily=BODY_FONT)
        ax.set_title(muscle, loc="left", fontfamily=TITLE_FONT, fontweight="bold",
                     color=color, fontsize=13, pad=4)
        style(ax)
        paint_band(ax)

    ax = ax_sig[3]
    for (axis, sig), c in zip(D["orient"].items(), AXIS_COLORS):
        ax.plot(D["t_imu"], sig, color=c, lw=1.3, label=axis, zorder=3)
    ax.set_ylim(-100, 100)
    ax.set_ylabel("Orient.\n(°)", fontsize=11, fontfamily=BODY_FONT)
    ax.set_title("IMU orientation", loc="left", fontfamily=TITLE_FONT,
                 fontweight="bold", color="#0E5A43", fontsize=13, pad=4)
    ax.legend(ncol=3, fontsize=9.5, frameon=True, framealpha=0.9, edgecolor="#DDD",
              loc="upper right", handlelength=1.2)
    style(ax)
    paint_band(ax)

    ax = ax_sig[4]
    for (axis, sig), c in zip(D["accel"].items(), AXIS_COLORS):
        ax.plot(D["t_imu"], sig, color=c, lw=1.3, label=f"a$_{axis}$", zorder=3)
    ax.set_ylim(-1.5, 1.5)
    ax.set_ylabel("Lin.acc.\n(g)", fontsize=11, fontfamily=BODY_FONT)
    ax.set_title("IMU linear acceleration", loc="left", fontfamily=TITLE_FONT,
                 fontweight="bold", color="#0E5A43", fontsize=13, pad=4)
    ax.legend(ncol=3, fontsize=9.5, frameon=True, framealpha=0.9, edgecolor="#DDD",
              loc="upper right", handlelength=1.2)
    style(ax)
    paint_band(ax)

    # (6) TEMPORAL ribbon: combined heat strip + per-modality curves
    ax = ax_rib
    ax.imshow(band[None, :], extent=[0, dur_used, 0, 1], aspect="auto",
              cmap=BAND_CMAP, alpha=0.85, vmin=0, vmax=1, zorder=0,
              interpolation="bilinear")
    for g, pr in profs.items():
        pr_n = (pr - pr.min()) / (pr.max() - pr.min() + 1e-9)
        ax.plot(t_steps, pr_n, color=RIBBON[g], lw=2.0, label=RIBBON_LBL[g], zorder=4)
    ax.set_ylim(-0.05, 1.12)
    ax.set_yticks([0, 1]); ax.set_yticklabels(["low", "high"], fontsize=9)
    ax.set_ylabel("Temporal\nattention", fontsize=11, fontfamily=BODY_FONT)
    ax.set_xlabel("Time into lift (s)", fontsize=12, fontfamily=BODY_FONT)
    ax.set_title("Temporal attention — when in the lift the model focuses",
                 loc="left", fontfamily=TITLE_FONT, fontweight="bold",
                 color="#1A1A1A", fontsize=13.5, pad=4)
    ax.legend(ncol=3, fontsize=9.5, frameon=False, loc="upper right", handlelength=1.3)
    style(ax)
    ax.set_xlim(0, dur)

    # ── Header + label badge ──
    imu_loc = {1: "forearm", 2: "upper arm"}.get(ARGS.imu)
    fig.text(0.115, 0.965, "Spatial & temporal attention over one lift",
             ha="left", fontfamily=TITLE_FONT, fontweight="bold", fontsize=18, color="#111111")
    fig.text(0.115, 0.937,
             f"ST-Transformer · {REGIME_LABEL[ARGS.regime]} · participant {meta['subject']} · "
             f"{meta['state']} · IMU ({imu_loc})",
             ha="left", fontfamily=BODY_FONT, fontsize=11.5, color="#555555")

    bx = fig.add_axes([0.66, 0.925, 0.27, 0.05]); bx.axis("off")
    bx.add_patch(plt.Rectangle((0, 0), 1, 1, transform=bx.transAxes,
                               facecolor="#2C497F", edgecolor="none", clip_on=False))
    bx.text(0.06, 0.72, "TRUE LABEL", transform=bx.transAxes, va="center", ha="left",
            fontfamily=TITLE_FONT, fontweight="bold", fontsize=8.5, color="#BBD0F0")
    bx.text(0.06, 0.30, "PREDICTED", transform=bx.transAxes, va="center", ha="left",
            fontfamily=TITLE_FONT, fontweight="bold", fontsize=8.5, color="#BBD0F0")
    bx.text(0.95, 0.72, f"{meta['true_weight']:.2f} kg", transform=bx.transAxes,
            va="center", ha="right", fontfamily=TITLE_FONT, fontweight="bold",
            fontsize=14, color="white")
    bx.text(0.95, 0.30, f"{pred:.2f} kg", transform=bx.transAxes,
            va="center", ha="right", fontfamily=TITLE_FONT, fontweight="bold",
            fontsize=14, color="#DDE8FA")

    out_dir = Path(__file__).resolve().parent / "segment_signal_plots"
    out_dir.mkdir(exist_ok=True)
    stem = Path(ARGS.out) if ARGS.out else out_dir / f"attention_over_signal_{ARGS.regime}_{ARGS.segment}"
    fig.savefig(stem.with_suffix(".png"), dpi=300)
    fig.savefig(stem.with_suffix(".pdf"))
    print(f"Saved:\n  {stem.with_suffix('.png')}\n  {stem.with_suffix('.pdf')}")
    print(f"Segment {ARGS.segment}: true={meta['true_weight']:.2f} kg  pred={pred:.2f} kg  "
          f"| windows used {n_used}/{n_windows_total}")


if __name__ == "__main__":
    main()
