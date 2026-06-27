#!/usr/bin/env python3
"""
attention_chord_temporal.py
───────────────────────────
Generate ONE spatial-attention chord per **feature window** of a single lift,
showing how the ST-Transformer3's channel↔channel coupling evolves over time.

The spatial attention is computed per (batch, time) inside the model
(`x.view(B*T, C, d)`), so with a single segment we get a separate C×C map for
each feature window. We average over the 4 ST blocks, subset to the 9 signals
shown in the segment-signal figure, and draw a chord frame for each window.
Edge widths share a global normalisation so frames are directly comparable.

Frames are written to a folder (PNG per window) + an animated GIF.

Usage
-----
    python -m visualization.attention_chord_temporal --segment segment_0009 --imu 1
    python -m visualization.attention_chord_temporal --segment segment_0009 --regime participant
"""
from __future__ import annotations

import argparse
import io
import os
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
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
_ap.add_argument("--gif", action="store_true", default=True, help="also assemble an animated GIF")
_ap.add_argument("--fps", type=float, default=2.0, help="GIF frames per second")
ARGS = _ap.parse_args()

os.environ.setdefault("THESIS_CV_STRATEGY", REGIME[ARGS.regime][1])
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import torch
import torch.nn as nn

from model.model_archs.spatio_temporal_transformer3 import SpatioTemporalTransformerRegressor3
from model.data_loader import DataLoader
from model.config_model import USE_PRECOMPUTED_FEATURES
from visualization.plot_segment_signals import load_segment
from visualization.attention_chord import (
    node_spec, draw_chord, TITLE_FONT, BODY_FONT,
)


def capture_spatial_per_window(reg, X):
    """Return per-window spatial attention [T, C, C] (avg over ST blocks) + prediction."""
    layers = []

    def hook(_m, _i, out):
        w = out[1]                       # [B*T, C, C], B=1 → [T, C, C]
        layers.append(w.detach().float().cpu().numpy())

    handles = [m.register_forward_hook(hook)
               for n, m in reg.model.named_modules()
               if n.endswith("spatial_attn") and isinstance(m, nn.MultiheadAttention)]
    with torch.no_grad():
        pred = float(np.array(reg.predict(X)).ravel()[0])
    for h in handles:
        h.remove()
    return np.mean(np.stack(layers, axis=0), axis=0), pred   # [T, C, C], pred


def main():
    model_path, _ = REGIME[ARGS.regime]
    reg = SpatioTemporalTransformerRegressor3.load(model_path)
    chan = list(reg.model.channel_names)
    idx_of = {c: i for i, c in enumerate(chan)}

    spec = node_spec(chan, ARGS.channels, ARGS.imu)
    sel = [idx_of[s[0]] for s in spec]

    D = load_segment(ARGS.file, ARGS.segment, ARGS.imu)
    dur = float(D["t_emg"][-1])
    meta = D["meta"]

    loader = DataLoader()
    df = loader.load_and_extract_features([Path(ARGS.file)], is_sequence=True,
                                          use_precomputed=USE_PRECOMPUTED_FEATURES)
    row = df[df["segment_id"] == ARGS.segment]
    if row.empty:
        sys.exit(f"Segment {ARGS.segment} not found (maybe blacklisted). Try another --segment.")
    X, _ = loader.prepare_for_ml(row, target_col="weight")

    spatial_TCC, pred = capture_spatial_per_window(reg, X)        # [T, 20, 20]
    spatial_TCC = spatial_TCC[:, sel][:, :, sel]                  # [T, C, C]
    T, C = spatial_TCC.shape[0], spatial_TCC.shape[1]

    # Symmetrize per frame, zero diagonal
    Ssym = (spatial_TCC + spatial_TCC.transpose(0, 2, 1)) / 2.0
    for t in range(T):
        np.fill_diagonal(Ssym[t], 0.0)

    # ── Dramatised per-edge intensity ─────────────────────────────────────────
    # The raw pattern is very stable over time (≈11% per-edge variation), so we
    # stretch EACH edge's temporal range to fill the visual scale: an edge goes
    # dim→bright across the windows where it is weakest→strongest. A mild absolute
    # term keeps the genuinely strong couplings generally more prominent.
    emin = Ssym.min(axis=0)                       # [C, C]
    emax = Ssym.max(axis=0)
    gmax = Ssym.max() or 1.0
    tnorm = (Ssym - emin) / (emax - emin + 1e-9)  # [T,C,C] each edge spans 0..1
    absstr = Ssym / gmax                           # [T,C,C] absolute strength 0..1
    intensity = (0.16 + 0.84 * tnorm) * (0.45 + 0.55 * absstr)   # [T,C,C] in ~[0,1]

    if ARGS.channels == "signal":
        out_dir = (Path(__file__).resolve().parent / "segment_signal_plots"
                   / f"attention_chord_frames_{ARGS.segment}_imu{ARGS.imu}_{ARGS.regime}")
    else:
        out_dir = (Path(__file__).resolve().parent / "segment_signal_plots"
                   / f"attention_chord_frames_{ARGS.segment}_all_{ARGS.regime}")
    out_dir.mkdir(parents=True, exist_ok=True)
    # clear any stale frames from a previous, differently-lengthed run
    for pat in ("frame_*.png", "frame_*.svg", "frame_*.pdf"):
        for old in out_dir.glob(pat):
            old.unlink()

    gif_buffers = []
    cross_def = "super" if ARGS.channels == "signal" else "three"
    style = "gradient"
    wide = (ARGS.channels == "signal")

    for t in range(T):
        t_sec = (t + 0.5) / T * dur
        fig, ax = plt.subplots(figsize=(8.4, 8.8))
        draw_chord(ax, Ssym[t], spec, style=style, cross_def=cross_def, wide=wide,
                   intensity=intensity[t], legend=False, lw_max=12.0, lw_power=2.0)
        fig.subplots_adjust(left=0.02, right=0.98, top=0.99, bottom=0.01)
        fig.savefig(out_dir / f"frame_{t:02d}.svg")            # vector frame to disk
        fig.savefig(out_dir / f"frame_{t:02d}.pdf")            # pdf frame to disk
        if ARGS.gif:                                            # rasterise in-memory for the GIF
            buf = io.BytesIO()
            fig.savefig(buf, format="png", dpi=150)
            buf.seek(0)
            gif_buffers.append(Image.open(buf).convert("RGB"))
        plt.close(fig)

    print(f"Saved {T} SVG/PDF frames → {out_dir}")
    print(f"Segment {ARGS.segment}: true={meta['true_weight']:.2f} kg  pred={pred:.2f} kg  "
          f"| {T} windows, gmax={gmax:.3f}")

    if ARGS.gif and gif_buffers:
        if ARGS.channels == "signal":
            gif_path = out_dir / f"attention_chord_{ARGS.segment}_imu{ARGS.imu}.gif"
        else:
            gif_path = out_dir / f"attention_chord_{ARGS.segment}_all.gif"
        gif_buffers[0].save(gif_path, save_all=True, append_images=gif_buffers[1:],
                            duration=int(1000 / ARGS.fps), loop=0)
        print(f"GIF → {gif_path}")


if __name__ == "__main__":
    main()
