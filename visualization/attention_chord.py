#!/usr/bin/env python3
"""
attention_chord.py
──────────────────
Presentation figure of the ST-Transformer3 **spatial attention** as a circular
chord / arc diagram. Sensors are nodes grouped by modality; arcs are weighted by
the channel→channel attention, with **cross-modal EMG↔IMU** arcs highlighted —
the headline fusion story.

Reads the cached attention from
``visualization/run_plots_attention/attention_data_{regime}.npz`` (no model run).

Two channel modes
-----------------
  --channels all     : all 20 model channels (abbreviated labels, modality colours)
  --channels signal  : ONLY the 9 channels shown in the segment-signal figure
                       (Brachioradialis / Biceps Brachii / Anterior Deltoid +
                        roll·pitch·yaw + aₓ·a_y·a_z of one IMU unit). Labels and
                        colours match plot_segment_signals.py for slide consistency.
                        Illustrative pairing for a signal slide — not segment-specific.

Usage
-----
    python -m visualization.attention_chord --channels signal --imu 1
    python -m visualization.attention_chord --channels all --regime kfold
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.path import Path as MplPath
from matplotlib.patches import PathPatch, Wedge
from matplotlib.collections import LineCollection
from matplotlib.colors import to_rgb

# ── Thesis font identity ───────────────────────────────────────────────────
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
                     "savefig.facecolor": "white"})

OUT_DIR = Path(__file__).resolve().parent / "run_plots_attention"
REGIME_LABEL = {"kfold": "Specialized (k-fold)", "participant": "Generalized (LOPO)"}

# Modality palette (thesis identity) — used for outer group wedges in gold mode
MOD_COLORS = {"EMG": "#E4572E", "IMU-Acc": "#137054", "IMU-Ori": "#2C497F"}
# Vivid ring-matched palette for gradient mode: rings, arc endpoints and legend
# all share these, so an arc's colour at a node matches the ring around that node.
VIVID_MOD = {"EMG": "#FF6A2E", "IMU-Acc": "#10C46B", "IMU-Ori": "#2E8BFF"}
MOD_ORDER = ["EMG", "IMU-Acc", "IMU-Ori"]
CROSS_COLOR = "#DDAA33"   # highlighted cross-modal arcs (Paul Tol gold)

# Vivid, saturated per-muscle EMG + per-axis IMU colours for a strong arc gradient
# (brighter cousins of the signal-figure hues; arcs blend warm EMG → cool IMU).
EMG_SIGNAL_COLORS = {"Brachioradialis": "#FF5A1F", "Biceps Brachii": "#FF2D2D",
                     "Anterior Deltoid": "#FFB300"}
AXIS_SIGNAL_COLORS = ["#00D1A7", "#22E06B", "#2E8BFF"]  # x/roll teal, y/pitch green, z/yaw blue

_EMG_ABBR = {
    "Anterior Deltoid": "A.Delt", "Lateral Deltoid": "L.Delt", "Posterior Deltoid": "P.Delt",
    "Triceps Brachii": "Triceps", "Biceps Brachii": "Biceps", "Brachioradialis": "Brachiorad",
    "Flexor Carpi Ulnaris (FCU)": "FCU", "Extensor Carpi Radialis (ECR)": "ECR",
}
_IMU_ABBR = {
    "ax1": r"$a_{x,1}$", "ay1": r"$a_{y,1}$", "az1": r"$a_{z,1}$",
    "ax2": r"$a_{x,2}$", "ay2": r"$a_{y,2}$", "az2": r"$a_{z,2}$",
    "roll_rad1": r"$\phi_1$", "pitch_rad1": r"$\theta_1$", "yaw_rad1": r"$\psi_1$",
    "roll_rad2": r"$\phi_2$", "pitch_rad2": r"$\theta_2$", "yaw_rad2": r"$\psi_2$",
}


def modality_of(ch):
    if "_EMG" in ch:
        return "EMG"
    if "_IMU" in ch and any(k in ch for k in ["ax", "ay", "az"]):
        return "IMU-Acc"
    if "_IMU" in ch and any(k in ch for k in ["roll", "pitch", "yaw", "rad"]):
        return "IMU-Ori"
    return "Other"


def _super(mod):
    """Collapse modality to EMG vs IMU (for cross-modal detection)."""
    return "EMG" if mod == "EMG" else "IMU"


def short_label(ch):
    base = ch.replace("_EMG", "").replace("_IMU", "")
    if "_EMG" in ch:
        return base
    return _IMU_ABBR.get(base, base)


def node_spec(available, mode, imu_unit):
    """Return ordered list of (npz_channel, label, node_color, modality)."""
    if mode == "signal":
        u = str(imu_unit)
        spec = [
            ("Brachioradialis_EMG",  "Brachioradialis",  EMG_SIGNAL_COLORS["Brachioradialis"],  "EMG"),
            ("Biceps Brachii_EMG",   "Biceps\nBrachii",   EMG_SIGNAL_COLORS["Biceps Brachii"],   "EMG"),
            ("Anterior Deltoid_EMG", "Anterior\nDeltoid", EMG_SIGNAL_COLORS["Anterior Deltoid"], "EMG"),
            (f"roll_rad{u}_IMU",  rf"$\phi_{u}$",     AXIS_SIGNAL_COLORS[0], "IMU-Ori"),
            (f"pitch_rad{u}_IMU", rf"$\theta_{u}$",    AXIS_SIGNAL_COLORS[1], "IMU-Ori"),
            (f"yaw_rad{u}_IMU",   rf"$\psi_{u}$",      AXIS_SIGNAL_COLORS[2], "IMU-Ori"),
            (f"ax{u}_IMU", rf"$a_{{x,{u}}}$", AXIS_SIGNAL_COLORS[0], "IMU-Acc"),
            (f"ay{u}_IMU", rf"$a_{{y,{u}}}$", AXIS_SIGNAL_COLORS[1], "IMU-Acc"),
            (f"az{u}_IMU", rf"$a_{{z,{u}}}$", AXIS_SIGNAL_COLORS[2], "IMU-Acc"),
        ]
        spec = [s for s in spec if s[0] in available]
        missing = [s[0] for s in [
            ("Brachioradialis_EMG",), ("Biceps Brachii_EMG",), ("Anterior Deltoid_EMG",)]
            if s[0] not in available]
        if missing:
            raise SystemExit(f"[attention_chord] Channels missing from cache: {missing}")
        return spec
    # mode == "all": every channel, grouped by 3-way modality, abbreviated labels
    chans = [c for c in available if modality_of(c) in MOD_ORDER]
    chans.sort(key=lambda c: (MOD_ORDER.index(modality_of(c)), c))
    out = []
    for c in chans:
        m = modality_of(c)
        out.append((c, short_label(c), MOD_COLORS[m], m))
    return out


def _bezier(p0, p1, frac=0.16):
    ctrl = (np.array(p0) + np.array(p1)) * frac
    return MplPath([p0, tuple(ctrl), p1],
                   [MplPath.MOVETO, MplPath.CURVE3, MplPath.CURVE3])


def _grad_arc(ax, p0, p1, c0, c1, lw, alpha, zorder, n=220, frac=0.16, fade=0.22):
    """Quadratic-Bézier arc whose colour blends c0 → c1 and whose opacity tapers
    to zero at both endpoints, so arcs glow in the middle and fade out before
    they converge at the nodes (no blob/dot where many arcs meet)."""
    p0, p1 = np.array(p0), np.array(p1)
    ctrl = (p0 + p1) * frac
    s = np.linspace(0, 1, n)[:, None]
    pts = (1 - s) ** 2 * p0 + 2 * (1 - s) * s * ctrl + s ** 2 * p1
    segs = np.stack([pts[:-1], pts[1:]], axis=1)
    r0, g0, b0 = to_rgb(c0); r1, g1, b1 = to_rgb(c1)
    sc = ((s[:-1] + s[1:]) / 2).ravel()

    def smooth(x):                       # smoothstep ramp
        x = np.clip(x, 0.0, 1.0)
        return x * x * (3 - 2 * x)
    taper = smooth(sc / fade) * smooth((1 - sc) / fade)   # 0 at ends → 1 in middle

    cols = np.column_stack([r0 + (r1 - r0) * sc, g0 + (g1 - g0) * sc,
                            b0 + (b1 - b0) * sc, alpha * taper])
    # flush ("butt") caps so the segments form one continuous line — round caps
    # would bead into evenly-spaced dots along the arc.
    ax.add_collection(LineCollection(segs, colors=cols, linewidths=lw,
                                     capstyle="butt", zorder=zorder, antialiased=True))


def draw_gradient_wedge_data_units(ax, r_outer, width, theta_start, theta_end, node_angles, node_colors, zorder=2, alpha=0.95):
    """Draw an arc from theta_start to theta_end (in degrees) with a color gradient
    using small Wedge patches so that width scales in data units.
    """
    num_segs = 120
    thetas = np.linspace(theta_start, theta_end, num_segs)
    
    # Convert node_angles to degrees if they are in radians
    node_angles_deg = np.rad2deg(node_angles)
    
    for i in range(num_segs - 1):
        t_start = thetas[i]
        t_end = thetas[i+1]
        t_mid = (t_start + t_end) / 2
        
        # Interpolate color at t_mid
        # Since node_angles_deg are descending
        if t_mid >= node_angles_deg[0]:
            c = node_colors[0]
        elif t_mid <= node_angles_deg[-1]:
            c = node_colors[-1]
        else:
            idx = 0
            for j in range(len(node_angles_deg) - 1):
                if node_angles_deg[j] >= t_mid >= node_angles_deg[j+1]:
                    idx = j
                    break
            t0, t1 = node_angles_deg[idx], node_angles_deg[idx+1]
            c0, c1 = to_rgb(node_colors[idx]), to_rgb(node_colors[idx+1])
            frac = (t0 - t_mid) / (t0 - t1 + 1e-9)
            c = tuple(np.array(c0) + frac * (np.array(c1) - np.array(c0)))
            
        ax.add_patch(Wedge((0, 0), r_outer, t_start, t_end, width=width,
                           facecolor=c, edgecolor="none", alpha=alpha, zorder=zorder))


def draw_chord(ax, S, spec, *, style="gradient", cross_def="super", wide=True,
               wmax=None, thresh=None, intensity=None, legend=True,
               lw_max=6.0, lw_power=1.0):
    """Render a chord diagram of symmetric coupling ``S`` (C×C) for nodes ``spec``
    onto ``ax``. ``spec`` is a list of (name, label, node_color, modality).

    style="gradient": each arc is colour-blended between its two endpoint dots
                      (so dot colours read through the arcs; long warm→cool arcs
                      are EMG↔IMU coupling). style="gold": single gold cross-modal
                      highlight (legacy 20-channel look).
    ``intensity`` (C×C, values ~[0,1]): if given, drives arc width/opacity directly
                  instead of ``S``/``wmax`` — used to dramatise per-window changes.
    ``wmax`` / ``thresh`` fix width-normalisation / visibility cutoff for ``S``.
    Returns (cross_n, intra_n).
    """
    ncolors = [s[2] for s in spec]
    mods = [s[3] for s in spec]
    labels = [s[1] for s in spec]
    C = len(spec)

    # Ring-matched palette: in gradient mode the arc endpoints take the colour of
    # the ring (modality wedge) around them, so node colours == ring colours.
    pal = VIVID_MOD if style != "gold" else MOD_COLORS
    if style != "gold" and not wide:
        ncolors = [pal[m] for m in mods]

    S = (np.asarray(S, dtype=float) + np.asarray(S, dtype=float).T) / 2.0
    np.fill_diagonal(S, 0.0)

    present = [m for m in MOD_ORDER if m in mods]
    gap = np.deg2rad(12 if C <= 12 else 10)
    span = 2 * np.pi - gap * len(present)
    per_node = span / C
    angles = np.zeros(C)
    a = np.pi / 2
    prev = None
    for i in sorted(range(C), key=lambda i: (MOD_ORDER.index(mods[i]),)):
        if prev is not None and mods[i] != prev:
            a -= gap
        angles[i] = a
        a -= per_node
        prev = mods[i]
    R = 1.0
    xy = np.column_stack([R * np.cos(angles), R * np.sin(angles)])

    iu = np.triu_indices(C, k=1)
    if wmax is None:
        wmax = S[iu].max() if S[iu].max() > 0 else 1.0
    if thresh is None:
        thresh = 0.0

    ax.set_aspect("equal"); ax.axis("off")
    ax.set_xlim(-1.60 if wide else -1.45, 1.45 if wide else 1.45)
    ax.set_ylim(-1.35 if wide else -1.35, 1.35 if wide else 1.35)
    lab_off = 0.22 if wide else 0.18

    # Outer modality arcs
    for m in present:
        idxs = [i for i in range(C) if mods[i] == m]
        a0 = np.rad2deg(angles[max(idxs, key=lambda k: -angles[k])]) - np.rad2deg(per_node) / 2
        a1 = np.rad2deg(angles[min(idxs, key=lambda k: -angles[k])]) + np.rad2deg(per_node) / 2
        
        if style != "gold":
            # Find node angles and colors for this group
            group_nodes = sorted([(angles[i], ncolors[i]) for i in idxs], key=lambda x: -x[0])
            group_angles = [gn[0] for gn in group_nodes]
            group_colors = [gn[1] for gn in group_nodes]
            draw_gradient_wedge_data_units(ax, R + 0.135, 0.055, a0, a1, group_angles, group_colors, zorder=2, alpha=0.95)
        else:
            ax.add_patch(Wedge((0, 0), R + 0.135, a0, a1, width=0.055,
                               facecolor=pal[m], edgecolor="none", alpha=0.95, zorder=2))

    # Edges (weak→strong so strong drawn on top)
    cross_n = intra_n = 0
    floor = 0.05
    for i, j, wij in sorted(zip(*iu, S[iu]), key=lambda e: e[2]):
        vis = float(intensity[i, j]) if intensity is not None else min(wij / wmax, 1.0)
        if intensity is None and (wij < thresh or wij <= 0):
            continue
        if intensity is not None and vis < floor:
            continue
        cross = (_super(mods[i]) != _super(mods[j])) if cross_def == "super" else (mods[i] != mods[j])
        cross_n, intra_n = (cross_n + 1, intra_n) if cross else (cross_n, intra_n + 1)
        if style == "gold":
            color = CROSS_COLOR if cross else MOD_COLORS[mods[i]]
            base_alpha, zorder, lwf = (0.85, 6, 4.4) if cross else (0.32, 4, 2.8)
            ax.add_patch(PathPatch(_bezier(xy[i], xy[j]), edgecolor=color, facecolor="none",
                                   lw=0.5 + lwf * (vis ** lw_power), alpha=base_alpha * (0.45 + 0.55 * (vis ** lw_power)),
                                   capstyle="round", zorder=zorder))
        else:  # gradient
            zorder = 6 if cross else 4
            lw = 0.6 + lw_max * (vis ** lw_power)
            alpha = min(0.10 + 0.90 * (vis ** lw_power), 1.0)
            _grad_arc(ax, xy[i], xy[j], ncolors[i], ncolors[j], lw, alpha, zorder)

    # Node markers + labels
    lab_fs = 12.5 if C <= 12 else 9.5
    for k in range(C):
        if style == "gold":
            ax.plot(*xy[k], "o", ms=10 if C <= 12 else 6, color=ncolors[k],
                    markeredgecolor="white", markeredgewidth=1.1, zorder=10)
        ang = angles[k]
        cos, sin = np.cos(ang), np.sin(ang)
        lx, ly = (R + lab_off) * cos, (R + lab_off) * sin
        # all labels horizontal; anchor outward from the ring
        ha = "left" if cos > 0.15 else ("right" if cos < -0.15 else "center")
        va = "bottom" if sin > 0.15 else ("top" if sin < -0.15 else "center")
        ax.text(lx, ly, labels[k], ha=ha, va=va, rotation=0,
                fontsize=lab_fs, fontfamily=BODY_FONT, color="#222222", zorder=11)

    if legend:
        handles = []
        if style == "gold":
            handles.append(plt.Line2D([0], [0], marker="o", color="none", markerfacecolor="#555555",
                                      markeredgecolor="white", markersize=9, label="sensor"))
        handles += [plt.Line2D([0], [0], color=pal[m], lw=8, label=m) for m in present]
        if style == "gold":
            handles.append(plt.Line2D([0], [0], color=CROSS_COLOR, lw=3.5, label="cross-modal (EMG–IMU)"))
        ax.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.5, -0.02),
                  ncol=len(handles), frameon=False, fontsize=10.5,
                  handlelength=1.5, columnspacing=1.4)
        note = ("Arc colour blends the two sensors it links — "
                "warm-to-cool arcs are EMG–IMU coupling") if style != "gold" else \
               "Dots = sensors (colour = signal trace)"
        ax.text(0.5, -0.075, note, transform=ax.transAxes, ha="center", va="top",
                fontsize=9.5, color="#777777", fontfamily=BODY_FONT)
    return cross_n, intra_n


def build_chord(regime, mode, imu_unit, topk, out_stem):
    data = np.load(OUT_DIR / f"attention_data_{regime}.npz", allow_pickle=True)
    available = list(data["channels"])
    A_full = np.array(data["spatial_mean"], dtype=float)
    idx_of = {c: i for i, c in enumerate(available)}

    spec = node_spec(available, mode, imu_unit)
    sel = [idx_of[s[0]] for s in spec]
    S = A_full[np.ix_(sel, sel)]
    C = len(spec)

    cross_def = "super" if mode == "signal" else "three"
    style = "gradient" if mode == "signal" else "gold"
    Ssym = (S + S.T) / 2.0
    np.fill_diagonal(Ssym, 0.0)
    w = Ssym[np.triu_indices(C, k=1)]
    thresh = np.sort(w)[-topk] if topk < len(w) else 0.0

    strip = (style == "gradient")   # signal-mode presentation chord → no title/legend chrome
    fig, ax = plt.subplots(figsize=(8.4, 8.8))
    cross_n, intra_n = draw_chord(ax, S, spec, style=style, cross_def=cross_def,
                                  wide=(mode == "signal"), thresh=thresh, legend=not strip)

    if not strip:
        fig.text(0.5, 0.965, "Spatial attention: sensor coupling",
                 ha="center", fontfamily=TITLE_FONT, fontweight="bold", fontsize=18, color="#1A1A1A")
        sub = (f"ST-Transformer · {REGIME_LABEL[regime]} · top {min(topk, len(w))} channel "
               f"pairs · cross-modal EMG–IMU highlighted")
        fig.text(0.5, 0.932, sub, ha="center", fontfamily=BODY_FONT, fontsize=12, color="#555555")
        fig.subplots_adjust(left=0.02, right=0.98, top=0.91, bottom=0.10)
    else:
        fig.subplots_adjust(left=0.02, right=0.98, top=0.99, bottom=0.01)

    svg, pdf = out_stem.with_suffix(".svg"), out_stem.with_suffix(".pdf")
    fig.savefig(svg); fig.savefig(pdf); plt.close(fig)
    print(f"Saved:\n  {svg}\n  {pdf}")
    print(f"Nodes: {C} | edges — cross-modal: {cross_n}, intra-modal: {intra_n}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--regime", default="kfold", choices=["kfold", "participant"])
    ap.add_argument("--channels", default="all", choices=["all", "signal"])
    ap.add_argument("--imu", type=int, default=1, choices=[1, 2], help="IMU unit for --channels signal")
    ap.add_argument("--topk", type=int, default=55, help="max strongest channel pairs to draw")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    out_dir = Path(__file__).resolve().parent / "segment_signal_plots"
    out_dir.mkdir(exist_ok=True)
    if args.out:
        stem = Path(args.out)
    elif args.channels == "signal":
        stem = out_dir / f"attention_chord_signal_imu{args.imu}_{args.regime}"
    else:
        stem = out_dir / f"attention_chord_{args.regime}"
    build_chord(args.regime, args.channels, args.imu, args.topk, stem)


if __name__ == "__main__":
    main()
