#!/usr/bin/env python3
"""
plot_segment_signals.py
───────────────────────
Presentation-quality vertical figure of the raw sensor signals contained in a
single lift *segment* from the BasData database.

Five stacked panels (shared time axis):
    1. Brachioradialis           (EMG)
    2. Biceps Brachii            (EMG)
    3. Anterior Deltoid          (EMG)
    4. Orientation roll/pitch/yaw (IMU, one panel, 3 axes)
    5. Linear acceleration aₓ/a_y/a_z (IMU, one panel, 3 axes)

The lift's load (the regression *label*) is shown as a prominent badge.

Usage
-----
    python -m visualization.plot_segment_signals                 # default segment
    python -m visualization.plot_segment_signals --segment segment_0238
    python -m visualization.plot_segment_signals --file <path.h5> --imu 2
"""
from __future__ import annotations

import argparse
from pathlib import Path

import h5py
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import matplotlib.patches as patches

# ──────────────────────────────────────────────────────────────────────────
# Thesis font + colour identity (mirrors visualization/run_viz.py & color_picker.py)
# ──────────────────────────────────────────────────────────────────────────
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

plt.rcParams.update({
    "font.family": BODY_FONT,
    "font.size": 14,
    "axes.titlesize": 15,
    "axes.labelsize": 14,
    "xtick.labelsize": 13,
    "ytick.labelsize": 12,
    "axes.edgecolor": "#444444",
    "axes.linewidth": 0.9,
    "figure.facecolor": "white",
    "savefig.facecolor": "white",
})

# EMG = warm orange-red family (vivid colors matching attention chord / temporal viz)
EMG_COLORS = {
    "Brachioradialis": "#FF5A1F",
    "Biceps Brachii":  "#FF2D2D",
    "Anterior Deltoid": "#FFB300",
}
# IMU = 3 distinct axis hues (matching AXIS_SIGNAL_COLORS)
AXIS_COLORS = ["#00D1A7", "#22E06B", "#2E8BFF"]  # x, y, z

# nominal → true (calibrated) load, kg  (model/config_model.py TRUE_WEIGHTS)
TRUE_WEIGHTS = {0.75: 0.90, 1.0: 0.98, 2.0: 1.97, 2.25: 2.24,
                3.0: 2.95, 4.25: 4.15, 6.0: 5.93}

DEFAULT_FILE = "/Volumes/Laurens SSD/BasData/segments/participant_P01_session_01_segments.h5"
FALLBACK_FILE = "/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Coding Exercise/participant_P01_session_01_segments.h5"


def _decode(raw):
    return [c.decode() if isinstance(c, bytes) else str(c) for c in raw]


def _attr(grp, name, default=""):
    v = grp.attrs.get(name, default)
    return v.decode() if isinstance(v, bytes) else v


def load_segment(file_path: str, segment: str, imu_unit: int):
    """Return a dict of signals + metadata for one segment."""
    with h5py.File(file_path, "r") as f:
        if segment not in f:
            raise SystemExit(f"Segment '{segment}' not in {file_path}. "
                             f"Available e.g.: {sorted(k for k in f if k.startswith('segment_'))[:5]} ...")
        g = f[segment]

        emg = g["emg"][:]
        emg_cols = _decode(g["emg"].attrs.get("column_names", []))
        emg_fs = float(g["emg"].attrs.get("fs", 2000.0))

        imu = g["imu"][:]
        imu_cols = _decode(g["imu"].attrs.get("column_names", []))
        imu_fs = float(g["imu"].attrs.get("fs", 2000.0))

        nominal_w = float(g.attrs.get("weight", -1.0))
        label = str(_attr(g, "label", "unknown"))
        true_w = 0.0 if label == "free_movement" else TRUE_WEIGHTS.get(nominal_w, nominal_w)
        meta = {
            "state": str(_attr(g, "state", "unknown")),
            "nominal_weight": 0.0 if label == "free_movement" else nominal_w,
            "true_weight": true_w,
            "subject": Path(file_path).stem.split("_")[1] if "participant" in Path(file_path).stem else "?",
            "segment": segment,
        }

    def emg_ch(name):
        return emg[:, emg_cols.index(name)]

    def imu_ch(name):
        return imu[:, imu_cols.index(name)]

    s = str(imu_unit)
    t_emg = np.arange(emg.shape[0]) / emg_fs
    t_imu = np.arange(imu.shape[0]) / imu_fs

    return {
        "t_emg": t_emg,
        "t_imu": t_imu,
        "emg": {m: emg_ch(m) for m in EMG_COLORS},
        # orientation in radians → degrees
        "orient": {
            "roll":  np.degrees(imu_ch(f"roll_rad{s}")),
            "pitch": np.degrees(imu_ch(f"pitch_rad{s}")),
            "yaw":   np.degrees(imu_ch(f"yaw_rad{s}")),
        },
        "accel": {
            "x": imu_ch(f"ax{s}") / 9.80665,
            "y": imu_ch(f"ay{s}") / 9.80665,
            "z": imu_ch(f"az{s}") / 9.80665,
        },
        "meta": meta,
        "imu_unit": imu_unit,
    }


def make_figure(D: dict, out_stem: Path, emg_lim: float, orient_lim: float, accel_lim: float):
    meta = D["meta"]
    imu_loc = {1: "forearm", 2: "upper arm"}.get(D["imu_unit"], f"IMU {D['imu_unit']}")

    fig, axes = plt.subplots(
        5, 1, figsize=(6.0, 11.2), sharex=True,
        gridspec_kw={"hspace": 0.38, "height_ratios": [1, 1, 1, 1.25, 1.25]},
    )

    def style(ax):
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.grid(True, axis="y", color="#E6E6E6", linewidth=0.8, zorder=0)
        ax.tick_params(length=3, color="#888888")
        ax.margins(x=0.005)

    def framed_legend(ax):
        leg = ax.legend(ncol=3, loc="upper right", fontsize=11.5, handlelength=1.3,
                        columnspacing=1.2, borderaxespad=0.3, frameon=True,
                        framealpha=0.9, edgecolor="#DDDDDD", fancybox=False)
        leg.get_frame().set_facecolor("white")
        leg.get_frame().set_linewidth(0.8)
        # headroom so the framed legend clears the traces
        ax.margins(y=0.18)

    # ── EMG panels (1–3) ──
    for ax, (muscle, color) in zip(axes[:3], EMG_COLORS.items()):
        ax.plot(D["t_emg"], D["emg"][muscle], color=color, lw=0.8, zorder=3)
        ax.axhline(0, color="#BBBBBB", lw=0.6, zorder=1)
        ax.set_ylabel("EMG\n(mV)", fontsize=12, fontfamily=BODY_FONT)
        ax.set_ylim(-emg_lim, emg_lim)
        ax.set_title(muscle, loc="left", fontfamily=TITLE_FONT,
                     fontweight="bold", color=color, fontsize=14.5, pad=6)
        style(ax)

    # ── IMU orientation panel (4) ──
    ax = axes[3]
    u = D["imu_unit"]
    orient_labels = {
        "roll": rf"$\phi_{u}$",
        "pitch": rf"$\theta_{u}$",
        "yaw": rf"$\psi_{u}$",
    }
    for (axis, sig), c in zip(D["orient"].items(), AXIS_COLORS):
        ax.plot(D["t_imu"], sig, color=c, lw=1.4, label=orient_labels.get(axis, axis), zorder=3)
    ax.set_ylabel("Orientation\n(°)", fontsize=12, fontfamily=BODY_FONT)
    ax.set_ylim(-orient_lim, orient_lim)
    ax.set_title("IMU Orientation", loc="left",
                 fontfamily=TITLE_FONT, fontweight="bold", color="#2E8BFF", fontsize=14.5, pad=6)
    style(ax)
    framed_legend(ax)

    # ── IMU acceleration panel (5) ──
    ax = axes[4]
    for (axis, sig), c in zip(D["accel"].items(), AXIS_COLORS):
        ax.plot(D["t_imu"], sig, color=c, lw=1.4, label=f"a$_{axis}$", zorder=3)
    ax.set_ylabel("Lin. accel.\n(g)", fontsize=12, fontfamily=BODY_FONT)
    ax.set_ylim(-accel_lim, accel_lim)
    ax.set_title("IMU Linear Acceleration", loc="left",
                 fontfamily=TITLE_FONT, fontweight="bold", color="#10C46B", fontsize=14.5, pad=6)
    style(ax)
    framed_legend(ax)

    axes[-1].set_xlabel("Time (s)", fontsize=14, fontfamily=BODY_FONT)

    # Load badge text axes (transparent, overlaying the background patch)
    badge = fig.add_axes([0.34, 0.94, 0.32, 0.045])
    badge.axis("off")

    # Load badge background (drawn in physical inches via dpi_scale_trans to prevent aspect ratio distortion of rounded corners)
    fig_w, fig_h = fig.get_size_inches()
    badge_x = 0.34 * fig_w
    badge_y = 0.94 * fig_h
    badge_w = 0.32 * fig_w
    badge_h = 0.045 * fig_h
    
    badge_patch = patches.FancyBboxPatch(
        (badge_x, badge_y), badge_w, badge_h,
        transform=fig.dpi_scale_trans,
        boxstyle="round,pad=0,rounding_size=0.08",
        facecolor="#373544", edgecolor="none",
        zorder=1, clip_on=False
    )
    badge.add_patch(badge_patch)

    badge.text(0.08, 0.5, "LOAD", transform=badge.transAxes, va="center", ha="left",
               fontfamily=TITLE_FONT, fontweight="bold", fontsize=12.5,
               color="#C2C0D4", zorder=10)
    badge.text(0.92, 0.5, f"{meta['true_weight']:.2f} kg", transform=badge.transAxes,
               va="center", ha="right", fontfamily=TITLE_FONT, fontweight="bold",
               fontsize=20.0, color="white", zorder=10)

    fig.subplots_adjust(left=0.15, right=0.96, top=0.910, bottom=0.055)

    png = out_stem.with_suffix(".png")
    pdf = out_stem.with_suffix(".pdf")
    svg = out_stem.with_suffix(".svg")
    fig.savefig(png, dpi=300)
    fig.savefig(pdf)
    fig.savefig(svg)
    plt.close(fig)
    print(f"Saved:\n  {png}\n  {pdf}\n  {svg}")
    print(f"Segment: {meta['segment']}  |  label = {meta['true_weight']:.2f} kg "
          f"(nominal {meta['nominal_weight']:.2f}) | state = {meta['state']}")


def get_segments_per_weight(file_path: str) -> dict[float, str]:
    """Find one representative segment for each weight label in the H5 file."""
    weights_map = {}
    with h5py.File(file_path, "r") as f:
        for key in sorted(f.keys()):
            if not key.startswith("segment_"):
                continue
            label = str(_attr(f[key], "label", "unknown"))
            w = 0.0 if label == "free_movement" else float(f[key].attrs.get("weight", -1.0))
            if w >= 0 and w not in weights_map:
                weights_map[w] = key
    return weights_map


def compute_fair_limits(file_path: str, imu_unit: int, segments: list[str]) -> tuple[float, float, float]:
    """Scan the specified segments to determine consistent y-limits for EMG, Orientation, and Acceleration."""
    max_emg = 0.0
    max_orient = 0.0
    max_accel = 0.0
    
    with h5py.File(file_path, "r") as f:
        for seg in segments:
            if seg not in f:
                continue
            g = f[seg]
            
            # EMG
            if "emg" in g:
                emg = g["emg"][:]
                cols = _decode(g["emg"].attrs.get("column_names", []))
                for muscle in EMG_COLORS:
                    if muscle in cols:
                        val = np.max(np.abs(emg[:, cols.index(muscle)]))
                        if val > max_emg:
                            max_emg = val
                            
            # IMU
            if "imu" in g:
                imu = g["imu"][:]
                cols = _decode(g["imu"].attrs.get("column_names", []))
                s = str(imu_unit)
                
                # Orientation
                for axis in ["roll_rad", "pitch_rad", "yaw_rad"]:
                    col_name = f"{axis}{s}"
                    if col_name in cols:
                        deg_val = np.max(np.abs(np.degrees(imu[:, cols.index(col_name)])))
                        if deg_val > max_orient:
                            max_orient = deg_val
                            
                # Acceleration
                for axis in ["ax", "ay", "az"]:
                    col_name = f"{axis}{s}"
                    if col_name in cols:
                        acc_val = np.max(np.abs(imu[:, cols.index(col_name)])) / 9.80665
                        if acc_val > max_accel:
                            max_accel = acc_val
                            
    # Clean rounding:
    # EMG: round up to nearest 5 (minimum 5.0)
    emg_lim = float(max(5.0, np.ceil(max_emg / 5.0) * 5.0))
    # Orientation: round up to nearest 10, max 180 (minimum 10.0)
    orient_lim = min(180.0, float(max(10.0, np.ceil(max_orient / 10.0) * 10.0)))
    # Acceleration: round up to nearest 0.5 (minimum 0.5)
    accel_lim = float(max(0.5, np.ceil(max_accel * 2.0) / 2.0))
    
    return emg_lim, orient_lim, accel_lim


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--file", default=DEFAULT_FILE, help="segmented HDF5 file")
    ap.add_argument("--segment", default=None, help="segment key, e.g. segment_0238 (default: all weights)")
    ap.add_argument("--imu", type=int, default=1, choices=[1, 2],
                    help="IMU unit: 1=forearm, 2=upper arm")
    ap.add_argument("--out", default=None, help="output path stem (no extension)")
    args = ap.parse_args()

    file_path = args.file
    if not Path(file_path).exists():
        if Path(FALLBACK_FILE).exists():
            file_path = FALLBACK_FILE
        else:
            # Look in parent directories for Coding Exercise or data
            possible_paths = list(Path(__file__).resolve().parents[2].glob("**/participant_P01_session_01_segments.h5"))
            if possible_paths:
                file_path = str(possible_paths[0])
            else:
                raise FileNotFoundError(f"Could not find segment H5 file at {DEFAULT_FILE} or fallback {FALLBACK_FILE}")

    out_dir = Path(__file__).resolve().parent / "segment_signal_plots" / "segment_signals"
    out_dir.mkdir(parents=True, exist_ok=True)

    # Find representative segments to compute consistent limits
    weights_map = get_segments_per_weight(file_path)
    if not weights_map:
        raise ValueError(f"No segments found in {file_path}")
        
    segments_for_limits = list(weights_map.values())
        
    emg_lim, orient_lim, accel_lim = compute_fair_limits(file_path, args.imu, segments_for_limits)
    print(f"Consistent Limits Determined:\n  EMG: ±{emg_lim} mV\n  Orientation: ±{orient_lim}°\n  Acceleration: ±{accel_lim} g")

    if args.segment:
        # Plot single segment
        D = load_segment(file_path, args.segment, args.imu)
        stem = Path(args.out) if args.out else out_dir / f"{D['meta']['subject']}_{args.segment}_imu{args.imu}"
        make_figure(D, stem, emg_lim, orient_lim, accel_lim)
    else:
        # Plot one segment per weight label
        print(f"Generating plots for weights: {sorted(weights_map.keys())}")
        for w, seg in sorted(weights_map.items()):
            D = load_segment(file_path, seg, args.imu)
            stem = out_dir / f"{D['meta']['subject']}_{seg}_imu{args.imu}"
            make_figure(D, stem, emg_lim, orient_lim, accel_lim)


if __name__ == "__main__":
    main()
