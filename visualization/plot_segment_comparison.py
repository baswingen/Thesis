#!/usr/bin/env python3
"""
plot_segment_comparison.py
──────────────────────────
Generates a side-by-side comparison figure of the first 1.0 second of raw sensor
signals for 6 specific load classes: 0kg, 0.98kg, 1.97kg, 2.95kg, 4.15kg, and 5.93kg.

The figure consists of 5 rows (channels) and 6 columns (load classes):
    Row 1: Brachioradialis (EMG)
    Row 2: Biceps Brachii (EMG)
    Row 3: Anterior Deltoid (EMG)
    Row 4: IMU Orientation (roll/pitch/yaw)
    Row 5: IMU Linear Acceleration (ax/ay/az)
"""
from __future__ import annotations

import argparse
from pathlib import Path

import h5py
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# ──────────────────────────────────────────────────────────────────────────
# Fonts and Colors (mirrors plot_segment_signals.py)
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
    "font.size": 12,
    "axes.titlesize": 13,
    "axes.labelsize": 12,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "axes.edgecolor": "#444444",
    "axes.linewidth": 0.8,
    "figure.facecolor": "white",
    "savefig.facecolor": "white",
})

EMG_COLORS = {
    "Brachioradialis": "#FF5A1F",
    "Biceps Brachii":  "#FF2D2D",
    "Anterior Deltoid": "#FFB300",
}
AXIS_COLORS = ["#00D1A7", "#22E06B", "#2E8BFF"]  # x/roll, y/pitch, z/yaw

DEFAULT_FILE = "/Volumes/Laurens SSD/BasData/segments/participant_P01_session_01_segments.h5"
FALLBACK_FILE = "/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Coding Exercise/participant_P01_session_01_segments.h5"

# The target load classes and their matching segment keys (for P01 session 01)
TARGET_LOADS = {
    "0.00 kg": "segment_0002",
    "0.98 kg": "segment_0011",
    "1.97 kg": "segment_0007",
    "2.95 kg": "segment_0009",
    "4.15 kg": "segment_0005",
    "5.93 kg": "segment_0019",
}


def _decode(raw):
    return [c.decode() if isinstance(c, bytes) else str(c) for c in raw]


def load_sliced_segment(file_path: str, segment: str, imu_unit: int, t_max: float = 1.0) -> dict:
    """Load and slice the first t_max seconds of a segment."""
    with h5py.File(file_path, "r") as f:
        g = f[segment]
        
        # Load EMG
        emg_raw = g["emg"][:]
        emg_cols = _decode(g["emg"].attrs.get("column_names", []))
        emg_fs = float(g["emg"].attrs.get("fs", 2000.0))
        t_emg = np.arange(emg_raw.shape[0]) / emg_fs
        
        # Slice EMG
        emg_mask = t_emg <= t_max
        t_emg_sliced = t_emg[emg_mask]
        emg_sliced = emg_raw[emg_mask]
        
        # Load IMU
        imu_raw = g["imu"][:]
        imu_cols = _decode(g["imu"].attrs.get("column_names", []))
        imu_fs = float(g["imu"].attrs.get("fs", 2000.0))
        t_imu = np.arange(imu_raw.shape[0]) / imu_fs
        
        # Slice IMU
        imu_mask = t_imu <= t_max
        t_imu_sliced = t_imu[imu_mask]
        imu_sliced = imu_raw[imu_mask]
        
    def emg_ch(name):
        return emg_sliced[:, emg_cols.index(name)]

    def imu_ch(name):
        return imu_sliced[:, imu_cols.index(name)]

    s = str(imu_unit)
    return {
        "t_emg": t_emg_sliced,
        "t_imu": t_imu_sliced,
        "emg": {m: emg_ch(m) for m in EMG_COLORS},
        "orient": {
            "roll":  np.degrees(imu_ch(f"roll_rad{s}")),
            "pitch": np.degrees(imu_ch(f"pitch_rad{s}")),
            "yaw":   np.degrees(imu_ch(f"yaw_rad{s}")),
        },
        "accel": {
            "x": imu_ch(f"ax{s}") / 9.80665,
            "y": imu_ch(f"ay{s}") / 9.80665,
            "z": imu_ch(f"az{s}") / 9.80665,
        }
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--file", default=DEFAULT_FILE, help="segmented HDF5 file")
    ap.add_argument("--imu", type=int, default=1, choices=[1, 2], help="IMU unit: 1=forearm, 2=upper arm")
    ap.add_argument("--out", default=None, help="output path stem")
    args = ap.parse_args()

    file_path = args.file
    if not Path(file_path).exists():
        if Path(FALLBACK_FILE).exists():
            file_path = FALLBACK_FILE
        else:
            possible_paths = list(Path(__file__).resolve().parents[2].glob("**/participant_P01_session_01_segments.h5"))
            if possible_paths:
                file_path = str(possible_paths[0])
            else:
                raise FileNotFoundError(f"Could not find segment H5 file")

    out_dir = Path(__file__).resolve().parent / "segment_signal_plots"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_stem = Path(args.out) if args.out else out_dir / f"segment_side_by_side_imu{args.imu}"

    # Load all targeted segments
    data = {}
    for label, seg_key in TARGET_LOADS.items():
        data[label] = load_sliced_segment(file_path, seg_key, args.imu)

    # Compute global limits over these sliced 1-second snippets
    max_emg = 0.0
    max_orient = 0.0
    max_accel = 0.0
    for D in data.values():
        for muscle in EMG_COLORS:
            max_emg = max(max_emg, np.max(np.abs(D["emg"][muscle])))
        for axis in ["roll", "pitch", "yaw"]:
            max_orient = max(max_orient, np.max(np.abs(D["orient"][axis])))
        for axis in ["x", "y", "z"]:
            max_accel = max(max_accel, np.max(np.abs(D["accel"][axis])))

    # Rounding limits
    emg_lim = float(max(1.0, np.ceil(max_emg / 5.0) * 5.0))
    orient_lim = min(180.0, float(max(10.0, np.ceil(max_orient / 10.0) * 10.0)))
    accel_lim = float(max(0.5, np.ceil(max_accel * 2.0) / 2.0))

    print(f"Comparison Limits:\n  EMG: ±{emg_lim} mV\n  Orientation: ±{orient_lim}°\n  Acceleration: ±{accel_lim} g")

    # Create figure: 5 rows (channels), 6 columns (load classes)
    fig, axes = plt.subplots(
        5, 6, figsize=(16.0, 7.5), sharex="col", sharey="row",
        gridspec_kw={"wspace": 0.08, "hspace": 0.25}
    )

    columns = list(TARGET_LOADS.keys())
    muscles = list(EMG_COLORS.keys())

    for col_idx, col_name in enumerate(columns):
        D = data[col_name]
        
        # ── Row 1–3: EMG ──
        for row_idx, muscle in enumerate(muscles):
            ax = axes[row_idx, col_idx]
            color = EMG_COLORS[muscle]
            ax.plot(D["t_emg"], D["emg"][muscle], color=color, lw=0.7, zorder=3)
            ax.axhline(0, color="#BBBBBB", lw=0.5, zorder=1)
            ax.set_ylim(-emg_lim, emg_lim)
            
            # Styling
            ax.spines["top"].set_visible(False)
            ax.spines["right"].set_visible(False)
            ax.grid(True, axis="y", color="#E6E6E6", linewidth=0.6, zorder=0)
            
            # Column headers
            if row_idx == 0:
                ax.set_title(col_name, fontfamily=TITLE_FONT, fontweight="bold", fontsize=14, pad=8)
            
            # Row labels
            if col_idx == 0:
                ax.set_ylabel(f"{muscle}\n(mV)", fontsize=11, fontfamily=TITLE_FONT, fontweight="bold", color=color)

        # ── Row 4: Orientation ──
        ax = axes[3, col_idx]
        u = str(args.imu)
        orient_labels = {
            "roll": rf"$\phi_{u}$",
            "pitch": rf"$\theta_{u}$",
            "yaw": rf"$\psi_{u}$",
        }
        for (axis, sig), c in zip(D["orient"].items(), AXIS_COLORS):
            ax.plot(D["t_imu"], sig, color=c, lw=1.2, label=orient_labels.get(axis, axis), zorder=3)
        ax.set_ylim(-orient_lim, orient_lim)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.grid(True, axis="y", color="#E6E6E6", linewidth=0.6, zorder=0)
        
        if col_idx == 0:
            ax.set_ylabel("Orientation\n(°)", fontsize=11, fontfamily=TITLE_FONT, fontweight="bold", color="#2E8BFF")

        # ── Row 5: Acceleration ──
        ax = axes[4, col_idx]
        for (axis, sig), c in zip(D["accel"].items(), AXIS_COLORS):
            ax.plot(D["t_imu"], sig, color=c, lw=1.2, label=f"a$_{{{axis},{u}}}$", zorder=3)
        ax.set_ylim(-accel_lim, accel_lim)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.grid(True, axis="y", color="#E6E6E6", linewidth=0.6, zorder=0)
        
        if col_idx == 0:
            ax.set_ylabel("Lin. accel.\n(g)", fontsize=11, fontfamily=TITLE_FONT, fontweight="bold", color="#10C46B")

    # Add a single centered x-label at the bottom
    fig.supxlabel("Time (s)", fontsize=12, fontfamily=BODY_FONT, y=0.08)

    # Add a single unified legend beneath the plots
    import matplotlib.lines as mlines
    u = str(args.imu)
    handle_x = mlines.Line2D([], [], color=AXIS_COLORS[0], lw=1.5, label=rf"$\phi_{u}$ / a$_{{x,{u}}}$")
    handle_y = mlines.Line2D([], [], color=AXIS_COLORS[1], lw=1.5, label=rf"$\theta_{u}$ / a$_{{y,{u}}}$")
    handle_z = mlines.Line2D([], [], color=AXIS_COLORS[2], lw=1.5, label=rf"$\psi_{u}$ / a$_{{z,{u}}}$")
    fig.legend(
        handles=[handle_x, handle_y, handle_z],
        loc="lower center", ncol=3, fontsize=11, frameon=True,
        edgecolor="#DDDDDD", fancybox=False, bbox_to_anchor=(0.5, 0.02)
    )

    # Adjust layout to fit ticks, shared label, and legend nicely
    plt.subplots_adjust(left=0.08, right=0.97, top=0.91, bottom=0.14)

    # Save outputs
    png = out_stem.with_suffix(".png")
    pdf = out_stem.with_suffix(".pdf")
    svg = out_stem.with_suffix(".svg")
    
    fig.savefig(png, dpi=300)
    fig.savefig(pdf)
    fig.savefig(svg)
    plt.close(fig)
    print(f"Saved side-by-side figures:\n  {png}\n  {pdf}\n  {svg}")


if __name__ == "__main__":
    main()
