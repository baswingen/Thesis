"""
Segment-Level Data Quality Scanner
====================================
Scans every segment in every HDF5 segment file and flags corrupted or
low-quality segments.  Produces:

  1. segment_quality_report.csv   — per-segment metrics
  2. blacklist.json               — segments to exclude (consumed by data_loader)
  3. quality_summary.txt          — human-readable overview

Quality Checks
--------------
  1. IMU flatline detection   (consecutive identical samples)
  2. EMG flatline detection   (consecutive identical samples)
  3. Near-zero variance       (dead channel detection)
  4. NaN / Inf presence
  5. Extreme outlier density  (>6σ from channel global mean)
  6. Segment duration bounds  (min and max)
  7. Cross-modal consistency  (EMG active but IMU frozen, or vice versa)

Run from the project root:
    python data_analysis/segment_quality_scan.py

    Options:
        --segments-dir PATH     (default: /Volumes/Laurens SSD/BasData/segments)
        --out-dir PATH          (default: data_analysis/results)
        --flatline-threshold 5  (percent; flag segments with flatline% above this)
        --min-duration 0.5      (seconds; flag segments shorter than this)
        --max-duration 3.5      (seconds; flag segments longer than this)
        --outlier-sigma 6       (σ threshold for outlier detection)
        --outlier-threshold 2   (percent; flag segments with outlier% above this)
"""

import argparse
import json
import shutil
import sys
import tempfile
from collections import defaultdict
from pathlib import Path

import h5py
import numpy as np
import pandas as pd
from tqdm import tqdm

# Project root on sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _decode(raw) -> list[str]:
    return [c.decode() if isinstance(c, (bytes, np.bytes_)) else str(c) for c in raw]


def _flatline_ratio(signal_1d: np.ndarray) -> float:
    """Fraction of samples that are part of a consecutive-identical run."""
    if signal_1d.shape[0] < 2:
        return 0.0
    diffs_zero = np.diff(signal_1d) == 0
    return float(diffs_zero.sum()) / (signal_1d.shape[0] - 1)


def _max_flatline_run(signal_1d: np.ndarray) -> int:
    """Length of the longest consecutive run of identical values."""
    if signal_1d.shape[0] < 2:
        return 0
    diffs_zero = np.diff(signal_1d) == 0
    if not np.any(diffs_zero):
        return 0
    # Find run lengths of True values
    changes = np.diff(diffs_zero.astype(int))
    starts = np.where(changes == 1)[0] + 1
    ends = np.where(changes == -1)[0] + 1
    # Handle edge cases: starts/ends at boundaries
    if diffs_zero[0]:
        starts = np.r_[0, starts]
    if diffs_zero[-1]:
        ends = np.r_[ends, len(diffs_zero)]
    if len(starts) == 0:
        return 0
    return int(np.max(ends - starts)) + 1  # +1 because diff reduces length by 1


def robust_copy(src: Path, dst: Path):
    """Copy a file, retrying once on failure (iCloud eviction)."""
    try:
        shutil.copy2(src, dst)
    except OSError:
        import time
        time.sleep(1)
        shutil.copy2(src, dst)


# ─────────────────────────────────────────────────────────────────────────────
# Phase 1: Collect global channel statistics (for σ-based outlier detection)
# ─────────────────────────────────────────────────────────────────────────────

def _collect_global_stats(
    h5_files: list[Path],
) -> dict[str, dict]:
    """
    Two-pass Welford online algorithm to compute per-channel mean and std
    across all segments without loading the entire dataset into RAM.

    Returns {channel_name: {"mean": float, "std": float}}
    """
    print("\n[1/3] Computing global channel statistics (online) …")

    # Accumulators: count, mean, M2 (Welford)
    acc: dict[str, dict] = {}  # channel -> {"n": int, "mean": float, "M2": float}

    for h5_path in tqdm(h5_files, desc="Global stats", unit="file"):
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir) / h5_path.name
            robust_copy(h5_path, tmp_path)

            with h5py.File(tmp_path, "r") as f:
                seg_keys = sorted(k for k in f.keys() if k.startswith("segment_"))
                for key in seg_keys:
                    grp = f[key]
                    for ds_name in ("emg", "imu"):
                        if ds_name not in grp:
                            continue
                        data = grp[ds_name][:]
                        cols = _decode(grp[ds_name].attrs.get("column_names", []))
                        for ci, cname in enumerate(cols):
                            col_data = data[:, ci]
                            # Skip NaN for stats
                            valid = col_data[np.isfinite(col_data)]
                            if valid.size == 0:
                                continue
                            if cname not in acc:
                                acc[cname] = {"n": 0, "mean": 0.0, "M2": 0.0}
                            a = acc[cname]
                            for x in valid:  # Welford update
                                a["n"] += 1
                                delta = x - a["mean"]
                                a["mean"] += delta / a["n"]
                                delta2 = x - a["mean"]
                                a["M2"] += delta * delta2

    stats = {}
    for cname, a in acc.items():
        if a["n"] < 2:
            stats[cname] = {"mean": a["mean"], "std": 0.0}
        else:
            stats[cname] = {
                "mean": a["mean"],
                "std": np.sqrt(a["M2"] / (a["n"] - 1)),
            }
    print(f"  Computed global stats for {len(stats)} channels.")
    return stats


# ─────────────────────────────────────────────────────────────────────────────
# Phase 2: Per-segment quality scan
# ─────────────────────────────────────────────────────────────────────────────

def _scan_segments(
    h5_files: list[Path],
    global_stats: dict[str, dict],
    outlier_sigma: float,
) -> list[dict]:
    """Scan every segment and return a list of metric dicts."""
    print("\n[2/3] Scanning segment quality …")
    rows: list[dict] = []

    for h5_path in tqdm(h5_files, desc="Scanning", unit="file"):
        participant = "unknown"
        parts = h5_path.stem.split("_")
        if "participant" in parts:
            participant = parts[parts.index("participant") + 1]

        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir) / h5_path.name
            robust_copy(h5_path, tmp_path)

            with h5py.File(tmp_path, "r") as f:
                seg_keys = sorted(k for k in f.keys() if k.startswith("segment_"))

                for key in seg_keys:
                    grp = f[key]
                    row: dict = {
                        "file": h5_path.name,
                        "participant": participant,
                        "segment": key,
                        "state": "",
                        "weight": -1.0,
                    }

                    # --- Metadata ---
                    state = grp.attrs.get("state", b"unknown")
                    if isinstance(state, bytes):
                        state = state.decode()
                    row["state"] = state
                    row["weight"] = float(grp.attrs.get("weight", -1.0))

                    # --- Load signals ---
                    emg = grp["emg"][:] if "emg" in grp else np.empty((0, 0))
                    imu = grp["imu"][:] if "imu" in grp else np.empty((0, 0))
                    emg_cols = _decode(grp["emg"].attrs.get("column_names", [])) if "emg" in grp else []
                    imu_cols = _decode(grp["imu"].attrs.get("column_names", [])) if "imu" in grp else []
                    emg_fs = float(grp["emg"].attrs.get("fs", 2000.0)) if "emg" in grp else 2000.0

                    n_samples = max(emg.shape[0] if emg.size > 0 else 0,
                                    imu.shape[0] if imu.size > 0 else 0)
                    row["n_samples"] = n_samples
                    row["duration_sec"] = n_samples / emg_fs if emg_fs > 0 else 0.0

                    # ── Check 1 & 2: Flatline detection (IMU + EMG) ──────────
                    imu_flatline_max = 0.0
                    imu_flatline_worst_ch = ""
                    imu_flatline_max_run = 0

                    for ci, cname in enumerate(imu_cols):
                        ratio = _flatline_ratio(imu[:, ci])
                        run = _max_flatline_run(imu[:, ci])
                        if ratio > imu_flatline_max:
                            imu_flatline_max = ratio
                            imu_flatline_worst_ch = cname
                            imu_flatline_max_run = run

                    row["imu_flatline_pct"] = imu_flatline_max * 100
                    row["imu_flatline_worst_ch"] = imu_flatline_worst_ch
                    row["imu_flatline_max_run"] = imu_flatline_max_run

                    emg_flatline_max = 0.0
                    emg_flatline_worst_ch = ""

                    for ci, cname in enumerate(emg_cols):
                        ratio = _flatline_ratio(emg[:, ci])
                        if ratio > emg_flatline_max:
                            emg_flatline_max = ratio
                            emg_flatline_worst_ch = cname

                    row["emg_flatline_pct"] = emg_flatline_max * 100
                    row["emg_flatline_worst_ch"] = emg_flatline_worst_ch

                    # ── Check 3: Near-zero variance (dead channels) ──────────
                    dead_chs = []
                    for ci, cname in enumerate(imu_cols):
                        if np.var(imu[:, ci]) < 1e-12:
                            dead_chs.append(cname)
                    for ci, cname in enumerate(emg_cols):
                        if np.var(emg[:, ci]) < 1e-12:
                            dead_chs.append(cname)

                    row["n_dead_channels"] = len(dead_chs)
                    row["dead_channels"] = ";".join(dead_chs) if dead_chs else ""

                    # ── Check 4: NaN / Inf ───────────────────────────────────
                    nan_count = 0
                    inf_count = 0
                    if emg.size > 0:
                        nan_count += int(np.isnan(emg).sum())
                        inf_count += int(np.isinf(emg).sum())
                    if imu.size > 0:
                        nan_count += int(np.isnan(imu).sum())
                        inf_count += int(np.isinf(imu).sum())

                    row["nan_count"] = nan_count
                    row["inf_count"] = inf_count

                    # ── Check 5: Outlier density (σ-based) ───────────────────
                    outlier_count = 0
                    total_values = 0

                    for ds, cols in [(emg, emg_cols), (imu, imu_cols)]:
                        if ds.size == 0:
                            continue
                        for ci, cname in enumerate(cols):
                            if cname not in global_stats:
                                continue
                            gs = global_stats[cname]
                            if gs["std"] < 1e-12:
                                continue
                            lo = gs["mean"] - outlier_sigma * gs["std"]
                            hi = gs["mean"] + outlier_sigma * gs["std"]
                            col_data = ds[:, ci]
                            outlier_count += int(np.sum((col_data < lo) | (col_data > hi)))
                            total_values += col_data.shape[0]

                    row["outlier_pct"] = (outlier_count / total_values * 100) if total_values > 0 else 0.0

                    # ── Check 7: Cross-modal consistency ─────────────────────
                    emg_energy = float(np.mean(np.var(emg, axis=0))) if emg.size > 0 else 0.0
                    imu_energy = float(np.mean(np.var(imu, axis=0))) if imu.size > 0 else 0.0
                    row["emg_mean_var"] = emg_energy
                    row["imu_mean_var"] = imu_energy
                    # Flag: one modality alive, the other dead
                    row["cross_modal_mismatch"] = (
                        (emg_energy > 1e-6 and imu_energy < 1e-12)
                        or (imu_energy > 1e-6 and emg_energy < 1e-12)
                    )

                    rows.append(row)

    print(f"  Scanned {len(rows)} segments across {len(h5_files)} files.")
    return rows


# ─────────────────────────────────────────────────────────────────────────────
# Phase 3: Apply thresholds → blacklist
# ─────────────────────────────────────────────────────────────────────────────

def _build_blacklist(
    df: pd.DataFrame,
    flatline_threshold: float,
    min_duration: float,
    max_duration: float,
    outlier_threshold: float,
) -> pd.DataFrame:
    """
    Flag segments that fail any quality check.
    Returns the input DataFrame with added 'flagged' and 'flag_reasons' columns.
    """
    flags = []
    reasons_list = []

    for _, row in df.iterrows():
        seg_reasons = []

        if row["imu_flatline_pct"] > flatline_threshold:
            seg_reasons.append(f"imu_flatline({row['imu_flatline_pct']:.1f}%/{row['imu_flatline_worst_ch']})")

        if row["emg_flatline_pct"] > flatline_threshold:
            seg_reasons.append(f"emg_flatline({row['emg_flatline_pct']:.1f}%/{row['emg_flatline_worst_ch']})")

        if row["n_dead_channels"] > 0:
            seg_reasons.append(f"dead_channels({row['dead_channels']})")

        if row["nan_count"] > 0:
            seg_reasons.append(f"nan({row['nan_count']})")

        if row["inf_count"] > 0:
            seg_reasons.append(f"inf({row['inf_count']})")

        if row["outlier_pct"] > outlier_threshold:
            seg_reasons.append(f"outlier({row['outlier_pct']:.2f}%)")

        if row["duration_sec"] < min_duration:
            seg_reasons.append(f"too_short({row['duration_sec']:.3f}s)")

        if row["duration_sec"] > max_duration:
            seg_reasons.append(f"too_long({row['duration_sec']:.3f}s)")

        if row["cross_modal_mismatch"]:
            seg_reasons.append("cross_modal_mismatch")

        flags.append(len(seg_reasons) > 0)
        reasons_list.append("; ".join(seg_reasons) if seg_reasons else "")

    df["flagged"] = flags
    df["flag_reasons"] = reasons_list
    return df


# ─────────────────────────────────────────────────────────────────────────────
# Output writers
# ─────────────────────────────────────────────────────────────────────────────

def _write_outputs(df: pd.DataFrame, out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1. Full CSV report
    csv_path = out_dir / "segment_quality_report.csv"
    df.to_csv(csv_path, index=False, float_format="%.4f")
    print(f"  ✓ {csv_path.relative_to(PROJECT_ROOT)}")

    # 2. Blacklist JSON
    flagged = df[df["flagged"]]
    blacklist = []
    for _, row in flagged.iterrows():
        blacklist.append({
            "file": row["file"],
            "segment": row["segment"],
            "participant": row["participant"],
            "reasons": row["flag_reasons"],
        })

    bl_path = out_dir / "blacklist.json"
    with open(bl_path, "w") as f:
        json.dump(blacklist, f, indent=2)
    print(f"  ✓ {bl_path.relative_to(PROJECT_ROOT)}")

    # 3. Summary text
    total = len(df)
    n_flagged = int(flagged.shape[0])
    n_clean = total - n_flagged

    lines = [
        "=" * 65,
        "  SEGMENT-LEVEL DATA QUALITY REPORT",
        "=" * 65,
        "",
        f"  Total segments scanned : {total:,}",
        f"  Clean segments         : {n_clean:,} ({n_clean/total*100:.1f}%)",
        f"  Flagged for exclusion  : {n_flagged:,} ({n_flagged/total*100:.1f}%)",
        "",
    ]

    # Reason breakdown
    if n_flagged > 0:
        reason_counts = defaultdict(int)
        for reasons_str in flagged["flag_reasons"]:
            for r in reasons_str.split("; "):
                tag = r.split("(")[0]
                reason_counts[tag] += 1

        lines.append("── Flag Reason Breakdown ──────────────────────────────────────")
        for tag, count in sorted(reason_counts.items(), key=lambda x: -x[1]):
            lines.append(f"  {tag:<30s}  {count:>5d} segments")
        lines.append("")

    # Per-participant breakdown
    lines.append("── Per-Participant Breakdown ──────────────────────────────────")
    lines.append(f"  {'Participant':<14s}  {'Total':>6s}  {'Flagged':>7s}  {'Clean':>6s}  {'Clean%':>6s}")
    lines.append("  " + "-" * 50)
    for p in sorted(df["participant"].unique()):
        p_df = df[df["participant"] == p]
        p_total = len(p_df)
        p_flagged = int(p_df["flagged"].sum())
        p_clean = p_total - p_flagged
        pct = p_clean / p_total * 100 if p_total > 0 else 0
        lines.append(f"  {p:<14s}  {p_total:>6d}  {p_flagged:>7d}  {p_clean:>6d}  {pct:>5.1f}%")
    lines.append("")

    # Distribution stats for key metrics
    lines.append("── Key Metric Distributions ──────────────────────────────────")
    for col, label in [
        ("imu_flatline_pct", "IMU Flatline %"),
        ("emg_flatline_pct", "EMG Flatline %"),
        ("outlier_pct", "Outlier %"),
        ("duration_sec", "Duration (s)"),
    ]:
        vals = df[col]
        lines.append(
            f"  {label:<20s}  "
            f"mean={vals.mean():.3f}  "
            f"median={vals.median():.3f}  "
            f"p95={vals.quantile(0.95):.3f}  "
            f"max={vals.max():.3f}"
        )
    lines.append("")

    # Weight class balance (after cleaning)
    clean_df = df[~df["flagged"]]
    lines.append("── Weight Class Balance (clean segments) ─────────────────────")
    weight_counts = clean_df.groupby("weight").size().sort_index()
    for w, count in weight_counts.items():
        pct = count / len(clean_df) * 100
        bar = "█" * int(pct)
        lines.append(f"  {w:>6.3f} kg  {count:>5d}  ({pct:>5.1f}%)  {bar}")
    lines.append("")

    lines.append("=" * 65)

    txt_path = out_dir / "quality_summary.txt"
    report_text = "\n".join(lines)
    txt_path.write_text(report_text, encoding="utf-8")
    print(f"  ✓ {txt_path.relative_to(PROJECT_ROOT)}")
    print(report_text)


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Segment-level data quality scanner.")
    parser.add_argument("--segments-dir", default="/Volumes/Laurens SSD/BasData/segments",
                        help="Directory containing segmented *.h5 files.")
    parser.add_argument("--out-dir", default=None,
                        help="Output directory (default: data_analysis/results).")
    parser.add_argument("--flatline-threshold", type=float, default=5.0,
                        help="Flag segments with flatline%% above this (default: 5.0).")
    parser.add_argument("--min-duration", type=float, default=0.5,
                        help="Flag segments shorter than this in seconds (default: 0.5).")
    parser.add_argument("--max-duration", type=float, default=3.5,
                        help="Flag segments longer than this in seconds (default: 3.5).")
    parser.add_argument("--outlier-sigma", type=float, default=6.0,
                        help="Sigma threshold for outlier detection (default: 6.0).")
    parser.add_argument("--outlier-threshold", type=float, default=2.0,
                        help="Flag segments with outlier%% above this (default: 2.0).")
    args = parser.parse_args()

    seg_dir = Path(args.segments_dir)
    out_dir = Path(args.out_dir) if args.out_dir else PROJECT_ROOT / "data_analysis" / "results"

    h5_files = sorted([p for p in seg_dir.glob("*.h5") if not p.name.startswith("._")])
    if not h5_files:
        sys.exit(f"[ERR] No HDF5 files found in {seg_dir}")

    print(f"Found {len(h5_files)} segment files in {seg_dir}")

    # Phase 1: Global stats
    global_stats = _collect_global_stats(h5_files)

    # Phase 2: Per-segment scan
    rows = _scan_segments(h5_files, global_stats, outlier_sigma=args.outlier_sigma)

    # Phase 3: Apply thresholds and generate outputs
    df = pd.DataFrame(rows)
    df = _build_blacklist(df, args.flatline_threshold, args.min_duration, args.max_duration, args.outlier_threshold)

    print("\n[3/3] Writing outputs …")
    _write_outputs(df, out_dir)
    print("\nDone.")


if __name__ == "__main__":
    main()
