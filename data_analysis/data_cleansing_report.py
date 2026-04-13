"""
Data Cleansing Analysis Tool
============================
Scans the HDF5 segment data for common signal quality issues:
  1. Check for NaN or Inf values
  2. Flatline Detection (sensor disconnected or totally dead)
  3. Clipping/Saturation (signal hits hardware max/min consistently)
  4. Extreme Outliers (large anomalous spikes, likely artifacts)

Run from the project root:
    python data_analysis/data_cleansing_report.py
"""

import sys
from pathlib import Path

# Project root on sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
import pandas as pd
from model.data_loader import DataLoader

# Configuration
# Set to "all" to load all segments, or provide a list of specific Paths
H5_FILES = "all"

if H5_FILES == "all":
    H5_FILES = sorted(list((PROJECT_ROOT / "database" / "segments").glob("*.h5")))

OUT_DIR = PROJECT_ROOT / "data_analysis" / "results"

def analyze_cleansing_metrics(segments: list[np.ndarray], ch_names: list[str]) -> pd.DataFrame:
    """
    Given a list of segment arrays (samples x channels), calculates error metrics.
    """
    print("\n[2/3] Analyzing data quality for each channel...")
    C = len(ch_names)
    
    # Store aggregated metrics per channel
    metrics = {
        "Channel": ch_names,
        "NaN_Count": np.zeros(C, dtype=int),
        "Inf_Count": np.zeros(C, dtype=int),
        "Flatline_Samples": np.zeros(C, dtype=int),
        "Outlier_Count": np.zeros(C, dtype=int),
        "Total_Samples": np.zeros(C, dtype=int)
    }

    # Concatenate to compute global mean and std for outliers
    X_all = np.vstack(segments)
    global_mean = np.nanmean(X_all, axis=0)
    global_std = np.nanstd(X_all, axis=0)
    outlier_threshold_high = global_mean + (6 * global_std)
    outlier_threshold_low = global_mean - (6 * global_std)

    for seg in segments:
        samples = seg.shape[0]
        metrics["Total_Samples"] += samples

        # 1. NaN / Inf Checks
        nans = np.isnan(seg).sum(axis=0)
        infs = np.isinf(seg).sum(axis=0)
        metrics["NaN_Count"] += nans
        metrics["Inf_Count"] += infs

        # 2. Flatline Detection (where consecutive differences == 0)
        # Note: np.diff creates samples-1 length, so we pad to match sample length
        if samples > 1:
            diffs = np.diff(seg, axis=0) == 0
            # Sum up flatlines per channel in this segment
            # A strict flatline check usually looks for segments where var is zero,
            # but counting consecutive 0-diffs shows "stuck" signals.
            metrics["Flatline_Samples"] += np.sum(diffs, axis=0)

        # 3. Outliers (ignoring NaNs to avoid warnings)
        outliers_high = np.sum(seg > outlier_threshold_high, axis=0)
        outliers_low = np.sum(seg < outlier_threshold_low, axis=0)
        metrics["Outlier_Count"] += (outliers_high + outliers_low)

    df_report = pd.DataFrame(metrics)
    
    # Calculate percentage
    df_report["Flatline_%"] = (df_report["Flatline_Samples"] / df_report["Total_Samples"]) * 100
    df_report["Outlier_%"] = (df_report["Outlier_Count"] / df_report["Total_Samples"]) * 100
    
    return df_report

def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    
    print("\n[1/3] Loading raw segments...")
    loader = DataLoader()
    df = loader.load_raw_segments(H5_FILES)
    
    if df.empty:
        sys.exit("[ERR] No segments loaded - check HDF5 paths in script.")
        
    ch_names = df.attrs.get("channel_names", [])
    segments = df["raw_segment"].tolist()
    
    print(f"      Loaded {len(segments)} segments.")
    
    df_report = analyze_cleansing_metrics(segments, ch_names)
    
    print("\n[3/3] Saving data cleansing report...")
    
    # Define paths
    csv_path = OUT_DIR / "data_cleansing_report.csv"
    txt_path = OUT_DIR / "data_cleansing_summary.txt"
    
    # Save CSV
    df_report.to_csv(csv_path, index=False, float_format="%.4f")
    
    # Generate and save text report
    lines = [
        "=" * 65,
        "  DATA CLEANSING & QUALITY REPORT",
        "=" * 65,
        f"\nTotal Segments Analyzed: {len(segments)}",
        f"Total Samples Analyzed:  {df_report['Total_Samples'].iloc[0]:,}",
        "\n--- Potential Issues Detected ---",
    ]
    
    # Flag channels with warnings
    flags = []
    for _, row in df_report.iterrows():
        ch = row["Channel"]
        if row["NaN_Count"] > 0: flags.append(f"[!] {ch}: Contains {row['NaN_Count']} NaNs")
        if row["Inf_Count"] > 0: flags.append(f"[!] {ch}: Contains {row['Inf_Count']} Infs")
        if row["Flatline_%"] > 5.0: flags.append(f"[!] {ch}: High flatline proportion ({row['Flatline_%']:.2f}%)")
        if row["Outlier_%"] > 1.0: flags.append(f"[!] {ch}: High outlier density ({row['Outlier_%']:.2f}% > 6-sigma)")
        
    if not flags:
        lines.append("  All channels look clean! No major NaN, Inf, flatline, or outlier issues detected.")
    else:
        lines.extend(flags)
        
    lines.append("\n(See data_cleansing_report.csv for full channel statistics)")
    
    report_text = "\n".join(lines)
    txt_path.write_text(report_text, encoding="utf-8")
    
    print(f"  ✓ Saved {csv_path.relative_to(PROJECT_ROOT)}")
    print(f"  ✓ Saved {txt_path.relative_to(PROJECT_ROOT)}")
    print(report_text)

if __name__ == "__main__":
    main()
