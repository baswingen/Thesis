"""
Channel Cross-Correlation & Covariance Analysis
================================================
Loads all raw EMG + IMU segments from the P01 dataset (sessions 1–3),
concatenates every sample into one large matrix, and produces:

  1. Pearson correlation heatmaps  (EMG-only, IMU-only, full C×C)
  2. Covariance eigenvalue spectrum  (PCA-style explained variance)
  3. Plain-text summary report

Outputs are written to:  data_analysis/results/

Run from the project root:
    python data_analysis/channel_analysis.py
"""

from __future__ import annotations

import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Project root on sys.path so model.* imports work
# ---------------------------------------------------------------------------
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")          # non-interactive backend — safe on any machine
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from sklearn.preprocessing import StandardScaler

from model.data_loader import DataLoader
from model.config_model import CHANNEL_CONFIG

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# Set to "all" to load all segments, or provide a list of specific Paths
H5_FILES = "all"

if H5_FILES == "all":
    H5_FILES = sorted(list((PROJECT_ROOT / "database" / "segments").glob("*.h5")))

OUT_DIR = PROJECT_ROOT / "data_analysis" / "results"

# ── Plot style ──────────────────────────────────────────────────────────────
CMAP_DIV   = "RdBu_r"     # diverging  → correlation heatmaps
CMAP_SEQ   = "viridis"    # sequential → eigenspectrum
FIG_DPI    = 150
FONT_SMALL = 7
FONT_MED   = 9

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.size":   FONT_MED,
    "axes.titlesize": FONT_MED,
    "axes.labelsize": FONT_MED,
    "xtick.labelsize": FONT_SMALL,
    "ytick.labelsize": FONT_SMALL,
})

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _short_name(full_name: str) -> str:
    """Shorten long channel names so axis ticks stay readable."""
    replacements = {
        "Anterior Deltoid":              "A.Delt",
        "Lateral Deltoid":               "L.Delt",
        "Posterior Deltoid":             "P.Delt",
        "Triceps Brachii":               "Triceps",
        "Biceps Brachii":                "Biceps",
        "Brachioradialis":               "Brach.",
        "Flexor Carpi Ulnaris (FCU)":    "FCU",
        "Extensor Carpi Radialis (ECR)": "ECR",
        "roll_rad_diff":  "roll_diff",
        "pitch_rad_diff": "pitch_diff",
        "yaw_rad_diff":   "yaw_diff",
    }
    return replacements.get(full_name, full_name)


def _annotate_heatmap(ax, data, fmt=".2f", fontsize=6, threshold=0.5):
    """Overlay numeric annotations on a heatmap axes."""
    for i in range(data.shape[0]):
        for j in range(data.shape[1]):
            val = data[i, j]
            color = "white" if abs(val) > threshold else "black"
            ax.text(j, i, f"{val:{fmt}}", ha="center", va="center",
                    fontsize=fontsize, color=color)


def _save(fig, name: str):
    path = OUT_DIR / name
    fig.savefig(path, dpi=FIG_DPI, bbox_inches="tight")
    plt.close(fig)
    print(f"  ✓  {path.relative_to(PROJECT_ROOT)}")


# ---------------------------------------------------------------------------
# 1 — Load & flatten all raw segments
# ---------------------------------------------------------------------------

def load_data() -> tuple[np.ndarray, list[str]]:
    """
    Returns
    -------
    X      : (total_samples, n_channels)  — z-score normalised
    ch_names: list of channel name strings (EMG first, then IMU)
    """
    print("\n[1/4] Loading raw segments …")
    loader = DataLoader()
    df = loader.load_raw_segments(H5_FILES)

    if df.empty:
        sys.exit("[ERR] No segments loaded — check HDF5 paths.")

    ch_names: list[str] = df.attrs.get("channel_names", [])
    segments: list[np.ndarray] = df["raw_segment"].tolist()

    # Concatenate every time sample from every segment
    X_raw = np.vstack(segments)                        # (total_samples, C)
    print(f"       Segments  : {len(segments)}")
    print(f"       Samples   : {X_raw.shape[0]:,}")
    print(f"       Channels  : {X_raw.shape[1]}  {ch_names}")

    # z-score per channel (same as model pipeline)
    scaler = StandardScaler()
    X = scaler.fit_transform(X_raw)

    return X, ch_names


# ---------------------------------------------------------------------------
# 2 — Pearson correlation heatmaps
# ---------------------------------------------------------------------------

def _corr_heatmap(corr: np.ndarray, labels: list[str], title: str,
                  out_name: str, annotate: bool = True):
    C = len(labels)
    fig_size = max(6, C * 0.55)
    fig, ax = plt.subplots(figsize=(fig_size, fig_size * 0.85))

    im = ax.imshow(corr, cmap=CMAP_DIV, vmin=-1, vmax=1, aspect="auto")
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label="Pearson r")

    ax.set_xticks(range(C))
    ax.set_yticks(range(C))
    short = [_short_name(n) for n in labels]
    ax.set_xticklabels(short, rotation=45, ha="right")
    ax.set_yticklabels(short)
    ax.set_title(title, pad=10, fontsize=FONT_MED + 1, fontweight="bold")

    if annotate and C <= 26:    # skip annotations for very large matrices
        _annotate_heatmap(ax, corr, fontsize=max(4, 7 - C // 5))

    plt.tight_layout()
    _save(fig, out_name)


def plot_correlations(X: np.ndarray, ch_names: list[str]):
    print("\n[2/4] Computing & plotting Pearson correlations …")

    # Identify EMG vs IMU index ranges from CHANNEL_CONFIG
    emg_ch_names = list(CHANNEL_CONFIG.get("emg_channels", {}).keys())
    imu_ch_names = list(CHANNEL_CONFIG.get("imu_channels", {}).keys())

    emg_idx = [i for i, n in enumerate(ch_names) if n in emg_ch_names]
    imu_idx = [i for i, n in enumerate(ch_names) if n in imu_ch_names]

    corr_full = np.corrcoef(X.T)     # (C, C)

    # — Full heatmap —
    _corr_heatmap(corr_full, ch_names,
                  title="Full Channel Correlation Matrix (EMG + IMU)",
                  out_name="corr_full.png",
                  annotate=(len(ch_names) <= 30))

    # — EMG-only —
    if emg_idx:
        emg_labels = [ch_names[i] for i in emg_idx]
        _corr_heatmap(corr_full[np.ix_(emg_idx, emg_idx)], emg_labels,
                      title="EMG Channel Correlation Matrix",
                      out_name="corr_emg.png")

    # — IMU-only —
    if imu_idx:
        imu_labels = [ch_names[i] for i in imu_idx]
        _corr_heatmap(corr_full[np.ix_(imu_idx, imu_idx)], imu_labels,
                      title="IMU Channel Correlation Matrix",
                      out_name="corr_imu.png",
                      annotate=True)

    # Save CSV
    csv_path = OUT_DIR / "corr_matrix.csv"
    pd.DataFrame(corr_full, index=ch_names, columns=ch_names).to_csv(csv_path, float_format="%.4f")
    print(f"  ✓  {csv_path.relative_to(PROJECT_ROOT)}")

    return corr_full


# ---------------------------------------------------------------------------
# 3 — Covariance eigenvalue spectrum
# ---------------------------------------------------------------------------

def plot_eigenspectrum(X: np.ndarray, ch_names: list[str]) -> tuple[np.ndarray, np.ndarray]:
    """
    PCA eigenvalue decomposition of the covariance matrix.
    Returns (eigenvalues_descending, cumulative_explained_variance).
    """
    print("\n[3/4] Computing covariance eigenspectrum …")
    C = X.shape[1]
    cov = np.cov(X.T)           # (C, C)

    eigvals, _ = np.linalg.eigh(cov)           # ascending order
    eigvals = eigvals[::-1]                     # descending
    eigvals = np.maximum(eigvals, 0.0)          # numerical safety

    explained = eigvals / eigvals.sum()
    cum_explained = np.cumsum(explained)

    n_90 = int(np.searchsorted(cum_explained, 0.90)) + 1
    n_95 = int(np.searchsorted(cum_explained, 0.95)) + 1
    print(f"       Components for 90% variance: {n_90} / {C}")
    print(f"       Components for 95% variance: {n_95} / {C}")

    # — Plot —
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))

    # Left: individual explained variance (bar)
    ax = axes[0]
    ax.bar(range(1, C + 1), explained * 100, color="#4C72B0", edgecolor="white", linewidth=0.4)
    ax.set_xlabel("Principal Component")
    ax.set_ylabel("Explained Variance (%)")
    ax.set_title("Individual Explained Variance per Component")
    ax.xaxis.set_major_locator(ticker.MaxNLocator(integer=True))

    # Right: cumulative explained variance
    ax2 = axes[1]
    ax2.plot(range(1, C + 1), cum_explained * 100, marker="o",
             markersize=4, color="#C44E52", linewidth=1.5)
    ax2.axhline(90, linestyle="--", linewidth=0.9, color="gray",  label="90%")
    ax2.axhline(95, linestyle="--", linewidth=0.9, color="black", label="95%")
    ax2.axvline(n_90, linestyle=":",  linewidth=0.9, color="gray",
                label=f"k={n_90} (90%)")
    ax2.axvline(n_95, linestyle=":",  linewidth=0.9, color="black",
                label=f"k={n_95} (95%)")
    ax2.set_xlabel("Number of Components")
    ax2.set_ylabel("Cumulative Explained Variance (%)")
    ax2.set_title("Cumulative Explained Variance")
    ax2.set_ylim(0, 101)
    ax2.xaxis.set_major_locator(ticker.MaxNLocator(integer=True))
    ax2.legend(fontsize=FONT_SMALL)

    fig.suptitle("Covariance Eigenvalue Spectrum — All Channels (z-scored)",
                 fontsize=FONT_MED + 1, fontweight="bold")
    plt.tight_layout()
    _save(fig, "eigenvalue_spectrum.png")

    # Save CSV
    ev_df = pd.DataFrame({
        "component": range(1, C + 1),
        "eigenvalue": eigvals,
        "explained_variance": explained,
        "cumulative_explained_variance": cum_explained,
    })
    csv_path = OUT_DIR / "explained_variance.csv"
    ev_df.to_csv(csv_path, index=False, float_format="%.6f")
    print(f"  ✓  {csv_path.relative_to(PROJECT_ROOT)}")

    return eigvals, cum_explained, n_90, n_95


# ---------------------------------------------------------------------------
# 4 — Summary report
# ---------------------------------------------------------------------------

def write_report(X: np.ndarray, ch_names: list[str],
                 corr: np.ndarray,
                 eigvals: np.ndarray, cum_explained: np.ndarray,
                 n_90: int, n_95: int):
    print("\n[4/4] Writing summary report …")
    C = len(ch_names)

    # Top correlated pairs (upper triangle, exclude diagonal)
    pairs = []
    for i in range(C):
        for j in range(i + 1, C):
            pairs.append((abs(corr[i, j]), corr[i, j], ch_names[i], ch_names[j]))
    pairs.sort(reverse=True)

    # Cluster detection: channels with mean |r| > 0.7 to all others
    cluster_threshold = 0.7
    mean_abs_corr = (np.abs(corr) - np.eye(C)).sum(axis=1) / (C - 1)
    clustered = [(ch_names[i], float(mean_abs_corr[i]))
                 for i in range(C) if mean_abs_corr[i] >= cluster_threshold]
    clustered.sort(key=lambda x: -x[1])

    lines = [
        "=" * 65,
        "  CHANNEL CROSS-CORRELATION & COVARIANCE ANALYSIS REPORT",
        "=" * 65,
        "",
        "── Dataset Statistics ─────────────────────────────────────────",
        f"  Sessions      : P01 sessions 01, 02, 03",
        f"  Total samples : {X.shape[0]:,}",
        f"  Channels      : {C}",
        "  Channel list  :",
    ]
    for i, n in enumerate(ch_names, 1):
        lines.append(f"    {i:2d}. {n}")

    lines += [
        "",
        "── Pearson Correlation — Top 15 Most Correlated Pairs ─────────",
        f"  {'|r|':>6}   {'r':>7}   Channel A                      Channel B",
        "  " + "-" * 60,
    ]
    for abs_r, r, a, b in pairs[:15]:
        lines.append(f"  {abs_r:6.3f}   {r:7.3f}   {a:<30} {b}")

    lines += [
        "",
        "── Covariance Eigenvalue Spectrum ─────────────────────────────",
        f"  Components needed for 90% variance : {n_90:2d} / {C}",
        f"  Components needed for 95% variance : {n_95:2d} / {C}",
        f"  Top eigenvalue fraction (λ₁ / Σλ)  : {eigvals[0]/eigvals.sum():.3f}",
    ]

    if clustered:
        lines += [
            "",
            f"── Highly Correlated Channels (mean |r| ≥ {cluster_threshold}) ──────────",
        ]
        for name, val in clustered:
            lines.append(f"  {name:<40}  mean |r| = {val:.3f}")
    else:
        lines += [
            "",
            f"  No channels exceeded the mean |r| ≥ {cluster_threshold} cluster threshold.",
        ]

    lines += ["", "=" * 65, ""]

    report_text = "\n".join(lines)
    report_path = OUT_DIR / "analysis_report.txt"
    report_path.write_text(report_text, encoding="utf-8")
    print(f"  ✓  {report_path.relative_to(PROJECT_ROOT)}")
    print()
    print(report_text)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    print(f"Output directory: {OUT_DIR.relative_to(PROJECT_ROOT)}")

    X, ch_names = load_data()
    corr        = plot_correlations(X, ch_names)
    eigvals, cum_explained, n_90, n_95 = plot_eigenspectrum(X, ch_names)
    write_report(X, ch_names, corr, eigvals, cum_explained, n_90, n_95)

    print("\nDone. All outputs written to data_analysis/results/")
