"""Recover sweep results from a SLURM stdout log when the job was killed
before the final report was written (results only persist at sweep end).

Parses 'features'-mode iteration blocks from the log and regenerates the
sweep_results.csv, ridge contribution analysis, and sweep_report.md that
sweep_spatio_temporal_transformer3.py would have produced.

Usage:
    python recover_sweep_from_log.py logs/run_10122647.out
"""

import re
import sys
from pathlib import Path
from datetime import datetime

import numpy as np
import pandas as pd
from sklearn.linear_model import RidgeCV

# Mirror the category definitions from sweep_spatio_temporal_transformer3.py
emg_feats = [
    "MAV", "RMS", "WL", "ZC", "SSC", "VAR", "WAMP", "IEMG", "LogDet",
    "Skew", "Kurt", "HjMob", "HjComp", "Myopulse", "MNF", "MDF", "Power",
    "SpecEntropy", "PeakFreq", "BW"
]
imu_feats = [
    "Mean", "Var", "Std", "Max", "SMA", "P2P", "Skew", "Kurt", "Jerk",
    "DomFreq", "SpecEnergy", "MNF", "SVM_Mean"
]
ALL_CATEGORIES = sorted([f"EMG_{f}" for f in emg_feats] + [f"IMU_{f}" for f in imu_feats])


def parse_log(log_path: Path):
    text = log_path.read_text()

    iter_pattern = re.compile(
        r"Iteration (\d+)/\d+ \[Elapsed: [\d.]+m\]\n"
        r"Active Feature Categories \((\d+)/\d+\):\n"
        r"\s+(.+?)\n",
    )
    result_pattern = re.compile(
        r"Results for Features Iteration (\d+):\n"
        r"\s+RMSE: ([\d.]+) \| MAE: ([\d.]+) \| R²: ([-\d.]+)\n"
        r"\s+Parameters: ([\d,]+) \| Training Time: ([\d.]+)s",
    )
    feat_pattern = re.compile(r"Active features: (\d+)")

    configs = {}
    for m in iter_pattern.finditer(text):
        idx = int(m.group(1))
        cats = [c.strip() for c in m.group(3).split(",")]
        unknown = set(cats) - set(ALL_CATEGORIES)
        if unknown:
            raise ValueError(f"Iteration {idx}: unknown categories {unknown}")
        configs[idx] = {"active_categories": sorted(cats), "n_categories": int(m.group(2))}

    # n_features lines appear in iteration order, same count as config blocks
    n_feature_vals = [int(m.group(1)) for m in feat_pattern.finditer(text)]
    for idx, n_feat in zip(sorted(configs), n_feature_vals):
        configs[idx]["n_features"] = n_feat

    results = []
    for m in result_pattern.finditer(text):
        idx = int(m.group(1))
        cfg = configs[idx]
        toggles = {cat: (cat in cfg["active_categories"]) for cat in ALL_CATEGORIES}
        results.append({
            "iteration": idx,
            "toggles": toggles,
            "active_categories": cfg["active_categories"],
            "n_categories": cfg["n_categories"],
            "n_features": cfg.get("n_features", float("nan")),
            "rmse": float(m.group(2)),
            "mae": float(m.group(3)),
            "r2": float(m.group(4)),
            "total_params": int(m.group(5).replace(",", "")),
            "training_time": float(m.group(6)),
        })
    return results


def main():
    log_path = Path(sys.argv[1])
    results = parse_log(log_path)
    print(f"Recovered {len(results)} completed iterations from {log_path.name}")
    if not results:
        sys.exit("No iterations found — is this a 'features'-mode log?")

    base_dir = Path(__file__).resolve().parent.parent.parent.parent
    run_dir_match = re.search(r"sweep_st_transformer3_\w+_\d+_\d+", log_path.read_text())
    run_name = run_dir_match.group(0) if run_dir_match else f"recovered_{log_path.stem}"
    run_dir = base_dir / "model" / "model_results" / f"{run_name}_RECOVERED"
    run_dir.mkdir(parents=True, exist_ok=True)

    # Ridge contribution analysis (same as the sweep script's post-processing)
    X_reg = np.array([[1.0 if r["toggles"][c] else 0.0 for c in ALL_CATEGORIES] for r in results])
    y_reg = np.array([r["r2"] for r in results])
    ridge = RidgeCV(alphas=[0.1, 1.0, 10.0, 100.0])
    ridge.fit(X_reg, y_reg)
    contribution = sorted(
        ({"category": c, "coef": coef,
          "impact": "Positive (Improves Generalization)" if coef > 0
          else "Negative (Reduces Generalization)"}
         for c, coef in zip(ALL_CATEGORIES, ridge.coef_)),
        key=lambda x: x["coef"], reverse=True,
    )

    print(f"\nRidge fit R²: {ridge.score(X_reg, y_reg):.4f} | Intercept (avg R²): {ridge.intercept_:.4f}")
    print("\nFeature Generalization Contributions (Ridge Coefficients on R²):")
    for rank, c in enumerate(contribution, 1):
        print(f"  #{rank:2d}: {c['category']:15s} -> {c['coef']:+.5f} ({c['impact']})")

    results.sort(key=lambda r: r["rmse"])

    rows = []
    for r in results:
        row = {k: r[k] for k in ["iteration", "n_categories", "n_features",
                                 "rmse", "mae", "r2", "total_params", "training_time"]}
        for cat in ALL_CATEGORIES:
            row[f"toggle_{cat}"] = 1 if r["toggles"][cat] else 0
        rows.append(row)
    csv_file = run_dir / "sweep_results.csv"
    pd.DataFrame(rows).to_csv(csv_file, index=False)

    report_file = run_dir / "sweep_report.md"
    with open(report_file, "w") as f:
        f.write("# Spatio-Temporal Transformer 3 Consolidated Sweep Report (FEATURES) — RECOVERED\n\n")
        f.write(f"**Recovered from log:** `{log_path.name}` (job hit 24h SLURM limit; "
                f"{len(results)} of 60 iterations completed)\n\n")
        f.write(f"**Date:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        f.write("## 🎯 Ridge Regression Feature Contribution Analysis\n\n")
        f.write(f"* **Ridge Regression Fit $R^2$ Score:** {ridge.score(X_reg, y_reg):.4f}\n")
        f.write(f"* **Baseline Intercept (Average $R^2$):** {ridge.intercept_:.4f}\n\n")
        f.write("| Rank | Feature Category | Ridge Coefficient (Delta R²) | Impact on Generalization |\n")
        f.write("| :---: | :--- | :---: | :--- |\n")
        for rank, c in enumerate(contribution, 1):
            emoji = "✅" if c["coef"] > 0 else "❌"
            f.write(f"| {rank} | `{c['category']}` | **{c['coef']:+.5f}** | {emoji} {c['impact']} |\n")
        f.write("\n## 🏆 Ranked Leaderboard (Sorted by RMSE)\n\n")
        f.write("| Rank | Iteration | Active Categories | Features | Val RMSE | Val MAE | Val R² | Model Params | Training Time |\n")
        f.write("| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n")
        for rank, r in enumerate(results, 1):
            cats = ", ".join(r["active_categories"])
            if len(cats) > 40:
                cats = cats[:37] + "..."
            f.write(f"| {rank} | #{r['iteration']} | {r['n_categories']} (`{cats}`) | {r['n_features']} | "
                    f"**{r['rmse']:.4f}** | {r['mae']:.4f} | {r['r2']:.4f} | "
                    f"{r['total_params']:,} | {r['training_time']:.1f}s |\n")

    print(f"\nRecovered CSV: {csv_file}")
    print(f"Recovered report: {report_file}")


if __name__ == "__main__":
    main()
