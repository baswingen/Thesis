"""
Analyse the sensor-practicality ablation grid (model/run_sensor_practicality_ablation.py).

Consumes the per-combination run_data.json files plus the cross-val2 baseline DeepSHAP /
permutation importances, and produces:

  1. ablation_results.csv      ranked performance table (overall + weighted-sample metrics)
  2. group_importance_comparison.png   Shapley(ablation) vs DeepSHAP vs Permutation per group
  3. practicality_frontier.png         weighted-MAE vs #channels, Pareto front + minimal sets
  4. practicality_cost_frontier.png    weighted-MAE vs setup-cost proxy, Pareto front
  5. combo_heatmap.png                 group-presence matrix sorted by weighted-MAE
  6. analysis_summary.txt              printed ranking / Shapley / Spearman / recommendations

Primary metric = weighted-sample MAE (`zero_vs_weight.weighted.MAE`): the 0 kg class is
trivially easy, so the load signal that discriminates sensors lives in the loaded samples.

Usage:
    python -m model.analyze_sensor_ablation --grid_dir model/model_results/<grid-run-dir>
    # baseline defaults to ST-transformer-par-spec-cross-val2 (for SHAP/perm + canonical fallback)
"""
import sys
import json
import argparse
import math
from pathlib import Path
from itertools import combinations

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
# Reuse the exact group definitions used to RUN the grid (single source of truth).
import model.run_sensor_practicality_ablation as drv

GROUP_ORDER = drv.GROUP_ORDER
GROUP_LABELS = {
    "IMUup": "IMU upper arm", "IMUfo": "IMU forearm",
    "EMGfo": "EMG forearm", "EMGup": "EMG upper arm", "EMGsh": "EMG shoulder",
}
# Setup-burden proxy (assumption, easily edited): EMG needs electrode placement + skin
# prep; IMU is a quick strap. Per-group cost units.
GROUP_COST = {"IMUup": 1, "IMUfo": 1, "EMGfo": 3, "EMGup": 3, "EMGsh": 3}
# n channels per group
GROUP_NCH = {g: len(drv.IMU_GROUPS.get(g, drv.EMG_GROUPS.get(g, []))) for g in GROUP_ORDER}

NAME_TO_GROUPS = {drv.combo_name(c): set(c) for c in drv.all_combos()}
DEFAULT_BASELINE = "model/model_results/ST-transformer-par-spec-cross-val2"


# ──────────────────────────────────────────────────────────────────────────
# Loading
# ──────────────────────────────────────────────────────────────────────────
def _get(d, *path, default=None):
    for k in path:
        if not isinstance(d, dict) or k not in d:
            return default
        d = d[k]
    return d


def _safe_pw_float(val):
    if val is None:
        return None
    try:
        f = float(val)
        if np.isnan(f) or np.isinf(f):
            return None
        return f
    except ValueError:
        return None


def load_combo(run_dir: Path, name: str):
    """Load one combo's metrics from <run_dir>/<name>/run_data.json. Returns dict or None."""
    jp = run_dir / name / "run_data.json"
    if not jp.exists():
        return None
    d = json.load(open(jp))
    ev = d.get("evaluation", {}) or {}
    macro = ev.get("macro_avg", {}) or {}
    zw = ev.get("zero_vs_weight", {}) or {}
    weighted = zw.get("weighted", {}) or {}
    zero = zw.get("zero_kg", {}) or {}
    groups = NAME_TO_GROUPS.get(name, set())

    # Calculate class-macro MAE, RMSE, and R2 from raw predictions if available
    predictions = d.get("predictions")
    class_macro_mae = None
    class_macro_rmse = None
    class_macro_r2 = None
    if predictions and isinstance(predictions, dict) and predictions.get("y_true") and predictions.get("y_pred"):
        y_t = np.array(predictions["y_true"], dtype=float).flatten()
        y_p = np.array(predictions["y_pred"], dtype=float).flatten()
        unique_weights = np.unique(y_t)
        if len(unique_weights) > 0:
            maes = []
            rmses = []
            mses = []
            for w in unique_weights:
                mask = np.abs(y_t - w) < 0.05
                if np.any(mask):
                    diff = y_p[mask] - y_t[mask]
                    maes.append(np.mean(np.abs(diff)))
                    mse = np.mean(diff ** 2)
                    mses.append(mse)
                    rmses.append(np.sqrt(mse))
            if maes:
                class_macro_mae = sum(maes) / len(maes)
            if rmses:
                class_macro_rmse = sum(rmses) / len(rmses)
            if len(unique_weights) > 1 and mses:
                mean_w = np.mean(unique_weights)
                ss_tot = np.mean((unique_weights - mean_w) ** 2)
                ss_res = np.mean(mses)
                class_macro_r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    else:
        # Fallback to per_weight array if predictions are missing
        pw = ev.get("per_weight") or []
        if pw:
            maes = [_safe_pw_float(row.get("MAE")) for row in pw]
            rmses = [_safe_pw_float(row.get("RMSE")) for row in pw]
            maes = [m for m in maes if m is not None]
            rmses = [r for r in rmses if r is not None]
            if maes:
                class_macro_mae = sum(maes) / len(maes)
            if rmses:
                class_macro_rmse = sum(rmses) / len(rmses)

    row = {
        "combo": name,
        "groups": "+".join(g for g in GROUP_ORDER if g in groups),
        "n_groups": len(groups),
        "n_channels": sum(GROUP_NCH[g] for g in groups),
        "cost": sum(GROUP_COST[g] for g in groups),
        "has_emg": any(g.startswith("EMG") for g in groups),
        "has_imu": any(g.startswith("IMU") for g in groups),
        "MAE": macro.get("MAE"), "MAE_std": macro.get("MAE_std"),
        "RMSE": macro.get("RMSE"), "R2": macro.get("R2"),
        "w_MAE": weighted.get("MAE"), "w_RMSE": weighted.get("RMSE"), "w_R2": weighted.get("R2"),
        "class_macro_MAE": class_macro_mae, "class_macro_RMSE": class_macro_rmse, "class_macro_R2": class_macro_r2,
        "zero_MAE": zero.get("MAE"),
        "n_folds": ev.get("n_folds"),
        "_y_true": _get(d, "predictions", "y_true"),
    }
    for g in GROUP_ORDER:
        row[g] = int(g in groups)
    return row


def load_grid(grid_dir: Path, baseline_dir: Path):
    """Load all 31 combos, falling back to baseline_dir for any missing canonical combo."""
    rows, missing, fellback = [], [], []
    for c in drv.all_combos():
        name = drv.combo_name(c)
        row = load_combo(grid_dir, name)
        if row is None and baseline_dir is not None:
            row = load_combo(baseline_dir, name)
            if row is not None:
                fellback.append(name)
        if row is None:
            missing.append(name)
            continue
        rows.append(row)
    return pd.DataFrame(rows), missing, fellback


# ──────────────────────────────────────────────────────────────────────────
# Shapley value over the 5 groups (ablation-based importance)
# ──────────────────────────────────────────────────────────────────────────
def get_empty_baseline(y_true, metric):
    """Empty-set baseline metric (mean predictor) on the appropriate samples."""
    y = np.asarray([v for v in y_true if v is not None], dtype=float)
    if y.size == 0:
        return None
    if metric.startswith("w_"):
        yw = y[y > 1e-6]
        if yw.size == 0:
            return None
        if "MAE" in metric:
            return float(np.mean(np.abs(yw - yw.mean())))
        elif "RMSE" in metric:
            return float(np.std(yw))
        elif "R2" in metric:
            return 0.0
    else:
        if "MAE" in metric:
            return float(np.mean(np.abs(y - y.mean())))
        elif "RMSE" in metric:
            return float(np.std(y))
        elif "R2" in metric:
            return 0.0
    return None


def shapley_groups(df, metric_col, v_empty):
    """
    Shapley value of error-REDUCTION (or R2 increase) per group, using cost
    c(S)=metric_col[S], c(empty)=v_empty. phi_g > 0 => group g improves performance.
    """
    cost = {frozenset(NAME_TO_GROUPS[r.combo]): r.__getattribute__(metric_col)
            for r in df.itertuples() if getattr(r, metric_col) is not None}

    def c(S):
        S = frozenset(S)
        if len(S) == 0:
            return v_empty
        return cost.get(S)

    n = len(GROUP_ORDER)
    phi = {}
    is_r2 = "R2" in metric_col
    for g in GROUP_ORDER:
        others = [x for x in GROUP_ORDER if x != g]
        total = 0.0
        ok = True
        for s in range(len(others) + 1):
            w = math.factorial(s) * math.factorial(n - s - 1) / math.factorial(n)
            for S in combinations(others, s):
                cs, csg = c(S), c(set(S) | {g})
                if cs is None or csg is None:
                    ok = False
                    continue
                if is_r2:
                    total += w * (csg - cs)   # R2 increase from adding g
                else:
                    total += w * (cs - csg)   # error reduction from adding g
        phi[g] = total if ok else float("nan")
    return phi


# ──────────────────────────────────────────────────────────────────────────
# Channel-level attribution -> group aggregation
# ──────────────────────────────────────────────────────────────────────────
def channel_to_group(key):
    """'Brachioradialis_EMG' / 'ax1_IMU' -> group code, or None."""
    if key.endswith("_EMG"):
        name = key[:-4]
        for g, chans in drv.EMG_GROUPS.items():
            if name in chans:
                return g
    elif key.endswith("_IMU"):
        name = key[:-4]
        for g, chans in drv.IMU_GROUPS.items():
            if name in chans:
                return g
    return None


def aggregate_attribution(baseline_dir: Path, fi_key: str):
    """Sum channel-level importances (deepshap_channel / permutation_channel) into 5 groups."""
    jp = baseline_dir / "all" / "run_data.json"
    if not jp.exists():
        return None
    fi = json.load(open(jp)).get("feature_importance", {}) or {}
    chan = fi.get(fi_key)
    if not chan:
        return None
    out = {g: 0.0 for g in GROUP_ORDER}
    for k, v in chan.items():
        g = channel_to_group(k)
        if g is not None:
            out[g] += max(float(v), 0.0)   # clip tiny negatives (permutation noise)
    return out


def normalize(d):
    s = sum(d.values())
    return {k: (v / s if s else 0.0) for k, v in d.items()}


# ──────────────────────────────────────────────────────────────────────────
# Pareto front (minimise x and y)
# ──────────────────────────────────────────────────────────────────────────
def pareto_mask(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    keep = np.ones(len(x), bool)
    for i in range(len(x)):
        for j in range(len(x)):
            if j == i:
                continue
            if x[j] <= x[i] and y[j] <= y[i] and (x[j] < x[i] or y[j] < y[i]):
                keep[i] = False
                break
    return keep


# ──────────────────────────────────────────────────────────────────────────
# Plots
# ──────────────────────────────────────────────────────────────────────────
def plot_group_importance(shap_ablation, methods, out_path):
    """methods: dict label -> {group: value} (already normalised to fractions)."""
    labels = ["Ablation (Shapley)"] + list(methods.keys())
    series = [shap_ablation] + [methods[k] for k in methods]
    x = np.arange(len(GROUP_ORDER))
    w = 0.8 / len(series)
    fig, ax = plt.subplots(figsize=(10, 5.5))
    for i, (lab, s) in enumerate(zip(labels, series)):
        ax.bar(x + i * w, [s.get(g, 0) for g in GROUP_ORDER], w, label=lab)
    ax.set_xticks(x + w * (len(series) - 1) / 2)
    ax.set_xticklabels([GROUP_LABELS[g] for g in GROUP_ORDER], rotation=20, ha="right")
    ax.set_ylabel("Normalised importance (fraction)")
    ax.set_title("Per-group importance: ablation Shapley vs attribution methods")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_frontier(df, metric_col, xcol, xlabel, out_path, full_val):
    sub = df.dropna(subset=[metric_col, xcol]).copy()
    is_r2 = "R2" in metric_col
    y_values = sub[metric_col].values
    y_for_pareto = -y_values if is_r2 else y_values
    keep = pareto_mask(sub[xcol].values, y_for_pareto)
    
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.scatter(sub[xcol], sub[metric_col], c=np.where(keep, "#d62728", "#9aa0a6"),
               s=np.where(keep, 70, 35), zorder=3)
    pf = sub[keep].sort_values(xcol)
    ax.plot(pf[xcol], pf[metric_col], "--", color="#d62728", lw=1, zorder=2)
    for _, r in pf.iterrows():
        ax.annotate(r["combo"], (r[xcol], r[metric_col]), fontsize=7,
                    xytext=(4, 4), textcoords="offset points")
    if full_val is not None:
        val_5 = full_val * 0.95 if is_r2 else full_val * 1.05
        val_10 = full_val * 0.90 if is_r2 else full_val * 1.10
        ax.axhline(full_val, color="#1f77b4", lw=1, ls=":")
        ax.axhline(val_5, color="#2ca02c", lw=0.8, ls=":")
        ax.axhline(val_10, color="#ff7f0e", lw=0.8, ls=":")
        ax.text(sub[xcol].max(), full_val, " full set", color="#1f77b4", va="bottom", fontsize=8)
        ax.text(sub[xcol].max(), val_10, " -10%" if is_r2 else " +10%", color="#ff7f0e", va="bottom", fontsize=8)
        
    ax.set_xlabel(xlabel)
    
    # Format label nicely
    if metric_col == "w_MAE":
        ylabel = "Weighted-sample MAE (kg)"
    elif metric_col == "MAE":
        ylabel = "Macro MAE (kg)"
    elif metric_col == "w_RMSE":
        ylabel = "Weighted-sample RMSE (kg)"
    elif metric_col == "RMSE":
        ylabel = "Macro RMSE (kg)"
    elif metric_col == "w_R2":
        ylabel = "Weighted R²"
    elif metric_col == "R2":
        ylabel = "Macro R²"
    elif metric_col == "class_macro_MAE":
        ylabel = "Class-macro MAE (kg)"
    elif metric_col == "class_macro_RMSE":
        ylabel = "Class-macro RMSE (kg)"
    elif metric_col == "class_macro_R2":
        ylabel = "Class-macro R²"
    else:
        ylabel = metric_col
        
    ax.set_ylabel(ylabel)
    ax.set_title(f"Practicality frontier (red = Pareto-optimal): {metric_col} vs {xlabel}")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_heatmap(df, metric_col, out_path):
    is_r2 = "R2" in metric_col
    ascending = not is_r2
    sub = df.dropna(subset=[metric_col]).sort_values(metric_col, ascending=ascending).reset_index(drop=True)
    mat = sub[GROUP_ORDER].values.astype(float)
    fig, (ax, axb) = plt.subplots(1, 2, figsize=(11, max(6, 0.28 * len(sub))),
                                  gridspec_kw={"width_ratios": [3, 1]})
    ax.imshow(mat, aspect="auto", cmap="Greens", vmin=0, vmax=1)
    ax.set_xticks(range(len(GROUP_ORDER)))
    ax.set_xticklabels([GROUP_LABELS[g] for g in GROUP_ORDER], rotation=30, ha="right")
    ax.set_yticks(range(len(sub)))
    ax.set_yticklabels(sub["combo"], fontsize=7)
    ax.set_title("Sensor groups present (sorted best→worst)")
    axb.barh(range(len(sub)), sub[metric_col], color="#d62728")
    axb.set_ylim(ax.get_ylim())
    axb.set_yticks([])
    axb.invert_yaxis()
    ax.invert_yaxis()
    
    if metric_col == "w_MAE":
        xlabel = "Weighted MAE (kg)"
    elif metric_col == "MAE":
        xlabel = "Macro MAE (kg)"
    elif metric_col == "w_RMSE":
        xlabel = "Weighted RMSE (kg)"
    elif metric_col == "RMSE":
        xlabel = "Macro RMSE (kg)"
    elif metric_col == "w_R2":
        xlabel = "Weighted R²"
    elif metric_col == "R2":
        xlabel = "Macro R²"
    elif metric_col == "class_macro_MAE":
        xlabel = "Class-macro MAE (kg)"
    elif metric_col == "class_macro_RMSE":
        xlabel = "Class-macro RMSE (kg)"
    elif metric_col == "class_macro_R2":
        xlabel = "Class-macro R²"
    else:
        xlabel = metric_col
        
    axb.set_xlabel(xlabel)
    axb.set_title("Performance")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


# ──────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid_dir", required=True, help="Dir with the 31 combo subdirs.")
    ap.add_argument("--baseline_dir", default=DEFAULT_BASELINE,
                    help="cross-val2 dir for SHAP/perm + canonical fallback.")
    ap.add_argument("--out_dir", default=None, help="Output dir (default: <grid_dir>/analysis).")
    ap.add_argument("--metric", default="class_macro_RMSE",
                    choices=["MAE", "RMSE", "R2", "w_MAE", "w_RMSE", "w_R2", "class_macro_MAE", "class_macro_RMSE", "class_macro_R2"],
                    help="Primary metric to use for ranking and Shapley values (default: class_macro_RMSE).")
    args = ap.parse_args()

    grid_dir = Path(args.grid_dir)
    baseline_dir = Path(args.baseline_dir) if args.baseline_dir else None
    out_dir = Path(args.out_dir) if args.out_dir else grid_dir / "analysis"
    out_dir.mkdir(parents=True, exist_ok=True)

    df, missing, fellback = load_grid(grid_dir, baseline_dir)
    if df.empty:
        raise SystemExit(f"No combo run_data.json found under {grid_dir}")

    metric = args.metric
    is_r2 = "R2" in metric
    ascending = not is_r2
    df = df.sort_values(metric, ascending=ascending, na_position="last").reset_index(drop=True)
    
    full_row = df[df["combo"] == "all"]
    full_val = float(full_row[metric].iloc[0]) if not full_row.empty else None

    # CSV
    csv_cols = ["combo", "groups", "n_groups", "n_channels", "cost", "has_emg", "has_imu",
                "MAE", "MAE_std", "RMSE", "R2", "w_MAE", "w_RMSE", "w_R2",
                "class_macro_MAE", "class_macro_RMSE", "class_macro_R2", "zero_MAE", "n_folds"]
    df[csv_cols].to_csv(out_dir / "ablation_results.csv", index=False)

    # Shapley (ablation importance) on selected metric
    y_true = next((r["_y_true"] for _, r in df.iterrows() if r["_y_true"]), None)
    v_empty = get_empty_baseline(y_true, metric) if y_true else None
    phi = shapley_groups(df, metric, v_empty) if v_empty is not None else {}
    phi_norm = normalize({k: max(v, 0.0) for k, v in phi.items()}) if phi else {}

    # Attribution aggregation
    methods = {}
    shap_g = aggregate_attribution(baseline_dir, "deepshap_channel") if baseline_dir else None
    perm_g = aggregate_attribution(baseline_dir, "permutation_channel") if baseline_dir else None
    if shap_g:
        methods["DeepSHAP"] = normalize(shap_g)
    if perm_g:
        methods["Permutation"] = normalize(perm_g)

    # Spearman rank correlation (ablation vs each attribution method)
    spear = {}
    try:
        from scipy.stats import spearmanr
        if phi_norm:
            for lab, m in methods.items():
                rho, p = spearmanr([phi_norm[g] for g in GROUP_ORDER],
                                   [m[g] for g in GROUP_ORDER])
                spear[lab] = (rho, p)
    except Exception as e:
        spear["_error"] = str(e)

    # Plots
    if phi_norm and methods:
        plot_group_importance(phi_norm, methods, out_dir / "group_importance_comparison.png")
    plot_frontier(df, metric, "n_channels", "number of channels", out_dir / "practicality_frontier.png", full_val)
    plot_frontier(df, metric, "cost", "setup-cost proxy (EMG=3, IMU=1 per group)",
                  out_dir / "practicality_cost_frontier.png", full_val)
    plot_heatmap(df, metric, out_dir / "combo_heatmap.png")

    # Minimal sets within 5% / 10% of full
    rec = {}
    if full_val is not None:
        for tol in (0.05, 0.10):
            if is_r2:
                cand = df[df[metric] >= full_val * (1 - tol)].dropna(subset=[metric])
            else:
                cand = df[df[metric] <= full_val * (1 + tol)].dropna(subset=[metric])
                
            if not cand.empty:
                best = cand.sort_values(["n_channels", metric], ascending=[True, not is_r2]).iloc[0]
                rec[tol] = (best["combo"], int(best["n_channels"]), float(best[metric]))

    # Text summary
    lines = []
    lines.append("=" * 125)
    lines.append("SENSOR-PRACTICALITY ABLATION — ANALYSIS SUMMARY")
    lines.append("=" * 125)
    if missing:
        lines.append(f"[WARN] missing combos (skipped): {missing}")
    if fellback:
        lines.append(f"[NOTE] pulled from baseline_dir (different fold count): {fellback}")
    lines.append("")
    lines.append(f"RANKING (sorted by {metric}, best → worst):")
    lines.append(f"  {'Combination':32s} {'Ch':2s} {'Cost':4s} {'c_mRMSE':7s} {'c_mMAE':6s} {'c_mR2':6s} {'w_MAE':6s} {'w_RMSE':6s} {'w_R2':6s} {'MAE':6s} {'RMSE':6s} {'R2':6s}")
    lines.append(f"  {'-'*32} {'--':2s} {'----':4s} {'-------':7s} {'------':6s} {'------':6s} {'------':6s} {'------':6s} {'------':6s} {'------':6s} {'------':6s} {'------':6s}")
    for _, r in df.iterrows():
        v_cm_rmse = r.get("class_macro_RMSE")
        v_cm_mae = r.get("class_macro_MAE")
        v_cm_r2 = r.get("class_macro_R2")
        v_w_mae = r.get("w_MAE")
        v_w_rmse = r.get("w_RMSE")
        v_w_r2 = r.get("w_R2")
        v_mae = r.get("MAE")
        v_rmse = r.get("RMSE")
        v_r2 = r.get("R2")
        
        s_cm_rmse = f"{v_cm_rmse:.4f}" if v_cm_rmse == v_cm_rmse and v_cm_rmse is not None else "N/A"
        s_cm_mae = f"{v_cm_mae:.4f}" if v_cm_mae == v_cm_mae and v_cm_mae is not None else "N/A"
        s_cm_r2 = f"{v_cm_r2:.4f}" if v_cm_r2 == v_cm_r2 and v_cm_r2 is not None else "N/A"
        s_w_mae = f"{v_w_mae:.4f}" if v_w_mae == v_w_mae and v_w_mae is not None else "N/A"
        s_w_rmse = f"{v_w_rmse:.4f}" if v_w_rmse == v_w_rmse and v_w_rmse is not None else "N/A"
        s_w_r2 = f"{v_w_r2:.4f}" if v_w_r2 == v_w_r2 and v_w_r2 is not None else "N/A"
        s_mae = f"{v_mae:.4f}" if v_mae == v_mae and v_mae is not None else "N/A"
        s_rmse = f"{v_rmse:.4f}" if v_rmse == v_rmse and v_rmse is not None else "N/A"
        s_r2 = f"{v_r2:.4f}" if v_r2 == v_r2 and v_r2 is not None else "N/A"
        
        lines.append(f"  {r['combo']:32s} {int(r['n_channels']):2d} {int(r['cost']):4d} "
                     f"{s_cm_rmse:7s} {s_cm_mae:6s} {s_cm_r2:6s} {s_w_mae:6s} {s_w_rmse:6s} {s_w_r2:6s} "
                     f"{s_mae:6s} {s_rmse:6s} {s_r2:6s}")
    lines.append("")
    if full_val is not None:
        lines.append(f"Full-set {metric} = {full_val:.4f}")
    if phi:
        lines.append(f"\nGROUP IMPORTANCE — Shapley value of {metric} reduction (higher=more useful):" if not is_r2 else
                     f"\nGROUP IMPORTANCE — Shapley value of {metric} increase (higher=more useful):")
        for g in sorted(phi, key=lambda k: -(phi[k] if phi[k] == phi[k] else -9)):
            lines.append(f"  {GROUP_LABELS[g]:16s} φ = {phi[g]:+.4f}")
    if spear:
        lines.append("\nABLATION vs ATTRIBUTION (Spearman ρ over the 5 groups):")
        for lab, val in spear.items():
            if lab == "_error":
                lines.append(f"  [scipy unavailable: {val}]")
            else:
                lines.append(f"  Ablation-Shapley vs {lab:12s}: ρ={val[0]:+.3f} (p={val[1]:.3f})")
    if rec:
        lines.append("\nMINIMAL PRACTICAL SETS:")
        for tol, (name, nch, val) in rec.items():
            lines.append(f"  within {int(tol*100)}% of full: {name} ({nch} ch, {metric}={val:.4f})")
    lines.append("\nReminder: this grid is pooled k-fold (within-subject CALIBRATION). It is an "
                 "upper bound on each sensor's usable info. Run Phase-2 LOPO on the top sets to "
                 "test cross-subject transfer before drawing deployment conclusions.")
    summary = "\n".join(lines)
    (out_dir / "analysis_summary.txt").write_text(summary)
    print(summary)
    print(f"\n[done] artifacts written to {out_dir}")


if __name__ == "__main__":
    main()
