"""
Make the CALIBRATED predictions the main version of a canonical run dir, keeping
the raw version under an `uncalibrated/` subfolder.

For a canonical par-spec dir with structure:
    <dir>/run_data.json            (consolidated {all,emg_only,imu_only})
    <dir>/{all,emg_only,imu_only}/ (per-modality run_data.json + artifacts)

it:
  1. Copies the current (raw) all/emg_only/imu_only/ + run_data.json into <dir>/uncalibrated/.
  2. Fits an affine calibration  true ~= b*pred + a  (OLS) per modality.
  3. Overwrites the TOP-LEVEL per-modality jsons AND the consolidated root json so that
     predictions are calibrated and the evaluation block (pooled, macro_avg, per_fold,
     per_weight, per_participant, zero_vs_weight) is RECOMPUTED from calibrated preds.
     Stores y_pred_uncalibrated and a `recalibration` block for the calibration plot.
  4. Regenerates each modality's regression_plot.png and writes a recalibrated report.

Sections that need per-sample metadata not stored in predictions (per_seqlen,
per_duration, anova) are left unchanged and flagged in the report.

CAVEAT: calibration is fit IN-SAMPLE on the held-out fold predictions (optimistic).

Usage:
    python model/calibrate_canonical.py model/model_results/ST-transformer-par-spec-P01-st3
"""
import sys, json, shutil
from copy import deepcopy
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

MODALITIES = ["all", "emg_only", "imu_only"]


def basic_metrics(true, pred):
    err = pred - true
    mse = float(np.mean(err ** 2))
    ss_res = float(np.sum(err ** 2))
    ss_tot = float(np.sum((true - true.mean()) ** 2))
    return dict(MAE=float(np.mean(np.abs(err))), MSE=mse, RMSE=float(np.sqrt(mse)),
                R2=(1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")),
                Corr=(float(np.corrcoef(true, pred)[0, 1]) if len(true) > 1 else float("nan")))


def recompute_eval(ev_raw, true, pred):
    ev = deepcopy(ev_raw)
    m = basic_metrics(true, pred)
    if isinstance(ev.get("pooled"), dict):
        ev["pooled"].update(MAE=m["MAE"], RMSE=m["RMSE"], R2=m["R2"], correlation=m["Corr"])
    if isinstance(ev.get("macro_avg"), dict):
        ma = ev["macro_avg"]
        ma.update(MAE=m["MAE"], MSE=m["MSE"], RMSE=m["RMSE"], R2=m["R2"], Correlation=m["Corr"])
        if "Train MAE" in ma:
            ma["Generalization Gap (MAE)"] = m["MAE"] - ma["Train MAE"]
        if ma.get("Train RMSE"):
            ma["Overfit Ratio (RMSE)"] = m["RMSE"] / ma["Train RMSE"]
    if ev.get("per_fold"):
        f = ev["per_fold"][0]
        f.update(MAE=m["MAE"], MSE=m["MSE"], RMSE=m["RMSE"], R2=m["R2"], Correlation=m["Corr"])
        if "Train MAE" in f:
            f["Generalization Gap (MAE)"] = m["MAE"] - f["Train MAE"]
        if f.get("Train RMSE"):
            f["Overfit Ratio (RMSE)"] = m["RMSE"] / f["Train RMSE"]
    if ev.get("per_weight"):
        for row in ev["per_weight"]:
            w = float(str(row["Weight"]).replace("kg", "").strip())
            e = pred[np.abs(true - w) < 0.05] - w
            row["MAE"] = f"{np.mean(np.abs(e)):.4f}"
            row["RMSE"] = f"{np.sqrt(np.mean(e ** 2)):.4f}"
    if ev.get("per_participant"):
        for row in ev["per_participant"]:
            row["MAE"] = m["MAE"]; row["RMSE"] = m["RMSE"]
    if isinstance(ev.get("zero_vs_weight"), dict):
        zw = ev["zero_vs_weight"]
        for key, mask in [("zero_kg", np.abs(true) < 0.05), ("weighted", true > 0.05)]:
            if isinstance(zw.get(key), dict) and mask.sum() > 0:
                e = pred[mask] - true[mask]
                d = zw[key]
                if "MAE" in d: d["MAE"] = float(np.mean(np.abs(e)))
                if "RMSE" in d: d["RMSE"] = float(np.sqrt(np.mean(e ** 2)))
                if "R2" in d and mask.sum() > 1:
                    ss = float(np.sum(e ** 2)); st = float(np.sum((true[mask] - true[mask].mean()) ** 2))
                    d["R2"] = (1.0 - ss / st) if st > 0 else d.get("R2")
    return ev


def calibrate_node(node, b, a):
    """In-place: calibrate predictions + recompute evaluation of one run dict."""
    true = np.array(node["predictions"]["y_true"], dtype=float).flatten()
    praw = np.array(node["predictions"]["y_pred"], dtype=float).flatten()
    pcal = b * praw + a
    before, after = basic_metrics(true, praw), basic_metrics(true, pcal)
    node["predictions"]["y_pred"] = pcal.tolist()
    node["predictions"]["y_pred_uncalibrated"] = praw.tolist()
    node["evaluation"] = recompute_eval(node["evaluation"], true, pcal)
    node["recalibration"] = {"method": "affine_ols_in_sample", "formula": "pred_cal = b*pred + a",
                             "slope_b": float(b), "intercept_a": float(a),
                             "metrics_before": before, "metrics_after": after}
    return before, after


def regression_plot(true, pred, out_path, title, r2, mae):
    weights = sorted(set(np.round(true, 2)))
    data = [pred[np.abs(true - w) < 0.05] for w in weights]
    fig, ax = plt.subplots(figsize=(8, 6))
    lim = max(true.max(), pred.max()) * 1.08
    ax.plot([0, lim], [0, lim], "r--", label="Perfect Prediction")
    ax.boxplot(data, positions=weights, widths=0.18, showfliers=True,
               flierprops=dict(marker="o", markersize=4, markerfacecolor="#9ecae1",
                               markeredgecolor="#9ecae1", alpha=0.6),
               medianprops=dict(color="black"))
    ax.set_xlabel("Actual Weight (kg)"); ax.set_ylabel("Predicted Weight (kg) [calibrated]")
    ax.set_title(title, fontweight="bold")
    ax.set_xticks(weights); ax.set_xticklabels([f"{w:.2f}" for w in weights])
    ax.text(0.03, 0.97, f"CALIBRATED\nR² = {r2:.3f}\nMAE = {mae:.3f} kg", transform=ax.transAxes,
            va="top", bbox=dict(boxstyle="round", fc="white", ec="gray"))
    ax.legend(loc="lower right"); fig.tight_layout(); fig.savefig(out_path, dpi=120); plt.close(fig)


def main():
    if len(sys.argv) < 2:
        print("usage: python model/calibrate_canonical.py <canonical_run_dir>"); sys.exit(1)
    canon = Path(sys.argv[1]).resolve()
    unc = canon / "uncalibrated"
    if unc.exists():
        print(f"[abort] {unc} already exists — remove it first to re-run."); sys.exit(1)

    # 1. preserve raw under uncalibrated/
    unc.mkdir(parents=True)
    for item in MODALITIES + ["run_data.json"]:
        src = canon / item
        if src.exists():
            (shutil.copytree if src.is_dir() else shutil.copy2)(src, unc / item)
    print(f"[ok] raw preserved -> {unc}")

    # 2. fit calibration per modality (from raw subdir predictions)
    coeffs = {}
    for mod in MODALITIES:
        raw = json.loads((unc / mod / "run_data.json").read_text())
        t = np.array(raw["predictions"]["y_true"], float).flatten()
        p = np.array(raw["predictions"]["y_pred"], float).flatten()
        b, a = np.polyfit(p, t, 1)
        coeffs[mod] = (float(b), float(a))

    # 3a. overwrite top-level per-modality jsons + regenerate plot/report
    for mod in MODALITIES:
        jpath = canon / mod / "run_data.json"
        node = json.loads(jpath.read_text())
        b, a = coeffs[mod]
        before, after = calibrate_node(node, b, a)
        jpath.write_text(json.dumps(node, indent=2))
        true = np.array(node["predictions"]["y_true"], float).flatten()
        pcal = np.array(node["predictions"]["y_pred"], float).flatten()
        regression_plot(true, pcal, canon / mod / "regression_plot.png",
                        f"{mod.upper()}: CALIBRATED PREDICTED vs ACTUAL", after["R2"], after["MAE"])
        (canon / mod / "performance_report_CALIBRATED.txt").write_text(
            f"CALIBRATED ({mod})  pred_cal = {b:.6f}*pred + {a:.6f}  (in-sample OLS)\n"
            f"MAE {before['MAE']:.4f} -> {after['MAE']:.4f} | RMSE {before['RMSE']:.4f} -> {after['RMSE']:.4f} | "
            f"R2 {before['R2']:.4f} -> {after['R2']:.4f}\n"
            f"NOTE: per_seqlen/per_duration/anova in run_data.json are NOT recalibrated "
            f"(no per-sample metadata stored).\n")
        print(f"[{mod:9}] pred_cal = {b:.4f}*pred + {a:.4f} | "
              f"MAE {before['MAE']:.4f}->{after['MAE']:.4f} | R2 {before['R2']:.4f}->{after['R2']:.4f}")

    # 3b. overwrite consolidated root json (nested modalities)
    rpath = canon / "run_data.json"
    root = json.loads(rpath.read_text())
    for mod in MODALITIES:
        if mod in root and isinstance(root[mod], dict) and "predictions" in root[mod]:
            calibrate_node(root[mod], *coeffs[mod])
    rpath.write_text(json.dumps(root, indent=2))
    print(f"[ok] consolidated root json calibrated -> {rpath}")
    print(f"\nDone. Calibrated = main; raw kept at {unc}")


if __name__ == "__main__":
    main()
