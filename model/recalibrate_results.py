"""
Post-hoc linear recalibration of a finished run's predictions.

Creates a *copy* of a run directory with `_recalibrated` suffix. For every
modality subdir that has a run_data.json, it:
  1. Fits an affine calibration  true ~= b * pred + a  (ordinary least squares)
     on the available (test/oof) predictions, then applies  pred_cal = b*pred + a.
  2. Recomputes the metrics that depend only on (y_true, y_pred): overall
     MAE/MSE/RMSE/R2/corr, per-weight MAE/RMSE + signed bias, 0kg-vs-loaded.
  3. Regenerates regression_plot.png from the calibrated predictions.
  4. Writes a recalibrated performance_report.txt (original kept as
     performance_report_ORIGINAL.txt) and updates run_data.json.
  5. Copies all other artifacts unchanged (model, DeepSHAP, permutation,
     loss/seqlen/participant plots) — these are NOT recalibrated.

IMPORTANT CAVEAT: the calibration here is fit on the SAME evaluation set it is
scored on (in-sample), because only the held-out fold predictions are stored.
The improved numbers are therefore an optimistic upper bound demonstrating that
the bias is affine/removable. For an unbiased estimate, fit the calibration on
training/validation predictions and apply to the untouched test set.

Usage:
    python model/recalibrate_results.py model/model_results/run_20260605_113834
"""
import sys, json, shutil
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def metrics(true, pred):
    err = pred - true
    mae = float(np.mean(np.abs(err)))
    mse = float(np.mean(err ** 2))
    rmse = float(np.sqrt(mse))
    ss_res = float(np.sum(err ** 2))
    ss_tot = float(np.sum((true - true.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    corr = float(np.corrcoef(true, pred)[0, 1]) if len(true) > 1 else float("nan")
    return dict(MAE=mae, MSE=mse, RMSE=rmse, R2=r2, Corr=corr)


def per_weight(true, pred):
    rows = []
    for w in sorted(set(np.round(true, 2))):
        m = np.abs(true - w) < 0.05
        e = pred[m] - w
        rows.append((float(w), int(m.sum()), float(np.mean(np.abs(e))),
                     float(np.sqrt(np.mean(e ** 2))), float(np.mean(e))))
    return rows


def make_regression_plot(true, pred, out_path, title, r2, mae):
    weights = sorted(set(np.round(true, 2)))
    data = [pred[np.abs(true - w) < 0.05] for w in weights]
    fig, ax = plt.subplots(figsize=(8, 6))
    lim = max(true.max(), pred.max()) * 1.08
    ax.plot([0, lim], [0, lim], "r--", label="Perfect Prediction", zorder=1)
    ax.boxplot(data, positions=weights, widths=0.18, showfliers=True,
               flierprops=dict(marker="o", markersize=4, markerfacecolor="#9ecae1",
                               markeredgecolor="#9ecae1", alpha=0.6),
               medianprops=dict(color="black"))
    ax.set_xlabel("Actual Weight (kg)")
    ax.set_ylabel("Predicted Weight (kg)  [recalibrated]")
    ax.set_title(title, fontweight="bold")
    ax.set_xticks(weights)
    ax.set_xticklabels([f"{w:.2f}" for w in weights])
    ax.text(0.03, 0.97, f"RECALIBRATED\nR² = {r2:.3f}\nMAE = {mae:.3f} kg",
            transform=ax.transAxes, va="top", ha="left",
            bbox=dict(boxstyle="round", fc="white", ec="gray"))
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def write_report(path, modality, a, b, before, after, pw_before, pw_after, n):
    L = []
    L.append("=" * 60)
    L.append("RECALIBRATED MODEL PERFORMANCE REPORT")
    L.append(f"Modality: {modality.upper()}")
    L.append("=" * 60)
    L.append("")
    L.append("--- RECALIBRATION ---")
    L.append("Affine post-hoc calibration:  pred_cal = b * pred + a")
    L.append(f"  slope b   : {b:.6f}")
    L.append(f"  intercept a: {a:.6f}")
    L.append(f"  samples    : {n}")
    L.append("  CAVEAT: fit IN-SAMPLE on the held-out fold predictions (no separate")
    L.append("  calibration set was stored). Numbers are an optimistic upper bound")
    L.append("  showing the bias is affine/removable; for an unbiased estimate fit")
    L.append("  the calibration on train/val predictions and apply to untouched test.")
    L.append("")
    L.append("--- OVERALL METRICS: BEFORE -> AFTER ---")
    for k in ["MAE", "RMSE", "R2", "Corr"]:
        L.append(f"  {k:5}: {before[k]:.4f}  ->  {after[k]:.4f}")
    L.append("")
    L.append("--- PER-WEIGHT METRICS (recalibrated) ---")
    L.append(f"{'Weight':>8} {'Count':>6} {'MAE_before':>11} {'MAE_after':>10} "
             f"{'RMSE_after':>11} {'bias_after':>11}")
    L.append("-" * 62)
    for (w, c, mae_b, rmse_b, bias_b), (_, _, mae_a, rmse_a, bias_a) in zip(pw_before, pw_after):
        L.append(f"{w:8.2f} {c:6d} {mae_b:11.4f} {mae_a:10.4f} {rmse_a:11.4f} {bias_a:+11.4f}")
    L.append("")
    L.append("Note: per-seqlen / per-participant / importance / timing sections are")
    L.append("unchanged from performance_report_ORIGINAL.txt (not recalibrated).")
    L.append("")
    Path(path).write_text("\n".join(L))


def process_modality(src_dir: Path, dst_dir: Path):
    rd_path = src_dir / "run_data.json"
    d = json.loads(rd_path.read_text())
    true = np.array(d["predictions"]["y_true"], dtype=float).flatten()
    pred = np.array(d["predictions"]["y_pred"], dtype=float).flatten()

    b, a = np.polyfit(pred, true, 1)          # true ~= b*pred + a
    pred_cal = b * pred + a

    before, after = metrics(true, pred), metrics(true, pred_cal)
    pw_before, pw_after = per_weight(true, pred), per_weight(true, pred_cal)

    # copy entire modality dir, then overwrite the recalibrated artifacts
    if dst_dir.exists():
        shutil.rmtree(dst_dir)
    shutil.copytree(src_dir, dst_dir)

    # keep original report, write recalibrated report
    if (dst_dir / "performance_report.txt").exists():
        shutil.move(str(dst_dir / "performance_report.txt"),
                    str(dst_dir / "performance_report_ORIGINAL.txt"))
    write_report(dst_dir / "performance_report.txt", src_dir.name, a, b,
                 before, after, pw_before, pw_after, len(true))

    # update run_data.json predictions + evaluation
    d["predictions"]["y_pred"] = pred_cal.tolist()
    d["predictions"]["y_pred_uncalibrated"] = pred.tolist()
    d["recalibration"] = {"method": "affine_ols_in_sample",
                          "formula": "pred_cal = b*pred + a",
                          "slope_b": float(b), "intercept_a": float(a),
                          "metrics_before": before, "metrics_after": after}
    if isinstance(d.get("evaluation"), dict):
        d["evaluation"]["recalibrated_metrics"] = after
    (dst_dir / "run_data.json").write_text(json.dumps(d, indent=2))

    # regenerate regression plot from calibrated preds
    make_regression_plot(true, pred_cal, dst_dir / "regression_plot.png",
                         f"{src_dir.name.upper()}: RECALIBRATED PREDICTED vs ACTUAL",
                         after["R2"], after["MAE"])

    return b, a, before, after


def main():
    if len(sys.argv) < 2:
        print("usage: python model/recalibrate_results.py <run_dir>")
        sys.exit(1)
    run_dir = Path(sys.argv[1]).resolve()
    out_dir = run_dir.parent / (run_dir.name + "_recalibrated")
    out_dir.mkdir(parents=True, exist_ok=True)

    modalities = [p for p in sorted(run_dir.iterdir())
                  if p.is_dir() and (p / "run_data.json").exists()]
    if not modalities:
        print(f"No complete modality subdirs (with run_data.json) under {run_dir}")
        sys.exit(1)

    print(f"Recalibrating {[m.name for m in modalities]}")
    print(f"Output -> {out_dir}\n")
    for m in modalities:
        b, a, before, after = process_modality(m, out_dir / m.name)
        print(f"[{m.name:9}] pred_cal = {b:.4f}*pred + {a:.4f}")
        print(f"            MAE {before['MAE']:.4f} -> {after['MAE']:.4f} | "
              f"RMSE {before['RMSE']:.4f} -> {after['RMSE']:.4f} | "
              f"R2 {before['R2']:.4f} -> {after['R2']:.4f}")
    print(f"\nDone. Recalibrated copy: {out_dir}")


if __name__ == "__main__":
    main()
