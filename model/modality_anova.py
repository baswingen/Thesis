"""Cross-modality statistical comparison (SF vs EMG vs IMU).

The per-modality runs (``all``/``emg_only``/``imu_only``) each store a
per-participant, class-macro MAE/RMSE under ``evaluation.per_participant`` (every
weight class counts equally within a participant; see :mod:`model.macro_metrics`).
This module treats those per-participant values as the repeated-measures
observations and asks whether the modalities differ.

For each metric (MAE, RMSE) it computes a one-way repeated-measures ANOVA across
the modalities (within-subjects factor = modality, N = shared participants) and
Bonferroni-corrected paired-t post-hoc comparisons for every modality pair. A
human-readable "pairwise relations" string is derived by ordering modalities by
mean and joining neighbours with ``<`` (significant, p_adj < 0.05) or ``~``
(not significant).

Only scipy is required (no statsmodels/pingouin): the RM-ANOVA is the textbook
partition of the total sum of squares into condition, subject and error terms.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy import stats

# Standard 3-modality ablation -> display labels used in the thesis table.
DISPLAY_LABELS = {"all": "SF", "emg_only": "EMG", "imu_only": "IMU"}
DEFAULT_METRICS = ("MAE", "RMSE")
ALPHA = 0.05


def _participant_values(per_participant, metric):
    """Map a per_participant list to {participant_id: metric_value}."""
    out = {}
    for row in per_participant or []:
        pid = row.get("Participant")
        val = row.get(metric)
        if pid is None or val is None:
            continue
        try:
            out[str(pid)] = float(val)
        except (TypeError, ValueError):
            continue
    return out


def _rm_anova(M):
    """One-way repeated-measures ANOVA on an (n_subjects x k_conditions) matrix."""
    n, k = M.shape
    grand = M.mean()
    cond_means = M.mean(axis=0)
    subj_means = M.mean(axis=1)
    ss_cond = n * np.sum((cond_means - grand) ** 2)
    ss_subj = k * np.sum((subj_means - grand) ** 2)
    ss_err = np.sum((M - grand) ** 2) - ss_cond - ss_subj
    df_cond = k - 1
    df_err = (k - 1) * (n - 1)
    ms_err = ss_err / df_err if df_err > 0 else float("nan")
    F = (ss_cond / df_cond) / ms_err if (df_cond > 0 and ms_err > 0) else float("nan")
    p = float(stats.f.sf(F, df_cond, df_err)) if np.isfinite(F) else float("nan")
    return {
        "F": float(F),
        "df_between": int(df_cond),
        "df_error": int(df_err),
        "p": p,
    }


def _posthoc(M, labels):
    """Bonferroni-corrected paired t-tests for every condition pair."""
    k = M.shape[1]
    n_pairs = k * (k - 1) // 2
    out = []
    for i in range(k):
        for j in range(i + 1, k):
            t, p = stats.ttest_rel(M[:, i], M[:, j])
            out.append({
                "a": labels[i],
                "b": labels[j],
                "mean_diff": float(M[:, i].mean() - M[:, j].mean()),
                "t": float(t),
                "p_unadj": float(p),
                "p_adj": float(min(p * n_pairs, 1.0)),
            })
    return out


def _relations(labels, means, posthoc):
    """Order modalities by mean and join neighbours with '<' or '~'."""
    p_adj = {}
    for ph in posthoc:
        p_adj[frozenset((ph["a"], ph["b"]))] = ph["p_adj"]
    order = sorted(range(len(labels)), key=lambda i: means[i])
    parts = [labels[order[0]]]
    for prev, cur in zip(order, order[1:]):
        sig = p_adj.get(frozenset((labels[prev], labels[cur])), 1.0) < ALPHA
        parts.append("<" if sig else "~")
        parts.append(labels[cur])
    return " ".join(parts)


def compute_modality_comparison(modality_to_pp, metrics=DEFAULT_METRICS,
                                labels=None):
    """Compute the cross-modality RM-ANOVA + post-hoc for each metric.

    ``modality_to_pp`` maps a modality key (e.g. ``"all"``) to its
    ``per_participant`` list. Returns ``None`` if fewer than two modalities or
    fewer than three shared participants are available.
    """
    labels = labels or DISPLAY_LABELS
    mods = [m for m in modality_to_pp if modality_to_pp.get(m)]
    if len(mods) < 2:
        return None

    result = {
        "description": (
            "Repeated-measures ANOVA across modalities on per-participant "
            "class-macro metrics (each weight class weighted equally within a "
            "participant). Post-hoc: Bonferroni-corrected paired t-tests. "
            "Pairwise relations use '<' for p_adj < %.2f, '~' otherwise." % ALPHA
        ),
        "alpha": ALPHA,
        "modalities": {m: labels.get(m, m) for m in mods},
        "metrics": {},
    }

    for metric in metrics:
        per_mod = {m: _participant_values(modality_to_pp[m], metric) for m in mods}
        shared = sorted(set.intersection(*[set(v) for v in per_mod.values()]))
        if len(shared) < 3:
            continue
        disp = [labels.get(m, m) for m in mods]
        M = np.array([[per_mod[m][p] for m in mods] for p in shared], dtype=float)
        anova = _rm_anova(M)
        posthoc = _posthoc(M, disp)
        means = M.mean(axis=0)
        result["metrics"][metric] = {
            "n_participants": len(shared),
            "means": {disp[i]: float(means[i]) for i in range(len(mods))},
            "anova": anova,
            "posthoc": posthoc,
            "relations": _relations(disp, list(means), posthoc),
        }

    return result if result["metrics"] else None


def _fmt_p(p):
    if not np.isfinite(p):
        return "nan"
    if p < 0.001:
        return f"{p:.2e}"
    return f"{p:.4f}"


def format_modality_comparison_report(result):
    """Render the comparison dict as a plain-text report block."""
    if not result:
        return ""
    lines = [
        "=======================================================",
        "CROSS-MODALITY COMPARISON (RM-ANOVA)",
        "=======================================================",
        "Per-participant class-macro metrics; within-subjects factor = modality.",
        "Post-hoc: Bonferroni-corrected paired t-tests.",
        "",
    ]
    for metric, m in result["metrics"].items():
        a = m["anova"]
        means = m["means"]
        lines.append(f"--- {metric} ---")
        lines.append("  Means: " + "  ".join(f"{k}={v:.4f}" for k, v in means.items()))
        lines.append(
            f"  RM-ANOVA: F({a['df_between']},{a['df_error']})={a['F']:.2f}  "
            f"p={_fmt_p(a['p'])}   (N={m['n_participants']})"
        )
        for ph in m["posthoc"]:
            lines.append(f"  {ph['a']}--{ph['b']}: p_adj={_fmt_p(ph['p_adj'])}")
        lines.append(f"  Pairwise relations: {m['relations']}")
        lines.append("")
    return "\n".join(lines)


def comparison_from_master_data(master_data, metrics=DEFAULT_METRICS):
    """Build the comparison from a consolidated master run_data dict.

    ``master_data`` maps each modality key to its full run_data dict (the nested
    structure written by the master serializer in run_model).
    """
    modality_to_pp = {}
    for mod, data in master_data.items():
        if not isinstance(data, dict):
            continue
        pp = (data.get("evaluation") or {}).get("per_participant")
        if pp:
            modality_to_pp[mod] = pp
    return compute_modality_comparison(modality_to_pp, metrics=metrics)


def backfill_run_dir(run_dir, write=True):
    """Compute the comparison for an existing run dir and (optionally) persist it.

    Writes the comparison into the top-level run_data.json under
    ``modality_comparison`` and a ``modality_comparison_report.txt`` alongside it.
    Returns the comparison dict (or None).
    """
    run_dir = Path(run_dir)
    master_path = run_dir / "run_data.json"
    if not master_path.exists():
        raise FileNotFoundError(f"No consolidated run_data.json in {run_dir}")
    with open(master_path) as f:
        master_data = json.load(f)

    result = comparison_from_master_data(master_data)
    if result is None:
        print(f"[modality_anova] Not enough modalities/participants in {run_dir}; skipped.")
        return None

    if write:
        master_data["modality_comparison"] = result
        with open(master_path, "w") as f:
            json.dump(master_data, f, indent=2)
        report = format_modality_comparison_report(result)
        with open(run_dir / "modality_comparison_report.txt", "w") as f:
            f.write(report + "\n")
        print(f"[modality_anova] Wrote modality_comparison to {master_path}")
        print(f"[modality_anova] Wrote modality_comparison_report.txt in {run_dir}")
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Cross-modality RM-ANOVA for a run directory.")
    ap.add_argument("run_dirs", nargs="+", help="Run directories containing a consolidated run_data.json.")
    ap.add_argument("--dry-run", action="store_true", help="Print the report without writing files.")
    args = ap.parse_args()
    for rd in args.run_dirs:
        res = backfill_run_dir(rd, write=not args.dry_run)
        if res:
            print(format_modality_comparison_report(res))
