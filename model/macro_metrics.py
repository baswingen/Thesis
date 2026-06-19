"""Macro-averaged regression metrics for load estimation.

The dataset is heavily imbalanced: the 0 kg (free-move) class holds ~58% of all
segments, and participants contribute very different sample counts. Pooled metrics
(MAE/RMSE/R2 over all predictions) are therefore dominated by the easy 0 kg class
and by high-volume participants. This module provides three balanced views:

  * ``class_macro``            -- every weight class counts equally
  * ``participant_macro``      -- every participant counts equally
  * ``class_participant_macro``-- every (participant, weight) cell counts equally
                                  (doubly balanced; the PRIMARY headline metric)

MAE and RMSE average cleanly under any of these schemes. R2 is special: per a
single weight class the target variance is zero, so a per-class R2 is undefined.
Wherever a scheme balances across classes we therefore compute R2 with
class-balanced sample weights (each class contributes equally to both the
residual and total sum of squares) rather than averaging undefined per-class
values. Participant-macro R2 averages the ordinary within-participant R2, which
is well defined because each participant spans every load level.

Predictions are clipped at 0 (``max(0, y_pred)``) to match the convention used
elsewhere in the pipeline (negative load is non-physical).
"""

from __future__ import annotations

import numpy as np
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score


def _clip(y_pred):
    return np.maximum(0.0, np.asarray(y_pred, dtype=float))


def _class_balanced_weights(y_true):
    """Sample weights that make every distinct class contribute equally.

    Each sample gets weight 1 / n_samples_in_its_class, rescaled so the weights
    sum to len(y) (keeps R2 on a familiar scale).
    """
    y = np.asarray(y_true, dtype=float)
    w = np.ones(len(y), dtype=float)
    for c in np.unique(y):
        m = y == c
        n = int(m.sum())
        if n > 0:
            w[m] = 1.0 / n
    s = w.sum()
    if s > 0:
        w *= len(y) / s
    return w


def _class_balanced_r2(y_true, y_pred):
    """R2 with class-balanced sample weights. NaN if it is undefined."""
    y_t = np.asarray(y_true, dtype=float)
    y_p = _clip(y_pred)
    if len(y_t) < 2 or len(np.unique(y_t)) < 2:
        return float("nan")
    w = _class_balanced_weights(y_t)
    # Weighted variance of the target must be non-zero for R2 to be defined.
    wmean = np.average(y_t, weights=w)
    if np.average((y_t - wmean) ** 2, weights=w) <= 0:
        return float("nan")
    return float(r2_score(y_t, y_p, sample_weight=w))


def class_macro(y_true, y_pred):
    """Metrics with every weight class weighted equally."""
    y_t = np.asarray(y_true, dtype=float)
    y_p = _clip(y_pred)
    classes = np.unique(y_t)
    maes, rmses = [], []
    for c in classes:
        m = y_t == c
        maes.append(mean_absolute_error(y_t[m], y_p[m]))
        rmses.append(np.sqrt(mean_squared_error(y_t[m], y_p[m])))
    return {
        "MAE": float(np.mean(maes)) if maes else float("nan"),
        "RMSE": float(np.mean(rmses)) if rmses else float("nan"),
        "R2": _class_balanced_r2(y_t, y_p),
        "n_classes": int(len(classes)),
    }


def participant_macro(y_true, y_pred, participants):
    """Metrics with every participant weighted equally."""
    y_t = np.asarray(y_true, dtype=float)
    y_p = _clip(y_pred)
    parts = np.asarray(participants)
    maes, rmses, r2s = [], [], []
    for p in np.unique(parts):
        m = parts == p
        maes.append(mean_absolute_error(y_t[m], y_p[m]))
        rmses.append(np.sqrt(mean_squared_error(y_t[m], y_p[m])))
        if int(m.sum()) > 1 and np.var(y_t[m]) > 0:
            r2s.append(r2_score(y_t[m], y_p[m]))
    return {
        "MAE": float(np.mean(maes)) if maes else float("nan"),
        "RMSE": float(np.mean(rmses)) if rmses else float("nan"),
        "R2": float(np.mean(r2s)) if r2s else float("nan"),
        "n_participants": int(len(np.unique(parts))),
    }


def class_participant_macro(y_true, y_pred, participants):
    """Doubly-balanced metrics: every (participant, weight) cell weighted equally.

    For each participant the class-macro metric is computed (equal weight per
    weight class), then averaged across participants (equal weight per
    participant). This is the PRIMARY headline metric.
    """
    y_t = np.asarray(y_true, dtype=float)
    y_p = _clip(y_pred)
    parts = np.asarray(participants)
    maes, rmses, r2s = [], [], []
    for p in np.unique(parts):
        m = parts == p
        cm = class_macro(y_t[m], y_p[m])
        maes.append(cm["MAE"])
        rmses.append(cm["RMSE"])
        if not np.isnan(cm["R2"]):
            r2s.append(cm["R2"])
    return {
        "MAE": float(np.mean(maes)) if maes else float("nan"),
        "RMSE": float(np.mean(rmses)) if rmses else float("nan"),
        "R2": float(np.mean(r2s)) if r2s else float("nan"),
        "n_participants": int(len(np.unique(parts))),
    }


def compute_all_macro_metrics(y_true, y_pred, participants=None):
    """Return all three macro views plus the pooled baseline.

    ``class_participant_macro`` is flagged as the primary metric. Participant-aware
    views are omitted (None) when ``participants`` is not available (e.g. a single
    train/test split where labels are not aligned with predictions).
    """
    y_t = np.asarray(y_true, dtype=float)
    y_p = _clip(y_pred)

    pooled = {
        "MAE": float(mean_absolute_error(y_t, y_p)),
        "RMSE": float(np.sqrt(mean_squared_error(y_t, y_p))),
        "R2": float(r2_score(y_t, y_p)) if (len(y_t) > 1 and np.var(y_t) > 0) else float("nan"),
    }

    out = {
        "primary": "class_participant_macro",
        "description": (
            "Balanced regression metrics. class_participant_macro (primary) weights "
            "every (participant, weight) cell equally, removing both the 0 kg majority "
            "bias and per-participant sample-count bias. R2 under class balancing uses "
            "class-balanced sample weights; participant_macro R2 averages within-"
            "participant R2."
        ),
        "pooled": pooled,
        "class_macro": class_macro(y_t, y_p),
    }
    if participants is not None and len(np.asarray(participants)) == len(y_t):
        out["participant_macro"] = participant_macro(y_t, y_p, participants)
        out["class_participant_macro"] = class_participant_macro(y_t, y_p, participants)
    else:
        out["participant_macro"] = None
        out["class_participant_macro"] = None
    return out


def format_macro_report(macro: dict) -> str:
    """Render a macro-metrics dict as a plain-text report block."""
    if not macro:
        return ""
    lines = ["--- MACRO-AVERAGED METRICS (BALANCED) ---"]
    lines.append("Primary metric: class_participant_macro "
                 "(equal weight per participant AND per weight class)")
    lines.append("")
    header = f"{'View':<28} | {'MAE':<8} | {'RMSE':<8} | {'R2':<8}"
    lines.append(header)
    lines.append("-" * len(header))

    order = [
        ("class_participant_macro", "class+participant macro *"),
        ("participant_macro", "participant macro"),
        ("class_macro", "class macro"),
        ("pooled", "pooled (unbalanced)"),
    ]
    for key, label in order:
        m = macro.get(key)
        if not m:
            continue
        mae = m.get("MAE", float("nan"))
        rmse = m.get("RMSE", float("nan"))
        r2 = m.get("R2", float("nan"))
        lines.append(f"{label:<28} | {mae:<8.4f} | {rmse:<8.4f} | {r2:<8.4f}")
    lines.append("")
    lines.append("* PRIMARY. R2 under class balancing uses class-balanced sample weights")
    lines.append("  (per-class R2 is undefined; a single class has zero target variance).")
    return "\n".join(lines)
