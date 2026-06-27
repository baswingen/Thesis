"""
Backfill the participant-class-macro (PCM) headline metric into existing runs.

participant-class-macro (``class_participant_macro`` in model/macro_metrics.py) is
the study-wide PRIMARY metric: every (participant, weight) cell weighted equally.
Newer runs already store it under ``evaluation.macro_metrics``; older runs (e.g. the
architecture-comparison grid and the sensor-group ablation grids) carry the raw
predictions and participant labels but not the computed block.

This utility recomputes the full macro-metrics block FROM THE STORED PREDICTIONS only
(no dataset, no retraining) and injects it where missing. It is additive: it never
overwrites existing fields unless --force is given, and it only touches nodes that
carry per-sample participant labels (without them PCM is undefined).

Usage:
    python -m model.backfill_pcm_metrics --dirs model/model_results/arch_comp \
        model/model_results/arch_comp2 model/model_results/run_modality_ablation_spec \
        model/model_results/run_modality_ablation_gen
    python -m model.backfill_pcm_metrics --all          # every */run_data.json under model_results
    python -m model.backfill_pcm_metrics --all --dry_run # report only, write nothing
"""
import sys
import json
import glob
import argparse
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from model.macro_metrics import compute_all_macro_metrics, class_macro


class _NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.floating):
            return None if (np.isnan(obj) or np.isinf(obj)) else float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return super().default(obj)


def _process_node(node, force=False):
    """Inject macro_metrics (PCM) into one run node. Returns the PCM MAE or None."""
    if not isinstance(node, dict):
        return None
    preds = node.get("predictions")
    if not isinstance(preds, dict):
        return None
    y_true = preds.get("y_true")
    y_pred = preds.get("y_pred")
    parts = preds.get("participant")
    if not y_true or not y_pred or not parts:
        return None
    y_t = np.asarray(y_true, dtype=float)
    y_p = np.asarray(y_pred, dtype=float)
    pa = np.asarray(parts)
    if not (len(y_t) == len(y_p) == len(pa)) or len(y_t) == 0:
        return None

    ev = node.setdefault("evaluation", {})
    if not isinstance(ev, dict):
        ev = node["evaluation"] = {}

    existing = (ev.get("macro_metrics") or {}).get("class_participant_macro")
    if existing and existing.get("MAE") is not None and not force:
        return existing.get("MAE")

    macro = compute_all_macro_metrics(y_t, y_p, participants=pa)
    ev["macro_metrics"] = macro
    cm = class_macro(y_t, y_p)
    ev.setdefault("class_macro_avg", {"MAE": cm["MAE"], "RMSE": cm["RMSE"], "R2": cm["R2"]})
    return macro["class_participant_macro"]["MAE"]


def _iter_nodes(d):
    """Yield run nodes: the top-level dict and any dict child that holds predictions."""
    if isinstance(d, dict):
        if "predictions" in d:
            yield d
        for v in d.values():
            if isinstance(v, dict) and "predictions" in v:
                yield v


def process_file(fp: Path, force=False, dry_run=False):
    try:
        d = json.loads(fp.read_text())
    except Exception as e:
        print(f"  [skip] {fp}: {e}")
        return 0
    changed = []
    for node in _iter_nodes(d):
        mae = _process_node(node, force=force)
        if mae is not None:
            changed.append(mae)
    if changed and not dry_run:
        fp.write_text(json.dumps(d, indent=2, cls=_NumpyEncoder))
    if changed:
        tag = "[dry]" if dry_run else "[ok ]"
        rel = str(fp).replace(str(Path.cwd()) + "/", "")
        print(f"  {tag} {rel}  PCM-MAE={', '.join(f'{m:.4f}' for m in changed)}")
    return len(changed)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dirs", nargs="*", default=[], help="Run dirs to scan recursively for run_data.json.")
    ap.add_argument("--all", action="store_true", help="Scan every run_data.json under model/model_results.")
    ap.add_argument("--force", action="store_true", help="Recompute even if a PCM block already exists.")
    ap.add_argument("--dry_run", action="store_true", help="Report only; write nothing.")
    args = ap.parse_args()

    files = []
    if args.all:
        files += glob.glob("model/model_results/**/run_data.json", recursive=True)
    for d in args.dirs:
        files += glob.glob(str(Path(d) / "**" / "run_data.json"), recursive=True)
    files = sorted(set(files))
    if not files:
        raise SystemExit("No run_data.json found. Pass --dirs or --all.")

    total = 0
    for f in files:
        total += process_file(Path(f), force=args.force, dry_run=args.dry_run)
    print(f"\n[done] injected/updated PCM in {total} node(s) across {len(files)} file(s).")


if __name__ == "__main__":
    main()
