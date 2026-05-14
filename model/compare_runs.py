"""
compare_runs.py
===============
CLI tool to load and compare multiple model runs from their ``run_data.json``
files.  Provides the infrastructure for cross-run comparison; individual plot
functions can be added incrementally.

Usage
-----
    # Compare specific runs
    python -m model.compare_runs \\
        model/model_results/run_20260512_174758 \\
        model/model_results/run_20260513_092906

    # Compare all runs matching a glob
    python -m model.compare_runs --glob "model/model_results/run_2026051*"

    # Specify labels for the legend
    python -m model.compare_runs --labels "Transformer2" "Transformer3" \\
        model/model_results/run_20260512_174758 \\
        model/model_results/run_20260513_092906
"""

import json
import argparse
import numpy as np
from pathlib import Path
from typing import Optional


# ---------------------------------------------------------------------------
# Data Loading
# ---------------------------------------------------------------------------

def load_run(run_dir: str | Path) -> Optional[dict]:
    """Load a single ``run_data.json`` from a run directory."""
    p = Path(run_dir) / "run_data.json"
    if not p.exists():
        print(f"[WARN] No run_data.json found in {run_dir}, skipping.")
        return None
    with open(p, "r") as f:
        return json.load(f)


def load_runs(run_dirs: list[str | Path]) -> list[dict]:
    """Load ``run_data.json`` from each directory, skipping missing ones."""
    runs = []
    for d in run_dirs:
        data = load_run(d)
        if data is not None:
            runs.append(data)
    return runs


def get_label(run: dict, custom_label: Optional[str] = None) -> str:
    """Generate a short label for a run.
    
    Priority: custom_label > model_type (date)
    """
    if custom_label:
        return custom_label
    meta = run.get('meta', {})
    model_type = meta.get('model_type', 'unknown')
    timestamp = meta.get('timestamp', '')[:8]  # YYYYMMDD
    return f"{model_type} ({timestamp})"


def summarise_runs(runs: list[dict], labels: Optional[list[str]] = None):
    """Print a quick tabular summary of loaded runs."""
    print(f"\n{'=' * 80}")
    print(f"{'Label':<35} | {'Model':<30} | {'Params':>10} | {'MAE':>8} | {'R²':>8}")
    print(f"{'-' * 80}")
    for i, run in enumerate(runs):
        label = get_label(run, labels[i] if labels and i < len(labels) else None)
        model_type = run.get('meta', {}).get('model_type', '?')
        params = run.get('model_info', {}).get('trainable_parameters', '?')
        
        # Prefer macro-average, fall back to pooled
        ev = run.get('evaluation', {})
        macro = ev.get('macro_avg')
        pooled = ev.get('pooled')
        
        if macro:
            mae = macro.get('MAE', '?')
            r2 = macro.get('R2', '?')
        elif pooled:
            mae = pooled.get('MAE', '?')
            r2 = pooled.get('R2', '?')
        else:
            mae, r2 = '?', '?'
        
        mae_str = f"{mae:.4f}" if isinstance(mae, (int, float)) else str(mae)
        r2_str = f"{r2:.4f}" if isinstance(r2, (int, float)) else str(r2)
        params_str = f"{params:,}" if isinstance(params, int) else str(params)
        
        print(f"{label:<35} | {model_type:<30} | {params_str:>10} | {mae_str:>8} | {r2_str:>8}")
    print(f"{'=' * 80}\n")


# ---------------------------------------------------------------------------
# Comparison plot stubs — to be filled in as needed
# ---------------------------------------------------------------------------

def plot_metrics_comparison(runs: list[dict], output_dir: Path, labels: list[str]):
    """Grouped bar chart: MAE, RMSE, R² per model with error bars."""
    # TODO: implement
    pass


def plot_participant_heatmap(runs: list[dict], output_dir: Path, labels: list[str]):
    """Heatmap: rows=participants, cols=models, cells=MAE."""
    # TODO: implement
    pass


def plot_training_curves_overlay(runs: list[dict], output_dir: Path, labels: list[str]):
    """Mean ± std train/val loss overlay across models."""
    # TODO: implement
    pass


def plot_per_weight_comparison(runs: list[dict], output_dir: Path, labels: list[str]):
    """Grouped bar chart per weight class per model."""
    # TODO: implement
    pass


def plot_seqlen_overlay(runs: list[dict], output_dir: Path, labels: list[str]):
    """MAE vs time-into-lift for each model."""
    # TODO: implement
    pass


def plot_radar_chart(runs: list[dict], output_dir: Path, labels: list[str]):
    """Multi-axis radar: MAE, RMSE, R², generalization gap, params."""
    # TODO: implement
    pass


def plot_generalization_gap(runs: list[dict], output_dir: Path, labels: list[str]):
    """Train vs Test MAE side-by-side."""
    # TODO: implement
    pass


def plot_parameter_efficiency(runs: list[dict], output_dir: Path, labels: list[str]):
    """Scatter: parameters vs MAE (or R²)."""
    # TODO: implement
    pass


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Compare multiple model runs from run_data.json files."
    )
    parser.add_argument(
        "run_dirs", nargs="*",
        help="Paths to run directories containing run_data.json"
    )
    parser.add_argument(
        "--glob", type=str, default=None,
        help="Glob pattern for run directories (e.g. 'model/model_results/run_2026*')"
    )
    parser.add_argument(
        "--labels", nargs="*", default=None,
        help="Custom labels for each run (must match number of run_dirs)"
    )
    parser.add_argument(
        "--output", type=str, default="comparison_results",
        help="Output directory for comparison plots"
    )
    args = parser.parse_args()

    # Resolve directories
    if args.glob:
        run_dirs = sorted(Path(".").glob(args.glob))
    else:
        run_dirs = [Path(d) for d in args.run_dirs]

    if not run_dirs:
        print("No run directories specified. Use positional args or --glob.")
        return

    runs = load_runs(run_dirs)
    if not runs:
        print("No valid run_data.json files found.")
        return

    # Generate labels
    labels = []
    for i, run in enumerate(runs):
        custom = args.labels[i] if args.labels and i < len(args.labels) else None
        labels.append(get_label(run, custom))

    summarise_runs(runs, labels)

    if len(runs) < 2:
        print("Need at least 2 runs to generate comparison plots.")
        return

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Generate comparison plots (stubs for now)
    plot_metrics_comparison(runs, output_dir, labels)
    plot_participant_heatmap(runs, output_dir, labels)
    plot_training_curves_overlay(runs, output_dir, labels)
    plot_per_weight_comparison(runs, output_dir, labels)
    plot_seqlen_overlay(runs, output_dir, labels)
    plot_radar_chart(runs, output_dir, labels)
    plot_generalization_gap(runs, output_dir, labels)
    plot_parameter_efficiency(runs, output_dir, labels)

    print(f"All comparison plots saved to {output_dir}")


if __name__ == "__main__":
    main()
