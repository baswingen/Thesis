import os
import sys
import re
import copy
import argparse
import numpy as np
import pandas as pd
from pathlib import Path
from datetime import datetime

# Add project root to sys.path so 'model' package can be found
sys.path.append(str(Path(__file__).parent.parent))

from model.data_loader import DataLoader
from model.run_model import initialize_model, MODEL_TYPE
from model.config_model import CV_CONFIG

from sklearn.model_selection import StratifiedKFold, train_test_split
from sklearn.metrics import accuracy_score

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

MODALITIES = ['both', 'emg', 'imu']

# EMG feature-type suffixes (must match what feature_extraction.py produces)
EMG_FEATURE_TYPES = [
    'EMG_MAV', 'EMG_RMS', 'EMG_WL', 'EMG_ZC',
    'EMG_SSC', 'EMG_VAR', 'EMG_MNF', 'EMG_MDF', 'EMG_Power',
]

# IMU feature-type suffixes
IMU_FEATURE_TYPES = [
    'IMU_Mean', 'IMU_Var', 'IMU_Std', 'IMU_Max', 'IMU_Min',
    'IMU_RMS', 'IMU_SMA', 'IMU_DomFreq', 'IMU_SpecEnergy',
]

ALL_FEATURE_TYPES = EMG_FEATURE_TYPES + IMU_FEATURE_TYPES

# ---------------------------------------------------------------------------
# Feature helpers
# ---------------------------------------------------------------------------

def get_all_column_names(df: pd.DataFrame, is_sequence: bool) -> list:
    """Return all feature column/key names from the dataset."""
    if is_sequence:
        # Peek at the first row's first window dict to get all keys
        first_row = df['sequence_dicts'].iloc[0]
        if first_row:
            return list(first_row[0].keys())
        return []
    else:
        # All columns except meta columns
        exclude = {'weight', 'label', 'participant', 'trial', 'segment'}
        return [c for c in df.columns if c not in exclude]


def get_unique_channels(all_columns: list) -> list:
    """
    Extract unique sensor channel prefixes from feature column names.
    Convention: <channel>_EMG_<feature> or <channel>_IMU_<feature>
    Returns a sorted list of unique channel identifiers, e.g. ['CH1', 'ACC_X', ...].
    """
    channels = set()
    pattern = re.compile(r'^(.+?)_(EMG|IMU)_')
    for col in all_columns:
        m = pattern.match(col)
        if m:
            channels.add(m.group(1))
    return sorted(channels)


def get_present_feature_types(all_columns: list) -> list:
    """Return only the feature types that actually exist in the column list."""
    present = []
    for ft in ALL_FEATURE_TYPES:
        suffix = f'_{ft}'
        if any(col.endswith(suffix) for col in all_columns):
            present.append(ft)
    return present


# ---------------------------------------------------------------------------
# Filtering helpers
# ---------------------------------------------------------------------------

def _cols_matching_patterns(all_cols: list, patterns: list) -> set:
    """Return the set of column names that match any of the given substring patterns."""
    to_drop = set()
    for col in all_cols:
        for pat in patterns:
            if pat in col:
                to_drop.add(col)
                break
    return to_drop


def filter_features(df: pd.DataFrame, modality: str, is_sequence: bool) -> pd.DataFrame:
    """
    Prop 0 (existing): filter by modality — 'both', 'emg', or 'imu'.
    """
    df_filtered = df.copy(deep=True)

    if modality == 'both':
        return df_filtered

    if is_sequence:
        for i in range(len(df_filtered)):
            new_seq = []
            for seq_dict in df_filtered.at[i, 'sequence_dicts']:
                if modality == 'emg':
                    filtered = {k: v for k, v in seq_dict.items() if '_IMU_' not in k}
                else:  # imu
                    filtered = {k: v for k, v in seq_dict.items() if '_EMG_' not in k}
                new_seq.append(filtered)
            df_filtered.at[i, 'sequence_dicts'] = new_seq
    else:
        drop_pat = '_IMU_' if modality == 'emg' else '_EMG_'
        cols_to_drop = [c for c in df_filtered.columns if drop_pat in c]
        df_filtered.drop(columns=cols_to_drop, inplace=True)

    return df_filtered


def exclude_by_patterns(df: pd.DataFrame, patterns: list, is_sequence: bool) -> pd.DataFrame:
    """
    Generic exclusion: drops any feature column/key whose name contains ANY of the given pattern strings.
    Used by both channel ablation and feature-type ablation.
    """
    df_filtered = df.copy(deep=True)

    if is_sequence:
        for i in range(len(df_filtered)):
            new_seq = []
            for seq_dict in df_filtered.at[i, 'sequence_dicts']:
                filtered = {
                    k: v for k, v in seq_dict.items()
                    if not any(pat in k for pat in patterns)
                }
                new_seq.append(filtered)
            df_filtered.at[i, 'sequence_dicts'] = new_seq
    else:
        all_cols = list(df_filtered.columns)
        cols_to_drop = _cols_matching_patterns(all_cols, patterns)
        # Never drop meta columns
        meta = {'weight', 'label', 'participant', 'trial', 'segment'}
        cols_to_drop -= meta
        df_filtered.drop(columns=list(cols_to_drop), inplace=True, errors='ignore')

    return df_filtered


# ---------------------------------------------------------------------------
# Evaluation
# ---------------------------------------------------------------------------

def run_evaluation(X, y, model_type, is_regression, strat_labels=None, use_cv=False, n_folds=5):
    """Train and evaluate a model, returning a metrics dict or None on failure."""
    if use_cv:
        skf = StratifiedKFold(n_splits=n_folds, shuffle=True, random_state=42)
        cv_metrics = []

        for fold, (train_idx, test_idx) in enumerate(skf.split(X, strat_labels), 1):
            X_train_fold = X.iloc[train_idx].copy()
            X_test_fold  = X.iloc[test_idx].copy()
            y_train_fold = y.iloc[train_idx].copy()
            y_test_fold  = y.iloc[test_idx].copy()

            model = initialize_model(model_type)
            if model is None:
                return None

            model.fit(X_train_fold, y_train_fold)

            if is_regression:
                metrics, _ = model.evaluate(X_test_fold, y_test_fold)
                cv_metrics.append(metrics)
            else:
                y_pred_fold = model.predict(X_test_fold)
                acc = accuracy_score(y_test_fold, y_pred_fold)
                cv_metrics.append({'accuracy': acc})

        if is_regression:
            avg_metrics = {k: np.mean([m[k] for m in cv_metrics]) for k in cv_metrics[0].keys()}
            return avg_metrics
        else:
            avg_acc = np.mean([m['accuracy'] for m in cv_metrics])
            return {'accuracy': avg_acc}

    else:
        X_train, X_test, y_train, y_test = train_test_split(
            X, y, test_size=0.2, random_state=7, stratify=strat_labels
        )

        model = initialize_model(model_type)
        if model is None:
            return None

        model.fit(X_train, y_train)

        if is_regression:
            metrics, _ = model.evaluate(X_test, y_test)
            return metrics
        else:
            y_pred = model.predict(X_test)
            accuracy = accuracy_score(y_test, y_pred)
            return {'accuracy': accuracy}


# ---------------------------------------------------------------------------
# Ablation runners
# ---------------------------------------------------------------------------

def run_modality_ablation(df_raw, loader, model_type, is_regression, is_sequence,
                           target_col, use_cv, n_folds):
    """
    Phase 1 (original): evaluate with all features, EMG only, IMU only.
    Returns dict: {modality: metrics_dict}
    """
    results = {}
    print(f"\n{'='*60}")
    print("PHASE 1 — MODALITY ABLATION (both / emg / imu)")
    print(f"{'='*60}")

    for modality in MODALITIES:
        print(f"\n  Modality: {modality.upper()}")
        df_filtered = filter_features(df_raw, modality, is_sequence)
        X, y = loader.prepare_for_ml(df_filtered, target_col=target_col)
        strat_labels = df_filtered["label"] if "label" in df_filtered.columns else None
        metrics = run_evaluation(X, y, model_type, is_regression, strat_labels, use_cv, n_folds)
        results[modality] = metrics
        if metrics:
            for k, v in metrics.items():
                print(f"    {k}: {v:.4f}")
        else:
            print("    Model failed to initialize.")

    return results


def run_channel_ablation(df_raw, loader, model_type, is_regression, is_sequence,
                          target_col, use_cv, n_folds, baseline_metrics, primary_metric):
    """
    Proposition 1 — Leave-one-channel-out ablation.
    For each sensor channel, drops ALL features from that channel and evaluates.
    Returns dict: {channel: {'metrics': ..., 'delta': ...}}
    """
    all_cols = get_all_column_names(df_raw, is_sequence)
    channels  = get_unique_channels(all_cols)

    print(f"\n{'='*60}")
    print(f"PHASE 2 — CHANNEL ABLATION (leave-one-channel-out, {len(channels)} channels)")
    print(f"{'='*60}")

    results = {}
    base_val = baseline_metrics.get(primary_metric) if baseline_metrics else None

    for ch in channels:
        # Drop all features belonging to this channel (both EMG and IMU variants)
        patterns = [f'{ch}_EMG_', f'{ch}_IMU_']
        df_excl = exclude_by_patterns(df_raw, patterns, is_sequence)

        X, y    = loader.prepare_for_ml(df_excl, target_col=target_col)
        strat_labels = df_excl["label"] if "label" in df_excl.columns else None
        metrics = run_evaluation(X, y, model_type, is_regression, strat_labels, use_cv, n_folds)

        delta = None
        if metrics and base_val is not None:
            delta = metrics.get(primary_metric, None)
            if delta is not None:
                delta = delta - base_val  # positive = worse (for MAE); negative = worse (for R2)

        results[ch] = {'metrics': metrics, 'delta': delta}

        delta_str = f"{delta:+.4f}" if delta is not None else "N/A"
        print(f"\n  Excluded channel: {ch}  |  Δ{primary_metric}: {delta_str}")
        if metrics:
            for k, v in metrics.items():
                print(f"    {k}: {v:.4f}")
        else:
            print("    Model failed.")

    return results


def run_feature_type_ablation(df_raw, loader, model_type, is_regression, is_sequence,
                               target_col, use_cv, n_folds, baseline_metrics, primary_metric):
    """
    Proposition 2 — Leave-one-feature-type-out ablation.
    For each feature type (e.g. EMG_RMS, IMU_Mean), drops that feature type across all channels.
    Returns dict: {feature_type: {'metrics': ..., 'delta': ...}}
    """
    all_cols      = get_all_column_names(df_raw, is_sequence)
    present_types = get_present_feature_types(all_cols)

    print(f"\n{'='*60}")
    print(f"PHASE 3 — FEATURE-TYPE ABLATION (leave-one-out, {len(present_types)} types)")
    print(f"{'='*60}")

    results = {}
    base_val = baseline_metrics.get(primary_metric) if baseline_metrics else None

    for ft in present_types:
        # A feature type appears as a suffix: e.g. '_EMG_MAV' in 'CH1_EMG_MAV'
        pattern = [f'_{ft}']
        df_excl = exclude_by_patterns(df_raw, pattern, is_sequence)

        X, y = loader.prepare_for_ml(df_excl, target_col=target_col)
        strat_labels = df_excl["label"] if "label" in df_excl.columns else None
        metrics = run_evaluation(X, y, model_type, is_regression, strat_labels, use_cv, n_folds)

        delta = None
        if metrics and base_val is not None:
            delta = metrics.get(primary_metric, None)
            if delta is not None:
                delta = delta - base_val

        results[ft] = {'metrics': metrics, 'delta': delta}

        delta_str = f"{delta:+.4f}" if delta is not None else "N/A"
        print(f"\n  Excluded feature type: {ft}  |  Δ{primary_metric}: {delta_str}")
        if metrics:
            for k, v in metrics.items():
                print(f"    {k}: {v:.4f}")
        else:
            print("    Model failed.")

    return results


def include_only_channel(df: pd.DataFrame, channel: str, is_sequence: bool) -> pd.DataFrame:
    """
    Keep ONLY features belonging to the given channel; drop everything else.
    The inverse of exclude_by_patterns for a single channel.
    """
    df_filtered = df.copy(deep=True)
    # A feature belongs to `channel` if its name starts with '<channel>_EMG_' or '<channel>_IMU_'
    keep_patterns = [f'{channel}_EMG_', f'{channel}_IMU_']

    if is_sequence:
        for i in range(len(df_filtered)):
            new_seq = []
            for seq_dict in df_filtered.at[i, 'sequence_dicts']:
                kept = {
                    k: v for k, v in seq_dict.items()
                    if any(k.startswith(pat) for pat in keep_patterns)
                }
                new_seq.append(kept)
            df_filtered.at[i, 'sequence_dicts'] = new_seq
    else:
        meta = {'weight', 'label', 'participant', 'trial', 'segment'}
        cols_to_drop = [
            c for c in df_filtered.columns
            if c not in meta and not any(c.startswith(pat) for pat in keep_patterns)
        ]
        df_filtered.drop(columns=cols_to_drop, inplace=True, errors='ignore')

    return df_filtered


def run_single_channel_ablation(df_raw, loader, model_type, is_regression, is_sequence,
                                 target_col, use_cv, n_folds, primary_metric):
    """
    Proposition 1b — Single-channel ablation.
    For each sensor channel, trains on ONLY that channel's features.
    Returns dict: {channel: {'metrics': ..., 'rank_val': ...}}
    where rank_val is the primary metric value (lower MAE / higher R2 = better).
    """
    all_cols = get_all_column_names(df_raw, is_sequence)
    channels = get_unique_channels(all_cols)

    print(f"\n{'='*60}")
    print(f"PHASE — SINGLE-CHANNEL ABLATION ({len(channels)} channels)")
    print(f"{'='*60}")

    results = {}

    for ch in channels:
        df_only = include_only_channel(df_raw, ch, is_sequence)

        X, y = loader.prepare_for_ml(df_only, target_col=target_col)
        strat_labels = df_only["label"] if "label" in df_only.columns else None
        metrics = run_evaluation(X, y, model_type, is_regression, strat_labels, use_cv, n_folds)

        rank_val = metrics.get(primary_metric) if metrics else None
        results[ch] = {'metrics': metrics, 'rank_val': rank_val}

        rank_str = f"{rank_val:.4f}" if rank_val is not None else "N/A"
        print(f"\n  Channel only: {ch}  |  {primary_metric}: {rank_str}")
        if metrics:
            for k, v in metrics.items():
                print(f"    {k}: {v:.4f}")
        else:
            print("    Model failed.")

    return results


# ---------------------------------------------------------------------------
# Report helpers
# ---------------------------------------------------------------------------

def _write_section_header(f, title):
    f.write("\n" + "=" * 60 + "\n")
    f.write(f"{title}\n")
    f.write("=" * 60 + "\n\n")


def _write_metrics(f, metrics, indent=2):
    pad = " " * indent
    if metrics:
        for k, v in metrics.items():
            f.write(f"{pad}{k}: {v:.4f}\n")
    else:
        f.write(f"{pad}Model failed to initialize.\n")


def _rank_by_delta(results_dict, primary_metric, higher_is_worse=True):
    """
    Sort results dict by absolute delta descending (most impactful first).
    For metrics where higher = worse (MAE), a larger positive delta means
    the excluded item was more important.
    """
    items = []
    for name, val in results_dict.items():
        delta = val.get('delta')
        if delta is not None:
            impact = delta if higher_is_worse else -delta
            items.append((name, delta, impact))
    items.sort(key=lambda x: x[2], reverse=True)
    return items


def write_report(run_dir, model_type, timestamp, use_cv, modes_run,
                 primary_metric, is_regression,
                 modality_results=None, channel_leave_results=None,
                 feature_results=None, channel_single_results=None):
    report_file = run_dir / "ablation_report.txt"

    # MAE/RMSE: higher after exclusion = worse = item was important
    # R2: lower after exclusion = worse
    higher_is_worse = is_regression and primary_metric in ('MAE', 'mae', 'RMSE', 'rmse')
    # For single-channel: lower MAE = better; lower R2 = worse
    single_lower_is_better = higher_is_worse  # same logic: MAE → lower is better

    with open(report_file, "w") as f:
        f.write("=" * 60 + "\n")
        f.write("ABLATION STUDY REPORT\n")
        f.write(f"Model        : {model_type.upper()}\n")
        f.write(f"Run Timestamp: {timestamp}\n")
        f.write(f"Eval Mode    : {'Cross-Validation' if use_cv else 'Train/Test Split'}\n")
        f.write(f"Primary Metric: {primary_metric}\n")
        f.write(f"Phases run   : {', '.join(modes_run)}\n")
        f.write("=" * 60 + "\n")

        # ---- Modality
        if modality_results is not None:
            _write_section_header(f, "MODALITY ABLATION (both / emg-only / imu-only)")
            for modality in MODALITIES:
                f.write(f"  Modality: {modality.upper()}\n")
                _write_metrics(f, modality_results.get(modality), indent=4)
                f.write("\n")

        # ---- Leave-one-channel-out
        if channel_leave_results is not None:
            _write_section_header(f, "CHANNEL ABLATION — leave-one-channel-out")
            baseline_val = (
                modality_results['both'].get(primary_metric)
                if modality_results and modality_results.get('both') else None
            )
            if baseline_val is not None:
                f.write(f"  Baseline ({primary_metric}): {baseline_val:.4f}\n\n")

            ranked = _rank_by_delta(channel_leave_results, primary_metric, higher_is_worse)
            f.write("  Ranked by impact (most → least important):\n")
            f.write(f"  {'Channel':<22} {'Δ'+primary_metric:>12}  {'|Δ|':>8}\n")
            f.write("  " + "-" * 46 + "\n")
            for name, delta, impact in ranked:
                f.write(f"  {name:<22} {delta:>+12.4f}  {impact:>8.4f}\n")

            f.write("\n  Full results per channel:\n")
            for ch, val in channel_leave_results.items():
                f.write(f"\n  Excluded: {ch}\n")
                _write_metrics(f, val['metrics'], indent=4)

        # ---- Single-channel
        if channel_single_results is not None:
            _write_section_header(f, "CHANNEL ABLATION — single-channel-only")
            # Rank: if MAE, lower rank_val = better; sort ascending
            items = [
                (ch, val['rank_val'])
                for ch, val in channel_single_results.items()
                if val['rank_val'] is not None
            ]
            items.sort(key=lambda x: x[1], reverse=not single_lower_is_better)
            f.write(f"  Ranked by {primary_metric} (best performing channel first):\n")
            f.write(f"  {'Channel':<22} {primary_metric:>12}\n")
            f.write("  " + "-" * 36 + "\n")
            for name, val in items:
                f.write(f"  {name:<22} {val:>12.4f}\n")

            f.write("\n  Full results per channel:\n")
            for ch, val in channel_single_results.items():
                f.write(f"\n  Only: {ch}\n")
                _write_metrics(f, val['metrics'], indent=4)

        # ---- Leave-one-feature-type-out
        if feature_results is not None:
            _write_section_header(f, "FEATURE-TYPE ABLATION — leave-one-type-out")
            baseline_val = (
                modality_results['both'].get(primary_metric)
                if modality_results and modality_results.get('both') else None
            )
            if baseline_val is not None:
                f.write(f"  Baseline ({primary_metric}): {baseline_val:.4f}\n\n")

            ranked = _rank_by_delta(feature_results, primary_metric, higher_is_worse)
            f.write("  Ranked by impact (most → least important):\n")
            f.write(f"  {'Feature Type':<22} {'Δ'+primary_metric:>12}  {'|Δ|':>8}\n")
            f.write("  " + "-" * 46 + "\n")
            for name, delta, impact in ranked:
                f.write(f"  {name:<22} {delta:>+12.4f}  {impact:>8.4f}\n")

            f.write("\n  Full results per feature type:\n")
            for ft, val in feature_results.items():
                f.write(f"\n  Excluded: {ft}\n")
                _write_metrics(f, val['metrics'], indent=4)

    return report_file


# ---------------------------------------------------------------------------
# CLI argument parsing
# ---------------------------------------------------------------------------

VALID_MODES = {
    'modality'       : 'Modality ablation: both / emg-only / imu-only',
    'leave_channel'  : 'Leave-one-channel-out (Prop 1 — which channel hurts most when removed)',
    'single_channel' : 'Single-channel-only  (Prop 1b — which channel alone is best)',
    'leave_feature'  : 'Leave-one-feature-type-out (Prop 2)',
    'all'            : 'Run every phase above',
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Ablation study for weight-estimation models.",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    mode_help = "Phase(s) to run (space-separated). Choices:\n" + "\n".join(
        f"  {k:<18} {v}" for k, v in VALID_MODES.items()
    )
    parser.add_argument(
        '--mode', '-m',
        nargs='+',
        default=['all'],
        choices=list(VALID_MODES.keys()),
        metavar='MODE',
        help=mode_help,
    )
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = parse_args()
    modes = set(args.mode)
    if 'all' in modes:
        modes = {'modality', 'leave_channel', 'single_channel', 'leave_feature'}

    base_dir     = Path(__file__).parent.parent
    segments_dir = base_dir / "database" / "segments"
    results_dir  = base_dir / "model" / "model_results"

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir   = results_dir / f"ablation_{MODEL_TYPE}_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)

    print(f"Results will be saved to: {run_dir}")
    print(f"Phases selected  : {', '.join(sorted(modes))}")

    h5_paths = list(segments_dir.glob("*.h5"))
    if not h5_paths:
        print(f"No HDF5 segment files found in {segments_dir}.")
        return

    loader = DataLoader()
    print("Extracting features from HDF5 files...")

    model_type    = MODEL_TYPE.lower()
    is_sequence   = model_type in ["gru", "lstm", "cnn_lstm", "transformer"]
    is_regression = model_type in ["svr", "rf", "gb", "mlp", "gru", "lstm", "cnn_lstm", "transformer"]

    if is_sequence:
        if model_type == "gru":
            from model.config_model import GRU_CONFIG as conf
        elif model_type == "lstm":
            from model.config_model import LSTM_CONFIG as conf
        elif model_type == "cnn_lstm":
            from model.config_model import CNN_LSTM_CONFIG as conf
        else:
            from model.config_model import TRANSFORMER_CONFIG as conf
        df_raw = loader.load_and_extract_features(
            h5_paths,
            window_size_sec=conf.get('window_size_sec', 0.25),
            window_step_sec=conf.get('window_step_sec', 0.1)
        )
    else:
        df_raw = loader.load_and_extract_features(h5_paths)

    if df_raw.empty:
        print("Data extraction failed or produced an empty DataFrame.")
        return

    target_col     = "weight" if is_regression else "label"
    use_cv         = CV_CONFIG.get('use_cross_val', False)
    n_folds        = CV_CONFIG.get('n_folds', 5)
    primary_metric = "MAE" if is_regression else "accuracy"

    # Dataset overview
    all_cols   = get_all_column_names(df_raw, is_sequence)
    channels   = get_unique_channels(all_cols)
    feat_types = get_present_feature_types(all_cols)
    print(f"\nDataset: {len(df_raw)} samples")
    print(f"Unique channels  : {len(channels)} → {channels}")
    print(f"Feature types    : {len(feat_types)} → {feat_types}")

    # ---- Dispatch to selected phases ----------------------------------------
    modality_results      = None
    channel_leave_results = None
    channel_single_results = None
    feature_results       = None

    # Modality phase — also serves as the baseline for leave-one-out deltas
    if 'modality' in modes or 'leave_channel' in modes or 'leave_feature' in modes:
        modality_results = run_modality_ablation(
            df_raw, loader, model_type, is_regression, is_sequence,
            target_col, use_cv, n_folds
        )

    baseline_metrics = modality_results.get('both') if modality_results else None

    if 'leave_channel' in modes:
        channel_leave_results = run_channel_ablation(
            df_raw, loader, model_type, is_regression, is_sequence,
            target_col, use_cv, n_folds, baseline_metrics, primary_metric
        )

    if 'single_channel' in modes:
        channel_single_results = run_single_channel_ablation(
            df_raw, loader, model_type, is_regression, is_sequence,
            target_col, use_cv, n_folds, primary_metric
        )

    if 'leave_feature' in modes:
        feature_results = run_feature_type_ablation(
            df_raw, loader, model_type, is_regression, is_sequence,
            target_col, use_cv, n_folds, baseline_metrics, primary_metric
        )

    # ---- Save report ---------------------------------------------------------
    report_file = write_report(
        run_dir=run_dir,
        model_type=model_type,
        timestamp=timestamp,
        use_cv=use_cv,
        modes_run=sorted(modes),
        primary_metric=primary_metric,
        is_regression=is_regression,
        modality_results=modality_results,
        channel_leave_results=channel_leave_results,
        feature_results=feature_results,
        channel_single_results=channel_single_results,
    )

    print(f"\n{'='*60}")
    print(f"Ablation study complete. Report saved to:\n  {report_file}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
