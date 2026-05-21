"""
Feature Sweep for On-Device Inference Optimization
==================================================
Runs a systematic evaluation of 7 different feature configurations to identify
the optimal trade-off between model accuracy and computational cost.

This sweep is DEV_MODE-aware:
  - DEV_MODE ON  → small subset of data, short epochs, fast run.
  - DEV_MODE OFF → full dataset, full epochs.

Configurations swept:
  1. Baseline (All 318 features)
  2. No Skewness & Kurtosis (Removes stats.skew/kurtosis overhead)
  3. No Frequency Domain (Removes signal.welch FFT overhead)
  4. Pure Time Domain (Removes both skew/kurt and freq)
  5. Minimal Cheap (Ultra Real-Time: keeps only cheap O(N) operations)
  6. EMG Only
  7. IMU Only

Usage:
  python -m model.sweep_features
"""

import sys
import time
import copy
import argparse
import numpy as np
import pandas as pd
from pathlib import Path
from datetime import datetime
from sklearn.model_selection import GroupShuffleSplit
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score

# Add project root to sys.path
sys.path.append(str(Path(__file__).resolve().parent.parent))

from model.data_loader import DataLoader, _EXCLUDED_TRUE_WEIGHTS
from model.model_archs.spatio_temporal_transformer3 import SpatioTemporalTransformerRegressor3
from model.config_model import (
    SPATIO_TEMPORAL_TRANSFORMER3_CONFIG, FEATURE_CONFIG,
    DATABASE_CONFIG, AUGMENTATION_CONFIG,
    DEV_MODE, DEV_FRACTION, DEV_EPOCHS, DEV_EARLY_STOPPING_PATIENCE,
    GLOBAL_RANDOM_STATE, MODEL_TYPE
)

CONFIG_GROUPS = {
    "baseline": {
        "name": "Config 1: All Features (Baseline)",
        "desc": "Baseline feature set with all 318 features active (EMG + IMU).",
        "filter": lambda k: True
    },
    "no_skew_kurt": {
        "name": "Config 2: No Skewness & Kurtosis",
        "desc": "Disables stats.skew & kurtosis (removes 62.7% of feature extraction compute).",
        "filter": lambda k: not (k.endswith("_Skew") or k.endswith("_Kurt"))
    },
    "no_freq": {
        "name": "Config 3: No Frequency Domain",
        "desc": "Disables spectral FFT-based features (removes signal.welch & entropy overhead).",
        "filter": lambda k: not any(k.endswith(s) for s in ["_MNF", "_MDF", "_Power", "_SpecEntropy", "_PeakFreq", "_BW", "_DomFreq", "_SpecEnergy"])
    },
    "pure_time": {
        "name": "Config 4: Pure Time Domain",
        "desc": "Keeps only simple time-domain features (removes both skew/kurt & FFT spectral calculations).",
        "filter": lambda k: not any(k.endswith(s) for s in ["_Skew", "_Kurt", "_MNF", "_MDF", "_Power", "_SpecEntropy", "_PeakFreq", "_BW", "_DomFreq", "_SpecEnergy"])
    },
    "minimal_cheap": {
        "name": "Config 5: Minimal Cheap (Ultra Real-Time)",
        "desc": "Keeps only O(N) linear time-domain metrics (MAV, WL, ZC, SSC, WAMP, SMA, Mean, Max, Jerk, SVM_Mean).",
        "filter": lambda k: any(k.endswith(s) for s in ["_MAV", "_WL", "_ZC", "_SSC", "_WAMP", "_Myopulse", "_Mean", "_Max", "_SMA", "_P2P", "_Jerk", "_SVM_Mean"])
    },
    "emg_only": {
        "name": "Config 6: EMG Only",
        "desc": "Baseline features extracted ONLY from the 8 EMG channels (IMU disabled).",
        "filter": lambda k: "_EMG_" in k
    },
    "imu_only": {
        "name": "Config 7: IMU Only",
        "desc": "Baseline features extracted ONLY from the 12 IMU/SVM channels (EMG disabled).",
        "filter": lambda k: "_IMU_" in k or "_SVM_" in k
    }
}


def filter_features(X_df, config_key):
    """Filters sequence dictionary keys dynamically based on the configuration rules."""
    key_test = CONFIG_GROUPS[config_key]["filter"]

    new_seq_dicts = []
    for seq in X_df['sequence_dicts']:
        new_seq = []
        for w in seq:
            new_w = {k: v for k, v in w.items() if key_test(k)}
            new_seq.append(new_w)
        new_seq_dicts.append(new_seq)
        
    X_filtered = X_df.copy()
    X_filtered['sequence_dicts'] = new_seq_dicts
    return X_filtered


def main():
    parser = argparse.ArgumentParser(
        description="Run a systematic feature sweep to evaluate performance vs computational trade-offs."
    )
    parser.add_argument("--epochs", type=int, default=None,
                        help="Number of epochs to train per configuration (defaults to DEV config or 200).")
    args = parser.parse_args()

    epochs = args.epochs if args.epochs is not None else (DEV_EPOCHS if DEV_MODE else 200)
    patience = 5 if DEV_MODE else 20

    base_dir = Path(__file__).resolve().parent.parent
    segments_dir = DATABASE_CONFIG['segments_dir']
    results_dir = base_dir / "model" / "model_results"

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = results_dir / f"sweep_features_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 65)
    print("  ON-DEVICE FEATURE CONFIGURATION SWEEP")
    print("=" * 65)
    print(f"  DEV_MODE             : {DEV_MODE}")
    print(f"  Epochs per config    : {epochs}")
    print(f"  Early-stop patience  : {patience}")
    print(f"  Results directory    : {run_dir}")
    print("=" * 65)

    # 1. Load data once
    h5_paths = sorted([p for p in segments_dir.glob("*.h5") if not p.name.startswith("._")])
    if not h5_paths:
        print(f"No HDF5 segment files found in {segments_dir}.")
        return

    loader = DataLoader()
    print("\nLoading precomputed features (sequence mode)…")
    df = loader.load_and_extract_features(
        h5_paths,
        is_sequence=True,
        use_precomputed=True,
    )
    if df.empty:
        print("Feature loading failed or produced an empty DataFrame.")
        return

    # Filter weight classes
    if _EXCLUDED_TRUE_WEIGHTS and "weight" in df.columns:
        before = len(df)
        mask = ~df["weight"].isin(_EXCLUDED_TRUE_WEIGHTS)
        df = df[mask].reset_index(drop=True)
        n_dropped = before - len(df)
        if n_dropped > 0:
            print(f"[Sweep] Dropped {n_dropped} segments with excluded weight classes.")

    # DEV_MODE subsampling
    if DEV_MODE:
        n_keep = max(1, int(len(df) * DEV_FRACTION))
        df = df.sample(n=n_keep, random_state=GLOBAL_RANDOM_STATE).reset_index(drop=True)
        print(f"[DEV MODE] Subsampled to {len(df)} segments ({DEV_FRACTION*100:.0f}%)")

    print(f"Total segments: {len(df)}")

    X, y = loader.prepare_for_ml(df, target_col="weight")

    if "subject" not in df.columns:
        print("Error: 'subject' column missing. Cannot split by participant.")
        return

    subjects = df["subject"].values
    X["subject"] = subjects

    # 2. Setup a fixed cross-participant split for fair evaluation
    n_unique_subs = len(np.unique(subjects))
    test_ratio = min(0.3, max(0.1, 4 / max(1, n_unique_subs)))

    gss = GroupShuffleSplit(n_splits=1, test_size=test_ratio, random_state=42)
    train_idx, val_idx = next(gss.split(X, y, groups=subjects))

    X_train_raw = X.iloc[train_idx].copy()
    y_train     = y.iloc[train_idx].copy()
    X_val_raw   = X.iloc[val_idx].copy()
    y_val       = y.iloc[val_idx].copy()

    val_subs = sorted(X_val_raw["subject"].unique())
    train_subs = sorted(X_train_raw["subject"].unique())

    print(f"\nParticipant Split:")
    print(f"  Training   ({len(train_subs)} subjects, {len(X_train_raw)} segs): {', '.join(train_subs)}")
    print(f"  Validation ({len(val_subs)} subjects, {len(X_val_raw)} segs): {', '.join(val_subs)}")

    # 3. Sweep loop
    results = []
    sweep_start = time.time()

    for idx, (config_key, config_info) in enumerate(CONFIG_GROUPS.items(), 1):
        print(f"\n{'='*65}")
        print(f"Running Configuration {idx}/7: {config_info['name']}")
        print(f"Description: {config_info['desc']}")
        print(f"{'='*65}")

        # Filter features dynamically
        X_train_filtered = filter_features(X_train_raw, config_key)
        X_val_filtered   = filter_features(X_val_raw, config_key)

        # Drop subject column for model training
        X_train = X_train_filtered.drop(columns=["subject"])
        X_val   = X_val_filtered.drop(columns=["subject"])

        # Count the number of active features by looking at the first dictionary
        active_features = []
        for seq in X_train['sequence_dicts']:
            if seq and seq[0]:
                active_features = sorted(list(seq[0].keys()))
                break
        n_features = len(active_features)
        print(f"  Active features: {n_features}")

        if n_features == 0:
            print("  [WARNING] No active features found for this configuration. Skipping.")
            continue

        # Build model config
        model_config = copy.deepcopy(SPATIO_TEMPORAL_TRANSFORMER3_CONFIG)
        model_config['epochs'] = epochs
        model_config['early_stopping_patience'] = patience
        model_config['use_checkpointing'] = False  # Faster sweep execution

        # Initialize model
        model = SpatioTemporalTransformerRegressor3(**model_config)

        iter_start = time.time()
        try:
            # Fit model
            model.fit(X_train, y_train, X_val=X_val, y_val=y_val)

            # Evaluate
            preds = model.predict(X_val)
            preds = np.maximum(0.0, preds)  # clamp negatives

            mae = mean_absolute_error(y_val, preds)
            mse = mean_squared_error(y_val, preds)
            rmse = np.sqrt(mse)
            r2 = r2_score(y_val, preds)

            elapsed = time.time() - iter_start
            total_params = sum(p.numel() for p in model.model.parameters())

            print(f"\n  Results for {config_key.upper()}:")
            print(f"    RMSE: {rmse:.4f} | MAE: {mae:.4f} | R²: {r2:.4f}")
            print(f"    Parameters: {total_params:,} | Training Time: {elapsed:.1f}s")

            results.append({
                "config_key": config_key,
                "name": config_info["name"],
                "n_features": n_features,
                "rmse": rmse,
                "mae": mae,
                "r2": r2,
                "total_params": total_params,
                "training_time": elapsed,
                "epochs_trained": len(model.loss_history["train"]),
            })
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print(f"  [ERROR] Configuration failed: {exc}. Skipping.")
            continue

    total_sweep_time = time.time() - sweep_start
    print("\n" + "=" * 65)
    print("  FEATURE SWEEP COMPLETE")
    print(f"  Total sweep time: {total_sweep_time/60:.1f} minutes")
    print("=" * 65)

    if not results:
        print("All configurations failed. No report saved.")
        return

    # Sort results by validation RMSE
    results.sort(key=lambda r: r["rmse"])

    # 4. Generate Report
    report_file = run_dir / "sweep_features_report.md"
    csv_file = run_dir / "sweep_features_results.csv"

    # Save CSV
    pd.DataFrame(results).to_csv(csv_file, index=False)

    # Save Markdown Report
    with open(report_file, "w") as f:
        f.write("# On-Device Feature Configuration Sweep Report\n\n")
        f.write(f"**Date:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        f.write(f"**DEV_MODE:** {DEV_MODE}\n\n")
        f.write(f"**Total Sweep Execution Time:** {total_sweep_time/60:.1f} minutes\n\n")
        f.write(f"**Cross-Participant Validation Group:** held out subjects `{', '.join(val_subs)}` for testing.\n\n")

        f.write("## Ranked Feature Configurations (Sorted by RMSE)\n\n")
        f.write("| Rank | Configuration | Features | Val RMSE | Val MAE | Val R² | Model Params | Training Time | Epochs |\n")
        f.write("| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n")

        for rank, res in enumerate(results, 1):
            f.write(f"| {rank} | {res['name']} | {res['n_features']} | **{res['rmse']:.4f}** | {res['mae']:.4f} | {res['r2']:.4f} | {res['total_params']:,} | {res['training_time']:.1f}s | {res['epochs_trained']} |\n")

        f.write("\n## Configuration Key Details & Interpretations\n\n")
        for res in results:
            key = res["config_key"]
            f.write(f"### {res['name']}\n")
            f.write(f"* **Active Features:** {res['n_features']}\n")
            f.write(f"* **Val RMSE / MAE:** {res['rmse']:.4f} / {res['mae']:.4f}\n")
            f.write(f"* **Generalization R²:** {res['r2']:.4f}\n")
            f.write(f"* **On-Device Impact:**\n")
            if key == "baseline":
                f.write("  * Baseline full dataset. Highest mathematical richness, but requires 318 features including expensive windowed Welch FFTs and third/fourth-order SciPy moments.\n")
            elif key == "no_skew_kurt":
                f.write("  * **Eliminates ~62.7% of Python's feature extraction overhead.** Removes stats.skew and stats.kurtosis. If accuracy is matching baseline, this is an automatic green light for real-time loops.\n")
            elif key == "no_freq":
                f.write("  * **Eliminates ~19.5% of feature extraction compute.** Bypasses windowed Welch periodograms and spectral entropy calculations. Very strong efficiency gains.\n")
            elif key == "pure_time":
                f.write("  * **Eliminates ~82.2% of total feature extraction compute.** No FFTs, no higher-order statistical moments. Relies solely on standard vectorized metrics (MAV, RMS, WL, etc.).\n")
            elif key == "minimal_cheap":
                f.write("  * **Ultra Real-Time Optimal.** Restricts computations purely to $O(N)$ linear vectorized math with no logs, exponentials, or complex sorting. Highly recommended for ultra-low power devices.\n")
            elif key == "emg_only":
                f.write("  * Analyzes the predictive capacity of EMG muscle signals alone. Eliminates all 12 IMU kinematic channels.\n")
            elif key == "imu_only":
                f.write("  * Analyzes the predictive capacity of IMU kinematics alone. Eliminates all 8 EMG muscle activation channels.\n")
            f.write("\n")

    print(f"\nFeature sweep report saved to: {report_file}")
    print(f"Feature sweep CSV results saved to: {csv_file}")


if __name__ == "__main__":
    main()
