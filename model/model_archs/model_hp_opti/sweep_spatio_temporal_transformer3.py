"""
Spatio-Temporal Transformer 3 Hyperparameter & Feature Sweep
============================================================
Runs a randomized grid-search or feature selection sweep for the Modality-Grouped
Spatio-Temporal Transformer 3 with a *single* fixed cross-participant holdout split.

It supports 3 modes (GRID_SEARCH_MODE in config_model.py or --mode CLI argument):
  1. model_arch     : Sweeps core spatial/temporal attention architecture parameters.
  2. generalization : Sweeps regularization (dropout, weight decay) and data augmentation.
  3. features       : Sweeps the feature selection space (150 random toggle configurations)
                      and runs a Ridge regression contribution analysis on validation R².

This sweep is DEV_MODE-aware:
  - DEV_MODE ON  → small n_iter, subsampled data, short epochs, fast run.
  - DEV_MODE OFF → full dataset, full iterations, full epochs.

Usage:
  # Run via run_model.py by setting RUN_GRID_SEARCH = True in config_model.py
  # Standalone:
  python sweep_spatio_temporal_transformer3.py --mode model_arch
  python sweep_spatio_temporal_transformer3.py --mode generalization
  python sweep_spatio_temporal_transformer3.py --mode features
"""

import sys
import time
import copy
import argparse
import numpy as np
import pandas as pd
from pathlib import Path
from datetime import datetime
from sklearn.model_selection import ParameterSampler, GroupShuffleSplit
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
from sklearn.linear_model import RidgeCV

# Add project root to sys.path
sys.path.append(str(Path(__file__).resolve().parent.parent.parent.parent))

try:
    from model.data_loader import DataLoader, _EXCLUDED_TRUE_WEIGHTS
except ImportError:
    # fallback if there was a typo in import
    from model.data_loader import DataLoader
    _EXCLUDED_TRUE_WEIGHTS = [0.9, 2.24]

from model.model_archs.spatio_temporal_transformer3 import SpatioTemporalTransformerRegressor3
from model.config_model import (
    SPATIO_TEMPORAL_TRANSFORMER3_CONFIG, FEATURE_CONFIG,
    DATABASE_CONFIG, AUGMENTATION_CONFIG,
    DEV_MODE, DEV_FRACTION, DEV_EPOCHS, DEV_EARLY_STOPPING_PATIENCE,
    GLOBAL_RANDOM_STATE, GRID_SEARCH_MODE, CV_STRATEGY
)

# Define 33 core feature categories for "features" mode
FEATURE_CATEGORIES = {}

# 20 EMG feature categories
emg_feats = [
    "MAV", "RMS", "WL", "ZC", "SSC", "VAR", "WAMP", "IEMG", "LogDet",
    "Skew", "Kurt", "HjMob", "HjComp", "Myopulse", "MNF", "MDF", "Power",
    "SpecEntropy", "PeakFreq", "BW"
]
for feat in emg_feats:
    FEATURE_CATEGORIES[f"EMG_{feat}"] = lambda k, f=feat: k.endswith(f"_{f}") and "_EMG_" in k

# 13 IMU feature categories
imu_feats = [
    "Mean", "Var", "Std", "Max", "SMA", "P2P", "Skew", "Kurt", "Jerk",
    "DomFreq", "SpecEnergy", "MNF", "SVM_Mean"
]
for feat in imu_feats:
    FEATURE_CATEGORIES[f"IMU_{feat}"] = lambda k, f=feat: k.endswith(f"_{f}") and ("_IMU_" in k or "_SVM_" in k)

ALL_CATEGORIES = sorted(list(FEATURE_CATEGORIES.keys()))


def filter_features(X_df, active_categories):
    """Filters sequence dictionary keys dynamically based on active categories."""
    matchers = [FEATURE_CATEGORIES[cat] for cat in active_categories]

    new_seq_dicts = []
    for seq in X_df['sequence_dicts']:
        new_seq = []
        for w in seq:
            new_w = {k: v for k, v in w.items() if any(m(k) for m in matchers)}
            new_seq.append(new_w)
        new_seq_dicts.append(new_seq)
        
    X_filtered = X_df.copy()
    X_filtered['sequence_dicts'] = new_seq_dicts
    return X_filtered


def main():
    # ------------------------------------------------------------------
    # CLI & Dynamic Defaulting
    # ------------------------------------------------------------------
    # Detect default mode from config_model.py, fallback to model_arch if missing
    default_mode = "model_arch"
    try:
        default_mode = GRID_SEARCH_MODE
    except NameError:
        pass

    parser = argparse.ArgumentParser(
        description="Unified hyperparameter and feature selection sweep for Spatio-Temporal Transformer 3"
    )
    parser.add_argument("--mode", type=str, choices=["model_arch", "generalization", "features", "architecture", "regularization"], default=default_mode,
                        help="Which sweep mode to run (model_arch, generalization, features).")
    parser.add_argument("--n_iter", type=int, default=None,
                        help="Number of iterations (default: 12 in DEV, 150/250 in production).")
    parser.add_argument("--test_participants", type=int, default=4,
                        help="Number of participants to hold out for validation")
    args = parser.parse_args()

    # Map older string arguments to new ones for backward compatibility
    mode = args.mode
    if mode == "architecture":
        mode = "model_arch"
    elif mode == "regularization":
        mode = "generalization"

    # Set DEV_MODE-aware defaults for iteration count
    if args.n_iter is not None:
        n_iter = args.n_iter
    else:
        if DEV_MODE:
            n_iter = 10 if mode == "features" else 12
        else:
            n_iter = 60

    test_participants_count = args.test_participants
    sweep_epochs = DEV_EPOCHS if DEV_MODE else 200
    sweep_patience = 5 if DEV_MODE else 20

    # ------------------------------------------------------------------
    # Paths & Setup
    # ------------------------------------------------------------------
    base_dir = Path(__file__).resolve().parent.parent.parent.parent
    segments_dir = DATABASE_CONFIG['segments_dir']
    results_dir = base_dir / "model" / "model_results"

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = results_dir / f"sweep_st_transformer3_{mode}_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 65)
    print(f"  SPATIO-TEMPORAL TRANSFORMER 3 UNIFIED SWEEP")
    print(f"  Active Sweep Mode    : {mode.upper()}")
    print("=" * 65)
    print(f"  DEV_MODE             : {DEV_MODE}")
    print(f"  Configurations/Iters : {n_iter}")
    print(f"  Epochs per config    : {sweep_epochs}")
    print(f"  Early-stop patience  : {sweep_patience}")
    print(f"  Test participants    : {test_participants_count}")
    print(f"  Results directory    : {run_dir}")
    print("=" * 65)

    # ------------------------------------------------------------------
    # 1. Load Data (precomputed features — sequence mode)
    # ------------------------------------------------------------------
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

    # Re-attach subject for splitting
    subjects = df["subject"].values
    X["subject"] = subjects

    # ------------------------------------------------------------------
    # 2. Setup a fixed cross-participant or stratified split for fair evaluation
    # ------------------------------------------------------------------
    if CV_STRATEGY == "kfold":
        from sklearn.model_selection import StratifiedShuffleSplit
        sss = StratifiedShuffleSplit(n_splits=1, test_size=0.2, random_state=42)
        strat_labels = df["weight"].astype(str) if "weight" in df.columns else None
        train_idx, val_idx = next(sss.split(X, strat_labels))
    else:
        n_unique_subs = len(np.unique(subjects))
        test_ratio = min(0.3, max(0.1, test_participants_count / max(1, n_unique_subs)))
        gss = GroupShuffleSplit(n_splits=1, test_size=test_ratio, random_state=42)
        train_idx, val_idx = next(gss.split(X, y, groups=subjects))

    X_train_raw = X.iloc[train_idx].copy()
    y_train     = y.iloc[train_idx].copy()
    X_val_raw   = X.iloc[val_idx].copy()
    y_val       = y.iloc[val_idx].copy()

    val_subs = sorted(X_val_raw["subject"].unique()) if "subject" in X_val_raw.columns else []
    train_subs = sorted(X_train_raw["subject"].unique()) if "subject" in X_train_raw.columns else []

    if CV_STRATEGY == "kfold":
        print(f"\nStratified Split (Mixed-Participant):")
        print(f"  Training   ({len(X_train_raw)} segs)")
        print(f"  Validation ({len(X_val_raw)} segs)")
    else:
        print(f"\nParticipant Split:")
        print(f"  Training   ({len(train_subs)} subjects, {len(X_train_raw)} segs): {', '.join(train_subs)}")
        print(f"  Validation ({len(val_subs)} subjects, {len(X_val_raw)} segs): {', '.join(val_subs)}")

    # ------------------------------------------------------------------
    # 3. Mode Routing & Parameter Sweeps
    # ------------------------------------------------------------------
    results = []
    sweep_start = time.time()

    if mode == "features":
        # ──────────────────────────────────────────────────────────────
        # FEATURES MODE: Randomized Feature Toggles & Ridge Regression
        # ──────────────────────────────────────────────────────────────
        np.random.seed(42)
        sampled_configs = []
        while len(sampled_configs) < n_iter:
            # Toggle each category with 50% probability
            toggles = {cat: bool(np.random.rand() > 0.5) for cat in ALL_CATEGORIES}
            
            # Ensure at least 1 feature category is active
            if not any(toggles.values()):
                toggles[np.random.choice(ALL_CATEGORIES)] = True
                
            # Avoid duplicate configurations
            if toggles not in sampled_configs:
                sampled_configs.append(toggles)

        for idx, toggles in enumerate(sampled_configs, 1):
            active_categories = sorted([cat for cat, active in toggles.items() if active])
            
            print(f"\n{'='*65}")
            print(f"Iteration {idx}/{n_iter} [Elapsed: {(time.time() - sweep_start)/60:.1f}m]")
            print(f"Active Feature Categories ({len(active_categories)}/33):")
            print(f"  {', '.join(active_categories)}")
            print(f"{'='*65}")

            # Filter features dynamically
            X_train_filtered = filter_features(X_train_raw, active_categories)
            X_val_filtered   = filter_features(X_val_raw, active_categories)

            # Retain subject column to allow participant balancing during augmentation
            X_train = X_train_filtered.copy()
            X_val   = X_val_filtered.copy()

            # Count individual active features
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

            # Load baseline model config
            model_config = copy.deepcopy(SPATIO_TEMPORAL_TRANSFORMER3_CONFIG)
            model_config['epochs'] = sweep_epochs
            model_config['use_checkpointing'] = False  # Faster sweep execution

            # Disable early stopping if baseline is OneCycleLR or WarmupCosine
            sched_type = model_config.get('scheduler', {}).get('type', 'ReduceLROnPlateau')
            if sched_type in ['OneCycleLR', 'WarmupCosine']:
                model_config['early_stopping_patience'] = sweep_epochs
            else:
                model_config['early_stopping_patience'] = sweep_patience

            model = SpatioTemporalTransformerRegressor3(**model_config)

            iter_start = time.time()
            try:
                model.fit(X_train, y_train, X_val=X_val, y_val=y_val)

                # Evaluate
                preds = model.predict(X_val)
                preds = np.maximum(0.0, preds)  # Clamp negatives

                mae = mean_absolute_error(y_val, preds)
                mse = mean_squared_error(y_val, preds)
                rmse = np.sqrt(mse)
                r2 = r2_score(y_val, preds)

                elapsed = time.time() - iter_start
                total_params = sum(p.numel() for p in model.model.parameters())

                print(f"\n  Results for Features Iteration {idx}:")
                print(f"    RMSE: {rmse:.4f} | MAE: {mae:.4f} | R²: {r2:.4f}")
                print(f"    Parameters: {total_params:,} | Training Time: {elapsed:.1f}s")

                results.append({
                    "iteration": idx,
                    "toggles": toggles,
                    "active_categories": active_categories,
                    "n_categories": len(active_categories),
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
                print(f"  [ERROR] Iteration failed: {exc}. Skipping.")
                continue

    else:
        # ──────────────────────────────────────────────────────────────
        # MODEL_ARCH & GENERALIZATION MODES: Hyperparameter Search
        # ──────────────────────────────────────────────────────────────
        # Retain subject columns to allow participant balancing during augmentation
        X_train = X_train_raw.copy()
        X_val   = X_val_raw.copy()

        if mode == "model_arch":
            # Sweep core architecture holding regularization fixed
            param_grid = {
                'd_model':              [64, 96, 128, 160],
                'nhead_spatial':        [4, 8],
                'num_layers_spatial':   [2, 3, 4],
                'nhead_temporal':       [2, 4, 8],
                'num_layers_temporal':  [1, 2, 3, 4],
                'dim_feedforward':      [256, 512, 1024],
                'dropout_rate':         [0.25], 
                'weight_decay':         [0.005],
                'learning_rate':        [3e-4],
                'batch_size':           [64],
                'aug_p':                [0.5],
                'aug_noise_std':        [0.05],
                'aug_stretch_range':    [(0.75, 1.25)],
                'aug_channel_dropout':  [0.1],
                'scheduler_type':       ['OneCycleLR'],
                'scheduler_pct_start':  [0.3],
            }
        else:
            # Sweep regularization holding architecture fixed
            param_grid = {
                'd_model':              [96, 128],
                'nhead_spatial':        [8],
                'num_layers_spatial':   [4],
                'nhead_temporal':       [2],
                'num_layers_temporal':  [4],
                'dim_feedforward':      [512, 1024],
                'dropout_rate':         [0.2, 0.25, 0.35, 0.5], 
                'weight_decay':         [1e-05, 1e-04, 1e-03, 5e-03],
                'learning_rate':        [1e-4, 2e-4, 3e-4, 5e-4],
                'batch_size':           [64, 128],
                'aug_p':                [0.5, 0.8],
                'aug_noise_std':        [0.05, 0.1],
                'aug_stretch_range':    [(0.75, 1.25), (0.9, 1.1)],
                'aug_channel_dropout':  [0.1, 0.25],
                'scheduler_type':       ['OneCycleLR'],
                'scheduler_pct_start':  [0.1, 0.2, 0.3, 0.4, 0.5],
            }

        sampler = ParameterSampler(param_grid, n_iter=n_iter, random_state=42)

        for i, params in enumerate(sampler, 1):
            iter_start = time.time()
            elapsed = time.time() - sweep_start

            # Validate d_model / nhead compatibility
            d_model = params['d_model']
            nhead_s = params['nhead_spatial']
            nhead_t = params['nhead_temporal']
            while d_model % nhead_s != 0 and nhead_s > 1:
                nhead_s -= 1
            while d_model % nhead_t != 0 and nhead_t > 1:
                nhead_t -= 1
            params['nhead_spatial'] = nhead_s
            params['nhead_temporal'] = nhead_t

            # Extract scheduler & augmentation params
            scheduler_type = params.pop('scheduler_type', 'OneCycleLR')
            scheduler_pct_start = params.pop('scheduler_pct_start', 0.3)
            scheduler_patience = params.pop('scheduler_patience', 5)
            scheduler_factor = params.pop('scheduler_factor', 0.5)
            
            aug_cfg = copy.deepcopy(AUGMENTATION_CONFIG)
            aug_cfg['p'] = params.pop('aug_p', aug_cfg.get('p', 0.5))
            aug_cfg['channel_dropout_p'] = params.pop('aug_channel_dropout', aug_cfg.get('channel_dropout_p', 0.25))
            aug_cfg['noise_std'] = params.pop('aug_noise_std', aug_cfg.get('noise_std', 0.05))
            aug_cfg['stretch_factor_range'] = params.pop('aug_stretch_range', aug_cfg.get('stretch_factor_range', (0.85, 1.15)))

            # Override early stopping patience for OneCycleLR to let the schedule complete
            current_early_stopping_patience = sweep_epochs

            print(f"\n{'='*65}")
            print(f"Hyperparameter Iteration {i}/{n_iter} [Elapsed: {elapsed/60:.1f}m]")
            print(f"  d_model={d_model}, nhead_s={nhead_s}, nhead_t={nhead_t}, "
                  f"layers_s={params['num_layers_spatial']}, layers_t={params['num_layers_temporal']}")
            print(f"  ff={params['dim_feedforward']}, dropout={params['dropout_rate']}, wd={params['weight_decay']}")
            print(f"  scheduler={scheduler_type}, pct_start={scheduler_pct_start}, learning_rate={params['learning_rate']}")
            print(f"{'='*65}")

            try:
                model = SpatioTemporalTransformerRegressor3(
                    d_model=params['d_model'],
                    nhead_spatial=params['nhead_spatial'],
                    num_layers_spatial=params['num_layers_spatial'],
                    nhead_temporal=params['nhead_temporal'],
                    num_layers_temporal=params['num_layers_temporal'],
                    dim_feedforward=params['dim_feedforward'],
                    dropout_rate=params['dropout_rate'],
                    learning_rate=params['learning_rate'],
                    weight_decay=params['weight_decay'],
                    batch_size=params['batch_size'],
                    epochs=sweep_epochs,
                    early_stopping_patience=current_early_stopping_patience,
                    scheduler_patience=scheduler_patience,
                    scheduler_factor=scheduler_factor,
                    use_checkpointing=False,
                    use_amp=True,
                    scheduler={
                        'type': 'OneCycleLR',
                        'max_lr': params['learning_rate'],
                        'pct_start': scheduler_pct_start
                    },
                    augmentation_config=aug_cfg,
                    random_state=GLOBAL_RANDOM_STATE,
                )

                model.fit(X_train, y_train, X_val=X_val, y_val=y_val)

                # Evaluate
                preds = model.predict(X_val)
                preds = np.maximum(0.0, preds)

                mae = mean_absolute_error(y_val, preds)
                mse = mean_squared_error(y_val, preds)
                rmse = np.sqrt(mse)
                r2 = r2_score(y_val, preds)

                elapsed_iter = time.time() - iter_start
                total_params = sum(p.numel() for p in model.model.parameters())

                print(f"\n  Results for HP Iteration {i}:")
                print(f"    RMSE: {rmse:.4f} | MAE: {mae:.4f} | R²: {r2:.4f}")
                print(f"    Parameters: {total_params:,} | Training Time: {elapsed_iter:.1f}s")

                # Log all parameters including scheduler settings
                saved_params = copy.deepcopy(params)
                saved_params["scheduler_type"] = scheduler_type
                saved_params["scheduler_pct_start"] = scheduler_pct_start
                saved_params["scheduler_patience"] = scheduler_patience
                saved_params["scheduler_factor"] = scheduler_factor

                results.append({
                    "iteration": i,
                    "params": saved_params,
                    "metrics": {"RMSE": rmse, "MAE": mae, "R2": r2},
                    "rmse": rmse,
                    "mae": mae,
                    "r2": r2,
                    "total_params": total_params,
                    "training_time": elapsed_iter,
                    "epochs_trained": len(model.loss_history["train"]),
                })
            except Exception as exc:
                import traceback
                traceback.print_exc()
                print(f"  [ERROR] Iteration failed: {exc}. Skipping.")
                continue

    # ------------------------------------------------------------------
    # 4. Sweep Complete & Report Generation
    # ------------------------------------------------------------------
    total_sweep_time = time.time() - sweep_start
    print("\n" + "=" * 65)
    print("  SWEEP COMPLETELY FINISHED")
    print(f"  Total execution time: {total_sweep_time/60:.1f} minutes")
    print("=" * 65)

    if not results:
        print("All sweep configurations failed. No report saved.")
        return

    # Post-processing for "features" mode
    contribution_results = []
    ridge_cv_r2 = 0.0
    intercept = 0.0

    if mode == "features":
        print("\nFitting Ridge Regression to calculate feature contributions...")
        X_reg = []
        y_reg = []
        for res in results:
            row = [1.0 if res['toggles'][cat] else 0.0 for cat in ALL_CATEGORIES]
            X_reg.append(row)
            y_reg.append(res['r2'])

        X_reg = np.array(X_reg)
        y_reg = np.array(y_reg)

        if len(X_reg) >= 5:
            try:
                ridge = RidgeCV(alphas=[0.1, 1.0, 10.0, 100.0])
                ridge.fit(X_reg, y_reg)
                coefs = ridge.coef_
                intercept = ridge.intercept_
                ridge_cv_r2 = ridge.score(X_reg, y_reg)

                for cat, coef in zip(ALL_CATEGORIES, coefs):
                    contribution_results.append({
                        "category": cat,
                        "coef": coef,
                        "impact": "Positive (Improves Generalization)" if coef > 0 else "Negative (Reduces Generalization)"
                    })
                
                contribution_results.sort(key=lambda x: x['coef'], reverse=True)

                print("\nFeature Generalization Contributions (Ridge Coefficients on R²):")
                for rank, c_res in enumerate(contribution_results, 1):
                    print(f"  #{rank:2d}: {c_res['category']:15s} -> {c_res['coef']:+.5f} ({c_res['impact']})")
            except Exception as e:
                print(f"Ridge regression fitting failed: {e}")
        else:
            print("Not enough successful iterations (need >= 5) to fit Ridge Regression.")

    # Sort results by validation RMSE for the leaderboard
    results.sort(key=lambda r: r["rmse"])

    # Save CSV and MD Reports
    csv_file = run_dir / "sweep_results.csv"
    report_file = run_dir / "sweep_report.md"

    # Save CSV
    if mode == "features":
        rows = []
        for res in results:
            row = {
                "iteration": res["iteration"],
                "n_categories": res["n_categories"],
                "n_features": res["n_features"],
                "rmse": res["rmse"],
                "mae": res["mae"],
                "r2": res["r2"],
                "total_params": res["total_params"],
                "training_time": res["training_time"],
                "epochs_trained": res["epochs_trained"],
            }
            for cat in ALL_CATEGORIES:
                row[f"toggle_{cat}"] = 1 if res["toggles"][cat] else 0
            rows.append(row)
        pd.DataFrame(rows).to_csv(csv_file, index=False)
    else:
        rows = []
        for res in results:
            row = {
                "iteration": res["iteration"],
                "rmse": res["rmse"],
                "mae": res["mae"],
                "r2": res["r2"],
                "total_params": res["total_params"],
                "training_time": res["training_time"],
                "epochs_trained": res["epochs_trained"],
                **res["params"]
            }
            rows.append(row)
        pd.DataFrame(rows).to_csv(csv_file, index=False)

    # Save Markdown Report
    with open(report_file, "w") as f:
        f.write(f"# Spatio-Temporal Transformer 3 Consolidated Sweep Report ({mode.upper()})\n\n")
        f.write(f"**Date:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        f.write(f"**DEV_MODE:** {DEV_MODE}\n\n")
        f.write(f"**Total Execution Time:** {total_sweep_time/60:.1f} minutes\n\n")
        f.write(f"**Cross-Participant Validation Group:** held out subjects `{', '.join(val_subs)}` for testing.\n\n")

        if mode == "features" and contribution_results:
            f.write("## 🎯 Ridge Regression Feature Contribution Analysis\n\n")
            f.write("Fits a Ridge regression model mapping feature category binary indicators (0 or 1) to validation $R^2$ scores across all iterations. ")
            f.write("A **positive coefficient** means that including this feature group on average **increases** $R^2$ (improves generalization), while a **negative coefficient** indicates it degrades generalization or adds noise.\n\n")
            f.write(f"* **Ridge Regression Fit $R^2$ Score:** {ridge_cv_r2:.4f}\n")
            f.write(f"* **Baseline Intercept (Average $R^2$):** {intercept:.4f}\n\n")
            f.write("| Rank | Feature Category | Ridge Coefficient (Delta R²) | Impact on Generalization |\n")
            f.write("| :---: | :--- | :---: | :--- |\n")
            for rank, c_res in enumerate(contribution_results, 1):
                impact_emoji = "✅" if c_res['coef'] > 0 else "❌"
                f.write(f"| {rank} | `{c_res['category']}` | **{c_res['coef']:+.5f}** | {impact_emoji} {c_res['impact']} |\n")
            f.write("\n")

        f.write("## 🏆 Ranked Leaderboard (Sorted by RMSE)\n\n")
        if mode == "features":
            f.write("| Rank | Iteration | Active Categories | Features | Val RMSE | Val MAE | Val R² | Model Params | Training Time | Epochs |\n")
            f.write("| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n")
            for rank, res in enumerate(results, 1):
                categories_str = ", ".join(res["active_categories"])
                if len(categories_str) > 40:
                    categories_str = categories_str[:37] + "..."
                f.write(f"| {rank} | #{res['iteration']} | {res['n_categories']} (`{categories_str}`) | {res['n_features']} | **{res['rmse']:.4f}** | {res['mae']:.4f} | {res['r2']:.4f} | {res['total_params']:,} | {res['training_time']:.1f}s | {res['epochs_trained']} |\n")
        else:
            f.write("| Rank | Iter | Parameters | Val RMSE | Val MAE | Val R² | Model Params | Training Time | Epochs |\n")
            f.write("| :---: | :---: | :--- | :---: | :---: | :---: | :---: | :---: | :---: |\n")
            for rank, res in enumerate(results, 1):
                p = res["params"]
                p_desc = f"d={p['d_model']}, layers_s={p['num_layers_spatial']}, layers_t={p['num_layers_temporal']}, ff={p['dim_feedforward']}, lr={p['learning_rate']}, wd={p['weight_decay']}, sched_pct={p.get('scheduler_pct_start', 'N/A')}"
                f.write(f"| {rank} | #{res['iteration']} | `{p_desc}` | **{res['rmse']:.4f}** | {res['mae']:.4f} | {res['r2']:.4f} | {res['total_params']:,} | {res['training_time']:.1f}s | {res['epochs_trained']} |\n")

    print(f"\nConsolidated sweep report saved to: {report_file}")
    print(f"Consolidated sweep CSV results saved to: {csv_file}")


if __name__ == "__main__":
    main()
