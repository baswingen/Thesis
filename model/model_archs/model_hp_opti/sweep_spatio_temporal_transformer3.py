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
import json
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


def save_partial_csv(results, run_dir, mode):
    """Persist results after every iteration so a wall-time kill loses at most
    the iteration in flight (final report still overwrites this at sweep end)."""
    rows = []
    for res in results:
        if mode == "features":
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
        else:
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
    pd.DataFrame(rows).to_csv(run_dir / "sweep_results_partial.csv", index=False)


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
    parser.add_argument("--mode", type=str, choices=["model_arch", "generalization", "features", "architecture", "regularization", "full"], default=default_mode,
                        help="Which sweep mode to run (model_arch, generalization, features, "
                             "or 'full' for the budgeted feature+HP+OneCycleLR pipeline).")
    parser.add_argument("--time_budget_hours", type=float, default=23.0,
                        help="Wall-clock budget for --mode full; the sweep stops scheduling new "
                             "fits when the remaining budget cannot fit the next one (default 23h, "
                             "leaving margin under the 24h SLURM limit).")
    parser.add_argument("--n_iter", type=int, default=None,
                        help="Number of iterations (default: 12 in DEV, 150/250 in production).")
    parser.add_argument("--test_participants", type=int, default=4,
                        help="Number of participants to hold out for validation")
    parser.add_argument("--start_iter", type=int, default=1,
                        help="Resume from this iteration (1-based). Configs are sampled with a "
                             "fixed seed, so earlier iterations are reproduced and skipped.")
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
            if idx < args.start_iter:
                continue
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
                save_partial_csv(results, run_dir, mode)
            except Exception as exc:
                import traceback
                traceback.print_exc()
                print(f"  [ERROR] Iteration failed: {exc}. Skipping.")
                continue

    elif mode == "full":
        # ──────────────────────────────────────────────────────────────
        # FULL MODE: feature set + HP/regularization/OneCycleLR in one
        # wall-clock-budgeted job.
        #
        # Stage A re-evaluates the four strongest feature sets from the
        # recovered features sweep (job 10122647) at the 200-epoch
        # baseline; differences there were within noise (R² 0.959–0.971
        # across 51 random sets), so four targeted candidates replace a
        # full re-randomization. Stage B runs successive halving on the
        # winning set: OneCycleLR compresses cleanly because total_steps
        # = epochs × batches, so a 60-epoch schedule is a complete cycle
        # usable as a cheap fidelity. A wall-clock guard (measured
        # sec/epoch) stops scheduling fits that cannot finish in budget.
        # ──────────────────────────────────────────────────────────────
        budget_s = args.time_budget_hours * 3600.0
        finalize_margin_s = 20 * 60.0
        timing = {"sec_per_epoch": 9.0, "overhead_s": 180.0}  # prior from job 10122647 (~7.8–8.5 s/epoch)

        def est_fit_s(n_epochs):
            return n_epochs * timing["sec_per_epoch"] + timing["overhead_s"]

        def budget_left_s():
            return budget_s - (time.time() - sweep_start)

        def save_full_partial():
            rows = []
            for r in results:
                rows.append({
                    "stage": r["stage"],
                    "tag": r["tag"],
                    "feature_set": r.get("feature_set", ""),
                    "config_id": r.get("config_id", ""),
                    "fidelity_epochs": r["fidelity_epochs"],
                    "rmse": r["rmse"],
                    "mae": r["mae"],
                    "r2": r["r2"],
                    "n_features": r.get("n_features", ""),
                    "total_params": r["total_params"],
                    "training_time": r["training_time"],
                    "epochs_trained": r["epochs_trained"],
                    "params_json": json.dumps(r.get("params", {}), default=str),
                })
            pd.DataFrame(rows).to_csv(run_dir / "sweep_results_partial.csv", index=False)

        def run_config(stage, tag, X_tr, X_v, model_kwargs, n_epochs, extra=None):
            kwargs = copy.deepcopy(model_kwargs)
            kwargs["epochs"] = n_epochs
            kwargs["use_checkpointing"] = False
            t0 = time.time()
            print(f"\n{'='*65}")
            print(f"[{stage}] {tag} | epochs={n_epochs} | budget left: {budget_left_s()/3600:.2f}h")
            print(f"{'='*65}")
            try:
                mdl = SpatioTemporalTransformerRegressor3(**kwargs)
                mdl.fit(X_tr, y_train, X_val=X_v, y_val=y_val)
                preds = np.maximum(0.0, mdl.predict(X_v))
                rmse = float(np.sqrt(mean_squared_error(y_val, preds)))
                mae = float(mean_absolute_error(y_val, preds))
                r2 = float(r2_score(y_val, preds))
                iter_s = time.time() - t0
                epochs_trained = len(mdl.loss_history["train"])
                spe = max(0.5, (iter_s - timing["overhead_s"]) / max(1, epochs_trained))
                timing["sec_per_epoch"] = 0.5 * timing["sec_per_epoch"] + 0.5 * spe
                res = {
                    "stage": stage,
                    "tag": tag,
                    "fidelity_epochs": n_epochs,
                    "rmse": rmse,
                    "mae": mae,
                    "r2": r2,
                    "total_params": sum(p.numel() for p in mdl.model.parameters()),
                    "training_time": iter_s,
                    "epochs_trained": epochs_trained,
                    **(extra or {}),
                }
                results.append(res)
                save_full_partial()
                print(f"\n  [{stage}] {tag}: RMSE {rmse:.4f} | MAE {mae:.4f} | R² {r2:.4f} "
                      f"({iter_s/60:.1f} min, sec/epoch now {timing['sec_per_epoch']:.1f})")
                return res
            except Exception as exc:
                import traceback
                traceback.print_exc()
                print(f"  [ERROR] [{stage}] {tag} failed: {exc}. Skipping.")
                return None

        # ── Stage A: feature set confirmation ────────────────────────
        # Candidates derived from the recovered features sweep
        # (model_results/sweep_st_transformer3_features_20260609_223220_RECOVERED):
        # the two best single runs, the ridge-positive categories, and
        # the all-categories baseline.
        candidate_feature_sets = {
            "all_33": list(ALL_CATEGORIES),
            "sweep_best_rmse_20": [
                "EMG_BW", "EMG_HjComp", "EMG_IEMG", "EMG_MAV", "EMG_MNF",
                "EMG_Power", "EMG_RMS", "EMG_SSC", "EMG_Skew", "EMG_SpecEntropy",
                "EMG_VAR", "EMG_WAMP", "IMU_Jerk", "IMU_Kurt", "IMU_Max",
                "IMU_Mean", "IMU_SMA", "IMU_SVM_Mean", "IMU_SpecEnergy", "IMU_Var",
            ],
            "sweep_lean_14": [
                "EMG_BW", "EMG_IEMG", "EMG_Kurt", "EMG_LogDet", "EMG_MAV",
                "EMG_Myopulse", "EMG_WL", "IMU_Jerk", "IMU_MNF", "IMU_Max",
                "IMU_SMA", "IMU_SVM_Mean", "IMU_SpecEnergy", "IMU_Var",
            ],
            "ridge_positive_17": [
                "EMG_BW", "EMG_HjComp", "EMG_IEMG", "EMG_LogDet", "EMG_MAV",
                "EMG_MNF", "EMG_Power", "EMG_RMS", "EMG_Skew", "EMG_VAR",
                "EMG_WL", "IMU_Mean", "IMU_P2P", "IMU_SMA", "IMU_SpecEnergy",
                "IMU_Std", "IMU_Var",
            ],
        }

        if DEV_MODE:
            stage_a_epochs = DEV_EPOCHS
            rung_epochs = [2, 3, max(4, DEV_EPOCHS)]
            rung1_width = args.n_iter if args.n_iter is not None else 6
        else:
            stage_a_epochs = 200
            rung_epochs = [60, 120, 200]
            rung1_width = args.n_iter if args.n_iter is not None else 48
        rung_survivors = [12, 4]

        base_cfg = copy.deepcopy(SPATIO_TEMPORAL_TRANSFORMER3_CONFIG)
        base_sched = base_cfg.get("scheduler", {}).get("type", "ReduceLROnPlateau")
        base_cfg["early_stopping_patience"] = (
            stage_a_epochs if base_sched in ["OneCycleLR", "WarmupCosine"] else sweep_patience
        )

        stage_a_results = []
        filtered_cache = {}
        for set_name, cats in candidate_feature_sets.items():
            if budget_left_s() < est_fit_s(stage_a_epochs) + finalize_margin_s:
                print(f"[Budget] Skipping remaining Stage A sets (left: {budget_left_s()/60:.0f}m).")
                break
            X_tr = filter_features(X_train_raw, sorted(cats)).copy()
            X_v = filter_features(X_val_raw, sorted(cats)).copy()
            filtered_cache[set_name] = (X_tr, X_v)
            n_features = 0
            for seq in X_tr["sequence_dicts"]:
                if seq and seq[0]:
                    n_features = len(seq[0])
                    break
            res = run_config(
                "Stage A", set_name, X_tr, X_v, base_cfg, stage_a_epochs,
                extra={"feature_set": set_name, "n_categories": len(cats), "n_features": n_features},
            )
            if res:
                stage_a_results.append(res)

        # Winner: leanest set within 0.010 RMSE of the best — single runs
        # cannot resolve smaller differences, so prefer fewer features.
        if stage_a_results:
            best_a_rmse = min(r["rmse"] for r in stage_a_results)
            eligible = [r for r in stage_a_results if r["rmse"] <= best_a_rmse + 0.010]
            winner = min(eligible, key=lambda r: r["n_features"])
            winner_name = winner["feature_set"]
        else:
            winner_name = "sweep_best_rmse_20"
            if winner_name not in filtered_cache:
                cats = candidate_feature_sets[winner_name]
                filtered_cache[winner_name] = (
                    filter_features(X_train_raw, sorted(cats)).copy(),
                    filter_features(X_val_raw, sorted(cats)).copy(),
                )
        print(f"\n[Stage A] Winning feature set: {winner_name} "
              f"({len(candidate_feature_sets[winner_name])} categories)")
        X_tr_b, X_v_b = filtered_cache[winner_name]

        # ── Stage B: successive halving over arch/regularization/OneCycle ──
        # Architecture dims are searched in a neighborhood of the best
        # model_arch-sweep config (d=96-128, 4 spatial / 4 temporal layers,
        # nhead 8/2) rather than the full blind grid.
        param_grid = {
            'd_model':              [96, 128, 160],
            'nhead_spatial':        [4, 8],
            'num_layers_spatial':   [3, 4],
            'nhead_temporal':       [2, 4],
            'num_layers_temporal':  [3, 4],
            'dim_feedforward':      [512, 1024],
            'dropout_rate':         [0.2, 0.25, 0.35, 0.5],
            'weight_decay':         [1e-05, 1e-04, 1e-03, 5e-03],
            'learning_rate':        [1e-4, 2e-4, 3e-4, 5e-4, 8e-4],
            'batch_size':           [64, 128],
            'aug_p':                [0.5, 0.8],
            'aug_noise_std':        [0.05, 0.1],
            'aug_stretch_range':    [(0.75, 1.25), (0.9, 1.1)],
            'aug_channel_dropout':  [0.1, 0.25],
            'scheduler_pct_start':  [0.05, 0.1, 0.2, 0.3, 0.4],
        }
        sampled = list(ParameterSampler(param_grid, n_iter=rung1_width, random_state=42))
        current = [{"config_id": i, "params": p} for i, p in enumerate(sampled, 1)]

        rung_leaderboards = []
        for rung_idx, n_epochs in enumerate(rung_epochs, 1):
            scored = []
            for cfg in current:
                if budget_left_s() < est_fit_s(n_epochs) + finalize_margin_s:
                    print(f"[Budget] Stopping rung {rung_idx} after {len(scored)} of "
                          f"{len(current)} configs (left: {budget_left_s()/60:.0f}m).")
                    break
                params = copy.deepcopy(cfg["params"])

                d_model = params['d_model']
                nhead_s = params['nhead_spatial']
                nhead_t = params['nhead_temporal']
                while d_model % nhead_s != 0 and nhead_s > 1:
                    nhead_s -= 1
                while d_model % nhead_t != 0 and nhead_t > 1:
                    nhead_t -= 1

                pct_start = params.pop('scheduler_pct_start')
                aug_cfg = copy.deepcopy(AUGMENTATION_CONFIG)
                aug_cfg['p'] = params.pop('aug_p')
                aug_cfg['channel_dropout_p'] = params.pop('aug_channel_dropout')
                aug_cfg['noise_std'] = params.pop('aug_noise_std')
                aug_cfg['stretch_factor_range'] = params.pop('aug_stretch_range')

                model_kwargs = dict(
                    d_model=d_model,
                    nhead_spatial=nhead_s,
                    num_layers_spatial=params['num_layers_spatial'],
                    nhead_temporal=nhead_t,
                    num_layers_temporal=params['num_layers_temporal'],
                    dim_feedforward=params['dim_feedforward'],
                    dropout_rate=params['dropout_rate'],
                    learning_rate=params['learning_rate'],
                    weight_decay=params['weight_decay'],
                    batch_size=params['batch_size'],
                    early_stopping_patience=n_epochs,  # let the OneCycle schedule complete
                    use_amp=True,
                    scheduler={
                        'type': 'OneCycleLR',
                        'max_lr': params['learning_rate'],
                        'pct_start': pct_start,
                    },
                    augmentation_config=aug_cfg,
                    random_state=GLOBAL_RANDOM_STATE,
                )
                res = run_config(
                    f"Stage B rung {rung_idx}", f"config #{cfg['config_id']}",
                    X_tr_b, X_v_b, model_kwargs, n_epochs,
                    extra={"config_id": cfg["config_id"], "feature_set": winner_name,
                           "params": cfg["params"]},
                )
                if res:
                    scored.append((res["rmse"], cfg))

            scored.sort(key=lambda t: t[0])
            rung_leaderboards.append((rung_idx, n_epochs, list(scored)))
            if not scored:
                print(f"[Stage B] Rung {rung_idx} produced no results; stopping.")
                break
            if rung_idx == len(rung_epochs):
                break
            k = rung_survivors[rung_idx - 1]
            next_epochs = rung_epochs[rung_idx]
            affordable = int(max(0, (budget_left_s() - finalize_margin_s) // est_fit_s(next_epochs)))
            k = min(k, len(scored), affordable)
            if k == 0:
                print(f"[Budget] No budget for rung {rung_idx + 1}; finalizing with rung {rung_idx} results.")
                break
            current = [c for _, c in scored[:k]]
            print(f"\n[Stage B] Promoting {k} configs to rung {rung_idx + 1} ({next_epochs} epochs): "
                  f"{', '.join('#' + str(c['config_id']) for c in current)}")

        # ── Final report ──────────────────────────────────────────────
        # Best config = lowest RMSE at the highest fidelity that ran.
        b_results = [r for r in results if r["stage"].startswith("Stage B")]
        best = None
        if b_results:
            top_fidelity = max(r["fidelity_epochs"] for r in b_results)
            best = min((r for r in b_results if r["fidelity_epochs"] == top_fidelity),
                       key=lambda r: r["rmse"])

        total_sweep_time = time.time() - sweep_start
        print("\n" + "=" * 65)
        print("  FULL SWEEP FINISHED")
        print(f"  Total execution time: {total_sweep_time/60:.1f} minutes "
              f"(budget: {args.time_budget_hours:.1f}h)")
        print("=" * 65)

        if best:
            best_dump = {
                "feature_set": winner_name,
                "feature_categories": sorted(candidate_feature_sets[winner_name]),
                "params": best["params"],
                "fidelity_epochs": best["fidelity_epochs"],
                "metrics": {"rmse": best["rmse"], "mae": best["mae"], "r2": best["r2"]},
            }
            with open(run_dir / "best_config.json", "w") as f:
                json.dump(best_dump, f, indent=2, default=str)
            print(f"\nBest config ({best['tag']} @ {best['fidelity_epochs']} epochs): "
                  f"RMSE {best['rmse']:.4f} | MAE {best['mae']:.4f} | R² {best['r2']:.4f}")
            print(json.dumps(best_dump, indent=2, default=str))

        report_file = run_dir / "sweep_report.md"
        with open(report_file, "w") as f:
            f.write("# Spatio-Temporal Transformer 3 Full Sweep Report (FEATURES + HP + OneCycleLR)\n\n")
            f.write(f"**Date:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            f.write(f"**DEV_MODE:** {DEV_MODE}\n\n")
            f.write(f"**Total Execution Time:** {total_sweep_time/60:.1f} minutes "
                    f"(budget {args.time_budget_hours:.1f}h)\n\n")
            f.write("## Stage A — Feature Set Confirmation\n\n")
            f.write("| Feature Set | Categories | Features | Val RMSE | Val MAE | Val R² |\n")
            f.write("| :--- | :---: | :---: | :---: | :---: | :---: |\n")
            for r in sorted(stage_a_results, key=lambda r: r["rmse"]):
                marker = " **(winner)**" if r["feature_set"] == winner_name else ""
                f.write(f"| `{r['feature_set']}`{marker} | {r['n_categories']} | {r['n_features']} | "
                        f"**{r['rmse']:.4f}** | {r['mae']:.4f} | {r['r2']:.4f} |\n")
            f.write(f"\nWinner: `{winner_name}` (leanest set within 0.010 RMSE of the best).\n\n")
            f.write("## Stage B — Successive Halving (OneCycleLR fidelities)\n\n")
            for rung_idx, n_epochs, scored in rung_leaderboards:
                f.write(f"### Rung {rung_idx} — {n_epochs}-epoch schedules ({len(scored)} configs)\n\n")
                f.write("| Rank | Config | Val RMSE | Parameters |\n")
                f.write("| :---: | :---: | :---: | :--- |\n")
                for rank, (rmse_v, cfg) in enumerate(scored, 1):
                    p = cfg["params"]
                    p_desc = (f"d={p['d_model']}, ls={p['num_layers_spatial']}, lt={p['num_layers_temporal']}, "
                              f"nh={p['nhead_spatial']}/{p['nhead_temporal']}, ff={p['dim_feedforward']}, "
                              f"do={p['dropout_rate']}, wd={p['weight_decay']}, lr={p['learning_rate']}, "
                              f"bs={p['batch_size']}, pct={p['scheduler_pct_start']}, aug_p={p['aug_p']}")
                    f.write(f"| {rank} | #{cfg['config_id']} | **{rmse_v:.4f}** | `{p_desc}` |\n")
                f.write("\n")
            if best:
                f.write("## 🏆 Best Configuration\n\n")
                f.write("```json\n")
                f.write(json.dumps(best_dump, indent=2, default=str))
                f.write("\n```\n")

        print(f"\nFull sweep report saved to: {report_file}")
        print(f"Per-fit results saved to: {run_dir / 'sweep_results_partial.csv'}")
        if best:
            print(f"Best config JSON saved to: {run_dir / 'best_config.json'}")
        return

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
            if i < args.start_iter:
                continue
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
                save_partial_csv(results, run_dir, mode)
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
