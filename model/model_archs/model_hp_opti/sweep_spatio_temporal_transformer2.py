"""
Spatio-Temporal Transformer 2 Hyperparameter Sweep
==================================================
Runs a randomised hyperparameter search for the Spatio-Temporal
Transformer 2 with a *single* fixed cross-participant holdout split.

The sweep is DEV_MODE-aware:
  - DEV_MODE ON  → small n_iter, subsampled data, short epochs
  - DEV_MODE OFF → full dataset, higher iteration budget

Strategy
--------
1. Hold out ~4 random participants as a fixed validation set.
2. Optionally sub-sample training data in DEV_MODE.
3. Train each config with early stopping.
4. Rank all configs by validation RMSE and save a detailed report.

Usage
-----
    # Triggered from run_model.py when RUN_GRID_SEARCH = True and
    # MODEL_TYPE = "spatio_temporal_transformer2"
    #
    # Or run standalone:
    python sweep_spatio_temporal_transformer2.py
    python sweep_spatio_temporal_transformer2.py --n_iter 5   # quick test
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

# Add project root to sys.path so 'model' package can be found
sys.path.append(str(Path(__file__).resolve().parent.parent.parent.parent))

from model.data_loader import DataLoader, _EXCLUDED_TRUE_WEIGHTS
from model.model_archs.spatio_temporal_transformer2 import SpatioTemporalTransformerRegressor2
from model.config_model import (
    SPATIO_TEMPORAL_TRANSFORMER_CONFIG, FEATURE_CONFIG,
    DATABASE_CONFIG, AUGMENTATION_CONFIG,
    DEV_MODE, DEV_FRACTION, DEV_EPOCHS, DEV_EARLY_STOPPING_PATIENCE,
    GLOBAL_RANDOM_STATE,
)


def main():
    # ------------------------------------------------------------------
    # CLI
    # ------------------------------------------------------------------
    parser = argparse.ArgumentParser(
        description="Spatio-Temporal Transformer 2 hyperparameter sweep"
    )
    parser.add_argument("--n_iter", type=int, default=None,
                        help="Number of random configurations (default: 8 in DEV, 40 otherwise)")
    parser.add_argument("--test_participants", type=int, default=4,
                        help="Number of participants to hold out for validation")
    parser.add_argument("--mode", type=str, choices=["architecture", "regularization"], default="architecture",
                        help="Which hyperparameters to sweep (architecture vs. regularization).")
    args = parser.parse_args()

    # DEV_MODE-aware defaults
    if args.n_iter is not None:
        n_iter = args.n_iter
    else:
        n_iter = 12 if DEV_MODE else 100

    test_participants_count = args.test_participants
    sweep_epochs = DEV_EPOCHS if DEV_MODE else 100
    sweep_patience = 8 if DEV_MODE else 30

    # ------------------------------------------------------------------
    # Paths
    # ------------------------------------------------------------------
    base_dir = Path(__file__).resolve().parent.parent.parent.parent
    segments_dir = DATABASE_CONFIG['segments_dir']
    results_dir = base_dir / "model" / "model_results"

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = results_dir / f"sweep_spatio_temporal_transformer2_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # Parameter grid — BROAD selection for full dataset
    # ------------------------------------------------------------------
    if args.mode == "regularization":
        param_grid = {
            # ── Fixed Architecture ──────────────────────────────
            'd_model':              [SPATIO_TEMPORAL_TRANSFORMER_CONFIG['d_model']],
            'nhead_spatial':        [SPATIO_TEMPORAL_TRANSFORMER_CONFIG['nhead_spatial']],
            'num_layers_spatial':   [SPATIO_TEMPORAL_TRANSFORMER_CONFIG['num_layers_spatial']],
            'nhead_temporal':       [SPATIO_TEMPORAL_TRANSFORMER_CONFIG['nhead_temporal']],
            'num_layers_temporal':  [SPATIO_TEMPORAL_TRANSFORMER_CONFIG['num_layers_temporal']],
            'dim_feedforward':      [SPATIO_TEMPORAL_TRANSFORMER_CONFIG['dim_feedforward']],

            # ── Regularisation (Network) ───────────
            'dropout_rate':         [0.2, 0.3, 0.4, 0.5],
            'weight_decay':         [1e-5, 1e-4, 1e-3, 1e-2],

            # ── Regularisation (Augmentation) ──────
            'aug_p':                [0.5, 0.8, 1.0],
            'aug_channel_dropout':  [0.1, 0.25, 0.4],
            'aug_mixup_alpha':      [0.2, 0.5, 0.8],

            # ── Training (Fixed) ────────────────────────────────────────────────
            'learning_rate':        [SPATIO_TEMPORAL_TRANSFORMER_CONFIG['learning_rate']],
            'batch_size':           [SPATIO_TEMPORAL_TRANSFORMER_CONFIG['batch_size']],
            'scheduler_type':       [SPATIO_TEMPORAL_TRANSFORMER_CONFIG.get('scheduler', {}).get('type', 'ReduceLROnPlateau')],
            'pct_start':            [0.1],     
        }
    else:
        param_grid = {
            # ── Architecture (Broad Search) ─────────────────────────────
            'd_model':              [64, 96, 128, 160, 192, 256],
            'nhead_spatial':        [2, 4, 8],
            'num_layers_spatial':   [1, 2, 3, 4],
            'nhead_temporal':       [2, 4, 8],
            'num_layers_temporal':  [1, 2, 3],
            'dim_feedforward':      [128, 256, 512, 1024],

            # ── Regularisation (Capture all possible configs) ───────────
            'dropout_rate':         [0.1, 0.15, 0.2, 0.3, 0.4, 0.5],
            'weight_decay':         [1e-6, 1e-5, 5e-5, 1e-4, 5e-4, 1e-3],

            # ── Training ────────────────────────────────────────────────
            'learning_rate':        [1e-4, 3e-4, 5e-4, 1e-3, 2e-3],
            'batch_size':           [32, 64, 128],
            'scheduler_type':       ['ReduceLROnPlateau'],
            'scheduler_patience':   [3, 5, 8],
            'scheduler_factor':     [0.1, 0.5],
        }

    print("=" * 65)
    print(f"  SPATIO-TEMPORAL TRANSFORMER 2 SWEEP ({args.mode.upper()})")
    print("=" * 65)
    print(f"  DEV_MODE             : {DEV_MODE}")
    print(f"  Configurations       : {n_iter}")
    print(f"  Epochs per config    : {sweep_epochs}")
    print(f"  Early-stop patience  : {sweep_patience}")
    print(f"  Test participants    : {test_participants_count}")
    print(f"  Results directory    : {run_dir}")
    print("=" * 65)

    # ------------------------------------------------------------------
    # 1. Load Data (precomputed features — sequence mode)
    # ------------------------------------------------------------------
    h5_paths = sorted([p for p in segments_dir.glob("*.h5")
                       if not p.name.startswith("._")])
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

    # --- Weight-class filter ---
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

    # Re-attach subject for splitting (prepare_for_ml drops meta-columns)
    subjects = df["subject"].values
    X["subject"] = subjects

    # ------------------------------------------------------------------
    # 2. Cross-Participant Split
    # ------------------------------------------------------------------
    n_unique_subs = len(np.unique(subjects))
    test_ratio = min(0.3, max(0.1, test_participants_count / max(1, n_unique_subs)))

    gss = GroupShuffleSplit(n_splits=1, test_size=test_ratio, random_state=42)
    train_idx, val_idx = next(gss.split(X, y, groups=subjects))

    X_train = X.iloc[train_idx].copy()
    y_train = y.iloc[train_idx].copy()
    X_val   = X.iloc[val_idx].copy()
    y_val   = y.iloc[val_idx].copy()

    val_subs = sorted(X_val["subject"].unique())
    train_subs = sorted(X_train["subject"].unique())

    # Drop subject column now that splitting is done
    X_train = X_train.drop(columns=["subject"])
    X_val   = X_val.drop(columns=["subject"])

    print(f"\nParticipant split:")
    print(f"  Training   ({len(train_subs)} subjects, {len(X_train)} segs): {', '.join(train_subs)}")
    print(f"  Validation ({len(val_subs)} subjects, {len(X_val)} segs): {', '.join(val_subs)}")

    # ------------------------------------------------------------------
    # 3. Randomised Sweep
    # ------------------------------------------------------------------
    sampler = ParameterSampler(param_grid, n_iter=n_iter, random_state=42)

    best_rmse = float('inf')
    best_params = None
    best_report = ""
    results = []
    sweep_start = time.time()

    for i, params in enumerate(sampler, 1):
        iter_start = time.time()

        # ── Validate d_model / nhead compatibility ──
        d_model = params['d_model']
        nhead_s = params['nhead_spatial']
        nhead_t = params['nhead_temporal']

        # Adjust nheads if they don't divide d_model evenly
        while d_model % nhead_s != 0 and nhead_s > 1:
            nhead_s -= 1
        while d_model % nhead_t != 0 and nhead_t > 1:
            nhead_t -= 1
        params['nhead_spatial'] = nhead_s
        params['nhead_temporal'] = nhead_t

        # Extract scheduler params
        scheduler_type = params.pop('scheduler_type')
        pct_start = params.pop('pct_start', 0.1)
        scheduler_patience = params.pop('scheduler_patience', 5)
        scheduler_factor = params.pop('scheduler_factor', 0.5)
        
        # Extract augmentation params (if sweeping)
        aug_cfg = copy.deepcopy(AUGMENTATION_CONFIG)
        aug_p = params.pop('aug_p', aug_cfg.get('p', 0.5))
        aug_channel_dropout = params.pop('aug_channel_dropout', aug_cfg.get('channel_dropout_p', 0.25))
        aug_mixup_alpha = params.pop('aug_mixup_alpha', aug_cfg.get('mixup_alpha', 0.5))
        
        aug_cfg['p'] = aug_p
        aug_cfg['channel_dropout_p'] = aug_channel_dropout
        aug_cfg['mixup_alpha'] = aug_mixup_alpha

        elapsed = time.time() - sweep_start
        print(f"\n{'='*65}")
        print(f"--- Iteration {i}/{n_iter} [Elapsed: {elapsed/60:.1f}m] ---")
        print(f"  d_model={d_model}, nhead_s={nhead_s}, nhead_t={nhead_t}, "
              f"layers_s={params['num_layers_spatial']}, layers_t={params['num_layers_temporal']}")
        print(f"  ff={params['dim_feedforward']}, dropout={params['dropout_rate']}, "
              f"wd={params['weight_decay']}")
        print(f"  lr={params['learning_rate']}, bs={params['batch_size']}, "
              f"scheduler={scheduler_type}")
        if args.mode == 'regularization':
            print(f"  aug_p={aug_p}, aug_channel_dropout={aug_channel_dropout}, aug_mixup_alpha={aug_mixup_alpha}")

        try:
            # Build scheduler config dict
            if scheduler_type == 'OneCycleLR':
                sched_cfg = {
                    'type': 'OneCycleLR',
                    'max_lr': params['learning_rate'],
                    'pct_start': pct_start,
                }
            else:
                sched_cfg = {'type': 'ReduceLROnPlateau'}

            model = SpatioTemporalTransformerRegressor2(
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
                early_stopping_patience=sweep_patience,
                scheduler_patience=scheduler_patience,
                scheduler_factor=scheduler_factor,
                use_checkpointing=False,   # Speed over memory for sweep
                use_amp=True,
                scheduler=sched_cfg,
                augmentation_config=aug_cfg,
                random_state=GLOBAL_RANDOM_STATE,
            )

            # Train with explicit cross-participant validation set
            model.fit(X_train, y_train, X_val=X_val, y_val=y_val)

            # Evaluate
            preds = model.predict(X_val)
            preds = np.maximum(0.0, preds)  # clamp negatives

            y_val_np = y_val.values
            mae = mean_absolute_error(y_val_np, preds)
            mse = mean_squared_error(y_val_np, preds)
            rmse = np.sqrt(mse)
            r2  = r2_score(y_val_np, preds)
            metrics = {'MAE': mae, 'MSE': mse, 'RMSE': rmse, 'R2': r2}

            iter_time = time.time() - iter_start

            # Overfitting diagnostics
            final_train_loss = model.loss_history['train'][-1] if model.loss_history['train'] else float('nan')
            final_val_loss   = model.loss_history['val'][-1]   if model.loss_history['val']   else float('nan')
            best_val_loss    = min(model.loss_history['val'])   if model.loss_history['val']   else float('nan')
            epochs_trained   = len(model.loss_history['train'])
            overfit_ratio    = final_val_loss / max(final_train_loss, 1e-8)

            print(f"  MAE: {mae:.4f} | RMSE: {rmse:.4f} | R²: {r2:.4f}")
            print(f"  Epochs: {epochs_trained} | "
                  f"Train/Val gap: {overfit_ratio:.1f}x | "
                  f"Time: {iter_time:.0f}s")

            result_entry = {
                'iteration': i,
                'params': {**params,
                           'scheduler_type': scheduler_type,
                           'pct_start': pct_start,
                           'aug_p': aug_p,
                           'aug_channel_dropout': aug_channel_dropout,
                           'aug_mixup_alpha': aug_mixup_alpha},
                'metrics': metrics,
                'epochs_trained': epochs_trained,
                'final_train_loss': final_train_loss,
                'best_val_loss': best_val_loss,
                'overfit_ratio': overfit_ratio,
                'time_seconds': iter_time,
            }
            results.append(result_entry)

            if rmse < best_rmse:
                best_rmse = rmse
                best_params = result_entry['params']
                best_report = (f"Mean Absolute Error: {mae:.4f}\n"
                               f"Mean Squared Error: {mse:.4f}\n"
                               f"Root Mean Squared Error: {rmse:.4f}\n"
                               f"R-squared Score: {r2:.4f}\n")
                print(f"  >>> NEW BEST MODEL! RMSE: {best_rmse:.4f}")

        except Exception as exc:
            import traceback
            traceback.print_exc()
            print(f"  [WARNING] Iteration {i} failed: {exc}. Skipping.")
            continue

    # ------------------------------------------------------------------
    # 4. Summary & Save
    # ------------------------------------------------------------------
    total_time = time.time() - sweep_start
    print("\n" + "=" * 65)
    print("  SPATIO-TEMPORAL TRANSFORMER 2 SWEEP COMPLETE")
    print(f"  Total time: {total_time/60:.1f} minutes ({total_time/3600:.2f} hours)")
    print("=" * 65)

    if best_params is None:
        print("All iterations failed — no results to save.")
        return

    # Sort results by RMSE
    results.sort(key=lambda r: r['metrics']['RMSE'])

    print("\nTop-5 Configurations:")
    for rank, res in enumerate(results[:5], 1):
        p = res['params']
        m = res['metrics']
        print(f"  #{rank}: RMSE={m['RMSE']:.4f} MAE={m['MAE']:.4f} R²={m['R2']:.4f} "
              f"gap={res['overfit_ratio']:.1f}x "
              f"(d={p['d_model']}, s={p['num_layers_spatial']}×{p['nhead_spatial']}, "
              f"t={p['num_layers_temporal']}×{p['nhead_temporal']}, "
              f"ff={p['dim_feedforward']}, do={p['dropout_rate']})")

    print(f"\nBest Parameters:")
    for k, v in best_params.items():
        print(f"  {k}: {v}")
    print(f"\nBest Validation Metrics:")
    print(best_report)

    # ── Write report ──
    report_file = run_dir / "sweep_report.txt"
    with open(report_file, "w") as f:
        f.write("SPATIO-TEMPORAL TRANSFORMER 2 HYPERPARAMETER SWEEP REPORT\n")
        f.write("=" * 65 + "\n")
        f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"DEV_MODE: {DEV_MODE}\n")
        f.write(f"Total time: {total_time/60:.1f} min ({total_time/3600:.2f} hours)\n")
        f.write(f"Configurations tested: {len(results)}/{n_iter}\n")
        f.write(f"Epochs per config: {sweep_epochs}\n")
        f.write(f"Early-stop patience: {sweep_patience}\n")
        f.write(f"Training subjects ({len(train_subs)}): {', '.join(train_subs)}\n")
        f.write(f"Validation subjects ({len(val_subs)}): {', '.join(val_subs)}\n")
        f.write(f"Training samples: {len(X_train)}\n")
        f.write(f"Validation samples: {len(X_val)}\n\n")

        f.write("BEST PARAMETERS (lowest RMSE):\n")
        f.write("-" * 40 + "\n")
        for k, v in best_params.items():
            f.write(f"  {k}: {v}\n")
        f.write(f"\nValidation metrics on unseen participants "
                f"({', '.join(val_subs)}):\n")
        f.write(best_report + "\n")

        f.write("\n" + "=" * 65 + "\n")
        f.write("ALL CONFIGURATIONS RANKED BY RMSE\n")
        f.write("=" * 65 + "\n")

        for rank, res in enumerate(results, 1):
            m = res['metrics']
            p = res['params']
            f.write(f"\n--- Rank {rank} (Iteration {res['iteration']}) ---\n")
            f.write(f"  RMSE: {m['RMSE']:.4f} | MAE: {m['MAE']:.4f} | "
                    f"R²: {m['R2']:.4f}\n")
            f.write(f"  Epochs: {res['epochs_trained']} | "
                    f"Overfit ratio: {res['overfit_ratio']:.1f}x | "
                    f"Best val loss: {res['best_val_loss']:.4f}\n")
            f.write(f"  Time: {res['time_seconds']:.0f}s\n")
            f.write(f"  Params:\n")
            for k, v in p.items():
                f.write(f"    {k}: {v}\n")

    # ── Save results as CSV for easy analysis ──
    rows = []
    for res in results:
        row = {**res['params'], **res['metrics'],
               'epochs_trained': res['epochs_trained'],
               'overfit_ratio': res['overfit_ratio'],
               'best_val_loss': res['best_val_loss'],
               'time_seconds': res['time_seconds'],
               'iteration': res['iteration']}
        # Convert list/tuple params to strings for CSV
        for k, v in row.items():
            if isinstance(v, (list, tuple)):
                row[k] = str(v)
        rows.append(row)
    pd.DataFrame(rows).to_csv(run_dir / "sweep_results.csv", index=False)

    print(f"\nReport saved to: {report_file}")
    print(f"CSV results saved to: {run_dir / 'sweep_results.csv'}")


if __name__ == "__main__":
    main()
