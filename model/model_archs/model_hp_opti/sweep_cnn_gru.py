"""
CNN-GRU Hyperparameter Sweep — Anti-Overfitting Focus
======================================================
Runs a randomised hyperparameter search with a *single* fixed
cross-participant holdout (instead of full LOPO CV) to maximise the
number of configurations testable within a 24-hour Slurm budget.

Strategy
--------
1. Hold out ~4 random participants as a fixed validation set.
2. Sub-sample training data (max 300 segments / participant).
3. Train each config with aggressive early stopping (patience=15).
4. Rank all configs by validation RMSE and save a detailed report.

Usage
-----
    python sweep_cnn_gru.py               # 60 random configs (default)
    python sweep_cnn_gru.py --n_iter 10   # quick test with 10 configs
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

from model.data_loader import DataLoader
from model.model_archs.cnn_gru import CNNGRURegressor
from model.config_model import CNN_GRU_CONFIG, DATABASE_CONFIG, AUGMENTATION_CONFIG


# ---------------------------------------------------------------------------
# Target transformation helpers
# ---------------------------------------------------------------------------

def apply_target_transform(y: pd.Series, method: str) -> pd.Series:
    """Forward transform on targets before training."""
    if method == 'sqrt':
        return y.apply(np.sqrt)
    elif method == 'log1p':
        return y.apply(np.log1p)
    return y  # 'none'


def inverse_target_transform(preds: np.ndarray, method: str) -> np.ndarray:
    """Inverse transform on predictions to return to original scale."""
    if method == 'sqrt':
        return np.square(preds)
    elif method == 'log1p':
        return np.expm1(preds)
    return preds  # 'none'


def main():
    # ------------------------------------------------------------------
    # CLI
    # ------------------------------------------------------------------
    parser = argparse.ArgumentParser(description="CNN-GRU anti-overfitting sweep")
    parser.add_argument("--n_iter", type=int, default=60,
                        help="Number of random configurations to evaluate")
    parser.add_argument("--max_seg", type=int, default=300,
                        help="Max segments per participant for sub-sampling")
    parser.add_argument("--test_participants", type=int, default=4,
                        help="Number of participants to hold out for validation")
    args = parser.parse_args()

    n_iter = args.n_iter
    max_segments_per_participant = args.max_seg
    test_participants_count = args.test_participants

    # ------------------------------------------------------------------
    # Paths
    # ------------------------------------------------------------------
    base_dir = Path(__file__).resolve().parent.parent.parent.parent
    segments_dir = DATABASE_CONFIG['segments_dir']
    results_dir = base_dir / "model" / "model_results"

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = results_dir / f"sweep_cnn_gru_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # Parameter grid — ANTI-OVERFITTING FOCUS
    # ------------------------------------------------------------------
    param_grid = {
        # ── Model capacity ──────────────────────────────────────────
        'cnn_filters': [
            [32, 64, 128],             # Light
            [64, 128, 256],            # Medium
            [64, 128, 256, 256],       # Current (heavy)
        ],
        'cnn_kernel_sizes': [
            [64, 5, 3],               # Current
            [32, 5, 3],               # Smaller first kernel
            [64, 5, 3, 3],            # Matches 4-block filter configs
        ],
        'pool_size': [4],             # Fixed — changing this dramatically alters seq lengths
        'gru_hidden_size': [128, 192, 256],
        'gru_num_layers': [1, 2, 3],

        # ── Regularisation ──────────────────────────────────────────
        'dropout_rate': [0.3, 0.4, 0.5, 0.6, 0.7],
        'weight_decay': [1e-4, 5e-4, 1e-3, 5e-3, 1e-2],

        # ── Training ────────────────────────────────────────────────
        'learning_rate': [0.0005, 0.001, 0.002],
        'batch_size': [32, 64, 128],
        'loss_type': ['mse', 'mae', 'huber'],

        # ── Augmentation ────────────────────────────────────────────
        'channel_dropout_p': [0.15, 0.25, 0.35, 0.5],
        'magnitude_scale_range': [(0.5, 2.0), (0.3, 3.0)],

        # ── Target transformation (anti regression-to-mean) ────────
        'target_transform': ['none', 'sqrt', 'log1p'],
    }

    print("=" * 60)
    print("CNN-GRU ANTI-OVERFITTING SWEEP")
    print("=" * 60)
    print(f"Configurations to test : {n_iter}")
    print(f"Max segments/participant: {max_segments_per_participant}")
    print(f"Test participants       : {test_participants_count}")
    print(f"Results directory       : {run_dir}")
    print("=" * 60)

    # ------------------------------------------------------------------
    # 1. Load Data
    # ------------------------------------------------------------------
    h5_paths = sorted([p for p in segments_dir.glob("*.h5") if not p.name.startswith("._")])
    if not h5_paths:
        print(f"No HDF5 segment files found in {segments_dir}.")
        return

    loader = DataLoader()
    print("\nLoading raw segments for sweep...")
    df = loader.load_raw_segments(h5_paths)
    if df.empty:
        print("Raw segment loading failed or produced an empty DataFrame.")
        return

    print(f"Loaded {len(df)} total segments.")
    X, y = loader.prepare_for_ml(df, target_col="weight")

    if "subject" not in df.columns:
        print("Error: 'subject' column missing. Cannot split by participant.")
        return

    # Re-attach subject for splitting (prepare_for_ml drops meta-columns)
    X["subject"] = df["subject"].values

    # ------------------------------------------------------------------
    # 2. Cross-Participant Split + Sub-sampling
    # ------------------------------------------------------------------
    subjects = X["subject"]
    n_unique_subs = len(subjects.unique())
    test_ratio = min(0.3, max(0.1, test_participants_count / max(1, n_unique_subs)))

    gss = GroupShuffleSplit(n_splits=1, test_size=test_ratio, random_state=42)
    train_idx, val_idx = next(gss.split(X, y, groups=subjects))

    X_train_full = X.iloc[train_idx]
    y_train_full = y.iloc[train_idx]
    X_val = X.iloc[val_idx]
    y_val = y.iloc[val_idx]

    val_subs = sorted(X_val["subject"].unique())
    train_subs = sorted(X_train_full["subject"].unique())

    print(f"\nParticipant split:")
    print(f"  Training ({len(train_subs)} subjects): {', '.join(train_subs)}")
    print(f"  Validation ({len(val_subs)} subjects): {', '.join(val_subs)}")

    # Sub-sample training data
    rng = np.random.RandomState(42)
    sampled_indices = []
    for sub in train_subs:
        sub_indices = X_train_full[X_train_full["subject"] == sub].index
        if len(sub_indices) > max_segments_per_participant:
            chosen = rng.choice(sub_indices, size=max_segments_per_participant, replace=False)
            sampled_indices.extend(chosen)
        else:
            sampled_indices.extend(sub_indices)

    X_train_sampled = X_train_full.loc[sampled_indices].copy()
    y_train_sampled = y_train_full.loc[sampled_indices].copy()

    shuffle_idx = rng.permutation(len(X_train_sampled))
    X_train_sampled = X_train_sampled.iloc[shuffle_idx]
    y_train_sampled = y_train_sampled.iloc[shuffle_idx]

    print(f"\nSub-sampling complete:")
    print(f"  Training : {len(X_train_full)} → {len(X_train_sampled)} segments "
          f"(max {max_segments_per_participant}/participant)")
    print(f"  Validation: {len(X_val)} segments (full, unseen participants)")

    # ------------------------------------------------------------------
    # 3. Randomised Sweep
    # ------------------------------------------------------------------
    sampler = ParameterSampler(param_grid, n_iter=n_iter, random_state=42)

    best_rmse = float('inf')
    best_params = None
    best_report = ""
    best_model = None
    results = []
    sweep_start = time.time()

    for i, params in enumerate(sampler, 1):
        iter_start = time.time()

        # Align cnn_filters and cnn_kernel_sizes lengths
        n_blocks = min(len(params['cnn_filters']), len(params['cnn_kernel_sizes']))
        params['cnn_filters'] = params['cnn_filters'][:n_blocks]
        params['cnn_kernel_sizes'] = params['cnn_kernel_sizes'][:n_blocks]

        # Extract non-model-constructor overrides
        aug_channel_dropout_p = params.pop('channel_dropout_p')
        aug_magnitude_scale_range = params.pop('magnitude_scale_range')
        target_transform = params.pop('target_transform')

        elapsed = time.time() - sweep_start
        print(f"\n{'='*60}")
        print(f"--- Iteration {i}/{n_iter} "
              f"[Elapsed: {elapsed/3600:.1f}h] ---")
        print(f"Model params: { {k: v for k, v in params.items()} }")
        print(f"Aug overrides: channel_dropout_p={aug_channel_dropout_p}, "
              f"magnitude_scale_range={aug_magnitude_scale_range}")
        print(f"Target transform: {target_transform}")

        try:
            # Override augmentation config for this iteration
            aug_config = copy.deepcopy(AUGMENTATION_CONFIG)
            aug_config['channel_dropout_p'] = aug_channel_dropout_p
            aug_config['magnitude_scale_range'] = aug_magnitude_scale_range

            # Temporarily patch the module-level AUGMENTATION_CONFIG
            # so CNNGRURegressor.fit() picks up the overrides
            import model.config_model as _cfg
            import model.model_archs.cnn_gru as _cnn_gru_mod
            original_aug = _cnn_gru_mod.AUGMENTATION_CONFIG
            _cnn_gru_mod.AUGMENTATION_CONFIG = aug_config

            model = CNNGRURegressor(
                cnn_filters=params['cnn_filters'],
                cnn_kernel_sizes=params['cnn_kernel_sizes'],
                pool_size=params['pool_size'],
                gru_hidden_size=params['gru_hidden_size'],
                gru_num_layers=params['gru_num_layers'],
                dropout_rate=params['dropout_rate'],
                learning_rate=params['learning_rate'],
                weight_decay=params['weight_decay'],
                batch_size=params['batch_size'],
                loss_type=params['loss_type'],
                epochs=200,
                early_stopping_patience=15,
                scheduler_patience=8,
                scheduler_factor=0.5,
                balance_weights=CNN_GRU_CONFIG.get('balance_weights', False),
                balance_participants=CNN_GRU_CONFIG.get('balance_participants', False),
                random_state=CNN_GRU_CONFIG.get('random_state', 42),
            )

            # Apply target transform to training and validation labels
            y_train_t = apply_target_transform(y_train_sampled, target_transform)
            y_val_t = apply_target_transform(y_val, target_transform)

            # Train with explicit cross-participant validation set
            # Note: model sees transformed targets — it learns to predict sqrt(y) or log1p(y)
            model.fit(X_train_sampled, y_train_t, X_val=X_val, y_val=y_val_t)

            # Restore original augmentation config
            _cnn_gru_mod.AUGMENTATION_CONFIG = original_aug

            # Evaluate: predict in transformed space, then inverse-transform
            # back to original kg scale for fair metric comparison
            preds_transformed = model.predict(X_val)
            preds_original = inverse_target_transform(preds_transformed, target_transform)
            preds_original = np.maximum(0.0, preds_original)  # clamp negatives

            y_val_np = y_val.values
            mae = mean_absolute_error(y_val_np, preds_original)
            mse = mean_squared_error(y_val_np, preds_original)
            rmse = np.sqrt(mse)
            r2 = r2_score(y_val_np, preds_original)
            metrics = {'MAE': mae, 'MSE': mse, 'RMSE': rmse, 'R2': r2}
            report_str = (f"Mean Absolute Error: {mae:.4f}\n"
                          f"Mean Squared Error: {mse:.4f}\n"
                          f"Root Mean Squared Error: {rmse:.4f}\n"
                          f"R-squared Score: {r2:.4f}\n")
            iter_time = time.time() - iter_start

            # Extract the train/val gap as an overfitting indicator
            final_train_loss = model.loss_history['train'][-1] if model.loss_history['train'] else float('nan')
            final_val_loss = model.loss_history['val'][-1] if model.loss_history['val'] else float('nan')
            best_val_loss = min(model.loss_history['val']) if model.loss_history['val'] else float('nan')
            epochs_trained = len(model.loss_history['train'])
            overfit_ratio = final_val_loss / max(final_train_loss, 1e-8)

            print(f"  MAE: {metrics['MAE']:.4f} | RMSE: {metrics['RMSE']:.4f} | "
                  f"R²: {metrics['R2']:.4f}")
            print(f"  Epochs: {epochs_trained} | "
                  f"Train/Val gap: {overfit_ratio:.1f}x | "
                  f"Time: {iter_time:.0f}s")

            result_entry = {
                'iteration': i,
                'params': {**params,
                           'channel_dropout_p': aug_channel_dropout_p,
                           'magnitude_scale_range': aug_magnitude_scale_range,
                           'target_transform': target_transform},
                'metrics': metrics,
                'epochs_trained': epochs_trained,
                'final_train_loss': final_train_loss,
                'best_val_loss': best_val_loss,
                'overfit_ratio': overfit_ratio,
                'time_seconds': iter_time,
            }
            results.append(result_entry)

            if metrics['RMSE'] < best_rmse:
                best_rmse = metrics['RMSE']
                best_params = result_entry['params']
                best_report = report_str
                best_model = model
                print(f"  >>> NEW BEST MODEL! RMSE: {best_rmse:.4f}")

        except Exception as exc:
            import traceback
            traceback.print_exc()
            print(f"  [WARNING] Iteration {i} failed: {exc}. Skipping.")
            # Restore augmentation config on failure too
            try:
                _cnn_gru_mod.AUGMENTATION_CONFIG = original_aug
            except NameError:
                pass
            continue

    # ------------------------------------------------------------------
    # 4. Summary & Save
    # ------------------------------------------------------------------
    total_time = time.time() - sweep_start
    print("\n" + "=" * 60)
    print("CNN-GRU ANTI-OVERFITTING SWEEP COMPLETE")
    print(f"Total time: {total_time/3600:.2f} hours")
    print("=" * 60)

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
              f"(dropout={p['dropout_rate']}, wd={p['weight_decay']}, "
              f"gru={p['gru_hidden_size']}×{p['gru_num_layers']}, "
              f"loss={p['loss_type']}, transform={p['target_transform']})")

    print(f"\nBest Parameters:")
    for k, v in best_params.items():
        print(f"  {k}: {v}")
    print(f"\nBest Validation Metrics:")
    print(best_report)

    # ── Write report ──
    report_file = run_dir / "sweep_report.txt"
    with open(report_file, "w") as f:
        f.write("CNN-GRU ANTI-OVERFITTING SWEEP REPORT\n")
        f.write("=" * 60 + "\n")
        f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Total time: {total_time/3600:.2f} hours\n")
        f.write(f"Configurations tested: {len(results)}/{n_iter}\n")
        f.write(f"Training subjects ({len(train_subs)}): {', '.join(train_subs)}\n")
        f.write(f"Validation subjects ({len(val_subs)}): {', '.join(val_subs)}\n")
        f.write(f"Training samples (subsampled): {len(X_train_sampled)}\n")
        f.write(f"Validation samples: {len(X_val)}\n\n")

        f.write("BEST PARAMETERS (lowest RMSE):\n")
        f.write("-" * 40 + "\n")
        for k, v in best_params.items():
            f.write(f"  {k}: {v}\n")
        f.write(f"\nValidation metrics on unseen participants ({', '.join(val_subs)}):\n")
        f.write(best_report + "\n")

        f.write("\n" + "=" * 60 + "\n")
        f.write("ALL CONFIGURATIONS RANKED BY RMSE\n")
        f.write("=" * 60 + "\n")

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

    # ── Save best model ──
    model_path = run_dir / "best_cnn_gru_sweep_model.joblib"
    best_model.save(model_path)

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
    print(f"Best model saved to: {model_path}")
    print(f"CSV results saved to: {run_dir / 'sweep_results.csv'}")


if __name__ == "__main__":
    main()
