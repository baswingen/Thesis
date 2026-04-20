import sys
import numpy as np
import pandas as pd
from pathlib import Path
from datetime import datetime
from sklearn.model_selection import ParameterSampler, GroupShuffleSplit
from sklearn.metrics import mean_squared_error

# Add project root to sys.path so 'model' package can be found
sys.path.append(str(Path(__file__).resolve().parent.parent.parent.parent))

from model.data_loader import DataLoader
from model.model_archs.cnn_lstm import CNNLSTMRegressor
from model.config_model import CNN_LSTM_CONFIG


def main():
    base_dir = Path(__file__).resolve().parent.parent.parent.parent
    segments_dir = base_dir / "database" / "segments"
    results_dir = base_dir / "model" / "model_results"

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = results_dir / f"sweep_cnn_lstm_gen_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------ #
    # Parameter grid HIGH CAPACITY (Generalization Focus)                   #
    # ------------------------------------------------------------------ #
    param_grid = {
        'cnn_filters': [
            [64, 128, 256],
            [128, 256],
            [128, 256, 512],
            [128, 256, 512, 1024]
        ],
        'cnn_kernel_sizes': [
            [7, 5, 3],
            [11, 7],
            [11, 7, 5],
            [11, 7, 5, 3]
        ],
        'pool_size': [2, 4],
        'lstm_hidden_size': [256, 512],
        'lstm_num_layers':  [3, 4],
        'dropout_rate':  [0.3, 0.4, 0.5],
        'weight_decay':  [1e-4, 1e-3, 5e-3], # Higher regularization for larger models
        'learning_rate': [0.0005, 0.001, 0.002],
        'batch_size':    [32, 64],
    }

    n_iter = 40  
    max_segments_per_participant = 300 

    print(f"Starting CNN-LSTM Generalization Sweep with {n_iter} iterations...")
    print(f"Results will be saved to: {run_dir}\n")

    # ------------------------------------------------------------------ #
    # 1. Load Data             #
    # ------------------------------------------------------------------ #
    h5_paths = list(segments_dir.glob("*.h5"))
    if not h5_paths:
        print(f"No HDF5 segment files found in {segments_dir}.")
        return

    loader = DataLoader()
    print("Loading raw segments for sweep...")
    df = loader.load_raw_segments(h5_paths)
    if df.empty:
        print("Raw segment loading failed or produced an empty DataFrame.")
        return

    print(f"Loaded {len(df)} total segments.")
    X, y = loader.prepare_for_ml(df, target_col="weight")
    
    if "subject" not in df.columns:
        print("Error: 'subject' column missing in DataFrame. Cannot perform GroupShuffleSplit.")
        return
        
    # prepare_for_ml drops meta-columns, so we must add subject back for splitting
    X["subject"] = df["subject"].values

    # ------------------------------------------------------------------ #
    # 2. Strict Cross-Participant Split & Sub-sampling                       #
    # ------------------------------------------------------------------ #
    # Divide total pool (e.g. 16/18 subjects) into ~12 Train and ~4 Test
    subjects = X["subject"]
    
    # test_size calculated dynamically for target exactly 4 participants if possible
    n_unique_subs = len(subjects.unique())
    test_participants_count = 4
    test_ratio = min(0.3, max(0.1, test_participants_count / max(1, n_unique_subs)))

    gss = GroupShuffleSplit(n_splits=1, test_size=test_ratio, random_state=42)
    train_idx, val_idx = next(gss.split(X, y, groups=subjects))

    X_train_full = X.iloc[train_idx]
    y_train_full = y.iloc[train_idx]
    X_val = X.iloc[val_idx]
    y_val = y.iloc[val_idx]

    val_subs = X_val["subject"].unique()
    train_subs = X_train_full["subject"].unique()
    
    print(f"Divided Participants:")
    print(f"  Training pool ({len(train_subs)} subjects): {', '.join(train_subs)}")
    print(f"  Validation pool ({len(val_subs)} subjects): {', '.join(val_subs)}")

    # Sub-sample the Training Group specifically
    sampled_indices = []
    for sub in train_subs:
        sub_mask = X_train_full["subject"] == sub
        sub_indices = X_train_full[sub_mask].index
        
        if len(sub_indices) > max_segments_per_participant:
            # Replace=False ensures unique subsets
            chosen = np.random.choice(sub_indices, size=max_segments_per_participant, replace=False)
            sampled_indices.extend(chosen)
        else:
            sampled_indices.extend(sub_indices)

    # Convert sampled_indices back to iloc-friendly boolean mask or slice
    # Note: indices are from the original dataframe, careful with .loc/.iloc
    X_train_sampled = X_train_full.loc[sampled_indices].copy()
    y_train_sampled = y_train_full.loc[sampled_indices].copy()

    # We shuffle the sampled dataset
    shuffle_mask = np.random.permutation(len(X_train_sampled))
    X_train_sampled = X_train_sampled.iloc[shuffle_mask]
    y_train_sampled = y_train_sampled.iloc[shuffle_mask]

    print(f"\nSubsampling Complete:")
    print(f"  Training segments compressed to {len(X_train_sampled)} samples (Max {max_segments_per_participant}/participant).")
    print(f"  Validation evaluating on {len(X_val)} unseen samples.")

    # ------------------------------------------------------------------ #
    # 3. Parameter Sweep                                                    #
    # ------------------------------------------------------------------ #
    sampler = ParameterSampler(param_grid, n_iter=n_iter, random_state=42)

    best_rmse    = float('inf')
    best_params  = None
    best_report  = ""
    best_model   = None
    results      = []

    for i, params in enumerate(sampler, 1):
        n_blocks = min(len(params['cnn_filters']), len(params['cnn_kernel_sizes']))
        params['cnn_filters']      = params['cnn_filters'][:n_blocks]
        params['cnn_kernel_sizes'] = params['cnn_kernel_sizes'][:n_blocks]

        print(f"\n--- Iteration {i}/{n_iter} ---")
        print(f"Testing parameters: {params}")

        try:
            model = CNNLSTMRegressor(
                cnn_filters=params['cnn_filters'],
                cnn_kernel_sizes=params['cnn_kernel_sizes'],
                pool_size=params['pool_size'],
                lstm_hidden_size=params['lstm_hidden_size'],
                lstm_num_layers=params['lstm_num_layers'],
                dropout_rate=params['dropout_rate'],
                learning_rate=params['learning_rate'],
                weight_decay=params['weight_decay'],
                batch_size=params['batch_size'],
                epochs=150,                  # Higher limit, rely heavily on early stopping
                early_stopping_patience=15,  # Much tighter early stopping (fast fails)
                scheduler_patience=8,
                scheduler_factor=0.5,
                random_state=CNN_LSTM_CONFIG.get('random_state', 42),
                balance_weights=CNN_LSTM_CONFIG.get('balance_weights', False),
            )

            # Pass X_val directly to explicitly link early stopping to generalization dataset
            model.fit(X_train_sampled, y_train_sampled, X_val=X_val, y_val=y_val)
            metrics, report_str = model.evaluate(X_val, y_val)

            print(f"Iteration MAE: {metrics['MAE']:.4f}, "
                  f"RMSE: {metrics['RMSE']:.4f}, "
                  f"R2: {metrics['R2']:.4f}")

            results.append({
                'iteration': i,
                'params':    params,
                'metrics':   metrics,
            })

            # Optimize for RMSE
            if metrics['RMSE'] < best_rmse:
                best_rmse    = metrics['RMSE']
                best_params = params
                best_report = report_str
                best_model  = model
                print(f">>> New Best Model! RMSE: {best_rmse:.4f}")

        except Exception as exc:
            import traceback
            traceback.print_exc()
            print(f"[WARNING] Iteration {i} failed: {exc}. Skipping.")
            continue

    # ------------------------------------------------------------------ #
    # 4. Summary & save                                                     #
    # ------------------------------------------------------------------ #
    print("\n" + "=" * 50)
    print("GENERALIZATION SWEEP COMPLETE")
    print("=" * 50)

    if best_params is None:
        print("All iterations failed — no results to save.")
        return

    print("Best Parameters:")
    for k, v in best_params.items():
        print(f"  {k}: {v}")
    print("\nBest Metrics:")
    print(best_report)
    print("=" * 50)

    report_file = run_dir / "sweep_gen_report.txt"
    with open(report_file, "w") as f:
        f.write("CNN-LSTM GENERALIZATION SWEEP REPORT\n")
        f.write("=" * 50 + "\n\n")
        f.write("BEST PARAMETERS (based on lowest RMSE):\n")
        for k, v in best_params.items():
            f.write(f"  {k}: {v}\n")
        f.write(f"\nValidation metrics on Unseen Participants ({', '.join(val_subs)}):\n")
        f.write(best_report)
        f.write("\n" + "-" * 50 + "\n")
        f.write("ALL CONFIGURATIONS TESTED:\n")
        for res in results:
            f.write(f"\nIteration {res['iteration']}\n")
            f.write(f"Params: {res['params']}\n")
            f.write(f"MAE: {res['metrics']['MAE']:.4f}, "
                    f"RMSE: {res['metrics']['RMSE']:.4f}, "
                    f"R2: {res['metrics']['R2']:.4f}\n")

    model_path = run_dir / "best_cnn_lstm_gen_model.joblib"
    best_model.save(model_path)
    print(f"\nReport and best model saved to {run_dir}")


if __name__ == "__main__":
    main()
