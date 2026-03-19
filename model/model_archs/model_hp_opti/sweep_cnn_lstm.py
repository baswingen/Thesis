import sys
import numpy as np
from pathlib import Path
from datetime import datetime
from sklearn.model_selection import train_test_split, ParameterSampler

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
    run_dir = results_dir / f"sweep_cnn_lstm_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------ #
    # Parameter grid                                                        #
    # No window/sliding parameters — CNN extracts features automatically.  #
    # ------------------------------------------------------------------ #
    param_grid = {
        # CNN architecture: sampled as paired lists (aligned by index)
        'cnn_filters': [
            [16, 32],
            [32, 64],
            [32, 64, 128],
            [64, 128],
            [64, 128, 256],
            [128, 256],
            [128, 256, 512],
        ],
        'cnn_kernel_sizes': [
            [3, 3],
            [5, 3],
            [7, 5],
            [7, 5, 3],
            [9, 7],
            [11, 7],
            [11, 7, 5],
        ],
        'pool_size': [2, 4],

        # LSTM head
        'lstm_hidden_size': [64, 128, 256, 512],
        'lstm_num_layers':  [1, 2, 3, 4],

        # Regularisation
        'dropout_rate':  [0.1, 0.2, 0.3, 0.4, 0.5],
        'weight_decay':  [0.0, 1e-5, 1e-4, 1e-3],

        # Optimisation
        'learning_rate': [0.0005, 0.001, 0.002, 0.005, 0.01],
        'batch_size':    [16, 32, 64, 128],
    }

    n_iter = 100  # Extensive random search

    print(f"Starting CNN-LSTM hyperparameter sweep with {n_iter} iterations...")
    print(f"Results will be saved to: {run_dir}\n")

    # ------------------------------------------------------------------ #
    # 1.  Load raw segments ONCE (window params are not swept)             #
    # ------------------------------------------------------------------ #
    h5_paths = list(segments_dir.glob("*.h5"))
    if not h5_paths:
        print(f"No HDF5 segment files found in {segments_dir}.")
        return

    loader = DataLoader()
    print("Loading raw segments (done once for all iterations)...")
    df = loader.load_raw_segments(h5_paths)
    if df.empty:
        print("Raw segment loading failed or produced an empty DataFrame.")
        return

    print(f"Loaded {len(df)} segments.\n")

    X, y = loader.prepare_for_ml(df, target_col="weight")

    # Fixed train/test split — same across every iteration for fair comparison
    stratify = df["weight"].astype(str) if "weight" in df.columns else None
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=7, stratify=stratify
    )
    print(f"Training on {len(X_train)} segments, testing on {len(X_test)} segments.\n")

    # ------------------------------------------------------------------ #
    # 2.  Sweep                                                             #
    # ------------------------------------------------------------------ #
    sampler = ParameterSampler(param_grid, n_iter=n_iter, random_state=42)

    best_mae     = float('inf')
    best_params  = None
    best_report  = ""
    best_model   = None
    results      = []

    for i, params in enumerate(sampler, 1):
        # Align CNN list lengths (ParameterSampler draws them independently)
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
                epochs=300,                  # fixed; rely on early stopping
                early_stopping_patience=80,  # fixed
                validation_split=CNN_LSTM_CONFIG.get('validation_split', 0.2),
                scheduler_patience=10,
                scheduler_factor=0.5,
                random_state=CNN_LSTM_CONFIG.get('random_state', 42),
            )

            model.fit(X_train, y_train)
            metrics, report_str = model.evaluate(X_test, y_test)

            print(f"Iteration MAE: {metrics['MAE']:.4f}, "
                  f"RMSE: {metrics['RMSE']:.4f}, "
                  f"R2: {metrics['R2']:.4f}")

            results.append({
                'iteration': i,
                'params':    params,
                'metrics':   metrics,
            })

            if metrics['MAE'] < best_mae:
                best_mae    = metrics['MAE']
                best_params = params
                best_report = report_str
                best_model  = model
                print(f">>> New Best Model! MAE: {best_mae:.4f}")

        except Exception as exc:
            print(f"[WARNING] Iteration {i} failed: {exc}. Skipping.")
            continue

    # ------------------------------------------------------------------ #
    # 3.  Summary & save                                                    #
    # ------------------------------------------------------------------ #
    print("\n" + "=" * 50)
    print("SWEEP COMPLETE")
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

    report_file = run_dir / "sweep_report.txt"
    with open(report_file, "w") as f:
        f.write("CNN-LSTM HYPERPARAMETER SWEEP REPORT\n")
        f.write("=" * 50 + "\n\n")
        f.write("BEST PARAMETERS:\n")
        for k, v in best_params.items():
            f.write(f"  {k}: {v}\n")
        f.write("\nBEST EVALUATION METRICS:\n")
        f.write(best_report)
        f.write("\n" + "-" * 50 + "\n")
        f.write("ALL CONFIGURATIONS TESTED:\n")
        for res in results:
            f.write(f"\nIteration {res['iteration']}\n")
            f.write(f"Params: {res['params']}\n")
            f.write(f"MAE: {res['metrics']['MAE']:.4f}, "
                    f"RMSE: {res['metrics']['RMSE']:.4f}, "
                    f"R2: {res['metrics']['R2']:.4f}\n")

    model_path = run_dir / "best_cnn_lstm_model.joblib"
    best_model.save(model_path)
    print(f"\nReport and best model saved to {run_dir}")


if __name__ == "__main__":
    main()
