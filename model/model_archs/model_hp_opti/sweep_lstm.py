import os
import sys
import numpy as np
import pandas as pd
from pathlib import Path
from datetime import datetime
from sklearn.model_selection import train_test_split, ParameterSampler

# Add project root to sys.path so 'model' package can be found
sys.path.append(str(Path(__file__).resolve().parent.parent.parent.parent))

from model.data_loader import DataLoader
from model.model_archs.lstm import LSTMRegressor
from model.config_model import LSTM_CONFIG, FEATURE_CONFIG

def main():
    base_dir = Path(__file__).resolve().parent.parent.parent.parent
    segments_dir = base_dir / "database" / "segments"
    results_dir = base_dir / "model" / "model_results"
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = results_dir / f"sweep_lstm_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    
    # Define Parameter Grid for LSTM
    param_grid = {
        'hidden_size': [32, 64, 128, 256, 512],
        'num_layers': [1, 2, 3, 4],
        'dropout_rate': [0.1, 0.2, 0.3, 0.4, 0.5],
        'learning_rate': [0.0005, 0.001, 0.005, 0.01],
        'batch_size': [32, 64, 128, 256],
        'weight_decay': [0.0, 1e-5, 1e-4, 1e-3],
        'emg_window_size_sec': [0.1, 0.15, 0.2, 0.25],
        'imu_window_size_sec': [0.15, 0.2, 0.25, 0.3],
        'window_step_sec': [0.05, 0.1, 0.15]
    }
    
    n_iter = 100  # Extensive search
    
    print(f"Starting LSTM hyperparameter sweep with {n_iter} iterations...")
    print(f"Results will be saved to: {run_dir}\n")
    
    # 1. Prepare Paths
    h5_paths = list(segments_dir.glob("*.h5"))
    if not h5_paths:
        print(f"No HDF5 segment files found in {segments_dir}.")
        return

    loader = DataLoader()
    sampler = ParameterSampler(param_grid, n_iter=n_iter, random_state=42)
    
    best_mae = float('inf')
    best_params = None
    best_report = ""
    best_model = None
    
    results = []
    
    # Sweep Loop
    for i, params in enumerate(sampler, 1):
        print(f"\n--- Iteration {i}/{n_iter} ---")
        print(f"Testing parameters: {params}")
        
        # 2. Load and Extract Data with current iteration window settings
        df = loader.load_and_extract_features(h5_paths, 
                                              emg_window_size_sec=params['emg_window_size_sec'],
                                              imu_window_size_sec=params['imu_window_size_sec'],
                                              window_step_sec=params['window_step_sec'])
        if df.empty:
            print("Data extraction failed or skipped for this configuration.")
            continue
            
        X, y = loader.prepare_for_ml(df, target_col="weight")
        
        # 3. Train/Test Split
        stratify = df["label"] if "label" in df.columns else None
        X_train, X_test, y_train, y_test = train_test_split(
            X, y, test_size=0.2, random_state=7, stratify=stratify
        )
        
        # 4. Initialize LSTM Regressor with fixed user-requested settings and sweep params
        model = LSTMRegressor(
            hidden_size=params['hidden_size'],
            num_layers=params['num_layers'],
            dropout_rate=params['dropout_rate'],
            learning_rate=params['learning_rate'],
            weight_decay=params['weight_decay'],
            batch_size=params['batch_size'],
            epochs=250,                                      # User forced
            early_stopping_patience=100,                     # User forced
            validation_split=LSTM_CONFIG.get('validation_split', 0.2),
            emg_window_size_sec=params['emg_window_size_sec'],
            imu_window_size_sec=params['imu_window_size_sec'],
            window_step_sec=params['window_step_sec'],
            random_state=LSTM_CONFIG.get('random_state', 42)
        )
        
        # Train
        model.fit(X_train, y_train)
        
        # Evaluate
        metrics, report_str = model.evaluate(X_test, y_test)
        print(f"Iteration MAE: {metrics['MAE']:.4f}, R2: {metrics['R2']:.4f}\n")
        
        # Store results
        results.append({
            'iteration': i,
            'params': params,
            'metrics': metrics
        })
        
        if metrics['MAE'] < best_mae:
            best_mae = metrics['MAE']
            best_params = params
            best_report = report_str
            best_model = model
            print(f">>> New Best Model! MAE: {best_mae:.4f}")
            
    # 4. Save best run results
    print("\n" + "="*50)
    print("SWEEP COMPLETE")
    print("="*50)
    print("Best Parameters:")
    for k, v in best_params.items():
        print(f"  {k}: {v}")
    print("\nBest Metrics:")
    print(best_report)
    print("="*50)
    
    # Save sweep details to text file
    report_file = run_dir / "sweep_report.txt"
    with open(report_file, "w") as f:
        f.write("LSTM HYPERPARAMETER SWEEP REPORT\n")
        f.write("="*50 + "\n\n")
        f.write("BEST PARAMETERS:\n")
        for k, v in best_params.items():
            f.write(f"{k}: {v}\n")
        f.write("\nBEST EVALUATION METRICS:\n")
        f.write(best_report)
        f.write("\n" + "-"*50 + "\n")
        f.write("ALL CONFIGURATIONS TESTED:\n")
        for res in results:
            f.write(f"\nIteration {res['iteration']}\n")
            f.write(f"Params: {res['params']}\n")
            f.write(f"MAE: {res['metrics']['MAE']:.4f}, R2: {res['metrics']['R2']:.4f}\n")

    # Save best model
    model_path = run_dir / "best_lstm_model.joblib"
    best_model.save(model_path)
    print(f"Report and best model saved to {run_dir}")

if __name__ == "__main__":
    main()
