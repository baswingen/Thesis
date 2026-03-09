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
from model.config_model import LSTM_CONFIG

def main():
    base_dir = Path(__file__).resolve().parent.parent.parent.parent
    segments_dir = base_dir / "database" / "segments"
    results_dir = base_dir / "model" / "model_results"
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = results_dir / f"sweep_lstm_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    
    # Define Parameter Grid for LSTM
    param_grid = {
        'hidden_size': [64, 128, 256, 512],
        'num_layers': [1, 2, 3, 4],
        'dropout_rate': [0.1, 0.2, 0.3, 0.4, 0.5],
        'learning_rate': [0.0001, 0.0005, 0.001, 0.005, 0.01],
        'batch_size': [16, 32, 64, 128],
        'epochs': [100, 150]  # Let them train a bit longer
    }
    
    n_iter = 15  # Increased for a more thorough search
    
    print(f"Starting LSTM hyperparameter sweep with {n_iter} iterations...")
    print(f"Results will be saved to: {run_dir}\n")
    
    # 1. Load Data
    h5_paths = list(segments_dir.glob("*.h5"))
    if not h5_paths:
        print(f"No HDF5 segment files found in {segments_dir}.")
        return

    loader = DataLoader()
    print("Extracting features (This only happens once for the sweep)...")
    window_size_sec = LSTM_CONFIG.get('window_size_sec', 0.25)
    window_step_sec = LSTM_CONFIG.get('window_step_sec', 0.1)
    
    df = loader.load_and_extract_features(h5_paths, 
                                          window_size_sec=window_size_sec,
                                          window_step_sec=window_step_sec)
    
    if df.empty:
        print("Data extraction failed.")
        return
        
    X, y = loader.prepare_for_ml(df, target_col="weight")
    
    # 2. Train/Test Split (CV is disabled for this sweep to speed it up)
    stratify = df["label"] if "label" in df.columns else None
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=7, stratify=stratify
    )
    
    print(f"\nTraining on {len(X_train)} samples, testing on {len(X_test)} samples.\n")
    
    sampler = ParameterSampler(param_grid, n_iter=n_iter, random_state=42)
    
    best_mae = float('inf')
    best_params = None
    best_report = ""
    best_model = None
    
    results = []
    
    # 3. Sweep Loop
    for i, params in enumerate(sampler, 1):
        print(f"--- Iteration {i}/{n_iter} ---")
        print(f"Testing parameters: {params}")
        
        # Initialize LSTM Regressor with these params + static params
        model = LSTMRegressor(
            hidden_size=params['hidden_size'],
            num_layers=params['num_layers'],
            dropout_rate=params['dropout_rate'],
            learning_rate=params['learning_rate'],
            batch_size=params['batch_size'],
            epochs=params['epochs'],
            validation_split=LSTM_CONFIG.get('validation_split', 0.2),
            early_stopping_patience=LSTM_CONFIG.get('early_stopping_patience', 10),
            window_size_sec=window_size_sec,
            window_step_sec=window_step_sec,
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
