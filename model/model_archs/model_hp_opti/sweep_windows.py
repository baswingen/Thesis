import os
import json
import numpy as np
import sys
import pandas as pd
from pathlib import Path
from datetime import datetime
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score

sys.path.append(str(Path(__file__).parent.parent))

from model.data_loader import DataLoader
from model.model_archs.lstm import LSTMRegressor
from model.config_model import FEATURE_CONFIG, LSTM_CONFIG

def main():
    base_dir = Path(__file__).parent.parent
    segments_dir = base_dir / "database" / "segments"
    results_dir = base_dir / "model" / "model_results"
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    sweep_dir = results_dir / f"sweep_windows_{timestamp}"
    sweep_dir.mkdir(parents=True, exist_ok=True)
    
    h5_paths = list(segments_dir.glob("*.h5"))
    if not h5_paths:
        print("No HDF5 segment files found.")
        return

    # Define the sweep grid
    # For EMG, fast transients are important, try short windows.
    # For IMU, slow arm kinematics are important, try long windows.
    emg_window_sizes = [0.1, 0.2, 0.5]
    imu_window_sizes = [0.25, 0.5, 1.0]
    window_step = 0.1
    
    results = []
    
    print(f"Results will be saved to: {sweep_dir}")
    print(f"Starting sweep across {len(emg_window_sizes) * len(imu_window_sizes)} combinations...\n")

    for emg_ws in emg_window_sizes:
        for imu_ws in imu_window_sizes:
            print(f"============================================================")
            print(f"Evaluating: EMG Window = {emg_ws}s | IMU Window = {imu_ws}s")
            print(f"============================================================")
            
            # Temporarily update the global config just in case, though we pass explicitly
            FEATURE_CONFIG['emg_window_size_sec'] = emg_ws
            FEATURE_CONFIG['imu_window_size_sec'] = imu_ws
            
            loader = DataLoader()
            
            print("  Extracting features...")
            df = loader.load_and_extract_features(
                h5_paths, 
                is_sequence=True,
                emg_window_size_sec=emg_ws,
                imu_window_size_sec=imu_ws,
                window_step_sec=window_step
            )
            
            if df.empty:
                print("  Failed to extract features or empty dataframe. Skipping.")
                continue
                
            X, y = loader.prepare_for_ml(df, target_col="weight")
            
            stratify = df["weight"].astype(str) if "weight" in df.columns else None
            X_train, X_test, y_train, y_test = train_test_split(
                X, y, test_size=0.2, random_state=42, stratify=stratify
            )
            
            print(f"  Training on {len(X_train)} samples, testing on {len(X_test)} samples.")
            
            model = LSTMRegressor(**LSTM_CONFIG)
            model.emg_window_size_sec = emg_ws
            model.imu_window_size_sec = imu_ws
            model.window_step_sec = window_step
            # reduce epochs a bit to speed up the sweep while still getting relative performance
            # default is 100, we'll keep it via early stopping
            
            model.fit(X_train, y_train)
            
            y_pred = model.predict(X_test)
            mae = mean_absolute_error(y_test, y_pred)
            rmse = np.sqrt(mean_squared_error(y_test, y_pred))
            r2 = r2_score(y_test, y_pred)
            
            print(f"  -> MAE: {mae:.4f} | RMSE: {rmse:.4f} | R2: {r2:.4f}\n")
            
            results.append({
                'emg_window_size_sec': emg_ws,
                'imu_window_size_sec': imu_ws,
                'MAE': mae,
                'RMSE': rmse,
                'R2': r2,
                'Epochs_Trained': len(model.loss_history.get('val', []))
            })
            
            # Save intermediate results so we don't lose them if crashed
            df_results = pd.DataFrame(results)
            df_results = df_results.sort_values(by='MAE', ascending=True)
            df_results.to_csv(sweep_dir / "sweep_results.csv", index=False)

    print("\n================ SWEEP COMPLETE ================")
    df_results = pd.DataFrame(results).sort_values(by='MAE', ascending=True)
    print("Top 5 configurations by lowest MAE:")
    print(df_results.head(5).to_string(index=False))
    
    report_file = sweep_dir / "sweep_report.txt"
    with open(report_file, "w") as f:
        f.write("WINDOW SIZE SWEEP RESULTS\n")
        f.write("==============================\n")
        f.write(df_results.to_string(index=False))
        
    print(f"\nFinal sweep report saved to {report_file}")

if __name__ == "__main__":
    main()
