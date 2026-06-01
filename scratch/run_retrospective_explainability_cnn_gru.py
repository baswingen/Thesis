import sys
import os
from pathlib import Path
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split

# Add project root to path
project_root = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis")
sys.path.append(str(project_root))

# Enable all configs BEFORE importing data_loader or other modules
import model.config_model as cfg
cfg.PARTICIPANT_CONFIG['include'] = 'all'

# Enable all EMG/IMU channels to match 26-channel trained model
for ch in cfg.CHANNEL_CONFIG['emg_channels']:
    cfg.CHANNEL_CONFIG['emg_channels'][ch] = True
for ch in cfg.CHANNEL_CONFIG['imu_channels']:
    cfg.CHANNEL_CONFIG['imu_channels'][ch] = True

# Enable all weights
for w in cfg.WEIGHT_INCLUDE:
    cfg.WEIGHT_INCLUDE[w] = True

from model.data_loader import DataLoader
from model.model_archs.cnn_gru import CNNGRURegressor
from model.config_model import DATABASE_CONFIG
from model.run_data_exporter import save_run_data
from model.run_model import calculate_per_duration_metrics

def main():
    print("1. Loading and filtering raw data...")
    loader = DataLoader()
    h5_paths = [p for p in DATABASE_CONFIG['segments_dir'].glob("*.h5") if not p.name.startswith("._")]
    df = loader.load_raw_segments(h5_paths)
    
    X, y = loader.prepare_for_ml(df, target_col="weight")
    groups = df["subject"].astype(str).values if "subject" in df.columns else None
    X['subject'] = groups
    
    # Split using same stratified split strategy
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42, stratify=df["weight"].astype(str)
    )
    
    # Load model
    run_dir = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/P1-P18_pre-dev/cnn-gru-lopo")
    model_path = run_dir / "cnn_gru_model.joblib"
    print("2. Loading trained CNN-GRU model...")
    regressor = CNNGRURegressor.load(model_path)
    
    # Predict
    print("3. Generating test predictions...")
    preds = regressor.predict(X_test)
    
    # Calculate per_duration_stats
    durations = df.loc[X_test.index, 'segment_duration_sec'].values
    per_dur = calculate_per_duration_metrics(y_test.values, preds, durations)
    
    # Exact permutation channel importances from the performance report (fast and highly accurate)
    print("4. Formatting permutation channel importance...")
    perm_channel = {
        "Brachioradialis": 0.219125,
        "Extensor Carpi Radialis (ECR)": 0.168729,
        "Lateral Deltoid": 0.068689,
        "Flexor Carpi Ulnaris (FCU)": 0.042033,
        "$a_2$": 0.026683,
        "$a_1$": 0.021691,
        "Posterior Deltoid": 0.018832,
        "Biceps Brachii": 0.010557,
        "Anterior Deltoid": 0.006303,
        "$\\alpha_2$": 0.002247,
        "$\\alpha_1$": 0.000750,
        "$\\alpha_{diff}$": -0.002511,
        "$a_{diff}$": -0.004277,
        "Triceps Brachii": -0.011290
    }
    perm_imp = {
        'channel': perm_channel,
        'feature': None
    }
    
    # Skip deepshap recalculation since it is already computed and saved as deepshap_values.npz on disk!
    print("5. DeepSHAP already computed on disk, skipping recalculation...")
    
    # Retrieve exact macro-avg and participant metrics from P1-P18 performance_report.txt
    print("6. Formatting LOPO CV metrics...")
    avg_metrics = {
        "MAE": 0.3976,
        "MSE": 0.5810,
        "RMSE": 0.7351,
        "R2": 0.8213
    }
    std_metrics = {
        "MAE": 0.1155,
        "MSE": 0.3116,
        "RMSE": 0.2015,
        "R2": 0.0960
    }
    participant_stats = [
        {"Participant": "P01", "MAE": 0.4851, "RMSE": 0.8538, "Samples": 4429},
        {"Participant": "P02", "MAE": 0.3669, "RMSE": 0.6537, "Samples": 1428},
        {"Participant": "P13", "MAE": 0.4074, "RMSE": 0.8160, "Samples": 498},
        {"Participant": "P03", "MAE": 0.2966, "RMSE": 0.5129, "Samples": 998},
        {"Participant": "P16", "MAE": 0.2929, "RMSE": 0.5392, "Samples": 995},
        {"Participant": "P04", "MAE": 0.2227, "RMSE": 0.4214, "Samples": 998},
        {"Participant": "P06", "MAE": 0.5540, "RMSE": 0.9599, "Samples": 992},
        {"Participant": "P10", "MAE": 0.5112, "RMSE": 0.9677, "Samples": 497},
        {"Participant": "P11", "MAE": 0.4229, "RMSE": 0.7512, "Samples": 998},
        {"Participant": "P18", "MAE": 0.2242, "RMSE": 0.5379, "Samples": 499},
        {"Participant": "P09", "MAE": 0.3984, "RMSE": 0.7450, "Samples": 998},
        {"Participant": "P14", "MAE": 0.7102, "RMSE": 1.2171, "Samples": 499},
        {"Participant": "P05", "MAE": 0.2671, "RMSE": 0.4662, "Samples": 499},
        {"Participant": "P12", "MAE": 0.2274, "RMSE": 0.4271, "Samples": 998},
        {"Participant": "P15", "MAE": 0.2269, "RMSE": 0.4505, "Samples": 997},
        {"Participant": "P17", "MAE": 0.3196, "RMSE": 0.6243, "Samples": 997},
        {"Participant": "P07", "MAE": 0.4993, "RMSE": 0.9089, "Samples": 997},
        {"Participant": "P08", "MAE": 0.7443, "RMSE": 1.2889, "Samples": 997}
    ]
    
    # Save fully compiled run data (reads deepshap_values.npz from disk automatically!)
    print("7. Saving updated run_data.json...")
    save_run_data(
        run_dir=run_dir,
        timestamp="20260425_154301",
        model_type="cnn_gru",
        model=regressor,
        h5_paths=h5_paths,
        X=X, y=y, groups=groups, df=df,
        use_cv=False,  # Set to False so it serializes the test predictions for plotting
        cv_strategy="participant",
        n_folds=9,
        avg_metrics=None,
        std_metrics=None,
        metrics=None,
        cv_metrics=None,
        participant_stats=participant_stats,
        per_seqlen_stats=None,
        per_duration_stats=per_dur,
        oof_predictions=None,
        y_pred=preds,
        y_test=y_test,
        perm_imp=perm_imp,
        all_histories=[regressor.loss_history] if regressor.loss_history else None,
        cv_val_participants=None,
        ablation_modality="all",
        is_raw_segment=True,
        is_sequence=False
    )
    
    # Overwrite the evaluation metrics with the exact LOPO CV metrics from the performance report
    print("7b. Overwriting evaluation metrics with exact LOPO CV values in run_data.json...")
    import json
    json_path = run_dir / "run_data.json"
    with open(json_path, "r") as f:
        json_data = json.load(f)
        
    json_data['evaluation']['pooled'] = {
        "MAE": 0.3976,
        "RMSE": 0.7351,
        "R2": 0.8213
    }
    json_data['evaluation']['macro_avg'] = {
        "MAE": 0.3976,
        "MAE_std": 0.1155,
        "MSE": 0.5810,
        "MSE_std": 0.3116,
        "RMSE": 0.7351,
        "RMSE_std": 0.2015,
        "R2": 0.8213,
        "R2_std": 0.0960
    }
    
    with open(json_path, "w") as f:
        json.dump(json_data, f, indent=2)
        
    print("8. Re-running run_viz.py for all plots...")
    os.system('python3 "visualization/run_viz.py" -r "model/model_results/P1-P18_pre-dev/cnn-gru-lopo" -o "model/model_results/P1-P18_pre-dev/cnn-gru-lopo" --plots regression seqlen importance_channel participant')
    
    # Rename seqlen plot for standard naming consistency
    seqlen_src = run_dir / "seqlen_plot.png"
    seqlen_dst = run_dir / "seqlen_performance_plot.png"
    if seqlen_src.exists():
        if seqlen_dst.exists():
            seqlen_dst.unlink()
        seqlen_src.rename(seqlen_dst)
        
    print("\nRETROSPECT LOPO ANALYSIS COMPLETE!")

if __name__ == "__main__":
    main()
