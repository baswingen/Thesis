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
cfg.PARTICIPANT_CONFIG['include'] = ['P01']

# Enable all EMG/IMU channels and features to match trained model
for ch in cfg.CHANNEL_CONFIG['emg_channels']:
    cfg.CHANNEL_CONFIG['emg_channels'][ch] = True
for ch in cfg.CHANNEL_CONFIG['imu_channels']:
    cfg.CHANNEL_CONFIG['imu_channels'][ch] = True
for f in cfg.FEATURE_CONFIG['emg_features']:
    cfg.FEATURE_CONFIG['emg_features'][f] = True
for f in cfg.FEATURE_CONFIG['imu_features']:
    cfg.FEATURE_CONFIG['imu_features'][f] = True

from model.data_loader import DataLoader
from model.model_archs.spatio_temporal_transformer import SpatioTemporalTransformerRegressor
from model.config_model import DATABASE_CONFIG, FEATURE_CONFIG as FEAT_CFG
from model.run_data_exporter import save_run_data
from model.run_model import calculate_per_seqlen_metrics
from model import deepshap_analysis

def main():
    print("1. Loading and filtering data...")
    loader = DataLoader()
    h5_paths = [p for p in DATABASE_CONFIG['segments_dir'].glob("*.h5") if not p.name.startswith("._")]
    df = loader.load_and_extract_features(h5_paths, is_sequence=True, use_precomputed=True)
    
    # Weight-class filter
    from model.data_loader import _EXCLUDED_TRUE_WEIGHTS
    if _EXCLUDED_TRUE_WEIGHTS and "weight" in df.columns:
        mask = ~df["weight"].isin(_EXCLUDED_TRUE_WEIGHTS)
        df = df[mask].reset_index(drop=True)
        
    X, y = loader.prepare_for_ml(df, target_col="weight")
    
    # Split
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=245, stratify=df["weight"].astype(str)
    )
    
    # Load model
    model_path = "/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/ST-transformer-par-spec-P01/spatio_temporal_transformer_model.joblib"
    regressor = SpatioTemporalTransformerRegressor.load(model_path)
    
    # Predict
    print("2. Generating test predictions...")
    preds = regressor.predict(X_test)
    
    # Calculate per_seqlen
    seq_col = 'sequence_dicts'
    if seq_col in df.columns:
        seq_lens = np.array([len(row) for row in df.loc[X_test.index, seq_col]])
    else:
        seq_lens = np.array([len(row) for row in X_test.iloc[:, 0]])
        
    per_seqlen = calculate_per_seqlen_metrics(y_test.values, preds, seq_lens)
    if per_seqlen:
        step = FEAT_CFG.get('window_step_sec', 0.1)
        max_win = max(FEAT_CFG.get('emg_window_size_sec', 0.15), FEAT_CFG.get('imu_window_size_sec', 0.3))
        for row in per_seqlen:
            row['TimeAtPrediction'] = f"{max_win + (row['SeqLen'] - 1) * step:.3f}s"
            
    # Computations for Retrospective Explainability
    run_dir = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/ST-transformer-par-spec-P01")
    
    print("3. Computing permutation channel importance...")
    perm_channel = regressor.permutation_importance(X_test, y_test, n_repeats=5, importance_type='channel')
    
    print("4. Computing permutation feature importance...")
    perm_feature = regressor.permutation_importance(X_test, y_test, n_repeats=5, importance_type='feature')
    
    perm_imp = {
        'channel': perm_channel,
        'feature': perm_feature
    }
    
    print("5. Computing DeepSHAP attribution (n_bg=100, n_exp=50)...")
    # This automatically saves deepshap_values.npz and PNGs into run_dir
    deepshap_analysis.run_deep_shap_analysis(regressor, X_train, X_test, run_dir, n_bg=100, n_exp=50)
    
    # Save fully compiled run data (it will read deepshap_values.npz from disk automatically!)
    print("6. Saving updated run_data.json...")
    metrics = {'MAE': 0.0945, 'RMSE': 0.1975, 'R2': 0.9895, 'Correlation': 0.9952}
    save_run_data(
        run_dir=run_dir,
        timestamp="20260507_103419",
        model_type="spatio_temporal_transformer",
        model=regressor,
        h5_paths=h5_paths,
        X=X, y=y, groups=df['subject'].values if 'subject' in df.columns else None, df=df,
        use_cv=False,
        cv_strategy="single_split",
        n_folds=0,
        avg_metrics=None,
        std_metrics=None,
        metrics=metrics,
        cv_metrics=None,
        participant_stats=None,
        per_seqlen_stats=per_seqlen,
        per_duration_stats=None,
        oof_predictions=None,
        y_pred=preds,
        y_test=y_test,
        perm_imp=perm_imp,
        all_histories=[regressor.loss_history] if regressor.loss_history else None,
        cv_val_participants=None,
        ablation_modality="all",
        is_raw_segment=False,
        is_sequence=True
    )
    print("7. Re-running run_viz.py for all plots...")
    os.system('python3 "visualization/run_viz.py" -r "model/model_results/ST-transformer-par-spec-P01" -o "model/model_results/ST-transformer-par-spec-P01" --plots regression seqlen importance_channel importance_feature')
    
    # Rename seqlen plot for standard naming consistency
    seqlen_src = run_dir / "seqlen_plot.png"
    seqlen_dst = run_dir / "seqlen_performance_plot.png"
    if seqlen_src.exists():
        if seqlen_dst.exists():
            seqlen_dst.unlink()
        seqlen_src.rename(seqlen_dst)
        
    print("\nRETROSPECT ANALYSIS COMPLETE!")

if __name__ == "__main__":
    main()
