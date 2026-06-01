import torch
import pandas as pd
import numpy as np
from pathlib import Path
import sys

sys.path.append(str(Path(__file__).resolve().parent.parent))

from model.model_archs.spatio_temporal_transformer6 import SpatioTemporalTransformerRegressor6

print("--- VERIFYING SST6 Bottleneck Option B Model ---")

# 1. Create a dummy dataset (20 samples, 24 windows, 12 features)
np.random.seed(42)
n_samples = 20
max_len = 24
n_features = 12

# Generate feature names (e.g. 6 EMG and 6 IMU)
feature_names = [f"Ch{i}_EMG_RMS" for i in range(6)] + [f"ax1_IMU_Mean", "ay1_IMU_Mean", "az1_IMU_Mean", "roll_rad1_IMU_Mean", "pitch_rad1_IMU_Mean", "yaw_rad1_IMU_Mean"]

sequences = []
for _ in range(n_samples):
    seq_len = np.random.randint(10, max_len + 1)
    seq = []
    for _ in range(seq_len):
        w = {fname: np.random.randn() for fname in feature_names}
        seq.append(w)
    sequences.append(seq)

X = pd.DataFrame({"sequence_dicts": sequences, "subject": ["P01"] * 10 + ["P02"] * 10})
y = pd.Series(np.random.randint(0, 6, size=n_samples))

# 2. Instantiate SST6 Regressor with Gated Feature (Option B)
config = {
    'd_model': 32,
    'nhead_spatial': 2,
    'num_layers': 2,
    'nhead_temporal': 2,
    'dim_feedforward': 64,
    'dropout_rate': 0.1,
    'learning_rate': 0.001,
    'batch_size': 4,
    'epochs': 2,
    'fusion_mode': 'gated_feature',
    'random_state': 42
}

print("Initializing model...")
regressor = SpatioTemporalTransformerRegressor6(**config)

print("\nFitting model on dummy data...")
try:
    regressor.fit(X, y)
    print("  SUCCESS: Model fitted successfully!")
except Exception as e:
    print(f"  FAILED: Model fit failed: {e}")
    sys.exit(1)

print("\nPredicting on dummy data...")
try:
    preds = regressor.predict(X)
    print(f"  SUCCESS: Predictions generated! Shape: {preds.shape}")
except Exception as e:
    print(f"  FAILED: Prediction failed: {e}")
    sys.exit(1)

# 3. Verify Serialization
temp_model_path = Path("scratch/temp_verify_sst6.joblib")
print(f"\nSaving model to {temp_model_path}...")
try:
    regressor.save(temp_model_path)
    print("  SUCCESS: Model saved!")
except Exception as e:
    print(f"  FAILED: Save failed: {e}")
    sys.exit(1)

print(f"Loading model from {temp_model_path}...")
try:
    loaded_regressor = SpatioTemporalTransformerRegressor6.load(temp_model_path)
    loaded_preds = loaded_regressor.predict(X)
    assert np.allclose(preds, loaded_preds)
    print("  SUCCESS: Model loaded and predictions match exactly!")
except Exception as e:
    print(f"  FAILED: Load failed: {e}")
    sys.exit(1)
finally:
    if temp_model_path.exists():
        temp_model_path.unlink()

print("\n=== VERIFICATION COMPLETE: ALL PASS! ===")
