import torch
import numpy as np
from pathlib import Path
import pandas as pd
import sys

sys.path.append(str(Path(__file__).resolve().parent.parent))

from model.data_loader import DataLoader
from model.model_archs.spatio_temporal_transformer6 import SpatioTemporalTransformerRegressor6, pad_collate_fn, SequenceDataset
from model.config_model import DATABASE_CONFIG, USE_PRECOMPUTED_FEATURES

model_path = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/run_20260530_142426/all/spatio_temporal_transformer6_model.joblib")

if not model_path.exists():
    print(f"Model not found at {model_path}")
    sys.exit(1)

print("Loading model...")
regressor = SpatioTemporalTransformerRegressor6.load(model_path)
model = regressor.model
model.eval()

print("Loading dataset...")
loader = DataLoader()
h5_paths = [p for p in DATABASE_CONFIG['segments_dir'].glob("*.h5") if not p.name.startswith("._")]
X, y, groups, df = loader.load_and_extract_features(
    h5_paths,
    is_sequence=True,
    use_precomputed=True
), None, None, None

# Filter weight classes just like run_model.py
from model.data_loader import _EXCLUDED_TRUE_WEIGHTS
if _EXCLUDED_TRUE_WEIGHTS and "weight" in X.columns:
    mask = ~X["weight"].isin(_EXCLUDED_TRUE_WEIGHTS)
    X = X[mask].reset_index(drop=True)

groups = X["subject"].astype(str).values if "subject" in X.columns else None
y = X["weight"]
X_df = X.drop(columns=["label", "state", "weight", "segment_id", "trial_file", "subject", "emg_fs", "emg_fs_orig", "imu_fs", "imu_fs_orig"], errors='ignore')

print(f"Loaded {len(X_df)} samples.")

# Prepare sequences
sequences = regressor._extract_sequences(X_df)
if regressor.max_seq_len is not None:
    sequences = [seq[:regressor.max_seq_len] for seq in sequences]

scaled_seqs = []
for seq in sequences:
    seq_arr = np.array([[w.get(k, 0.0) for k in regressor.feature_names] for w in seq])
    scaled_arr = regressor.scaler.transform(regressor._sanitise(seq_arr)).astype(np.float32)
    scaled_seqs.append(torch.from_numpy(scaled_arr))

dummy_y = torch.zeros((len(scaled_seqs), 1))
dataset = SequenceDataset(scaled_seqs, dummy_y)
data_loader = torch.utils.data.DataLoader(dataset, batch_size=64, shuffle=False, collate_fn=pad_collate_fn)

print("Running inference and extracting gate weights...")
all_gates = []
with torch.no_grad():
    for batch_x, _, lengths in data_loader:
        batch_x = batch_x.to(regressor.device)
        lengths = lengths.to(regressor.device)
        _ = model(batch_x, lengths)
        # Capture the gates from model.last_gates
        gates = model.last_gates # shape: [B, N * d_model] = [B, 3 * 96]
        all_gates.append(gates)

all_gates = np.vstack(all_gates) # [Total_Samples, 288]
d_model = model.d_model

# Split gates by modality
split_gates = np.split(all_gates, model.num_modalities, axis=1) # list of 3 arrays of shape [Total_Samples, d_model]

# Compute mean gate weight per sample for each modality
mean_gates = {}
for gname, gates_arr in zip(model.active_groups, split_gates):
    mean_gates[gname] = gates_arr.mean(axis=1) # [Total_Samples]

gates_df = pd.DataFrame(mean_gates)
gates_df['subject'] = groups
gates_df['weight'] = y.values

print("\n=== OVERALL MEAN GATE WEIGHTS ===")
for gname in model.active_groups:
    print(f"  {gname}: {gates_df[gname].mean():.4f} (±{gates_df[gname].std():.4f})")

print("\n=== MEAN GATES PER PARTICIPANT ===")
participant_breakdown = gates_df.groupby('subject')[model.active_groups].mean()
print(participant_breakdown.to_string())

print("\n=== MEAN GATES PER WEIGHT CLASS ===")
weight_breakdown = gates_df.groupby('weight')[model.active_groups].mean()
print(weight_breakdown.to_string())
