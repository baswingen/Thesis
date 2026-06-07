import os
import sys
import torch
import torch.nn as nn
import numpy as np
import pandas as pd
from pathlib import Path
from sklearn.model_selection import StratifiedKFold, train_test_split

project_root = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis")
sys.path.append(str(project_root))

from model.data_loader import DataLoader
from model.config_model import DATABASE_CONFIG, PARTICIPANT_CONFIG, FEATURE_CONFIG, SPATIO_TEMPORAL_TRANSFORMER3_CONFIG
from model.model_archs.spatio_temporal_transformer3 import SpatioTemporalTransformerRegressor3, pad_collate_fn, SequenceDataset

# Set device
device = torch.device("cpu") # use cpu for easier debugging

# Load data
loader = DataLoader()
h5_paths = [p for p in DATABASE_CONFIG["segments_dir"].glob("*.h5") if not p.name.startswith("._")]
df = loader.load_and_extract_features(h5_paths, is_sequence=True, use_precomputed=True)

# Split Fold 3
skf = StratifiedKFold(n_splits=5, shuffle=True, random_state=245)
strat_labels = df["weight"].astype(str)
cv_iterator = list(skf.split(df, strat_labels))

train_idx, test_idx = cv_iterator[2] # Fold 3 is index 2
df_train = df.iloc[train_idx]
df_test = df.iloc[test_idx]

# Instantiate regressor
regressor = SpatioTemporalTransformerRegressor3(**SPATIO_TEMPORAL_TRANSFORMER3_CONFIG)
regressor.device = device

# Prepare sequences just like fit()
X_train, X_val, y_train, y_val = train_test_split(
    df_train.drop(columns=["weight"]), df_train["weight"], 
    test_size=regressor.validation_split, random_state=regressor.random_state, 
    stratify=df_train["label"] if "label" in df_train.columns else None
)

sequences_train = regressor._extract_sequences(X_train)
sequences_val = regressor._extract_sequences(X_val)

train_lengths = [len(seq) for seq in sequences_train]
from collections import Counter
counts = Counter(train_lengths)
min_count = max(2, int(len(sequences_train) * 0.005))
valid_lengths = [l for l, c in counts.items() if c >= min_count]
regressor.max_seq_len = max(valid_lengths) if valid_lengths else max(train_lengths)

sequences_train = [seq[:regressor.max_seq_len] for seq in sequences_train]
sequences_val = [seq[:regressor.max_seq_len] for seq in sequences_val]

regressor.feature_names = []
seen_keys = set()
for i in range(min(10, len(sequences_train))):
    if sequences_train[i]:
        for w in sequences_train[i]:
            for k in w.keys():
                if k not in seen_keys:
                    regressor.feature_names.append(k)
                    seen_keys.add(k)

regressor._build_channel_indices()

flat_data_train = []
for seq in sequences_train:
    seq_arr = np.array([[w.get(k, 0.0) for k in regressor.feature_names] for w in seq])
    flat_data_train.append(seq_arr)
    
all_features_train = regressor._sanitise(np.vstack(flat_data_train))
regressor.scaler.fit(all_features_train)

scaled_seqs_val = []
for seq in sequences_val:
    seq_arr = np.array([[w.get(k, 0.0) for k in regressor.feature_names] for w in seq])
    scaled_arr = regressor.scaler.transform(regressor._sanitise(seq_arr)).astype(np.float32)
    scaled_seqs_val.append(scaled_arr)

scaled_seqs_val = [torch.from_numpy(a) for a in scaled_seqs_val]
y_tensor_val = torch.from_numpy(y_val.values.astype(np.float32)).unsqueeze(1)

# Initialize network
from model.model_archs.spatio_temporal_transformer3 import SpatioTemporalTransformerNetwork3
model = SpatioTemporalTransformerNetwork3(
    channel_indices=regressor.channel_indices,
    d_model=regressor.d_model, 
    nhead_spatial=regressor.nhead_spatial, 
    nhead_temporal=regressor.nhead_temporal,
    num_layers=regressor.num_layers,
    dim_feedforward=regressor.dim_feedforward, 
    dropout_rate=regressor.dropout_rate,
    use_checkpointing=regressor.use_checkpointing
).to(device)

model.eval()

# Let us run the validation loader batch by batch and print intermediate values in forward pass
dataset_val = SequenceDataset(scaled_seqs_val, y_tensor_val)
loader_val = torch.utils.data.DataLoader(dataset_val, batch_size=regressor.batch_size, shuffle=False, collate_fn=pad_collate_fn)

# Register hooks or add print statements to model/modules
# Let us override the forward of AttentionPooling, ModalityGroupedSTBlock, and SpatioTemporalTransformerNetwork3 to check for NaNs

def debug_pooling_forward(self, x):
    print("AttentionPooling Input has NaNs?", torch.isnan(x).any().item())
    attn_scores = self.attention(x)
    print("attn_scores has NaNs?", torch.isnan(attn_scores).any().item())
    attn_scores_clamped = torch.clamp(attn_scores, min=-65500, max=65500)
    attn_weights = torch.softmax(attn_scores_clamped, dim=1)
    print("attn_weights has NaNs?", torch.isnan(attn_weights).any().item())
    pooled = torch.sum(x * attn_weights, dim=1)
    print("pooled has NaNs?", torch.isnan(pooled).any().item())
    return pooled, attn_weights

model.attention_pooling.forward = debug_pooling_forward.__get__(model.attention_pooling, model.attention_pooling.__class__)

def debug_block_forward(self, x, causal_mask=None, padding_mask=None):
    print(f"STBlock Input has NaNs? {torch.isnan(x).any().item()}")
    B, T, C, d = x.shape
    x_spatial = x.view(B * T, C, d)
    spatial_out, _ = self.spatial_attn(x_spatial, x_spatial, x_spatial)
    spatial_out = spatial_out.view(B, T, C, d)
    print(f"  spatial_out has NaNs? {torch.isnan(spatial_out).any().item()}")
    
    temporal_out = torch.zeros_like(x, dtype=spatial_out.dtype)
    for gname, ch_indices in self.modality_groups.items():
        n_ch = len(ch_indices)
        group_x = x[:, :, ch_indices, :]
        group_x_flat = group_x.transpose(1, 2).contiguous().view(B * n_ch, T, d)
        
        group_padding_float = None
        if padding_mask is not None:
            group_padding = padding_mask.repeat_interleave(n_ch, dim=0)
            group_padding_float = torch.zeros_like(group_padding, dtype=x.dtype).masked_fill(group_padding, float("-inf"))
            print(f"    gname={gname}: group_padding_float has NaNs? {torch.isnan(group_padding_float).any().item()}")
            print(f"    gname={gname}: group_padding_float has Infs? {torch.isinf(group_padding_float).any().item()}")
            
        group_out, _ = self.temporal_attns[gname](
            group_x_flat, group_x_flat, group_x_flat,
            attn_mask=causal_mask,
            key_padding_mask=group_padding_float,
            is_causal=True if causal_mask is not None else False
        )
        print(f"    gname={gname}: group_out has NaNs? {torch.isnan(group_out).any().item()}")
        
        group_out = group_out.view(B, n_ch, T, d).transpose(1, 2).contiguous()
        temporal_out[:, :, ch_indices, :] = group_out.to(temporal_out.dtype)
        
    attn_out = self.dropout(spatial_out + temporal_out)
    x = self.norm1(x + attn_out)
    print(f"  After attention & norm1, has NaNs? {torch.isnan(x).any().item()}")
    
    ffn_out = self.dropout(self.ffn(x))
    x = self.norm2(x + ffn_out)
    print(f"  After FFN & norm2, has NaNs? {torch.isnan(x).any().item()}")
    return x

for i, block in enumerate(model.blocks):
    block.forward = debug_block_forward.__get__(block, block.__class__)

print("\n--- Starting validation forward pass ---")
with torch.no_grad():
    for batch_idx, (batch_x, batch_y, lengths) in enumerate(loader_val):
        print(f"\n--- Batch {batch_idx} ---")
        print("batch_x has NaNs?", torch.isnan(batch_x).any().item())
        print("lengths:", lengths)
        outputs = model(batch_x, lengths)
        print("outputs has NaNs?", torch.isnan(outputs).any().item())
        print("outputs:", outputs[:5].flatten())
        if torch.isnan(outputs).any():
            break
