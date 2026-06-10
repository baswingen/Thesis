import torch
import json
from pathlib import Path

path = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/ST-transformer-par-spec-cross-val/all/spatio_temporal_transformer3_model.joblib")

state = torch.load(path, map_location='cpu', weights_only=False)
print("Config in checkpoint:")
config = state.get('config', {})
print(json.dumps(config, indent=2))

print("\nModel state parameter counts:")
model_state = state.get('model_state', {})
for k, v in model_state.items():
    print(f"  {k}: shape={list(v.shape)}, numel={v.numel():,}")
