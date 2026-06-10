import torch
from pathlib import Path

path = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/ST-transformer-par-spec-cross-val/all/spatio_temporal_transformer3_model.joblib")

print("Loading model from:", path)
try:
    state = torch.load(path, map_location='cpu', weights_only=False)
    print("Loaded keys:", list(state.keys()))
    if 'model_state' in state:
        model_state = state['model_state']
        total = sum(p.numel() for p in model_state.values())
        print(f"Total params in model_state: {total:,}")
        print("\nNamed parameters and their shapes (first 25):")
        for name, param in list(model_state.items())[:25]:
            print(f"  {name}: {param.shape} (numel={param.numel():,})")
        if len(model_state) > 25:
            print(f"  ... and {len(model_state) - 25} more parameters.")
    else:
        print("model_state not found in state dict.")
except Exception as e:
    print("Error:", e)
