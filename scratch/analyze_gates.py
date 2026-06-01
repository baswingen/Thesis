import torch
import numpy as np
from pathlib import Path
import pandas as pd
import sys

# Add project root to sys.path
sys.path.append(str(Path(__file__).resolve().parent.parent))

model_path = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/run_20260530_142426/all/spatio_temporal_transformer6_model.joblib")

if not model_path.exists():
    print(f"Model not found at {model_path}")
    sys.exit(1)

from model.model_archs.spatio_temporal_transformer6 import SpatioTemporalTransformerRegressor6

print("Loading model and scaler...")
regressor = SpatioTemporalTransformerRegressor6.load(model_path)
model = regressor.model
model.eval()

# Let's inspect the gate weights inside the model
print(f"\nModel Fusion Mode: {model.fusion_mode}")
print(f"Active Modality Groups: {model.active_groups}")

# Inspect the gate generator weights to see if they are biased or regularized
if hasattr(model, 'fusion_layer') and model.fusion_layer is not None:
    fusion = model.fusion_layer
    print("\n--- Gate Generator Architecture ---")
    print(fusion.gate_generator)
    
    # Analyze the output weights of the gate generator
    # We can inspect the biases of the last layer to see the default gate behavior
    last_layer = fusion.gate_generator[-2] # The Linear layer before Sigmoid
    if hasattr(last_layer, 'bias') and last_layer.bias is not None:
        biases = last_layer.bias.detach().cpu().numpy()
        # Biases are of shape [3 * d_model] = 288
        # Let's split them by modality
        d_model = model.d_model
        split_biases = np.split(biases, model.num_modalities)
        
        print("\n--- Modality Gate Biases (Pre-Sigmoid) ---")
        for gname, bias_vec in zip(model.active_groups, split_biases):
            # Compute sigmoid of the average bias as the default gate activation
            default_activation = 1.0 / (1.0 + np.exp(-bias_vec))
            print(f"  {gname}:")
            print(f"    Mean Bias: {bias_vec.mean():.4f}")
            print(f"    Std Bias:  {bias_vec.std():.4f}")
            print(f"    Mean Default Activation (Sigmoid): {default_activation.mean():.4f}")
            print(f"    Min / Max Default Activation:     {default_activation.min():.4f} / {default_activation.max():.4f}")
            
else:
    print("Model does not use a gated fusion layer.")
