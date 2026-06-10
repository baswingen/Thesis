import torch
import sys
import copy
from pathlib import Path

# Add project root to sys.path
sys.path.append(str(Path(__file__).resolve().parent.parent))

from model.model_archs.spatio_temporal_transformer3 import SpatioTemporalTransformerRegressor3, SpatioTemporalTransformerNetwork3
from model import performance_utils

path = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/ST-transformer-par-spec-cross-val/all/spatio_temporal_transformer3_model.joblib")

print("Loading model from:", path)
try:
    state = torch.load(path, map_location='cpu', weights_only=False)
    config = copy.deepcopy(state['config'])
    
    if 'num_layers' not in config:
        config['num_layers'] = max(config.get('num_layers_spatial', 3), config.get('num_layers_temporal', 3))
        
    regressor = SpatioTemporalTransformerRegressor3(**config)
    regressor.device = torch.device("cpu")
    regressor.scaler = state['scaler']
    regressor.feature_names = state['feature_names']
    regressor.channel_indices = state['channel_indices']
    
    regressor.model = SpatioTemporalTransformerNetwork3(
        channel_indices=state['channel_indices'],
        d_model=state['config']['d_model'], 
        nhead_spatial=state['config']['nhead_spatial'], 
        nhead_temporal=state['config']['nhead_temporal'],
        num_layers=regressor.num_layers,
        dim_feedforward=state['config']['dim_feedforward'],
        dropout_rate=state['config']['dropout_rate'],
        use_checkpointing=state['config'].get('use_checkpointing', False)
    ).to(regressor.device)
    
    regressor.model.load_state_dict(state['model_state'])
    regressor.model.eval()
    
    # Let's run a custom loop to print FLOPs contributed by each submodule
    total_flops = 0
    print("\n--- FLOPs Contribution Breakdown ---")
    for name, module in regressor.model.named_modules():
        # Only print leaf-level modules that are actually counted by performance_utils.estimate_flops
        if isinstance(module, torch.nn.Linear):
            # 2 * in * out
            flops = 2 * module.in_features * module.out_features
            total_flops += flops
            print(f"Linear Module: {name:<50} | In: {module.in_features:>4} | Out: {module.out_features:>4} | FLOPs: {flops:>10,}")
            
    print(f"Total FLOPs counted: {total_flops:,} ({total_flops / 1e6:.6f} MFLOPs / {total_flops / 1e9:.9f} GFLOPs)")
    
except Exception as e:
    import traceback
    traceback.print_exc()
