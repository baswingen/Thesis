import torch
import sys
import copy
from pathlib import Path

# Add project root to sys.path
sys.path.append(str(Path(__file__).resolve().parent.parent))

from model.model_archs.spatio_temporal_transformer3 import SpatioTemporalTransformerRegressor3, SpatioTemporalTransformerNetwork3
from model import performance_utils

path = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/final_run/all/spatio_temporal_transformer3_model.joblib")

print("Loading final_run model from:", path)
try:
    state = torch.load(path, map_location='cpu', weights_only=False)
    config = copy.deepcopy(state['config'])
    
    if 'num_layers' not in config:
        config['num_layers'] = max(config.get('num_layers_spatial', 3), config.get('num_layers_temporal', 3))
        
    # Instantiate the regressor on CPU
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
    print("Model loaded successfully on CPU!")
    
    total, trainable = performance_utils.count_parameters(regressor.model)
    print(f"performance_utils.count_parameters(regressor.model) -> Total: {total:,}, Trainable: {trainable:,}")
    
    # State dict info
    print("State dict keys count:", len(regressor.model.state_dict()))
    state_dict_total = sum(p.numel() for p in regressor.model.state_dict().values())
    print(f"Sum of state_dict values numel -> {state_dict_total:,}")
    
except Exception as e:
    print("Error:", e)
