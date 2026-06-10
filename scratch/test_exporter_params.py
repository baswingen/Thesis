import torch
import sys
import copy
from pathlib import Path

# Add project root to sys.path
sys.path.append(str(Path(__file__).resolve().parent.parent))

from model.model_archs.spatio_temporal_transformer3 import SpatioTemporalTransformerRegressor3, SpatioTemporalTransformerNetwork3
import model.performance_utils as pu

path = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/ST-transformer-par-spec-cross-val/all/spatio_temporal_transformer3_model.joblib")

try:
    state = torch.load(path, map_location='cpu', weights_only=False)
    config = copy.deepcopy(state['config'])
    if 'num_layers' not in config:
        config['num_layers'] = max(config.get('num_layers_spatial', 3), config.get('num_layers_temporal', 3))
        
    net = SpatioTemporalTransformerNetwork3(
        channel_indices=state['channel_indices'],
        d_model=state['config']['d_model'], 
        nhead_spatial=state['config']['nhead_spatial'], 
        nhead_temporal=state['config']['nhead_temporal'],
        num_layers=config['num_layers'],
        dim_feedforward=state['config']['dim_feedforward'],
        dropout_rate=state['config']['dropout_rate'],
        use_checkpointing=state['config'].get('use_checkpointing', False)
    )
    net.load_state_dict(state['model_state'])
    net.eval()
    
    total, trainable = pu.count_parameters(net)
    print("Direct count_parameters call total:", total)
    
except Exception as e:
    print("Error:", e)
