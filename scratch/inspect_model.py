import torch
from pathlib import Path

model_path = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/run_20260530_142426/all/spatio_temporal_transformer6_model.joblib")

if not model_path.exists():
    print(f"Model not found at {model_path}")
else:
    print(f"Loading model from {model_path}...")
    try:
        # Load the saved state dict/metadata
        state = torch.load(model_path, map_location='cpu', weights_only=False)
        
        print("\n--- Model Metadata ---")
        print(f"Keys in saved state: {list(state.keys())}")
        
        if 'split_info' in state:
            print(f"Split info: {state['split_info']}")
            
        if 'config' in state:
            print(f"Config keys: {list(state['config'].keys())}")
            print(f"Fusion mode: {state['config'].get('fusion_mode')}")
            print(f"Max seq len: {state['config'].get('max_seq_len')}")
            
        if 'channel_indices' in state:
            print(f"\n--- Channel Indices ({len(state['channel_indices'])} channels) ---")
            for ch, indices in sorted(state['channel_indices'].items()):
                print(f"  {ch}: {len(indices)} features (e.g. index {indices[0]})")
                
        if 'feature_names' in state:
            feats = state['feature_names']
            print(f"\n--- Feature Names ({len(feats)} features total) ---")
            print(f"First 10 features: {feats[:10]}")
            print(f"Last 10 features: {feats[-10:]}")
            
            # Check for IMU and EMG count
            emg_feats = [f for f in feats if '_EMG_' in f]
            imu_feats = [f for f in feats if '_IMU_' in f]
            svm_feats = [f for f in feats if '_SVM_' in f]
            print(f"\nEMG features count: {len(emg_feats)}")
            print(f"IMU features count: {len(imu_feats)}")
            print(f"SVM features count: {len(svm_feats)}")
            
    except Exception as e:
        print(f"Failed to inspect model: {e}")
