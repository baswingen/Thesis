import json
from pathlib import Path

runs = {
    "ST-transformer-par-spec-cross-val": Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/ST-transformer-par-spec-cross-val/run_data.json"),
    "final_run": Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/final_run/run_data.json")
}

for name, path in runs.items():
    print("=" * 85)
    print(f"RUN: {name}")
    print("=" * 85)
    if not path.exists():
        print(f"File not found: {path}")
        continue
    
    with open(path, "r") as f:
        data = json.load(f)
        
    for modality in ['all', 'emg_only', 'imu_only']:
        print(f"\n---> MODALITY: {modality}")
        m_data = data.get(modality, {})
        if not m_data:
            print("No data for this modality")
            continue
            
        meta = m_data.get("meta", {})
        print(f"  Model Type: {meta.get('model_type')}, Timestamp: {meta.get('timestamp')}")
        print(f"  Raw Segment: {meta.get('is_raw_segment')}, Sequence: {meta.get('is_sequence')}")
        
        m_info = m_data.get("model_info", {})
        print(f"  Params - Total: {m_info.get('total_parameters'):,}, Trainable: {m_info.get('trainable_parameters'):,}")
        print(f"  Estimated GFLOPS: {m_info.get('estimated_gflops')}, Device: {m_info.get('device')}")
        
        timing = m_data.get("timing", {})
        if timing:
            print(f"  Timing - Avg Train: {timing.get('avg_train_time_sec')}s")
            print(f"  Timing - Avg Inference Total: {timing.get('avg_inference_time_total_sec')}s")
            print(f"  Timing - Avg Inference Per Sample: {timing.get('avg_inference_per_sample_sec')}s")
        else:
            print("  Timing: None")
            
        eval_section = m_data.get("evaluation", {})
        macro = eval_section.get("macro_avg")
        if macro:
            print(f"  Evaluation (Macro Average): MAE={macro.get('MAE'):.4f} +/- {macro.get('MAE_std'):.4f}, RMSE={macro.get('RMSE'):.4f}, R2={macro.get('R2'):.4f} +/- {macro.get('R2_std'):.4f}")
        pooled = eval_section.get("pooled")
        if pooled:
            print(f"  Evaluation (Pooled): MAE={pooled.get('MAE'):.4f}, RMSE={pooled.get('RMSE'):.4f}, R2={pooled.get('R2'):.4f}, Corr={pooled.get('correlation'):.4f}")
            
        dataset = m_data.get("dataset", {})
        print(f"  Dataset: Total Samples={dataset.get('total_samples')}, N Participants={dataset.get('n_participants')}")
