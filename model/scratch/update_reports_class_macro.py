import json
import sys
import re
from pathlib import Path
import numpy as np

# Add project root to path
sys.path.append(str(Path(__file__).resolve().parent.parent.parent))

from model.macro_metrics import class_macro, class_participant_macro, format_macro_report, compute_all_macro_metrics
from model.run_model import calculate_per_seqlen_metrics, calculate_per_duration_metrics

MODALITIES = ["all", "emg_only", "imu_only"]

# We will load the dataset alignment once.
print("Loading dataset for alignment...")
from model.data_loader import DataLoader
from model.run_model import load_and_prepare_data
from model.config_model import DATABASE_CONFIG

loader = DataLoader()
h5_paths = [p for p in DATABASE_CONFIG['segments_dir'].glob("*.h5") if not p.name.startswith("._")]
X, y, groups, df = load_and_prepare_data(loader, h5_paths, 'spatio_temporal_transformer3', is_raw_segment=False, is_sequence=True, use_precomputed=True)

# Extract sequence lengths
seq_col = 'sequence_dicts'
if seq_col in df.columns:
    seq_lens = np.array([len(row) for row in df[seq_col]])
else:
    seq_lens = np.array([len(row) for row in X.iloc[:, 0]])

# Extract durations
if 'segment_duration_sec' in df.columns:
    durations = df['segment_duration_sec'].values
else:
    durations = None

print(f"Dataset loaded. Total samples: {len(y)}")

def update_report_file(report_path, macro_report_str, participant_stats, per_seqlen, per_dur):
    if not report_path.exists():
        return
    content = report_path.read_text()
    
    # 1. Replace or insert the macro metrics block.
    # Remove any old macro averaged metrics block
    content = re.sub(
        r'--- MACRO-AVERAGED METRICS \(BALANCED\) ---.*?(?=\n--- PER-WEIGHT METRICS ---)', 
        macro_report_str.strip() + "\n\n", 
        content, 
        flags=re.DOTALL
    )
    # Remove any old 'METHOD: CLASS-MACRO-AVERAGE (New)' block
    content = re.sub(
        r'\nMETHOD: CLASS-MACRO-AVERAGE \(New\).*?(?=\n\n--- PER-WEIGHT METRICS ---|\n--- PER-WEIGHT METRICS ---)', 
        "", 
        content, 
        flags=re.DOTALL
    )
    # If '--- MACRO-AVERAGED METRICS (BALANCED) ---' was not present, let's insert it before '--- PER-WEIGHT METRICS ---'
    if "--- MACRO-AVERAGED METRICS (BALANCED) ---" not in content:
        content = content.replace("--- PER-WEIGHT METRICS ---", macro_report_str.strip() + "\n\n--- PER-WEIGHT METRICS ---")
        
    # 2. Replace '--- PER-PARTICIPANT METRICS ---' section
    part_lines = ["--- PER-PARTICIPANT METRICS ---"]
    part_lines.append(f"{'Participant':<12} | {'Samples':<8} | {'MAE':<10} | {'RMSE':<10} | {'R2':<10}")
    part_lines.append("-" * 63)
    for p in participant_stats:
        r2_val = p.get('R2', float('nan'))
        r2_str = f"{r2_val:<10.4f}" if (r2_val is not None and not np.isnan(r2_val)) else f"{'nan':<10}"
        mae_val = p.get('MAE', float('nan'))
        rmse_val = p.get('RMSE', float('nan'))
        mae_str = f"{mae_val:<10.4f}" if (mae_val is not None and not np.isnan(mae_val)) else f"{'nan':<10}"
        rmse_str = f"{rmse_val:<10.4f}" if (rmse_val is not None and not np.isnan(rmse_val)) else f"{'nan':<10}"
        part_lines.append(f"{p['Participant']:<12} | {p['Samples']:<8} | {mae_str} | {rmse_str} | {r2_str}")
    part_lines.append("")
    
    new_part_section = "\n".join(part_lines)
    content = re.sub(
        r'--- PER-PARTICIPANT METRICS ---.*?(?=\n--- PER-SEQUENCE-LENGTH METRICS ---|\n--- COMPUTE & TIMING METRICS ---|\n--- PER-SEGMENT-DURATION METRICS ---|\n$)',
        new_part_section,
        content,
        flags=re.DOTALL
    )
    
    # 3. Replace '--- PER-SEQUENCE-LENGTH METRICS ---' section
    if per_seqlen:
        seq_lines = ["--- PER-SEQUENCE-LENGTH METRICS ---"]
        seq_lines.append(f"{'SeqLen':<12} | {'TimeAtPrediction':<16} | {'Count':<8} | {'MAE':<10} | {'RMSE':<10}")
        seq_lines.append("-" * 63)
        for s in per_seqlen:
            mae_str = f"{float(s['MAE']):<10.4f}" if s.get('MAE') not in [None, 'nan'] else f"{'nan':<10}"
            rmse_str = f"{float(s['RMSE']):<10.4f}" if s.get('RMSE') not in [None, 'nan'] else f"{'nan':<10}"
            seq_lines.append(f"{s['SeqLen']:<12} | {s['TimeAtPrediction']:<16} | {s['Count']:<8} | {mae_str} | {rmse_str}")
        seq_lines.append("")
        
        new_seq_section = "\n".join(seq_lines)
        content = re.sub(
            r'--- PER-SEQUENCE-LENGTH METRICS ---.*?(?=\n--- COMPUTE & TIMING METRICS ---|\n--- PER-SEGMENT-DURATION METRICS ---|\n$)',
            new_seq_section,
            content,
            flags=re.DOTALL
        )
        
    # 4. Replace '--- PER-SEGMENT-DURATION METRICS (CNN-LSTM) ---' section
    if per_dur:
        dur_lines = ["--- PER-SEGMENT-DURATION METRICS (CNN-LSTM) ---"]
        dur_lines.append("(Prediction error vs. elapsed time into lift, binned from raw segment duration)")
        dur_lines.append(f"{'Bin':<6} | {'Time into lift (mid)':<22} | {'Count':<8} | {'MAE':<10} | {'RMSE':<10}")
        dur_lines.append("-" * 68)
        for s in per_dur:
            mae_str = f"{float(s['MAE']):<10.4f}" if s.get('MAE') not in [None, 'nan'] else f"{'nan':<10}"
            rmse_str = f"{float(s['RMSE']):<10.4f}" if s.get('RMSE') not in [None, 'nan'] else f"{'nan':<10}"
            dur_lines.append(f"{s['SeqLen']:<6} | {s['TimeAtPrediction']:<22} | {s['Count']:<8} | {mae_str} | {rmse_str}")
        dur_lines.append("")
        
        new_dur_section = "\n".join(dur_lines)
        content = re.sub(
            r'--- PER-SEGMENT-DURATION METRICS \(CNN-LSTM\) ---.*?(?=\n--- COMPUTE & TIMING METRICS ---|\n$)',
            new_dur_section,
            content,
            flags=re.DOTALL
        )
        
    report_path.write_text(content)
    print(f"Updated report text file: {report_path}")

def process_node(node):
    """Update participant and seqlen/duration stats for a single run node in JSON."""
    predictions = node.get("predictions")
    if not predictions or not isinstance(predictions, dict):
        return False
        
    y_true = np.array(predictions.get("y_true"), dtype=float)
    y_pred = np.array(predictions.get("y_pred"), dtype=float)
    
    if len(y_true) != len(y):
        print(f"[WARN] Length mismatch: predictions has {len(y_true)} samples, dataset has {len(y)} samples. Skipping node.")
        return False
        
    # Recalculate participant stats
    participant_stats = []
    for p in np.unique(groups):
        p_mask = (groups == p)
        if np.any(p_mask):
            p_y_true = y_true[p_mask]
            p_y_pred = y_pred[p_mask]
            cm = class_macro(p_y_true, p_y_pred)
            participant_stats.append({
                'Participant': p,
                'MAE': cm["MAE"],
                'RMSE': cm["RMSE"],
                'R2': cm["R2"],
                'Samples': int(np.sum(p_mask))
            })
    participant_stats = sorted(participant_stats, key=lambda x: x['Participant'])
    
    # Recalculate sequence length stats
    per_seqlen = calculate_per_seqlen_metrics(y_true, y_pred, seq_lens, participants=groups)
    # Fill TimeAtPrediction
    step = 0.1
    max_win = 0.3
    for row in per_seqlen:
        row['TimeAtPrediction'] = f"{max_win + (row['SeqLen'] - 1) * step:.3f}s"
        
    # Recalculate duration stats (if durations are present)
    per_dur = None
    if durations is not None:
        per_dur = calculate_per_duration_metrics(y_true, y_pred, durations, participants=groups)
        
    # Calculate overall class-macro metrics
    overall_cm = class_macro(y_true, y_pred)
    
    # Write back to node
    if "evaluation" not in node or node["evaluation"] is None:
        node["evaluation"] = {}
        
    node["evaluation"]["class_macro_avg"] = {
        "MAE": overall_cm["MAE"],
        "RMSE": overall_cm["RMSE"],
        "R2": overall_cm["R2"]
    }
    
    # Also calculate all macro metrics block
    macro_metrics = compute_all_macro_metrics(y_true, y_pred, participants=groups)
    node["evaluation"]["macro_metrics"] = macro_metrics
    node["evaluation"]["per_participant"] = participant_stats
    node["evaluation"]["per_seqlen"] = per_seqlen
    node["evaluation"]["per_duration"] = per_dur
    
    # Also ensure predictions contains participant
    predictions["participant"] = groups.tolist()
    
    # Format macro report string for txt file
    macro_report_str = format_macro_report(macro_metrics)
    
    return participant_stats, per_seqlen, per_dur, macro_report_str

def update_run_data_json(filepath):
    if not filepath.exists():
        return None
    d = json.loads(filepath.read_text())
    updated = False
    
    # Check if this is a consolidated json
    is_consolidated = False
    results = {}
    for mod in MODALITIES:
        if mod in d and isinstance(d[mod], dict) and ("predictions" in d[mod] or "evaluation" in d[mod]):
            is_consolidated = True
            node = d[mod]
            res = process_node(node)
            if res:
                results[mod] = res # (participant_stats, per_seqlen, per_dur, macro_report_str)
                updated = True
                
    if not is_consolidated:
        res = process_node(d)
        if res:
            results["single"] = res
            updated = True
            
    if updated:
        # Use custom Encoder or default to JSON serialization
        class NumpyEncoder(json.JSONEncoder):
            def default(self, obj):
                if isinstance(obj, (np.integer,)): return int(obj)
                if isinstance(obj, (np.floating,)): return None if (np.isnan(obj) or np.isinf(obj)) else float(obj)
                if isinstance(obj, np.ndarray): return obj.tolist()
                return super().default(obj)
                
        filepath.write_text(json.dumps(d, indent=2, cls=NumpyEncoder))
        print(f"Updated JSON: {filepath}")
    return results

def main():
    target_dirs = [
        "model/model_results/final_run",
        "model/model_results/ST-transformer-par-spec-cross-val",
        "model/model_results/ST-transformer-par-spec-cross-val2"
    ]
    
    for tdir_str in target_dirs:
        tdir = Path(tdir_str).resolve()
        if not tdir.exists():
            print(f"Directory not found: {tdir}")
            continue
            
        print(f"\nProcessing directory: {tdir.name}")
        
        # 1. Update consolidated root JSON
        root_json = tdir / "run_data.json"
        root_metrics = update_run_data_json(root_json)
        
        # 2. Update individual modality folders
        for mod in MODALITIES:
            mod_dir = tdir / mod
            if not mod_dir.exists():
                continue
                
            mod_json = mod_dir / "run_data.json"
            mod_metrics = update_run_data_json(mod_json)
            
            # Retrieve metrics for report
            part_stats, per_seqlen, per_dur, macro_report_str = None, None, None, None
            if mod_metrics and "single" in mod_metrics:
                part_stats, per_seqlen, per_dur, macro_report_str = mod_metrics["single"]
            elif root_metrics and mod in root_metrics:
                part_stats, per_seqlen, per_dur, macro_report_str = root_metrics[mod]
                
            # If we have metrics, update the performance_report.txt
            if part_stats is not None:
                report_path = mod_dir / "performance_report.txt"
                update_report_file(report_path, macro_report_str, part_stats, per_seqlen, per_dur)
                
                # Also check for performance_report_CALIBRATED.txt
                cal_report_path = mod_dir / "performance_report_CALIBRATED.txt"
                if cal_report_path.exists():
                    update_report_file(cal_report_path, macro_report_str, part_stats, per_seqlen, per_dur)

if __name__ == "__main__":
    main()
