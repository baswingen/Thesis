import os
import json
import h5py
import numpy as np
from pathlib import Path

def main():
    # 1. Load predictions
    run_dir = Path('model/model_results/final_run_lopo_2')
    run_data_path = run_dir / 'run_data.json'
    
    with open(run_data_path, 'r') as f:
        run_data = json.load(f)
        
    preds = run_data['all']['predictions']
    y_true = np.asarray(preds['y_true'])
    y_pred = np.asarray(preds['y_pred'])
    part = np.asarray(preds['participant'])
    h5_files = run_data['all']['dataset']['h5_files']
    
    # 2. Load blacklist
    blacklist_path = Path('data_analysis/results/blacklist.json')
    blacklist = set()
    if blacklist_path.exists():
        with open(blacklist_path, 'r') as f:
            entries = json.load(f)
        blacklist = {(e['file'], e['segment']) for e in entries}
        
    # 3. Load all segments grouped by participant (alphabetically sorted)
    segments_dir = Path('/Volumes/Laurens SSD/BasData/segments')
    
    TRUE_WEIGHTS = {0.75: 0.90, 1.0: 0.98, 2.0: 1.97, 2.25: 2.24, 3.0: 2.95, 4.25: 4.15, 6.0: 5.93}
    excluded_weights = {0.75, 2.25}
    
    # Get all h5 files
    all_files = [f.name for f in segments_dir.glob('*.h5') if not f.name.startswith('._')]
    
    # Group files by participant
    files_by_part = {}
    for f in all_files:
        p = f.split('_')[1]
        files_by_part.setdefault(p, []).append(f)
        
    # Find the exact index of each file in run_data.json's h5_files list to preserve loading order
    file_order_map = {name: idx for idx, name in enumerate(h5_files)}
    
    # Sort files of each participant according to their position in h5_files
    for p in files_by_part:
        files_by_part[p] = sorted(files_by_part[p], key=lambda f: file_order_map.get(f, 999))
        
    # Load segments participant by participant alphabetically
    loaded_segments = []
    include_participants = sorted(list(files_by_part.keys()))
    if 'P13' in include_participants:
        include_participants.remove('P13')
        
    for p in include_participants:
        p_files = files_by_part[p]
        for fname in p_files:
            fp = segments_dir / fname
            with h5py.File(fp, 'r') as f:
                for seg_key in sorted(f.keys()):
                    if not seg_key.startswith('segment_'):
                        continue
                    if (fname, seg_key) in blacklist:
                        continue
                    grp = f[seg_key]
                    
                    label = grp.attrs.get('label')
                    if isinstance(label, bytes):
                        label = label.decode('utf-8')
                    nominal_w = 0.0 if label == 'free_movement' else float(grp.attrs.get('weight', -1.0))
                    if nominal_w in excluded_weights:
                        continue
                    weight = 0.0 if label == 'free_movement' else TRUE_WEIGHTS.get(nominal_w, nominal_w)
                    
                    state = grp.attrs.get('state')
                    if isinstance(state, bytes):
                        state = state.decode('utf-8')
                        
                    src_row = int(grp.attrs.get('src_row', 0))
                    src_col = int(grp.attrs.get('src_col', 0))
                    tgt_row = int(grp.attrs.get('tgt_row', 0))
                    tgt_col = int(grp.attrs.get('tgt_col', 0))
                    t_start = float(grp.attrs.get('t_start', 0.0))
                    t_end = float(grp.attrs.get('t_end', 0.0))
                    
                    duration = t_end - t_start
                    dx = tgt_col - src_col
                    dy = tgt_row - src_row
                    distance_cm = np.sqrt(dx**2 + dy**2) * 15.0
                    
                    loaded_segments.append({
                        'participant': p,
                        'filename': fname,
                        'segment_id': seg_key,
                        'state': state,
                        'weight': weight,
                        'duration': duration,
                        'distance_cm': distance_cm
                    })
                    
    print(f"Predictions length: {len(y_true)}")
    print(f"Loaded segments length: {len(loaded_segments)}")
    
    # Check element-by-element match of weight and participant
    mismatch_count = 0
    first_mismatches = []
    for idx in range(min(len(loaded_segments), len(y_true))):
        ls = loaded_segments[idx]
        y_t = y_true[idx]
        p_t = part[idx]
        
        weight_match = np.isclose(ls['weight'], y_t)
        part_match = (ls['participant'] == p_t)
        
        if not (weight_match and part_match):
            mismatch_count += 1
            if len(first_mismatches) < 10:
                first_mismatches.append((idx, ls['participant'], p_t, ls['weight'], y_t))
                
    print(f"Mismatches: {mismatch_count} / {min(len(loaded_segments), len(y_true))}")
    if first_mismatches:
        print("First few mismatches (idx, loaded_part, pred_part, loaded_weight, pred_weight):")
        for m in first_mismatches:
            print(m)
            
if __name__ == '__main__':
    main()
