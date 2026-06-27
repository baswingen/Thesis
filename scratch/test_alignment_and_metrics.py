import json
import numpy as np
import pandas as pd
import h5py
from pathlib import Path
from scipy.stats import pearsonr, spearmanr

def main():
    # 1. Load predictions
    run_dir = Path("model/model_results/final_run_lopo_2")
    run_data_path = run_dir / "run_data.json"
    
    with open(run_data_path, "r") as f:
        run_data = json.load(f)
        
    preds = run_data['all']['predictions']
    y_true = np.asarray(preds['y_true'])
    y_pred = np.asarray(preds['y_pred'])
    h5_files = run_data['all']['dataset']['h5_files']
    
    print(f"Predictions length: {len(y_true)}")
    
    # 2. Load blacklist
    blacklist_path = Path("data_analysis/results/blacklist.json")
    blacklist = set()
    if blacklist_path.exists():
        with open(blacklist_path, "r") as f:
            entries = json.load(f)
        blacklist = {(e['file'], e['segment']) for e in entries}
        print(f"Loaded blacklist with {len(blacklist)} entries.")
        
    # 3. Load segments in the exact order of h5_files, skipping P13
    segments_dir = Path("/Volumes/Laurens SSD/BasData/segments")
    
    TRUE_WEIGHTS = {0.75: 0.90, 1.0: 0.98, 2.0: 1.97, 2.25: 2.24, 3.0: 2.95, 4.25: 4.15, 6.0: 5.93}
    excluded_weights = {0.75, 2.25}
    
    aligned_records = []
    
    # Skip participants not in include
    include_participants = {'P01','P02','P03','P04','P05','P06','P07','P08','P09','P10','P11','P12','P14','P15','P16','P17','P18'}
    
    for fname in h5_files:
        subject = fname.split('_')[1]
        if subject not in include_participants:
            print(f"Skipping file for excluded participant: {fname}")
            continue
            
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
                
                aligned_records.append({
                    "participant": subject,
                    "filename": fname,
                    "segment_id": seg_key,
                    "state": state,
                    "weight": weight,
                    "duration": duration,
                    "distance_cm": distance_cm
                })
                
    print(f"Loaded segments length: {len(aligned_records)}")
    
    # Assert and verify alignment
    mismatch_count = 0
    first_mismatches = []
    for i in range(len(y_true)):
        pred_w = y_true[i]
        load_w = aligned_records[i]['weight']
        if not np.isclose(pred_w, load_w):
            mismatch_count += 1
            if len(first_mismatches) < 5:
                first_mismatches.append((i, aligned_records[i]['participant'], pred_w, load_w))
            
    print(f"Weight mismatches: {mismatch_count} / {len(y_true)}")
    if first_mismatches:
        print("First few mismatches (idx, part, pred_w, load_w):", first_mismatches)
    
    # 4. Integrate prediction values
    for i in range(len(y_true)):
        aligned_records[i]["y_true"] = y_true[i]
        aligned_records[i]["y_pred"] = y_pred[i]
        aligned_records[i]["error"] = abs(y_true[i] - y_pred[i])
        
    df = pd.DataFrame(aligned_records)
    
    # Let's check statistics for carrying phase
    df_carry = df[df["state"] == "carrying"].copy()
    r_carry, p_carry = pearsonr(df_carry["distance_cm"], df_carry["duration"])
    rho_carry, p_rho_carry = spearmanr(df_carry["distance_cm"], df_carry["duration"])
    print(f"Carrying phase duration vs distance:")
    print(f"  Pearson r = {r_carry:.4f} (p={p_carry:.3e})")
    print(f"  Spearman rho = {rho_carry:.4f} (p={p_rho_carry:.3e})")
    
    # Let's check statistics for free movement phase
    df_free = df[df["state"] == "free_movement"].copy()
    r_free, p_free = pearsonr(df_free["distance_cm"], df_free["duration"])
    rho_free, p_rho_free = spearmanr(df_free["distance_cm"], df_free["duration"])
    print(f"Free movement phase duration vs distance:")
    print(f"  Pearson r = {r_free:.4f} (p={p_free:.3e})")
    print(f"  Spearman rho = {rho_free:.4f} (p={p_rho_free:.3e})")
    
    # Let's check statistics for prediction error vs distance
    r_err, p_err = pearsonr(df_carry["distance_cm"], df_carry["error"])
    print(f"Carrying phase prediction error vs distance:")
    print(f"  Pearson r = {r_err:.4f} (p={p_err:.3e})")

if __name__ == "__main__":
    main()
