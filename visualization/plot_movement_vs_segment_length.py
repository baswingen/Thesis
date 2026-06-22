#!/usr/bin/env python3
"""
plot_movement_vs_segment_length.py
==================================
Retrieves start and end positions of movements on the 3x4 button matrix grid
(15 cm spacing) and plots them against segment duration (time) and
model weight estimation error.
"""

import os
import sys
import glob
import h5py
import json
import numpy as np
import pandas as pd
from scipy.stats import pearsonr, spearmanr
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from pathlib import Path

# Add project root to path to import visualization style
sys.path.append(str(Path(__file__).resolve().parent.parent))
from visualization.run_viz import ThesisStyle

def main():
    # 1. Load run configuration and predictions
    run_dir = Path("model/model_results/final_run_lopo_2")
    run_data_path = run_dir / "run_data.json"
    
    if not run_data_path.exists():
        print(f"[ERROR] Run data file not found: {run_data_path}")
        sys.exit(1)
        
    print(f"Loading predictions from {run_data_path}...")
    with open(run_data_path, "r") as f:
        run_data = json.load(f)
        
    preds = run_data['all']['predictions']
    preds_y_true = np.asarray(preds['y_true'])
    preds_y_pred = np.asarray(preds['y_pred'])
    preds_part = np.asarray(preds['participant'])
    
    # 2. Load blacklist
    blacklist_path = Path("data_analysis/results/blacklist.json")
    blacklist = set()
    if blacklist_path.exists():
        with open(blacklist_path, "r") as f:
            entries = json.load(f)
        blacklist = {(e['file'], e['segment']) for e in entries}
        print(f"Loaded blacklist with {len(blacklist)} segments.")
    else:
        print("[Warning] Blacklist file not found.")

    # 3. Load segment metadata from HDF5 database
    segments_dir = Path("/Volumes/Laurens SSD/BasData/segments")
    if not segments_dir.exists():
        print(f"[ERROR] Segments directory not found: {segments_dir}")
        sys.exit(1)
        
    h5_files = glob.glob(os.path.join(segments_dir, "*.h5"))
    h5_files = [f for f in h5_files if not os.path.basename(f).startswith("._")]
    
    # We only include the active participants
    include_participants = ['P01','P02','P03','P04','P05','P06','P07','P08','P09','P10','P11','P12','P14','P15','P16','P17','P18']
    filtered_files = sorted([f for f in h5_files if os.path.basename(f).split('_')[1] in include_participants])
    
    print(f"Loading segment metadata from HDF5 files for {len(include_participants)} participants...")
    
    # Calibration constants
    TRUE_WEIGHTS = {0.75: 0.90, 1.0: 0.98, 2.0: 1.97, 2.25: 2.24, 3.0: 2.95, 4.25: 4.15, 6.0: 5.93}
    excluded_weights = {0.75, 2.25}
    
    raw_records = []
    for fp in filtered_files:
        fname = os.path.basename(fp)
        subject = fname.split('_')[1]
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
                
                raw_records.append({
                    "participant": subject,
                    "filename": fname,
                    "segment_id": seg_key,
                    "state": state,
                    "weight": weight,
                    "duration": duration,
                    "distance_cm": distance_cm
                })

    print(f"Loaded {len(raw_records)} segments from HDF5.")
    
    # 4. Sequentially align prediction lists with loaded HDF5 segments for each participant
    aligned_records = []
    
    # Group records by participant
    rec_by_part = {}
    for r in raw_records:
        rec_by_part.setdefault(r["participant"], []).append(r)
        
    # Group predictions by participant
    preds_by_part = {}
    for i in range(len(preds_y_true)):
        p = preds_part[i]
        preds_by_part.setdefault(p, []).append({
            "y_true": preds_y_true[i],
            "y_pred": preds_y_pred[i],
            "error": abs(preds_y_true[i] - preds_y_pred[i])
        })
        
    print("Aligning segments with predictions sequentially by participant and weight...")
    
    total_aligned = 0
    for p in sorted(include_participants):
        p_recs = rec_by_part.get(p, [])
        p_preds = preds_by_part.get(p, [])
        
        # Align using a pointer-matching algorithm to skip records that are missing in predictions
        # (e.g., due to duration thresholding or channel gaps)
        rec_idx = 0
        pred_idx = 0
        
        while rec_idx < len(p_recs) and pred_idx < len(p_preds):
            rec = p_recs[rec_idx]
            pred = p_preds[pred_idx]
            
            # Check if weights match
            if np.isclose(rec["weight"], pred["y_true"]):
                # Found a match!
                rec["y_true"] = pred["y_true"]
                rec["y_pred"] = pred["y_pred"]
                rec["error"] = pred["error"]
                rec["is_predicted"] = True
                aligned_records.append(rec)
                
                rec_idx += 1
                pred_idx += 1
                total_aligned += 1
            else:
                # Mismatch - advance the record pointer (assuming this record was skipped during training)
                rec_idx += 1
                
        # Fill in the rest of records as not predicted
        while rec_idx < len(p_recs):
            rec = p_recs[rec_idx]
            rec["y_true"] = None
            rec["y_pred"] = None
            rec["error"] = None
            rec["is_predicted"] = False
            aligned_records.append(rec)
            rec_idx += 1
            
    print(f"Alignment complete: successfully aligned {total_aligned} / {len(preds_y_true)} predictions.")

    # 5. Convert to pandas DataFrame
    df_all = pd.DataFrame(aligned_records)
    df_pred = df_all[df_all["is_predicted"] == True].copy()
    
    # 6. Apply Styles
    ThesisStyle.apply("double") # applies standard fonts, axes, etc.
    
    # Override default figure size for 3-panel horizontal layout
    # Width=7.0" is the exact double-column text width for the LaTeX template.
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(7.0, 2.7))
    
    # --- Statistics Helper ---
    def add_stats_text(ax, x, y, clean_df, state_label):
        # Remove outlier durations (e.g. > 5s) for cleaner correlation coefficient
        clean_sub = clean_df[clean_df["duration"] < 5.0]
        r_val, p_val = pearsonr(clean_sub[x], clean_sub[y])
        rho_val, p_val_rho = spearmanr(clean_sub[x], clean_sub[y])
        
        # Display stats
        textstr = (
            f"Pearson $r$: {r_val:.2f}\n"
            f"Spearman $\\rho$: {rho_val:.2f}\n"
            f"$p$-val: < 0.001" if p_val < 0.001 else f"$p$-val: {p_val:.3f}"
        )
        props = dict(boxstyle='round', facecolor='white', edgecolor='#E0E0E0', alpha=0.9, pad=0.3)
        ax.text(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=plt.rcParams['font.size'] - 1.5,
                verticalalignment='top', bbox=props)
        return r_val
        
    # Set styling parameters for scatter
    scatter_kwargs = {'alpha': 0.12, 's': 2, 'zorder': 2}
    violin_kwargs = {'showmeans': True, 'showextrema': False, 'showmedians': False}
    
    # Unique distances in cm
    unique_dists = sorted(df_all["distance_cm"].unique())
    x_pos = np.arange(len(unique_dists))
    dist_map = {d: idx for idx, d in enumerate(unique_dists)}
    
    # Ensure distance lists are mapped to positions for plotting
    df_all["dist_pos"] = df_all["distance_cm"].map(dist_map)
    df_pred["dist_pos"] = df_pred["distance_cm"].map(dist_map)
    
    # --- Panel A: Carrying Phase ---
    df_carry = df_all[df_all["state"] == "carrying"].copy()
    df_carry_clean = df_carry[df_carry["duration"] < 5.0]  # strip outliers for display
    
    # Group durations by distance for violin plot
    carry_durs_by_dist = [df_carry_clean[df_carry_clean["distance_cm"] == d]["duration"].values for d in unique_dists]
    
    # Plot violin distributions
    parts1 = ax1.violinplot(carry_durs_by_dist, positions=x_pos, **violin_kwargs)
    for pc in parts1['bodies']:
        pc.set_facecolor(ThesisStyle.COLOR_EMG)
        pc.set_edgecolor(ThesisStyle.COLOR_EMG)
        pc.set_alpha(0.3)
    parts1['cmeans'].set_color(ThesisStyle.COLOR_EMG)
    parts1['cmeans'].set_linewidth(1.5)
    
    # Plot individual jittered scatter points
    jitter = np.random.normal(0, 0.08, len(df_carry_clean))
    ax1.scatter(df_carry_clean["dist_pos"] + jitter, df_carry_clean["duration"], 
                color=ThesisStyle.COLOR_EMG, **scatter_kwargs)
    
    # Fit and plot regression line on the mean durations
    means_carry = [np.mean(durs) for durs in carry_durs_by_dist]
    ax1.plot(x_pos, means_carry, color=ThesisStyle.COLOR_EMG, linestyle='-', linewidth=1.2, zorder=3, 
             label="Mean Duration")
    
    ax1.set_ylabel("Segment Duration (s)", labelpad=4)
    ax1.set_xlabel("Movement Distance (cm)", labelpad=4)
    ax1.set_xticks(x_pos)
    ax1.set_xticklabels([f"{d:.0f}" for d in unique_dists], rotation=45)
    ax1.set_ylim(0, 3.5)
    ThesisStyle.set_title(ax1, "Carrying Phase (Active)")
    add_stats_text(ax1, "distance_cm", "duration", df_carry, "carrying")
    
    # --- Panel B: Free Movement Phase ---
    df_free = df_all[df_all["state"] == "free_movement"].copy()
    df_free_clean = df_free[df_free["duration"] < 5.0]
    
    free_durs_by_dist = [df_free_clean[df_free_clean["distance_cm"] == d]["duration"].values for d in unique_dists]
    
    # Plot violin distributions
    parts2 = ax2.violinplot(free_durs_by_dist, positions=x_pos, **violin_kwargs)
    for pc in parts2['bodies']:
        pc.set_facecolor(ThesisStyle.NEUTRAL_GRAY)
        pc.set_edgecolor(ThesisStyle.NEUTRAL_GRAY)
        pc.set_alpha(0.3)
    parts2['cmeans'].set_color(ThesisStyle.NEUTRAL_GRAY)
    parts2['cmeans'].set_linewidth(1.5)
    
    # Plot individual jittered scatter points
    jitter = np.random.normal(0, 0.08, len(df_free_clean))
    ax2.scatter(df_free_clean["dist_pos"] + jitter, df_free_clean["duration"], 
                color=ThesisStyle.NEUTRAL_GRAY, **scatter_kwargs)
    
    # Fit and plot regression line on the mean durations
    means_free = [np.mean(durs) for durs in free_durs_by_dist]
    ax2.plot(x_pos, means_free, color=ThesisStyle.NEUTRAL_GRAY, linestyle='-', linewidth=1.2, zorder=3)
    
    ax2.set_xlabel("Movement Distance (cm)", labelpad=4)
    ax2.set_xticks(x_pos)
    ax2.set_xticklabels([f"{d:.0f}" for d in unique_dists], rotation=45)
    ax2.set_ylim(0, 3.5)
    ThesisStyle.set_title(ax2, "Free Movement Phase (Empty)")
    add_stats_text(ax2, "distance_cm", "duration", df_free, "free_movement")
    
    # --- Panel C: Weight Estimation Error ---
    df_pred_carry = df_pred[df_pred["state"] == "carrying"].copy()
    
    errors_by_dist = [df_pred_carry[df_pred_carry["distance_cm"] == d]["error"].values for d in unique_dists]
    
    # Plot violin distributions
    parts3 = ax3.violinplot(errors_by_dist, positions=x_pos, **violin_kwargs)
    for pc in parts3['bodies']:
        pc.set_facecolor(ThesisStyle.COLOR_RMSE)
        pc.set_edgecolor(ThesisStyle.COLOR_RMSE)
        pc.set_alpha(0.3)
    parts3['cmeans'].set_color(ThesisStyle.COLOR_RMSE)
    parts3['cmeans'].set_linewidth(1.5)
    
    # Plot individual jittered scatter points
    jitter = np.random.normal(0, 0.08, len(df_pred_carry))
    ax3.scatter(df_pred_carry["dist_pos"] + jitter, df_pred_carry["error"], 
                color=ThesisStyle.COLOR_RMSE, **scatter_kwargs)
    
    # Plot mean prediction error
    means_error = [np.mean(errs) for errs in errors_by_dist]
    ax3.plot(x_pos, means_error, color=ThesisStyle.COLOR_RMSE, linestyle='-', linewidth=1.2, zorder=3,
             label="Mean Absolute Error")
    
    ax3.set_ylabel("Absolute Estimation Error (kg)", labelpad=4)
    ax3.set_xlabel("Movement Distance (cm)", labelpad=4)
    ax3.set_xticks(x_pos)
    ax3.set_xticklabels([f"{d:.0f}" for d in unique_dists], rotation=45)
    ax3.set_ylim(0, 3.5)
    ThesisStyle.set_title(ax3, "Model Estimation Error")
    
    # Add correlation statistics for error vs distance
    r_val, p_val = pearsonr(df_pred_carry["distance_cm"], df_pred_carry["error"])
    err_text = f"Pearson $r$: {r_val:.3f}\n$p$-val: {p_val:.2f}"
    props = dict(boxstyle='round', facecolor='white', edgecolor='#E0E0E0', alpha=0.9, pad=0.3)
    ax3.text(0.05, 0.95, err_text, transform=ax3.transAxes, fontsize=plt.rcParams['font.size'] - 1.5,
            verticalalignment='top', bbox=props)
            
    # Apply clean axes ticks/spines styling
    for ax in [ax1, ax2, ax3]:
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.grid(True, which='both', axis='y', linestyle='--', linewidth=0.5, color=ThesisStyle.GRID_GRAY, alpha=0.5)
        
    plt.suptitle("Movement Distance vs. Segment Duration and Model Prediction Error",
                 fontproperties=fm.FontProperties(family='sans-serif', weight='bold', size=plt.rcParams['axes.titlesize'] + 0.5),
                 color=ThesisStyle.RHO_BLUE, y=1.01)
    
    # Adjust layout
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # Save files
    output_dir = Path("visualization/run_plots_comp")
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "comp_movement_vs_segment_length"
    
    ThesisStyle.save_figure(fig, output_path)
    print(f"Successfully generated and saved plots:\n  - {output_path}.png\n  - {output_path}.pdf")
    plt.close(fig)

if __name__ == "__main__":
    main()
