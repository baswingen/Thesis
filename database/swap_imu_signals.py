import h5py
import sys
import numpy as np
from pathlib import Path

def swap_imu_signals(h5_path):
    print(f"[INFO] Processing: {h5_path}")
    
    # Define pairs to swap (IMU1 name, IMU2 name)
    swap_pairs = [
        ("imu1_ok", "imu2_ok"),
        ("yaw1", "yaw2"),
        ("pitch1", "pitch2"),
        ("roll1", "roll2"),
        ("ax1", "ax2"),
        ("ay1", "ay2"),
        ("az1", "az2"),
    ]
    
    datasets_to_check = ["_raw/stm32", "synced/data"]
    
    try:
        with h5py.File(h5_path, "r+") as f:
            for dset_name in datasets_to_check:
                if dset_name not in f:
                    print(f"[WARN] Dataset '{dset_name}' not found in {h5_path}. Skipping.")
                    continue
                
                dset = f[dset_name]
                col_names = list(dset.attrs.get("column_names", []))
                
                if not col_names:
                    print(f"[WARN] No column_names attribute found for '{dset_name}'. Skipping.")
                    continue
                
                # Identify indices to swap
                indices_to_swap = []
                for imu1_name, imu2_name in swap_pairs:
                    if imu1_name in col_names and imu2_name in col_names:
                        idx1 = col_names.index(imu1_name)
                        idx2 = col_names.index(imu2_name)
                        indices_to_swap.append((idx1, idx2, imu1_name, imu2_name))
                    else:
                        print(f"[DEBUG] Pair ({imu1_name}, {imu2_name}) not found in {dset_name}")
                
                if not indices_to_swap:
                    print(f"[WARN] No matching IMU columns found in {dset_name}. Skipping.")
                    continue
                
                print(f"[INFO] Swapping {len(indices_to_swap)} column pairs in '{dset_name}'...")
                
                # Read dataset
                data = dset[:]
                
                # Perform swap
                for idx1, idx2, name1, name2 in indices_to_swap:
                    # Use a temp variable to swap rows
                    # (In numpy, data[:, [idx1, idx2]] = data[:, [idx2, idx1]] also works)
                    temp = data[:, idx1].copy()
                    data[:, idx1] = data[:, idx2]
                    data[:, idx2] = temp
                    print(f"  - Swapped: {name1} <-> {name2} (indices {idx1} <-> {idx2})")
                
                # Write back
                dset[...] = data
                print(f"[SUCCESS] Updated '{dset_name}'")
                
        print(f"[DONE] Finished {h5_path}\n")
        return True
    except Exception as e:
        print(f"[ERR] Error processing {h5_path}: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: py database/swap_imu_signals.py <file1.h5> <file2.h5> ...")
        sys.exit(1)
    
    paths = sys.argv[1:]
    success_count = 0
    for p in paths:
        if swap_imu_signals(Path(p)):
            success_count += 1
            
    print(f"Summary: Successfully updated {success_count}/{len(paths)} files.")
