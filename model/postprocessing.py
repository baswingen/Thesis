"""
EMG Data Postprocessing for Machine Learning
=============================================
This script processes the raw synced EMG data from HDF5 trial files.

Processing steps (applied per channel):
1. Bandpass filter (10-500 Hz)
2. Notch filter (50 Hz)
3. Full-wave rectification
4. Normalization (divide by maximum peak of the channel)

The processed data is saved back into the HDF5 file under a new dataset: `synced/emg_processed`.
"""

import h5py
import numpy as np
import argparse
import sys
from pathlib import Path

# Add project root to sys.path to allow running as a script
project_root = Path(__file__).parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from src.emg_processing import BandpassFilter, NotchFilter

def process_emg_data(file_path: str | Path, fs: float = 4000.0, overwrite: bool = True):
    """
    Reads the synced matrix from an HDF5 file, processes the EMG channels,
    and saves the processed data back to the file.
    """
    file_path = Path(file_path)
    if not file_path.exists():
        raise FileNotFoundError(f"HDF5 file not found: {file_path}")
        
    print(f"Loading {file_path}...")
    
    with h5py.File(file_path, 'a') as f:
        # Check if synced/data exists
        if 'synced/data' not in f:
            raise KeyError("Dataset 'synced/data' not found in the HDF5 file.")
            
        dset_synced = f['synced/data']
        col_names = dset_synced.attrs.get('column_names', [])
        
        if len(col_names) == 0:
            print("Warning: 'column_names' missing in 'synced/data' attributes. Attempting to deduce channels.")
            # Default columns from hdf5_logger.py: 
            # 1 (t_pc_common) + 22 (STM32) + 33 (EMG) + 6 (LABEL)
            # EMG channels are typically indices 23 to 54.
            col_names = ["t_pc_common"] + [f"stm_{i}" for i in range(22)] + \
                        ["t_tmsi"] + [f"ch{i}" for i in range(1, 33)] + ["trig"] + \
                        ["state_id", "weight", "src_r", "src_c", "tgt_r", "tgt_c"]
                        
        # Find the indices of EMG channels (ch1 to ch32)
        emg_col_indices = []
        emg_col_names = []
        col_names_str = []
        for i, name in enumerate(col_names):
            # Handle possible bytes
            if isinstance(name, bytes):
                name_str = name.decode('utf-8')
            else:
                name_str = str(name)
                
            col_names_str.append(name_str)
            
            if name_str.startswith("ch") and name_str[2:].isdigit():
                emg_col_indices.append(i)
                emg_col_names.append(name_str)
                
        if not emg_col_indices:
            raise ValueError("No EMG channels found in the dataset columns.")
            
        print(f"Found {len(emg_col_indices)} EMG channels to process: {emg_col_names}")
        
        # Load the data matrix
        # For large datasets, we process in memory since EMG filtered data is typically small enough to fit in RAM
        data = dset_synced[:]
        
        raw_emg = data[:, emg_col_indices]
        
        # Initialize filters
        bp_filter = BandpassFilter(lowcut=10.0, highcut=500.0, fs=fs, order=4)
        notch_filter = NotchFilter(freq=50.0, fs=fs)
        
        print("Applying filtering and normalization...")
        processed_emg = np.zeros_like(raw_emg)
        
        # Process each channel independent of others
        for i in range(raw_emg.shape[1]):
            # Simple progress print
            if i % 8 == 0 or i == raw_emg.shape[1] - 1:
                print(f"Processing channel {i+1}/{raw_emg.shape[1]}...")
                
            channel_data = raw_emg[:, i]
            
            # Replace NaNs to prevent filter instability
            channel_data = np.nan_to_num(channel_data, nan=0.0)
            
            # 1. Bandpass filter
            channel_data = bp_filter.filter(channel_data)
            bp_filter.reset()
            
            # 2. Notch filter
            channel_data = notch_filter.filter(channel_data)
            notch_filter.reset()
            
            # 3. Full-wave rectification
            channel_data = np.abs(channel_data)
            
            # 4. Normalization (Peak normalization)
            peak = np.nanmax(channel_data)
            if peak > 0 and not np.isnan(peak):
                channel_data = channel_data / peak
                
            processed_emg[:, i] = channel_data
            
        # 5. Bring over timestamp columns (t_pc_common and t_tmsi if present)
        time_cols = []
        time_col_names = []
        
        if "t_pc_common" in col_names_str:
            idx = col_names_str.index("t_pc_common")
            time_cols.append(data[:, idx:idx+1])
            time_col_names.append("t_pc_common")
            
        if "t_tmsi" in col_names_str:
            idx = col_names_str.index("t_tmsi")
            time_cols.append(data[:, idx:idx+1])
            time_col_names.append("t_tmsi")
            
        if time_cols:
            processed_emg = np.hstack((*time_cols, processed_emg))
            final_col_names = time_col_names + emg_col_names
        else:
            final_col_names = emg_col_names
            
        # Save back to HDF5
        out_dataset_name = 'synced/emg_processed'
        if out_dataset_name in f:
            if overwrite:
                print(f"Overwriting existing dataset: {out_dataset_name}")
                del f[out_dataset_name]
            else:
                raise ValueError(f"Dataset {out_dataset_name} already exists. Use overwrite=True to replace it.")
                
        out_dset = f.create_dataset(
            out_dataset_name,
            data=processed_emg,
            dtype='float64',
            compression="gzip",
            compression_opts=4,
            chunks=True
        )
        
        # We need to save as numpy string array for h5py, or unicode
        out_dset.attrs['column_names'] = [n.encode('utf-8') for n in final_col_names]
        out_dset.attrs['description'] = "Postprocessed EMG: BP(10-500Hz) -> Notch(50Hz) -> Rectified -> Peak Normalized"
        out_dset.attrs['fs'] = fs
        
        print(f"Processed data saved to {out_dataset_name} successfully.")
        

def process_all_in_database(database_dir: str | Path, fs: float = 4000.0, overwrite: bool = True):
    """
    Recursively finds all HDF5 trial files in the database directory and processes them.
    """
    base_dir = Path(database_dir)
    h5_files = list(base_dir.rglob("*.h5"))
    
    if not h5_files:
        print(f"No HDF5 files found in {database_dir}")
        return
        
    print(f"Found {len(h5_files)} HDF5 files to process.")
    for file_path in h5_files:
        try:
            print(f"\n--- Processing {file_path.name} ---")
            process_emg_data(file_path, fs=fs, overwrite=overwrite)
        except Exception as e:
            print(f"Failed to process {file_path.name}: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process EMG channels in an HDF5 trial file.")
    parser.add_argument("path", nargs="?", default="database", help="Path to the HDF5 trial file or directory (searched recursively). Supports participant_PXX/session_NN/trial_*.h5 layout. Defaults to 'database'.")
    parser.add_argument("--fs", type=float, default=4000.0, help="Sampling frequency of the EMG data (default: 4000.0)")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite the processed dataset if it already exists")
    
    args = parser.parse_args()
    
    target_path = Path(args.path)
    if target_path.is_file():
        process_emg_data(target_path, fs=args.fs, overwrite=args.overwrite)
    elif target_path.is_dir():
        process_all_in_database(target_path, fs=args.fs, overwrite=args.overwrite)
    else:
        print(f"Error: Path {args.path} does not exist.")
