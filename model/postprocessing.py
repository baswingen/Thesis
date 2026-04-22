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

from src.emg_processing import BandpassFilter, NotchFilter, EMGEnvelopeExtractor
from model.config_model import EMG_CHANNEL_CONFIG

def process_emg_data(file_path: str | Path, fs_fallback: float = 2000.0, overwrite: bool = True, channel_stats: dict = None):
    """
    Reads the synced matrix from an HDF5 file, processes the EMG channels,
    and saves the processed data back to the file.
    If channel_stats is provided (containing median and iqr), it uses those for robust scaling.
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
            col_names = ["t_pc_common"] + [f"stm_{i}" for i in range(22)] + \
                        ["t_tmsi"] + [f"ch{i}" for i in range(1, 33)] + ["trig"] + \
                        ["state_id", "weight", "src_r", "src_c", "tgt_r", "tgt_c"]
                        
        # Find the indices of EMG channels (ch1 to ch32)
        emg_col_indices = []
        emg_col_names = []
        col_names_str = []
        t_pc_idx = -1
        for i, name in enumerate(col_names):
            if isinstance(name, bytes):
                name_str = name.decode('utf-8')
            else:
                name_str = str(name)
                
            col_names_str.append(name_str)
            
            if name_str == "t_pc_common":
                t_pc_idx = i
            
            if name_str.startswith("ch") and name_str[2:].isdigit():
                emg_col_indices.append(i)
                emg_col_names.append(name_str)
                
        if not emg_col_indices:
            raise ValueError("No EMG channels found in the dataset columns.")
            
        # Load the data matrix
        data = dset_synced[:]
        
        # Calculate robust FS from t_pc_common
        fs = fs_fallback
        if t_pc_idx != -1:
            t_common = data[:, t_pc_idx]
            dt = np.median(np.diff(t_common))
            if dt > 0:
                fs = 1.0 / dt
                print(f"Derived effective sampling rate from PC clock: {fs:.2f} Hz")

        raw_emg = data[:, emg_col_indices]
        
        # Filter out channels that are all zeros or all NaNs
        valid_indices_mask = []
        for i in range(raw_emg.shape[1]):
            col_data = raw_emg[:, i]
            if np.all(np.isnan(col_data)) or np.all(col_data == 0):
                valid_indices_mask.append(False)
            else:
                valid_indices_mask.append(True)
        
        emg_col_indices = [emg_col_indices[i] for i, valid in enumerate(valid_indices_mask) if valid]
        emg_col_names = [emg_col_names[i] for i, valid in enumerate(valid_indices_mask) if valid]
        raw_emg = data[:, emg_col_indices]
        
        if not emg_col_indices:
            print("Warning: No valid EMG channels remaining after filtering.")
            return

        # Derive channels according to EMG_CHANNEL_CONFIG
        derived_emg_data = []
        derived_emg_names = []
        
        for muscle_name, channels in EMG_CHANNEL_CONFIG.items():
            if isinstance(channels, tuple):
                ch1, ch2 = channels
                if ch1 in emg_col_names and ch2 in emg_col_names:
                    idx1 = emg_col_names.index(ch1)
                    idx2 = emg_col_names.index(ch2)
                    diff_signal = raw_emg[:, idx1] - raw_emg[:, idx2]
                    derived_emg_data.append(diff_signal)
                    derived_emg_names.append(muscle_name)
                else:
                    print(f"Warning: Channels {ch1} or {ch2} for {muscle_name} not found.")
            else:
                ch = channels
                if ch in emg_col_names:
                    idx = emg_col_names.index(ch)
                    derived_emg_data.append(raw_emg[:, idx])
                    derived_emg_names.append(muscle_name)
                else:
                    print(f"Warning: Channel {ch} for {muscle_name} not found.")
                    
        if not derived_emg_data:
            print("Warning: No defined EMG channels found after applying configuration.")
            return
            
        raw_emg = np.column_stack(derived_emg_data)
        emg_col_names = derived_emg_names

        # Estimate original fs from raw data
        fs_orig = fs_fallback
        if '_raw/emg' in f:
            raw_emg_dset = f['_raw/emg']
            raw_cols = [n.decode('utf-8') if isinstance(n, bytes) else str(n) for n in raw_emg_dset.attrs.get('column_names', [])]
            if "t_pc" in raw_cols:
                t_raw_idx = raw_cols.index("t_pc")
                t_raw = raw_emg_dset[:, t_raw_idx]
                dt_raw = np.median(np.diff(t_raw))
                if dt_raw > 0:
                    fs_orig = 1.0 / dt_raw
                    print(f"Derived original EMG sampling rate from raw PC clock: {fs_orig:.2f} Hz")
        else:
            print(f"Warning: '_raw/emg' not found. Falling back to fs_orig = {fs_orig:.2f} Hz")

        # Initialize filters
        bp_filter = BandpassFilter(lowcut=20.0, highcut=500.0, fs=fs, order=4)
        notch_filter = NotchFilter(freq=50.0, fs=fs)
        
        print("Applying filtering and normalization...")
        processed_emg = np.zeros_like(raw_emg)
        
        for i in range(raw_emg.shape[1]):
            channel_data = raw_emg[:, i]
            channel_data = np.nan_to_num(channel_data, nan=0.0)
            
            # Filtering pipeline
            channel_data = bp_filter.filter(channel_data)
            bp_filter.reset()
            channel_data = notch_filter.filter(channel_data)
            notch_filter.reset()
            
            # Normalization (Robust Scaling)
            if channel_stats and emg_col_names[i] in channel_stats:
                stat = channel_stats[emg_col_names[i]]
                median = stat['median']
                iqr = stat['iqr']
            else:
                # Per-trial fallback
                median = np.nanmedian(channel_data)
                q1 = np.nanpercentile(channel_data, 25)
                q3 = np.nanpercentile(channel_data, 75)
                iqr = q3 - q1
                
            if iqr > 1e-9:
                channel_data = (channel_data - median) / iqr
            else:
                # Fallback to centering only if signal is flat
                channel_data = channel_data - median
                
            processed_emg[:, i] = channel_data
            
        # Bring over timestamp columns
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
                del f[out_dataset_name]
            else:
                raise ValueError(f"Dataset {out_dataset_name} already exists.")
                
        out_dset = f.create_dataset(
            out_dataset_name, data=processed_emg, dtype='float64',
            compression="gzip", compression_opts=4, chunks=True
        )
        
        out_dset.attrs['column_names'] = [n.encode('utf-8') for n in final_col_names]
        out_dset.attrs['description'] = "Postprocessed EMG: BP(20-500Hz) -> Notch(50Hz) -> Normalized (Unrectified Raw Oscillating)"
        out_dset.attrs['fs'] = fs
        out_dset.attrs['fs_orig'] = fs_orig
        out_dset.attrs['normalization'] = "robust_scaling_session_level" if channel_stats else "robust_scaling_per_trial"
        
        print(f"Processed data saved to {out_dataset_name} successfully.")
        

def process_imu_data(file_path: str | Path, fs_fallback: float = 500.0, overwrite: bool = True):
    """
    Reads the synced matrix from an HDF5 file, processes the IMU channels.
    Calculates effective sampling rate from PC clock.
    """
    file_path = Path(file_path)
    if not file_path.exists():
        raise FileNotFoundError(f"HDF5 file not found: {file_path}")
        
    print(f"Processing IMU data for {file_path.name}...")
        
    with h5py.File(file_path, 'a') as f:
        if 'synced/data' not in f:
            print("Dataset 'synced/data' not found.")
            return
            
        dset_synced = f['synced/data']
        col_names = dset_synced.attrs.get('column_names', [])
        
        col_names_str = []
        for name in col_names:
            if isinstance(name, bytes):
                col_names_str.append(name.decode('utf-8'))
            else:
                col_names_str.append(str(name))
                
        data = dset_synced[:]
        
        # Calculate robust FS from t_pc_common
        # The true effective synchronized matrix maps all data to the `t_pc_common` grid.
        # So effective fs_imu for the matrix is just the sampling rate of the matrix.
        fs_imu = fs_fallback
        if "t_pc_common" in col_names_str:
            idx = col_names_str.index("t_pc_common")
            t_common = data[:, idx]
            dt = np.median(np.diff(t_common))
            if dt > 0:
                fs_imu = 1.0 / dt
                print(f"Derived effective IMU sampling rate (synced matrix): {fs_imu:.2f} Hz")

        # Estimate original fs from raw data
        fs_imu_orig = fs_fallback
        if '_raw/stm32' in f:
            raw_imu_dset = f['_raw/stm32']
            raw_cols = [n.decode('utf-8') if isinstance(n, bytes) else str(n) for n in raw_imu_dset.attrs.get('column_names', [])]
            if "t_pc" in raw_cols:
                t_raw_idx = raw_cols.index("t_pc")
                t_raw = raw_imu_dset[:, t_raw_idx]
                dt_raw = np.median(np.diff(t_raw))
                if dt_raw > 0:
                    fs_imu_orig = 1.0 / dt_raw
                    print(f"Derived original IMU sampling rate from raw PC clock: {fs_imu_orig:.2f} Hz")
        else:
            print(f"Warning: '_raw/stm32' not found. Falling back to fs_orig = {fs_imu_orig:.2f} Hz")

        # Look for IMU channels
        imu_base_names = ['ax', 'ay', 'az', 'roll', 'pitch', 'yaw']
        imu_cols_dict = {1: {}, 2: {}} # sensor 1 and 2
        for s in [1, 2]:
            for base in imu_base_names:
                col_name = f"{base}{s}"
                if col_name in col_names_str:
                    imu_cols_dict[s][base] = col_names_str.index(col_name)
                    
        if not imu_cols_dict[1] and not imu_cols_dict[2]:
            print("Warning: No IMU channels found in the dataset.")
            return
            
        final_data = []
        final_names = []
        
        # Bring over timestamp columns
        if "t_pc_common" in col_names_str:
            idx = col_names_str.index("t_pc_common")
            final_data.append(data[:, idx:idx+1])
            final_names.append("t_pc_common")
            
        if "t_tmsi" in col_names_str:
            idx = col_names_str.index("t_tmsi")
            final_data.append(data[:, idx:idx+1])
            final_names.append("t_tmsi")

        processed_sensors = {}
        
        from scipy.signal import butter, filtfilt
        # Design a zero-phase lowpass filter to smooth the zero-order hold stair-steps
        # A 15 Hz cutoff clears the steps while preserving human movement frequencies
        nyq = 0.5 * fs_imu
        cutoff = 15.0
        if cutoff >= nyq:
            cutoff = nyq * 0.99
        b, a = butter(4, cutoff / nyq, btype='low', analog=False)

        for s in [1, 2]:
            if not imu_cols_dict[s]: continue
            
            s_data = {}
            # Accelerations: keep raw signal directly (no gravity/linear split)
            for axis in ['ax', 'ay', 'az']:
                if axis in imu_cols_dict[s]:
                    raw_acc = data[:, imu_cols_dict[s][axis]]
                    raw_acc = np.nan_to_num(raw_acc, nan=0.0)
                    if len(raw_acc) > 18:
                        raw_acc = filtfilt(b, a, raw_acc)
                    s_data[f'{axis}{s}'] = raw_acc
                    
            # Euler angles: convert degrees → radians (one signal per orientation)
            # Filter after unwrapping to prevent artifacts on angle crossovers
            for angle in ['roll', 'pitch', 'yaw']:
                if angle in imu_cols_dict[s]:
                    raw_ang = data[:, imu_cols_dict[s][angle]]
                    raw_ang = np.nan_to_num(raw_ang, nan=0.0)
                    rad_ang = np.deg2rad(raw_ang)
                    if len(rad_ang) > 18:
                        unwrapped = np.unwrap(rad_ang)
                        smoothed = filtfilt(b, a, unwrapped)
                        rad_ang = (smoothed + np.pi) % (2 * np.pi) - np.pi
                    s_data[f'{angle}_rad{s}'] = rad_ang
                    
            processed_sensors[s] = s_data
            
        # Compile all newly generated channels
        for s in [1, 2]:
            if s in processed_sensors:
                for k, v in processed_sensors[s].items():
                    final_names.append(k)
                    final_data.append(v.reshape(-1, 1))
                    
        # Add Difference Channels (Sensor 2 - Sensor 1) describing Elbow joint relative kinematics
        if 1 in processed_sensors and 2 in processed_sensors:
            for k1 in processed_sensors[1].keys():
                base = k1[:-1] # strip the '1'
                k2 = f"{base}2"
                if k2 in processed_sensors[2]:
                    diff = processed_sensors[2][k2] - processed_sensors[1][k1]
                    diff_name = f"{base}_diff"
                    final_names.append(diff_name)
                    final_data.append(diff.reshape(-1, 1))
                    
        if not final_data:
            return
            
        final_matrix = np.hstack(final_data)
        
        out_dataset_name = 'synced/imu_processed'
        if out_dataset_name in f:
            if overwrite:
                del f[out_dataset_name]
            else:
                raise ValueError(f"Dataset {out_dataset_name} already exists.")
                
        out_dset = f.create_dataset(
            out_dataset_name, data=final_matrix, dtype='float64',
            compression="gzip", compression_opts=4, chunks=True
        )
        
        out_dset.attrs['column_names'] = [n.encode('utf-8') for n in final_names]
        out_dset.attrs['description'] = "Postprocessed IMU: Raw Accel LPF(15Hz), Euler->Radians LPF(15Hz), RelDiffs (Imu2-Imu1)"
        out_dset.attrs['fs'] = fs_imu
        out_dset.attrs['fs_orig'] = fs_imu_orig
        
        print(f"Processed IMU data saved to {out_dataset_name} successfully.")
        

def get_session_stats(session_dir: Path, fs: float = 2000.0) -> dict:
    """
    Scans a session directory and returns the session-wide Median and IQR
    for each muscle channel. Used for cross-trial Robust Scaling.
    """
    h5_files = list(session_dir.glob("*.h5"))
    if not h5_files: return {}
    
    # Store processed envelope data temporarily to compute statistics
    session_data = {muscle: [] for muscle in EMG_CHANNEL_CONFIG.keys()}
    
    print(f"Calculating session-wide statistics for {session_dir.name}...")
    for path in h5_files:
        with h5py.File(path, 'r') as f:
            if 'synced/data' not in f: continue
            dset = f['synced/data']
            cols = [n.decode() if isinstance(n, bytes) else str(n) for n in dset.attrs.get('column_names', [])]
            emg_indices = [i for i, name in enumerate(cols) if name.startswith("ch") and name[2:].isdigit()]
            if not emg_indices: continue
            
            data = dset[:]
            
            # Effective FS for filtering
            fs_eff = fs
            if "t_pc_common" in cols:
                dt = np.median(np.diff(data[:, cols.index("t_pc_common")]))
                if dt > 0: fs_eff = 1.0 / dt
                
            bp = BandpassFilter(20.0, 500.0, fs_eff)
            nt = NotchFilter(50.0, fs_eff)
            
            raw_emg = data[:, emg_indices]
            emg_col_names = [cols[i] for i in emg_indices]
            
            for muscle_name, channels in EMG_CHANNEL_CONFIG.items():
                signal = None
                if isinstance(channels, tuple):
                    ch1, ch2 = channels
                    if ch1 in emg_col_names and ch2 in emg_col_names:
                        idx1, idx2 = emg_col_names.index(ch1), emg_col_names.index(ch2)
                        signal = raw_emg[:, idx1] - raw_emg[:, idx2]
                else:
                    if channels in emg_col_names:
                        signal = raw_emg[:, emg_col_names.index(channels)]
                
                if signal is not None:
                    channel_data = np.nan_to_num(signal, nan=0.0)
                    if not np.all(channel_data == 0):
                        processed = nt.filter(bp.filter(channel_data))
                        bp.reset(); nt.reset()
                        session_data[muscle_name].append(processed)
    
    session_stats = {}
    for name, data_list in session_data.items():
        if not data_list: continue
        all_data = np.concatenate(data_list)
        median = np.nanmedian(all_data)
        q1 = np.nanpercentile(all_data, 25)
        q3 = np.nanpercentile(all_data, 75)
        iqr = q3 - q1
        session_stats[name] = {'median': float(median), 'iqr': float(iqr)}
        
    return session_stats

def process_session_data(session_dir: str | Path, fs: float = 2000.0, overwrite: bool = True):
    """
    Processes all trials in a session using session-level Robust Scaling (Median/IQR).
    """
    session_dir = Path(session_dir)
    stats = get_session_stats(session_dir, fs=fs)
    print(f"Session {session_dir.name} stats (median/iqr):")
    for k, v in stats.items():
        print(f"  {k}: {v['median']:.2e} / {v['iqr']:.2e}")
    
    h5_files = list(session_dir.glob("*.h5"))
    for path in h5_files:
        try:
            process_emg_data(path, fs_fallback=fs, overwrite=overwrite, channel_stats=stats)
            process_imu_data(path, fs_fallback=500.0, overwrite=overwrite)
        except Exception as e:
            print(f"Failed session process for {path.name}: {e}")

def process_all_in_database(database_dir: str | Path, fs: float = 2000.0, overwrite: bool = True, participants: list[str] = None):
    """
    Recursively finds session directories and processes them with session-level normalization.
    """
    base_dir = Path(database_dir)
    # Find all session_NN directories
    session_dirs = sorted(p for p in base_dir.rglob("session_*") if p.is_dir())
    
    if participants:
        session_dirs = [p for p in session_dirs if p.parent.name in participants]

    
    if not session_dirs:
        print(f"No session directories found in {database_dir}. Falling back to per-trial.")
        h5_files = list(base_dir.rglob("*.h5"))
        for file_path in h5_files:
            process_emg_data(file_path, fs_fallback=fs, overwrite=overwrite)
            process_imu_data(file_path, overwrite=overwrite)
        return
        
    print(f"Found {len(session_dirs)} sessions to process.")
    for sdir in session_dirs:
        print(f"\n>>>> PROCESSING SESSION: {sdir.parent.name}/{sdir.name} <<<<")
        process_session_data(sdir, fs=fs, overwrite=overwrite)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process EMG/IMU channels with Robust Scaling (Median/IQR).")
    parser.add_argument("path", nargs="?", default="database", help="Path to trial file, session dir, or database.")
    parser.add_argument("--fs", type=float, default=2000.0, help="Sampling frequency fallback (default: 2000.0)")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite existing datasets")
    parser.add_argument("--participants", type=str, nargs="+", help="List of participants to process")
    
    args = parser.parse_args()
    target = Path(args.path)
    
    if target.is_file():
        process_emg_data(target, fs_fallback=args.fs, overwrite=args.overwrite)
        process_imu_data(target, fs_fallback=500.0, overwrite=args.overwrite)
    elif target.name.startswith("session_") and target.is_dir():
        process_session_data(target, fs=args.fs, overwrite=args.overwrite)
    elif target.is_dir():
        process_all_in_database(target, fs=args.fs, overwrite=args.overwrite, participants=args.participants)
    else:
        print(f"Error: Path {args.path} not found.")
