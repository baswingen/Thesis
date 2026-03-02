import h5py
import numpy as np
import time
import os
from pathlib import Path
from typing import Dict, Any, Optional

class HDF5TrialLogger:
    """
    Logs raw and synchronized trial data to an HDF5 file.
    Designed for dynamic resizing during continuous acquisition.
    Optimized for H5Web visualization and future ML training.
    """

    # --- Column Definitions for Metadata and ML Indexing ---
    STM32_COLUMNS = [
        "t_pc", "t_stm32", "imu1_ok", "imu2_ok", 
        "yaw1", "pitch1", "roll1", "ax1", "ay1", "az1",
        "yaw2", "pitch2", "roll2", "ax2", "ay2", "az2",
        "keys_mask", "keys_rise", "keys_fall",
        "prbs_tick", "prbs_lvl", "in_mark"
    ]
    
    EMG_COLUMNS = ["t_pc", "t_tmsi"] + [f"ch{i}" for i in range(1, 33)] + ["trig"]
    
    SYNC_METRICS_COLUMNS = ["t_effective", "delay_ms", "confidence"]

    # Unified columns: STM32 data followed by EMG data (excluding EMG's t_pc/t_tmsi)
    # This creates a single wide matrix indexed by t_common
    UNIFIED_COLUMNS = STM32_COLUMNS + EMG_COLUMNS[2:]

    # Synced dataset columns: common PC clock prepended, then all signals
    # Stored under _raw/synced so it appears alongside the other raw streams
    SYNCED_COLUMNS = ["t_pc_common"] + UNIFIED_COLUMNS

    def __init__(
        self, 
        base_dir: str, 
        participant_id: str, 
        trial_num: int,
        metadata: Dict[str, Any]
    ):
        self.participant_id = participant_id
        self.trial_num = trial_num
        self.metadata = metadata
        
        # Setup directories
        self.base_dir = Path(base_dir)
        self.participant_dir = self.base_dir / f"participant_{participant_id}"
        self.participant_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.filename = self.participant_dir / f"trial_{trial_num}_{timestamp}.h5"
        
        # Initialize file and dataset structures
        self._init_file()
        
    def _init_file(self):
        """Create the HDF5 file and base datasets with initial metadata."""
        with h5py.File(self.filename, 'w') as f:
            # 1. Store global physiological and technical metadata
            f.attrs['participant_id'] = self.participant_id
            f.attrs['trial_num'] = self.trial_num
            f.attrs['creation_time'] = time.time()
            
            for k, v in self.metadata.items():
                if v is not None:
                    f.attrs[k] = v
            
            # 2. Setup Internal Groups (Hidden-ish for raw data)
            g_raw = f.create_group('_raw')
            
            # 3. Define raw datasets (extendable along axis 0)
            # STM32 Raw: 22 columns
            ds_stm32 = g_raw.create_dataset(
                'stm32', shape=(0, len(self.STM32_COLUMNS)), 
                maxshape=(None, len(self.STM32_COLUMNS)), 
                dtype='float64', compression="gzip", compression_opts=4,
                chunks=(1000, len(self.STM32_COLUMNS))
            )
            ds_stm32.attrs['column_names'] = self.STM32_COLUMNS
            
            # EMG Raw: 35 columns
            ds_emg = g_raw.create_dataset(
                'emg', shape=(0, len(self.EMG_COLUMNS)), 
                maxshape=(None, len(self.EMG_COLUMNS)), 
                dtype='float64', compression="gzip", compression_opts=4,
                chunks=(1000, len(self.EMG_COLUMNS))
            )
            ds_emg.attrs['column_names'] = self.EMG_COLUMNS

            # Sync Metrics: 3 columns
            ds_sync = g_raw.create_dataset(
                'metrics', shape=(0, len(self.SYNC_METRICS_COLUMNS)), 
                maxshape=(None, len(self.SYNC_METRICS_COLUMNS)), 
                dtype='float64', compression="gzip", compression_opts=4,
                chunks=(100, len(self.SYNC_METRICS_COLUMNS))
            )
            ds_sync.attrs['column_names'] = self.SYNC_METRICS_COLUMNS
            
            # 4. Pre-create the top-level /synced group and its dataset
            # (populated at trial stop via save_synchronized_datasets)
            # t_pc_common (1) + STM32 cols (22) + EMG signal cols (33) = 56 columns
            n_synced_cols = len(self.SYNCED_COLUMNS)
            g_synced = f.create_group('synced')
            ds_synced = g_synced.create_dataset(
                'data',
                shape=(0, n_synced_cols),
                maxshape=(None, n_synced_cols),
                dtype='float64', compression="gzip", compression_opts=4,
                chunks=(2000, n_synced_cols)
            )
            ds_synced.attrs['column_names'] = self.SYNCED_COLUMNS
            ds_synced.attrs['description'] = (
                "Unified synced matrix: t_pc_common + STM32 + EMG on a shared PC clock. "
                "STM32 data is interpolated onto the EMG sample grid using Kalman-smoothed delay."
            )

    def append_stm32_data(self, data: np.ndarray):
        """Append 2D array of raw STM32 data (shape: N, 22)"""
        cols = len(self.STM32_COLUMNS)
        if data.size == 0 or len(data.shape) != 2 or data.shape[1] != cols:
            return
            
        with h5py.File(self.filename, 'a') as f:
            dset = f['_raw/stm32']
            curr_len = dset.shape[0]
            new_len = curr_len + data.shape[0]
            dset.resize(new_len, axis=0)
            dset[curr_len:new_len, :] = data

    def append_emg_data(self, data: np.ndarray):
        """Append 2D array of raw EMG data (shape: N, 35)"""
        cols = len(self.EMG_COLUMNS)
        if data.size == 0 or len(data.shape) != 2 or data.shape[1] != cols:
            return
            
        with h5py.File(self.filename, 'a') as f:
            dset = f['_raw/emg']
            curr_len = dset.shape[0]
            new_len = curr_len + data.shape[0]
            dset.resize(new_len, axis=0)
            dset[curr_len:new_len, :] = data

    def append_sync_metrics(self, data: np.ndarray):
        """Append 2D array of sync metrics (shape: N, 3)"""
        if data.size == 0 or len(data.shape) != 2 or data.shape[1] != 3:
            return
            
        with h5py.File(self.filename, 'a') as f:
            dset = f['_raw/metrics']
            curr_len = dset.shape[0]
            new_len = curr_len + data.shape[0]
            dset.resize(new_len, axis=0)
            dset[curr_len:new_len, :] = data

    def save_synchronized_datasets(
        self, 
        t_common: np.ndarray, 
        unified_matrix: np.ndarray,
        kalman_delay_ms: Optional[float] = None,
        kalman_drift_ppm: Optional[float] = None
    ):
        """
        Writes the final unified synced matrix into _raw/synced.
        The matrix has t_pc_common prepended as column 0, followed by
        all STM32 signal columns and all EMG signal columns.
        """
        # Prepend t_common as column 0
        synced_matrix = np.column_stack((t_common, unified_matrix))

        with h5py.File(self.filename, 'a') as f:
            # 1. Write into the pre-allocated /synced/data dataset
            dset = f['synced/data']
            n = synced_matrix.shape[0]
            dset.resize(n, axis=0)
            dset[:n, :] = synced_matrix

            # 2. Attach final Kalman sync metrics as root attributes
            if kalman_delay_ms is not None:
                f.attrs['final_kalman_delay_ms'] = kalman_delay_ms
            if kalman_drift_ppm is not None:
                f.attrs['final_kalman_drift_ppm'] = kalman_drift_ppm

        print(
            f"[LOGGER] Wrote _raw/synced: shape={synced_matrix.shape}, "
            f"cols: t_pc_common + {len(self.UNIFIED_COLUMNS)} signal columns."
        )
            
    def close(self):
        """Finalize the file if any background resources were held."""
        pass # All functions use context managers so file is safely closed after every write
