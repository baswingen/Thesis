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
            
            # 2. Setup Groups
            g_stm32 = f.create_group('stm32')
            g_emg = f.create_group('emg')
            g_sync = f.create_group('sync')

            # 3. Define raw datasets (extendable along axis 0)
            # Optimized for ML: compression level 4 (balance) and chunking
            
            # STM32 Raw: 22 columns
            ds_stm32 = g_stm32.create_dataset(
                'raw', shape=(0, len(self.STM32_COLUMNS)), 
                maxshape=(None, len(self.STM32_COLUMNS)), 
                dtype='float64', compression="gzip", compression_opts=4,
                chunks=(1000, len(self.STM32_COLUMNS)) # ~1s chunks at 1kHz
            )
            ds_stm32.attrs['column_names'] = self.STM32_COLUMNS
            ds_stm32.attrs['interpreter'] = 'curve'
            
            # EMG Raw: 35 columns
            ds_emg = g_emg.create_dataset(
                'raw', shape=(0, len(self.EMG_COLUMNS)), 
                maxshape=(None, len(self.EMG_COLUMNS)), 
                dtype='float64', compression="gzip", compression_opts=4,
                chunks=(1000, len(self.EMG_COLUMNS)) # ~1s chunks at 1kHz
            )
            ds_emg.attrs['column_names'] = self.EMG_COLUMNS
            ds_emg.attrs['interpreter'] = 'curve'

            # Sync Metrics: 3 columns
            ds_sync = g_sync.create_dataset(
                'metrics', shape=(0, len(self.SYNC_METRICS_COLUMNS)), 
                maxshape=(None, len(self.SYNC_METRICS_COLUMNS)), 
                dtype='float64', compression="gzip", compression_opts=4,
                chunks=(100, len(self.SYNC_METRICS_COLUMNS))
            )
            ds_sync.attrs['column_names'] = self.SYNC_METRICS_COLUMNS
            ds_sync.attrs['interpreter'] = 'curve'
            
            # 4. Define synchronized data arrays (saved at the END of the trial)
            # We don't stream these dynamically because time-shifting relies on 
            # calculating the delay over the whole trial, or retrospectively aligning.
            # We initialize empty datasets to be populated on close().
            # Note: The unified clock reference will be the STM32's time vector.
            
    def append_stm32_data(self, data: np.ndarray):
        """Append 2D array of raw STM32 data (shape: N, 22)"""
        cols = len(self.STM32_COLUMNS)
        if data.size == 0 or len(data.shape) != 2 or data.shape[1] != cols:
            return
            
        with h5py.File(self.filename, 'a') as f:
            dset = f['stm32/raw']
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
            dset = f['emg/raw']
            curr_len = dset.shape[0]
            new_len = curr_len + data.shape[0]
            dset.resize(new_len, axis=0)
            dset[curr_len:new_len, :] = data

    def append_sync_metrics(self, data: np.ndarray):
        """Append 2D array of sync metrics (shape: N, 3)"""
        if data.size == 0 or len(data.shape) != 2 or data.shape[1] != 3:
            return
            
        with h5py.File(self.filename, 'a') as f:
            dset = f['sync/metrics']
            curr_len = dset.shape[0]
            new_len = curr_len + data.shape[0]
            dset.resize(new_len, axis=0)
            dset[curr_len:new_len, :] = data

    def save_synchronized_datasets(
        self, 
        t_common: np.ndarray, 
        stm32_synced: np.ndarray, 
        emg_synced: np.ndarray,
        emg_synced_processed: np.ndarray,
        unified_matrix: Optional[np.ndarray] = None,
        kalman_delay_ms: Optional[float] = None,
        kalman_drift_ppm: Optional[float] = None
    ):
        """
        Saves the final post-processed, time-aligned datasets sharing a common clock.
        Called at the end of the trial.
        
        Args:
            t_common: Shared 1D time array (e.g. STM32 aligned wall time or trial time)
            stm32_synced: Aligned STM32 data (N, features)
            emg_synced:  Aligned raw EMG data (N, channels)
            emg_synced_processed: Filtered/enveloped aligned EMG data (N, channels)
            unified_matrix: Optional full matrix combine (N, STM32 + EMG_features)
            kalman_delay_ms: The final smoothed delay estimate
            kalman_drift_ppm: The final estimated clock drift
        """
        with h5py.File(self.filename, 'a') as f:
            # Create a common time array dataset for reference
            ds_t = f.create_dataset('sync/t_common', data=t_common, compression="gzip", compression_opts=4)
            ds_t.attrs['units'] = 's'
            
            # Create the data arrays (dropping them in their respective groups)
            ds_stm32 = f.create_dataset('stm32/synced', data=stm32_synced, compression="gzip", compression_opts=4)
            ds_emg = f.create_dataset('emg/synced', data=emg_synced, compression="gzip", compression_opts=4)
            ds_emg_proc = f.create_dataset('emg/synced_processed', data=emg_synced_processed, compression="gzip", compression_opts=4)
            
            # Store metadata for H5Web and ML
            ds_stm32.attrs['description'] = "STM32 data aligned to t_common"
            ds_stm32.attrs['column_names'] = self.STM32_COLUMNS
            ds_stm32.attrs['interpreter'] = 'curve'
            
            ds_emg.attrs['description'] = "Raw EMG data interpolated/aligned to t_common"
            ds_emg.attrs['column_names'] = self.EMG_COLUMNS
            ds_emg.attrs['interpreter'] = 'curve'
            
            ds_emg_proc.attrs['description'] = "Processed (enveloped) EMG data aligned to t_common"
            ds_emg_proc.attrs['column_names'] = self.EMG_COLUMNS # assuming same channel mapping
            ds_emg_proc.attrs['interpreter'] = 'curve'

            # 6. Save Unified Matrix if provided
            if unified_matrix is not None:
                ds_unified = f.create_dataset('sync/unified', data=unified_matrix, compression="gzip", compression_opts=4)
                ds_unified.attrs['description'] = "Unified matrix: STM32 + EMG features synced on t_common"
                ds_unified.attrs['column_names'] = self.UNIFIED_COLUMNS
                ds_unified.attrs['interpreter'] = 'curve'
                
                if kalman_delay_ms is not None:
                    ds_unified.attrs['kalman_delay_ms'] = kalman_delay_ms
                if kalman_drift_ppm is not None:
                    ds_unified.attrs['kalman_drift_ppm'] = kalman_drift_ppm
            
    def close(self):
        """Finalize the file if any background resources were held."""
        pass # All functions use context managers so file is safely closed after every write
