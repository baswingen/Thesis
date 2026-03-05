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
    
    # Sync Metrics: ["t_effective", "delay_ms", "confidence"]
    SYNC_METRICS_COLUMNS = ["t_effective", "delay_ms", "confidence"]

    # Events Columns matching TrialLogic queue
    EVENT_COLUMNS = ["t_pc", "state", "weight", "src_r", "src_c", "tgt_r", "tgt_c"]
    
    # State string mapping to IDs for numerical matrix
    STATE_MAPPING = {
        "CONFIGURING": 0,
        "PHASE_1_PLACEMENT": 1,
        "PHASE_2_AWAITING_PICKUP": 2,
        "PHASE_2_AWAITING_PLACEMENT": 3,
        "ERROR_WRONG_PICKUP": 4,
        "ERROR_WRONG_PLACEMENT": 5,
        "ERROR_MISSING_WEIGHT": 6,
        "FINISHED": 7
    }

    # Unified columns: STM32 data followed by EMG data (excluding EMG's t_pc/t_tmsi)
    # This creates a single wide matrix indexed by t_common
    UNIFIED_COLUMNS = STM32_COLUMNS + EMG_COLUMNS[2:]
    
    # ML Label Columns added to the matrix via step-filling
    LABEL_COLUMNS = ["state_id", "weight", "src_r", "src_c", "tgt_r", "tgt_c"]

    # Synced dataset columns: common PC clock prepended, then all signals, then ML labels
    # Stored under _raw/synced so it appears alongside the other raw streams
    SYNCED_COLUMNS = ["t_pc_common"] + UNIFIED_COLUMNS + LABEL_COLUMNS

    def __init__(
        self, 
        base_dir: str, 
        participant_id: str, 
        session_num: int | str,
        trial_num: int,
        metadata: Dict[str, Any]
    ):
        self.participant_id = participant_id
        self.session_num = session_num
        self.trial_num = trial_num
        self.metadata = metadata
        
        # Setup directories: database/participant_PXX/session_NN/
        self.base_dir = Path(base_dir)
        self.participant_dir = self.base_dir / f"participant_{participant_id}"
        self.session_dir = self.participant_dir / f"session_{int(session_num):02d}"
        self.session_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.filename = self.session_dir / f"trial_{trial_num}_{timestamp}.h5"
        
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

            # Save everything from trial_config.py
            try:
                import trial.trial_config as tc
                import json
                f.attrs['config_stm32_connections'] = json.dumps(tc.STM32_CONNECTIONS)
                f.attrs['config_emg_connections'] = json.dumps(tc.EMG_CONNECTIONS)
                f.attrs['config_emg_plot_labels'] = json.dumps(tc.EMG_PLOT_LABELS)
                f.attrs['config_participant'] = json.dumps(tc.PARTICIPANT_CONFIG)
                f.attrs['config_trial_logic'] = json.dumps(tc.TRIAL_LOGIC_CONFIG)
                f.attrs['config_starting_slots'] = json.dumps(tc.STARTING_SLOTS_CONFIG)
            except Exception as e:
                print(f"[LOGGER] Failed to save trial_config metadata: {e}")
            
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
            
            # Events: Variable length strings and numerical data
            # HDF5 string type
            dt_str = h5py.string_dtype(encoding='utf-8')
            dt_event = np.dtype([
                ('t_pc', 'f8'), ('state', dt_str), ('weight', 'f8'),
                ('src_r', 'i4'), ('src_c', 'i4'), ('tgt_r', 'i4'), ('tgt_c', 'i4')
            ])
            ds_event = g_raw.create_dataset(
                'events', shape=(0,), 
                maxshape=(None,), 
                dtype=dt_event, compression="gzip", compression_opts=4,
                chunks=(100,)
            )
            ds_event.attrs['column_names'] = self.EVENT_COLUMNS
            
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

    def append_events(self, events: list):
        """Append a list of event tuples to the logger."""
        if not events:
            return
            
        # Convert to numpy structured array
        dt_str = h5py.string_dtype(encoding='utf-8')
        dt_event = np.dtype([
            ('t_pc', 'f8'), ('state', dt_str), ('weight', 'f8'),
            ('src_r', 'i4'), ('src_c', 'i4'), ('tgt_r', 'i4'), ('tgt_c', 'i4')
        ])
        
        arr = np.array(events, dtype=dt_event)
        
        with h5py.File(self.filename, 'a') as f:
            dset = f['_raw/events']
            curr_len = dset.shape[0]
            new_len = curr_len + len(arr)
            dset.resize(new_len, axis=0)
            dset[curr_len:new_len] = arr

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
        all STM32 signal columns, followed by all EMG signal columns, 
        and lastly step-filled ML label columns.
        """
        # Step-fill ML label columns based on events
        labels = np.zeros((len(t_common), len(self.LABEL_COLUMNS)), dtype=np.float64)
        
        with h5py.File(self.filename, 'r') as f:
            if '_raw/events' in f and f['_raw/events'].shape[0] > 0:
                events = f['_raw/events'][:]
                evt_times = events['t_pc']
                
                # For each event, map state string to ID
                numeric_events = np.zeros((len(events), len(self.LABEL_COLUMNS)))
                for i, e in enumerate(events):
                    state_str = e['state'].decode('utf-8') if isinstance(e['state'], bytes) else e['state']
                    state_id = self.STATE_MAPPING.get(state_str, -1)
                    numeric_events[i] = [state_id, e['weight'], e['src_r'], e['src_c'], e['tgt_r'], e['tgt_c']]
                
                # Use searchsorted to find which event is active at each t_common
                # searchsorted with side='right' returns the insertion index
                # We subtract 1 to get the index of the latest event *before* (or at) t_common
                idx = np.searchsorted(evt_times, t_common, side='right') - 1
                
                # Where idx is < 0, no event had occurred yet. Keep zeroes.
                valid_mask = idx >= 0
                idx_clamped = np.maximum(0, idx)
                
                # Check bounds
                idx_clamped = np.minimum(idx_clamped, len(numeric_events) - 1)
                labels[valid_mask] = numeric_events[idx_clamped[valid_mask]] 

        # Prepend t_common as column 0, and append labels at the end
        synced_matrix = np.column_stack((t_common, unified_matrix, labels))

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
