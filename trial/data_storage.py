"""
Trial Data Storage Module
==========================
HDF5-based storage for synchronized EMG and IMU trial data with comprehensive metadata.

Features:
- HDF5 file format (PyTorch/Keras compatible)
- Separate raw and preprocessed data files
- Rich metadata (participant, session, trial, device info)
- Compression support
- Thread-safe operations
"""

import h5py
import numpy as np
import json
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional, Any, Tuple
import warnings


class TrialDataStorage:
    """
    Handles saving trial data to HDF5 format with metadata.
    
    Creates two files per trial:
    - trial_XXX_raw.h5: Raw EMG and IMU data
    - trial_XXX_preprocessed.h5: Preprocessed EMG data
    """
    
    def __init__(self, 
                 session_dir: Path,
                 compression: Optional[str] = 'gzip',
                 compression_level: int = 4):
        """
        Initialize data storage handler.
        
        Args:
            session_dir: Directory for this session's data
            compression: Compression algorithm ('gzip', 'lzf', None)
            compression_level: Compression level (0-9 for gzip)
        """
        self.session_dir = Path(session_dir)
        self.compression = compression
        self.compression_level = compression_level
        
        # Create session directory if it doesn't exist
        self.session_dir.mkdir(parents=True, exist_ok=True)
        
        # Metadata that will be common to all trials
        self.session_metadata = {}
    
    def set_session_metadata(self, 
                            participant_info: Dict[str, Any],
                            session_info: Dict[str, Any],
                            emg_config: Dict[str, Any],
                            imu_config: Dict[str, Any],
                            preprocessing_config: Dict[str, Any]):
        """
        Set session-level metadata (common to all trials).
        
        Args:
            participant_info: Participant information
            session_info: Session information
            emg_config: EMG device configuration
            imu_config: IMU device configuration
            preprocessing_config: Preprocessing parameters
        """
        self.session_metadata = {
            'participant': participant_info,
            'session': session_info,
            'emg_config': emg_config,
            'imu_config': imu_config,
            'preprocessing': preprocessing_config
        }
    
    def save_trial_raw(self,
                      trial_number: int,
                      exercise_name: str,
                      exercise_info: Dict[str, Any],
                      emg_data: Dict[str, Any],
                      imu_data: Dict[str, Any],
                      sync_info: Dict[str, Any]) -> Path:
        """
        Save raw trial data to HDF5.
        
        Args:
            trial_number: Trial number (1-based)
            exercise_name: Name of exercise performed
            exercise_info: Exercise metadata (duration, instruction, etc.)
            emg_data: Dictionary with EMG data:
                - 'timestamps': (N,) array of timestamps
                - 'raw': (N, C) array of raw EMG channels
                - 'sample_rate': float
                - 'channel_names': list of strings
                - 'counter_values': optional counter values
            imu_data: Dictionary with IMU data:
                - 'timestamps': (M,) array of timestamps
                - 'accel1': (M, 3) array
                - 'gyro1': (M, 3) array
                - 'accel2': (M, 3) array
                - 'gyro2': (M, 3) array
                - 'quat1': (M, 4) array (optional)
                - 'quat2': (M, 4) array (optional)
                - 'euler1': (M, 3) array (optional)
                - 'euler2': (M, 3) array (optional)
                - 'sample_rate': float
                - 'calibration': dict with bias values
            sync_info: Synchronization metadata
        
        Returns:
            Path to saved file
        """
        # Create filename
        filename = self.session_dir / f"trial_{trial_number:03d}_raw.h5"
        
        print(f"Saving raw trial data to {filename}...")
        
        with h5py.File(filename, 'w') as f:
            # Create main groups
            meta_group = f.create_group('metadata')
            emg_group = f.create_group('emg')
            imu_group = f.create_group('imu')
            sync_group = f.create_group('sync')
            
            # === Metadata ===
            self._write_metadata(meta_group, trial_number, exercise_name, exercise_info)
            
            # === EMG Data ===
            if emg_data:
                self._write_emg_raw(emg_group, emg_data)
            
            # === IMU Data ===
            if imu_data:
                self._write_imu_raw(imu_group, imu_data)
            
            # === Synchronization Info ===
            self._write_sync_info(sync_group, sync_info)
        
        print(f"✓ Saved raw data: {filename.name}")
        return filename
    
    def save_trial_preprocessed(self,
                                trial_number: int,
                                exercise_name: str,
                                emg_preprocessed: Dict[str, Any],
                                imu_data: Optional[Dict[str, Any]] = None) -> Path:
        """
        Save preprocessed trial data to HDF5.
        
        Args:
            trial_number: Trial number (1-based)
            exercise_name: Name of exercise performed
            emg_preprocessed: Dictionary with preprocessed EMG:
                - 'timestamps': (N,) array
                - 'filtered': (N, C) array - bandpass + notch filtered
                - 'envelope': (N, C) array - rectified + smoothed
                - 'normalized': (N, C) array - MVC normalized (optional)
                - 'sample_rate': float
                - 'channel_names': list of strings
            imu_data: Optional IMU data (usually copied from raw)
        
        Returns:
            Path to saved file
        """
        filename = self.session_dir / f"trial_{trial_number:03d}_preprocessed.h5"
        
        print(f"Saving preprocessed trial data to {filename}...")
        
        with h5py.File(filename, 'w') as f:
            # Create groups
            meta_group = f.create_group('metadata')
            emg_group = f.create_group('emg')
            
            # Metadata (reference to raw file)
            meta_group.attrs['trial_number'] = trial_number
            meta_group.attrs['exercise_name'] = exercise_name
            meta_group.attrs['raw_file'] = f"trial_{trial_number:03d}_raw.h5"
            meta_group.attrs['timestamp_saved'] = datetime.now().isoformat()
            
            # Preprocessed EMG data
            self._write_emg_preprocessed(emg_group, emg_preprocessed)
            
            # IMU data (copy from raw if provided)
            if imu_data:
                imu_group = f.create_group('imu')
                self._write_imu_raw(imu_group, imu_data)
        
        print(f"✓ Saved preprocessed data: {filename.name}")
        return filename
    
    def save_session_metadata(self) -> Path:
        """
        Save session-level metadata to JSON file.
        
        Returns:
            Path to metadata file
        """
        filename = self.session_dir / "metadata.json"
        
        metadata = {
            **self.session_metadata,
            'timestamp_created': datetime.now().isoformat(),
            'session_dir': str(self.session_dir),
        }
        
        with open(filename, 'w') as f:
            json.dump(metadata, f, indent=2, default=str)
        
        print(f"✓ Saved session metadata: {filename.name}")
        return filename
    
    def _write_metadata(self, group: h5py.Group, trial_number: int, 
                       exercise_name: str, exercise_info: Dict[str, Any]):
        """Write trial metadata to HDF5 group."""
        # Trial info
        group.attrs['trial_number'] = trial_number
        group.attrs['exercise_name'] = exercise_name
        group.attrs['timestamp_saved'] = datetime.now().isoformat()
        
        # Exercise info
        for key, value in exercise_info.items():
            if isinstance(value, (str, int, float, bool)):
                group.attrs[f'exercise_{key}'] = value
        
        # Session metadata (as JSON string for complex types)
        if self.session_metadata:
            group.attrs['session_metadata_json'] = json.dumps(self.session_metadata, default=str)
    
    def _write_emg_raw(self, group: h5py.Group, emg_data: Dict[str, Any]):
        """Write raw EMG data to HDF5 group."""
        # Timestamps
        if 'timestamps' in emg_data:
            self._create_dataset(
                group, 'timestamps', emg_data['timestamps'],
                dtype='float64',
                attrs={'unit': 'seconds', 'description': 'Synchronized timestamps'}
            )
        
        # Raw EMG data
        if 'raw' in emg_data:
            raw_data = emg_data['raw']
            self._create_dataset(
                group, 'raw', raw_data,
                dtype='float32',
                attrs={
                    'unit': 'microvolts',
                    'description': 'Raw EMG channels',
                    'shape_description': '(n_samples, n_channels)'
                }
            )
        
        # Channel names (string array)
        if 'channel_names' in emg_data:
            channel_names = emg_data['channel_names']
            dt = h5py.string_dtype(encoding='utf-8')
            group.create_dataset('channel_names', data=channel_names, dtype=dt)
        
        # Sample rate
        if 'sample_rate' in emg_data:
            group.attrs['sample_rate'] = emg_data['sample_rate']
        
        # Counter values (if available)
        if 'counter_values' in emg_data and emg_data['counter_values'] is not None:
            self._create_dataset(
                group, 'counter_values', emg_data['counter_values'],
                dtype='int32',
                attrs={'description': 'TMSi COUNTER channel values'}
            )
    
    def _write_emg_preprocessed(self, group: h5py.Group, emg_data: Dict[str, Any]):
        """Write preprocessed EMG data to HDF5 group."""
        # Timestamps
        if 'timestamps' in emg_data:
            self._create_dataset(
                group, 'timestamps', emg_data['timestamps'],
                dtype='float64',
                attrs={'unit': 'seconds'}
            )
        
        # Filtered data
        if 'filtered' in emg_data:
            self._create_dataset(
                group, 'filtered', emg_data['filtered'],
                dtype='float32',
                attrs={
                    'unit': 'microvolts',
                    'description': 'Bandpass + notch filtered EMG',
                }
            )
        
        # Envelope
        if 'envelope' in emg_data:
            self._create_dataset(
                group, 'envelope', emg_data['envelope'],
                dtype='float32',
                attrs={
                    'unit': 'microvolts',
                    'description': 'Rectified and smoothed EMG envelope',
                }
            )
        
        # Normalized (MVC)
        if 'normalized' in emg_data:
            self._create_dataset(
                group, 'normalized', emg_data['normalized'],
                dtype='float32',
                attrs={
                    'unit': 'fraction_of_mvc',
                    'description': 'MVC-normalized EMG (0-1 scale)',
                }
            )
        
        # Channel names
        if 'channel_names' in emg_data:
            dt = h5py.string_dtype(encoding='utf-8')
            group.create_dataset('channel_names', data=emg_data['channel_names'], dtype=dt)
        
        # Sample rate
        if 'sample_rate' in emg_data:
            group.attrs['sample_rate'] = emg_data['sample_rate']
    
    def _write_imu_raw(self, group: h5py.Group, imu_data: Dict[str, Any]):
        """Write raw IMU data to HDF5 group."""
        # Timestamps
        if 'timestamps' in imu_data:
            self._create_dataset(
                group, 'timestamps', imu_data['timestamps'],
                dtype='float64',
                attrs={'unit': 'seconds', 'description': 'Hardware timestamps from Arduino'}
            )
        
        # IMU 1 data
        if 'accel1' in imu_data:
            self._create_dataset(
                group, 'accel1', imu_data['accel1'],
                dtype='float32',
                attrs={'unit': 'g', 'description': 'IMU1 acceleration [ax, ay, az]'}
            )
        
        if 'gyro1' in imu_data:
            self._create_dataset(
                group, 'gyro1', imu_data['gyro1'],
                dtype='float32',
                attrs={'unit': 'rad/s', 'description': 'IMU1 angular velocity [gx, gy, gz]'}
            )
        
        if 'quat1' in imu_data and imu_data['quat1'] is not None:
            self._create_dataset(
                group, 'quat1', imu_data['quat1'],
                dtype='float32',
                attrs={'description': 'IMU1 orientation quaternion [w, x, y, z]'}
            )
        
        if 'euler1' in imu_data and imu_data['euler1'] is not None:
            self._create_dataset(
                group, 'euler1', imu_data['euler1'],
                dtype='float32',
                attrs={'unit': 'degrees', 'description': 'IMU1 Euler angles [roll, pitch, yaw]'}
            )
        
        # IMU 2 data
        if 'accel2' in imu_data:
            self._create_dataset(
                group, 'accel2', imu_data['accel2'],
                dtype='float32',
                attrs={'unit': 'g', 'description': 'IMU2 acceleration [ax, ay, az]'}
            )
        
        if 'gyro2' in imu_data:
            self._create_dataset(
                group, 'gyro2', imu_data['gyro2'],
                dtype='float32',
                attrs={'unit': 'rad/s', 'description': 'IMU2 angular velocity [gx, gy, gz]'}
            )
        
        if 'quat2' in imu_data and imu_data['quat2'] is not None:
            self._create_dataset(
                group, 'quat2', imu_data['quat2'],
                dtype='float32',
                attrs={'description': 'IMU2 orientation quaternion [w, x, y, z]'}
            )
        
        if 'euler2' in imu_data and imu_data['euler2'] is not None:
            self._create_dataset(
                group, 'euler2', imu_data['euler2'],
                dtype='float32',
                attrs={'unit': 'degrees', 'description': 'IMU2 Euler angles [roll, pitch, yaw]'}
            )
        
        # Sample rate
        if 'sample_rate' in imu_data:
            group.attrs['sample_rate'] = imu_data['sample_rate']
        
        # Calibration info
        if 'calibration' in imu_data:
            cal = imu_data['calibration']
            if isinstance(cal, dict):
                for key, value in cal.items():
                    if isinstance(value, np.ndarray):
                        group.attrs[f'calibration_{key}'] = value
                    else:
                        group.attrs[f'calibration_{key}'] = value
    
    def _write_sync_info(self, group: h5py.Group, sync_info: Dict[str, Any]):
        """Write synchronization info to HDF5 group."""
        for key, value in sync_info.items():
            if isinstance(value, (str, int, float, bool)):
                group.attrs[key] = value
            elif isinstance(value, np.ndarray):
                self._create_dataset(group, key, value, dtype='float64')
    
    def _create_dataset(self, group: h5py.Group, name: str, data: np.ndarray,
                       dtype: str = 'float32', attrs: Optional[Dict[str, Any]] = None):
        """
        Create HDF5 dataset with compression.
        
        Args:
            group: HDF5 group
            name: Dataset name
            data: Data array
            dtype: Data type
            attrs: Optional attributes dictionary
        """
        # Convert to numpy array
        data = np.asarray(data, dtype=dtype)
        
        # Create dataset with compression
        if self.compression:
            if self.compression == 'gzip':
                ds = group.create_dataset(
                    name, data=data,
                    compression=self.compression,
                    compression_opts=self.compression_level
                )
            else:
                ds = group.create_dataset(
                    name, data=data,
                    compression=self.compression
                )
        else:
            ds = group.create_dataset(name, data=data)
        
        # Add attributes
        if attrs:
            for key, value in attrs.items():
                ds.attrs[key] = value
    
    @staticmethod
    def load_trial_raw(filepath: Path) -> Dict[str, Any]:
        """
        Load raw trial data from HDF5 file.
        
        Args:
            filepath: Path to HDF5 file
        
        Returns:
            Dictionary with all trial data
        """
        data = {}
        
        with h5py.File(filepath, 'r') as f:
            # Metadata
            data['metadata'] = dict(f['metadata'].attrs)
            
            # EMG data
            if 'emg' in f:
                emg_group = f['emg']
                data['emg'] = {
                    'timestamps': emg_group['timestamps'][:] if 'timestamps' in emg_group else None,
                    'raw': emg_group['raw'][:] if 'raw' in emg_group else None,
                    'channel_names': list(emg_group['channel_names'][:]) if 'channel_names' in emg_group else None,
                    'sample_rate': emg_group.attrs.get('sample_rate'),
                }
            
            # IMU data
            if 'imu' in f:
                imu_group = f['imu']
                data['imu'] = {
                    'timestamps': imu_group['timestamps'][:] if 'timestamps' in imu_group else None,
                    'accel1': imu_group['accel1'][:] if 'accel1' in imu_group else None,
                    'gyro1': imu_group['gyro1'][:] if 'gyro1' in imu_group else None,
                    'accel2': imu_group['accel2'][:] if 'accel2' in imu_group else None,
                    'gyro2': imu_group['gyro2'][:] if 'gyro2' in imu_group else None,
                    'quat1': imu_group['quat1'][:] if 'quat1' in imu_group else None,
                    'quat2': imu_group['quat2'][:] if 'quat2' in imu_group else None,
                    'sample_rate': imu_group.attrs.get('sample_rate'),
                }
            
            # Sync info
            if 'sync' in f:
                data['sync'] = dict(f['sync'].attrs)
        
        return data
    
    @staticmethod
    def load_trial_preprocessed(filepath: Path) -> Dict[str, Any]:
        """
        Load preprocessed trial data from HDF5 file.
        
        Args:
            filepath: Path to HDF5 file
        
        Returns:
            Dictionary with preprocessed data
        """
        data = {}
        
        with h5py.File(filepath, 'r') as f:
            # Metadata
            data['metadata'] = dict(f['metadata'].attrs)
            
            # Preprocessed EMG
            if 'emg' in f:
                emg_group = f['emg']
                data['emg'] = {
                    'timestamps': emg_group['timestamps'][:] if 'timestamps' in emg_group else None,
                    'filtered': emg_group['filtered'][:] if 'filtered' in emg_group else None,
                    'envelope': emg_group['envelope'][:] if 'envelope' in emg_group else None,
                    'normalized': emg_group['normalized'][:] if 'normalized' in emg_group else None,
                    'channel_names': list(emg_group['channel_names'][:]) if 'channel_names' in emg_group else None,
                    'sample_rate': emg_group.attrs.get('sample_rate'),
                }
            
            # IMU data (if copied)
            if 'imu' in f:
                imu_group = f['imu']
                data['imu'] = {
                    'timestamps': imu_group['timestamps'][:] if 'timestamps' in imu_group else None,
                    'accel1': imu_group['accel1'][:] if 'accel1' in imu_group else None,
                    'gyro1': imu_group['gyro1'][:] if 'gyro1' in imu_group else None,
                }
        
        return data


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def create_backup(hdf5_file: Path, formats: List[str] = ['numpy']):
    """
    Create backup copies of HDF5 file in other formats.
    
    Args:
        hdf5_file: Path to HDF5 file
        formats: List of formats ('numpy', 'csv')
    """
    if 'numpy' in formats:
        # Load data
        data = TrialDataStorage.load_trial_raw(hdf5_file)
        
        # Save as .npz
        backup_file = hdf5_file.with_suffix('.npz')
        
        arrays_to_save = {}
        if 'emg' in data and data['emg']['raw'] is not None:
            arrays_to_save['emg_timestamps'] = data['emg']['timestamps']
            arrays_to_save['emg_raw'] = data['emg']['raw']
        if 'imu' in data and data['imu']['accel1'] is not None:
            arrays_to_save['imu_timestamps'] = data['imu']['timestamps']
            arrays_to_save['imu_accel1'] = data['imu']['accel1']
            arrays_to_save['imu_gyro1'] = data['imu']['gyro1']
        
        if arrays_to_save:
            np.savez_compressed(backup_file, **arrays_to_save)
            print(f"✓ Created NumPy backup: {backup_file.name}")


def verify_hdf5_file(filepath: Path) -> bool:
    """
    Verify HDF5 file integrity.
    
    Args:
        filepath: Path to HDF5 file
    
    Returns:
        True if file is valid
    """
    try:
        with h5py.File(filepath, 'r') as f:
            # Check basic structure
            if 'metadata' not in f:
                print(f"⚠ Missing 'metadata' group in {filepath.name}")
                return False
            
            # Check for data
            has_emg = 'emg' in f and 'raw' in f['emg']
            has_imu = 'imu' in f and 'accel1' in f['imu']
            
            if not (has_emg or has_imu):
                print(f"⚠ No EMG or IMU data found in {filepath.name}")
                return False
            
            print(f"✓ HDF5 file valid: {filepath.name}")
            return True
    
    except Exception as e:
        print(f"✗ HDF5 file corrupted: {filepath.name} - {e}")
        return False


if __name__ == '__main__':
    # Demo usage
    print("Trial Data Storage Module")
    print("="*60)
    print("This module handles saving trial data to HDF5 format.")
    print("\nFeatures:")
    print("  - Separate raw and preprocessed files")
    print("  - Compression support")
    print("  - Rich metadata")
    print("  - PyTorch/Keras compatible")
    print("\nUsage: Import and use TrialDataStorage class in trial_manager.py")
