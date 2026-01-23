"""Thesis project package."""

__version__ = "0.1.0"

# Import main EMG acquisition interface
from .emg_acquisition import (
    EMGDevice, 
    acquire_emg_data,
    EMGChannelSelector,
    fix_channel_name
)

# Import EMG signal processing
from .emg_processing import (
    BandpassFilter,
    NotchFilter,
    EMGEnvelopeExtractor,
    EMGProcessor,
    calculate_signal_quality
)

# Import EMG visualization
from .emg_visualization import (
    EMGPlotter,
    MultiChannelPlotter,
    quick_plot_emg
)

# Import main IMU acquisition interface (dual BMI160 with ASCII protocol)
from .imu_acquisition import (
    IMUDevice, 
    acquire_imu_data,
    IMUReading,
    IMUCalibration,
    IMUConfig,
    MahonyIMU,
    RawSample,
    quat_mul,
    quat_conj,
    quat_norm,
    quat_inv,
    quat_to_euler,
    rotate_vec_by_quat
)

# Import synchronized acquisition (IMU + EMG)
from .synchronized_acquisition import (
    SynchronizedAcquisition,
    SyncConfig,
    IMUSyncConfig,
    EMGSyncConfig,
    EMGSelection,
    IMUSample,
    EMGChunk,
    TimestampedBuffer,
)

# Legacy alias: DualIMU is the same as IMUDevice
DualIMU = IMUDevice

__all__ = [
    # EMG Acquisition
    'EMGDevice', 
    'acquire_emg_data',
    'EMGChannelSelector',
    'fix_channel_name',
    
    # EMG Processing
    'BandpassFilter',
    'NotchFilter',
    'EMGEnvelopeExtractor',
    'EMGProcessor',
    'calculate_signal_quality',
    
    # EMG Visualization
    'EMGPlotter',
    'MultiChannelPlotter',
    'quick_plot_emg',
    
    # IMU
    'IMUDevice',
    'DualIMU',  # Alias for compatibility
    'acquire_imu_data',
    'IMUReading',
    'IMUCalibration',
    'IMUConfig',
    'MahonyIMU',
    'RawSample',
    # Quaternion math
    'quat_mul',
    'quat_conj',
    'quat_norm',
    'quat_inv',
    'quat_to_euler',
    'rotate_vec_by_quat'
    ,
    # Synchronized acquisition
    'SynchronizedAcquisition',
    'SyncConfig',
    'IMUSyncConfig',
    'EMGSyncConfig',
    'EMGSelection',
    'IMUSample',
    'EMGChunk',
    'TimestampedBuffer',
]
