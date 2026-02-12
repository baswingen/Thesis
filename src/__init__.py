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

# Import main IMU acquisition interface (supports BMI160 and BNO085)
from .imu_acquisition import (
    IMUDevice, 
    acquire_imu_data,
    IMUReading,
    IMUHealth,
    IMUCalibration,
    IMUConfig,
    IMUType,
    MahonyIMU,
    RawSample,
    QuaternionSample,
    DualEulerSample,
    quat_mul,
    quat_conj,
    quat_norm,
    quat_inv,
    quat_to_euler,
    euler_zyx_deg_to_quat,
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

# STM32-EMG PRBS synchronization (10 Hz chip rate, Kalman + postprocessing)
from .stm32_emg_sync import (
    SyncDelayEstimator,
    SyncDelayResult,
    DelaySignal,
    compute_sync_delay_signal,
    align_emg_to_stm32,
)

# Native signal acquisition (STM32 + TMSi Porti7 EMG + PRBS sync) — primary module
from .stm32_reader import STM32Reader, SampleSTM32
from .signal_acquisition import (
    SignalAcquisition,
    SignalAcquisitionConfig,
    EMGData,
    SyncState,
    RecordedSession,
    HardwarePRBSSync,
    EMGAcquisitionThread,
    fix_channel_name as fix_emg_channel_name,
    run_stm32_only,
    run_emg_only,
    run_prbs_test,
    run_combined,
    main as signal_acquisition_main,
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
    'IMUHealth',
    'IMUCalibration',
    'IMUConfig',
    'IMUType',
    'MahonyIMU',
    'RawSample',
    'QuaternionSample',
    'DualEulerSample',
    # Quaternion math
    'quat_mul',
    'quat_conj',
    'quat_norm',
    'quat_inv',
    'quat_to_euler',
    'euler_zyx_deg_to_quat',
    'rotate_vec_by_quat',
    # Synchronized acquisition
    'SynchronizedAcquisition',
    'SyncConfig',
    'IMUSyncConfig',
    'EMGSyncConfig',
    'EMGSelection',
    'IMUSample',
    'EMGChunk',
    'TimestampedBuffer',
    # STM32-EMG sync
    'SyncDelayEstimator',
    'SyncDelayResult',
    'DelaySignal',
    'compute_sync_delay_signal',
    'align_emg_to_stm32',
    # Native signal acquisition (STM32 + EMG + PRBS)
    'SignalAcquisition',
    'SignalAcquisitionConfig',
    'STM32Reader',
    'SampleSTM32',
    'EMGData',
    'SyncState',
    'RecordedSession',
    'HardwarePRBSSync',
    'EMGAcquisitionThread',
    'fix_emg_channel_name',
    'run_stm32_only',
    'run_emg_only',
    'run_prbs_test',
    'run_combined',
    'signal_acquisition_main',
]
