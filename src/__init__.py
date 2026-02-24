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
    # STM32-EMG sync
    'SyncDelayEstimator',
    'SyncDelayResult',
    'DelaySignal',
    'compute_sync_delay_signal',
    'align_emg_to_stm32',
    # Native signal acquisition
    'STM32Reader',
    'SampleSTM32',
]
