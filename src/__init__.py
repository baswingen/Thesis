"""Thesis project package."""

__version__ = "0.1.0"

# Import main EMG acquisition interface
from .emg_acquisition import EMGDevice, acquire_emg_data

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

# Legacy alias: DualIMU is the same as IMUDevice
DualIMU = IMUDevice

__all__ = [
    # EMG
    'EMGDevice', 
    'acquire_emg_data',
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
]
