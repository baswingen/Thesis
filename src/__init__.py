"""Thesis project package."""

__version__ = "0.1.0"

# Import main EMG acquisition interface
from .emg_acquisition import EMGDevice, acquire_emg_data

# Import main IMU acquisition interface
from .imu_acquisition import (
    IMUDevice, 
    acquire_imu_data,
    IMUReading,
    IMUCalibration,
    MadgwickIMU
)

__all__ = [
    # EMG
    'EMGDevice', 
    'acquire_emg_data',
    # IMU
    'IMUDevice',
    'acquire_imu_data',
    'IMUReading',
    'IMUCalibration',
    'MadgwickIMU'
]
