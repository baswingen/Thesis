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

# Import main IMU acquisition interface
from .imu_acquisition import (
    IMUDevice, 
    acquire_imu_data,
    IMUReading,
    IMUCalibration,
    MadgwickIMU
)

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
    'acquire_imu_data',
    'IMUReading',
    'IMUCalibration',
    'MadgwickIMU'
]
