import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import time
import numpy as np
from model.feature_extraction import FeatureExtractor

# Benchmark settings
num_runs = 1000
emg_fs = 2000.0
imu_fs = 500.0  # typical for IMU in this codebase

# Window sizes
emg_win_size = int(0.15 * emg_fs)  # 300 samples
imu_win_size = int(0.20 * imu_fs)  # 100 samples

# Channel names and count
emg_cols = [f"ch{i}_EMG" for i in range(1, 9)]  # 8 muscles
imu_cols = [
    'ax1', 'ay1', 'az1', 'roll_rad1', 'pitch_rad1', 'yaw_rad1',
    'ax2', 'ay2', 'az2', 'roll_rad2', 'pitch_rad2', 'yaw_rad2'
]  # 12 channels

# Setup FeatureExtractor with active features in K-Fold config
# EMG: 20 features, IMU: 11 features
emg_features_config = {
    'MAV': True, 'RMS': True, 'WL': True, 'ZC': True, 'SSC': True,
    'VAR': True, 'WAMP': True, 'IEMG': True, 'LogDet': True, 'Skew': True,
    'Kurt': True, 'HjMob': True, 'HjComp': True, 'Myopulse': True,
    'MNF': True, 'MDF': True, 'Power': True, 'SpecEntropy': True, 'PeakFreq': True, 'BW': True
}

imu_features_config = {
    'Mean': True, 'Var': True, 'Std': True, 'Max': True, 'Min': False,
    'RMS': False, 'SMA': True, 'P2P': True, 'IQR': False, 'Skew': True,
    'Kurt': True, 'Jerk': True, 'ZC': False, 'Energy': False,
    'DomFreq': True, 'SpecEnergy': True, 'MNF': False, 'MDF': False, 'SpecEntropy': False,
    'SVM_Mean': False, 'SVM_Std': False
}

extractor = FeatureExtractor(
    emg_fs=emg_fs,
    imu_fs=imu_fs,
    emg_threshold=1e-5,
    emg_features=emg_features_config,
    imu_features=imu_features_config
)

# Generate dummy data
dummy_emg = np.random.normal(0, 1, (emg_win_size, len(emg_cols)))
dummy_imu = np.random.normal(0, 1, (imu_win_size, len(imu_cols)))

# Warmup
for _ in range(50):
    _ = extractor.extract_features(dummy_emg, emg_cols, dummy_imu, imu_cols)

# Benchmark EMG features only
t0 = time.perf_counter()
for _ in range(num_runs):
    _ = extractor.extract_emg_features(dummy_emg, emg_cols)
t1 = time.perf_counter()
emg_time_ms = ((t1 - t0) / num_runs) * 1000

# Benchmark IMU features only
t0 = time.perf_counter()
for _ in range(num_runs):
    _ = extractor.extract_imu_features(dummy_imu, imu_cols)
t1 = time.perf_counter()
imu_time_ms = ((t1 - t0) / num_runs) * 1000

# Benchmark total extraction
t0 = time.perf_counter()
for _ in range(num_runs):
    _ = extractor.extract_features(dummy_emg, emg_cols, dummy_imu, imu_cols)
t1 = time.perf_counter()
total_time_ms = ((t1 - t0) / num_runs) * 1000

print(f"EMG Feature Extraction Latency (8 channels): {emg_time_ms:.4f} ms")
print(f"IMU Feature Extraction Latency (12 channels): {imu_time_ms:.4f} ms")
print(f"Total Feature Extraction Latency (20 channels): {total_time_ms:.4f} ms")
