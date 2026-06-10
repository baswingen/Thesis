import torch
import sys
from pathlib import Path

# Add project root to sys.path
sys.path.append(str(Path(__file__).resolve().parent.parent))

from model.model_archs.spatio_temporal_transformer3 import SpatioTemporalTransformerRegressor3
from model.config_model import SPATIO_TEMPORAL_TRANSFORMER3_CONFIG, CHANNEL_CONFIG, FEATURE_CONFIG

# Let's inspect the local config
print("Local config:")
print("d_model:", SPATIO_TEMPORAL_TRANSFORMER3_CONFIG.get('d_model'))
print("nhead_spatial:", SPATIO_TEMPORAL_TRANSFORMER3_CONFIG.get('nhead_spatial'))
print("nhead_temporal:", SPATIO_TEMPORAL_TRANSFORMER3_CONFIG.get('nhead_temporal'))
print("num_layers:", SPATIO_TEMPORAL_TRANSFORMER3_CONFIG.get('num_layers'))
print("dim_feedforward:", SPATIO_TEMPORAL_TRANSFORMER3_CONFIG.get('dim_feedforward'))

# Let's check how many EMG and IMU channels are enabled
emg_ch = [k for k, v in CHANNEL_CONFIG.get('emg_channels', {}).items() if v]
imu_ch = [k for k, v in CHANNEL_CONFIG.get('imu_channels', {}).items() if v]
print(f"Enabled EMG: {len(emg_ch)}, Enabled IMU: {len(imu_ch)}")

# Let's check how many features per channel are extracted in K-Fold configuration
# We can simulate the feature names list
# For EMG, let's see which features are enabled under kfold
emg_features = [k for k, v in FEATURE_CONFIG.get('emg_features', {}).items() if v or (k in ['MAV', 'VAR', 'IEMG', 'Skew', 'Kurt', 'HjComp', 'MDF', 'SpecEntropy', 'PeakFreq'])]
# Under kfold, all emg_features are enabled (20 features)
print("EMG features count (KFold):", len(emg_features))

# For IMU, let's check features under kfold
imu_features = [k for k, v in FEATURE_CONFIG.get('imu_features', {}).items() if v or (k in ['RMS', 'SpecEnergy', 'SVM_Mean'])]
print("IMU features count (KFold):", len(imu_features))
