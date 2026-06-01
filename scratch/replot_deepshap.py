import sys
from pathlib import Path
import numpy as np

# Add project root to path
project_root = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis")
sys.path.append(str(project_root))

# Enable all configs BEFORE importing data_loader or other modules
import model.config_model as cfg
cfg.PARTICIPANT_CONFIG['include'] = 'all'

# Enable all EMG/IMU channels
for ch in cfg.CHANNEL_CONFIG['emg_channels']:
    cfg.CHANNEL_CONFIG['emg_channels'][ch] = True
for ch in cfg.CHANNEL_CONFIG['imu_channels']:
    cfg.CHANNEL_CONFIG['imu_channels'][ch] = True

from model.deepshap_analysis import process_and_plot_shap

def main():
    run_dir = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/P1-P18_pre-dev/cnn-gru-lopo")
    npz_path = run_dir / "deepshap_values.npz"
    
    print(f"Loading SHAP values from {npz_path}...")
    data = np.load(npz_path, allow_pickle=True)
    shap_values = data['shap_values']
    channel_names = list(data['channel_names'])
    
    print(f"Re-processing and generating premium plots in {run_dir}...")
    process_and_plot_shap(shap_values, channel_names, run_dir)
    print("SHAP processing and plotting completed successfully!")

if __name__ == "__main__":
    main()
