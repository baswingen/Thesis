import os
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from model.model_archs.spatio_temporal_transformer3 import SpatioTemporalTransformerRegressor3
from model.data_loader import DataLoader
from model.config_model import USE_PRECOMPUTED_FEATURES
from visualization.plot_segment_signals import load_segment

h5_file = "/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Coding Exercise/participant_P01_session_01_segments.h5"
segment = "segment_0009"
imu_unit = 1

os.environ["THESIS_CV_STRATEGY"] = "kfold"
model_path = "model/model_results/final_run_par_spec/all/spatio_temporal_transformer3_model.joblib"

reg = SpatioTemporalTransformerRegressor3.load(model_path)
chan = list(reg.model.channel_names)

D = load_segment(h5_file, segment, imu_unit)
dur = float(D["t_emg"][-1])
print(f"Segment duration: {dur:.3f} s")
print(f"EMG signal length: {len(D['t_emg'])} samples, fs={len(D['t_emg'])/dur:.1f}Hz")
print(f"IMU signal length: {len(D['t_imu'])} samples, fs={len(D['t_imu'])/dur:.1f}Hz")

loader = DataLoader()
df = loader.load_and_extract_features([Path(h5_file)], is_sequence=True, use_precomputed=USE_PRECOMPUTED_FEATURES)
row = df[df["segment_id"] == segment]
X, _ = loader.prepare_for_ml(row, target_col="weight")

# Let's count windows in features
windows = row.iloc[0]["sequence_dicts"]
print(f"Number of windows in features (sequence_dicts): {len(windows)}")

# Run forward pass with hooks
layers = []
def hook(_m, _i, out):
    w = out[1]
    layers.append(w.detach().float().cpu().numpy())

handles = [m.register_forward_hook(hook)
           for n, m in reg.model.named_modules()
           if n.endswith("spatial_attn") and isinstance(m, torch.nn.MultiheadAttention)]

import torch
with torch.no_grad():
    X_tensor, _ = loader.prepare_for_ml(row, target_col="weight")
    pred = float(np.array(reg.predict(X_tensor)).ravel()[0])

for h in handles:
    h.remove()

spatial_TCC = np.mean(np.stack(layers, axis=0), axis=0)
print(f"Model spatial attention shape: {spatial_TCC.shape}")
T = spatial_TCC.shape[0]
print(f"Number of windows from forward pass T: {T}")

# Let's see how t_end aligns
emg_window_size_sec = 0.15
imu_window_size_sec = 0.20
window_step_sec = 0.10

t_end = 0.20
for t in range(T):
    print(f"Window {t:02d}: t_end={t_end:.2f} s | EMG window=[{max(0.0, t_end-emg_window_size_sec):.2f}, {t_end:.2f}] | IMU window=[{max(0.0, t_end-imu_window_size_sec):.2f}, {t_end:.2f}]")
    t_end += window_step_sec
