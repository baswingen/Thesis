import json
from pathlib import Path

p = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/ST-transformer-par-spec-cross-val/run_data.json")

with open(p, "r") as f:
    data = json.load(f)

for mod in ['all', 'emg_only', 'imu_only']:
    print(f"\n=================== {mod.upper()} ===================")
    m_data = data.get(mod, {})
    print("model_info:", m_data.get("model_info"))
    print("timing:", m_data.get("timing"))
    config = m_data.get("config", {})
    print("ablation_modality:", config.get("ablation_modality"))
    print("model_hyperparameters:", config.get("model_hyperparameters"))
