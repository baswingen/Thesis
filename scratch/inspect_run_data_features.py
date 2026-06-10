import json
from pathlib import Path

p = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/ST-transformer-par-spec-cross-val/run_data.json")

with open(p, "r") as f:
    data = json.load(f)

for mod in ['all']:
    m_data = data.get(mod, {})
    config = m_data.get("config", {})
    print("Modality:", mod)
    print("Channel config (emg):", list(config.get("channel_config", {}).get("emg_channels", {}).keys()))
    print("Feature config (emg):", [k for k, v in config.get("feature_config", {}).get("emg_features", {}).items() if v])
    print("Model hyperparameters:", config.get("model_hyperparameters"))
    print("Feature importance keys:", list(m_data.get("feature_importance", {}).keys()) if m_data.get("feature_importance") else None)
