import json
from pathlib import Path

run_dir = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/P1-P18_pre-dev/cnn-gru-lopo")
json_path = run_dir / "run_data.json"

with open(json_path, "r") as f:
    data = json.load(f)

feat_imp = data.get("feature_importance", {})
print("deepshap_modality:")
print(json.dumps(feat_imp.get("deepshap_modality"), indent=2))

print("\ndeepshap_channel keys (first 10):")
channels = list(feat_imp.get("deepshap_channel", {}).keys())
for ch in channels[:10]:
    print(f" - {ch}: {feat_imp['deepshap_channel'][ch]:.6f}")

print("\nPermutation channel keys (first 10):")
perm_channels = list(feat_imp.get("permutation_channel", {}).keys())
for ch in perm_channels[:10]:
    print(f" - {ch}: {feat_imp['permutation_channel'][ch]:.6f}")
