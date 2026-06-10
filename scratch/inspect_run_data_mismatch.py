import json
from pathlib import Path

p1 = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/ST-transformer-par-spec-cross-val/run_data.json")
p2 = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/ST-transformer-par-spec-cross-val/all/run_data.json")

print("p1 exists:", p1.exists())
print("p2 exists:", p2.exists())

if p1.exists():
    with open(p1, "r") as f:
        d1 = json.load(f)
    print("p1 keys:", list(d1.keys()))
    if 'all' in d1:
        print("p1 'all' model_info:", d1['all'].get('model_info'))

if p2.exists():
    with open(p2, "r") as f:
        d2 = json.load(f)
    print("p2 keys:", list(d2.keys()))
    print("p2 model_info:", d2.get('model_info'))
