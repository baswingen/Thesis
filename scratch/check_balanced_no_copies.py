import h5py
import numpy as np
from pathlib import Path
from collections import defaultdict
import sys

# Add model to path to load config
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from model.config_model import PARTICIPANT_CONFIG, TRUE_WEIGHTS

def main():
    data_dir = Path("/Volumes/Laurens SSD/BasData/segments")
    h5_paths = sorted(list(data_dir.glob("participant_P*_session_*_segments.h5")))
    
    include_participants = set(PARTICIPANT_CONFIG.get('include', []))
    print(f"Included participants ({len(include_participants)}): {include_participants}")
    
    # Filter files
    filtered_paths = []
    for p in h5_paths:
        parts = p.stem.split("_")
        if "participant" in parts:
            idx = parts.index("participant") + 1
            if idx < len(parts) and parts[idx] in include_participants:
                filtered_paths.append(p)
                
    print(f"Found {len(h5_paths)} files, filtered to {len(filtered_paths)}")
    
    # Import blacklist and weight filter from model.data_loader
    from model.data_loader import SEGMENT_BLACKLIST, WEIGHT_INCLUDE
    
    _EXCLUDED_TRUE_WEIGHTS = set()
    for nom_w, enabled in WEIGHT_INCLUDE.items():
        if not enabled:
            _EXCLUDED_TRUE_WEIGHTS.add(TRUE_WEIGHTS.get(nom_w, nom_w))
    print(f"Excluded true weights: {_EXCLUDED_TRUE_WEIGHTS}")
    
    # Read segment weights per participant
    group_counts = defaultdict(int)
    total_original = 0
    
    for path in filtered_paths:
        subject = path.stem.split("_")[1]
        with h5py.File(path, "r") as f:
            for key in f.keys():
                if not key.startswith("segment_"):
                    continue
                # Check blacklist
                if (path.name, key) in SEGMENT_BLACKLIST:
                    continue
                    
                grp = f[key]
                
                label_attr = grp.attrs.get("label", "")
                label = label_attr.decode() if isinstance(label_attr, bytes) else str(label_attr)
                
                if label == "free_movement":
                    weight = 0.0
                else:
                    nom_w_attr = grp.attrs.get("weight", -1.0)
                    nom_w = float(nom_w_attr)
                    weight = TRUE_WEIGHTS.get(nom_w, nom_w)
                
                # Check weight exclusion filter
                if weight in _EXCLUDED_TRUE_WEIGHTS:
                    continue
                
                # Map true weights to float keys rounded to 2 decimals for grouping consistency
                weight = round(float(weight), 2)
                group_counts[(subject, weight)] += 1
                total_original += 1
                
    print(f"Total original segments counted (after filters): {total_original}")
    print(f"Number of participant-weight groups: {len(group_counts)}")
    
    # Compute the cap at 250
    target = 250
    total_capped = 0
    for g, count in group_counts.items():
        capped = min(count, target)
        total_capped += capped
        
    print(f"Total segments after applying cap of {target} (without oversampling copies): {total_capped}")

if __name__ == "__main__":
    main()
