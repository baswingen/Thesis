import sys
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path("c:/Users/Bas/Documents/Thesis")
sys.path.append(str(PROJECT_ROOT))

from inference.signal_buffer import _name_to_raw_idx
from inference.inference_config import EMG_CHANNEL_MAPPING

def test_indexing():
    print("Testing _name_to_raw_idx...")
    # New logic: num - 1
    tests = {
        "ch1": 0,
        "ch16": 15,
        "ch17": 16,
        "ch20": 19,
    }
    
    for ch, expected in tests.items():
        actual = _name_to_raw_idx(ch)
        print(f"  {ch} -> {actual} (Expected: {expected})")
        assert actual == expected, f"FAILED for {ch}"
    print("Indexing logic OK.\n")

def test_mapping_consistency():
    print("Checking EMG_CHANNEL_MAPPING consistency...")
    for muscle, mapping in EMG_CHANNEL_MAPPING.items():
        if isinstance(mapping, tuple):
            idx1 = _name_to_raw_idx(mapping[0])
            idx2 = _name_to_raw_idx(mapping[1])
            print(f"  {muscle:25}: ({mapping[0]}={idx1}, {mapping[1]}={idx2})")
        else:
            idx = _name_to_raw_idx(mapping)
            print(f"  {muscle:25}: {mapping:10} = {idx}")
    print("Mapping check complete.\n")

if __name__ == "__main__":
    try:
        test_indexing()
        test_mapping_consistency()
        print("Verification SUCCESSFUL.")
    except Exception as e:
        print(f"Verification FAILED: {e}")
        sys.exit(1)
