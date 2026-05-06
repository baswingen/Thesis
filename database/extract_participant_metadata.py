import h5py
import json
import os
import pandas as pd
from pathlib import Path
from tqdm import tqdm

def extract_metadata(base_dir):
    base_path = Path(base_dir)
    if not base_path.exists():
        print(f"[ERR] Directory not found: {base_dir}")
        return None

    all_metadata = []
    processed_participants = set()

    # Find all .h5 files recursively
    # We use a pattern to avoid AppleDouble files (._*.h5) which cause errors
    h5_files = list(base_path.glob("**/trial_*.h5"))
    h5_files = [f for f in h5_files if not f.name.startswith("._")]
    
    print(f"[INFO] Found {len(h5_files)} HDF5 files. Extracting metadata...")

    for file_path in tqdm(h5_files, desc="Processing files"):
        try:
            with h5py.File(file_path, "r") as f:
                if "config_participant" in f.attrs:
                    config = json.loads(f.attrs["config_participant"])
                    p_id = config.get("ID")
                    
                    # We only need one entry per participant since metadata should be constant across trials
                    if p_id and p_id not in processed_participants:
                        # Flatten or clean up if needed, but the current structure seems flat enough
                        print(f"  + Found and added participant: {p_id}")
                        all_metadata.append(config)
                        processed_participants.add(p_id)
        except Exception as e:
            print(f"[WARN] Failed to read {file_path}: {e}")

    if not all_metadata:
        print("[WARN] No participant metadata found.")
        return None

    # Convert to DataFrame and sort by ID
    df = pd.DataFrame(all_metadata)
    
    # Drop Trial and Session as requested
    cols_to_drop = ["Trial", "Session", "trial_num", "session_num"]
    df = df.drop(columns=[c for c in cols_to_drop if c in df.columns])

    # Rename columns to shorter versions as requested by user's pattern
    rename_map = {
        "Total Arm Length [cm]": "TotalArm",
        "Upper Arm Length [cm]": "UpperArm",
        "Forearm Length [cm]": "Forearm",
        "Hand Length [cm]": "Hand",
        "Upper Arm Circumference [cm]": "UpperArmCirc",
        "Fore Arm Circumference [cm]": "ForearmCirc"
    }
    df = df.rename(columns=rename_map)

    if "ID" in df.columns:
        df = df.sort_values("ID")
    
    return df

def main():
    base_dir = "/Volumes/Laurens SSD/BasData"
    output_dir = Path("database/metadata_exports")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"[INFO] Starting metadata extraction from {base_dir}")
    df = extract_metadata(base_dir)
    
    if df is not None:
        csv_path = output_dir / "participant_metadata.csv"
        json_path = output_dir / "participant_metadata.json"
        
        df.to_csv(csv_path, index=False)
        df.to_json(json_path, orient="records", indent=4)
        
        print(f"[SUCCESS] Metadata extracted for {len(df)} participants.")
        print(f"  - CSV saved to: {csv_path}")
        print(f"  - JSON saved to: {json_path}")
        
        # Also print a summary
        print("\n--- Summary ---")
        print(df.to_string(index=False))
    else:
        print("[ERR] Extraction failed or no data found.")

if __name__ == "__main__":
    main()
