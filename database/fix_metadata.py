import h5py
import json
import os
from pathlib import Path

def update_metadata(file_path, participant_id=None, trial_num=None):
    print(f"[INFO] Updating metadata for: {file_path}")
    if not os.path.exists(file_path):
        print(f"[ERR] File not found: {file_path}")
        return False
    
    try:
        with h5py.File(file_path, "r+") as f:
            attrs = f.attrs
            
            # Update explicit attributes
            if participant_id:
                if "ID" in attrs: attrs["ID"] = participant_id
                if "participant_id" in attrs: attrs["participant_id"] = participant_id
                print(f"  - Updated ID/participant_id to: {participant_id}")
            
            if trial_num is not None:
                trial_str = f"{trial_num:02d}"
                if "Trial" in attrs: attrs["Trial"] = trial_str
                if "trial_num" in attrs: attrs["trial_num"] = trial_num
                print(f"  - Updated Trial/trial_num to: {trial_str}/{trial_num}")
                
            # Update config_participant JSON
            if "config_participant" in attrs:
                config = json.loads(attrs["config_participant"])
                if participant_id:
                    config["ID"] = participant_id
                if trial_num is not None:
                    config["Trial"] = f"{trial_num:02d}"
                attrs["config_participant"] = json.dumps(config)
                print(f"  - Updated config_participant JSON")
                
        return True
    except Exception as e:
        print(f"[ERR] Failed to update {file_path}: {e}")
        return False

def main():
    # Case 1: P08 Trial Correction
    p08_dir = Path("database/participant_P08/session_01")
    old_p08_file = p08_dir / "trial_1_20260408_132419.h5"
    new_p08_file = p08_dir / "trial_2_20260408_132419.h5"
    
    if old_p08_file.exists():
        print(f"[INFO] Renaming {old_p08_file} to {new_p08_file}")
        os.rename(old_p08_file, new_p08_file)
        update_metadata(new_p08_file, trial_num=2)
    elif new_p08_file.exists():
        print(f"[INFO] P08 file already renamed. Updating metadata...")
        update_metadata(new_p08_file, trial_num=2)
    else:
        print(f"[WARN] Could not find P08 trial file to fix.")

    # Case 2: P09 Participant Correction
    p09_file = Path("database/participant_P09/session_01/trial_1_20260408_163009.h5")
    if p09_file.exists():
        update_metadata(p09_file, participant_id="P09")
    else:
        print(f"[WARN] Could not find P09 trial file to fix.")

if __name__ == "__main__":
    main()
