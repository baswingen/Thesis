import sys
from pathlib import Path

project_root = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis")
sys.path.append(str(project_root))

import model.config_model as cfg
print("DATABASE_ROOT:", cfg.DATABASE_ROOT)
print("DATABASE_ROOT exists:", cfg.DATABASE_ROOT.exists())
if cfg.DATABASE_ROOT.exists():
    print("Files in DATABASE_ROOT:")
    for p in list(cfg.DATABASE_ROOT.glob("*"))[:10]:
        print(" -", p)
    segments_dir = cfg.DATABASE_CONFIG['segments_dir']
    print("segments_dir exists:", segments_dir.exists())
    if segments_dir.exists():
        h5_files = list(segments_dir.glob("*.h5"))
        print(f"Total H5 files: {len(h5_files)}")
        for f in h5_files[:5]:
            print(" -", f.name)
