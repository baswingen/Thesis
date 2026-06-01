import sys
from pathlib import Path

search_dirs = [
    Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis"),
    Path("/Users/baswingen/Downloads"),
    Path("/Users/baswingen/Desktop"),
    Path("/Users/baswingen/Documents")
]

print("Searching for H5 files...")
found = False
for d in search_dirs:
    if d.exists():
        print(f"Searching in {d}...")
        h5_files = list(d.glob("**/*.h5"))
        if h5_files:
            print(f"Found {len(h5_files)} H5 files under {d}:")
            for f in h5_files[:10]:
                print(" -", f)
            found = True
            break

if not found:
    print("No H5 files found in search directories.")
