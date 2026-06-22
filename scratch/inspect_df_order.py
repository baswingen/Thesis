import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent.parent))

from model.data_loader import DataLoader
from model.config_model import DATABASE_CONFIG

def main():
    loader = DataLoader()
    h5_paths = [p for p in DATABASE_CONFIG['segments_dir'].glob("*.h5") if not p.name.startswith("._")]
    
    print(f"Loaded h5_paths length: {len(h5_paths)}")
    print("First 5 paths:")
    for p in h5_paths[:5]:
        print(f" - {p.name}")
        
    df = loader.load_raw_segments(h5_paths)
    print(f"df shape: {df.shape}")
    print("df columns:", df.columns)
    
    # Check subjects
    subjects = df["subject"].values
    transitions = []
    prev = None
    for i, s in enumerate(subjects):
        if s != prev:
            transitions.append((i, s))
            prev = s
    print("Transitions in df['subject']:", transitions)

if __name__ == "__main__":
    main()
