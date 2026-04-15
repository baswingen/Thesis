import sys
from pathlib import Path
import matplotlib.pyplot as plt

# Add project root to sys.path so 'model' package can be found
sys.path.append(str(Path(__file__).parent.parent))

from model.data_loader import DataLoader
from model.config_model import DATABASE_CONFIG, CNN_LSTM_CONFIG
from model.model_archs.cnn_lstm import CNNLSTMRegressor
from model.run_model import load_and_prepare_data, setup_run_dir

def main():
    print("Initializing Data Loader...")
    loader = DataLoader()
    h5_paths = list(DATABASE_CONFIG['segments_dir'].glob("*.h5"))
    
    if not h5_paths:
        print(f"No HDF5 segment files found in {DATABASE_CONFIG['segments_dir']}.")
        return

    # We only care about running the LR finder, so we can use a subset or the whole thing
    print("Loading raw segment data for LR Finder...")
    
    # Using the same data logic from run_model
    X, y, groups, df = load_and_prepare_data(
        loader=loader, 
        h5_paths=h5_paths, 
        model_type="cnn_lstm", 
        is_raw_segment=True, 
        is_sequence=False, 
        use_precomputed=True
    )
    
    if X is None:
        print("Data extraction failed.")
        return

    print(f"Loaded {len(X)} samples.")
    
    # Initialize the architecture (CNN-LSTM as blueprint)
    model = CNNLSTMRegressor(**CNN_LSTM_CONFIG)
    
    # Start LR range test
    # We will aim for ~100-200 iterations. It takes very little time.
    base_dir = Path(__file__).parent.parent
    run_dir, timestamp = setup_run_dir(base_dir)
    save_path = run_dir / "lr_finder_plot.png"
    
    # We pass the full dataset. The finder will automatically extract a DataLoader 
    # and stop after `num_iter` batches.
    print("\n--- Starting LR Finder ---")
    model.lr_find(X, y, end_lr=10.0, num_iter=150, save_path=save_path)
    
    print("\nDone! Look at the generated plot to pick your ideal max_lr.")
    print("It should be the value at the steepest downward slope before the loss starts increasing.")

if __name__ == "__main__":
    main()
