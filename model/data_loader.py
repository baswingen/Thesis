import h5py
import numpy as np
import pandas as pd
from pathlib import Path
from typing import Tuple, List, Optional
import sys
import os

from model.feature_extraction import FeatureExtractor
from model.config_model import FEATURE_CONFIG

class DataLoader:
    """
    DataLoader handles taking segmented HDF5 files, iterating through their segments,
    extracting features using FeatureExtractor, and preparing the standard X & y arrays
    for traditional Machine Learning classification algorithms.
    """
    def __init__(self, emg_fs: float = 2000, 
                 imu_fs: float = 2000):
        self.extractor = FeatureExtractor(
            emg_fs=emg_fs, 
            imu_fs=imu_fs,
            emg_threshold=FEATURE_CONFIG.get('emg_threshold', 1e-5),
            emg_features=FEATURE_CONFIG.get('emg_features'),
            imu_features=FEATURE_CONFIG.get('imu_features')
        )

    def _decode_cols(self, raw) -> List[str]:
        return [c.decode() if isinstance(c, bytes) else str(c) for c in raw]

    def load_and_extract_features(self, h5_paths: List[Path],
                                  is_sequence: bool = True,
                                  emg_window_size_sec: Optional[float] = None,
                                  imu_window_size_sec: Optional[float] = None,
                                  window_step_sec: Optional[float] = None) -> pd.DataFrame:
        """
        Iterates over a list of segmented HDF5 files, extracts temporal and spectral 
        features from the EMG and IMU segments within it, and returns a single DataFrame 
        containing all features and labels. If window parameters are provided, extracts
        sequences of features over sliding windows.
        
        Defaults to FEATURE_CONFIG for window properties if not provided.
        """
        # Fallback to FEATURE_CONFIG if not provided as arguments
        if emg_window_size_sec is None:
            emg_window_size_sec = FEATURE_CONFIG.get('emg_window_size_sec')
        if imu_window_size_sec is None:
            imu_window_size_sec = FEATURE_CONFIG.get('imu_window_size_sec')
        if window_step_sec is None:
            window_step_sec = FEATURE_CONFIG.get('window_step_sec')
            
        all_features = []
        
        for path in h5_paths:
            path = Path(path)
            if not path.exists():
                print(f"[WARN] File not found: {path}")
                continue
                
            with h5py.File(path, "r") as f:
                # Find all segment groups
                segment_keys = sorted(k for k in f.keys() if k.startswith("segment_"))
                
                for key in segment_keys:
                    grp = f[key]
                    
                    # Extract Data
                    imu = grp["imu"][:] if "imu" in grp else np.empty((0, 0))
                    emg = grp["emg"][:] if "emg" in grp else np.empty((0, 0))

                    imu_cols = self._decode_cols(grp["imu"].attrs.get("column_names", [])) if "imu" in grp else []
                    emg_cols = self._decode_cols(grp["emg"].attrs.get("column_names", [])) if "emg" in grp else []

                    # Read dynamic sampling rates if available (stored by postprocessing.py)
                    # We look at the attributes of the datasets inside the segment
                    fs_emg_eff = grp["emg"].attrs.get("fs", self.extractor.emg_fs) if "emg" in grp else self.extractor.emg_fs
                    fs_imu_eff = grp["imu"].attrs.get("fs", self.extractor.imu_fs) if "imu" in grp else self.extractor.imu_fs
                    
                    # Update extractor temporarily for this segment if rates differ
                    orig_emg_fs = self.extractor.emg_fs
                    orig_imu_fs = self.extractor.imu_fs
                    self.extractor.emg_fs = fs_emg_eff
                    self.extractor.imu_fs = fs_imu_eff

                    # Run Feature Extraction
                    if is_sequence and emg_window_size_sec is not None and imu_window_size_sec is not None and window_step_sec is not None:
                        windows = self.extractor.extract_features_windowed(
                            emg_data=emg, emg_cols=emg_cols,
                            imu_data=imu, imu_cols=imu_cols,
                            emg_window_size_sec=emg_window_size_sec,
                            imu_window_size_sec=imu_window_size_sec,
                            window_step_sec=window_step_sec
                        )
                        if not windows:
                            # Reset extractor before skipping
                            self.extractor.emg_fs = orig_emg_fs
                            self.extractor.imu_fs = orig_imu_fs
                            continue 
                        features = {"sequence_dicts": windows}
                    else:
                        features = self.extractor.extract_features(
                            emg_data=emg, emg_cols=emg_cols,
                            imu_data=imu, imu_cols=imu_cols
                        )
                    
                    # Reset extractor
                    self.extractor.emg_fs = orig_emg_fs
                    self.extractor.imu_fs = orig_imu_fs
                    
                    # Add Metadata Attributes to feature row
                    def _a(name, default=""):
                        v = grp.attrs.get(name, default)
                        return v.decode() if isinstance(v, bytes) else v
                        
                    # Target labels / stratification parameters
                    label = str(_a("label", "unknown"))
                    features["label"] = label
                    features["state"] = str(_a("state", "unknown"))
                    
                    # For regression: map 'free_movement' to 0.0kg. 
                    # Others use the weight recorded in h5 attributes.
                    if label == "free_movement":
                        features["weight"] = 0.0
                    else:
                        features["weight"] = float(grp.attrs.get("weight", -1.0))
                    
                    # Context elements (if we want to use them for nested cross-validation later)
                    features["segment_id"] = key
                    features["trial_file"] = _a("trial_file", "unknown")
                    features["subject"] = path.stem.split("_")[1] if "participant" in path.stem else "unknown"

                    all_features.append(features)

        if not all_features:
            print("[WARN] No features were extracted. Check if valid HDF5 file(s) were provided.")
            return pd.DataFrame()

        df = pd.DataFrame(all_features)
        
        # Optionally, handle NaNs that may have appeared due to missing features
        # (e.g. if a segment randomly lacked IMU data) by filling with 0 or dropping.
        if "sequence_dicts" not in df.columns:
            df.fillna(0, inplace=True) 
        
        return df

    def prepare_for_ml(self, df: pd.DataFrame, target_col: str = "label", 
                       drop_cols: Optional[List[str]] = None) -> Tuple[pd.DataFrame, pd.Series]:
        """
        Splits the DataFrame prepared by `load_and_extract_features` into a 
        feature matrix X and label series y, ready for scikit-learn or similar APIs.
        """
        if df.empty:
            return pd.DataFrame(), pd.Series(dtype=float)
            
        if target_col not in df.columns:
            raise ValueError(f"Target column '{target_col}' not found in dataframe.")
            
        y = df[target_col]
        
        # Columns that are purely metadata / labels and shouldn't be trained on
        default_drop = ["label", "state", "weight", "segment_id", "trial_file", "subject"]
        
        if drop_cols:
            default_drop.extend(drop_cols)
            
        drop_cols_present = [c for c in default_drop if c in df.columns]
        
        X = df.drop(columns=drop_cols_present)
        
        return X, y

if __name__ == "__main__":
    # Example usage / basic test
    sample_file = Path(__file__).parent.parent / "database" / "segments" / "participant_P01_session_01_segments.h5"
    
    if sample_file.exists():
        print(f"Loading from: {sample_file}")
        loader = DataLoader()
        
        # Extract features
        df = loader.load_and_extract_features([sample_file])
        
        print(f"Extracted features shape: {df.shape}")
        
        if not df.empty:
            # Prepare ML structures
            X, y = loader.prepare_for_ml(df, target_col="label")
            print(f"X shape: {X.shape}, y shape: {y.shape}")
            print("\nClasses present:", y.unique())
            
            print("\nSample features subset:")
            print(X.iloc[0, :5])  # Display 5 features of row 0
    else:
        print(f"Could not perform test, sample file {sample_file} not found.")

