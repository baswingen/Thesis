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

    Supports two modes:
    - **Precomputed** (default): reads features previously written by
      ``feature_extraction.py`` from a ``"features"`` group inside each
      segment.  Fast – no signal processing at runtime.
    - **Live extraction**: runs FeatureExtractor on the raw IMU/EMG arrays
      inside each segment.  Used as an automatic fallback if no precomputed
      features are found, and by ``feature_extraction.py`` itself.
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

    # ------------------------------------------------------------------
    # Precomputed feature loading
    # ------------------------------------------------------------------

    def load_precomputed_features(self, h5_paths: List[Path]) -> pd.DataFrame:
        """
        Load features that were pre-extracted and stored inside the segmented
        HDF5 files by ``feature_extraction.py``.

        Returns a DataFrame in exactly the same format as
        ``load_and_extract_features``, or an empty DataFrame if no
        precomputed features are found in any of the files.
        """
        from tqdm import tqdm
        from model.segmentation import load_precomputed_features as _load_feats

        all_features = []

        for path in h5_paths:
            path = Path(path)
            if not path.exists():
                print(f"[WARN] File not found: {path}")
                continue

            seg_feats = _load_feats(path)
            if not seg_feats:
                return pd.DataFrame()  # caller will fall back to live extraction

            with h5py.File(path, "r") as f:
                for seg_key in tqdm(
                    sorted(seg_feats.keys()),
                    desc=f"Loading precomputed ({path.name})",
                    unit="segment",
                ):
                    feat_dict = seg_feats[seg_key]
                    grp = f[seg_key]

                    def _a(name, default=""):
                        v = grp.attrs.get(name, default)
                        return v.decode() if isinstance(v, bytes) else v

                    label = str(_a("label", "unknown"))
                    feat_dict["label"] = label
                    feat_dict["state"] = str(_a("state", "unknown"))
                    feat_dict["weight"] = (
                        0.0 if label == "free_movement"
                        else float(grp.attrs.get("weight", -1.0))
                    )
                    feat_dict["segment_id"] = seg_key
                    feat_dict["trial_file"] = _a("trial_file", "unknown")
                    feat_dict["subject"] = (
                        path.stem.split("_")[1]
                        if "participant" in path.stem
                        else "unknown"
                    )

                    # Sampling-rate metadata (stored as attrs on the raw datasets)
                    if "emg" in grp:
                        feat_dict["emg_fs"] = grp["emg"].attrs.get("fs", 2000.0)
                        feat_dict["emg_fs_orig"] = grp["emg"].attrs.get("fs_orig", feat_dict["emg_fs"])
                    if "imu" in grp:
                        feat_dict["imu_fs"] = grp["imu"].attrs.get("fs", 2000.0)
                        feat_dict["imu_fs_orig"] = grp["imu"].attrs.get("fs_orig", feat_dict["imu_fs"])

                    all_features.append(feat_dict)

        if not all_features:
            return pd.DataFrame()

        df = pd.DataFrame(all_features)
        if "sequence_dicts" not in df.columns:
            df.fillna(0, inplace=True)
        return df

    # ------------------------------------------------------------------
    # Raw segment loading (for CNN-LSTM end-to-end models)
    # ------------------------------------------------------------------

    def load_raw_segments(self, h5_paths: List[Path]) -> pd.DataFrame:
        """
        Load raw EMG + IMU arrays directly from segmented HDF5 files.

        Each segment produces one row with a ``raw_segment`` column containing
        a numpy array of shape ``(N_samples, n_emg + n_imu)``.  Channel
        filtering from ``FEATURE_CONFIG`` is applied.

        This path is used exclusively by **CNN-LSTM**, which learns its own
        features via 1-D convolutions instead of relying on hand-crafted
        windowed features.
        """
        from tqdm import tqdm

        all_rows: list[dict] = []
        channel_names: list[str] | None = None

        for path in h5_paths:
            path = Path(path)
            if not path.exists():
                print(f"[WARN] File not found: {path}")
                continue

            with h5py.File(path, "r") as f:
                segment_keys = sorted(k for k in f.keys() if k.startswith("segment_"))

                for key in tqdm(segment_keys, desc=f"Loading raw ({path.name})", unit="seg"):
                    grp = f[key]

                    # --- EMG ---
                    emg = grp["emg"][:] if "emg" in grp else np.empty((0, 0))
                    emg_cols = self._decode_cols(grp["emg"].attrs.get("column_names", [])) if "emg" in grp else []

                    emg_channels_config = FEATURE_CONFIG.get('emg_channels', {})
                    if emg_cols and emg_channels_config:
                        valid_idx = [i for i, c in enumerate(emg_cols) if emg_channels_config.get(c, True)]
                        emg_cols = [emg_cols[i] for i in valid_idx]
                        if emg.size > 0:
                            emg = emg[:, valid_idx]

                    # --- IMU ---
                    imu = grp["imu"][:] if "imu" in grp else np.empty((0, 0))
                    imu_cols = self._decode_cols(grp["imu"].attrs.get("column_names", [])) if "imu" in grp else []

                    imu_channels_config = FEATURE_CONFIG.get('imu_channels', {})
                    if imu_cols and imu_channels_config:
                        valid_idx = [i for i, c in enumerate(imu_cols) if imu_channels_config.get(c, True)]
                        imu_cols = [imu_cols[i] for i in valid_idx]
                        if imu.size > 0:
                            imu = imu[:, valid_idx]

                    # Concatenate EMG + IMU along the channel axis
                    if emg.size > 0 and imu.size > 0:
                        # Align row counts (they share the same sample clock)
                        n = min(emg.shape[0], imu.shape[0])
                        raw = np.hstack([emg[:n], imu[:n]]).astype(np.float32)
                        cols = emg_cols + imu_cols
                    elif emg.size > 0:
                        raw = emg.astype(np.float32)
                        cols = emg_cols
                    elif imu.size > 0:
                        raw = imu.astype(np.float32)
                        cols = imu_cols
                    else:
                        continue  # skip empty segments

                    if channel_names is None:
                        channel_names = cols

                    # --- Metadata ---
                    def _a(name, default=""):
                        v = grp.attrs.get(name, default)
                        return v.decode() if isinstance(v, bytes) else v

                    label = str(_a("label", "unknown"))
                    row = {
                        "raw_segment": raw,
                        "label": label,
                        "state": str(_a("state", "unknown")),
                        "weight": 0.0 if label == "free_movement" else float(grp.attrs.get("weight", -1.0)),
                        "segment_id": key,
                        "trial_file": _a("trial_file", "unknown"),
                        "subject": path.stem.split("_")[1] if "participant" in path.stem else "unknown",
                    }
                    all_rows.append(row)

        if not all_rows:
            print("[WARN] No raw segments loaded.")
            return pd.DataFrame()

        df = pd.DataFrame(all_rows)
        # Store the channel order for reference
        df.attrs["channel_names"] = channel_names
        print(f"[DataLoader] Loaded {len(df)} raw segments "
              f"({channel_names and len(channel_names) or '?'} channels each)")
        return df

    # ------------------------------------------------------------------
    # Live feature extraction (also used as fallback)
    # ------------------------------------------------------------------

    def load_and_extract_features(self, h5_paths: List[Path],
                                  is_sequence: bool = True,
                                  emg_window_size_sec: Optional[float] = None,
                                  imu_window_size_sec: Optional[float] = None,
                                  window_step_sec: Optional[float] = None,
                                  use_precomputed: bool = True) -> pd.DataFrame:
        """
        Iterates over a list of segmented HDF5 files, extracts temporal and spectral 
        features from the EMG and IMU segments within it, and returns a single DataFrame 
        containing all features and labels. If window parameters are provided, extracts
        sequences of features over sliding windows.
        
        Parameters
        ----------
        use_precomputed : bool, default True
            When True, attempts to read features stored by ``feature_extraction.py``
            instead of re-running FeatureExtractor.  Falls back to live extraction
            automatically if no precomputed features are present, printing a warning.

        Defaults to FEATURE_CONFIG for window properties if not provided.
        """
        # ── Try precomputed path first ─────────────────────────────────────
        if use_precomputed:
            df = self.load_precomputed_features(h5_paths)
            if not df.empty:
                return df
            print(
                "[DataLoader] No precomputed features found. "
                "Falling back to live extraction.\n"
                "  → Run 'python -m model.feature_extraction' to precompute features."
            )

        # ── Live extraction ────────────────────────────────────────────────
        # Fallback to FEATURE_CONFIG if not provided as arguments
        if emg_window_size_sec is None:
            emg_window_size_sec = FEATURE_CONFIG.get('emg_window_size_sec')
        if imu_window_size_sec is None:
            imu_window_size_sec = FEATURE_CONFIG.get('imu_window_size_sec')
        if window_step_sec is None:
            window_step_sec = FEATURE_CONFIG.get('window_step_sec')
            
        from tqdm import tqdm
        
        all_features = []
        
        # Pre-scan total number of segments across all valid files for a smooth progress bar
        total_segments = 0
        valid_paths = []
        for path in h5_paths:
            path = Path(path)
            if path.exists():
                valid_paths.append(path)
                with h5py.File(path, "r") as f:
                    total_segments += sum(1 for k in f.keys() if k.startswith("segment_"))
            else:
                print(f"[WARN] File not found: {path}")
                
        # Main extraction loop
        pbar = tqdm(total=total_segments, desc="Extracting Features", unit="segment")
        for path in valid_paths:
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

                    # Filter EMG channels based on config
                    emg_channels_config = FEATURE_CONFIG.get('emg_channels', {})
                    if emg_cols and emg_channels_config:
                        valid_emg_idx = [i for i, col in enumerate(emg_cols) if emg_channels_config.get(col, True)]
                        emg_cols = [emg_cols[i] for i in valid_emg_idx]
                        if emg.size > 0:
                            emg = emg[:, valid_emg_idx]

                    # Filter IMU channels based on config
                    imu_channels_config = FEATURE_CONFIG.get('imu_channels', {})
                    if imu_cols and imu_channels_config:
                        valid_imu_idx = [i for i, col in enumerate(imu_cols) if imu_channels_config.get(col, True)]
                        imu_cols = [imu_cols[i] for i in valid_imu_idx]
                        if imu.size > 0:
                            imu = imu[:, valid_imu_idx]

                    # Read dynamic sampling rates if available (stored by postprocessing.py & segmentation.py)
                    # We look at the attributes of the datasets inside the segment
                    fs_emg_eff = grp["emg"].attrs.get("fs", self.extractor.emg_fs) if "emg" in grp else self.extractor.emg_fs
                    fs_imu_eff = grp["imu"].attrs.get("fs", self.extractor.imu_fs) if "imu" in grp else self.extractor.imu_fs
                    fs_emg_orig = grp["emg"].attrs.get("fs_orig", fs_emg_eff) if "emg" in grp else fs_emg_eff
                    fs_imu_orig = grp["imu"].attrs.get("fs_orig", fs_imu_eff) if "imu" in grp else fs_imu_eff
                    
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
                    
                    # Save exact numerical sampling rates for records
                    features["emg_fs"] = fs_emg_eff
                    features["emg_fs_orig"] = fs_emg_orig
                    features["imu_fs"] = fs_imu_eff
                    features["imu_fs_orig"] = fs_imu_orig

                    all_features.append(features)
                    pbar.update(1)
                    
        pbar.close()

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
        default_drop = ["label", "state", "weight", "segment_id", "trial_file", "subject", 
                        "emg_fs", "emg_fs_orig", "imu_fs", "imu_fs_orig"]
        
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
