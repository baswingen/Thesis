"""
generate_tsne.py
================
Standalone t-SNE plot of CNN-extracted features for a trained CNN-LSTM model.

Usage
-----
Run AFTER training with run_model.py. Set SAVED_MODEL_PATH below, then:

    python -m model.generate_tsne

The script loads the full dataset, passes every segment through the trained
CNN encoder (global-average-pooled), reduces to 2-D with t-SNE, and saves a
scatter plot coloured by participant (distinct hue) and weight (light = low,
dark = heavy) next to the saved model file.

Outputs (saved in the same folder as the model):
  - tsne_cnn_features.png
"""

import sys
import numpy as np
from pathlib import Path

# ---------------------------------------------------------------------------
# Project root on sys.path so 'model' package is importable
# ---------------------------------------------------------------------------
ROOT = Path(__file__).resolve().parent.parent
sys.path.append(str(ROOT))

from model.data_loader import DataLoader
from model.model_archs.cnn_lstm import CNNLSTMRegressor
from model.config_model import DATABASE_CONFIG

###########################################################
# CONFIGURATION — edit these before running
###########################################################

# Path to a saved CNN-LSTM model (.joblib produced by run_model.py)
SAVED_MODEL_PATH = "model/model_results/run_20260403_220609/cnn_lstm_model.joblib"

# t-SNE perplexity (30 is a good default; try 15-50 depending on dataset size)
PERPLEXITY: int = 50

# Output filename (saved in the same folder as SAVED_MODEL_PATH)
OUT_NAME: str = "tsne_cnn_features.png"

###########################################################


def main():
    # ── Resolve model path ───────────────────────────────────────────────────
    model_path = Path(SAVED_MODEL_PATH)
    if not model_path.is_absolute():
        model_path = ROOT / model_path

    if not model_path.exists():
        print(f"[ERROR] Model file not found: {model_path}")
        sys.exit(1)

    out_dir = model_path.parent

    # ── Load model ───────────────────────────────────────────────────────────
    print(f"Loading model from: {model_path}")
    model = CNNLSTMRegressor.load(model_path)
    print(f"  Device: {model.device}")

    # ── Load full dataset ────────────────────────────────────────────────────
    segments_dir = DATABASE_CONFIG["segments_dir"]
    h5_paths = sorted(segments_dir.glob("*.h5"))

    if not h5_paths:
        print(f"[ERROR] No HDF5 segment files found in {segments_dir}")
        sys.exit(1)

    print(f"Loading raw segments from {len(h5_paths)} HDF5 file(s)…")
    loader = DataLoader()
    df = loader.load_raw_segments(h5_paths)

    if df.empty:
        print("[ERROR] DataFrame is empty after loading.")
        sys.exit(1)

    print(f"  Total segments: {len(df)}")

    if "subject" not in df.columns:
        print("[ERROR] 'subject' column not found — cannot colour by participant.")
        sys.exit(1)

    # ── Prepare inputs ───────────────────────────────────────────────────────
    X, y = loader.prepare_for_ml(df, target_col="weight")

    participants = df.loc[X.index, "subject"].astype(str).values
    weights = y.values

    # ── Generate & save t-SNE ────────────────────────────────────────────────
    out_path = out_dir / OUT_NAME
    print(f"\nExtracting CNN features and running t-SNE (perplexity={PERPLEXITY})…")
    model.plot_tsne(
        X=X,
        participants=participants,
        weights=weights,
        save_path=out_path,
        perplexity=PERPLEXITY,
    )
    print(f"\nDone. t-SNE plot saved to: {out_path}")


if __name__ == "__main__":
    main()
