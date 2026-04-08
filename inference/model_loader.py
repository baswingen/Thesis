"""
model_loader.py
===============

Loads a trained CNN-LSTM checkpoint and provides a thread-safe
`predict(raw_segment)` method for inference.
"""

from __future__ import annotations

import re
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, List, Dict

import numpy as np
import pandas as pd
import sys

# Ensure torch is available (DLLs should be handled by run_inference.py)
try:
    import torch
except ImportError:
    pass

# Path setup
_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

try:
    from model.model_archs.cnn_lstm import CNNLSTMRegressor
except ImportError:
    # Handle the case where the model package isn't in sys.path yet
    sys.path.append(str(_PROJECT_ROOT))
    from model.model_archs.cnn_lstm import CNNLSTMRegressor

@dataclass
class ModelMetadata:
    """Display metadata parsed from performance_report.txt."""
    run_timestamp: str = "unknown"
    architecture: str = "CNN-LSTM"
    participants: List[str] = field(default_factory=list)
    total_samples: int = 0
    eval_mode: str = ""
    n_channels: int = 26
    cnn_filters: List[int] = field(default_factory=list)
    cnn_kernel_sizes: List[int] = field(default_factory=list)
    pool_size: int = 4
    lstm_hidden_size: int = 512
    lstm_num_layers: int = 4
    dropout_rate: float = 0.4
    loss_function: str = "MSE"
    mae: Optional[float] = None
    mae_std: Optional[float] = None
    rmse: Optional[float] = None
    rmse_std: Optional[float] = None
    r2: Optional[float] = None
    r2_std: Optional[float] = None
    inference_time_per_sample_ms: Optional[float] = None
    per_weight: Dict[float, float] = field(default_factory=dict)
    per_participant: Dict[str, float] = field(default_factory=dict)

def _parse_performance_report(report_path: Path) -> ModelMetadata:
    meta = ModelMetadata()
    if not report_path.exists():
        return meta
    text = report_path.read_text(encoding="utf-8", errors="replace")
    m = re.search(r"Run Timestamp:\s*(\S+)", text)
    if m: meta.run_timestamp = m.group(1)
    m = re.search(r"Participants included:\s*(.+)", text)
    if m: meta.participants = [p.strip() for p in m.group(1).split(",")]
    m = re.search(r"Total samples:\s*(\d+)", text)
    if m: meta.total_samples = int(m.group(1))
    m = re.search(r"Evaluation Mode:\s*(.+)", text)
    if m: meta.eval_mode = m.group(1).strip()
    m = re.search(r"Input Channels:\s*(\d+)", text)
    if m: meta.n_channels = int(m.group(1))
    m = re.search(r"CNN Filters:\s*\[([^\]]+)\]", text)
    if m: meta.cnn_filters = [int(x.strip()) for x in m.group(1).split(",")]
    m = re.search(r"CNN Kernel Sizes:\s*\[([^\]]+)\]", text)
    if m: meta.cnn_kernel_sizes = [int(x.strip()) for x in m.group(1).split(",")]
    m = re.search(r"Pool Size:\s*(\d+)", text)
    if m: meta.pool_size = int(m.group(1))
    m = re.search(r"LSTM Hidden Size:\s*(\d+)", text)
    if m: meta.lstm_hidden_size = int(m.group(1))
    m = re.search(r"LSTM Num Layers:\s*(\d+)", text)
    if m: meta.lstm_num_layers = int(m.group(1))
    m = re.search(r"Dropout Rate:\s*([\d.]+)", text)
    if m: meta.dropout_rate = float(m.group(1))
    m = re.search(r"Loss Function:\s*(\S+)", text)
    if m: meta.loss_function = m.group(1)
    m = re.search(r"^MAE:\s*([\d.]+)\s*\(±([\d.]+)\)", text, re.MULTILINE)
    if m:
        meta.mae = float(m.group(1))
        meta.mae_std = float(m.group(2))
    m = re.search(r"^RMSE:\s*([\d.]+)\s*\(±([\d.]+)\)", text, re.MULTILINE)
    if m:
        meta.rmse = float(m.group(1))
        meta.rmse_std = float(m.group(2))
    m = re.search(r"^R2:\s*([\d.]+)\s*\(±([\d.]+)\)", text, re.MULTILINE)
    if m:
        meta.r2 = float(m.group(1))
        meta.r2_std = float(m.group(2))
    m = re.search(r"Avg Inference Time \(per sample\):\s*([\d.]+)ms", text)
    if m: meta.inference_time_per_sample_ms = float(m.group(1))
    for m in re.finditer(r"([\d.]+)\s+kg\s+\|\s+\d+\s+\|\s+([\d.]+)", text):
        meta.per_weight[float(m.group(1))] = float(m.group(2))
    for m in re.finditer(r"(P\d+)\s+\|\s+\d+\s+\|\s+([\d.]+)", text):
        meta.per_participant[m.group(1)] = float(m.group(2))
    return meta

class ModelLoader:
    def __init__(self, run_dir: str | Path):
        run_dir = Path(run_dir)
        if not run_dir.is_absolute():
            run_dir = _PROJECT_ROOT / run_dir
        self._run_dir = run_dir
        self._lock = threading.Lock()
        checkpoint_path = run_dir / "cnn_lstm_model.joblib"
        if not checkpoint_path.exists():
            raise FileNotFoundError(f"[ModelLoader] Checkpoint not found: {checkpoint_path}")
        print(f"[ModelLoader] Loading checkpoint: {checkpoint_path}")
        self._regressor = CNNLSTMRegressor.load(checkpoint_path)
        self._regressor.model.eval()
        self.n_channels = self._regressor.n_channels
        self.cnn_filters = self._regressor.cnn_filters
        self.cnn_kernel_sizes = self._regressor.cnn_kernel_sizes
        self.pool_size = self._regressor.pool_size
        self.lstm_hidden_size = self._regressor.lstm_hidden_size
        self.lstm_num_layers = self._regressor.lstm_num_layers
        self.dropout_rate = self._regressor.dropout_rate
        print(f"[ModelLoader] Loaded: {self.n_channels} channels | CNN {self.cnn_filters} | LSTM h={self.lstm_hidden_size}x{self.lstm_num_layers}")
        report_path = run_dir / "performance_report.txt"
        self.metadata = _parse_performance_report(report_path)
        if self.metadata.n_channels != self.n_channels:
            self.metadata.n_channels = self.n_channels
        print(f"[ModelLoader] Metadata: MAE={self.metadata.mae} kg (±{self.metadata.mae_std}), R^2={self.metadata.r2}")

    def predict(self, raw_segment: np.ndarray) -> float:
        if raw_segment.ndim != 2:
            raise ValueError(f"raw_segment must be 2-D (T, C), got shape {raw_segment.shape}")
        if raw_segment.shape[1] != self.n_channels:
            raise ValueError(f"raw_segment has {raw_segment.shape[1]} channels; model expects {self.n_channels}")
        df = pd.DataFrame({"raw_segment": [raw_segment.astype(np.float32)]})
        with self._lock:
            preds = self._regressor.predict(df)
        return float(max(0.0, preds[0]))

    @property
    def run_dir(self) -> Path:
        return self._run_dir

    def summary(self) -> str:
        m = self.metadata
        mae_str = f"{m.mae:.3f} \u00b1{m.mae_std:.3f}" if m.mae is not None else "N/A"
        r2_str = f"{m.r2:.3f}" if m.r2 is not None else "N/A"
        return (f"CNN-LSTM | {self.n_channels} ch | CNN {self.cnn_filters} | LSTM h={self.lstm_hidden_size}x{self.lstm_num_layers} | "
                f"MAE={mae_str} kg | R^2={r2_str} | run={m.run_timestamp}")
