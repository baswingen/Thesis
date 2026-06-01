import sys
from pathlib import Path
import torch

project_root = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis")
sys.path.append(str(project_root))

from model.model_archs.cnn_gru import CNNGRURegressor

model_path = project_root / "model/model_results/P1-P18_pre-dev/cnn-gru-lopo/cnn_gru_model.joblib"
print("Loading model...")
regressor = CNNGRURegressor.load(model_path)
print("Model loaded successfully!")
print("n_channels:", regressor.n_channels)
print("n_static_features (anthropometrics):", regressor.n_static_features)
print("use_anthropometrics:", regressor.use_anthropometrics)
print("cnn_filters:", regressor.cnn_filters)
print("cnn_kernel_sizes:", regressor.cnn_kernel_sizes)
print("pool_size:", regressor.pool_size)
print("gru_hidden_size:", regressor.gru_hidden_size)
print("gru_num_layers:", regressor.gru_num_layers)
print("dropout_rate:", regressor.dropout_rate)
print("target_transform:", regressor.target_transform)
print("loss_history keys:", regressor.loss_history.keys() if regressor.loss_history else "None")
if regressor.loss_history:
    print("train loss len:", len(regressor.loss_history.get("train", [])))
    print("val loss len:", len(regressor.loss_history.get("val", [])))
