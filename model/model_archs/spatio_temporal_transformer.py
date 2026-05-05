import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.utils.rnn as rnn_utils
from torch.utils.data import DataLoader, Dataset
import pandas as pd
import numpy as np
import sys
from pathlib import Path
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import math
from collections import defaultdict
import copy

# Add project root to sys.path
sys.path.append(str(Path(__file__).resolve().parent.parent.parent))

from model.config_model import SPATIO_TEMPORAL_TRANSFORMER_CONFIG, FEATURE_CONFIG, GLOBAL_LOSS_FUNCTION, AUGMENTATION_CONFIG
from model import plotting_utils
from model.data_augmentation import SequenceAugmenter

class SequenceDataset(Dataset):
    def __init__(self, sequences, labels):
        self.sequences = sequences
        self.labels = labels
        
    def __len__(self):
        return len(self.sequences)
        
    def __getitem__(self, idx):
        return self.sequences[idx], self.labels[idx]

def pad_collate_fn(batch):
    sequences, labels = zip(*batch)
    lengths = torch.tensor([len(seq) for seq in sequences])
    padded_seqs = rnn_utils.pad_sequence(sequences, batch_first=True, padding_value=0.0)
    labels = torch.stack(labels)
    return padded_seqs, labels, lengths

class PositionalEncoding(nn.Module):
    def __init__(self, d_model: int, max_len: int = 5000):
        super().__init__()
        pe = torch.zeros(max_len, d_model)
        position = torch.arange(0, max_len, dtype=torch.float).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, d_model, 2).float() * (-math.log(10000.0) / d_model))
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        self.register_buffer('pe', pe.unsqueeze(0))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = x + self.pe[:, :x.size(1), :]
        return x

class SpatioTemporalTransformerNetwork(nn.Module):
    def __init__(self, channel_indices: dict, d_model: int, nhead_spatial: int, num_layers_spatial: int, 
                 nhead_temporal: int, num_layers_temporal: int, dim_feedforward: int, dropout_rate: float,
                 use_checkpointing: bool = False):
        super().__init__()
        
        self.d_model = d_model
        self.use_checkpointing = use_checkpointing
        
        # 1. Channel Projections
        self.channel_projections = nn.ModuleDict()
        self.channel_names = sorted(list(channel_indices.keys()))
        self.channel_indices = channel_indices
        
        for ch in self.channel_names:
            safe_name = ch.replace(' ', '_').replace('(', '').replace(')', '')
            self.channel_projections[safe_name] = nn.Linear(len(channel_indices[ch]), d_model)
            
        num_channels = len(self.channel_names)
        
        # 2. Spatial Positional Encoding (Learnable)
        self.spatial_pos_embedding = nn.Parameter(torch.randn(1, 1, num_channels, d_model))
        
        # 3. Spatial Transformer
        spatial_layer = nn.TransformerEncoderLayer(
            d_model=d_model, nhead=nhead_spatial, dim_feedforward=dim_feedforward, 
            dropout=dropout_rate, batch_first=True
        )
        self.spatial_transformer = nn.TransformerEncoder(
            spatial_layer, num_layers=num_layers_spatial, enable_nested_tensor=True
        )
        
        # 4. Temporal Positional Encoding
        self.temporal_pos_encoder = PositionalEncoding(d_model)
        
        # 5. Temporal Transformer
        temporal_layer = nn.TransformerEncoderLayer(
            d_model=d_model, nhead=nhead_temporal, dim_feedforward=dim_feedforward, 
            dropout=dropout_rate, batch_first=True
        )
        self.temporal_transformer = nn.TransformerEncoder(
            temporal_layer, num_layers=num_layers_temporal, enable_nested_tensor=True
        )
        
        # 6. Final Regressor
        self.fc = nn.Linear(d_model, 1)

    def forward(self, x, lengths):
        # x shape: [batch_size, seq_len, total_features]
        B, T, F = x.shape
        
        # 1. Project individual channels
        channel_tokens = []
        for ch in self.channel_names:
            safe_name = ch.replace(' ', '_').replace('(', '').replace(')', '')
            idxs = self.channel_indices[ch]
            
            # Slice the features for this channel
            ch_x = x[:, :, idxs] # [B, T, features_c]
            proj = self.channel_projections[safe_name](ch_x) # [B, T, d_model]
            channel_tokens.append(proj)
            
        # Stack channels: [B, T, C, d_model]
        x_spatial = torch.stack(channel_tokens, dim=2)
        C = x_spatial.shape[2]
        
        # Add spatial positional embedding
        x_spatial = x_spatial + self.spatial_pos_embedding
        
        # Reshape for Spatial Transformer: [B*T, C, d_model]
        x_spatial = x_spatial.view(B * T, C, self.d_model)
        
        # Pass through Spatial Transformer
        if self.use_checkpointing and self.training:
            def create_custom_forward(module):
                def custom_forward(*inputs):
                    return module(*inputs)
                return custom_forward
            
            # Use checkpointing for the whole spatial encoder
            x_spatial_out = torch.utils.checkpoint.checkpoint(
                create_custom_forward(self.spatial_transformer), 
                x_spatial, 
                use_reentrant=False
            )
        else:
            x_spatial_out = self.spatial_transformer(x_spatial)
        
        # Pool spatial tokens (mean pooling across channels) to create one spatio-temporal token per timestep
        x_temporal_input = x_spatial_out.mean(dim=1) # [B*T, d_model]
        
        # Reshape back for Temporal Transformer: [B, T, d_model]
        x_temporal = x_temporal_input.view(B, T, self.d_model)
        
        # Add Temporal Positional Encoding
        x_temporal = self.temporal_pos_encoder(x_temporal)
        
        # Create Temporal Masks
        padding_mask = torch.arange(T, device=x.device).expand(B, T) >= lengths.unsqueeze(1)
        # Use a float mask instead of bool to avoid -inf overflow under AMP float16.
        # generate_square_subsequent_mask already returns 0.0 / -inf in float,
        # which is handled correctly by scaled_dot_product_attention.
        causal_mask = nn.Transformer.generate_square_subsequent_mask(T, device=x.device)
        
        # Pass through Temporal Transformer
        if self.use_checkpointing and self.training:
            def create_custom_forward(module):
                def custom_forward(src, mask, src_key_padding_mask):
                    return module(src, mask=mask, src_key_padding_mask=src_key_padding_mask)
                return custom_forward
            
            x_temporal_out = torch.utils.checkpoint.checkpoint(
                create_custom_forward(self.temporal_transformer),
                x_temporal, causal_mask, padding_mask,
                use_reentrant=False
            )
        else:
            x_temporal_out = self.temporal_transformer(
                x_temporal, mask=causal_mask, src_key_padding_mask=padding_mask
            )
        
        # Pull last valid token for regression
        last_tokens = x_temporal_out[torch.arange(B), lengths - 1, :]
        
        # Final prediction
        out = self.fc(last_tokens)
        return out


class SpatioTemporalTransformerRegressor:
    def __init__(self, 
                 d_model: int = SPATIO_TEMPORAL_TRANSFORMER_CONFIG['d_model'],
                 nhead_spatial: int = SPATIO_TEMPORAL_TRANSFORMER_CONFIG['nhead_spatial'],
                 num_layers_spatial: int = SPATIO_TEMPORAL_TRANSFORMER_CONFIG['num_layers_spatial'],
                 nhead_temporal: int = SPATIO_TEMPORAL_TRANSFORMER_CONFIG['nhead_temporal'],
                 num_layers_temporal: int = SPATIO_TEMPORAL_TRANSFORMER_CONFIG['num_layers_temporal'],
                 dim_feedforward: int = SPATIO_TEMPORAL_TRANSFORMER_CONFIG['dim_feedforward'],
                 dropout_rate: float = SPATIO_TEMPORAL_TRANSFORMER_CONFIG['dropout_rate'],
                 learning_rate: float = SPATIO_TEMPORAL_TRANSFORMER_CONFIG['learning_rate'],
                 weight_decay: float = SPATIO_TEMPORAL_TRANSFORMER_CONFIG.get('weight_decay', 0.0),
                 batch_size: int = SPATIO_TEMPORAL_TRANSFORMER_CONFIG['batch_size'],
                 epochs: int = SPATIO_TEMPORAL_TRANSFORMER_CONFIG['epochs'],
                 validation_split: float = SPATIO_TEMPORAL_TRANSFORMER_CONFIG.get('validation_split', 0.2),
                 early_stopping_patience: int = SPATIO_TEMPORAL_TRANSFORMER_CONFIG.get('early_stopping_patience', 10),
                 scheduler_patience: int = SPATIO_TEMPORAL_TRANSFORMER_CONFIG.get('scheduler_patience', 5),
                 scheduler_factor: float = SPATIO_TEMPORAL_TRANSFORMER_CONFIG.get('scheduler_factor', 0.5),
                 emg_window_size_sec: float = FEATURE_CONFIG['emg_window_size_sec'],
                 imu_window_size_sec: float = FEATURE_CONFIG['imu_window_size_sec'],
                 window_step_sec: float = FEATURE_CONFIG['window_step_sec'],
                 loss_type: str = GLOBAL_LOSS_FUNCTION,
                 use_checkpointing: bool = SPATIO_TEMPORAL_TRANSFORMER_CONFIG.get('use_checkpointing', True),
                 use_amp: bool = SPATIO_TEMPORAL_TRANSFORMER_CONFIG.get('use_amp', True),
                 random_state: int = SPATIO_TEMPORAL_TRANSFORMER_CONFIG['random_state'],
                 max_seq_len: int = None,
                 scheduler: dict = None,
                 **kwargs):
                 
        self.d_model = d_model
        self.nhead_spatial = nhead_spatial
        self.num_layers_spatial = num_layers_spatial
        self.nhead_temporal = nhead_temporal
        self.num_layers_temporal = num_layers_temporal
        self.dim_feedforward = dim_feedforward
        self.dropout_rate = dropout_rate
        self.learning_rate = learning_rate
        self.weight_decay = weight_decay
        self.batch_size = batch_size
        self.epochs = epochs
        self.validation_split = validation_split
        self.early_stopping_patience = early_stopping_patience
        self.scheduler_patience = scheduler_patience
        self.scheduler_factor = scheduler_factor
        self.emg_window_size_sec = emg_window_size_sec
        self.imu_window_size_sec = imu_window_size_sec
        self.window_step_sec = window_step_sec
        self.loss_type = loss_type.lower()
        self.use_checkpointing = use_checkpointing
        self.use_amp = use_amp
        self.random_state = random_state
        self.max_seq_len = max_seq_len
        
        self.loss_history = {"train": [], "val": []}
        self.train_samples = 0
        self.val_samples = 0
        
        torch.manual_seed(self.random_state)
        
        self.scaler = StandardScaler()
        self.model = None
        self.feature_names = None
        self.channel_indices = None
        
        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        elif torch.backends.mps.is_available():
            self.device = torch.device("mps")
        else:
            self.device = torch.device("cpu")
        print(f"[{self.__class__.__name__}] Using device: {self.device}")
        
    def _extract_sequences(self, X: pd.DataFrame) -> list:
        if 'sequence_dicts' not in X.columns:
            raise ValueError("Transformer expects DataFrame with 'sequence_dicts' column.")
        return X['sequence_dicts'].tolist()

    @staticmethod
    def _sanitise(arr: np.ndarray, clip: float = 1e9) -> np.ndarray:
        arr = np.where(np.isfinite(arr), arr, 0.0)
        return np.clip(arr, -clip, clip)
        
    def _build_channel_indices(self):
        self.channel_indices = defaultdict(list)
        for i, fname in enumerate(self.feature_names):
            if '_EMG_' in fname:
                channel = fname.split('_EMG_')[0] + "_EMG"
            elif '_IMU_' in fname:
                channel = fname.split('_IMU_')[0] + "_IMU"
            elif '_SVM_' in fname:
                channel = fname.split('_SVM_')[0] + "_SVM"
            else:
                channel = 'Other'
            self.channel_indices[channel].append(i)
            
        print(f"[{self.__class__.__name__}] Grouped features into {len(self.channel_indices)} spatial channels.")

    def fit(self, X: pd.DataFrame, y: pd.Series, sample_weight=None, X_val=None, y_val=None, **kwargs):
        from sklearn.model_selection import train_test_split
        from tqdm import tqdm
        
        if X_val is not None and y_val is not None:
            X_train, y_train = X, y
            print(f"[{self.__class__.__name__}] Using explicitly provided validation set ({len(X_val)} samples).")
        else:
            stratify = X["label"] if "label" in X.columns else None
            X_train, X_val, y_train, y_val = train_test_split(
                X, y, test_size=self.validation_split, random_state=self.random_state, stratify=stratify
            )
        
        sequences_train = self._extract_sequences(X_train)
        sequences_val = self._extract_sequences(X_val)
        
        # Determine max_seq_len to cap dataset and avoid excessive padding
        train_lengths = [len(seq) for seq in sequences_train]
        from collections import Counter
        counts = Counter(train_lengths)
        min_count = max(2, int(len(sequences_train) * 0.005))
        valid_lengths = [l for l, c in counts.items() if c >= min_count]
        self.max_seq_len = max(valid_lengths) if valid_lengths else max(train_lengths)
        print(f"[{self.__class__.__name__}] Capping sequence length at {self.max_seq_len} based on sample distribution (min_count={min_count}).")
        
        sequences_train = [seq[:self.max_seq_len] for seq in sequences_train]
        sequences_val = [seq[:self.max_seq_len] for seq in sequences_val]
        
        # Determine feature names robustly (union of all keys in a few samples)
        self.feature_names = []
        seen_keys = set()
        for i in range(min(10, len(sequences_train))):
            if sequences_train[i]:
                for w in sequences_train[i]:
                    for k in w.keys():
                        if k not in seen_keys:
                            self.feature_names.append(k)
                            seen_keys.add(k)
        
        if not self.feature_names:
            raise ValueError("No features found in training sequences.")
            
        self._build_channel_indices()
        
        # Convert to arrays and fit scaler
        flat_data_train = []
        for seq in sequences_train:
            seq_arr = np.array([[w.get(k, 0.0) for k in self.feature_names] for w in seq])
            flat_data_train.append(seq_arr)
            
        all_features_train = self._sanitise(np.vstack(flat_data_train))
        self.scaler.fit(all_features_train)
        
        # Transform sequences
        scaled_seqs_train = []
        for arr in flat_data_train:
            scaled_arr = self.scaler.transform(self._sanitise(arr)).astype(np.float32)
            scaled_seqs_train.append(scaled_arr) 
            
        scaled_seqs_val = []
        for seq in sequences_val:
            seq_arr = np.array([[w.get(k, 0.0) for k in self.feature_names] for w in seq])
            scaled_arr = self.scaler.transform(self._sanitise(seq_arr)).astype(np.float32)
            scaled_seqs_val.append(scaled_arr)
            
        y_np_train = y_train.values.astype(np.float32)
        
        # Augmentation
        augmenter = SequenceAugmenter(config=AUGMENTATION_CONFIG)
        scaled_seqs_train, y_np_train = augmenter.augment_dataset(scaled_seqs_train, y_np_train)
        
        # To tensors
        scaled_seqs_train = [torch.from_numpy(a) for a in scaled_seqs_train]
        y_tensor_train = torch.from_numpy(y_np_train).unsqueeze(1)
        
        scaled_seqs_val = [torch.from_numpy(a) for a in scaled_seqs_val]
        y_tensor_val = torch.from_numpy(y_val.values.astype(np.float32)).unsqueeze(1)
        
        self.train_samples = len(X_train)
        self.val_samples = len(X_val)
        
        # Init Model
        self.model = SpatioTemporalTransformerNetwork(
            channel_indices=self.channel_indices,
            d_model=self.d_model, 
            nhead_spatial=self.nhead_spatial, 
            num_layers_spatial=self.num_layers_spatial,
            nhead_temporal=self.nhead_temporal,
            num_layers_temporal=self.num_layers_temporal,
            dim_feedforward=self.dim_feedforward, 
            dropout_rate=self.dropout_rate,
            use_checkpointing=self.use_checkpointing
        ).to(self.device)
        
        optimizer = optim.AdamW(self.model.parameters(), lr=self.learning_rate, weight_decay=self.weight_decay)
        
        if self.loss_type == 'mae':
            criterion = nn.L1Loss()
        else:
            criterion = nn.MSELoss()
        
        dataset_train = SequenceDataset(scaled_seqs_train, y_tensor_train)
        loader_train = DataLoader(dataset_train, batch_size=self.batch_size, shuffle=True, collate_fn=pad_collate_fn)
        
        from model.config_model import SPATIO_TEMPORAL_TRANSFORMER_CONFIG
        scheduler_config = SPATIO_TEMPORAL_TRANSFORMER_CONFIG.get('scheduler', {'type': 'ReduceLROnPlateau'})
        
        if scheduler_config.get('type') == 'OneCycleLR':
            total_steps = self.epochs * len(loader_train)
            scheduler = optim.lr_scheduler.OneCycleLR(
                optimizer,
                max_lr=scheduler_config.get('max_lr', self.learning_rate),
                total_steps=total_steps,
                pct_start=scheduler_config.get('pct_start', 0.1),
                anneal_strategy='cos'
            )
        else:
            scheduler = optim.lr_scheduler.ReduceLROnPlateau(
                optimizer, mode='min', patience=self.scheduler_patience, factor=self.scheduler_factor
            )
        
        dataset_val = SequenceDataset(scaled_seqs_val, y_tensor_val)
        loader_val = DataLoader(dataset_val, batch_size=self.batch_size, shuffle=False, collate_fn=pad_collate_fn)
        
        best_val_loss = float('inf')
        patience_counter = 0
        best_model_weights = copy.deepcopy(self.model.state_dict())
        
        scaler = torch.amp.GradScaler('cuda', enabled=self.use_amp)
        
        with tqdm(range(self.epochs), desc="Training Spatio-Temporal Transformer", unit="epoch") as pbar:
            for epoch in pbar:
                self.model.train()
                running_loss = 0.0
                n_train_samples = 0
                for batch_x, batch_y, lengths in loader_train:
                    batch_x, batch_y = batch_x.to(self.device), batch_y.to(self.device)
                    lengths = lengths.to(self.device)
                    
                    optimizer.zero_grad()
                    
                    # Use AMP for forward pass
                    with torch.amp.autocast('cuda', enabled=self.use_amp):
                        outputs = self.model(batch_x, lengths)
                        loss = criterion(outputs, batch_y)
                    
                    # Guard against NaN loss (AMP float16 overflow)
                    if not torch.isfinite(loss):
                        optimizer.zero_grad()
                        continue
                    
                    # Scales loss, and calls backward() to create scaled gradients
                    scaler.scale(loss).backward()
                    
                    # Unscales gradients and clips them
                    scaler.unscale_(optimizer)
                    torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=1.0)
                    
                    # Optimizer step and scaler update
                    scaler.step(optimizer)
                    scaler.update()
                    
                    if scheduler_config.get('type') == 'OneCycleLR':
                        scheduler.step()
                        
                    running_loss += loss.item() * batch_x.size(0)
                    n_train_samples += batch_x.size(0)
                    
                avg_train_loss = running_loss / max(n_train_samples, 1)
                
                self.model.eval()
                val_loss = 0.0
                with torch.no_grad():
                    with torch.amp.autocast('cuda', enabled=self.use_amp):
                        for batch_x, batch_y, lengths in loader_val:
                            batch_x, batch_y = batch_x.to(self.device), batch_y.to(self.device)
                            lengths = lengths.to(self.device)
                            outputs = self.model(batch_x, lengths)
                            loss = criterion(outputs, batch_y)
                            val_loss += loss.item() * batch_x.size(0)
                
                avg_val_loss = val_loss / len(dataset_val)
                pbar.set_postfix({"Loss": f"{avg_train_loss:.4f}", "Val Loss": f"{avg_val_loss:.4f}"})
                
                if scheduler_config.get('type') != 'OneCycleLR':
                    scheduler.step(avg_val_loss)
                    
                self.loss_history["train"].append(avg_train_loss)
                self.loss_history["val"].append(avg_val_loss)
                
                if avg_val_loss < best_val_loss:
                    best_val_loss = avg_val_loss
                    patience_counter = 0
                    best_model_weights = copy.deepcopy(self.model.state_dict())
                else:
                    patience_counter += 1
                    
                if patience_counter >= self.early_stopping_patience:
                    print(f"\nEarly stopping triggered at epoch {epoch+1}. Best Val Loss: {best_val_loss:.4f}")
                    break
                    
        self.model.load_state_dict(best_model_weights)
        self.model.eval()
                
    def predict(self, X: pd.DataFrame):
        if self.model is None:
            raise ValueError("Model not fitted.")
            
        sequences = self._extract_sequences(X)
        if getattr(self, 'max_seq_len', None) is not None:
            sequences = [seq[:self.max_seq_len] for seq in sequences]
            
        scaled_seqs = []
        for seq in sequences:
            # Use .get(k, 0.0) to handle missing features in test data
            seq_arr = np.array([[w.get(k, 0.0) for k in self.feature_names] for w in seq])
            scaled_arr = self.scaler.transform(self._sanitise(seq_arr)).astype(np.float32)
            scaled_seqs.append(torch.from_numpy(scaled_arr))
            
        dummy_y = torch.zeros((len(scaled_seqs), 1))
        dataset = SequenceDataset(scaled_seqs, dummy_y)
        loader = DataLoader(dataset, batch_size=self.batch_size, shuffle=False, collate_fn=pad_collate_fn)
        
        self.model.eval()
        all_preds = []
        with torch.no_grad():
            for batch_x, _, lengths in loader:
                batch_x = batch_x.to(self.device)
                lengths = lengths.to(self.device)
                preds = self.model(batch_x, lengths).cpu().numpy()
                all_preds.extend(preds.flatten())
                
        return np.maximum(0.0, np.array(all_preds))

    def evaluate(self, X_test: pd.DataFrame, y_test: pd.Series):
        y_pred = self.predict(X_test)
        mae = mean_absolute_error(y_test, y_pred)
        mse = mean_squared_error(y_test, y_pred)
        rmse = np.sqrt(mse)
        r2 = r2_score(y_test, y_pred)
        
        metrics = {"MAE": mae, "MSE": mse, "RMSE": rmse, "R2": r2}
        report_str = (
            f"Mean Absolute Error: {mae:.4f}\n"
            f"Mean Squared Error: {mse:.4f}\n"
            f"Root Mean Squared Error: {rmse:.4f}\n"
            f"R-squared Score: {r2:.4f}\n"
        )
        return metrics, report_str
        
    def save(self, filepath: str | Path):
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        
        state = {
            'model_state': self.model.state_dict(),
            'scaler': self.scaler,
            'feature_names': self.feature_names,
            'channel_indices': self.channel_indices,
            'config': {
                'd_model': self.d_model,
                'nhead_spatial': self.nhead_spatial,
                'num_layers_spatial': self.num_layers_spatial,
                'nhead_temporal': self.nhead_temporal,
                'num_layers_temporal': self.num_layers_temporal,
                'dim_feedforward': self.dim_feedforward,
                'dropout_rate': self.dropout_rate,
                'learning_rate': self.learning_rate,
                'weight_decay': self.weight_decay,
                'batch_size': self.batch_size,
                'epochs': self.epochs,
                'loss_type': self.loss_type,
                'validation_split': self.validation_split,
                'early_stopping_patience': self.early_stopping_patience,
                'scheduler_patience': self.scheduler_patience,
                'scheduler_factor': self.scheduler_factor,
                'emg_window_size_sec': self.emg_window_size_sec,
                'imu_window_size_sec': self.imu_window_size_sec,
                'window_step_sec': self.window_step_sec,
                'use_checkpointing': self.use_checkpointing,
                'use_amp': self.use_amp,
                'random_state': self.random_state,
                'max_seq_len': getattr(self, 'max_seq_len', None)
            },
            'split_info': {
                'train_samples': self.train_samples,
                'val_samples': self.val_samples
            },
            'loss_history': self.loss_history
        }
        torch.save(state, filepath)
        
    @classmethod
    def load(cls, filepath: str | Path):
        state = torch.load(filepath, weights_only=False)
        regressor = cls(**state['config'])
        regressor.scaler = state['scaler']
        regressor.feature_names = state['feature_names']
        regressor.channel_indices = state['channel_indices']
        
        regressor.model = SpatioTemporalTransformerNetwork(
            channel_indices=state['channel_indices'],
            d_model=state['config']['d_model'], 
            nhead_spatial=state['config']['nhead_spatial'], 
            num_layers_spatial=state['config']['num_layers_spatial'],
            nhead_temporal=state['config']['nhead_temporal'],
            num_layers_temporal=state['config']['num_layers_temporal'],
            dim_feedforward=state['config']['dim_feedforward'],
            dropout_rate=state['config']['dropout_rate'],
            use_checkpointing=state['config'].get('use_checkpointing', False)
        ).to(regressor.device)
        
        regressor.model.load_state_dict(state['model_state'])
        regressor.model.eval()
        
        if 'split_info' in state:
            regressor.train_samples = state['split_info'].get('train_samples', 0)
            regressor.val_samples = state['split_info'].get('val_samples', 0)
            
        if 'loss_history' in state:
            regressor.loss_history = state['loss_history']
            
        return regressor

    def plot_results(self, y_test: pd.Series, y_pred: np.ndarray, save_path: str | Path):
        plotting_utils.plot_regression_results(y_test, y_pred, save_path, model_name="Spatio-Temporal Transformer")

    def plot_loss(self, save_path: str | Path):
        plotting_utils.plot_training_loss(self.loss_history, save_path, model_name="Spatio-Temporal Transformer")
