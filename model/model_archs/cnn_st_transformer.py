import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.nn.utils.rnn as rnn_utils
from torch.utils.data import DataLoader, Dataset, WeightedRandomSampler
import pandas as pd
import numpy as np
import sys
from pathlib import Path
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
from collections import defaultdict
import copy
import math

# Add project root to sys.path
sys.path.append(str(Path(__file__).resolve().parent.parent.parent))

from model.config_model import CNN_ST_TRANSFORMER_CONFIG, GLOBAL_LOSS_FUNCTION, AUGMENTATION_CONFIG, GLOBAL_BALANCE_WEIGHTS
from model import plotting_utils
from model.data_augmentation import SequenceAugmenter
from sklearn.utils.class_weight import compute_sample_weight

# ---------------------------------------------------------------------------
# Dataset & collate for variable-length raw segments
# ---------------------------------------------------------------------------

class RawSegmentDataset(Dataset):
    """Holds raw segments as tensors of shape (time_steps, n_channels)."""
    def __init__(self, segments, labels, participant_ids=None, sample_weights=None,
                 static_features=None):
        self.segments = segments          # list of tensors (T_i, C)
        self.labels = labels              # tensor (N, 1)
        self.participant_ids = participant_ids  # list of str or None
        self.sample_weights = sample_weights    # tensor (N, 1) or None
        self.static_features = static_features  # tensor (N, D_static) or None

    def __len__(self):
        return len(self.segments)

    def __getitem__(self, idx):
        pid = self.participant_ids[idx] if self.participant_ids is not None else ""
        weight = self.sample_weights[idx] if self.sample_weights is not None else torch.tensor([1.0], dtype=torch.float32)
        static = self.static_features[idx] if self.static_features is not None else torch.empty(0)
        return self.segments[idx], self.labels[idx], pid, weight, static


def raw_pad_collate_fn(batch):
    """Pad variable-length segments and return lengths for packing/masking."""
    segments, labels, pids, weights, statics = zip(*batch)
    lengths = torch.tensor([seg.shape[0] for seg in segments])
    padded = rnn_utils.pad_sequence(segments, batch_first=True, padding_value=0.0)
    labels = torch.stack(labels)
    weights = torch.stack(weights)
    if statics[0].numel() > 0:
        statics = torch.stack(statics)
    else:
        statics = None
    return padded, labels, lengths, list(pids), weights, statics

# ---------------------------------------------------------------------------
# Transformer Utilities
# ---------------------------------------------------------------------------

class PositionalEncoding(nn.Module):
    def __init__(self, d_model: int, max_len: int = 15000):
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

class AttentionPooling(nn.Module):
    def __init__(self, d_model: int):
        super().__init__()
        self.attention = nn.Sequential(
            nn.Linear(d_model, d_model // 2),
            nn.Tanh(),
            nn.Linear(d_model // 2, 1)
        )

    def forward(self, x):
        # x: [Batch, Num_Channels, d_model]
        attn_weights = self.attention(x) # [Batch, Num_Channels, 1]
        attn_weights = F.softmax(attn_weights, dim=1)
        pooled = torch.sum(attn_weights * x, dim=1) # [Batch, d_model]
        return pooled

# ---------------------------------------------------------------------------
# CNN Tokenizer
# ---------------------------------------------------------------------------

class CNNFeatureExtractor(nn.Module):
    def __init__(self, n_channels: int, cnn_filters: list[int],
                 cnn_kernel_sizes: list[int], pool_size: int,
                 dropout_rate: float):
        super().__init__()
        layers = []
        in_ch = n_channels
        for filters, ks in zip(cnn_filters, cnn_kernel_sizes):
            padding = ks // 2
            layers.extend([
                nn.Conv1d(in_ch, filters, kernel_size=ks, padding=padding),
                nn.BatchNorm1d(filters),
                nn.ReLU(inplace=False),
                nn.MaxPool1d(kernel_size=pool_size),
                nn.Dropout(dropout_rate),
            ])
            in_ch = filters
        self.net = nn.Sequential(*layers)
        self.out_channels = cnn_filters[-1]
        self.pool_size = pool_size
        self.n_blocks = len(cnn_filters)

    def forward(self, x):
        # x: (batch, n_channels, time)
        out = self.net(x)          # (batch, out_channels, reduced_time)
        out = out.transpose(1, 2)  # (batch, reduced_time, out_channels)
        return out

# ---------------------------------------------------------------------------
# Full CNN-ST-Transformer Network
# ---------------------------------------------------------------------------

class CNNSTTransformerNetwork(nn.Module):
    def __init__(self, channel_indices: dict, cnn_filters: list[int],
                 cnn_kernel_sizes: list[int], pool_size: int,
                 d_model: int, nhead_spatial: int, num_layers_spatial: int, 
                 nhead_temporal: int, num_layers_temporal: int, dim_feedforward: int, dropout_rate: float,
                 use_checkpointing: bool = False, n_static_features: int = 0):
        super().__init__()
        
        self.d_model = d_model
        self.use_checkpointing = use_checkpointing
        self.n_static_features = n_static_features
        
        # 1. Channel CNNs
        self.channel_cnns = nn.ModuleDict()
        self.channel_names = sorted(list(channel_indices.keys()))
        self.channel_indices = channel_indices
        
        for ch in self.channel_names:
            safe_name = ch.replace(' ', '_').replace('(', '').replace(')', '')
            num_features = len(channel_indices[ch])
            self.channel_cnns[safe_name] = CNNFeatureExtractor(
                n_channels=num_features,
                cnn_filters=cnn_filters,
                cnn_kernel_sizes=cnn_kernel_sizes,
                pool_size=pool_size,
                dropout_rate=dropout_rate
            )
            
        num_channels = len(self.channel_names)
        assert cnn_filters[-1] == d_model, f"Last CNN filter ({cnn_filters[-1]}) must match d_model ({d_model})"
        
        # 2. Spatial Positional Encoding
        self.spatial_pos_embedding = nn.Parameter(torch.randn(1, 1, num_channels, d_model))
        
        # 3. Spatial Transformer
        spatial_layer = nn.TransformerEncoderLayer(
            d_model=d_model, nhead=nhead_spatial, dim_feedforward=dim_feedforward, 
            dropout=dropout_rate, batch_first=True
        )
        self.spatial_transformer = nn.TransformerEncoder(
            spatial_layer, num_layers=num_layers_spatial, enable_nested_tensor=False
        )
        
        # 4. Temporal Positional Encoding
        self.temporal_pos_encoder = PositionalEncoding(d_model)
        
        # 5. Temporal Transformer
        temporal_layer = nn.TransformerEncoderLayer(
            d_model=d_model, nhead=nhead_temporal, dim_feedforward=dim_feedforward, 
            dropout=dropout_rate, batch_first=True
        )
        self.temporal_transformer = nn.TransformerEncoder(
            temporal_layer, num_layers=num_layers_temporal, enable_nested_tensor=False
        )
        
        self.attention_pool = AttentionPooling(d_model)
        self.fc = nn.Linear(d_model + n_static_features, 1)
        self.dropout = nn.Dropout(dropout_rate)
        
        self.pool_size = pool_size
        self.n_blocks = len(cnn_filters)

    def _adjust_lengths(self, lengths):
        adj = lengths.float()
        for _ in range(self.n_blocks):
            adj = torch.floor(adj / self.pool_size)
        return adj.long().clamp(min=1)

    def forward(self, x, lengths, static_features=None):
        B, T_raw, F = x.shape
        
        channel_tokens = []
        for ch in self.channel_names:
            safe_name = ch.replace(' ', '_').replace('(', '').replace(')', '')
            idxs = self.channel_indices[ch]
            
            # [B, T_raw, features_c] -> [B, features_c, T_raw]
            ch_x = x[:, :, idxs].transpose(1, 2)
            cnn_out = self.channel_cnns[safe_name](ch_x) 
            channel_tokens.append(cnn_out)
            
        # [B, Reduced_Time, Num_Nodes, d_model]
        x_grid = torch.stack(channel_tokens, dim=2)
        T = x_grid.shape[1]
        C = x_grid.shape[2]
        
        x_grid = x_grid + self.spatial_pos_embedding
        adj_lengths = self._adjust_lengths(lengths).clamp(max=T)
        
        # Spatial
        x_spatial = x_grid.view(B * T, C, self.d_model)
        
        if self.use_checkpointing and self.training:
            def create_custom_forward(module):
                def custom_forward(*inputs): return module(*inputs)
                return custom_forward
            x_spatial_out = torch.utils.checkpoint.checkpoint(
                create_custom_forward(self.spatial_transformer), x_spatial, use_reentrant=False
            )
        else:
            x_spatial_out = self.spatial_transformer(x_spatial)
        
        # Temporal
        x_temporal = x_grid.transpose(1, 2).contiguous().view(B * C, T, self.d_model)
        x_temporal = self.temporal_pos_encoder(x_temporal)
        
        causal_mask = nn.Transformer.generate_square_subsequent_mask(T, device=x.device)
        padding_bool = torch.arange(T, device=x.device).expand(B, T) >= adj_lengths.unsqueeze(1)
        padding_bool_expanded = padding_bool.unsqueeze(1).expand(B, C, T).reshape(B * C, T)
        padding_mask = torch.zeros_like(padding_bool_expanded, dtype=causal_mask.dtype)
        padding_mask = padding_mask.masked_fill(padding_bool_expanded, float("-inf"))
        
        if self.use_checkpointing and self.training:
            def create_custom_forward(module):
                def custom_forward(src, mask, src_key_padding_mask):
                    return module(src, mask=mask, src_key_padding_mask=src_key_padding_mask)
                return custom_forward
            x_temporal_out = torch.utils.checkpoint.checkpoint(
                create_custom_forward(self.temporal_transformer),
                x_temporal, causal_mask, padding_mask, use_reentrant=False
            )
        else:
            x_temporal_out = self.temporal_transformer(
                x_temporal, mask=causal_mask, src_key_padding_mask=padding_mask
            )
        
        x_spatial_out = x_spatial_out.view(B, T, C, self.d_model)
        x_temporal_out = x_temporal_out.view(B, C, T, self.d_model).transpose(1, 2).contiguous()
        
        x_merged = x_spatial_out + x_temporal_out
        
        # Gather last tokens
        last_tokens = x_merged[torch.arange(B), adj_lengths - 1, :, :] # [B, C, d_model]
        pooled = self.attention_pool(last_tokens) # [B, d_model]
        pooled = self.dropout(pooled)
        
        if self.n_static_features > 0:
            if static_features is None:
                static_features = torch.zeros((pooled.size(0), self.n_static_features), device=pooled.device)
            pooled = torch.cat([pooled, static_features], dim=1)
            
        out = self.fc(pooled)
        return out

# ---------------------------------------------------------------------------
# Regressor Wrapper
# ---------------------------------------------------------------------------

class CNNSTTransformerRegressor:
    def __init__(self,
                 cnn_filters: list[int] = CNN_ST_TRANSFORMER_CONFIG['cnn_filters'],
                 cnn_kernel_sizes: list[int] = CNN_ST_TRANSFORMER_CONFIG['cnn_kernel_sizes'],
                 pool_size: int = CNN_ST_TRANSFORMER_CONFIG['pool_size'],
                 d_model: int = CNN_ST_TRANSFORMER_CONFIG['d_model'],
                 nhead_spatial: int = CNN_ST_TRANSFORMER_CONFIG['nhead_spatial'],
                 num_layers_spatial: int = CNN_ST_TRANSFORMER_CONFIG['num_layers_spatial'],
                 nhead_temporal: int = CNN_ST_TRANSFORMER_CONFIG['nhead_temporal'],
                 num_layers_temporal: int = CNN_ST_TRANSFORMER_CONFIG['num_layers_temporal'],
                 dim_feedforward: int = CNN_ST_TRANSFORMER_CONFIG['dim_feedforward'],
                 dropout_rate: float = CNN_ST_TRANSFORMER_CONFIG['dropout_rate'],
                 learning_rate: float = CNN_ST_TRANSFORMER_CONFIG['learning_rate'],
                 weight_decay: float = CNN_ST_TRANSFORMER_CONFIG.get('weight_decay', 0.0),
                 batch_size: int = CNN_ST_TRANSFORMER_CONFIG['batch_size'],
                 epochs: int = CNN_ST_TRANSFORMER_CONFIG['epochs'],
                 validation_split: float = CNN_ST_TRANSFORMER_CONFIG.get('validation_split', 0.2),
                 early_stopping_patience: int = CNN_ST_TRANSFORMER_CONFIG.get('early_stopping_patience', 10),
                 scheduler_patience: int = CNN_ST_TRANSFORMER_CONFIG.get('scheduler_patience', 5),
                 scheduler_factor: float = CNN_ST_TRANSFORMER_CONFIG.get('scheduler_factor', 0.5),
                 loss_type: str = GLOBAL_LOSS_FUNCTION,
                 use_checkpointing: bool = CNN_ST_TRANSFORMER_CONFIG.get('use_checkpointing', True),
                 use_amp: bool = CNN_ST_TRANSFORMER_CONFIG.get('use_amp', True),
                 balance_weights: bool = CNN_ST_TRANSFORMER_CONFIG.get('balance_weights', GLOBAL_BALANCE_WEIGHTS),
                 balance_participants: bool = CNN_ST_TRANSFORMER_CONFIG.get('balance_participants', False),
                 use_anthropometrics: bool = CNN_ST_TRANSFORMER_CONFIG.get('use_anthropometrics', False),
                 random_state: int = CNN_ST_TRANSFORMER_CONFIG['random_state'],
                 **kwargs):

        self.cnn_filters = list(cnn_filters)
        self.cnn_kernel_sizes = list(cnn_kernel_sizes)
        self.pool_size = pool_size
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
        self.loss_type = loss_type.lower()
        self.use_checkpointing = use_checkpointing
        self.use_amp = use_amp
        self.balance_weights = balance_weights
        self.balance_participants = balance_participants
        self.use_anthropometrics = use_anthropometrics
        self.random_state = random_state

        self.loss_history = {"train": [], "val": []}
        self.train_samples = 0
        self.val_samples = 0

        torch.manual_seed(self.random_state)

        self.scaler = StandardScaler()
        self.anthro_scaler = StandardScaler()
        self.model = None
        self.channel_names = None
        self.channel_indices = None
        self.n_static_features = 0

        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        elif torch.backends.mps.is_available():
            self.device = torch.device("mps")
        else:
            self.device = torch.device("cpu")
        print(f"[{self.__class__.__name__}] Using device: {self.device}")

    @staticmethod
    def _extract_raw_segments(X: pd.DataFrame) -> list[np.ndarray]:
        if 'raw_segment' not in X.columns:
            raise ValueError("Expected a DataFrame with a 'raw_segment' column.")
        return X['raw_segment'].tolist()

    @staticmethod
    def _extract_anthropometrics(X: pd.DataFrame) -> np.ndarray | None:
        if 'anthropometrics' not in X.columns:
            return None
        return np.vstack(X['anthropometrics'].values).astype(np.float32)

    @staticmethod
    def _sanitise(arr: np.ndarray, clip: float = 1e9) -> np.ndarray:
        arr = np.where(np.isfinite(arr), arr, 0.0)
        return np.clip(arr, -clip, clip)

    def _scale_segments(self, segments: list[np.ndarray], fit: bool = False) -> list[np.ndarray]:
        if fit:
            all_data = self._sanitise(np.vstack(segments))
            self.scaler.fit(all_data)
        scaled = []
        for seg in segments:
            scaled.append(self.scaler.transform(self._sanitise(seg)).astype(np.float32))
        return scaled

    def _build_channel_indices(self):
        self.channel_indices = defaultdict(list)
        for i, fname in enumerate(self.channel_names):
            if '_EMG_' in fname:
                channel = fname.split('_EMG_')[0] + "_EMG"
            elif '_IMU_' in fname:
                channel = fname.split('_IMU_')[0] + "_IMU"
            elif '_SVM_' in fname:
                channel = fname.split('_SVM_')[0] + "_SVM"
            # Fallbacks for raw columns without suffixes
            elif '1' in fname and fname.startswith(('ax', 'ay', 'az', 'roll', 'pitch', 'yaw')):
                channel = 'IMU1'
            elif '2' in fname and fname.startswith(('ax', 'ay', 'az', 'roll', 'pitch', 'yaw')):
                channel = 'IMU2'
            else:
                channel = fname + "_EMG"
            self.channel_indices[channel].append(i)
        print(f"[{self.__class__.__name__}] Grouped raw features into {len(self.channel_indices)} spatial nodes.")

    def fit(self, X: pd.DataFrame, y: pd.Series, sample_weight=None, X_val=None, y_val=None):
        from sklearn.model_selection import train_test_split
        from tqdm import tqdm

        if X_val is not None and y_val is not None:
            X_train, y_train = X, y
            print(f"[{self.__class__.__name__}] Using explicit validation set ({len(X_val)} samples).")
        else:
            stratify = X["label"] if "label" in X.columns else None
            X_train, X_val, y_train, y_val = train_test_split(
                X, y, test_size=self.validation_split, random_state=self.random_state, stratify=stratify
            )

        self.channel_names = X_train.attrs.get("channel_names")
        if not self.channel_names:
            raise ValueError("channel_names not found in DataFrame attributes.")
        self._build_channel_indices()

        segs_train = self._extract_raw_segments(X_train)
        segs_val = self._extract_raw_segments(X_val)

        scaled_train = self._scale_segments(segs_train, fit=True)
        scaled_val = self._scale_segments(segs_val, fit=False)
        y_np_train = y_train.values.astype(np.float32)
        y_np_val = y_val.values.astype(np.float32)

        augmenter = SequenceAugmenter(config=AUGMENTATION_CONFIG)
        pids_train = X_train['subject'].astype(str).values if 'subject' in X_train.columns else None
        scaled_train, y_np_train, aug_pids = augmenter.augment_dataset(
            scaled_train, y_np_train, participant_ids=pids_train, return_pids=True
        )

        anthro_train_tensor = None
        anthro_val_tensor = None
        anthro_raw = self._extract_anthropometrics(X_train) if self.use_anthropometrics else None
        if anthro_raw is not None:
            self.n_static_features = anthro_raw.shape[1]
            anthro_scaled = self.anthro_scaler.fit_transform(anthro_raw)
            if len(anthro_scaled) < len(scaled_train):
                n_orig = len(anthro_raw)
                n_total = len(scaled_train)
                anthro_scaled = np.vstack([
                    anthro_scaled,
                    anthro_scaled[np.random.choice(n_orig, n_total - n_orig, replace=True)]
                ])
            anthro_train_tensor = torch.from_numpy(anthro_scaled.astype(np.float32))
            anthro_val_raw = self._extract_anthropometrics(X_val)
            if anthro_val_raw is not None:
                anthro_val_tensor = torch.from_numpy(self.anthro_scaler.transform(anthro_val_raw).astype(np.float32))
        else:
            self.n_static_features = 0

        train_tensors = [torch.from_numpy(s) for s in scaled_train]
        y_tensor_train = torch.from_numpy(y_np_train).unsqueeze(1)
        val_tensors = [torch.from_numpy(s) for s in scaled_val]
        y_tensor_val = torch.from_numpy(y_np_val).unsqueeze(1)

        self.train_samples = len(X_train)
        self.val_samples = len(X_val)

        sampler = None
        if self.balance_weights:
            w = compute_sample_weight(class_weight='balanced', y=y_np_train.flatten())
            sampler_weights = torch.from_numpy(w.astype(np.float64))
            _g_sampler = torch.Generator()
            _g_sampler.manual_seed(self.random_state)
            sampler = WeightedRandomSampler(sampler_weights, len(sampler_weights), replacement=True, generator=_g_sampler)

        self.model = CNNSTTransformerNetwork(
            channel_indices=self.channel_indices,
            cnn_filters=self.cnn_filters,
            cnn_kernel_sizes=self.cnn_kernel_sizes,
            pool_size=self.pool_size,
            d_model=self.d_model,
            nhead_spatial=self.nhead_spatial,
            num_layers_spatial=self.num_layers_spatial,
            nhead_temporal=self.nhead_temporal,
            num_layers_temporal=self.num_layers_temporal,
            dim_feedforward=self.dim_feedforward,
            dropout_rate=self.dropout_rate,
            use_checkpointing=self.use_checkpointing,
            n_static_features=self.n_static_features
        ).to(self.device)

        optimizer = optim.AdamW(self.model.parameters(), lr=self.learning_rate, weight_decay=self.weight_decay)
        criterion = nn.L1Loss() if self.loss_type == 'mae' else nn.MSELoss()

        dataset_train = RawSegmentDataset(train_tensors, y_tensor_train, static_features=anthro_train_tensor)
        loader_train = DataLoader(dataset_train, batch_size=self.batch_size, shuffle=(sampler is None),
                                  sampler=sampler, collate_fn=raw_pad_collate_fn)

        scheduler = optim.lr_scheduler.ReduceLROnPlateau(
            optimizer, mode='min', factor=self.scheduler_factor, patience=self.scheduler_patience, min_lr=1e-6
        )

        dataset_val = RawSegmentDataset(val_tensors, y_tensor_val, static_features=anthro_val_tensor)
        loader_val = DataLoader(dataset_val, batch_size=self.batch_size, shuffle=False, collate_fn=raw_pad_collate_fn)

        best_val_loss = float('inf')
        patience_counter = 0
        best_weights = copy.deepcopy(self.model.state_dict())
        
        amp_device = 'cuda' if self.device.type == 'cuda' else 'cpu'
        scaler = torch.amp.GradScaler(amp_device, enabled=(self.use_amp and self.device.type == 'cuda'))

        with tqdm(range(self.epochs), desc="Training CNN-ST-Transformer", unit="epoch") as pbar:
            for epoch in pbar:
                self.model.train()
                running_loss = 0.0
                for batch_x, batch_y, lengths, _, _, batch_static in loader_train:
                    # Note: x shape is [B, T, F], which matches what forward expects. No transpose here.
                    batch_x, batch_y = batch_x.to(self.device), batch_y.to(self.device)
                    lengths = lengths.to(self.device)
                    batch_static_dev = batch_static.to(self.device) if batch_static is not None else None

                    optimizer.zero_grad()
                    with torch.amp.autocast(self.device.type, enabled=(self.use_amp and self.device.type == 'cuda')):
                        outputs = self.model(batch_x, lengths, static_features=batch_static_dev)
                        loss = criterion(outputs, batch_y)

                    if not torch.isfinite(loss):
                        optimizer.zero_grad()
                        continue

                    scaler.scale(loss).backward()
                    scaler.unscale_(optimizer)
                    torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=1.0)
                    scaler.step(optimizer)
                    scaler.update()

                    running_loss += loss.item() * batch_x.size(0)

                avg_train_loss = running_loss / len(dataset_train)

                self.model.eval()
                val_loss = 0.0
                with torch.no_grad():
                    with torch.amp.autocast(self.device.type, enabled=(self.use_amp and self.device.type == 'cuda')):
                        for batch_x, batch_y, lengths, _, _, batch_static in loader_val:
                            batch_x, batch_y = batch_x.to(self.device), batch_y.to(self.device)
                            lengths = lengths.to(self.device)
                            batch_static_dev = batch_static.to(self.device) if batch_static is not None else None

                            outputs = self.model(batch_x, lengths, static_features=batch_static_dev)
                            loss = criterion(outputs, batch_y)
                            val_loss += loss.item() * batch_x.size(0)
                
                avg_val_loss = val_loss / len(dataset_val)
                pbar.set_postfix({"Loss": f"{avg_train_loss:.4f}", "Val Loss": f"{avg_val_loss:.4f}"})
                
                self.loss_history["train"].append(avg_train_loss)
                self.loss_history["val"].append(avg_val_loss)
                
                scheduler.step(avg_val_loss)
                
                if avg_val_loss < best_val_loss:
                    best_val_loss = avg_val_loss
                    patience_counter = 0
                    best_weights = copy.deepcopy(self.model.state_dict())
                else:
                    patience_counter += 1

                if patience_counter >= self.early_stopping_patience:
                    print(f"\nEarly stopping at epoch {epoch+1}. Best Val Loss: {best_val_loss:.4f}")
                    break

        self.model.load_state_dict(best_weights)
        self.model.eval()

    def predict(self, X: pd.DataFrame):
        if self.model is None:
            raise ValueError("Model not fitted.")
        
        segments = self._extract_raw_segments(X)
        anthro = self._extract_anthropometrics(X) if self.use_anthropometrics and self.n_static_features > 0 else None
        
        scaled = self._scale_segments(segments, fit=False)
        tensors = [torch.from_numpy(s) for s in scaled]
        
        anthro_tensor = None
        if anthro is not None and self.n_static_features > 0:
            anthro_scaled = self.anthro_scaler.transform(anthro)
            anthro_tensor = torch.from_numpy(anthro_scaled.astype(np.float32))
            
        dummy_y = torch.zeros((len(tensors), 1))
        dataset = RawSegmentDataset(tensors, dummy_y, static_features=anthro_tensor)
        loader = DataLoader(dataset, batch_size=self.batch_size, shuffle=False, collate_fn=raw_pad_collate_fn)
        
        self.model.eval()
        all_preds = []
        with torch.no_grad():
            with torch.amp.autocast(self.device.type, enabled=(self.use_amp and self.device.type == 'cuda')):
                for batch_x, _, lengths, _, _, batch_static in loader:
                    batch_x = batch_x.to(self.device)
                    lengths = lengths.to(self.device)
                    batch_static_dev = batch_static.to(self.device) if batch_static is not None else None
                    preds = self.model(batch_x, lengths, static_features=batch_static_dev).cpu().numpy()
                    all_preds.extend(preds.flatten())
                    
        return np.maximum(0.0, np.array(all_preds))

    def evaluate(self, X_test: pd.DataFrame, y_test: pd.Series):
        y_pred = self.predict(X_test)
        mae = mean_absolute_error(y_test, y_pred)
        mse = mean_squared_error(y_test, y_pred)
        rmse = np.sqrt(mse)
        r2 = r2_score(y_test, y_pred)
        
        if len(y_test) > 1 and np.std(y_test) > 0 and np.std(y_pred) > 0:
            corr = np.corrcoef(y_test, y_pred)[0, 1]
        else:
            corr = 0.0
            
        metrics = {"MAE": mae, "MSE": mse, "RMSE": rmse, "R2": r2, "Correlation": corr}
        report_str = (
            f"Mean Absolute Error: {mae:.4f}\n"
            f"Mean Squared Error: {mse:.4f}\n"
            f"Root Mean Squared Error: {rmse:.4f}\n"
            f"R-squared Score: {r2:.4f}\n"
            f"Pearson Correlation: {corr:.4f}\n"
        )
        return metrics, report_str
