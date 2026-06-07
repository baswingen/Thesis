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

from model.config_model import SPATIO_TEMPORAL_TRANSFORMER3_CONFIG, FEATURE_CONFIG, GLOBAL_LOSS_FUNCTION, AUGMENTATION_CONFIG
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

class ModalityGroupedSTBlock(nn.Module):
    """Interleaved spatio-temporal block with modality-grouped temporal attention.
    
    Instead of one shared temporal MHA for all channels (transformer2) or one
    per channel (Aksan), this block creates one temporal MHA per *modality group*
    (e.g. EMG, IMU-Accel, IMU-Orient).  Channels within a group share temporal
    weights; different modalities get separate Q/K/V projections.
    """
    def __init__(self, d_model, nhead_spatial, nhead_temporal, dim_feedforward, dropout, modality_groups):
        super().__init__()
        # modality_groups: dict mapping group_name -> list of channel indices (positions in C dim)
        self.modality_groups = modality_groups
        
        # Shared spatial attention (cross-channel, same as transformer2)
        self.spatial_attn = nn.MultiheadAttention(d_model, nhead_spatial, dropout, batch_first=True)
        
        # One temporal MHA per modality group
        self.temporal_attns = nn.ModuleDict({
            gname: nn.MultiheadAttention(d_model, nhead_temporal, dropout, batch_first=True)
            for gname in modality_groups.keys()
        })
        
        # Norms & FFN
        self.norm1 = nn.LayerNorm(d_model)
        self.norm2 = nn.LayerNorm(d_model)
        self.ffn = nn.Sequential(
            nn.Linear(d_model, dim_feedforward),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(dim_feedforward, d_model)
        )
        self.dropout = nn.Dropout(dropout)
        
    def forward(self, x, causal_mask=None, padding_mask=None):
        B, T, C, d = x.shape
        
        # 1. Spatial Attention (over Channels) — shared, identical to transformer2
        x_spatial = x.view(B * T, C, d)
        spatial_out, _ = self.spatial_attn(x_spatial, x_spatial, x_spatial)
        spatial_out = spatial_out.view(B, T, C, d)
        
        # 2. Modality-Grouped Temporal Attention
        # spatial_out already went through MHA, so it reflects the current AMP dtype (Float or Half)
        temporal_out = torch.zeros_like(x, dtype=spatial_out.dtype)  # [B, T, C, d]
        
        for gname, ch_indices in self.modality_groups.items():
            n_ch = len(ch_indices)
            # Extract channels belonging to this modality: [B, T, n_ch, d]
            group_x = x[:, :, ch_indices, :]
            # Reshape for batched temporal attention: [B*n_ch, T, d]
            group_x_flat = group_x.transpose(1, 2).contiguous().view(B * n_ch, T, d)
            
            if padding_mask is not None:
                # Convert boolean padding mask to float mask [0.0, -inf] 
                # to match the causal_mask type and resolve numerical instability.
                group_padding = padding_mask.repeat_interleave(n_ch, dim=0)
                group_padding_float = torch.zeros_like(group_padding, dtype=x.dtype).masked_fill(group_padding, float('-inf'))
            else:
                group_padding_float = None
            
            group_out, _ = self.temporal_attns[gname](
                group_x_flat, group_x_flat, group_x_flat,
                attn_mask=causal_mask,
                key_padding_mask=group_padding_float,
                is_causal=True if causal_mask is not None else False
            )
            # Back to [B, T, n_ch, d]
            group_out = group_out.view(B, n_ch, T, d).transpose(1, 2).contiguous()
            
            # Scatter back into full temporal output.
            # We use .to(temporal_out.dtype) to be robust against AMP dtype mismatches.
            temporal_out[:, :, ch_indices, :] = group_out.to(temporal_out.dtype)
        
        # 3. Fusion and Residuals
        attn_out = self.dropout(spatial_out + temporal_out)
        x = self.norm1(x + attn_out)
        
        # 4. FFN
        ffn_out = self.dropout(self.ffn(x))
        x = self.norm2(x + ffn_out)
        
        return x

class AttentionPooling(nn.Module):
    def __init__(self, d_model):
        super().__init__()
        self.attention = nn.Sequential(
            nn.Linear(d_model, d_model // 2),
            nn.Tanh(),
            nn.Linear(d_model // 2, 1)
        )

    def forward(self, x):
        # x shape: [B, C, d_model]
        attn_scores = self.attention(x) # [B, C, 1]
        
        # Stability clamp for float16 (Half) precision on GPU
        attn_scores = torch.clamp(attn_scores, min=-65500, max=65500)
        
        attn_weights = torch.softmax(attn_scores, dim=1) 
        pooled = torch.sum(x * attn_weights, dim=1) # [B, d_model]
        return pooled, attn_weights

class SpatioTemporalTransformerNetwork3(nn.Module):
    """Modality-grouped spatio-temporal transformer.
    
    Classifies sensor channels into modality groups and gives each group
    its own temporal attention MHA while keeping spatial attention shared.
    """
    
    MODALITY_KEYWORDS = {
        'EMG':        lambda ch: '_EMG' in ch,
        'IMU_Accel':  lambda ch: ('_IMU' in ch and any(k in ch for k in ['ax', 'ay', 'az', 'Accel'])) or '_SVM' in ch,
        'IMU_Orient': lambda ch: '_IMU' in ch and any(k in ch for k in ['roll', 'pitch', 'yaw', 'Orient', 'rad']),
    }
    
    def __init__(self, channel_indices: dict, d_model: int, nhead_spatial: int, 
                 nhead_temporal: int, num_layers: int, dim_feedforward: int, dropout_rate: float,
                 use_checkpointing: bool = False):
        super().__init__()
        
        self.d_model = d_model
        self.use_checkpointing = use_checkpointing
        self.num_layers = num_layers
        
        # 1. Channel Projections
        self.channel_projections = nn.ModuleDict()
        self.channel_names = sorted(list(channel_indices.keys()))
        self.channel_indices = channel_indices
        
        for ch in self.channel_names:
            safe_name = ch.replace(' ', '_').replace('(', '').replace(')', '')
            self.channel_projections[safe_name] = nn.Linear(len(channel_indices[ch]), d_model)
            
        num_channels = len(self.channel_names)
        
        # 2. Build modality groups: map group_name -> list of positions in channel dim
        self.modality_groups = self._build_modality_groups(self.channel_names)
        group_summary = {g: len(idxs) for g, idxs in self.modality_groups.items()}
        print(f"[{self.__class__.__name__}] Modality groups: {group_summary}")
        
        # 3. Positional Encodings
        self.spatial_pos_embedding = nn.Parameter(torch.randn(1, 1, num_channels, d_model))
        self.temporal_pos_encoder = PositionalEncoding(d_model)
        
        # 4. Modality-Grouped Spatio-Temporal Blocks
        self.blocks = nn.ModuleList([
            ModalityGroupedSTBlock(d_model, nhead_spatial, nhead_temporal, dim_feedforward, dropout_rate, self.modality_groups)
            for _ in range(self.num_layers)
        ])
        
        # 5. Late Pooling & Regressor
        self.attention_pooling = AttentionPooling(d_model)
        self.fc = nn.Linear(d_model, 1)
    
    @classmethod
    def _build_modality_groups(cls, channel_names):
        """Classify each channel position into a modality group."""
        groups = defaultdict(list)
        for idx, ch in enumerate(channel_names):
            assigned = False
            for group_name, test_fn in cls.MODALITY_KEYWORDS.items():
                if test_fn(ch):
                    groups[group_name].append(idx)
                    assigned = True
                    break
            if not assigned:
                groups['Other'].append(idx)
        return dict(groups)

    def forward(self, x, lengths):
        B, T, F = x.shape
        
        # 1. Project individual channels
        channel_tokens = []
        for ch in self.channel_names:
            safe_name = ch.replace(' ', '_').replace('(', '').replace(')', '')
            idxs = self.channel_indices[ch]
            ch_x = x[:, :, idxs]
            proj = self.channel_projections[safe_name](ch_x)
            channel_tokens.append(proj)
            
        x_st = torch.stack(channel_tokens, dim=2)  # [B, T, C, d_model]
        
        # 2. Add Spatial Positional Embedding
        x_st = x_st + self.spatial_pos_embedding
        
        # 3. Add Temporal Positional Encoding
        dummy = torch.zeros(1, T, self.d_model, device=x.device)
        t_pe = self.temporal_pos_encoder(dummy)
        x_st = x_st + t_pe.unsqueeze(2)
        
        # 4. Create Unified Attention Mask
        # Combine causal mask and padding mask into a single float mask [-inf, 0.0]
        # to avoid UserWarnings and numerical instability in newer PyTorch versions.
        causal_mask = nn.Transformer.generate_square_subsequent_mask(T, device=x.device)
        padding_mask = torch.arange(T, device=x.device).expand(B, T) >= lengths.unsqueeze(1)
        
        # Convert boolean padding mask to float mask [0.0, -inf]
        # (B, 1, 1, T) for broadcasting over heads and queries
        padding_mask_float = torch.zeros((B, T), device=x.device).masked_fill(padding_mask, float('-inf'))
        
        # 5. Pass through Modality-Grouped Blocks
        if self.use_checkpointing and self.training:
            for block in self.blocks:
                def create_custom_forward(module):
                    def custom_forward(src, c_mask, p_mask):
                        # We pass padding_mask separately if the block logic needs it, 
                        # but MHA inside the block will use the unified mask logic.
                        return module(src, causal_mask=c_mask, padding_mask=p_mask)
                    return custom_forward
                
                x_st = torch.utils.checkpoint.checkpoint(
                    create_custom_forward(block), 
                    x_st, causal_mask, padding_mask, # Keep original masks for block API compatibility
                    use_reentrant=False
                )
        else:
            for block in self.blocks:
                x_st = block(x_st, causal_mask=causal_mask, padding_mask=padding_mask)
        
        # 6. Extract Last Valid Timestep
        # Ensure lengths is at least 1 to prevent index -1 issues or NaN pooling
        safe_lengths = torch.clamp(lengths, min=1)
        last_tokens = x_st[torch.arange(B), safe_lengths - 1, :, :]
        
        # 7. Late Attention Pooling over Channels
        pooled_out, _ = self.attention_pooling(last_tokens)
        
        # 8. Final prediction
        out = self.fc(pooled_out)
        return out


class SpatioTemporalTransformerRegressor3:
    def __init__(self, 
                 d_model: int = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['d_model'],
                 nhead_spatial: int = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['nhead_spatial'],
                 num_layers: int = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG.get('num_layers', 3),
                 nhead_temporal: int = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['nhead_temporal'],
                 dim_feedforward: int = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['dim_feedforward'],
                 dropout_rate: float = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['dropout_rate'],
                 learning_rate: float = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['learning_rate'],
                 weight_decay: float = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG.get('weight_decay', 0.0),
                 batch_size: int = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['batch_size'],
                 epochs: int = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['epochs'],
                 validation_split: float = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG.get('validation_split', 0.2),
                 early_stopping_patience: int = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG.get('early_stopping_patience', 10),
                 scheduler_patience: int = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG.get('scheduler_patience', 5),
                 scheduler_factor: float = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG.get('scheduler_factor', 0.5),
                 emg_window_size_sec: float = FEATURE_CONFIG['emg_window_size_sec'],
                 imu_window_size_sec: float = FEATURE_CONFIG['imu_window_size_sec'],
                 window_step_sec: float = FEATURE_CONFIG['window_step_sec'],
                 loss_type: str = GLOBAL_LOSS_FUNCTION,
                 class_balanced_loss: bool = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG.get('class_balanced_loss', False),
                 use_checkpointing: bool = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG.get('use_checkpointing', True),
                 use_amp: bool = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG.get('use_amp', True),
                 random_state: int = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['random_state'],
                 max_seq_len: int = None,
                 scheduler: dict = None,
                 augmentation_config: dict = None,
                 **kwargs):
                 
        # Resolve num_layers with fallback for explicit spatial/temporal layers in kwargs
        spatial_val = kwargs.get('num_layers_spatial')
        temporal_val = kwargs.get('num_layers_temporal')
        if spatial_val is not None or temporal_val is not None:
            num_layers = max(spatial_val or 1, temporal_val or 1)

        self.d_model = d_model
        self.nhead_spatial = nhead_spatial
        self.num_layers = num_layers
        # Backward compatibility alias fields
        self.num_layers_spatial = num_layers
        self.num_layers_temporal = num_layers
        self.nhead_temporal = nhead_temporal
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
        self.class_balanced_loss = class_balanced_loss
        self.use_checkpointing = use_checkpointing
        self.use_amp = use_amp
        self.random_state = random_state
        self.max_seq_len = max_seq_len
        self.augmentation_config = augmentation_config if augmentation_config is not None else AUGMENTATION_CONFIG
        self.scheduler_config_dict = scheduler
        
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
                channel = 'Anthropometrics'
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
        
        train_lengths = [len(seq) for seq in sequences_train]
        from collections import Counter
        counts = Counter(train_lengths)
        min_count = max(2, int(len(sequences_train) * 0.005))
        valid_lengths = [l for l, c in counts.items() if c >= min_count]
        self.max_seq_len = max(valid_lengths) if valid_lengths else max(train_lengths)
        print(f"[{self.__class__.__name__}] Capping sequence length at {self.max_seq_len} based on sample distribution (min_count={min_count}).")
        
        sequences_train = [seq[:self.max_seq_len] for seq in sequences_train]
        sequences_val = [seq[:self.max_seq_len] for seq in sequences_val]
        
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
        
        flat_data_train = []
        for seq in sequences_train:
            seq_arr = np.array([[w.get(k, 0.0) for k in self.feature_names] for w in seq])
            flat_data_train.append(seq_arr)
            
        all_features_train = self._sanitise(np.vstack(flat_data_train))
        self.scaler.fit(all_features_train)
        
        # Floor standard deviation scale to prevent dividing by near-zero values (feature explosion)
        if hasattr(self.scaler, "scale_") and self.scaler.scale_ is not None:
            self.scaler.scale_ = np.maximum(self.scaler.scale_, 1e-4)
            
        scaled_seqs_train = []
        for arr in flat_data_train:
            scaled_arr = self.scaler.transform(self._sanitise(arr))
            scaled_arr = np.clip(scaled_arr, -15.0, 15.0).astype(np.float32)
            scaled_seqs_train.append(scaled_arr) 
            
        scaled_seqs_val = []
        for seq in sequences_val:
            seq_arr = np.array([[w.get(k, 0.0) for k in self.feature_names] for w in seq])
            scaled_arr = self.scaler.transform(self._sanitise(seq_arr))
            scaled_arr = np.clip(scaled_arr, -15.0, 15.0).astype(np.float32)
            scaled_seqs_val.append(scaled_arr)
            
        y_np_train = y_train.values.astype(np.float32)
        
        augmenter = SequenceAugmenter(config=self.augmentation_config)
        participant_ids_train = X_train['subject'].values if 'subject' in X_train.columns else None
        scaled_seqs_train, y_np_train = augmenter.augment_dataset(
            scaled_seqs_train, y_np_train, participant_ids=participant_ids_train
        )
        
        scaled_seqs_train = [torch.from_numpy(a) for a in scaled_seqs_train]
        y_tensor_train = torch.from_numpy(y_np_train).unsqueeze(1)
        
        scaled_seqs_val = [torch.from_numpy(a) for a in scaled_seqs_val]
        y_tensor_val = torch.from_numpy(y_val.values.astype(np.float32)).unsqueeze(1)
        
        self.train_samples = len(X_train)
        self.val_samples = len(X_val)
        
        # Init Model (Modality-Grouped Transformer)
        self.model = SpatioTemporalTransformerNetwork3(
            channel_indices=self.channel_indices,
            d_model=self.d_model, 
            nhead_spatial=self.nhead_spatial, 
            nhead_temporal=self.nhead_temporal,
            num_layers=self.num_layers,
            dim_feedforward=self.dim_feedforward, 
            dropout_rate=self.dropout_rate,
            use_checkpointing=self.use_checkpointing
        ).to(self.device)
        
        optimizer = optim.AdamW(self.model.parameters(), lr=self.learning_rate, weight_decay=self.weight_decay)
        
        # Class-balanced (inverse-frequency) per-sample loss weighting to prevent the
        # 0kg-dominated gain bias. When enabled, the base criterion is computed per-element
        # and reduced with per-class weights derived from the (post-augmentation) train labels.
        self._balanced = bool(getattr(self, 'class_balanced_loss', False))
        _reduction = 'none' if self._balanced else 'mean'
        if self.loss_type == 'mae':
            criterion = nn.L1Loss(reduction=_reduction)
        else:
            criterion = nn.MSELoss(reduction=_reduction)

        if self._balanced:
            _vals, _counts = np.unique(np.round(y_np_train, 2), return_counts=True)
            _n_cls = len(_vals); _N = int(_counts.sum())
            _cw = _N / (_n_cls * _counts)  # sklearn 'balanced'; mean per-sample weight == 1
            self._cls_vals = torch.tensor(_vals, dtype=torch.float32, device=self.device)
            self._cls_w = torch.tensor(_cw, dtype=torch.float32, device=self.device)
            print(f"[class-balanced loss] {_n_cls} weight classes; per-class loss weights: "
                  f"{ {round(float(v), 2): round(float(w), 3) for v, w in zip(_vals, _cw)} }")

        def _reduce(per_elem, batch_y):
            """Reduce loss to a scalar; inverse-frequency weighted when balancing is on."""
            if not self._balanced:
                return per_elem  # criterion already returned the mean
            per = per_elem.view(per_elem.size(0), -1).mean(dim=1)            # (B,)
            idx = torch.argmin((batch_y.view(-1, 1) - self._cls_vals.view(1, -1)).abs(), dim=1)
            w = self._cls_w[idx]                                            # (B,)
            return (per * w).sum() / w.sum().clamp_min(1e-8)
        
        dataset_train = SequenceDataset(scaled_seqs_train, y_tensor_train)
        loader_train = DataLoader(dataset_train, batch_size=self.batch_size, shuffle=True, collate_fn=pad_collate_fn)
        
        # Read from constructor-passed scheduler config first, then from global fallback
        scheduler_config = getattr(self, 'scheduler_config_dict', None)
        if scheduler_config is None:
            from model.config_model import SPATIO_TEMPORAL_TRANSFORMER3_CONFIG
            scheduler_config = SPATIO_TEMPORAL_TRANSFORMER3_CONFIG.get('scheduler', {'type': 'ReduceLROnPlateau'})
        
        if scheduler_config.get('type') == 'OneCycleLR':
            total_steps = self.epochs * len(loader_train)
            scheduler = optim.lr_scheduler.OneCycleLR(
                optimizer,
                max_lr=scheduler_config.get('max_lr', self.learning_rate),
                total_steps=total_steps,
                pct_start=scheduler_config.get('pct_start', 0.1),
                anneal_strategy='cos'
            )
        elif scheduler_config.get('type') in ['WarmupCosine', 'WarmupReduceLROnPlateau', 'WarmupPlateau']:
            from model.model_archs.lr_scheduler import WarmupScheduler
            target_type = 'cosine' if 'Cosine' in scheduler_config.get('type') else 'plateau'
            warmup_epochs = scheduler_config.get('warmup_epochs', 5)
            min_lr = scheduler_config.get('min_lr', 1e-6)
            scheduler = WarmupScheduler(
                optimizer,
                warmup_epochs=warmup_epochs,
                initial_lr=self.learning_rate,
                total_epochs=self.epochs,
                target_scheduler_type=target_type,
                min_lr=min_lr,
                patience=self.scheduler_patience,
                factor=self.scheduler_factor
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
        
        amp_device = 'cuda' if self.device.type == 'cuda' else 'cpu'
        scaler = torch.amp.GradScaler(amp_device, enabled=(self.use_amp and self.device.type == 'cuda'))
        
        with tqdm(range(self.epochs), desc="Training Modality-Grouped ST-Transformer", unit="epoch") as pbar:
            for epoch in pbar:
                self.model.train()
                running_loss = 0.0
                n_train_samples = 0
                for batch_x, batch_y, lengths in loader_train:
                    batch_x, batch_y = batch_x.to(self.device), batch_y.to(self.device)
                    lengths = lengths.to(self.device)
                    
                    optimizer.zero_grad()
                    
                    with torch.amp.autocast(self.device.type, enabled=(self.use_amp and self.device.type == 'cuda')):
                        outputs = self.model(batch_x, lengths)
                        loss = _reduce(criterion(outputs, batch_y), batch_y)

                    if not torch.isfinite(loss):
                        optimizer.zero_grad()
                        continue
                    
                    scaler.scale(loss).backward()
                    scaler.unscale_(optimizer)
                    torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=1.0)
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
                    with torch.amp.autocast(self.device.type, enabled=(self.use_amp and self.device.type == 'cuda')):
                        for batch_x, batch_y, lengths in loader_val:
                            batch_x, batch_y = batch_x.to(self.device), batch_y.to(self.device)
                            lengths = lengths.to(self.device)
                            outputs = self.model(batch_x, lengths)
                            loss = _reduce(criterion(outputs, batch_y), batch_y)
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
            seq_arr = np.array([[w.get(k, 0.0) for k in self.feature_names] for w in seq])
            scaled_arr = self.scaler.transform(self._sanitise(seq_arr))
            scaled_arr = np.clip(scaled_arr, -15.0, 15.0).astype(np.float32)
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
                'num_layers': self.num_layers,
                # Include spatial/temporal aliases for loading in old code versions
                'num_layers_spatial': self.num_layers,
                'num_layers_temporal': self.num_layers,
                'nhead_temporal': self.nhead_temporal,
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
                'max_seq_len': getattr(self, 'max_seq_len', None),
                'augmentation_config': self.augmentation_config,
                'scheduler': getattr(self, 'scheduler_config_dict', None)
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
        config = copy.deepcopy(state['config'])
        
        # Load spatial/temporal parameters gracefully if loading an old model
        if 'num_layers' not in config:
            config['num_layers'] = max(config.get('num_layers_spatial', 3), config.get('num_layers_temporal', 3))
            
        regressor = cls(**config)
        regressor.scaler = state['scaler']
        regressor.feature_names = state['feature_names']
        regressor.channel_indices = state['channel_indices']
        
        regressor.model = SpatioTemporalTransformerNetwork3(
            channel_indices=state['channel_indices'],
            d_model=state['config']['d_model'], 
            nhead_spatial=state['config']['nhead_spatial'], 
            nhead_temporal=state['config']['nhead_temporal'],
            num_layers=regressor.num_layers,
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
        plotting_utils.plot_regression_results(y_test, y_pred, save_path, model_name="Modality-Grouped ST-Transformer")

    def plot_loss(self, save_path: str | Path):
        plotting_utils.plot_training_loss(self.loss_history, save_path, model_name="Modality-Grouped ST-Transformer")

    def permutation_importance(self, X_test: pd.DataFrame, y_test: pd.Series, n_repeats: int = 5, importance_type: str = 'channel'):
        # Implementation is identical to original model
        import copy
        from tqdm import tqdm
        from collections import defaultdict
        
        baseline_preds = self.predict(X_test)
        baseline_mse = mean_squared_error(y_test, baseline_preds)
        
        groups = defaultdict(list)
        for i, fname in enumerate(self.feature_names):
            if importance_type == 'channel':
                if '_EMG_' in fname:
                    group = fname.split('_EMG_')[0] + "_EMG"
                elif '_IMU_' in fname:
                    parts = fname.split('_IMU_')
                    group = parts[0] + "_IMU"
                elif '_SVM_' in fname:
                    group = fname.split('_SVM_')[0] + "_SVM"
                else:
                    group = 'Anthropometrics'
                groups[group].append(i)
                
            elif importance_type == 'feature':
                parts = fname.rsplit('_', 1)
                feat_suffix = parts[-1] if len(parts) > 1 else "raw"
                if '_EMG_' in fname:
                    feat_type = f"EMG_{feat_suffix}"
                elif '_IMU_' in fname or '_SVM_' in fname:
                    feat_type = f"IMU_{feat_suffix}"
                else:
                    feat_type = f"Anthro_{fname}"
                groups[feat_type].append(i)
            elif importance_type == 'individual_feature':
                groups[fname].append(i)
            else:
                raise ValueError("importance_type must be 'channel', 'feature', or 'individual_feature'")
                
        group_importances = {}
        group_names = list(groups.keys())
        
        orig_sequences = self._extract_sequences(X_test)
        if getattr(self, 'max_seq_len', None) is not None:
            orig_sequences = [seq[:self.max_seq_len] for seq in orig_sequences]
            
        for group in tqdm(group_names, desc=f"Permutation Importance ({importance_type})"):
            feat_idxs = groups[group]
            feat_names = [self.feature_names[i] for i in feat_idxs]
            
            scores = []
            for _ in range(n_repeats):
                all_values = []
                for seq in orig_sequences:
                    for window in seq:
                        vals = [window.get(k, 0.0) for k in feat_names]
                        all_values.append(vals)
                
                all_values_arr = np.array(all_values)
                np.random.shuffle(all_values_arr)
                
                perm_sequences = []
                val_idx = 0
                for i in range(len(orig_sequences)):
                    new_seq = []
                    for j in range(len(orig_sequences[i])):
                        new_window = orig_sequences[i][j].copy()
                        for k_idx, k_name in enumerate(feat_names):
                            new_window[k_name] = all_values_arr[val_idx, k_idx]
                        new_seq.append(new_window)
                        val_idx += 1
                    perm_sequences.append(new_seq)
                
                X_perm = X_test.copy()
                X_perm['sequence_dicts'] = perm_sequences
                
                perm_preds = self.predict(X_perm)
                perm_mse = mean_squared_error(y_test, perm_preds)
                scores.append(perm_mse - baseline_mse)
            
            group_importances[group] = np.mean(scores)
            
        return group_importances
