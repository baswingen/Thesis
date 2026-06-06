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

from model.config_model import ST_TRANSFORMER_AKSAN_CONFIG, FEATURE_CONFIG, GLOBAL_LOSS_FUNCTION, AUGMENTATION_CONFIG
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

class SensorEmbedding(nn.Module):
    def __init__(self, channel_indices: dict, embed_dim: int):
        super().__init__()
        self.channel_names = sorted(list(channel_indices.keys()))
        self.channel_indices = channel_indices
        self.num_nodes = len(self.channel_names)
        
        self.node_embeddings = nn.ModuleList([
            nn.Linear(len(channel_indices[name]), embed_dim)
            for name in self.channel_names
        ])

    def forward(self, x):
        # x: [B, T, F]
        embedded_nodes = []
        for n, layer in enumerate(self.node_embeddings):
            idxs = self.channel_indices[self.channel_names[n]]
            node_x = x[:, :, idxs]          # [B, T, M_n]
            node_e = layer(node_x)          # [B, T, D]
            embedded_nodes.append(node_e)
            
        x_embed = torch.stack(embedded_nodes, dim=2)
        # [B, T, N, D]
        return x_embed

class TemporalPositionalEncoding(nn.Module):
    def __init__(self, embed_dim, max_len=5000):
        super().__init__()
        pe = torch.zeros(max_len, embed_dim)
        position = torch.arange(0, max_len).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, embed_dim, 2) * (-math.log(10000.0) / embed_dim))
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        self.register_buffer("pe", pe)

    def forward(self, x):
        # x: [B, T, N, D]
        T = x.shape[1]
        return x + self.pe[:T].view(1, T, 1, -1)

class TemporalAttention(nn.Module):
    def __init__(self, num_nodes, embed_dim, num_heads, dropout):
        super().__init__()
        self.num_nodes = num_nodes
        self.attn_layers = nn.ModuleList([
            nn.MultiheadAttention(
                embed_dim=embed_dim,
                num_heads=num_heads,
                dropout=dropout,
                batch_first=True
            )
            for _ in range(num_nodes)
        ])

    def forward(self, x, causal_mask=None, padding_mask=None):
        # x: [B, T, N, D]
        B, T, N, D = x.shape
        outputs = []

        for n in range(N):
            node_x = x[:, :, n, :]  # [B, T, D]
            node_out, _ = self.attn_layers[n](
                query=node_x,
                key=node_x,
                value=node_x,
                attn_mask=causal_mask,
                key_padding_mask=padding_mask,
                is_causal=True if causal_mask is not None else False
            )
            outputs.append(node_out)

        return torch.stack(outputs, dim=2)

class SpatialAttention(nn.Module):
    def __init__(self, embed_dim, num_heads, dropout):
        super().__init__()
        self.attn = nn.MultiheadAttention(
            embed_dim=embed_dim,
            num_heads=num_heads,
            dropout=dropout,
            batch_first=True
        )

    def forward(self, x):
        # x: [B, T, N, D]
        B, T, N, D = x.shape
        x_reshaped = x.reshape(B * T, N, D)

        out, _ = self.attn(
            query=x_reshaped,
            key=x_reshaped,
            value=x_reshaped
        )

        out = out.reshape(B, T, N, D)
        return out

class STAttentionBlock(nn.Module):
    def __init__(self, num_nodes, embed_dim, num_heads, feedforward_dim, dropout):
        super().__init__()
        self.temporal_attn = TemporalAttention(num_nodes, embed_dim, num_heads, dropout)
        self.spatial_attn = SpatialAttention(embed_dim, num_heads, dropout)
        self.norm1 = nn.LayerNorm(embed_dim)
        self.norm2 = nn.LayerNorm(embed_dim)
        self.dropout = nn.Dropout(dropout)
        self.ffn = nn.Sequential(
            nn.Linear(embed_dim, feedforward_dim),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(feedforward_dim, embed_dim)
        )

    def forward(self, x, causal_mask=None, padding_mask=None):
        temporal_out = self.temporal_attn(x, causal_mask=causal_mask, padding_mask=padding_mask)
        spatial_out = self.spatial_attn(x)
        x = self.norm1(x + self.dropout(temporal_out + spatial_out))
        ffn_out = self.ffn(x)
        x = self.norm2(x + self.dropout(ffn_out))
        return x

class STTransformerForEMGIMU(nn.Module):
    def __init__(self, channel_indices, embed_dim=128, num_layers=4, num_heads=4, feedforward_dim=256, dropout=0.1, output_dim=1):
        super().__init__()
        self.embedding = SensorEmbedding(channel_indices, embed_dim)
        self.num_nodes = self.embedding.num_nodes
        self.pos_encoding = TemporalPositionalEncoding(embed_dim)
        self.dropout = nn.Dropout(dropout)
        self.layers = nn.ModuleList([
            STAttentionBlock(
                num_nodes=self.num_nodes,
                embed_dim=embed_dim,
                num_heads=num_heads,
                feedforward_dim=feedforward_dim,
                dropout=dropout
            )
            for _ in range(num_layers)
        ])
        
        self.head = nn.Sequential(
            nn.LayerNorm(embed_dim * self.num_nodes),
            nn.Linear(embed_dim * self.num_nodes, feedforward_dim),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(feedforward_dim, output_dim)
        )

    def forward(self, x, lengths):
        # x: [B, T, F]
        B, T, F = x.shape
        x = self.embedding(x) # [B, T, N, D]
        x = self.pos_encoding(x)
        x = self.dropout(x)
        
        causal_mask = nn.Transformer.generate_square_subsequent_mask(T, device=x.device)
        padding_mask = torch.arange(T, device=x.device).expand(B, T) >= lengths.unsqueeze(1)
        
        for layer in self.layers:
            x = layer(x, causal_mask=causal_mask, padding_mask=padding_mask)
            
        last_tokens = x[torch.arange(B), lengths - 1, :, :] # [B, N, D]
        x_flat = last_tokens.reshape(B, -1) # [B, N*D]
        output = self.head(x_flat)
        return output

class STTransformerAksanRegressor:
    def __init__(self, 
                 embed_dim: int = ST_TRANSFORMER_AKSAN_CONFIG.get('embed_dim', 128),
                 num_layers: int = ST_TRANSFORMER_AKSAN_CONFIG.get('num_layers', 4),
                 num_heads: int = ST_TRANSFORMER_AKSAN_CONFIG.get('num_heads', 4),
                 feedforward_dim: int = ST_TRANSFORMER_AKSAN_CONFIG.get('feedforward_dim', 256),
                 dropout_rate: float = ST_TRANSFORMER_AKSAN_CONFIG.get('dropout', 0.1),
                 learning_rate: float = ST_TRANSFORMER_AKSAN_CONFIG.get('learning_rate', 1e-4),
                 weight_decay: float = ST_TRANSFORMER_AKSAN_CONFIG.get('weight_decay', 1e-4),
                 batch_size: int = ST_TRANSFORMER_AKSAN_CONFIG.get('batch_size', 128),
                 epochs: int = ST_TRANSFORMER_AKSAN_CONFIG.get('epochs', 200),
                 validation_split: float = ST_TRANSFORMER_AKSAN_CONFIG.get('validation_split', 0.1),
                 early_stopping_patience: int = ST_TRANSFORMER_AKSAN_CONFIG.get('early_stopping_patience', 15),
                 scheduler_patience: int = ST_TRANSFORMER_AKSAN_CONFIG.get('scheduler_patience', 7),
                 scheduler_factor: float = ST_TRANSFORMER_AKSAN_CONFIG.get('scheduler_factor', 0.5),
                 loss_type: str = GLOBAL_LOSS_FUNCTION,
                 use_amp: bool = ST_TRANSFORMER_AKSAN_CONFIG.get('use_amp', True),
                 random_state: int = ST_TRANSFORMER_AKSAN_CONFIG.get('random_state', 42),
                 augmentation_config: dict = None,
                 **kwargs):
                 
        self.embed_dim = embed_dim
        self.num_layers = num_layers
        self.num_heads = num_heads
        self.feedforward_dim = feedforward_dim
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
        self.use_amp = use_amp
        self.random_state = random_state
        self.augmentation_config = augmentation_config if augmentation_config is not None else AUGMENTATION_CONFIG
        
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
            
        print(f"[{self.__class__.__name__}] Grouped features into {len(self.channel_indices)} sensor nodes.")

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
        
        self._build_channel_indices()
        
        flat_data_train = []
        for seq in sequences_train:
            seq_arr = np.array([[w.get(k, 0.0) for k in self.feature_names] for w in seq])
            flat_data_train.append(seq_arr)
            
        all_features_train = self._sanitise(np.vstack(flat_data_train))
        self.scaler.fit(all_features_train)
        
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
        
        self.model = STTransformerForEMGIMU(
            channel_indices=self.channel_indices,
            embed_dim=self.embed_dim, 
            num_layers=self.num_layers,
            num_heads=self.num_heads,
            feedforward_dim=self.feedforward_dim, 
            dropout=self.dropout_rate,
            output_dim=1
        ).to(self.device)
        
        optimizer = optim.AdamW(self.model.parameters(), lr=self.learning_rate, weight_decay=self.weight_decay)
        
        if self.loss_type == 'mae':
            criterion = nn.L1Loss()
        else:
            criterion = nn.MSELoss()
        
        dataset_train = SequenceDataset(scaled_seqs_train, y_tensor_train)
        loader_train = DataLoader(dataset_train, batch_size=self.batch_size, shuffle=True, collate_fn=pad_collate_fn)
        
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
        
        with tqdm(range(self.epochs), desc="Training ST-Transformer (Aksan)", unit="epoch") as pbar:
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
                            loss = criterion(outputs, batch_y)
                            val_loss += loss.item() * batch_x.size(0)
                
                avg_val_loss = val_loss / len(dataset_val)
                pbar.set_postfix({"Loss": f"{avg_train_loss:.4f}", "Val Loss": f"{avg_val_loss:.4f}"})
                
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
                'embed_dim': self.embed_dim,
                'num_layers': self.num_layers,
                'num_heads': self.num_heads,
                'feedforward_dim': self.feedforward_dim,
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
                'use_amp': self.use_amp,
                'random_state': self.random_state,
                'max_seq_len': getattr(self, 'max_seq_len', None),
                'augmentation_config': self.augmentation_config
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
        state = torch.load(filepath, map_location='cpu', weights_only=False)
        regressor = cls(**state['config'])
        regressor.scaler = state['scaler']
        regressor.feature_names = state['feature_names']
        regressor.channel_indices = state['channel_indices']
        
        regressor.model = STTransformerForEMGIMU(
            channel_indices=state['channel_indices'],
            embed_dim=state['config']['embed_dim'], 
            num_layers=state['config']['num_layers'],
            num_heads=state['config']['num_heads'],
            feedforward_dim=state['config']['feedforward_dim'],
            dropout=state['config']['dropout_rate'],
            output_dim=1
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
        plotting_utils.plot_regression_results(y_test, y_pred, save_path, model_name="ST-Transformer (Aksan)")

    def plot_loss(self, save_path: str | Path):
        plotting_utils.plot_training_loss(self.loss_history, save_path, model_name="ST-Transformer (Aksan)")

    def permutation_importance(self, X_test: pd.DataFrame, y_test: pd.Series, n_repeats: int = 5, importance_type: str = 'channel'):
        """
        Calculates permutation importance.
        importance_type: 'channel' groups by physical signal source (e.g. Biceps_EMG, IMU1_Accel).
                         'feature' groups by statistical extraction metric (e.g. mean, std) across all channels.
                         'individual_feature' permutes individual features one by one.
        Returns a dictionary mapping the group name to the increase in MSE.
        """
        import copy
        from tqdm import tqdm
        from collections import defaultdict
        
        # 1. Baseline performance
        baseline_preds = self.predict(X_test)
        baseline_mse = mean_squared_error(y_test, baseline_preds)
        
        # 2. Build permutation groups
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
                # Extract feature type (e.g. mean, std) and split by modality
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
        
        # We need the underlying sequences for permutation
        orig_sequences = self._extract_sequences(X_test)
        if getattr(self, 'max_seq_len', None) is not None:
            orig_sequences = [seq[:self.max_seq_len] for seq in orig_sequences]
            
        for group in tqdm(group_names, desc=f"Permutation Importance ({importance_type})"):
            feat_idxs = groups[group]
            feat_names = [self.feature_names[i] for i in feat_idxs]
            
            scores = []
            for _ in range(n_repeats):
                # Permute: we shuffle the values of this channel's features ACROSS the whole test set
                # But we must keep the sequence length structure.
                
                # Gather all values for this channel from all windows in all sequences
                all_values = []
                for seq in orig_sequences:
                    for window in seq:
                        vals = [window.get(k, 0.0) for k in feat_names]
                        all_values.append(vals)
                
                # Shuffle the pool of values
                all_values_arr = np.array(all_values)
                np.random.shuffle(all_values_arr)
                
                # Create permuted sequences
                perm_sequences = []
                val_idx = 0
                for i in range(len(orig_sequences)):
                    new_seq = []
                    for j in range(len(orig_sequences[i])):
                        # Start with original window data
                        new_window = orig_sequences[i][j].copy()
                        # Overwrite features for the target channel with shuffled values
                        for k_idx, k_name in enumerate(feat_names):
                            new_window[k_name] = all_values_arr[val_idx, k_idx]
                        new_seq.append(new_window)
                        val_idx += 1
                    perm_sequences.append(new_seq)
                
                # Create a temporary DataFrame with permuted sequences
                X_perm = X_test.copy()
                X_perm['sequence_dicts'] = perm_sequences
                
                # Predict and measure performance drop
                perm_preds = self.predict(X_perm)
                perm_mse = mean_squared_error(y_test, perm_preds)
                scores.append(perm_mse - baseline_mse)
            
            group_importances[group] = np.mean(scores)
            
        return group_importances

