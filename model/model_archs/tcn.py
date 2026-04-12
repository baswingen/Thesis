import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.nn.utils.rnn as rnn_utils
from torch.nn.utils import weight_norm
from torch.utils.data import DataLoader, Dataset
import pandas as pd
import numpy as np
import sys
from pathlib import Path
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import copy
from tqdm import tqdm

# Add project root to sys.path
sys.path.append(str(Path(__file__).resolve().parent.parent.parent))

from model.config_model import TCN_CONFIG, GLOBAL_LOSS_FUNCTION, AUGMENTATION_CONFIG, GLOBAL_BALANCE_WEIGHTS
from model import plotting_utils
from model.data_augmentation import SequenceAugmenter
from sklearn.utils.class_weight import compute_sample_weight

# ---------------------------------------------------------------------------
# TCN Modules
# ---------------------------------------------------------------------------

class Chomp1d(nn.Module):
    """Slices the output to ensure strict causality. Removes padding from the right."""
    def __init__(self, chomp_size):
        super(Chomp1d, self).__init__()
        self.chomp_size = chomp_size

    def forward(self, x):
        return x[:, :, :-self.chomp_size].contiguous()

class TemporalBlock(nn.Module):
    """A building block for the TCN with causal dilated convolutions and residual connections."""
    def __init__(self, n_inputs, n_outputs, kernel_size, stride, dilation, padding, dropout=0.2):
        super(TemporalBlock, self).__init__()
        self.conv1 = weight_norm(nn.Conv1d(n_inputs, n_outputs, kernel_size,
                                           stride=stride, padding=padding, dilation=dilation))
        self.chomp1 = Chomp1d(padding)
        self.relu1 = nn.ReLU()
        self.dropout1 = nn.Dropout(dropout)

        self.conv2 = weight_norm(nn.Conv1d(n_outputs, n_outputs, kernel_size,
                                           stride=stride, padding=padding, dilation=dilation))
        self.chomp2 = Chomp1d(padding)
        self.relu2 = nn.ReLU()
        self.dropout2 = nn.Dropout(dropout)

        self.net = nn.Sequential(self.conv1, self.chomp1, self.relu1, self.dropout1,
                                 self.conv2, self.chomp2, self.relu2, self.dropout2)
        self.downsample = nn.Conv1d(n_inputs, n_outputs, 1) if n_inputs != n_outputs else None
        self.relu = nn.ReLU()
        self.init_weights()

    def init_weights(self):
        self.conv1.weight.data.normal_(0, 0.01)
        self.conv2.weight.data.normal_(0, 0.01)
        if self.downsample is not None:
            self.downsample.weight.data.normal_(0, 0.01)

    def forward(self, x):
        out = self.net(x)
        res = x if self.downsample is None else self.downsample(x)
        return self.relu(out + res)

class TemporalConvNet(nn.Module):
    """The central strictly-causal TCN core."""
    def __init__(self, num_inputs, num_channels, kernel_size=2, dropout=0.2):
        super(TemporalConvNet, self).__init__()
        layers = []
        num_levels = len(num_channels)
        for i in range(num_levels):
            dilation_size = 2 ** i
            in_channels = num_inputs if i == 0 else num_channels[i-1]
            out_channels = num_channels[i]
            layers += [TemporalBlock(in_channels, out_channels, kernel_size, stride=1, dilation=dilation_size,
                                     padding=(kernel_size-1) * dilation_size, dropout=dropout)]

        self.network = nn.Sequential(*layers)

    def forward(self, x):
        return self.network(x)

# ---------------------------------------------------------------------------
# Full CNN-TCN Network
# ---------------------------------------------------------------------------

class TCNNetwork(nn.Module):
    def __init__(self, n_channels: int, cnn_filters: list[int],
                 cnn_kernel_sizes: list[int], pool_size: int,
                 tcn_channels: list[int], tcn_kernel_size: int,
                 dropout_rate: float):
        super().__init__()
        
        # CNN Extractor (downsampling via strided pooling)
        layers = []
        in_ch = n_channels
        for filters, ks in zip(cnn_filters, cnn_kernel_sizes):
            padding = ks // 2
            layers.extend([
                nn.Conv1d(in_ch, filters, kernel_size=ks, padding=padding),
                nn.InstanceNorm1d(filters, affine=True),
                nn.ReLU(inplace=False),
                nn.MaxPool1d(kernel_size=pool_size),
                nn.Dropout(dropout_rate),
            ])
            in_ch = filters
        self.cnn = nn.Sequential(*layers)
        
        self.tcn = TemporalConvNet(
            num_inputs=in_ch,
            num_channels=tcn_channels,
            kernel_size=tcn_kernel_size,
            dropout=dropout_rate
        )
        self.dropout = nn.Dropout(dropout_rate)
        self.fc = nn.Linear(tcn_channels[-1], 1)

        self._pool_size = pool_size
        self._n_blocks = len(cnn_filters)

    def _adjust_lengths(self, lengths):
        adj = lengths.float()
        for _ in range(self._n_blocks):
            adj = torch.floor(adj / self._pool_size)
        return adj.long().clamp(min=1)

    def forward(self, x, lengths):
        # x is (B, C, T)
        cnn_out = self.cnn(x)
        
        # Robustness step: remove overall amplitude level to stabilize subject variability
        cnn_out = F.normalize(cnn_out, p=2, dim=1) 
        
        # tcn_out is (B, tcn_channels[-1], reduced_T)
        tcn_out = self.tcn(cnn_out)
        
        # We must gather the valid "last" timestep for each padded sequence
        adj_lengths = self._adjust_lengths(lengths).clamp(max=tcn_out.size(-1)).cpu()
        batch_size = tcn_out.size(0)
        
        # Create an index tensor of shape (B, tcn_channels[-1], 1) representing the target time slice
        idx = (adj_lengths - 1).view(-1, 1, 1).expand(batch_size, tcn_out.size(1), 1).to(x.device)
        
        # Gather extracting the valid final vector: (B, tcn_channels[-1])
        last_out = tcn_out.gather(2, idx).squeeze(2)
        
        last_out = self.dropout(last_out)
        out = self.fc(last_out)
        return out


# ---------------------------------------------------------------------------
# Dataset & Collate (same as CNN-LSTM)
# ---------------------------------------------------------------------------

class RawSegmentDataset(Dataset):
    def __init__(self, segments, labels, participant_ids=None, sample_weights=None):
        self.segments = segments          
        self.labels = labels              
        self.participant_ids = participant_ids  
        self.sample_weights = sample_weights

    def __len__(self): return len(self.segments)

    def __getitem__(self, idx):
        pid = self.participant_ids[idx] if self.participant_ids is not None else ""
        weight = self.sample_weights[idx] if self.sample_weights is not None else torch.tensor([1.0], dtype=torch.float32)
        return self.segments[idx], self.labels[idx], pid, weight

def raw_pad_collate_fn(batch):
    segments, labels, pids, weights = zip(*batch)
    lengths = torch.tensor([seg.shape[0] for seg in segments])
    padded = rnn_utils.pad_sequence(segments, batch_first=True, padding_value=0.0)
    labels = torch.stack(labels)
    weights = torch.stack(weights)
    return padded, labels, lengths, list(pids), weights

# ---------------------------------------------------------------------------
# Scikit-learn-style regressor wrapper
# ---------------------------------------------------------------------------

class TCNRegressor:
    def __init__(self,
                 cnn_filters: list[int] = TCN_CONFIG['cnn_filters'],
                 cnn_kernel_sizes: list[int] = TCN_CONFIG['cnn_kernel_sizes'],
                 pool_size: int = TCN_CONFIG['pool_size'],
                 tcn_channels: list[int] = TCN_CONFIG['tcn_channels'],
                 tcn_kernel_size: int = TCN_CONFIG['tcn_kernel_size'],
                 dropout_rate: float = TCN_CONFIG['dropout_rate'],
                 learning_rate: float = TCN_CONFIG['learning_rate'],
                 weight_decay: float = TCN_CONFIG.get('weight_decay', 1e-4),
                 batch_size: int = TCN_CONFIG['batch_size'],
                 epochs: int = TCN_CONFIG['epochs'],
                 validation_split: float = TCN_CONFIG.get('validation_split', 0.2),
                 early_stopping_patience: int = TCN_CONFIG.get('early_stopping_patience', 50),
                 scheduler_T_0: int = TCN_CONFIG.get('scheduler_T_0', 50),
                 scheduler_T_mult: int = TCN_CONFIG.get('scheduler_T_mult', 2),
                 loss_type: str = GLOBAL_LOSS_FUNCTION,
                 balance_weights: bool = TCN_CONFIG.get('balance_weights', GLOBAL_BALANCE_WEIGHTS),
                 random_state: int = TCN_CONFIG['random_state']):

        self.cnn_filters = list(cnn_filters)
        self.cnn_kernel_sizes = list(cnn_kernel_sizes)
        self.pool_size = pool_size
        self.tcn_channels = list(tcn_channels)
        self.tcn_kernel_size = tcn_kernel_size
        self.dropout_rate = dropout_rate
        self.learning_rate = learning_rate
        self.weight_decay = weight_decay
        self.batch_size = batch_size
        self.epochs = epochs
        self.validation_split = validation_split
        self.early_stopping_patience = early_stopping_patience
        self.scheduler_T_0 = scheduler_T_0
        self.scheduler_T_mult = scheduler_T_mult
        self.loss_type = loss_type.lower()
        self.balance_weights = balance_weights
        self.random_state = random_state

        self.loss_history = {"train": [], "val": []}
        self.train_samples = 0
        self.val_samples = 0

        torch.manual_seed(self.random_state)

        self.scaler = StandardScaler()
        self.model = None
        self.n_channels = None

        if torch.cuda.is_available(): self.device = torch.device("cuda")
        elif torch.backends.mps.is_available(): self.device = torch.device("mps")
        else: self.device = torch.device("cpu")
        print(f"[{self.__class__.__name__}] Using device: {self.device}")

    @staticmethod
    def _extract_raw_segments(X: pd.DataFrame) -> list[np.ndarray]:
        if 'raw_segment' not in X.columns:
            raise ValueError("Data expects a 'raw_segment' column.")
        return X['raw_segment'].tolist()

    @staticmethod
    def _sanitise(arr: np.ndarray, clip: float = 1e9) -> np.ndarray:
        return np.clip(np.where(np.isfinite(arr), arr, 0.0), -clip, clip)

    def _scale_segments(self, segments: list[np.ndarray], fit: bool = False) -> list[np.ndarray]:
        if fit:
            all_data = self._sanitise(np.vstack(segments))
            self.scaler.fit(all_data)
        return [self.scaler.transform(self._sanitise(seg)).astype(np.float32) for seg in segments]

    def fit(self, X: pd.DataFrame, y: pd.Series, sample_weight=None):
        from sklearn.model_selection import train_test_split
        
        stratify = X["label"] if "label" in X.columns else None
        X_train, X_val, y_train, y_val = train_test_split(
            X, y, test_size=self.validation_split,
            random_state=self.random_state, stratify=stratify,
        )

        segs_train = self._extract_raw_segments(X_train)
        segs_val = self._extract_raw_segments(X_val)
        self.n_channels = segs_train[0].shape[1]

        scaled_train = self._scale_segments(segs_train, fit=True)
        scaled_val = self._scale_segments(segs_val, fit=False)
        y_np_train = y_train.values.astype(np.float32)

        augmenter = SequenceAugmenter(config=AUGMENTATION_CONFIG)
        participant_ids_train = X_train['subject'].values if 'subject' in X_train.columns else None
        scaled_train, y_np_train = augmenter.augment_dataset(
            scaled_train, y_np_train, participant_ids=participant_ids_train
        )

        train_tensors = [torch.from_numpy(s) for s in scaled_train]
        y_tensor_train = torch.from_numpy(y_np_train).unsqueeze(1)
        val_tensors = [torch.from_numpy(s) for s in scaled_val]
        y_tensor_val = torch.from_numpy(y_val.values.astype(np.float32)).unsqueeze(1)

        self.train_samples = len(X_train)
        self.val_samples = len(X_val)

        train_weights_tensor = None
        if self.balance_weights:
            weights_np = compute_sample_weight(class_weight='balanced', y=y_np_train.flatten())
            train_weights_tensor = torch.from_numpy(weights_np.astype(np.float32)).unsqueeze(1)

        self.model = TCNNetwork(
            n_channels=self.n_channels,
            cnn_filters=self.cnn_filters,
            cnn_kernel_sizes=self.cnn_kernel_sizes,
            pool_size=self.pool_size,
            tcn_channels=self.tcn_channels,
            tcn_kernel_size=self.tcn_kernel_size,
            dropout_rate=self.dropout_rate,
        ).to(self.device)

        optimizer = optim.AdamW(self.model.parameters(), lr=self.learning_rate, weight_decay=self.weight_decay)
        scheduler = optim.lr_scheduler.CosineAnnealingWarmRestarts(
            optimizer, T_0=self.scheduler_T_0, T_mult=self.scheduler_T_mult, eta_min=1e-6
        )

        criterion = nn.L1Loss() if self.loss_type == 'mae' else nn.MSELoss()

        dataset_train = RawSegmentDataset(train_tensors, y_tensor_train, sample_weights=train_weights_tensor)
        loader_train = DataLoader(dataset_train, batch_size=self.batch_size, shuffle=True, collate_fn=raw_pad_collate_fn)

        dataset_val = RawSegmentDataset(val_tensors, y_tensor_val)
        loader_val = DataLoader(dataset_val, batch_size=self.batch_size, shuffle=False, collate_fn=raw_pad_collate_fn)

        best_val_loss = float('inf')
        patience_counter = 0
        best_weights = copy.deepcopy(self.model.state_dict())

        with tqdm(range(self.epochs), desc="Training TCN", unit="epoch") as pbar:
            for epoch in pbar:
                self.model.train()
                running_loss = 0.0
                for batch_x, batch_y, lengths, batch_pids, batch_weights in loader_train:
                    batch_x = batch_x.transpose(1, 2).to(self.device)
                    batch_y = batch_y.to(self.device)
                    lengths = lengths.to(self.device)
                    batch_weights = batch_weights.to(self.device)

                    optimizer.zero_grad()
                    outputs = self.model(batch_x, lengths)
                    
                    if self.balance_weights:
                        criterion.reduction = 'none'
                        loss_unweighted = criterion(outputs, batch_y)
                        loss = (loss_unweighted * batch_weights).mean()
                    else:
                        criterion.reduction = 'mean'
                        loss = criterion(outputs, batch_y)
                        
                    loss.backward()
                    torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=1.0)
                    optimizer.step()

                    running_loss += loss.item() * batch_x.size(0)

                avg_train_loss = running_loss / len(dataset_train)

                self.model.eval()
                val_loss = 0.0
                criterion.reduction = 'mean'
                with torch.no_grad():
                    for batch_x, batch_y, lengths, _, _ in loader_val:
                        batch_x = batch_x.transpose(1, 2).to(self.device)
                        batch_y = batch_y.to(self.device)
                        lengths = lengths.to(self.device)

                        outputs = self.model(batch_x, lengths)
                        loss = criterion(outputs, batch_y)
                        val_loss += loss.item() * batch_x.size(0)

                avg_val_loss = val_loss / len(dataset_val)
                # Include Current LR in progress bar
                current_lr = optimizer.param_groups[0]['lr']
                pbar.set_postfix({"Loss": f"{avg_train_loss:.4f}", "Val Loss": f"{avg_val_loss:.4f}", "LR": f"{current_lr:.1e}"})
                
                # Step the CosineAnnealingWarmRestarts (does not take val_loss)
                scheduler.step()

                self.loss_history["train"].append(avg_train_loss)
                self.loss_history["val"].append(avg_val_loss)

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
        if self.model is None: raise ValueError("Model not fitted.")
        segments = self._extract_raw_segments(X)
        scaled = self._scale_segments(segments, fit=False)
        tensors = [torch.from_numpy(s) for s in scaled]
        dummy_y = torch.zeros((len(tensors), 1))
        dataset = RawSegmentDataset(tensors, dummy_y)
        loader = DataLoader(dataset, batch_size=self.batch_size, shuffle=False, collate_fn=raw_pad_collate_fn)

        self.model.eval()
        all_preds = []
        with torch.no_grad():
            for batch_x, _, lengths, _pids, _ in loader:
                batch_x = batch_x.transpose(1, 2).to(self.device)
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
            f"Mean Absolute Error: {mae:.4f}\nMean Squared Error: {mse:.4f}\n"
            f"Root Mean Squared Error: {rmse:.4f}\nR-squared Score: {r2:.4f}\n"
        )
        return metrics, report_str

    def save(self, filepath: str | Path):
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        state = {
            'model_state': self.model.state_dict(),
            'scaler': self.scaler,
            'n_channels': self.n_channels,
            'config': {
                'cnn_filters': self.cnn_filters, 'cnn_kernel_sizes': self.cnn_kernel_sizes,
                'pool_size': self.pool_size, 'tcn_channels': self.tcn_channels,
                'tcn_kernel_size': self.tcn_kernel_size, 'dropout_rate': self.dropout_rate,
                'learning_rate': self.learning_rate, 'weight_decay': self.weight_decay,
                'batch_size': self.batch_size, 'epochs': self.epochs, 'loss_type': self.loss_type,
                'balance_weights': self.balance_weights, 'validation_split': self.validation_split,
                'early_stopping_patience': self.early_stopping_patience,
                'scheduler_T_0': self.scheduler_T_0, 'scheduler_T_mult': self.scheduler_T_mult,
                'random_state': self.random_state,
            },
            'loss_history': self.loss_history,
        }
        torch.save(state, filepath)

    @classmethod
    def load(cls, filepath: str | Path):
        state = torch.load(filepath, map_location='cpu', weights_only=False)
        regressor = cls(**state['config'])
        regressor.scaler = state['scaler']
        regressor.n_channels = state['n_channels']
        regressor.model = TCNNetwork(
            n_channels=regressor.n_channels, cnn_filters=regressor.cnn_filters,
            cnn_kernel_sizes=regressor.cnn_kernel_sizes, pool_size=regressor.pool_size,
            tcn_channels=regressor.tcn_channels, tcn_kernel_size=regressor.tcn_kernel_size,
            dropout_rate=regressor.dropout_rate
        ).to(regressor.device)
        regressor.model.load_state_dict(state['model_state'])
        regressor.model.eval()
        regressor.loss_history = state.get('loss_history', {"train": [], "val": []})
        return regressor

    def plot_results(self, y_test: pd.Series, y_pred: np.ndarray, save_path: str | Path):
        plotting_utils.plot_regression_results(y_test, y_pred, save_path, model_name="TCN")

    def plot_loss(self, save_path: str | Path):
        plotting_utils.plot_training_loss(self.loss_history, save_path, model_name="TCN")
