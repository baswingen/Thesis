import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.nn.utils.rnn as rnn_utils
from torch.utils.data import DataLoader, Dataset
import pandas as pd
import numpy as np
import sys
from pathlib import Path
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score

# Add project root to sys.path
sys.path.append(str(Path(__file__).resolve().parent.parent.parent))

from model.config_model import CNN_LSTM_CONFIG, GLOBAL_LOSS_FUNCTION, AUGMENTATION_CONFIG, GLOBAL_BALANCE_WEIGHTS
from model import plotting_utils
from model.data_augmentation import SequenceAugmenter
from sklearn.utils.class_weight import compute_sample_weight

# ---------------------------------------------------------------------------
# Dataset & collate for variable-length raw segments
# ---------------------------------------------------------------------------

class RawSegmentDataset(Dataset):
    """Holds raw segments as tensors of shape (time_steps, n_channels)."""
    def __init__(self, segments, labels, participant_ids=None, sample_weights=None):
        self.segments = segments          # list of tensors (T_i, C)
        self.labels = labels              # tensor (N, 1)
        self.participant_ids = participant_ids  # list of str or None
        self.sample_weights = sample_weights    # tensor (N, 1) or None

    def __len__(self):
        return len(self.segments)

    def __getitem__(self, idx):
        pid = self.participant_ids[idx] if self.participant_ids is not None else ""
        # Return a weight of 1.0 if no weights are provided
        weight = self.sample_weights[idx] if self.sample_weights is not None else torch.tensor([1.0], dtype=torch.float32)
        return self.segments[idx], self.labels[idx], pid, weight


def raw_pad_collate_fn(batch):
    """Pad variable-length segments and return lengths for packing."""
    segments, labels, pids, weights = zip(*batch)
    lengths = torch.tensor([seg.shape[0] for seg in segments])
    # pad_sequence expects (T, C) tensors → pads along dim-0
    padded = rnn_utils.pad_sequence(segments, batch_first=True, padding_value=0.0)
    labels = torch.stack(labels)
    weights = torch.stack(weights)
    return padded, labels, lengths, list(pids), weights

# ---------------------------------------------------------------------------
# CNN Feature Extractor
# ---------------------------------------------------------------------------

class CNNFeatureExtractor(nn.Module):
    """
    Multi-layer 1-D CNN that downsamples a raw multi-channel time series
    into a shorter sequence of learned feature vectors.

    Input  : (batch, n_channels, time_steps)
    Output : (batch, reduced_time, cnn_filters[-1])
    """
    def __init__(self, n_channels: int, cnn_filters: list[int],
                 cnn_kernel_sizes: list[int], pool_size: int,
                 dropout_rate: float):
        super().__init__()
        layers = []
        in_ch = n_channels
        for filters, ks in zip(cnn_filters, cnn_kernel_sizes):
            padding = ks // 2  # 'same'-like padding
            layers.extend([
                nn.Conv1d(in_ch, filters, kernel_size=ks, padding=padding),
                nn.InstanceNorm1d(filters, affine=True),  # per-sample norm → strips participant-level statistics
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
# Full CNN-LSTM Network
# ---------------------------------------------------------------------------

class CNNLSTMNetwork(nn.Module):
    def __init__(self, n_channels: int, cnn_filters: list[int],
                 cnn_kernel_sizes: list[int], pool_size: int,
                 lstm_hidden_size: int, lstm_num_layers: int,
                 dropout_rate: float):
        super().__init__()
        self.cnn = CNNFeatureExtractor(
            n_channels, cnn_filters, cnn_kernel_sizes, pool_size, dropout_rate
        )
        self.lstm = nn.LSTM(
            input_size=self.cnn.out_channels,
            hidden_size=lstm_hidden_size,
            num_layers=lstm_num_layers,
            batch_first=True,
            dropout=dropout_rate if lstm_num_layers > 1 else 0.0,
        )
        self.dropout = nn.Dropout(dropout_rate)
        self.fc = nn.Linear(lstm_hidden_size, 1)

        # Store for length adjustment
        self._pool_size = pool_size
        self._n_blocks = len(cnn_filters)

    def _adjust_lengths(self, lengths):
        """Compute post-CNN sequence lengths from original sample lengths."""
        adj = lengths.float()
        for _ in range(self._n_blocks):
            adj = torch.floor(adj / self._pool_size)
        return adj.long().clamp(min=1)

    def forward(self, x, lengths):
        # x: (batch, n_channels, time_steps)
        cnn_out = self.cnn(x)

        # L2-normalise along the feature dim so all participants land on the
        # same unit hypersphere, removing between-participant magnitude offsets
        cnn_out = F.normalize(cnn_out, p=2, dim=-1)

        # Compute adjusted lengths for packing
        adj_lengths = self._adjust_lengths(lengths)
        # Clamp to actual reduced_time (safety)
        adj_lengths = adj_lengths.clamp(max=cnn_out.size(1))

        packed = rnn_utils.pack_padded_sequence(
            cnn_out, adj_lengths.cpu(), batch_first=True, enforce_sorted=False
        )
        _, (h_n, _) = self.lstm(packed)
        last_hidden = h_n[-1]           # (batch, hidden_size)
        last_hidden = self.dropout(last_hidden)
        out = self.fc(last_hidden)      # (batch, 1)
        return out

# ---------------------------------------------------------------------------
# Scikit-learn-style regressor wrapper
# ---------------------------------------------------------------------------

class CNNLSTMRegressor:
    def __init__(self,
                 cnn_filters: list[int] = CNN_LSTM_CONFIG['cnn_filters'],
                 cnn_kernel_sizes: list[int] = CNN_LSTM_CONFIG['cnn_kernel_sizes'],
                 pool_size: int = CNN_LSTM_CONFIG['pool_size'],
                 lstm_hidden_size: int = CNN_LSTM_CONFIG['lstm_hidden_size'],
                 lstm_num_layers: int = CNN_LSTM_CONFIG['lstm_num_layers'],
                 dropout_rate: float = CNN_LSTM_CONFIG['dropout_rate'],
                 learning_rate: float = CNN_LSTM_CONFIG['learning_rate'],
                 weight_decay: float = CNN_LSTM_CONFIG.get('weight_decay', 0.0),
                 batch_size: int = CNN_LSTM_CONFIG['batch_size'],
                 epochs: int = CNN_LSTM_CONFIG['epochs'],
                 validation_split: float = CNN_LSTM_CONFIG.get('validation_split', 0.2),
                 early_stopping_patience: int = CNN_LSTM_CONFIG.get('early_stopping_patience', 30),
                 scheduler_patience: int = CNN_LSTM_CONFIG.get('scheduler_patience', 10),
                 scheduler_factor: float = CNN_LSTM_CONFIG.get('scheduler_factor', 0.5),
                 loss_type: str = GLOBAL_LOSS_FUNCTION,
                 balance_weights: bool = CNN_LSTM_CONFIG.get('balance_weights', GLOBAL_BALANCE_WEIGHTS),
                 balance_participants: bool = CNN_LSTM_CONFIG.get('balance_participants', False),
                 random_state: int = CNN_LSTM_CONFIG['random_state']):

        self.cnn_filters = list(cnn_filters)
        self.cnn_kernel_sizes = list(cnn_kernel_sizes)
        self.pool_size = pool_size
        self.lstm_hidden_size = lstm_hidden_size
        self.lstm_num_layers = lstm_num_layers
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
        self.balance_weights = balance_weights
        self.balance_participants = balance_participants
        self.random_state = random_state

        # Tracking
        self.loss_history = {"train": [], "val": []}
        self.train_samples = 0
        self.val_samples = 0

        torch.manual_seed(self.random_state)

        self.scaler = StandardScaler()
        self.model = None
        self.n_channels = None

        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        elif torch.backends.mps.is_available():
            self.device = torch.device("mps")
        else:
            self.device = torch.device("cpu")
        print(f"[{self.__class__.__name__}] Using device: {self.device}")

    # ------------------------------------------------------------------
    # Data helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _extract_raw_segments(X: pd.DataFrame) -> list[np.ndarray]:
        """Extract raw segment arrays from the DataFrame."""
        if 'raw_segment' not in X.columns:
            raise ValueError(
                "CNNLSTMRegressor expects a DataFrame with a 'raw_segment' column. "
                "Use DataLoader.load_raw_segments() to prepare data."
            )
        return X['raw_segment'].tolist()

    @staticmethod
    def _sanitise(arr: np.ndarray, clip: float = 1e9) -> np.ndarray:
        """Replace NaN/Inf with 0 and clip abs-values to `clip` to prevent
        float64 overflow inside sklearn's StandardScaler."""
        arr = np.where(np.isfinite(arr), arr, 0.0)
        return np.clip(arr, -clip, clip)

    def _scale_segments(self, segments: list[np.ndarray], fit: bool = False) -> list[np.ndarray]:
        """Per-channel z-score standardisation."""
        if fit:
            # Flatten all time-steps across all segments to fit one scaler
            all_data = self._sanitise(np.vstack(segments))
            self.scaler.fit(all_data)

        scaled = []
        for seg in segments:
            scaled.append(self.scaler.transform(self._sanitise(seg)).astype(np.float32))
        return scaled

    # ------------------------------------------------------------------
    # Training
    # ------------------------------------------------------------------

    def fit(self, X: pd.DataFrame, y: pd.Series, sample_weight=None):
        from sklearn.model_selection import train_test_split
        from tqdm import tqdm
        import copy

        # 1. Train / validation split
        stratify = X["label"] if "label" in X.columns else None
        X_train, X_val, y_train, y_val = train_test_split(
            X, y, test_size=self.validation_split,
            random_state=self.random_state, stratify=stratify,
        )

        # 2. Extract raw segments
        segs_train = self._extract_raw_segments(X_train)
        segs_val = self._extract_raw_segments(X_val)
        self.n_channels = segs_train[0].shape[1]

        # 3. Fit scaler on training data, then transform both
        scaled_train = self._scale_segments(segs_train, fit=True)
        scaled_val = self._scale_segments(segs_val, fit=False)
        y_np_train = y_train.values.astype(np.float32)

        # 3b. Data augmentation (training only)
        augmenter = SequenceAugmenter(config=AUGMENTATION_CONFIG)
        participant_ids_train = (
            X_train['subject'].values if 'subject' in X_train.columns else None
        )
        scaled_train, y_np_train, aug_pids = augmenter.augment_dataset(
            scaled_train, y_np_train, participant_ids=participant_ids_train, return_pids=True
        )
        if AUGMENTATION_CONFIG.get('enabled', False):
            n_aug = len(scaled_train) - len(segs_train)
            print(f"[CNNLSTMRegressor] Augmentation: {len(segs_train)} → "
                  f"{len(scaled_train)} segments (+{n_aug} synthetic)")

        train_tensors = [torch.from_numpy(s) for s in scaled_train]
        y_tensor_train = torch.from_numpy(y_np_train).unsqueeze(1)
        val_tensors = [torch.from_numpy(s) for s in scaled_val]
        y_tensor_val = torch.from_numpy(y_val.values.astype(np.float32)).unsqueeze(1)

        self.train_samples = len(X_train)
        self.val_samples = len(X_val)

        # 4. Handle sample weighting
        train_weights_tensor = None
        if self.balance_weights and self.balance_participants:
            if aug_pids is None:
                raise ValueError("balance_participants is True but no participant IDs were provided in data.")
            w_class = compute_sample_weight(class_weight='balanced', y=y_np_train.flatten())
            w_part = compute_sample_weight(class_weight='balanced', y=aug_pids)
            combined = w_class * w_part
            combined = combined / np.mean(combined)
            train_weights_tensor = torch.from_numpy(combined.astype(np.float32)).unsqueeze(1)
            print(f"[CNNLSTMRegressor] Hybrid sample weighting enabled (Target Class * Participants). Weights assigned to {len(combined)} training samples.")
        elif self.balance_weights:
            weights_np = compute_sample_weight(class_weight='balanced', y=y_np_train.flatten())
            train_weights_tensor = torch.from_numpy(weights_np.astype(np.float32)).unsqueeze(1)
            print(f"[CNNLSTMRegressor] Sample weighting enabled (Target Class). Weights assigned to {len(weights_np)} training samples.")
        elif self.balance_participants:
            if aug_pids is None:
                raise ValueError("balance_participants is True but no participant IDs were provided in data.")
            weights_np = compute_sample_weight(class_weight='balanced', y=aug_pids)
            train_weights_tensor = torch.from_numpy(weights_np.astype(np.float32)).unsqueeze(1)
            print(f"[CNNLSTMRegressor] Sample weighting enabled (Participants). Weights assigned to {len(weights_np)} training samples.")

        # 4. Build model
        self.model = CNNLSTMNetwork(
            n_channels=self.n_channels,
            cnn_filters=self.cnn_filters,
            cnn_kernel_sizes=self.cnn_kernel_sizes,
            pool_size=self.pool_size,
            lstm_hidden_size=self.lstm_hidden_size,
            lstm_num_layers=self.lstm_num_layers,
            dropout_rate=self.dropout_rate,
        ).to(self.device)

        optimizer = optim.AdamW(self.model.parameters(),
                                lr=self.learning_rate, weight_decay=self.weight_decay)

        if self.loss_type == 'mae':
            criterion = nn.L1Loss()
        else:
            criterion = nn.MSELoss()
            if self.loss_type != 'mse':
                print(f"[CNNLSTMRegressor] Unknown loss '{self.loss_type}', defaulting to MSE.")

        # 5. Build datasets
        dataset_train = RawSegmentDataset(train_tensors, y_tensor_train, 
                                            sample_weights=train_weights_tensor)
        _g = torch.Generator()
        _g.manual_seed(self.random_state)
        loader_train = DataLoader(dataset_train, batch_size=self.batch_size,
                                  shuffle=True, collate_fn=raw_pad_collate_fn,
                                  generator=_g)

        scheduler = optim.lr_scheduler.ReduceLROnPlateau(
            optimizer, 
            mode='min', 
            factor=self.scheduler_factor,
            patience=self.scheduler_patience,
            min_lr=1e-6
        )

        dataset_val = RawSegmentDataset(val_tensors, y_tensor_val)  # val stays unweighted
        loader_val = DataLoader(dataset_val, batch_size=self.batch_size,
                                shuffle=False, collate_fn=raw_pad_collate_fn)

        # 5. Training loop
        best_val_loss = float('inf')
        patience_counter = 0
        best_weights = copy.deepcopy(self.model.state_dict())

        with tqdm(range(self.epochs), desc="Training CNN-LSTM", unit="epoch") as pbar:
            for epoch in pbar:
                # --- TRAIN ---
                self.model.train()
                running_loss = 0.0
                for batch_x, batch_y, lengths, batch_pids, batch_weights in loader_train:
                    # batch_x: (B, T, C) → transpose to (B, C, T) for Conv1d
                    batch_x = batch_x.transpose(1, 2).to(self.device)
                    batch_y = batch_y.to(self.device)
                    lengths = lengths.to(self.device)
                    batch_weights = batch_weights.to(self.device)

                    optimizer.zero_grad()
                    outputs = self.model(batch_x, lengths)
                    
                    # Weighted loss calculation
                    if self.balance_weights or self.balance_participants:
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

                # Val loop — val set has no pids, so pass dummy pids
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
                current_lr = optimizer.param_groups[0]['lr']
                pbar.set_postfix({"Loss": f"{avg_train_loss:.4f}",
                                  "Val Loss": f"{avg_val_loss:.4f}",
                                  "LR": f"{current_lr:.1e}"})

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
                    print(f"\nEarly stopping at epoch {epoch+1}. "
                          f"Best Val Loss: {best_val_loss:.4f}")
                    break

        self.model.load_state_dict(best_weights)
        self.model.eval()

    # ------------------------------------------------------------------
    # Learning Rate Finder
    # ------------------------------------------------------------------

    def lr_find(self, X: pd.DataFrame, y: pd.Series, end_lr=10.0, num_iter=100, save_path=None):
        """Runs the LR Finder range test and plots the result."""
        from model.lr_finder import LRFinder

        # 1. Extract raw segments
        segs = self._extract_raw_segments(X)
        self.n_channels = segs[0].shape[1]

        # 2. Fit scaler on training data
        scaled_train = self._scale_segments(segs, fit=True)
        y_np_train = y.values.astype(np.float32)

        train_tensors = [torch.from_numpy(s) for s in scaled_train]
        y_tensor_train = torch.from_numpy(y_np_train).unsqueeze(1)

        # 3. Build model
        self.model = CNNLSTMNetwork(
            n_channels=self.n_channels,
            cnn_filters=self.cnn_filters,
            cnn_kernel_sizes=self.cnn_kernel_sizes,
            pool_size=self.pool_size,
            lstm_hidden_size=self.lstm_hidden_size,
            lstm_num_layers=self.lstm_num_layers,
            dropout_rate=self.dropout_rate,
        ).to(self.device)

        # Set a very low initial LR for the finder
        optimizer = optim.AdamW(self.model.parameters(), lr=1e-7, weight_decay=self.weight_decay)

        if self.loss_type == 'mae':
            criterion = nn.L1Loss()
        else:
            criterion = nn.MSELoss()

        dataset_train = RawSegmentDataset(train_tensors, y_tensor_train)
        loader_train = DataLoader(dataset_train, batch_size=self.batch_size,
                                  shuffle=True, collate_fn=raw_pad_collate_fn)

        finder = LRFinder(self.model, optimizer, criterion, self.device)
        finder.range_test(loader_train, end_lr=end_lr, num_iter=num_iter)
        finder.plot(save_path=save_path)


    # ------------------------------------------------------------------
    # Prediction
    # ------------------------------------------------------------------

    def predict(self, X: pd.DataFrame):
        if self.model is None:
            raise ValueError("Model not fitted.")

        segments = self._extract_raw_segments(X)
        return self._predict_from_segments(segments)

    def _predict_from_segments(self, segments: list[np.ndarray]):
        """Helper to predict from a list of raw segments directly (bypassing DataFrame extraction)."""
        if self.model is None:
            raise ValueError("Model not fitted.")
            
        scaled = self._scale_segments(segments, fit=False)
        tensors = [torch.from_numpy(s) for s in scaled]

        dummy_y = torch.zeros((len(tensors), 1))
        dataset = RawSegmentDataset(tensors, dummy_y)
        loader = DataLoader(dataset, batch_size=self.batch_size,
                            shuffle=False, collate_fn=raw_pad_collate_fn)

        self.model.eval()
        all_preds = []
        with torch.no_grad():
            for batch_x, _, lengths, _pids, _ in loader:
                batch_x = batch_x.transpose(1, 2).to(self.device)
                lengths = lengths.to(self.device)
                preds = self.model(batch_x, lengths).cpu().numpy()
                all_preds.extend(preds.flatten())

        return np.maximum(0.0, np.array(all_preds))

    def permutation_importance(self, X_test: pd.DataFrame, y_test: pd.Series, n_repeats: int = 1) -> dict:
        """
        Calculate permutation importance for each channel or group of channels.
        IMU vector channels are grouped together and permuted simultaneously.
        """
        y_test_np = y_test.values
        baseline_pred = self.predict(X_test)
        baseline_mse = mean_squared_error(y_test_np, baseline_pred)
        
        segments = self._extract_raw_segments(X_test)
        
        channel_names = X_test.attrs.get("channel_names")
        if channel_names is None or len(channel_names) != self.n_channels:
            channel_names = [f"Ch_{i}" for i in range(self.n_channels)]
            
        imu_groups = {
            'ax1': '$a_1$', 'ay1': '$a_1$', 'az1': '$a_1$',
            'roll_rad1': '$\\alpha_1$', 'pitch_rad1': '$\\alpha_1$', 'yaw_rad1': '$\\alpha_1$',
            'ax2': '$a_2$', 'ay2': '$a_2$', 'az2': '$a_2$',
            'roll_rad2': '$\\alpha_2$', 'pitch_rad2': '$\\alpha_2$', 'yaw_rad2': '$\\alpha_2$',
            'ax_diff': '$a_{diff}$', 'ay_diff': '$a_{diff}$', 'az_diff': '$a_{diff}$',
            'roll_rad_diff': '$\\alpha_{diff}$', 'pitch_rad_diff': '$\\alpha_{diff}$', 'yaw_rad_diff': '$\\alpha_{diff}$',
        }
        
        from collections import defaultdict
        grouped_indices = defaultdict(list)
        for c, ch_name in enumerate(channel_names):
            group_label = imu_groups.get(ch_name, ch_name)
            grouped_indices[group_label].append(c)
            
        importances = {}
        for group_label, indices in grouped_indices.items():
            channel_mse_diffs = []
            for _ in range(n_repeats):
                permuted_segments = []
                for seg in segments:
                    seg_copy = seg.copy()
                    for c in indices:
                        np.random.shuffle(seg_copy[:, c]) 
                    permuted_segments.append(seg_copy)
                    
                perm_pred = self._predict_from_segments(permuted_segments)
                perm_mse = mean_squared_error(y_test_np, perm_pred)
                channel_mse_diffs.append(perm_mse - baseline_mse)
                
            importances[group_label] = np.mean(channel_mse_diffs)
            
        return importances

    # ------------------------------------------------------------------
    # Evaluation
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Save / Load
    # ------------------------------------------------------------------

    def save(self, filepath: str | Path):
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)

        state = {
            'model_state': self.model.state_dict(),
            'scaler': self.scaler,
            'n_channels': self.n_channels,
            'config': {
                'cnn_filters': self.cnn_filters,
                'cnn_kernel_sizes': self.cnn_kernel_sizes,
                'pool_size': self.pool_size,
                'lstm_hidden_size': self.lstm_hidden_size,
                'lstm_num_layers': self.lstm_num_layers,
                'dropout_rate': self.dropout_rate,
                'learning_rate': self.learning_rate,
                'weight_decay': self.weight_decay,
                'batch_size': self.batch_size,
                'epochs': self.epochs,
                'loss_type': self.loss_type,
                'balance_weights': self.balance_weights,
                'balance_participants': self.balance_participants,
                'validation_split': self.validation_split,
                'early_stopping_patience': self.early_stopping_patience,
                'scheduler_patience': self.scheduler_patience,
                'scheduler_factor': self.scheduler_factor,
                'random_state': self.random_state,
            },
            'split_info': {
                'train_samples': self.train_samples,
                'val_samples': self.val_samples,
            },
            'loss_history': self.loss_history,
        }
        torch.save(state, filepath)

    @classmethod
    def load(cls, filepath: str | Path):
        import inspect
        state = torch.load(filepath, map_location='cpu', weights_only=False)
        
        # Backwards compatibility for older saved models
        if 'hidden_size' in state['config']:
            state['config']['lstm_hidden_size'] = state['config'].pop('hidden_size')
        if 'num_layers' in state['config']:
            state['config']['lstm_num_layers'] = state['config'].pop('num_layers')
            
        # Filter out unexpectedly stored configuration keys (e.g. emg_window_size_sec)
        sig = inspect.signature(cls.__init__)
        valid_keys = set(sig.parameters.keys())
        valid_keys.discard('self')
        
        filtered_config = {k: v for k, v in state['config'].items() if k in valid_keys}
            
        regressor = cls(**filtered_config)
        regressor.balance_weights = state['config'].get('balance_weights', False)
        regressor.balance_participants = state['config'].get('balance_participants', False)
        regressor.scaler = state['scaler']
        
        # Infer n_channels for older models
        if 'n_channels' in state:
            n_channels = state['n_channels']
        else:
            n_channels = state['model_state']['cnn.net.0.weight'].shape[1]
        
        regressor.n_channels = n_channels
        regressor.model = CNNLSTMNetwork(
            n_channels=n_channels,
            cnn_filters=state['config'].get('cnn_filters', filtered_config.get('cnn_filters')),
            cnn_kernel_sizes=state['config'].get('cnn_kernel_sizes', filtered_config.get('cnn_kernel_sizes')),
            pool_size=state['config'].get('pool_size', filtered_config.get('pool_size')),
            lstm_hidden_size=state['config'].get('lstm_hidden_size', filtered_config.get('lstm_hidden_size')),
            lstm_num_layers=state['config'].get('lstm_num_layers', filtered_config.get('lstm_num_layers')),
            dropout_rate=state['config'].get('dropout_rate', filtered_config.get('dropout_rate')),
        ).to(regressor.device)
        # Load weights with strict=False to handle the transition from BatchNorm to InstanceNorm
        # (InstanceNorm doesn't use running statistics which are present in BN checkpoints)
        missing_keys, unexpected_keys = regressor.model.load_state_dict(state['model_state'], strict=False)
        if unexpected_keys:
            print(f"[CNNLSTMRegressor] Note: ignored {len(unexpected_keys)} unexpected keys during load "
                  f"(likely BatchNorm statistics being skipped for InstanceNorm layers).")
        if missing_keys:
            print(f"[CNNLSTMRegressor] WARNING: Missing keys during load: {missing_keys}")
        
        regressor.model.eval()

        if 'split_info' in state:
            regressor.train_samples = state['split_info'].get('train_samples', 0)
            regressor.val_samples = state['split_info'].get('val_samples', 0)
        if 'loss_history' in state:
            regressor.loss_history = state['loss_history']

        return regressor

    # ------------------------------------------------------------------
    # Feature extraction (CNN only — for t-SNE / analysis)
    # ------------------------------------------------------------------

    def extract_cnn_features(self, X: pd.DataFrame) -> np.ndarray:
        """Run only the CNN encoder and return global-average-pooled features.

        Returns
        -------
        np.ndarray of shape (N, cnn_filters[-1])
            One feature vector per segment, suitable for t-SNE / PCA.
        """
        from tqdm import tqdm

        if self.model is None:
            raise ValueError("Model not fitted.")

        segments = self._extract_raw_segments(X)
        scaled = self._scale_segments(segments, fit=False)
        tensors = [torch.from_numpy(s) for s in scaled]

        dummy_y = torch.zeros((len(tensors), 1))
        dataset = RawSegmentDataset(tensors, dummy_y)
        loader = DataLoader(dataset, batch_size=self.batch_size,
                            shuffle=False, collate_fn=raw_pad_collate_fn)

        self.model.eval()
        all_features = []
        with torch.no_grad():
            for batch_x, _, lengths, _pids, _ in tqdm(loader, desc="Extracting CNN features", unit="batch"):
                # batch_x: (B, T, C) → (B, C, T) for Conv1d
                batch_x = batch_x.transpose(1, 2).to(self.device)

                cnn_out = self.model.cnn(batch_x)  # (B, reduced_T, cnn_out_ch)
                # Global average pooling over the time dimension
                pooled = cnn_out.mean(dim=1)        # (B, cnn_out_ch)
                all_features.append(pooled.cpu().numpy())

        return np.concatenate(all_features, axis=0)  # (N, cnn_out_ch)

    # ------------------------------------------------------------------
    # Plotting passthrough
    # ------------------------------------------------------------------

    def plot_results(self, y_test: pd.Series, y_pred: np.ndarray, save_path: str | Path):
        plotting_utils.plot_regression_results(y_test, y_pred, save_path, model_name="CNN-LSTM")

    def plot_loss(self, save_path: str | Path):
        plotting_utils.plot_training_loss(self.loss_history, save_path, model_name="CNN-LSTM")

    def plot_tsne(self, X: pd.DataFrame, participants: np.ndarray,
                  weights: np.ndarray, save_path: str | Path,
                  perplexity: int = 30):
        """Extract CNN features and produce a t-SNE scatter plot.

        Parameters
        ----------
        X            : DataFrame with 'raw_segment' column (full dataset or test set).
        participants : (N,) array of participant IDs aligned with rows in X.
        weights      : (N,) array of weight labels aligned with rows in X.
        save_path    : where to save the PNG.
        perplexity   : t-SNE perplexity (default 30).
        """
        features = self.extract_cnn_features(X)
        plotting_utils.plot_cnn_tsne(
            features=features,
            participants=participants,
            weights=weights,
            save_path=save_path,
            model_name="CNN-LSTM",
            perplexity=perplexity,
            random_state=self.random_state,
        )
