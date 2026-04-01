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

# Add project root to sys.path
sys.path.append(str(Path(__file__).resolve().parent.parent.parent))

from model.config_model import TRANSFORMER_CONFIG, FEATURE_CONFIG, GLOBAL_LOSS_FUNCTION, AUGMENTATION_CONFIG
from model import plotting_utils
from model.data_augmentation import SequenceAugmenter

class SequenceDataset(Dataset):
    def __init__(self, sequences, labels):
        # sequences is a list of tensors of shape (seq_len, features)
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
        self.register_buffer('pe', pe.unsqueeze(0)) # Shape: (1, max_len, d_model)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Arguments:
            x: Tensor, shape ``[batch_size, seq_len, embedding_dim]``
        """
        x = x + self.pe[:, :x.size(1), :]
        return x

class TimeSeriesTransformerNetwork(nn.Module):
    def __init__(self, input_size: int, d_model: int, nhead: int, num_layers: int, dim_feedforward: int, dropout_rate: float):
        super().__init__()
        
        # Linear layer to project input features to d_model dimension
        self.input_projection = nn.Linear(input_size, d_model)
        
        # Positional Encoding injects temporal order information
        self.pos_encoder = PositionalEncoding(d_model)
        
        # Transformer Encoder
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=d_model, 
            nhead=nhead, 
            dim_feedforward=dim_feedforward, 
            dropout=dropout_rate, 
            batch_first=True
        )
        self.transformer_encoder = nn.TransformerEncoder(
            encoder_layer, 
            num_layers=num_layers,
            enable_nested_tensor=False # Required for MPS compatibility with masks
        )
        
        # Final fully connected layer for regression
        self.fc = nn.Linear(d_model, 1)
        
        self.d_model = d_model

    def forward(self, x, lengths):
        # x shape: [batch_size, seq_len, input_size]
        
        # 1. Project to d_model space: [batch_size, seq_len, d_model]
        x = self.input_projection(x) * math.sqrt(self.d_model)
        
        # 2. Add Positional Encoding
        x = self.pos_encoder(x)
        
        # 3. Create Padding Mask (True for padded positions)
        max_len = x.size(1)
        batch_size = x.size(0)
        padding_mask = torch.arange(max_len, device=x.device).expand(batch_size, max_len) >= lengths.unsqueeze(1)
        
        # 4. Create Causal Mask (Upper triangular mask) for real-time compatibility
        # This ensures token i does not attend to tokens > i
        causal_mask = nn.Transformer.generate_square_subsequent_mask(max_len).to(torch.bool).to(x.device)
        
        # 5. Pass through Transformer
        transformer_out = self.transformer_encoder(
            x, 
            mask=causal_mask, 
            src_key_padding_mask=padding_mask
        )
        
        # 6. Last Valid Token Pooling
        # Instead of averaging the whole sequence (which includes future info in non-causal case),
        # we take the representation of the last valid timestamp in each sequence.
        # This is more natural for causal real-time estimation.
        last_tokens = transformer_out[torch.arange(batch_size), lengths - 1, :]
        
        # 7. Final prediction
        out = self.fc(last_tokens)
        return out


class TimeSeriesTransformerRegressor:
    def __init__(self, 
                 d_model: int = TRANSFORMER_CONFIG['d_model'],
                 nhead: int = TRANSFORMER_CONFIG['nhead'],
                 num_layers: int = TRANSFORMER_CONFIG['num_layers'],
                 dim_feedforward: int = TRANSFORMER_CONFIG['dim_feedforward'],
                 dropout_rate: float = TRANSFORMER_CONFIG['dropout_rate'],
                 learning_rate: float = TRANSFORMER_CONFIG['learning_rate'],
                 weight_decay: float = TRANSFORMER_CONFIG.get('weight_decay', 0.0),
                 batch_size: int = TRANSFORMER_CONFIG['batch_size'],
                 epochs: int = TRANSFORMER_CONFIG['epochs'],
                 validation_split: float = TRANSFORMER_CONFIG.get('validation_split', 0.2),
                 early_stopping_patience: int = TRANSFORMER_CONFIG.get('early_stopping_patience', 10),
                 scheduler_patience: int = TRANSFORMER_CONFIG.get('scheduler_patience', 5),
                 scheduler_factor: float = TRANSFORMER_CONFIG.get('scheduler_factor', 0.5),
                 emg_window_size_sec: float = FEATURE_CONFIG['emg_window_size_sec'],
                 imu_window_size_sec: float = FEATURE_CONFIG['imu_window_size_sec'],
                 window_step_sec: float = FEATURE_CONFIG['window_step_sec'],
                 loss_type: str = GLOBAL_LOSS_FUNCTION,
                 random_state: int = TRANSFORMER_CONFIG['random_state']):
                 
        self.d_model = d_model
        self.nhead = nhead
        self.num_layers = num_layers
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
        self.random_state = random_state
        
        # Loss history
        self.loss_history = {"train": [], "val": []}
        
        # Split stats
        self.train_samples = 0
        self.val_samples = 0
        
        torch.manual_seed(self.random_state)
        
        self.scaler = StandardScaler()
        self.model = None
        self.feature_names = None
        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        elif torch.backends.mps.is_available():
            self.device = torch.device("mps")
        else:
            self.device = torch.device("cpu")
        print(f"[{self.__class__.__name__}] Using device: {self.device}")
        
    def _extract_sequences(self, X: pd.DataFrame) -> list:
        if 'sequence_dicts' not in X.columns:
            raise ValueError("TransformerRegressor expects DataFrame with 'sequence_dicts' column.")
            
        sequences = X['sequence_dicts'].tolist()
        return sequences

    @staticmethod
    def _sanitise(arr: np.ndarray, clip: float = 1e9) -> np.ndarray:
        """Replace NaN/Inf with 0 and clip abs-values to `clip` to prevent
        float64 overflow inside sklearn's StandardScaler."""
        arr = np.where(np.isfinite(arr), arr, 0.0)
        return np.clip(arr, -clip, clip)
        
    def fit(self, X: pd.DataFrame, y: pd.Series):
        from sklearn.model_selection import train_test_split
        from tqdm import tqdm
        import copy
        
        # 1. Split data into train and validation sets
        stratify = X["label"] if "label" in X.columns else None
        X_train, X_val, y_train, y_val = train_test_split(
            X, y, test_size=self.validation_split, random_state=self.random_state, stratify=stratify
        )
        
        # 2. Extract sequences
        sequences_train = self._extract_sequences(X_train)
        sequences_val = self._extract_sequences(X_val)
        self.feature_names = list(sequences_train[0][0].keys())
        
        # 3. Flatten train to fit scaler
        flat_data_train = []
        for seq in sequences_train:
            seq_arr = np.array([[w[k] for k in self.feature_names] for w in seq])
            flat_data_train.append(seq_arr)
            
        all_features_train = self._sanitise(np.vstack(flat_data_train))
        self.scaler.fit(all_features_train)
        
        # 4. Reconstruct scaled train sequences
        scaled_seqs_train = []
        for arr in flat_data_train:
            scaled_arr = self.scaler.transform(self._sanitise(arr)).astype(np.float32)
            scaled_seqs_train.append(scaled_arr)  # keep as numpy for augmentation
            
        y_np_train = y_train.values.astype(np.float32)
        
        # 4b. Data augmentation (training only, on scaled z-score arrays)
        augmenter = SequenceAugmenter(config=AUGMENTATION_CONFIG)
        scaled_seqs_train, y_np_train = augmenter.augment_dataset(scaled_seqs_train, y_np_train)
        if AUGMENTATION_CONFIG.get('enabled', True):
            n_aug = len(scaled_seqs_train) - len(flat_data_train)
            print(f"[{self.__class__.__name__}] Augmentation: {len(flat_data_train)} → {len(scaled_seqs_train)} sequences (+{n_aug} synthetic)")
        
        # Convert to tensors
        scaled_seqs_train = [torch.from_numpy(a) for a in scaled_seqs_train]
        y_tensor_train = torch.from_numpy(y_np_train).unsqueeze(1)
        
        # 5. Process validation sequences using fitted scaler
        scaled_seqs_val = []
        for seq in sequences_val:
            seq_arr = np.array([[w[k] for k in self.feature_names] for w in seq])
            scaled_arr = self.scaler.transform(self._sanitise(seq_arr)).astype(np.float32)
            scaled_seqs_val.append(torch.from_numpy(scaled_arr))
            
        y_tensor_val = torch.from_numpy(y_val.values.astype(np.float32)).unsqueeze(1)
        
        self.train_samples = len(X_train)
        self.val_samples = len(X_val)
        
        # 6. Initialize Model and Optimizers
        input_dim = len(self.feature_names)
        self.model = TimeSeriesTransformerNetwork(
            input_size=input_dim, 
            d_model=self.d_model, 
            nhead=self.nhead, 
            num_layers=self.num_layers, 
            dim_feedforward=self.dim_feedforward, 
            dropout_rate=self.dropout_rate
        ).to(self.device)
        
        optimizer = optim.AdamW(self.model.parameters(), lr=self.learning_rate, weight_decay=self.weight_decay)
        scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', patience=self.scheduler_patience, factor=self.scheduler_factor)
        
        if self.loss_type == 'mae':
            criterion = nn.L1Loss()
        else:
            criterion = nn.MSELoss()
            if self.loss_type != 'mse':
                print(f"[TransformerRegressor] Unknown loss type '{self.loss_type}', defaulting to MSE.")
        
        dataset_train = SequenceDataset(scaled_seqs_train, y_tensor_train)
        loader_train = DataLoader(dataset_train, batch_size=self.batch_size, shuffle=True, collate_fn=pad_collate_fn)
        
        dataset_val = SequenceDataset(scaled_seqs_val, y_tensor_val)
        loader_val = DataLoader(dataset_val, batch_size=self.batch_size, shuffle=False, collate_fn=pad_collate_fn)
        
        # Early stopping tracking
        best_val_loss = float('inf')
        patience_counter = 0
        best_model_weights = copy.deepcopy(self.model.state_dict())
        
        # Wrap the epoch range with tqdm
        with tqdm(range(self.epochs), desc="Training Transformer", unit="epoch") as pbar:
            for epoch in pbar:
                # TRAIN LOOP
                self.model.train()
                running_loss = 0.0
                for batch_x, batch_y, lengths in loader_train:
                    batch_x, batch_y = batch_x.to(self.device), batch_y.to(self.device)
                    lengths = lengths.to(self.device)
                    
                    optimizer.zero_grad()
                    outputs = self.model(batch_x, lengths)
                    loss = criterion(outputs, batch_y)
                    loss.backward()
                    
                    # Gradient clipping
                    torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=1.0)
                    
                    optimizer.step()
                    
                    running_loss += loss.item() * batch_x.size(0)
                    
                avg_train_loss = running_loss / len(dataset_train)
                
                # VALIDATION LOOP
                self.model.eval()
                val_loss = 0.0
                with torch.no_grad():
                    for batch_x, batch_y, lengths in loader_val:
                        batch_x, batch_y = batch_x.to(self.device), batch_y.to(self.device)
                        lengths = lengths.to(self.device)
                        
                        outputs = self.model(batch_x, lengths)
                        loss = criterion(outputs, batch_y)
                        val_loss += loss.item() * batch_x.size(0)
                
                avg_val_loss = val_loss / len(dataset_val)
                pbar.set_postfix({"Loss": f"{avg_train_loss:.4f}", "Val Loss": f"{avg_val_loss:.4f}"})
                
                # Step the scheduler
                scheduler.step(avg_val_loss)
                
                # TRACK LOSS
                self.loss_history["train"].append(avg_train_loss)
                self.loss_history["val"].append(avg_val_loss)
                
                # EARLY STOPPING CHECK
                if avg_val_loss < best_val_loss:
                    best_val_loss = avg_val_loss
                    patience_counter = 0
                    best_model_weights = copy.deepcopy(self.model.state_dict())
                else:
                    patience_counter += 1
                    
                if patience_counter >= self.early_stopping_patience:
                    print(f"\nEarly stopping triggered at epoch {epoch+1}. Best Val Loss: {best_val_loss:.4f}")
                    break
                    
        # Load best weights
        self.model.load_state_dict(best_model_weights)
        self.model.eval()
                
    def predict(self, X: pd.DataFrame):
        if self.model is None:
            raise ValueError("Model not fitted.")
            
        sequences = self._extract_sequences(X)
        
        scaled_seqs = []
        for seq in sequences:
            seq_arr = np.array([[w[k] for k in self.feature_names] for w in seq])
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
            'input_dim': len(self.feature_names),
            'config': {
                'd_model': self.d_model,
                'nhead': self.nhead,
                'num_layers': self.num_layers,
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
                'random_state': self.random_state
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
        
        regressor.model = TimeSeriesTransformerNetwork(
            input_size=state['input_dim'], 
            d_model=state['config']['d_model'], 
            nhead=state['config']['nhead'], 
            num_layers=state['config']['num_layers'],
            dim_feedforward=state['config']['dim_feedforward'],
            dropout_rate=state['config']['dropout_rate']
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
        plotting_utils.plot_regression_results(y_test, y_pred, save_path, model_name="Transformer")

    def plot_loss(self, save_path: str | Path):
        plotting_utils.plot_training_loss(self.loss_history, save_path, model_name="Transformer")
