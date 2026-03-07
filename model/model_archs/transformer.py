import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.utils.rnn as rnn_utils
from torch.utils.data import DataLoader, Dataset
import pandas as pd
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import math

from model.config_model import TRANSFORMER_CONFIG

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
        self.transformer_encoder = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)
        
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
        # padding_mask shape needs to be [batch_size, seq_len] for PyTorch TransformerEncoder when batch_first=True
        max_len = x.size(1)
        batch_size = x.size(0)
        padding_mask = torch.arange(max_len, device=x.device).expand(batch_size, max_len) >= lengths.unsqueeze(1)
        
        # 4. Pass through Transformer
        transformer_out = self.transformer_encoder(x, src_key_padding_mask=padding_mask)
        
        # 5. Global Average Pooling (ignore padded values)
        # We need to compute the mean over non-padded tokens for each sequence in the batch
        mask = (~padding_mask).float().unsqueeze(2) # [batch_size, seq_len, 1]
        sum_embeddings = (transformer_out * mask).sum(dim=1) # [batch_size, d_model]
        valid_lengths = lengths.float().unsqueeze(1) # [batch_size, 1]
        avg_pool = sum_embeddings / valid_lengths # [batch_size, d_model]
        
        # Alternative: Just take the last valid token (like LSTM hidden state) - less robust for transformers
        # last_tokens = transformer_out[torch.arange(batch_size), lengths - 1, :]
        
        # 6. Final prediction
        out = self.fc(avg_pool)
        return out


class TimeSeriesTransformerRegressor:
    def __init__(self, 
                 d_model: int = TRANSFORMER_CONFIG['d_model'],
                 nhead: int = TRANSFORMER_CONFIG['nhead'],
                 num_layers: int = TRANSFORMER_CONFIG['num_layers'],
                 dim_feedforward: int = TRANSFORMER_CONFIG['dim_feedforward'],
                 dropout_rate: float = TRANSFORMER_CONFIG['dropout_rate'],
                 learning_rate: float = TRANSFORMER_CONFIG['learning_rate'],
                 batch_size: int = TRANSFORMER_CONFIG['batch_size'],
                 epochs: int = TRANSFORMER_CONFIG['epochs'],
                 window_size_sec: float = TRANSFORMER_CONFIG['window_size_sec'],
                 window_step_sec: float = TRANSFORMER_CONFIG['window_step_sec'],
                 random_state: int = TRANSFORMER_CONFIG['random_state']):
                 
        self.d_model = d_model
        self.nhead = nhead
        self.num_layers = num_layers
        self.dim_feedforward = dim_feedforward
        self.dropout_rate = dropout_rate
        self.learning_rate = learning_rate
        self.batch_size = batch_size
        self.epochs = epochs
        self.window_size_sec = window_size_sec
        self.window_step_sec = window_step_sec
        self.random_state = random_state
        
        torch.manual_seed(self.random_state)
        
        self.scaler = StandardScaler()
        self.model = None
        self.feature_names = None
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
    def _extract_sequences(self, X: pd.DataFrame) -> list:
        if 'sequence_dicts' not in X.columns:
            raise ValueError("TransformerRegressor expects DataFrame with 'sequence_dicts' column.")
            
        sequences = X['sequence_dicts'].tolist()
        return sequences
        
    def fit(self, X: pd.DataFrame, y: pd.Series):
        sequences = self._extract_sequences(X)
        self.feature_names = list(sequences[0][0].keys())
        
        # Flatten and scale
        flat_data = []
        for seq in sequences:
            seq_arr = np.array([[w[k] for k in self.feature_names] for w in seq])
            flat_data.append(seq_arr)
            
        all_features = np.vstack(flat_data)
        self.scaler.fit(all_features)
        
        # Reconstruct scaled sequences
        scaled_seqs = []
        for arr in flat_data:
            scaled_arr = self.scaler.transform(arr).astype(np.float32)
            scaled_seqs.append(torch.from_numpy(scaled_arr))
            
        y_tensor = torch.from_numpy(y.values.astype(np.float32)).unsqueeze(1)
        
        input_dim = len(self.feature_names)
        self.model = TimeSeriesTransformerNetwork(
            input_size=input_dim, 
            d_model=self.d_model, 
            nhead=self.nhead, 
            num_layers=self.num_layers, 
            dim_feedforward=self.dim_feedforward, 
            dropout_rate=self.dropout_rate
        ).to(self.device)
        
        optimizer = optim.Adam(self.model.parameters(), lr=self.learning_rate)
        criterion = nn.MSELoss()
        
        dataset = SequenceDataset(scaled_seqs, y_tensor)
        loader = DataLoader(dataset, batch_size=self.batch_size, shuffle=True, collate_fn=pad_collate_fn)
        
        self.model.train()
        for epoch in range(self.epochs):
            running_loss = 0.0
            for batch_x, batch_y, lengths in loader:
                batch_x, batch_y = batch_x.to(self.device), batch_y.to(self.device)
                lengths = lengths.to(self.device)
                
                optimizer.zero_grad()
                outputs = self.model(batch_x, lengths)
                loss = criterion(outputs, batch_y)
                loss.backward()
                optimizer.step()
                
                running_loss += loss.item() * batch_x.size(0)
                
    def predict(self, X: pd.DataFrame):
        if self.model is None:
            raise ValueError("Model not fitted.")
            
        sequences = self._extract_sequences(X)
        
        scaled_seqs = []
        for seq in sequences:
            seq_arr = np.array([[w[k] for k in self.feature_names] for w in seq])
            scaled_arr = self.scaler.transform(seq_arr).astype(np.float32)
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
                'batch_size': self.batch_size,
                'epochs': self.epochs,
                'window_size_sec': self.window_size_sec,
                'window_step_sec': self.window_step_sec,
                'random_state': self.random_state
            }
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
        return regressor

    def plot_results(self, y_test: pd.Series, y_pred: np.ndarray, save_path: str | Path):
        plt.figure(figsize=(8, 6))
        
        actual_weights = np.sort(np.unique(y_test))
        pred_groups = [y_pred[y_test == w] for w in actual_weights]
        
        bp = plt.boxplot(pred_groups, positions=actual_weights, widths=0.4, patch_artist=True)
        for box in bp['boxes']:
            box.set(facecolor='lightblue', alpha=0.7)
        
        min_val = min(y_test.min(), y_pred.min())
        max_val = max(y_test.max(), y_pred.max())
        plt.plot([min_val, max_val], [min_val, max_val], 'r--', lw=2, label='Perfect Prediction')
        
        r2 = r2_score(y_test, y_pred)
        mae = mean_absolute_error(y_test, y_pred)
        plt.text(min_val + 0.1, max_val - 0.5, f"$R^2 = {r2:.3f}$\n$MAE = {mae:.3f}$ kg", 
                 bbox=dict(facecolor='white', alpha=0.8))
        
        plt.xlabel("Actual Weight (kg)")
        plt.ylabel("Predicted Weight (kg)")
        plt.title(f"Transformer: Predicted vs Actual Weight")
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close()
