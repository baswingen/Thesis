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

from model.config_model import CNN_LSTM_CONFIG
from model.model_archs.gru import SequenceDataset, pad_collate_fn  # Reuse dataloader utilities

class CNNLSTMNetwork(nn.Module):
    def __init__(self, input_size: int, cnn_filters: int, cnn_kernel_size: int, 
                 lstm_hidden_size: int, lstm_num_layers: int, dropout_rate: float):
        super().__init__()
        
        # 1D CNN for feature extraction over the sequence
        # We use padding='same' equivalent (padding = kernel_size // 2) to maintain sequence length
        padding = cnn_kernel_size // 2
        self.conv1d = nn.Conv1d(
            in_channels=input_size, 
            out_channels=cnn_filters, 
            kernel_size=cnn_kernel_size, 
            padding=padding
        )
        self.relu = nn.ReLU()
        
        # LSTM layer
        self.lstm = nn.LSTM(
            input_size=cnn_filters, 
            hidden_size=lstm_hidden_size, 
            num_layers=lstm_num_layers, 
            batch_first=True, 
            dropout=dropout_rate if lstm_num_layers > 1 else 0.0
        )
        self.dropout = nn.Dropout(dropout_rate)
        self.fc = nn.Linear(lstm_hidden_size, 1)
        
    def forward(self, x, lengths):
        # x shape: (batch_size, seq_len, input_size)
        
        # Conv1d expects (batch_size, channels, seq_len)
        x_cnn = x.transpose(1, 2)
        
        # Apply CNN
        cnn_out = self.conv1d(x_cnn)
        cnn_out = self.relu(cnn_out)
        
        # Transpose back for LSTM: (batch_size, seq_len, cnn_filters)
        lstm_input = cnn_out.transpose(1, 2)
        
        # Pack the sequence to avoid training on padding
        # Even though CNN processed padding, packing ignores it for the recurrent state
        packed_x = rnn_utils.pack_padded_sequence(lstm_input, lengths.cpu(), batch_first=True, enforce_sorted=False)
        
        packed_out, (hidden, cell) = self.lstm(packed_x) # hidden is (num_layers, batch, hidden_size)
        
        # Extract the hidden state from the last LSTM layer
        last_hidden = hidden[-1] # (batch, hidden_size)
        last_hidden = self.dropout(last_hidden)
        
        out = self.fc(last_hidden)
        return out

class CNNLSTMRegressor:
    def __init__(self, 
                 cnn_filters: int = CNN_LSTM_CONFIG['cnn_filters'],
                 cnn_kernel_size: int = CNN_LSTM_CONFIG['cnn_kernel_size'],
                 lstm_hidden_size: int = CNN_LSTM_CONFIG['lstm_hidden_size'],
                 lstm_num_layers: int = CNN_LSTM_CONFIG['lstm_num_layers'],
                 dropout_rate: float = CNN_LSTM_CONFIG['dropout_rate'],
                 learning_rate: float = CNN_LSTM_CONFIG['learning_rate'],
                 batch_size: int = CNN_LSTM_CONFIG['batch_size'],
                 epochs: int = CNN_LSTM_CONFIG['epochs'],
                 window_size_sec: float = CNN_LSTM_CONFIG['window_size_sec'],
                 window_step_sec: float = CNN_LSTM_CONFIG['window_step_sec'],
                 random_state: int = CNN_LSTM_CONFIG['random_state']):
                 
        self.cnn_filters = cnn_filters
        self.cnn_kernel_size = cnn_kernel_size
        self.lstm_hidden_size = lstm_hidden_size
        self.lstm_num_layers = lstm_num_layers
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
            raise ValueError("CNNLSTMRegressor expects DataFrame with 'sequence_dicts' column.")
        return X['sequence_dicts'].tolist()
        
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
        self.model = CNNLSTMNetwork(input_dim, self.cnn_filters, self.cnn_kernel_size,
                                    self.lstm_hidden_size, self.lstm_num_layers, 
                                    self.dropout_rate).to(self.device)
        
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
                'cnn_filters': self.cnn_filters,
                'cnn_kernel_size': self.cnn_kernel_size,
                'lstm_hidden_size': self.lstm_hidden_size,
                'lstm_num_layers': self.lstm_num_layers,
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
        regressor.model = CNNLSTMNetwork(state['input_dim'], 
                                         state['config']['cnn_filters'],
                                         state['config']['cnn_kernel_size'],
                                         state['config']['lstm_hidden_size'], 
                                         state['config']['lstm_num_layers'], 
                                         state['config']['dropout_rate']).to(regressor.device)
        regressor.model.load_state_dict(state['model_state'])
        regressor.model.eval()
        return regressor

    def plot_results(self, y_test: pd.Series, y_pred: np.ndarray, save_path: str | Path):
        plt.figure(figsize=(8, 6))
        plt.scatter(y_test, y_pred, alpha=0.5, color='royalblue', label='Predictions')
        
        min_val = min(y_test.min(), y_pred.min())
        max_val = max(y_test.max(), y_pred.max())
        plt.plot([min_val, max_val], [min_val, max_val], 'r--', lw=2, label='Perfect Prediction')
        
        r2 = r2_score(y_test, y_pred)
        mae = mean_absolute_error(y_test, y_pred)
        plt.text(min_val + 0.1, max_val - 0.5, f"$R^2 = {r2:.3f}$\n$MAE = {mae:.3f}$ kg", 
                 bbox=dict(facecolor='white', alpha=0.8))
        
        plt.xlabel("Actual Weight (kg)")
        plt.ylabel("Predicted Weight (kg)")
        plt.title(f"CNN-LSTM: Predicted vs Actual Weight")
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close()
