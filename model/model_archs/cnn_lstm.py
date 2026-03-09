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

# Add project root to sys.path
sys.path.append(str(Path(__file__).resolve().parent.parent.parent))

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
                 validation_split: float = CNN_LSTM_CONFIG.get('validation_split', 0.2),
                 early_stopping_patience: int = CNN_LSTM_CONFIG.get('early_stopping_patience', 10),
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
        self.validation_split = validation_split
        self.early_stopping_patience = early_stopping_patience
        self.window_size_sec = window_size_sec
        self.window_step_sec = window_step_sec
        self.random_state = random_state
        
        # Split stats
        self.train_samples = 0
        self.val_samples = 0
        
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
            
        all_features_train = np.vstack(flat_data_train)
        self.scaler.fit(all_features_train)
        
        # 4. Reconstruct scaled train sequences
        scaled_seqs_train = []
        for arr in flat_data_train:
            scaled_arr = self.scaler.transform(arr).astype(np.float32)
            scaled_seqs_train.append(torch.from_numpy(scaled_arr))
            
        y_tensor_train = torch.from_numpy(y_train.values.astype(np.float32)).unsqueeze(1)
        
        # 5. Process validation sequences using fitted scaler
        scaled_seqs_val = []
        for seq in sequences_val:
            seq_arr = np.array([[w[k] for k in self.feature_names] for w in seq])
            scaled_arr = self.scaler.transform(seq_arr).astype(np.float32)
            scaled_seqs_val.append(torch.from_numpy(scaled_arr))
            
        y_tensor_val = torch.from_numpy(y_val.values.astype(np.float32)).unsqueeze(1)
        
        self.train_samples = len(X_train)
        self.val_samples = len(X_val)
        
        # 6. Initialize Model and Optimizers
        input_dim = len(self.feature_names)
        self.model = CNNLSTMNetwork(input_dim, self.cnn_filters, self.cnn_kernel_size,
                                    self.lstm_hidden_size, self.lstm_num_layers, 
                                    self.dropout_rate).to(self.device)
        
        optimizer = optim.Adam(self.model.parameters(), lr=self.learning_rate)
        criterion = nn.MSELoss()
        
        dataset_train = SequenceDataset(scaled_seqs_train, y_tensor_train)
        loader_train = DataLoader(dataset_train, batch_size=self.batch_size, shuffle=True, collate_fn=pad_collate_fn)
        
        dataset_val = SequenceDataset(scaled_seqs_val, y_tensor_val)
        loader_val = DataLoader(dataset_val, batch_size=self.batch_size, shuffle=False, collate_fn=pad_collate_fn)
        
        # Early stopping tracking
        best_val_loss = float('inf')
        patience_counter = 0
        best_model_weights = copy.deepcopy(self.model.state_dict())
        
        # Wrap the epoch range with tqdm
        with tqdm(range(self.epochs), desc="Training CNN-LSTM", unit="epoch") as pbar:
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
                'validation_split': self.validation_split,
                'early_stopping_patience': self.early_stopping_patience,
                'window_size_sec': self.window_size_sec,
                'window_step_sec': self.window_step_sec,
                'random_state': self.random_state
            },
            'split_info': {
                'train_samples': self.train_samples,
                'val_samples': self.val_samples
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
        
        if 'split_info' in state:
            regressor.train_samples = state['split_info'].get('train_samples', 0)
            regressor.val_samples = state['split_info'].get('val_samples', 0)
            
        return regressor

    def plot_results(self, y_test: pd.Series, y_pred: np.ndarray, save_path: str | Path):
        plt.figure(figsize=(8, 6))
        
        # Extract unique actual weights and group predictions
        actual_weights = np.sort(np.unique(y_test))
        pred_groups = [y_pred[y_test == w] for w in actual_weights]
        
        # Create boxplot
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
        plt.title(f"CNN-LSTM: Predicted vs Actual Weight")
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close()
