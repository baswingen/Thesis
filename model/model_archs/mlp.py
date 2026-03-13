import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
import pandas as pd
import numpy as np
import sys
from pathlib import Path
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import joblib

# Add project root to sys.path
sys.path.append(str(Path(__file__).resolve().parent.parent.parent))

from model.config_model import MLP_CONFIG, GLOBAL_LOSS_FUNCTION
from model import plotting_utils

class MLP(nn.Module):
    """
    Standard Multi-Layer Perceptron for regression.
    """
    def __init__(self, input_dim: int, hidden_layers: list, dropout_rate: float):
        super(MLP, self).__init__()
        layers = []
        last_dim = input_dim
        
        for h_dim in hidden_layers:
            layers.append(nn.Linear(last_dim, h_dim))
            layers.append(nn.BatchNorm1d(h_dim))
            layers.append(nn.ReLU())
            layers.append(nn.Dropout(dropout_rate))
            last_dim = h_dim
            
        layers.append(nn.Linear(last_dim, 1))
        self.network = nn.Sequential(*layers)
        
    def forward(self, x):
        return self.network(x)

class MLPRegressor:
    """
    MLPRegressor handles scaling, PyTorch training loop, and evaluation.
    """
    def __init__(self, 
                 hidden_layers: list = MLP_CONFIG['hidden_layers'],
                 dropout_rate: float = MLP_CONFIG['dropout_rate'],
                 learning_rate: float = MLP_CONFIG['learning_rate'],
                 weight_decay: float = MLP_CONFIG.get('weight_decay', 0.0),
                 batch_size: int = MLP_CONFIG['batch_size'],
                 epochs: int = MLP_CONFIG['epochs'],
                 validation_split: float = MLP_CONFIG.get('validation_split', 0.2),
                 early_stopping_patience: int = MLP_CONFIG.get('early_stopping_patience', 10),
                 scheduler_patience: int = MLP_CONFIG.get('scheduler_patience', 5),
                 scheduler_factor: float = MLP_CONFIG.get('scheduler_factor', 0.5),
                 loss_type: str = GLOBAL_LOSS_FUNCTION,
                 random_state: int = MLP_CONFIG['random_state']):
        
        self.hidden_layers = hidden_layers
        self.dropout_rate = dropout_rate
        self.learning_rate = learning_rate
        self.weight_decay = weight_decay
        self.batch_size = batch_size
        self.epochs = epochs
        self.validation_split = validation_split
        self.early_stopping_patience = early_stopping_patience
        self.loss_type = loss_type.lower()
        self.random_state = random_state
        
        # Split stats
        self.train_samples = 0
        self.val_samples = 0
        
        # Loss history
        self.loss_history = {"train": [], "val": []}
        
        torch.manual_seed(self.random_state)
        
        self.scaler = StandardScaler()
        self.model = None
        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        elif torch.backends.mps.is_available():
            self.device = torch.device("mps")
        else:
            self.device = torch.device("cpu")
        print(f"[{self.__class__.__name__}] Using device: {self.device}")
        
    def fit(self, X: pd.DataFrame, y: pd.Series):
        """Fit the scaler and train the MLP with early stopping."""
        from sklearn.model_selection import train_test_split
        from tqdm import tqdm
        import copy
        
        # 1. Split data into train and validation sets
        stratify = y if isinstance(y.iloc[0], (str, np.integer)) else None # Basic stratify check
        X_train, X_val, y_train, y_val = train_test_split(
            X, y, test_size=self.validation_split, random_state=self.random_state, stratify=None # No stratify for regression by default
        )
        
        self.train_samples = len(X_train)
        self.val_samples = len(X_val)
        
        # 2. Scale features (fit only on training set)
        X_train_scaled = self.scaler.fit_transform(X_train).astype(np.float32)
        X_val_scaled = self.scaler.transform(X_val).astype(np.float32)
        
        y_train_arr = y_train.values.reshape(-1, 1).astype(np.float32)
        y_val_arr = y_val.values.reshape(-1, 1).astype(np.float32)
        
        # 3. Initialize model
        self.model = MLP(X_train_scaled.shape[1], self.hidden_layers, self.dropout_rate).to(self.device)
        optimizer = optim.AdamW(self.model.parameters(), lr=self.learning_rate, weight_decay=self.weight_decay)
        scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', patience=self.scheduler_patience, factor=self.scheduler_factor)
        
        if self.loss_type == 'mae':
            criterion = nn.L1Loss()
        else:
            criterion = nn.MSELoss()
            if self.loss_type != 'mse':
                print(f"[MLPRegressor] Unknown loss type '{self.loss_type}', defaulting to MSE.")
        
        # 4. Prepare DataLoaders
        dataset_train = TensorDataset(torch.from_numpy(X_train_scaled), torch.from_numpy(y_train_arr))
        loader_train = DataLoader(dataset_train, batch_size=self.batch_size, shuffle=True)
        
        dataset_val = TensorDataset(torch.from_numpy(X_val_scaled), torch.from_numpy(y_val_arr))
        loader_val = DataLoader(dataset_val, batch_size=self.batch_size, shuffle=False)
        
        # Early stopping tracking
        best_val_loss = float('inf')
        patience_counter = 0
        best_model_weights = copy.deepcopy(self.model.state_dict())
        
        # 5. Training loop
        with tqdm(range(self.epochs), desc="Training MLP", unit="epoch") as pbar:
            for epoch in pbar:
                # TRAIN PHASE
                self.model.train()
                running_train_loss = 0.0
                for batch_x, batch_y in loader_train:
                    batch_x, batch_y = batch_x.to(self.device), batch_y.to(self.device)
                    
                    optimizer.zero_grad()
                    outputs = self.model(batch_x)
                    loss = criterion(outputs, batch_y)
                    loss.backward()
                    optimizer.step()
                    
                    running_train_loss += loss.item() * batch_x.size(0)
                
                avg_train_loss = running_train_loss / len(dataset_train)
                
                # VALIDATION PHASE
                self.model.eval()
                running_val_loss = 0.0
                with torch.no_grad():
                    for batch_x, batch_y in loader_val:
                        batch_x, batch_y = batch_x.to(self.device), batch_y.to(self.device)
                        outputs = self.model(batch_x)
                        loss = criterion(outputs, batch_y)
                        running_val_loss += loss.item() * batch_x.size(0)
                
                avg_val_loss = running_val_loss / len(dataset_val)
                pbar.set_postfix({"Loss": f"{avg_train_loss:.4f}", "Val Loss": f"{avg_val_loss:.4f}"})
                
                # Step the scheduler based on validation loss
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
        """Standard scale input and perform inference."""
        if self.model is None:
            raise ValueError("Model has not been trained yet. Call fit() first.")
            
        X_scaled = self.scaler.transform(X).astype(np.float32)
        X_tensor = torch.from_numpy(X_scaled).to(self.device)
        
        self.model.eval()
        with torch.no_grad():
            preds = self.model(X_tensor).cpu().numpy()
            
        # Weight cannot be negative
        return np.maximum(0.0, preds.flatten())
        
    def evaluate(self, X_test: pd.DataFrame, y_test: pd.Series):
        """Evaluate and return metrics."""
        y_pred = self.predict(X_test)
        mae = mean_absolute_error(y_test, y_pred)
        mse = mean_squared_error(y_test, y_pred)
        rmse = np.sqrt(mse)
        r2 = r2_score(y_test, y_pred)
        
        metrics = {
            "MAE": mae,
            "MSE": mse,
            "RMSE": rmse,
            "R2": r2
        }
        
        report_str = (
            f"Mean Absolute Error: {mae:.4f}\n"
            f"Mean Squared Error: {mse:.4f}\n"
            f"Root Mean Squared Error: {rmse:.4f}\n"
            f"R-squared Score: {r2:.4f}\n"
        )
        
        return metrics, report_str
        
    def save(self, filepath: str | Path):
        """Save model state, scaler, and metadata."""
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        
        state = {
            'model_state': self.model.state_dict(),
            'scaler': self.scaler,
            'input_dim': self.model.network[0].in_features,
            'hidden_layers': self.hidden_layers,
            'dropout_rate': self.dropout_rate,
            'config': {
                'learning_rate': self.learning_rate,
                'weight_decay': self.weight_decay,
                'batch_size': self.batch_size,
                'epochs': self.epochs,
                'loss_type': self.loss_type,
                'validation_split': self.validation_split,
                'early_stopping_patience': self.early_stopping_patience,
                'scheduler_patience': self.scheduler_patience,
                'scheduler_factor': self.scheduler_factor,
                'random_state': self.random_state
            },
            'split_info': {
                'train_samples': self.train_samples,
                'val_samples': self.val_samples
            },
            'loss_history': self.loss_history
        }
        torch.save(state, filepath)
        # print(f"MLP Model saved to {filepath}")
        
    @classmethod
    def load(cls, filepath: str | Path):
        """Load the saved model and metadata."""
        state = torch.load(filepath, weights_only=False) # scaler might require weights_only=False for joblib/pickle compatibility in check
        
        regressor = cls(
            hidden_layers=state['hidden_layers'],
            dropout_rate=state['dropout_rate'],
            **state['config']
        )
        regressor.scaler = state['scaler']
        regressor.model = MLP(state['input_dim'], state['hidden_layers'], state['dropout_rate']).to(regressor.device)
        regressor.model.load_state_dict(state['model_state'])
        regressor.model.eval()
        
        if 'split_info' in state:
            regressor.train_samples = state['split_info'].get('train_samples', 0)
            regressor.val_samples = state['split_info'].get('val_samples', 0)
            
        if 'loss_history' in state:
            regressor.loss_history = state['loss_history']
            
        return regressor

    def plot_results(self, y_test: pd.Series, y_pred: np.ndarray, save_path: str | Path):
        plotting_utils.plot_regression_results(y_test, y_pred, save_path, model_name="MLP")

    def plot_loss(self, save_path: str | Path):
        plotting_utils.plot_training_loss(self.loss_history, save_path, model_name="MLP")
