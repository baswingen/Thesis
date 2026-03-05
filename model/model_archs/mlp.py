import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
import pandas as pd
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import joblib

from model.config_model import MLP_CONFIG

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
                 batch_size: int = MLP_CONFIG['batch_size'],
                 epochs: int = MLP_CONFIG['epochs'],
                 random_state: int = MLP_CONFIG['random_state']):
        
        self.hidden_layers = hidden_layers
        self.dropout_rate = dropout_rate
        self.learning_rate = learning_rate
        self.batch_size = batch_size
        self.epochs = epochs
        self.random_state = random_state
        
        torch.manual_seed(self.random_state)
        
        self.scaler = StandardScaler()
        self.model = None
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
    def fit(self, X: pd.DataFrame, y: pd.Series):
        """Fit the scaler and train the MLP."""
        X_scaled = self.scaler.fit_transform(X)
        y_scaled = y.values.reshape(-1, 1).astype(np.float32)
        X_scaled = X_scaled.astype(np.float32)
        
        # Initialize model
        self.model = MLP(X_scaled.shape[1], self.hidden_layers, self.dropout_rate).to(self.device)
        optimizer = optim.Adam(self.model.parameters(), lr=self.learning_rate)
        criterion = nn.MSELoss()
        
        # Prepare DataLoader
        dataset = TensorDataset(torch.from_numpy(X_scaled), torch.from_numpy(y_scaled))
        loader = DataLoader(dataset, batch_size=self.batch_size, shuffle=True)
        
        self.model.train()
        for epoch in range(self.epochs):
            running_loss = 0.0
            for batch_x, batch_y in loader:
                batch_x, batch_y = batch_x.to(self.device), batch_y.to(self.device)
                
                optimizer.zero_grad()
                outputs = self.model(batch_x)
                loss = criterion(outputs, batch_y)
                loss.backward()
                optimizer.step()
                
                running_loss += loss.item() * batch_x.size(0)
                
            if (epoch + 1) % 10 == 0:
                epoch_loss = running_loss / len(dataset)
                # print(f"Epoch [{epoch+1}/{self.epochs}], Loss: {epoch_loss:.4f}")
        
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
                'batch_size': self.batch_size,
                'epochs': self.epochs,
                'random_state': self.random_state
            }
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
        
        return regressor

    def plot_results(self, y_test: pd.Series, y_pred: np.ndarray, save_path: str | Path):
        """Predicted vs Actual scatter plot."""
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
        plt.title(f"MLP: Predicted vs Actual Weight")
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend()
        
        save_path = Path(save_path)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close()
