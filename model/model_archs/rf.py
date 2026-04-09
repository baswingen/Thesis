import pandas as pd
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from sklearn.ensemble import RandomForestRegressor
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import joblib


def _sanitise(arr: np.ndarray, clip: float = 1e9) -> np.ndarray:
    """Replace NaN/Inf with 0 and clip abs-values to `clip` to prevent
    float64 overflow inside sklearn's StandardScaler."""
    arr = np.where(np.isfinite(arr), arr, 0.0)
    return np.clip(arr, -clip, clip)

from model.config_model import RF_CONFIG
from model import plotting_utils

class RFRegressor:
    """
    RFRegressor wraps the data scaling and Random Forest Regression model 
    training into a single class.
    """
    def __init__(self, 
                 n_estimators: int = RF_CONFIG['n_estimators'], 
                 max_depth: int | None = RF_CONFIG['max_depth'], 
                 min_samples_split: int = RF_CONFIG['min_samples_split'],
                 random_state: int = RF_CONFIG['random_state']):
        self.n_estimators = n_estimators
        self.max_depth = max_depth
        self.min_samples_split = min_samples_split
        self.random_state = random_state
        
        self.scaler = StandardScaler()
        self.model = RandomForestRegressor(
            n_estimators=self.n_estimators, 
            max_depth=self.max_depth, 
            min_samples_split=self.min_samples_split,
            random_state=self.random_state
        )
        
    def fit(self, X: pd.DataFrame, y: pd.Series, sample_weight=None):
        """Fit the scaler on features and train the RF."""
        X_scaled = self.scaler.fit_transform(_sanitise(X.values))
        self.model.fit(X_scaled, y)
        
    def predict(self, X: pd.DataFrame):
        """Scale test data based on fitted scaler and return non-negative predictions."""
        X_scaled = self.scaler.transform(_sanitise(X.values))
        y_pred = self.model.predict(X_scaled)
        # Weight cannot be negative
        return np.maximum(0.0, y_pred)
        
    def evaluate(self, X_test: pd.DataFrame, y_test: pd.Series):
        """
        Evaluate the regressor on the test set and return metrics.
        Returns a dictionary containing MAE, MSE, RMSE, and R2.
        """
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
        """Save the model and scaler using joblib."""
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        joblib.dump({'model': self.model, 'scaler': self.scaler}, filepath)
        print(f"RF Model saved to {filepath}")
        
    @classmethod
    def load(cls, filepath: str | Path):
        """Load a saved model and scaler from joblib."""
        data = joblib.load(filepath)
        regressor = cls()
        regressor.model = data['model']
        regressor.scaler = data['scaler']
        return regressor

    def plot_results(self, y_test: pd.Series, y_pred: np.ndarray, save_path: str | Path):
        plotting_utils.plot_regression_results(y_test, y_pred, save_path, model_name="Random Forest")
        print(f"Regression plot saved to {save_path}")
