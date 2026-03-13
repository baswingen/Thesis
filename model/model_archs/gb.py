import pandas as pd
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import joblib

from model.config_model import GB_CONFIG
from model import plotting_utils

class GBRegressor:
    """
    GBRegressor wraps the data scaling and Gradient Boosting Regression model 
    training into a single class.
    """
    def __init__(self, 
                 n_estimators: int = GB_CONFIG['n_estimators'], 
                 learning_rate: float = GB_CONFIG['learning_rate'], 
                 max_depth: int = GB_CONFIG['max_depth'],
                 random_state: int = GB_CONFIG['random_state']):
        self.n_estimators = n_estimators
        self.learning_rate = learning_rate
        self.max_depth = max_depth
        self.random_state = random_state
        
        self.scaler = StandardScaler()
        self.model = GradientBoostingRegressor(
            n_estimators=self.n_estimators, 
            learning_rate=self.learning_rate, 
            max_depth=self.max_depth,
            random_state=self.random_state
        )
        
    def fit(self, X: pd.DataFrame, y: pd.Series):
        """Fit the scaler on features and train the GB."""
        X_scaled = self.scaler.fit_transform(X)
        self.model.fit(X_scaled, y)
        
    def predict(self, X: pd.DataFrame):
        """Scale test data based on fitted scaler and return non-negative predictions."""
        X_scaled = self.scaler.transform(X)
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
        print(f"GB Model saved to {filepath}")
        
    @classmethod
    def load(cls, filepath: str | Path):
        """Load a saved model and scaler from joblib."""
        data = joblib.load(filepath)
        regressor = cls()
        regressor.model = data['model']
        regressor.scaler = data['scaler']
        return regressor

    def plot_results(self, y_test: pd.Series, y_pred: np.ndarray, save_path: str | Path):
        plotting_utils.plot_regression_results(y_test, y_pred, save_path, model_name="Gradient Boosting")
        print(f"Regression plot saved to {save_path}")
